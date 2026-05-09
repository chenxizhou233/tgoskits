// Copyright 2025 The Axvisor Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

use alloc::{format, string::ToString, vec};
use core::{num::NonZeroUsize, ptr::NonNull, time::Duration};

use dma_api::DeviceDma;
use rdif_clk::ClockId;
use rdrive::{
    Device, DriverGeneric, PlatformDevice, module_driver, probe::OnProbeError, register::FdtInfo,
};
use sdhci_host::Sdhci;
use sdmmc_protocol::{
    Error,
    error::{ErrorContext, Phase},
    sdio::{DelayNs, SdioSdmmc},
};
use spin::Once;

use crate::drivers::{DmaImpl, blk::PlatformDeviceBlock, iomap};

const BLOCK_SIZE: usize = 512;
const SDHCI_POWER_330: u8 = 0x0e;
const USE_ADMA2_READ: bool = true;
const USE_ADMA2_WRITE: bool = true;

type RockchipSdhci = SdioSdmmc<Sdhci, AxDelay>;

module_driver!(
    name: "Rockchip sdhci",
    level: ProbeLevel::PostKernel,
    priority: ProbePriority::DEFAULT,
    probe_kinds: &[
        ProbeKind::Fdt {
            compatibles: &["rockchip,rk3588-dwcmshc", "rockchip,dwcmshc-sdhci"],
            on_probe: probe
        }
    ],
);

fn probe(info: FdtInfo<'_>, plat_dev: PlatformDevice) -> Result<(), OnProbeError> {
    let base_reg = info
        .node
        .regs()
        .into_iter()
        .next()
        .ok_or(OnProbeError::other(alloc::format!(
            "[{}] has no reg",
            info.node.name()
        )))?;

    let mmio_size = base_reg.size.unwrap_or(0x1000);
    info!(
        "rockchip-sdhci probe: node={}, addr={:#x}, size={:#x}",
        info.node.name(),
        base_reg.address as usize,
        mmio_size
    );
    let mmio_base = iomap((base_reg.address as usize).into(), mmio_size as usize)?;

    init_core_clock(&info)?;

    let mut host = unsafe { Sdhci::new(mmio_base) };
    if CLK_DEV.is_completed() {
        info!("rockchip-sdhci: using external CRU clock");
        host.set_external_clock(set_sdhci_clock);
    } else {
        warn!("rockchip-sdhci: no core clock found; using SDHCI internal clock divider");
    }
    info!("rockchip-sdhci: reset controller");
    host.reset_all()
        .map_err(|e| init_error(base_reg.address, mmio_size, e))?;
    host.set_power(SDHCI_POWER_330);
    host.enable_interrupts();

    info!("rockchip-sdhci: initialize card");
    let mut card = SdioSdmmc::new(host, AxDelay);
    let card_info = card
        .init()
        .map_err(|e| card_init_error(base_reg.address, mmio_size, e))?;
    info!(
        "SDHCI card: kind={:?} high_capacity={} rca={} ocr={:#010x} capacity_blocks={:?} cid={} \
         ext_csd={}",
        card_info.kind,
        card_info.high_capacity,
        card_info.rca,
        card_info.ocr,
        card_info.capacity_blocks,
        card_info.cid.is_some(),
        card_info.ext_csd.is_some()
    );

    let dev = BlockDevice {
        dev: Some(card),
        dma: DeviceDma::new(u32::MAX as u64, &DmaImpl),
        capacity_blocks: card_info.capacity_blocks.unwrap_or(0),
    };
    plat_dev.register_block(dev);
    info!("rockchip-sdhci block device registered");
    Ok(())
}

fn init_error(address: u64, size: u64, err: Error) -> OnProbeError {
    OnProbeError::other(format!(
        "failed to initialize SDHCI device at [PA:{:?}, SZ:0x{:x}): {err:?}",
        address, size
    ))
}

fn card_init_error(address: u64, size: u64, err: Error) -> OnProbeError {
    if is_absent_card_init_error(err) {
        warn!(
            "rockchip-sdhci: no responsive card at [PA:{:?}, SZ:0x{:x}); skipping controller: \
             {err:?}",
            address, size
        );
        return OnProbeError::NotMatch;
    }

    init_error(address, size, err)
}

fn is_absent_card_init_error(err: Error) -> bool {
    match err {
        Error::NoCard => true,
        Error::Timeout(ctx) | Error::Crc(ctx) | Error::BadResponse(ctx) => {
            ctx.cmd.is_some()
                && matches!(
                    ctx.phase,
                    Phase::CommandSend | Phase::ResponseWait | Phase::Init
                )
        }
        _ => false,
    }
}

fn init_core_clock(info: &FdtInfo<'_>) -> Result<(), OnProbeError> {
    for clk in info.node.clocks() {
        info!(
            "rockchip-sdhci clock: phandle <{}>, name: {:?}, cells: {}",
            clk.phandle, clk.name, clk.cells
        );
        if clk.name == Some("core".to_string()) {
            let device_id = info.phandle_to_device_id(clk.phandle).ok_or_else(|| {
                OnProbeError::other(format!(
                    "[{}] core clock phandle {} has no device id",
                    info.node.name(),
                    clk.phandle
                ))
            })?;
            let clk_dev = rdrive::get::<rdif_clk::Clk>(device_id).map_err(|_| {
                OnProbeError::other(format!(
                    "[{}] core clock device {:?} is not registered",
                    info.node.name(),
                    device_id
                ))
            })?;
            CLK_DEV.call_once(|| ClkDev {
                inner: clk_dev,
                id: (clk.select().unwrap_or(0) as usize).into(),
            });
            return Ok(());
        }
    }
    Ok(())
}

fn set_sdhci_clock(target_hz: u32) -> Result<(), Error> {
    let clk = CLK_DEV.wait();
    let mut clk_dev = clk.inner.lock().map_err(|_| clock_error())?;
    clk_dev
        .set_rate(clk.id, target_hz as u64)
        .map_err(|_| clock_error())?;
    let rate = clk_dev.get_rate(clk.id).map_err(|_| clock_error())?;
    info!("rockchip-sdhci: core clock set to {} Hz", rate);
    Ok(())
}

fn clock_error() -> Error {
    Error::BusError(ErrorContext::new(Phase::Init))
}

struct BlockDevice {
    dev: Option<RockchipSdhci>,
    dma: DeviceDma,
    capacity_blocks: u64,
}

struct BlockQueue {
    raw: RockchipSdhci,
    dma: DeviceDma,
    capacity_blocks: u64,
    adma_success_logged: bool,
    adma_failure_logged: bool,
}

impl DriverGeneric for BlockDevice {
    fn name(&self) -> &str {
        "rockchip-sdhci"
    }
}

impl rd_block::Interface for BlockDevice {
    fn create_queue(&mut self) -> Option<alloc::boxed::Box<dyn rd_block::IQueue>> {
        self.dev.take().map(|dev| {
            alloc::boxed::Box::new(BlockQueue {
                raw: dev,
                dma: self.dma.clone(),
                capacity_blocks: self.capacity_blocks,
                adma_success_logged: false,
                adma_failure_logged: false,
            }) as _
        })
    }

    fn enable_irq(&mut self) {}

    fn disable_irq(&mut self) {}

    fn is_irq_enabled(&self) -> bool {
        false
    }

    fn handle_irq(&mut self) -> rd_block::Event {
        rd_block::Event::none()
    }
}

impl rd_block::IQueue for BlockQueue {
    fn num_blocks(&self) -> usize {
        self.capacity_blocks as usize
    }

    fn block_size(&self) -> usize {
        BLOCK_SIZE
    }

    fn id(&self) -> usize {
        0
    }

    fn buff_config(&self) -> rd_block::BuffConfig {
        rd_block::BuffConfig {
            dma_mask: u64::MAX,
            align: BLOCK_SIZE,
            size: BLOCK_SIZE,
        }
    }

    fn submit_request(
        &mut self,
        request: rd_block::Request<'_>,
    ) -> Result<rd_block::RequestId, rd_block::BlkError> {
        let start_block = request.block_id as u32;
        match request.kind {
            rd_block::RequestKind::Read(mut buffer) => {
                if !buffer.len().is_multiple_of(BLOCK_SIZE) {
                    return Err(rd_block::BlkError::Other(
                        "read buffer is not block aligned".into(),
                    ));
                }
                let block_count = buffer.len() / BLOCK_SIZE;
                if USE_ADMA2_READ {
                    match self.try_read_dma(start_block, &mut buffer) {
                        Ok(()) => {
                            if !self.adma_success_logged {
                                info!(
                                    "rockchip-sdhci: ADMA2 per-request read active, first request \
                                     block={} blocks={}",
                                    start_block, block_count
                                );
                                self.adma_success_logged = true;
                            }
                            return Ok(rd_block::RequestId::new(0));
                        }
                        Err(err) => {
                            if !self.adma_failure_logged {
                                warn!(
                                    "rockchip-sdhci: ADMA2 read failed at block={} blocks={}: \
                                     {:?}; falling back to PIO",
                                    start_block, block_count, err
                                );
                                self.adma_failure_logged = true;
                            }
                        }
                    }
                }
                self.read_blocks_pio(start_block, &mut buffer)?;
            }
            rd_block::RequestKind::Write(items) => {
                if !items.len().is_multiple_of(BLOCK_SIZE) {
                    return Err(rd_block::BlkError::Other(
                        "write buffer is not block aligned".into(),
                    ));
                }
                let block_count = items.len() / BLOCK_SIZE;
                if USE_ADMA2_WRITE {
                    match self.try_write_dma(start_block, items) {
                        Ok(()) => {
                            if !self.adma_success_logged {
                                info!(
                                    "rockchip-sdhci: ADMA2 per-request write active, first \
                                     request block={} blocks={}",
                                    start_block, block_count
                                );
                                self.adma_success_logged = true;
                            }
                            return Ok(rd_block::RequestId::new(0));
                        }
                        Err(err) => {
                            if !self.adma_failure_logged {
                                warn!(
                                    "rockchip-sdhci: ADMA2 write failed at block={} blocks={}: \
                                     {:?}; falling back to PIO",
                                    start_block, block_count, err
                                );
                                self.adma_failure_logged = true;
                            }
                        }
                    }
                }
                self.write_blocks_pio(start_block, items)?;
            }
        }
        Ok(rd_block::RequestId::new(0))
    }

    fn poll_request(&mut self, _request: rd_block::RequestId) -> Result<(), rd_block::BlkError> {
        Ok(())
    }
}

impl BlockQueue {
    fn try_read_dma(
        &mut self,
        start_block: u32,
        buffer: &mut rd_block::Buffer,
    ) -> Result<(), rd_block::BlkError> {
        let size = NonZeroUsize::new(buffer.size).ok_or(rd_block::BlkError::Other(
            "zero-sized DMA read buffer".into(),
        ))?;
        let ptr = NonNull::new(buffer.virt)
            .ok_or(rd_block::BlkError::Other("null DMA read buffer".into()))?;
        self.dma_read_into(start_block, ptr, size)
    }

    fn dma_read_into(
        &mut self,
        start_block: u32,
        ptr: NonNull<u8>,
        size: NonZeroUsize,
    ) -> Result<(), rd_block::BlkError> {
        let total = size.get();
        if !total.is_multiple_of(BLOCK_SIZE) {
            return Err(rd_block::BlkError::Other(
                "DMA read buffer is not block aligned".into(),
            ));
        }
        let block_count = total / BLOCK_SIZE;
        if block_count == 0 {
            return Err(rd_block::BlkError::Other(
                "DMA read buffer is smaller than one block".into(),
            ));
        }

        let card_addr = block_addr_of(start_block, self.raw.is_high_capacity());
        self.raw
            .host_mut()
            .dma_read_blocks_into(card_addr, ptr, size, &self.dma)
            .map_err(map_dev_err_to_blk_err)
    }

    fn try_write_dma(&mut self, start_block: u32, buffer: &[u8]) -> Result<(), rd_block::BlkError> {
        let size = NonZeroUsize::new(buffer.len()).ok_or(rd_block::BlkError::Other(
            "zero-sized DMA write buffer".into(),
        ))?;
        let ptr = NonNull::new(buffer.as_ptr() as *mut u8)
            .ok_or(rd_block::BlkError::Other("null DMA write buffer".into()))?;
        self.dma_write_from(start_block, ptr, size)
    }

    fn dma_write_from(
        &mut self,
        start_block: u32,
        ptr: NonNull<u8>,
        size: NonZeroUsize,
    ) -> Result<(), rd_block::BlkError> {
        let total = size.get();
        if !total.is_multiple_of(BLOCK_SIZE) {
            return Err(rd_block::BlkError::Other(
                "DMA write buffer is not block aligned".into(),
            ));
        }
        let block_count = total / BLOCK_SIZE;
        if block_count == 0 {
            return Err(rd_block::BlkError::Other(
                "DMA write buffer is smaller than one block".into(),
            ));
        }

        let card_addr = block_addr_of(start_block, self.raw.is_high_capacity());
        self.raw
            .host_mut()
            .dma_write_blocks_from(card_addr, ptr, size, &self.dma)
            .map_err(map_dev_err_to_blk_err)
    }

    fn read_blocks_pio(
        &mut self,
        start_block: u32,
        out: &mut [u8],
    ) -> Result<(), rd_block::BlkError> {
        for (index, block) in out.chunks_exact_mut(BLOCK_SIZE).enumerate() {
            let mut block_buf = [0; BLOCK_SIZE];
            self.raw
                .read_block(start_block + index as u32, &mut block_buf)
                .map_err(map_dev_err_to_blk_err)?;
            block.copy_from_slice(&block_buf);
        }
        Ok(())
    }

    fn write_blocks_pio(
        &mut self,
        start_block: u32,
        items: &[u8],
    ) -> Result<(), rd_block::BlkError> {
        let mut blocks = vec![[0; BLOCK_SIZE]; items.len() / BLOCK_SIZE];
        for (block, item) in blocks.iter_mut().zip(items.chunks_exact(BLOCK_SIZE)) {
            block.copy_from_slice(item);
        }
        self.raw
            .write_blocks(start_block, &blocks)
            .map_err(map_dev_err_to_blk_err)
    }
}

fn block_addr_of(block: u32, high_capacity: bool) -> u32 {
    if high_capacity {
        block
    } else {
        block.saturating_mul(BLOCK_SIZE as u32)
    }
}

fn map_dev_err_to_blk_err(err: Error) -> rd_block::BlkError {
    match err {
        Error::Timeout(_) => rd_block::BlkError::Retry,
        Error::NoCard | Error::UnsupportedCommand | Error::CardLocked => {
            rd_block::BlkError::NotSupported
        }
        Error::Misaligned | Error::InvalidArgument => {
            rd_block::BlkError::Other("SD/MMC request is not block aligned".into())
        }
        _ => rd_block::BlkError::Other("SDHCI I/O error".into()),
    }
}

static CLK_DEV: Once<ClkDev> = Once::new();

struct ClkDev {
    inner: Device<rdif_clk::Clk>,
    id: ClockId,
}

#[derive(Clone, Copy)]
struct AxDelay;

impl DelayNs for AxDelay {
    fn delay_ns(&mut self, ns: u32) {
        axklib::time::busy_wait(Duration::from_nanos(ns as u64));
    }

    fn delay_us(&mut self, us: u32) {
        axklib::time::busy_wait(Duration::from_micros(us as u64));
    }

    fn delay_ms(&mut self, ms: u32) {
        axklib::time::busy_wait(Duration::from_millis(ms as u64));
    }
}
