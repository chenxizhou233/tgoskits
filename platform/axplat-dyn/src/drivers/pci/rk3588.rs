extern crate alloc;

use alloc::{format, vec::Vec};
use core::time::Duration;

use fdt_edit::{PciRange, PciSpace, RegFixed};
use mmio_api::{MmioAddr, MmioOp, MmioRaw};
use rdif_clk::ClockId;
use rdif_pcie::{PciMem64, PcieController};
use rdrive::{
    PlatformDevice, module_driver,
    probe::{OnProbeError, fdt::NodeType},
    register::FdtInfo,
};
use rk3588_pci::{
    Delay, HostConfig, IatuMode, MEM_ATU_FIRST_REGION, OutboundWindow, ResetControl, Rk3588PcieHost,
};

const RK3588_GPIO_BASES: [u64; 5] = [
    0xfd8a_0000,
    0xfec2_0000,
    0xfec3_0000,
    0xfec4_0000,
    0xfec5_0000,
];
const RK3588_GPIO_SIZE: usize = 0x110;
const RK3588_GPIO_SWPORT_DR_L: usize = 0x00;
const RK3588_GPIO_SWPORT_DR_H: usize = 0x04;
const RK3588_GPIO_SWPORT_DDR_L: usize = 0x08;
const RK3588_GPIO_SWPORT_DDR_H: usize = 0x0c;
const RK3588_PCIE_POWER_STABLE_MS: u64 = 10;
const RK3588_PCIE_PHY_STABLE_MS: u64 = 20;
const RK3588_PCIE_POWER_DOMAIN: rockchip_pm::PowerDomain = rockchip_pm::PowerDomain(34);
const RK3588_CRU_BASE: u64 = 0xfd7c_0000;
const RK3588_CRU_SIZE: usize = 0x5c000;
const RK3588_SOFTRST_OFFSET: usize = 0x0a00;
const RK3588_PHP_CRU_OFFSET: usize = 0x8000;
const RK3588_PHY_TYPE_PCIE: u32 = 2;
const RK3588_PHYREG11: usize = 0x28;
const RK3588_PHYREG12: usize = 0x2c;
const RK3588_PHYREG27: usize = 0x6c;
const RK3588_PHYREG33: usize = 0x80;
const RK3588_PHYREG11_SU_TRIM_0_7: u32 = 0xf0;
const RK3588_PHYREG12_PLL_LPF_ADJ_VALUE: u32 = 4;
const RK3588_PHYREG27_RX_TRIM: u32 = 0x4c;
const RK3588_PHYREG33_PLL_KVCO_MASK: u32 = 0x1c;
const RK3588_PHYREG33_PLL_KVCO_SHIFT: u32 = 2;
const RK3588_ACLK_PCIE_1L0_DBI: usize = 317;
const RK3588_ACLK_PCIE_1L1_DBI: usize = 318;
const RK3588_ACLK_PCIE_1L2_DBI: usize = 319;
const RK3588_ACLK_PCIE_1L0_MSTR: usize = 322;
const RK3588_ACLK_PCIE_1L1_MSTR: usize = 323;
const RK3588_ACLK_PCIE_1L2_MSTR: usize = 324;
const RK3588_ACLK_PCIE_1L0_SLV: usize = 327;
const RK3588_ACLK_PCIE_1L1_SLV: usize = 328;
const RK3588_ACLK_PCIE_1L2_SLV: usize = 329;
const RK3588_PCLK_PCIE_1L0: usize = 332;
const RK3588_PCLK_PCIE_1L1: usize = 333;
const RK3588_PCLK_PCIE_1L2: usize = 334;
const RK3588_CLK_PCIE_AUX2: usize = 337;
const RK3588_CLK_PCIE_AUX3: usize = 338;
const RK3588_CLK_PCIE_AUX4: usize = 339;
const RK3588_CLK_PIPEPHY0_REF: usize = 340;
const RK3588_CLK_PIPEPHY1_REF: usize = 341;
const RK3588_CLK_PIPEPHY2_REF: usize = 342;
const RK3588_CLK_PIPEPHY0_PIPE_G: usize = 364;
const RK3588_CLK_PIPEPHY1_PIPE_G: usize = 365;
const RK3588_CLK_PIPEPHY2_PIPE_G: usize = 366;
const RK3588_CLK_PIPEPHY0_PIPE_ASIC_G: usize = 367;
const RK3588_CLK_PIPEPHY1_PIPE_ASIC_G: usize = 368;
const RK3588_CLK_PIPEPHY2_PIPE_ASIC_G: usize = 369;
const RK3588_CLK_PCIE1L2_PIPE: usize = 371;
const RK3588_CLK_PCIE1L0_PIPE: usize = 687;
const RK3588_CLK_PCIE1L1_PIPE: usize = 688;
const RK3588_PCLK_PCIE_COMBO_PIPE_PHY0: usize = 374;
const RK3588_PCLK_PCIE_COMBO_PIPE_PHY1: usize = 375;
const RK3588_PCLK_PCIE_COMBO_PIPE_PHY2: usize = 376;
const RK3588_PCLK_PCIE_COMBO_PIPE_PHY: usize = 377;
const DEFAULT_CFG_SIZE: u64 = 0x10_0000;

struct AxDelay;

impl Delay for AxDelay {
    fn delay_us(&self, us: u64) {
        axklib::time::busy_wait(Duration::from_micros(us));
    }

    fn delay_ms(&self, ms: u64) {
        axklib::time::busy_wait(Duration::from_millis(ms));
    }
}

struct Rk3588GpioReset {
    pin: u8,
    active_high: bool,
    gpio: MmioRaw,
}

struct Rk3588GpioOutput {
    pin: u8,
    active_high: bool,
    gpio: MmioRaw,
}

impl Rk3588GpioOutput {
    fn map(bank: u8, pin: u8, active_high: bool) -> Result<Self, OnProbeError> {
        let phys = *RK3588_GPIO_BASES
            .get(usize::from(bank))
            .ok_or_else(|| OnProbeError::other(format!("invalid RK3588 GPIO bank {}", bank)))?;
        Ok(Self {
            pin,
            active_high,
            gpio: map_mmio(phys, RK3588_GPIO_SIZE)?,
        })
    }

    fn set_logical(&self, value: bool) {
        let physical = if self.active_high { value } else { !value };
        self.write_masked_pair(RK3588_GPIO_SWPORT_DR_L, RK3588_GPIO_SWPORT_DR_H, physical);
        self.write_masked_pair(RK3588_GPIO_SWPORT_DDR_L, RK3588_GPIO_SWPORT_DDR_H, true);
    }

    fn write_masked_pair(&self, low_offset: usize, high_offset: usize, value: bool) {
        let pin = u32::from(self.pin);
        let (offset, shift) = if pin < 16 {
            (low_offset, pin)
        } else {
            (high_offset, pin - 16)
        };
        let mask = 1_u32 << (shift + 16);
        let data = u32::from(value) << shift;
        self.gpio.write::<u32>(offset, mask | data);
    }
}

impl Rk3588GpioReset {
    fn map(_apb_phys: u64, bank: u8, pin: u8, active_high: bool) -> Result<Self, OnProbeError> {
        let phys = *RK3588_GPIO_BASES
            .get(usize::from(bank))
            .ok_or_else(|| OnProbeError::other(format!("invalid RK3588 GPIO bank {}", bank)))?;
        Ok(Self {
            pin,
            active_high,
            gpio: map_mmio(phys, RK3588_GPIO_SIZE)?,
        })
    }

    fn set_logical(&self, value: bool) {
        let physical = if self.active_high { value } else { !value };
        self.write_masked_pair(RK3588_GPIO_SWPORT_DR_L, RK3588_GPIO_SWPORT_DR_H, physical);
        self.write_masked_pair(RK3588_GPIO_SWPORT_DDR_L, RK3588_GPIO_SWPORT_DDR_H, true);
    }

    fn write_masked_pair(&self, low_offset: usize, high_offset: usize, value: bool) {
        let pin = u32::from(self.pin);
        let (offset, shift) = if pin < 16 {
            (low_offset, pin)
        } else {
            (high_offset, pin - 16)
        };
        let mask = 1_u32 << (shift + 16);
        let data = u32::from(value) << shift;
        self.gpio.write::<u32>(offset, mask | data);
    }
}

impl ResetControl for Rk3588GpioReset {
    fn assert_perst(&mut self) {
        self.set_logical(true);
    }

    fn deassert_perst(&mut self) {
        self.set_logical(false);
    }
}

fn probe_rk3588(info: FdtInfo<'_>, plat_dev: PlatformDevice) -> Result<(), OnProbeError> {
    let node_name = info.node.as_node().name();
    let NodeType::Pci(node) = info.node else {
        return Err(OnProbeError::NotMatch);
    };

    let regs = node.regs();
    let apb_reg = *regs
        .first()
        .ok_or_else(|| OnProbeError::other(format!("{node_name} has no APB register")))?;
    let dbi_reg = *regs
        .get(1)
        .ok_or_else(|| OnProbeError::other(format!("{node_name} has no DBI register")))?;

    let ranges = node.ranges().unwrap_or_default();
    let (cfg_phys, cfg_size) = config_window(&regs, &ranges)?;
    let (bus_base, logical_bus_end) = bus_range_info(node.bus_range());
    let mut reset = pcie_reset_gpio(&info, apb_reg.address);
    let delay = AxDelay;

    rk3588_pcie_pre_dbi_bring_up(&info, apb_reg.address, &delay)?;

    let apb_size = apb_reg.size.unwrap_or(0x10000) as usize;
    let dbi_size = dbi_reg.size.unwrap_or(0x400000) as usize;
    let apb = map_mmio(apb_reg.address, apb_size)?;
    let dbi = map_mmio(dbi_reg.address, dbi_size)?;
    let cfg = map_mmio(cfg_phys, cfg_size as usize)?;

    let mut host = Rk3588PcieHost::new(
        apb,
        dbi,
        cfg,
        HostConfig {
            apb_phys: apb_reg.address,
            cfg_phys,
            cfg_size: cfg_size as usize,
            bus_base,
            logical_bus_end,
            iatu_mode: IatuMode::Unroll,
        },
    )
    .map_err(map_rk3588_error)?;

    match reset.as_mut() {
        Some(reset) => {
            host.init(&delay, Some(reset));
        }
        None => {
            host.init(&delay, None);
        }
    }

    program_memory_windows(&host, &ranges, cfg_phys, cfg_size);
    host.unmask_legacy_intx_all();
    log_direct_endpoint(&host);
    super::register_legacy_irq(&info, logical_bus_end);

    let mut drv = PcieController::new(host);
    for range in &ranges {
        if is_config_range(range, cfg_phys, cfg_size) {
            continue;
        }
        set_rk3588_bar_range(&mut drv, range);
    }

    info!(
        "Rockchip RK3588 PCIe host {:#x}: registering config window {:#x}/{} bytes, DT buses \
         {:#x}..={:#x}, logical buses 0..={}",
        apb_reg.address,
        cfg_phys,
        cfg_size,
        bus_base,
        bus_base.saturating_add(logical_bus_end),
        logical_bus_end
    );
    plat_dev.register_pcie(drv);
    Ok(())
}

fn rk3588_pcie_pre_dbi_bring_up(
    info: &FdtInfo<'_>,
    apb_base: u64,
    delay: &dyn Delay,
) -> Result<(), OnProbeError> {
    enable_pcie_power_domain(apb_base);
    enable_fixed_regulator(info, "vpcie3v3-supply", apb_base, delay);
    assert_resets(info, apb_base);
    enable_pcie_clocks(info, apb_base);
    prepare_pcie_phy(info, apb_base, delay)?;
    deassert_resets(info, apb_base);
    Ok(())
}

fn map_mmio(phys: u64, size: usize) -> Result<MmioRaw, OnProbeError> {
    crate::boot::Kernel
        .ioremap(MmioAddr::from(phys), size)
        .map_err(|err| {
            OnProbeError::other(format!(
                "failed to map MMIO region at {phys:#x} size {size:#x}: {err:?}"
            ))
        })
}

fn map_rk3588_error(err: rk3588_pci::Error) -> OnProbeError {
    OnProbeError::other(format!("{err:?}"))
}

fn pcie_reset_gpio(info: &FdtInfo<'_>, apb_base: u64) -> Option<Rk3588GpioReset> {
    let default = rk3588_pcie_reset_pin(apb_base);
    let Some((pin, active_high)) = reset_gpio_cells(info)
        .or_else(|| default.map(|default| (default.pin, default.active_high)))
    else {
        warn!(
            "Rockchip RK3588 PCIe host {:#x}: no PERST GPIO described",
            apb_base
        );
        return None;
    };
    let Some(bank) = reset_gpio_bank(info).or_else(|| default.map(|default| default.bank)) else {
        warn!(
            "Rockchip RK3588 PCIe host {:#x}: no PERST GPIO bank resolved",
            apb_base
        );
        return None;
    };

    match Rk3588GpioReset::map(apb_base, bank, pin, active_high) {
        Ok(reset) => Some(reset),
        Err(err) => {
            warn!(
                "Rockchip RK3588 PCIe host {:#x}: failed to map PERST GPIO: {}",
                apb_base, err
            );
            None
        }
    }
}

#[derive(Clone, Copy)]
struct Rk3588ResetPin {
    bank: u8,
    pin: u8,
    active_high: bool,
}

fn rk3588_pcie_reset_pin(apb_base: u64) -> Option<Rk3588ResetPin> {
    match apb_base {
        0xfe18_0000 => Some(Rk3588ResetPin {
            bank: 3,
            pin: 11,
            active_high: true,
        }),
        0xfe19_0000 => Some(Rk3588ResetPin {
            bank: 4,
            pin: 2,
            active_high: true,
        }),
        _ => None,
    }
}

fn reset_gpio_cells(info: &FdtInfo<'_>) -> Option<(u8, bool)> {
    let prop = info.node.as_node().get_property("reset-gpios")?;
    let mut cells = prop.get_u32_iter();
    let _phandle = cells.next()?;
    let pin = cells.next()?.try_into().ok()?;
    let flags = cells.next().unwrap_or(0);
    let active_low = flags & 1 != 0;
    Some((pin, !active_low))
}

fn reset_gpio_bank(info: &FdtInfo<'_>) -> Option<u8> {
    let prop = info.node.as_node().get_property("reset-gpios")?;
    let mut cells = prop.get_u32_iter();
    let phandle = cells.next()?;
    rk3588_gpio_bank_from_phandle(phandle)
}

fn enable_pcie_power_domain(apb_base: u64) {
    for pm_dev in rdrive::get_list::<rockchip_pm::RockchipPM>() {
        let Ok(mut pm) = pm_dev.lock() else {
            warn!(
                "Rockchip RK3588 PCIe host {:#x}: power manager is locked",
                apb_base
            );
            continue;
        };
        match pm.power_domain_on(RK3588_PCIE_POWER_DOMAIN) {
            Ok(()) => {
                return;
            }
            Err(err) => {
                warn!(
                    "Rockchip RK3588 PCIe host {:#x}: failed to enable PCIe power domain: {:?}",
                    apb_base, err
                );
            }
        }
    }
}

fn enable_fixed_regulator(info: &FdtInfo<'_>, prop_name: &str, apb_base: u64, delay: &dyn Delay) {
    let Some(phandle) = single_phandle_property(info, prop_name) else {
        warn!(
            "Rockchip RK3588 PCIe host {:#x}: no {} regulator phandle",
            apb_base, prop_name
        );
        return;
    };
    let Some((bank, pin, active_high, startup_us)) = fixed_regulator_gpio(info, phandle) else {
        warn!(
            "Rockchip RK3588 PCIe host {:#x}: regulator phandle {} has no supported GPIO",
            apb_base, phandle
        );
        return;
    };
    match Rk3588GpioOutput::map(bank, pin, active_high) {
        Ok(gpio) => {
            gpio.set_logical(true);
            if startup_us != 0 {
                delay.delay_us(u64::from(startup_us));
            } else {
                delay.delay_ms(RK3588_PCIE_POWER_STABLE_MS);
            }
        }
        Err(err) => {
            warn!(
                "Rockchip RK3588 PCIe host {:#x}: failed to map regulator GPIO: {}",
                apb_base, err
            );
        }
    }
}

fn enable_pcie_clocks(info: &FdtInfo<'_>, apb_base: u64) {
    let clocks = info.node.clocks();
    if clocks.is_empty() {
        warn!(
            "Rockchip RK3588 PCIe host {:#x}: no controller clocks in FDT",
            apb_base
        );
        return;
    }

    for clk in clocks {
        let Some(device_id) = info.phandle_to_device_id(clk.phandle) else {
            warn!(
                "Rockchip RK3588 PCIe host {:#x}: clock {:?} phandle {} has no device id",
                apb_base, clk.name, clk.phandle
            );
            continue;
        };
        let Ok(clk_dev) = rdrive::get::<rdif_clk::Clk>(device_id) else {
            warn!(
                "Rockchip RK3588 PCIe host {:#x}: clock {:?} device {:?} not registered",
                apb_base, clk.name, device_id
            );
            continue;
        };
        let Ok(mut clk_guard) = clk_dev.lock() else {
            warn!(
                "Rockchip RK3588 PCIe host {:#x}: clock {:?} device {:?} is locked",
                apb_base, clk.name, device_id
            );
            continue;
        };
        let Some(clock_id) = controller_clock_id(apb_base, clk.name.as_deref())
            .or_else(|| clk.select().map(|id| id as usize))
        else {
            warn!(
                "Rockchip RK3588 PCIe host {:#x}: clock {:?} has no usable id",
                apb_base, clk.name
            );
            continue;
        };
        let clock_id = ClockId::from(clock_id);
        let clock_id_raw: usize = clock_id.into();
        let enable_result = clk_guard.set_rate(ClockId::from(0), clock_id_raw as u64);
        clk_guard.perper_enable();
        match enable_result {
            Ok(()) => {}
            Err(err) => warn!(
                "Rockchip RK3588 PCIe host {:#x}: failed to enable clock {:?} id {:?}: {:?}",
                apb_base, clk.name, clock_id, err
            ),
        }
    }
}

fn controller_clock_id(apb_base: u64, name: Option<&str>) -> Option<usize> {
    let index = rk3588_pcie_1l_index(apb_base)?;
    match name {
        Some("aclk_mst") => Some(match index {
            0 => RK3588_ACLK_PCIE_1L0_MSTR,
            1 => RK3588_ACLK_PCIE_1L1_MSTR,
            2 => RK3588_ACLK_PCIE_1L2_MSTR,
            _ => return None,
        }),
        Some("aclk_slv") => Some(match index {
            0 => RK3588_ACLK_PCIE_1L0_SLV,
            1 => RK3588_ACLK_PCIE_1L1_SLV,
            2 => RK3588_ACLK_PCIE_1L2_SLV,
            _ => return None,
        }),
        Some("aclk_dbi") => Some(match index {
            0 => RK3588_ACLK_PCIE_1L0_DBI,
            1 => RK3588_ACLK_PCIE_1L1_DBI,
            2 => RK3588_ACLK_PCIE_1L2_DBI,
            _ => return None,
        }),
        Some("pclk") => Some(match index {
            0 => RK3588_PCLK_PCIE_1L0,
            1 => RK3588_PCLK_PCIE_1L1,
            2 => RK3588_PCLK_PCIE_1L2,
            _ => return None,
        }),
        Some("aux") => Some(match index {
            0 => RK3588_CLK_PCIE_AUX2,
            1 => RK3588_CLK_PCIE_AUX3,
            2 => RK3588_CLK_PCIE_AUX4,
            _ => return None,
        }),
        Some("pipe") => Some(match index {
            0 => RK3588_CLK_PCIE1L0_PIPE,
            1 => RK3588_CLK_PCIE1L1_PIPE,
            2 => RK3588_CLK_PCIE1L2_PIPE,
            _ => return None,
        }),
        _ => None,
    }
}

fn rk3588_pcie_1l_index(apb_base: u64) -> Option<u8> {
    match apb_base {
        0xfe15_0000 => Some(0),
        0xfe18_0000 => Some(1),
        0xfe19_0000 => Some(2),
        _ => None,
    }
}

fn prepare_pcie_phy(
    info: &FdtInfo<'_>,
    apb_base: u64,
    delay: &dyn Delay,
) -> Result<(), OnProbeError> {
    let phys = phandle_array(info, "phys");
    if phys.is_empty() {
        warn!(
            "Rockchip RK3588 PCIe host {:#x}: no PHY phandle in FDT",
            apb_base
        );
        return Ok(());
    }
    let Some((&phandle, args)) = phys.split_first() else {
        return Ok(());
    };
    let phy_type = args.first().copied().unwrap_or(RK3588_PHY_TYPE_PCIE);
    if phy_type != RK3588_PHY_TYPE_PCIE {
        warn!(
            "Rockchip RK3588 PCIe host {:#x}: PHY phandle {} has unsupported type {}",
            apb_base, phandle, phy_type
        );
        return Ok(());
    }
    let Some(phy) = rk3588_naneng_combphy(phandle) else {
        warn!(
            "Rockchip RK3588 PCIe host {:#x}: unsupported RK3588 PCIe PHY phandle {}",
            apb_base, phandle
        );
        return Ok(());
    };
    enable_combphy_clocks(info, apb_base, phy);
    assert_reset_id(phy.apb_reset);
    assert_reset_id(phy.phy_reset);
    rk3588_combphy_config_pcie(apb_base, phy)?;
    deassert_reset_id(phy.apb_reset);
    deassert_reset_id(phy.phy_reset);
    delay.delay_ms(RK3588_PCIE_PHY_STABLE_MS);
    Ok(())
}

fn assert_resets(info: &FdtInfo<'_>, _apb_base: u64) {
    for reset in reset_ids(info) {
        assert_reset_id(reset);
    }
}

fn enable_clock_id(
    info: &FdtInfo<'_>,
    apb_base: u64,
    clock_id: ClockId,
    name: Option<&str>,
) -> bool {
    let Some(device_id) = info.phandle_to_device_id(0x02.into()) else {
        warn!(
            "Rockchip RK3588 PCIe host {:#x}: CRU phandle has no device id for clock {:?} id {:?}",
            apb_base, name, clock_id
        );
        return false;
    };
    let Ok(clk_dev) = rdrive::get::<rdif_clk::Clk>(device_id) else {
        warn!(
            "Rockchip RK3588 PCIe host {:#x}: CRU device {:?} not registered for clock {:?} id \
             {:?}",
            apb_base, device_id, name, clock_id
        );
        return false;
    };
    let Ok(mut clk_guard) = clk_dev.lock() else {
        warn!(
            "Rockchip RK3588 PCIe host {:#x}: CRU device {:?} is locked for clock {:?} id {:?}",
            apb_base, device_id, name, clock_id
        );
        return false;
    };
    let clock_id_raw: usize = clock_id.into();
    clk_guard
        .set_rate(ClockId::from(0), clock_id_raw as u64)
        .is_ok()
}

fn deassert_resets(info: &FdtInfo<'_>, _apb_base: u64) {
    for reset in reset_ids(info) {
        deassert_reset_id(reset);
    }
}

fn single_phandle_property(info: &FdtInfo<'_>, name: &str) -> Option<u32> {
    info.node
        .as_node()
        .get_property(name)?
        .get_u32_iter()
        .next()
}

fn phandle_array(info: &FdtInfo<'_>, name: &str) -> Vec<u32> {
    info.node
        .as_node()
        .get_property(name)
        .map(|prop| prop.get_u32_iter().collect())
        .unwrap_or_default()
}

fn reset_ids(info: &FdtInfo<'_>) -> Vec<u32> {
    info.node
        .as_node()
        .get_property("resets")
        .into_iter()
        .flat_map(|prop| prop.get_u32_iter())
        .skip(1)
        .step_by(2)
        .collect()
}

#[derive(Clone, Copy)]
struct Rk3588Combphy {
    id: u8,
    mmio_base: u64,
    phy_grf_base: u64,
    pipe_grf_base: u64,
    apb_reset: u32,
    phy_reset: u32,
}

fn rk3588_naneng_combphy(phandle: u32) -> Option<Rk3588Combphy> {
    match phandle {
        0x110 => Some(Rk3588Combphy {
            id: 0,
            mmio_base: 0xfee0_0000,
            phy_grf_base: 0xfd5b_c000,
            pipe_grf_base: 0xfd5b_0000,
            apb_reset: 0x20005,
            phy_reset: 0x4d6,
        }),
        0x1c5 => Some(Rk3588Combphy {
            id: 1,
            mmio_base: 0xfee1_0000,
            phy_grf_base: 0xfd5c_0000,
            pipe_grf_base: 0xfd5b_0000,
            apb_reset: 0x20006,
            phy_reset: 0x4d7,
        }),
        0x71 => Some(Rk3588Combphy {
            id: 2,
            mmio_base: 0xfee2_0000,
            phy_grf_base: 0xfd5c_4000,
            pipe_grf_base: 0xfd5b_0000,
            apb_reset: 0x20007,
            phy_reset: 0x4d8,
        }),
        _ => None,
    }
}

fn enable_combphy_clocks(info: &FdtInfo<'_>, apb_base: u64, phy: Rk3588Combphy) {
    let clocks = match phy.id {
        0 => [
            RK3588_PCLK_PCIE_COMBO_PIPE_PHY0,
            RK3588_PCLK_PCIE_COMBO_PIPE_PHY,
            RK3588_CLK_PIPEPHY0_REF,
            RK3588_CLK_PIPEPHY0_PIPE_G,
            RK3588_CLK_PIPEPHY0_PIPE_ASIC_G,
        ],
        1 => [
            RK3588_PCLK_PCIE_COMBO_PIPE_PHY1,
            RK3588_PCLK_PCIE_COMBO_PIPE_PHY,
            RK3588_CLK_PIPEPHY1_REF,
            RK3588_CLK_PIPEPHY1_PIPE_G,
            RK3588_CLK_PIPEPHY1_PIPE_ASIC_G,
        ],
        2 => [
            RK3588_PCLK_PCIE_COMBO_PIPE_PHY2,
            RK3588_PCLK_PCIE_COMBO_PIPE_PHY,
            RK3588_CLK_PIPEPHY2_REF,
            RK3588_CLK_PIPEPHY2_PIPE_G,
            RK3588_CLK_PIPEPHY2_PIPE_ASIC_G,
        ],
        _ => return,
    };
    for clock_id in clocks {
        enable_clock_id(info, apb_base, ClockId::from(clock_id as usize), None);
    }
}

fn rk3588_combphy_config_pcie(_apb_base: u64, phy: Rk3588Combphy) -> Result<(), OnProbeError> {
    let phy_grf = map_mmio(phy.phy_grf_base, 0x100)?;
    let pipe_grf = map_mmio(phy.pipe_grf_base, 0x1000)?;
    let phy_mmio = map_mmio(phy.mmio_base, 0x100)?;

    grf_param_write(&phy_grf, 0x0000, 15, 0, 0x1000);
    grf_param_write(&phy_grf, 0x0004, 15, 0, 0x0000);
    grf_param_write(&phy_grf, 0x0008, 15, 0, 0x0101);
    grf_param_write(&phy_grf, 0x000c, 15, 0, 0x0200);
    if phy.id == 1 {
        grf_param_write(&pipe_grf, 0x0100, 0, 0, 0x0);
    } else if phy.id == 2 {
        grf_param_write(&pipe_grf, 0x0100, 1, 1, 0x0);
    }
    grf_param_write(&phy_grf, 0x0004, 14, 13, 0x02);

    mmio_update32(
        &phy_mmio,
        RK3588_PHYREG33,
        RK3588_PHYREG33_PLL_KVCO_MASK,
        4 << RK3588_PHYREG33_PLL_KVCO_SHIFT,
    );
    phy_mmio.write::<u32>(RK3588_PHYREG12, RK3588_PHYREG12_PLL_LPF_ADJ_VALUE);
    phy_mmio.write::<u32>(RK3588_PHYREG27, RK3588_PHYREG27_RX_TRIM);
    phy_mmio.write::<u32>(RK3588_PHYREG11, RK3588_PHYREG11_SU_TRIM_0_7);

    Ok(())
}

fn grf_param_write(mmio: &MmioRaw, offset: usize, bitend: u32, bitstart: u32, value: u32) {
    let width = bitend - bitstart + 1;
    let field_mask = if width >= 32 {
        u32::MAX
    } else {
        ((1_u32 << width) - 1) << bitstart
    };
    mmio.write::<u32>(offset, ((field_mask & 0xffff) << 16) | (value << bitstart));
}

fn mmio_update32(mmio: &MmioRaw, offset: usize, mask: u32, value: u32) {
    let current = mmio.read::<u32>(offset);
    mmio.write::<u32>(offset, (current & !mask) | (value & mask));
}

fn assert_reset_id(reset_id: u32) {
    write_reset_id(reset_id, true);
}

fn deassert_reset_id(reset_id: u32) {
    write_reset_id(reset_id, false);
}

fn write_reset_id(reset_id: u32, assert: bool) {
    let Some((base, bank, bit)) = rk3588_reset_location(reset_id) else {
        warn!("Rockchip RK3588: unsupported reset id {:#x}", reset_id);
        return;
    };
    let Ok(cru) = map_mmio(base, RK3588_CRU_SIZE) else {
        warn!(
            "Rockchip RK3588: failed to map CRU for reset id {:#x}",
            reset_id
        );
        return;
    };
    let mask = 1_u32 << bit;
    let value = if assert {
        mask << 16 | mask
    } else {
        mask << 16
    };
    cru.write::<u32>(RK3588_SOFTRST_OFFSET + bank as usize * 4, value);
}

fn rk3588_reset_location(reset_id: u32) -> Option<(u64, u32, u32)> {
    if reset_id < 0x10000 {
        return Some((RK3588_CRU_BASE, reset_id / 16, reset_id % 16));
    }
    let location = match reset_id {
        0x20005 => (RK3588_CRU_BASE + RK3588_PHP_CRU_OFFSET as u64, 0, 5),
        0x20006 => (RK3588_CRU_BASE + RK3588_PHP_CRU_OFFSET as u64, 0, 6),
        0x20007 => (RK3588_CRU_BASE + RK3588_PHP_CRU_OFFSET as u64, 0, 7),
        _ => return None,
    };
    Some(location)
}

fn fixed_regulator_gpio(
    _info: &FdtInfo<'_>,
    regulator_phandle: u32,
) -> Option<(u8, u8, bool, u32)> {
    match regulator_phandle {
        // OrangePi-5-Plus: vcc3v3_pcie30, GPIO0_B6, active high, 5ms.
        0x1c2 => Some((0, 14, true, 5_000)),
        // OrangePi-5-Plus: vcc3v3_pcie2x1l0, GPIO0_C5, active high, 50ms.
        0x1c6 => Some((0, 21, true, 50_000)),
        // OrangePi-5-Plus: vcc3v3_pcie_eth, GPIO4_C4, active low, 50ms.
        0x4a4 => Some((4, 20, false, 50_000)),
        _ => None,
    }
}

fn rk3588_gpio_bank_from_phandle(phandle: u32) -> Option<u8> {
    match phandle {
        0xf8 | 0x104 => Some(0),
        0xf9 | 0x105 => Some(1),
        0xfa | 0x106 => Some(2),
        0xfb | 0x107 => Some(3),
        0xfc | 0x108 | 0x10e => Some(4),
        _ => None,
    }
}

fn config_window(regs: &[RegFixed], ranges: &[PciRange]) -> Result<(u64, u64), OnProbeError> {
    if let Some(reg) = regs.get(2) {
        return Ok((reg.address, reg.size.unwrap_or(DEFAULT_CFG_SIZE)));
    }

    ranges
        .iter()
        .find(|range| {
            matches!(range.space, PciSpace::Memory32)
                && range.size == DEFAULT_CFG_SIZE
                && range.cpu_address == range.bus_address
        })
        .map(|range| (range.cpu_address, range.size))
        .ok_or_else(|| OnProbeError::other("RK3588 PCIe host has no config window"))
}

fn bus_range_info(bus_range: Option<core::ops::Range<u32>>) -> (u8, u8) {
    let Some(bus_range) = bus_range else {
        return (0, u8::MAX);
    };
    let bus_base = bus_range.start.min(u32::from(u8::MAX)) as u8;
    let logical_end = bus_range
        .end
        .saturating_sub(bus_range.start)
        .clamp(1, u32::from(u8::MAX)) as u8;
    (bus_base, logical_end)
}

fn program_memory_windows(
    host: &Rk3588PcieHost,
    ranges: &[PciRange],
    cfg_phys: u64,
    cfg_size: u64,
) {
    let mut region = MEM_ATU_FIRST_REGION;
    for range in ranges {
        if is_config_range(range, cfg_phys, cfg_size) {
            continue;
        }
        match range.space {
            PciSpace::Memory32 | PciSpace::Memory64 => {
                let window = OutboundWindow {
                    cpu_base: range.cpu_address,
                    pci_base: range.bus_address,
                    size: range.size,
                };
                if let Err(err) = host.program_memory_window(region, window) {
                    warn!(
                        "PCIe host {:#x}: invalid outbound iATU region {}: {err:?}",
                        host.apb_phys(),
                        region
                    );
                }
                debug!(
                    "PCIe host {:#x}: iATU mem region {} cpu={:#x} pci={:#x} size={:#x}",
                    host.apb_phys(),
                    region,
                    range.cpu_address,
                    range.bus_address,
                    range.size
                );
                region = region.saturating_add(1);
            }
            PciSpace::IO => {}
        }
    }
}

fn log_direct_endpoint(host: &Rk3588PcieHost) {
    if let Some(endpoint) = host.direct_endpoint_info() {
        info!(
            "PCIe endpoint: {} {:04x}:{:04x} (rev {:02x}, class {:02x}{:02x}{:02x})",
            endpoint.address,
            endpoint.vendor_id,
            endpoint.device_id,
            endpoint.revision_id,
            endpoint.base_class,
            endpoint.sub_class,
            endpoint.prog_if
        );
    }
}

fn is_config_range(range: &PciRange, cfg_phys: u64, cfg_size: u64) -> bool {
    range.cpu_address == cfg_phys && range.size == cfg_size
}

fn set_rk3588_bar_range(drv: &mut PcieController, range: &PciRange) {
    super::set_pcie_mem_range(drv, range);
    if matches!(range.space, PciSpace::Memory32) {
        drv.set_mem64(
            PciMem64 {
                address: range.cpu_address,
                size: range.size,
            },
            range.prefetchable,
        );
    }
}

mod rk3588_pcie_fe180000 {
    use super::*;

    module_driver!(
        name: "Rockchip RK3588 PCIe host fe180000",
        level: ProbeLevel::PostKernel,
        priority: ProbePriority::DEFAULT,
        probe_kinds: &[
            ProbeKind::Fdt {
                compatibles: &["rockchip,rk3588-pcie"],
                on_probe: probe
            }
        ],
    );

    fn probe(info: FdtInfo<'_>, plat_dev: PlatformDevice) -> Result<(), OnProbeError> {
        probe_rk3588(info, plat_dev)
    }
}

mod rk3588_pcie_fe190000 {
    use super::*;

    module_driver!(
        name: "Rockchip RK3588 PCIe host fe190000",
        level: ProbeLevel::PostKernel,
        priority: ProbePriority::DEFAULT,
        probe_kinds: &[
            ProbeKind::Fdt {
                compatibles: &["rockchip,rk3588-pcie"],
                on_probe: probe
            }
        ],
    );

    fn probe(info: FdtInfo<'_>, plat_dev: PlatformDevice) -> Result<(), OnProbeError> {
        probe_rk3588(info, plat_dev)
    }
}
