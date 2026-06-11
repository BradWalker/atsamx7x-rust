//! Toggles the LED0 on a SAMv71 Explained Eval Board

#![no_std]
#![no_main]

use panic_rtt_target as _;

#[repr(transparent)]
#[derive(Copy, Clone, Debug, Default)]
pub struct XdmacMicroBlockControl(pub u32);

#[repr(C)]
#[derive(Copy, Clone, Debug, Default)]
pub struct XdmacDescriptorView0 {
    /// Next Descriptor Address
    pub mbr_nda: u32,

    /// Micro-block Control Member
    pub mbr_ubc: XdmacMicroBlockControl,

    /// Destination Address Member
    pub mbr_da: u32,
}

#[repr(C)]
#[derive(Copy, Clone, Debug, Default)]
pub struct XdmacDescriptorView1 {
    /// Next Descriptor Address
    pub mbr_nda: u32,

    /// Micro-block Control Member
    pub mbr_ubc: XdmacMicroBlockControl,

    /// Source Address Member
    pub mbr_sa: u32,

    /// Destination Address Member
    pub mbr_da: u32,
}

#[repr(C)]
#[derive(Copy, Clone, Debug, Default)]
pub struct XdmacDescriptorView2 {
    /// Next Descriptor Address
    pub mbr_nda: u32,

    /// Micro-block Control Member
    pub mbr_ubc: XdmacMicroBlockControl,

    /// Source Address Member
    pub mbr_sa: u32,

    /// Destination Address Member
    pub mbr_da: u32,

    /// Configuration Register
    pub mbr_cfg: u32,
}

#[repr(C)]
#[derive(Copy, Clone, Debug, Default)]
pub struct XdmacDescriptorView3 {
    /// Next Descriptor Address
    pub mbr_nda: u32,

    /// Micro-block Control Member
    pub mbr_ubc: XdmacMicroBlockControl,

    /// Source Address Member
    pub mbr_sa: u32,

    /// Destination Address Member
    pub mbr_da: u32,

    /// Configuration Register
    pub mbr_cfg: u32,

    /// Block Control Member
    pub mbr_bc: u32,

    /// Data Stride Member
    pub mbr_ds: u32,

    /// Source Micro-block Stride Member
    pub mbr_sus: u32,

    /// Destination Micro-block Stride Member
    pub mbr_dus: u32,
}

#[repr(transparent)]
#[derive(Copy, Clone, Debug, Default)]
pub struct XdmacDescriptorControl(pub u8);

impl XdmacDescriptorControl {
    const FETCH_ENABLE: u8      = 1 << 0;
    const SOURCE_UPDATE: u8     = 1 << 1;
    const DESTINATION_UPDATE: u8 = 1 << 2;
    const VIEW_SHIFT: u8        = 3;
    const VIEW_MASK: u8         = 0b11 << Self::VIEW_SHIFT;

    pub const fn new(
        fetch_enable: bool,
        source_update: bool,
        destination_update: bool,
        view: u8,
    ) -> Self {
        Self(
            (if fetch_enable { Self::FETCH_ENABLE } else { 0 }) |
                (if source_update { Self::SOURCE_UPDATE } else { 0 }) |
                (if destination_update { Self::DESTINATION_UPDATE } else { 0 }) |
                ((view & 0b11) << Self::VIEW_SHIFT)
        )
    }
}

pub const TX_FIRST_DESCRIPTOR_CONTROL: XdmacDescriptorControl =
    XdmacDescriptorControl::new(
        true,  // fetchEnable
        true,  // sourceUpdate
        true,  // destinationUpdate
        1,     // view
    );

pub const RX_FIRST_DESCRIPTOR_CONTROL: XdmacDescriptorControl =
    XdmacDescriptorControl::new(
        true,
        true,
        true,
        1,
    );



#[rtic::app(device = atsamx7x_hal::pac, peripherals = true, dispatchers = [EFC])]
mod app {
    use atsamx7x_hal as hal;
    use cortex_m::prelude::*;
    use hal::clocks::*;
    use hal::efc::*;
    use hal::ehal::digital::v2::ToggleableOutputPin;
    use hal::fugit::RateExtU32;
    use hal::nb::block;
    use hal::pio::*;
    use hal::serial::spi::*;
    use hal::serial::ExtBpsU32;
    use rtt_target::{rprintln, rtt_init_print};

    const ADS1278_SAMPLE_BYTES: usize = 3 * 1;
    const SPI_DUMMY_BYTE: u8 = 0x0;
    const ADS1278_SPI_BPS: u32 = 24_000_000;
    const XDMAC_PERID_SPI0_TX: u8 = 1;
    const XDMAC_PERID_SPI0_RX: u8 = 2;

    #[shared]
    struct Shared {
        spi: Spi<Spi0>,
        tx_buf: [u8; 3],
        rx_buf: [u8; 3],
        dma_busy: bool,
    }

    #[local]
    struct Local {
        led: Pin<PA23, Output>,
        irq: BankInterrupts<A>,

        tx_desc: [crate::XdmacDescriptorView1; 2],
        rx_desc: [crate::XdmacDescriptorView1; 2],
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        // Manual runtime configuration for Cortex-M7
        unsafe {
            let cp = cortex_m::Peripherals::steal();
            let scb = cp.SCB;

            // 1. Set the vector table offset address
            // Ensure the target address matches your alignment constraints
            scb.vtor.write(0x20400000);

            // 2. Clear pipelines and synchronize instructions
            cortex_m::asm::dsb(); // Data Synchronization Barrier
            cortex_m::asm::isb(); // Instruction Synchronization Barrier
        }

        /*

        let mut cp = cortex_m::Peripherals::take().unwrap();

        cp.SCB.enable_icache();

        cp.SCB.enable_dcache(&mut cp.CPUID);
        */
        rtt_init_print!();
        rprintln!("init");

        let vtor = unsafe { (*cortex_m::peripheral::SCB::PTR).vtor.read() };
        rprintln!("{:x}", vtor);

        let clocks = Tokens::new(
            (ctx.device.PMC, ctx.device.SUPC, ctx.device.UTMI),
            &ctx.device.WDT.into(),
        );
        let slck = clocks.slck.configure_external_normal();
        let mainck = clocks.mainck.configure_external_normal(10.MHz()).unwrap();
        let pllack = clocks
            .pllack
            .configure(&mainck, PllaConfig { div: 1, mult: 29 })
            .unwrap();
        let (hclk, mut mck) = HostClockController::new(clocks.hclk, clocks.mck)
            .configure(
                &pllack,
                &mut Efc::new(ctx.device.EFC, VddioLevel::V3),
                HostClockConfig {
                    pres: HccPrescaler::Div1,
                    div: MckDivider::Div2,
                },
            )
            .unwrap();

        rprintln!("hclk: {}", hclk.systick_freq().to_Hz());

        let xdmac = ctx.device.XDMAC;

        // Disable channels
        xdmac.gd().write(|w| unsafe { w.bits((1 << 0) | (1 << 1)) });

        let rx = xdmac.xdmac_chid(1);

        rx.cc().write(|w| unsafe {
            w.type_().set_bit(); // peripheral transfer
            w.dsync().set_bit(); // PER -> MEM
            w.perid().bits(XDMAC_PERID_SPI0_RX);

            w
        });

        let tx = xdmac.xdmac_chid(0);

        tx.cc().write(|w| unsafe {
            w.type_().set_bit(); // peripheral transfer
            w.dsync().clear_bit(); // MEM -> PER
            w.perid().bits(XDMAC_PERID_SPI0_TX);

            w
        });

        rx.cie().write(|w| w.bie().set_bit());
        unsafe {
            cortex_m::peripheral::NVIC::unmask(atsamx7x_hal::pac::Interrupt::XDMAC);
        }

        let banka = hal::pio::BankA::new(
            ctx.device.PIOA,
            &mut mck,
            &slck,
            BankConfiguration::default(),
        );

        // configure pin banks
        let bankb = hal::pio::BankB::new(
            ctx.device.PIOB,
            &mut mck,
            &slck,
            BankConfiguration::default(),
        );

        let bankd = hal::pio::BankD::new(
            ctx.device.PIOD,
            &mut mck,
            &slck,
            BankConfiguration::default(),
        );

        let mut drdy = banka.pa5.into_input(PullDir::Floating);
        drdy.set_interrupt(Some(InterruptType::FallingEdge));
        let led = banka.pa23.into_output(true);

        let miso = bankd.pd20.into_peripheral();
        let pck = bankd.pd22.into_peripheral();
        let mosi = bankd.pd21.into_peripheral();
        let pcs0 = bankb.pb2.into_peripheral();

        // Create a new spi, this always starts cs at index 0.
        let mut spi = Spi::new_spi0(
            ctx.device.SPI0,
            (pck, mosi, miso),
            SpiConfiguration::default().test_mode(false),
            &mut mck,
        )
        .unwrap();

        spi.setup_client(
            &pcs0,
            ClientConfiguration::default(ADS1278_SPI_BPS.bps(), hal::ehal::spi::MODE_1),
            &mck,
        )
        .unwrap();

        (
            Shared {
                spi,
                tx_buf: [0; 3],
                rx_buf: [0; 3],
                dma_busy: false,
            },
            Local {
                led,
                irq: banka.interrupts,

                tx_desc: [crate::XdmacDescriptorView1::default(); 2],
                rx_desc: [crate::XdmacDescriptorView1::default(); 2],

            },
            init::Monotonics(),
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(binds = PIOA, shared = [dma_busy, tx_buf, rx_buf])]
    fn drdy(mut cx: drdy::Context) {
        let start = cx.shared.dma_busy.lock(|b| {
            if *b {
                true
            } else {
                *b = true;
                false
            }
        });

        if start {
            return;
        }

        let spi = unsafe { &*atsamv71q21b::SPI0::ptr() };
        let xdmac = unsafe { &*atsamv71q21b::XDMAC::ptr() };

        let rx = xdmac.xdmac_chid(1);
        let tx = xdmac.xdmac_chid(0);

        xdmac.gd().write(|w| unsafe { w.bits((1 << 0) | (1 << 1)) });

        // RX destination
        cx.shared.rx_buf.lock(|buf| {
            rx.cda()
                .write(|w| unsafe { w.bits(buf.as_mut_ptr() as u32) });
        });

        // SPI RX source
        rx.csa()
            .write(|w| unsafe { w.bits(&spi.rdr() as *const _ as u32) });

        rx.cubc().write(|w| unsafe { w.bits(3) });

        // TX source
        let tx_ptr = cx.shared.tx_buf.lock(|buf| buf.as_ptr());

        tx.csa().write(|w| unsafe { w.bits(tx_ptr as u32) });

        // SPI TX destination
        tx.cda()
            .write(|w| unsafe { w.bits(&spi.tdr() as *const _ as u32) });

        tx.cubc().write(|w| unsafe { w.bits(3) });

        // START DMA
        xdmac.ge().write(|w| unsafe { w.bits((1 << 0) | (1 << 1)) });
    }

    #[task(binds = XDMAC)]
    fn xdmac_irq(_: xdmac_irq::Context) {
        let xdmac = unsafe { &*atsamv71q21b::XDMAC::ptr() };
        let chid = xdmac.xdmac_chid(1);

        let cis = chid.cis().read();

        if cis.bis().bit_is_set() {
            // clear busy flag, etc.

            // IMPORTANT: clear interrupt source
            let _ = cis.bits();

            // re-arm transfer
            chid.cubc().write(|w| unsafe { w.bits(3) });

            // re-enable channel (often required on SAMV71)
            xdmac.ge().write(|w| unsafe { w.bits(1 << 1) });
        }
    }
}
