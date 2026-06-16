//! DRDY-triggered SPI0 DMA capture on a SAM V71 Xplained Ultra board.
//!
//! PA5 falling edges start one 3-byte full-duplex SPI0 DMA transfer. The
//! received ADC sample bytes are appended to a caller-provided static buffer.
//! SPI TX sends dummy bytes only; MISO data is treated as ADC payload.
 
#![no_std]
#![no_main]
 
use panic_rtt_target as _;
 
#[rtic::app(device = atsamx7x_hal::pac, peripherals = true, dispatchers = [EFC])]
mod app {
    use atsamx7x_hal as hal;
    use hal::clocks::*;
    use hal::efc::*;
    use hal::ehal::digital::v2::OutputPin;
    use hal::fugit::RateExtU32;
    use hal::pio::*;
    use hal::serial::spi::*;
    use hal::serial::ExtBpsU32;
    use rtt_target::{rprintln, rtt_init_print};
 
    const ADS1278_SAMPLE_BYTES: usize = 3;
    const ADS1278_DUMMY_TX_BYTE: u8 = 0x00;
    const ADS1278_SPI_BPS: u32 = 24_000_000;
    const CAPTURE_SAMPLES: usize = 16;
    const CAPTURE_BUFFER_BYTES: usize = ADS1278_SAMPLE_BYTES * CAPTURE_SAMPLES;
    const CONTINUOUS_CAPTURE: bool = true;
 
    const DRDY_PIN: u8 = 5;
    const DMA_CHANNEL_RX: usize = 0;
    const DMA_CHANNEL_TX: usize = 1;
    const XDMAC_CIS_BIS: u32 = 1 << 0;
    const XDMAC_CIS_ERROR_MASK: u32 = (1 << 4) | (1 << 5) | (1 << 6);
 
    static mut CAPTURE_BUFFER: [u8; CAPTURE_BUFFER_BYTES] = [0; CAPTURE_BUFFER_BYTES];
 
    #[derive(Copy, Clone, Debug, Eq, PartialEq)]
    pub enum CaptureStartError {
        Busy,
        EmptyBuffer,
        MisalignedBuffer,
    }
 
    #[derive(Copy, Clone, Debug, Eq, PartialEq)]
    pub enum CaptureError {
        RxDma(u32),
        TxDma(u32),
    }
 
    #[derive(Copy, Clone, Debug, Eq, PartialEq)]
    pub enum CaptureProgress {
        None,
        Partial,
        SampleComplete,
        BufferComplete { samples: usize },
        Failed(CaptureError),
    }
 
    #[derive(Copy, Clone)]
    pub struct DmaTransfer {
        rx_addr: u32,
        tx_addr: u32,
        len: u32,
    }
 
    pub struct CaptureState {
        rx: Option<&'static mut [u8]>,
        offset: usize,
        sample_index: usize,
        in_flight: bool,
        rx_done: bool,
        tx_done: bool,
        done: bool,
        error: Option<CaptureError>,
        tx_buf: [u8; ADS1278_SAMPLE_BYTES],
    }
 
    impl CaptureState {
        const fn new() -> Self {
            Self {
                rx: None,
                offset: 0,
                sample_index: 0,
                in_flight: false,
                rx_done: false,
                tx_done: false,
                done: false,
                error: None,
                tx_buf: [ADS1278_DUMMY_TX_BYTE; ADS1278_SAMPLE_BYTES],
            }
        }
 
        fn begin_sample(&mut self) -> Option<DmaTransfer> {
            if self.rx.is_none() || self.done || self.error.is_some() || self.in_flight {
                return None;
            }
 
            let rx_len = self.rx.as_deref().map_or(0, |rx| rx.len());
            if self.offset >= rx_len {
                self.done = true;
                return None;
            }
 
            let rx_addr = self
                .rx
                .as_deref_mut()
                .unwrap()
                .as_mut_ptr()
                .wrapping_add(self.offset) as u32;
 
            self.in_flight = true;
            self.rx_done = false;
            self.tx_done = false;
 
            Some(DmaTransfer {
                rx_addr,
                tx_addr: self.tx_buf.as_ptr() as u32,
                len: ADS1278_SAMPLE_BYTES as u32,
            })
        }
 
        fn mark_dma_status(
            &mut self,
            rx_done: bool,
            tx_done: bool,
            error: Option<CaptureError>,
        ) -> CaptureProgress {
            if let Some(error) = error {
                self.fail(error);
                return CaptureProgress::Failed(error);
            }
 
            if !self.in_flight {
                return CaptureProgress::None;
            }
 
            self.rx_done |= rx_done;
            self.tx_done |= tx_done;
 
            if !(self.rx_done && self.tx_done) {
                return CaptureProgress::Partial;
            }
 
            self.offset += ADS1278_SAMPLE_BYTES;
            self.sample_index += 1;
            self.in_flight = false;
            self.rx_done = false;
            self.tx_done = false;
 
            let samples = self.sample_index;
            if self.rx.as_deref().is_some_and(|rx| self.offset >= rx.len()) {
                if CONTINUOUS_CAPTURE {
                    self.offset = 0;
                } else {
                    self.done = true;
                }
                CaptureProgress::BufferComplete { samples }
            } else {
                CaptureProgress::SampleComplete
            }
        }
 
        fn fail(&mut self, error: CaptureError) {
            self.error = Some(error);
            self.in_flight = false;
            self.rx_done = false;
            self.tx_done = false;
        }
    }
 
    fn start_capture(
        state: &mut CaptureState,
        rx: &'static mut [u8],
    ) -> Result<(), CaptureStartError> {
        if state.rx.is_some() && !state.done && state.error.is_none() {
            return Err(CaptureStartError::Busy);
        }
 
        if rx.is_empty() {
            return Err(CaptureStartError::EmptyBuffer);
        }
 
        if rx.len() % ADS1278_SAMPLE_BYTES != 0 {
            return Err(CaptureStartError::MisalignedBuffer);
        }
 
        rx.fill(0);
 
        state.rx = Some(rx);
        state.offset = 0;
        state.sample_index = 0;
        state.in_flight = false;
        state.rx_done = false;
        state.tx_done = false;
        state.done = false;
        state.error = None;
        state.tx_buf = [ADS1278_DUMMY_TX_BYTE; ADS1278_SAMPLE_BYTES];
 
        Ok(())
    }
 
    #[shared]
    struct Shared {
        capture: CaptureState,
    }
 
    #[local]
    struct Local {
        led: Pin<PA23, Output>,
        irq: BankInterrupts<A>,
    }
 
    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        rtt_init_print!();
        rprintln!("init");
 
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
 
        let banka = hal::pio::BankA::new(
            ctx.device.PIOA,
            &mut mck,
            &slck,
            BankConfiguration::default(),
        );
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
        let mut led = banka.pa23.into_output(true);
        led.set_high().unwrap();
 
        let miso = bankd.pd20.into_peripheral();
        let spck = bankd.pd22.into_peripheral();
        let mosi = bankd.pd21.into_peripheral();
        let pcs0 = bankb.pb2.into_peripheral();
 
        let mut spi = Spi::new_spi0(
            ctx.device.SPI0,
            (spck, mosi, miso),
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
 
        enable_xdmac_clock();
        configure_spi0_for_dma();
        configure_xdmac(ctx.device.XDMAC);
 
        let mut capture = CaptureState::new();
        let capture_buffer = unsafe {
            let ptr = core::ptr::addr_of_mut!(CAPTURE_BUFFER).cast::<u8>();
            core::slice::from_raw_parts_mut(ptr, CAPTURE_BUFFER_BYTES)
        };
 
        match start_capture(&mut capture, capture_buffer) {
            Ok(()) => rprintln!(
                "armed ADC DMA capture: {} samples, {} bytes{}",
                CAPTURE_SAMPLES,
                CAPTURE_BUFFER_BYTES,
                if CONTINUOUS_CAPTURE {
                    ", continuous"
                } else {
                    ""
                }
            ),
            Err(error) => rprintln!("failed to arm DMA capture: {:?}", error),
        }
 
        (
            Shared { capture },
            Local {
                led,
                irq: banka.interrupts,
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
 
    #[task(binds = PIOA, shared = [capture], local = [irq])]
    fn drdy(mut cx: drdy::Context) {
        if !cx.local.irq.iter().any(|pin| pin == DRDY_PIN) {
            return;
        }
 
        let transfer = cx.shared.capture.lock(|capture| capture.begin_sample());
 
        if let Some(transfer) = transfer {
            start_dma_transfer(transfer);
        }
    }
 
    #[task(binds = XDMAC, shared = [capture], local = [led])]
    fn xdmac_irq(mut cx: xdmac_irq::Context) {
        let xdmac = unsafe { &*atsamv71q21b::XDMAC::ptr() };
        let rx_status = xdmac.xdmac_chid(DMA_CHANNEL_RX).cis().read().bits();
        let tx_status = xdmac.xdmac_chid(DMA_CHANNEL_TX).cis().read().bits();
 
        let error = if rx_status & XDMAC_CIS_ERROR_MASK != 0 {
            Some(CaptureError::RxDma(rx_status))
        } else if tx_status & XDMAC_CIS_ERROR_MASK != 0 {
            Some(CaptureError::TxDma(tx_status))
        } else {
            None
        };
 
        let progress = cx.shared.capture.lock(|capture| {
            capture.mark_dma_status(
                rx_status & XDMAC_CIS_BIS != 0,
                tx_status & XDMAC_CIS_BIS != 0,
                error,
            )
        });
 
        match progress {
            CaptureProgress::BufferComplete { samples } => {
                if CONTINUOUS_CAPTURE {
                    if samples == CAPTURE_SAMPLES {
                        rprintln!("ADC DMA capture ring wrapped: {} samples captured", samples);
                    }
                } else {
                    disable_dma_channels();
                    release_spi0_chip_select(true);
                    cx.local.led.set_low().unwrap();
                    rprintln!("ADC DMA capture complete: {} samples captured", samples);
                }
            }
            CaptureProgress::Failed(error) => {
                disable_dma_channels();
                release_spi0_chip_select(false);
                cx.local.led.set_high().unwrap();
                rprintln!("ADC DMA capture failed: {:?}", error);
            }
            CaptureProgress::None | CaptureProgress::Partial | CaptureProgress::SampleComplete => {}
        }
    }
 
    fn enable_xdmac_clock() {
        let pmc = unsafe { &*atsamv71q21b::PMC::ptr() };
        pmc.pcer1().write(|w| w.pid58().set_bit());
    }
 
    fn configure_spi0_for_dma() {
        let spi = unsafe { &*atsamv71q21b::SPI0::ptr() };
 
        spi.mr().modify(|_, w| {
            w.ps().clear_bit();
            w.pcs().npcs0();
            w
        });
    }
 
    fn configure_xdmac(xdmac: atsamv71q21b::XDMAC) {
        xdmac.gd().write(|w| {
            w.di0().set_bit();
            w.di1().set_bit();
            w
        });
 
        let rx = xdmac.xdmac_chid(DMA_CHANNEL_RX);
        rx.cc().write(|w| {
            w.type_().per_tran();
            w.dsync().per2mem();
            w.swreq().hwr_connected();
            w.sam().fixed_am();
            w.dam().incremented_am();
            w.sif().ahb_if1();
            w.dif().ahb_if0();
            w.dwidth().byte();
            w.csize().chk_1();
            w.mbsize().single();
            w.perid().spi0_rx();
            w
        });
        rx.cie().write(|w| {
            w.bie().set_bit();
            w.rbie().set_bit();
            w.wbie().set_bit();
            w.roie().set_bit();
            w
        });
 
        let tx = xdmac.xdmac_chid(DMA_CHANNEL_TX);
        tx.cc().write(|w| {
            w.type_().per_tran();
            w.dsync().mem2per();
            w.swreq().hwr_connected();
            w.sam().incremented_am();
            w.dam().fixed_am();
            w.sif().ahb_if0();
            w.dif().ahb_if1();
            w.dwidth().byte();
            w.csize().chk_1();
            w.mbsize().single();
            w.perid().spi0_tx();
            w
        });
        tx.cie().write(|w| {
            w.bie().set_bit();
            w.rbie().set_bit();
            w.wbie().set_bit();
            w.roie().set_bit();
            w
        });
 
        xdmac.gie().write(|w| {
            w.ie0().set_bit();
            w.ie1().set_bit();
            w
        });
 
        unsafe {
            cortex_m::peripheral::NVIC::unmask(atsamx7x_hal::pac::Interrupt::XDMAC);
        }
    }
 
    fn start_dma_transfer(transfer: DmaTransfer) {
        let spi = unsafe { &*atsamv71q21b::SPI0::ptr() };
        let xdmac = unsafe { &*atsamv71q21b::XDMAC::ptr() };
        let rx = xdmac.xdmac_chid(DMA_CHANNEL_RX);
        let tx = xdmac.xdmac_chid(DMA_CHANNEL_TX);
 
        xdmac.gd().write(|w| {
            w.di0().set_bit();
            w.di1().set_bit();
            w
        });
 
        let _ = rx.cis().read().bits();
        let _ = tx.cis().read().bits();
 
        rx.csa()
            .write(|w| unsafe { w.bits(spi.rdr().as_ptr() as u32) });
        rx.cda().write(|w| unsafe { w.bits(transfer.rx_addr) });
        rx.cubc().write(|w| unsafe { w.bits(transfer.len) });
 
        tx.csa().write(|w| unsafe { w.bits(transfer.tx_addr) });
        tx.cda()
            .write(|w| unsafe { w.bits(spi.tdr().as_ptr() as u32) });
        tx.cubc().write(|w| unsafe { w.bits(transfer.len) });
 
        cortex_m::asm::dmb();
 
        xdmac.ge().write(|w| {
            w.en0().set_bit();
            w.en1().set_bit();
            w
        });
    }
 
    fn disable_dma_channels() {
        let xdmac = unsafe { &*atsamv71q21b::XDMAC::ptr() };
        xdmac.gd().write(|w| {
            w.di0().set_bit();
            w.di1().set_bit();
            w
        });
    }
 
    fn release_spi0_chip_select(wait_for_tx_empty: bool) {
        let spi = unsafe { &*atsamv71q21b::SPI0::ptr() };
 
        if wait_for_tx_empty {
            while spi.sr().read().txempty().bit_is_clear() {}
        }
 
        spi.cr().write(|w| w.lastxfer().set_bit());
    }
}

