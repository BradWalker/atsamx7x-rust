/*!
Extensible DMA Controller (XDMAC).
---

This module provides a small safe ownership layer over the SAMx7x
XDMAC. It supports runtime channel allocation, memory-to-memory
copies, SPI transfers, and USART UART-mode transfers.

# Cache coherency

The Cortex-M7 data cache is not managed by this module. Buffers used
with DMA must be placed in memory that is coherent with the XDMAC, or
the application must perform the required cache clean/invalidate
operations before and after transfers.
*/

use crate::clocks::{HostClock, PeripheralIdentifier};
use crate::generics;
use crate::pac::XDMAC;
use core::marker::PhantomData;

#[cfg(not(feature = "__pins-64"))]
use crate::serial::spi::{Spi, SpiMeta};
use crate::serial::usart::{uart, UsartMeta, UsartMode};

const CHANNELS: u8 = 24;
const UBLEN_MAX: usize = 0x00ff_ffff;

const CC_TYPE_PERIPHERAL: u32 = 1 << 0;
const CC_DSYNC_PER_TO_MEM: u32 = 1 << 4;
const CC_DWIDTH_BYTE: u32 = 0 << 11;
const CC_DWIDTH_HALFWORD: u32 = 1 << 11;
const CC_DWIDTH_WORD: u32 = 2 << 11;
const CC_SRC_INCREMENTED: u32 = 1 << 16;
const CC_DST_INCREMENTED: u32 = 1 << 18;
const CC_PERID_SHIFT: u32 = 24;

const CIE_BIE: u32 = 1 << 0;
const CIE_RBIE: u32 = 1 << 4;
const CIE_WBIE: u32 = 1 << 5;
const CIE_ROIE: u32 = 1 << 6;

/// Runtime owner for the XDMAC peripheral.
///
/// `Xdmac` owns the hardware peripheral token and tracks which of the
/// 24 hardware channels have been leased to transfer handles.
pub struct Xdmac {
    xdmac: XDMAC,
    allocated: u32,
}

impl Xdmac {
    /// Creates an XDMAC manager and enables the peripheral clock.
    pub fn new(xdmac: XDMAC, mck: &mut HostClock) -> Self {
        mck.enable_peripheral(PeripheralIdentifier::XDMAC);
        xdmac.gd().write(|w| unsafe { w.bits((1 << CHANNELS) - 1) });

        Self {
            xdmac,
            allocated: 0,
        }
    }

    /// Allocates the first free channel.
    ///
    /// The returned [`Channel`] is exclusive until it is released via
    /// [`Xdmac::release_channel`] or consumed by a transfer and later
    /// returned by [`Transfer::free`] or [`Transfer::cancel`].
    pub fn alloc_channel(&mut self) -> Result<Channel, DmaError> {
        for index in 0..CHANNELS {
            let mask = 1 << index;
            if self.allocated & mask == 0 {
                self.allocated |= mask;
                return Ok(Channel { index });
            }
        }

        Err(DmaError::NoChannel)
    }

    /// Releases a previously allocated idle channel.
    pub fn release_channel(&mut self, channel: Channel) -> Result<(), DmaError> {
        let mask = channel.mask();
        if self.allocated & mask == 0 {
            return Err(DmaError::UnallocatedChannel);
        }

        self.disable(channel.index);
        self.allocated &= !mask;
        Ok(())
    }

    /// Starts a memory-to-memory copy transfer.
    pub fn memcpy<'a>(
        &mut self,
        channel: Channel,
        src: &'a [u8],
        dst: &'a mut [u8],
        config: TransferConfig,
    ) -> Result<Transfer<'a, MemToMem>, DmaError> {
        let len = src.len();
        validate_len(len)?;
        if dst.len() < len {
            return Err(DmaError::BufferTooSmall);
        }

        self.configure(
            &channel,
            src.as_ptr() as u32,
            dst.as_mut_ptr() as u32,
            len,
            config.width.bits() | CC_SRC_INCREMENTED | CC_DST_INCREMENTED,
        );
        self.enable(channel.index);

        Ok(Transfer::new(channel))
    }

// FIXME - this needs to be refactored out..
/*
    /// Starts an SPI memory-to-peripheral transmit transfer.
    #[cfg(not(feature = "__pins-64"))]
    pub fn spi_tx<'a, M: SpiMeta + SpiDma>(
        &mut self,
        channel: Channel,
        spi: &'a mut Spi<M>,
        bytes: &'a [u8],
        config: TransferConfig,
    ) -> Result<Transfer<'a, PeripheralTx>, DmaError> {
        validate_len(bytes.len())?;
        self.configure(
            &channel,
            bytes.as_ptr() as u32,
            spi.dma_tdr_addr(),
            bytes.len(),
            CC_TYPE_PERIPHERAL
                | config.width.bits()
                | CC_SRC_INCREMENTED
                | ((M::TX_PERID as u32) << CC_PERID_SHIFT),
        );
        self.enable(channel.index);

        Ok(Transfer::new(channel))
    }

    /// Starts an SPI peripheral-to-memory receive transfer.
    #[cfg(not(feature = "__pins-64"))]
    pub fn spi_rx<'a, M: SpiMeta + SpiDma>(
        &mut self,
        channel: Channel,
        spi: &'a mut Spi<M>,
        bytes: &'a mut [u8],
        config: TransferConfig,
    ) -> Result<Transfer<'a, PeripheralRx>, DmaError> {
        validate_len(bytes.len())?;
        self.configure(
            &channel,
            spi.dma_rdr_addr(),
            bytes.as_mut_ptr() as u32,
            bytes.len(),
            CC_TYPE_PERIPHERAL
                | CC_DSYNC_PER_TO_MEM
                | config.width.bits()
                | CC_DST_INCREMENTED
                | ((M::RX_PERID as u32) << CC_PERID_SHIFT),
        );
        self.enable(channel.index);

        Ok(Transfer::new(channel))
    }

    /// Starts a full-duplex SPI transfer using separate TX and RX channels.
    #[cfg(not(feature = "__pins-64"))]
    pub fn spi_transfer<'a, M: SpiMeta + SpiDma>(
        &mut self,
        tx_channel: Channel,
        rx_channel: Channel,
        spi: &'a mut Spi<M>,
        tx: &'a [u8],
        rx: &'a mut [u8],
        config: TransferConfig,
    ) -> Result<Transfer<'a, FullDuplex>, DmaError> {
        if tx.len() != rx.len() {
            return Err(DmaError::LengthMismatch);
        }
        validate_len(tx.len())?;

        self.configure(
            &rx_channel,
            spi.dma_rdr_addr(),
            rx.as_mut_ptr() as u32,
            rx.len(),
            CC_TYPE_PERIPHERAL
                | CC_DSYNC_PER_TO_MEM
                | config.width.bits()
                | CC_DST_INCREMENTED
                | ((M::RX_PERID as u32) << CC_PERID_SHIFT),
        );
        self.configure(
            &tx_channel,
            tx.as_ptr() as u32,
            spi.dma_tdr_addr(),
            tx.len(),
            CC_TYPE_PERIPHERAL
                | config.width.bits()
                | CC_SRC_INCREMENTED
                | ((M::TX_PERID as u32) << CC_PERID_SHIFT),
        );
        self.enable(rx_channel.index);
        self.enable(tx_channel.index);

        Ok(Transfer::new_two(tx_channel, rx_channel))
    }

    /// Starts a USART UART-mode memory-to-peripheral transmit transfer.
    pub fn usart_tx<'a, M: UsartMeta + UsartDma>(
        &mut self,
        channel: Channel,
        tx: &'a mut uart::Tx<M>,
        bytes: &'a [u8],
        config: TransferConfig,
    ) -> Result<Transfer<'a, PeripheralTx>, DmaError> {
        if tx.dma_mode() != UsartMode::Uart {
            return Err(DmaError::InvalidMode);
        }
        validate_len(bytes.len())?;
        self.configure(
            &channel,
            bytes.as_ptr() as u32,
            tx.dma_thr_addr(),
            bytes.len(),
            CC_TYPE_PERIPHERAL
                | config.width.bits()
                | CC_SRC_INCREMENTED
                | ((M::TX_PERID as u32) << CC_PERID_SHIFT),
        );
        self.enable(channel.index);

        Ok(Transfer::new(channel))
    }

    /// Starts a USART UART-mode peripheral-to-memory receive transfer.
    pub fn usart_rx<'a, M: UsartMeta + UsartDma>(
        &mut self,
        channel: Channel,
        rx: &'a mut uart::Rx<M>,
        bytes: &'a mut [u8],
        config: TransferConfig,
    ) -> Result<Transfer<'a, PeripheralRx>, DmaError> {
        if rx.dma_mode() != UsartMode::Uart {
            return Err(DmaError::InvalidMode);
        }
        validate_len(bytes.len())?;
        self.configure(
            &channel,
            rx.dma_rhr_addr(),
            bytes.as_mut_ptr() as u32,
            bytes.len(),
            CC_TYPE_PERIPHERAL
                | CC_DSYNC_PER_TO_MEM
                | config.width.bits()
                | CC_DST_INCREMENTED
                | ((M::RX_PERID as u32) << CC_PERID_SHIFT),
        );
        self.enable(channel.index);

        Ok(Transfer::new(channel))
    }

    /// Starts a USART UART-mode memory-to-peripheral transmit transfer.
    pub fn usart_uart_tx<'a, M: UsartMeta + UsartDma>(
        &mut self,
        channel: Channel,
        uart: &'a mut uart::Uart<M>,
        bytes: &'a [u8],
        config: TransferConfig,
    ) -> Result<Transfer<'a, PeripheralTx>, DmaError> {
        if uart.dma_mode() != UsartMode::Uart {
            return Err(DmaError::InvalidMode);
        }
        validate_len(bytes.len())?;
        self.configure(
            &channel,
            bytes.as_ptr() as u32,
            uart.dma_thr_addr(),
            bytes.len(),
            CC_TYPE_PERIPHERAL
                | config.width.bits()
                | CC_SRC_INCREMENTED
                | ((M::TX_PERID as u32) << CC_PERID_SHIFT),
        );
        self.enable(channel.index);

        Ok(Transfer::new(channel))
    }

    /// Starts a USART UART-mode peripheral-to-memory receive transfer.
    pub fn usart_uart_rx<'a, M: UsartMeta + UsartDma>(
        &mut self,
        channel: Channel,
        uart: &'a mut uart::Uart<M>,
        bytes: &'a mut [u8],
        config: TransferConfig,
    ) -> Result<Transfer<'a, PeripheralRx>, DmaError> {
        if uart.dma_mode() != UsartMode::Uart {
            return Err(DmaError::InvalidMode);
        }
        validate_len(bytes.len())?;
        self.configure(
            &channel,
            uart.dma_rhr_addr(),
            bytes.as_mut_ptr() as u32,
            bytes.len(),
            CC_TYPE_PERIPHERAL
                | CC_DSYNC_PER_TO_MEM
                | config.width.bits()
                | CC_DST_INCREMENTED
                | ((M::RX_PERID as u32) << CC_PERID_SHIFT),
        );
        self.enable(channel.index);

        Ok(Transfer::new(channel))
    }
*/

    fn configure(&mut self, channel: &Channel, src: u32, dst: u32, len: usize, cc: u32) {
        self.disable(channel.index);
        let chid = self.xdmac.xdmac_chid(channel.index as usize);
        chid.cid().write(|w| unsafe { w.bits(u32::MAX) });
        let _ = chid.cis().read().bits();
        chid.csa().write(|w| unsafe { w.bits(src) });
        chid.cda().write(|w| unsafe { w.bits(dst) });
        chid.cnda().write(|w| unsafe { w.bits(0) });
        chid.cndc().write(|w| unsafe { w.bits(0) });
        chid.cubc().write(|w| unsafe { w.bits(len as u32) });
        chid.cc().write(|w| unsafe { w.bits(cc) });
        chid.cie()
            .write(|w| unsafe { w.bits(CIE_BIE | CIE_RBIE | CIE_WBIE | CIE_ROIE) });
    }

    fn enable(&mut self, index: u8) {
        self.xdmac.ge().write(|w| unsafe { w.bits(1 << index) });
    }

    fn disable(&mut self, index: u8) {
        self.xdmac.gd().write(|w| unsafe { w.bits(1 << index) });
    }
}

/// A leased XDMAC channel.
#[derive(Debug, Eq, PartialEq)]
pub struct Channel {
    index: u8,
}

impl Channel {
    /// Returns the channel number.
    pub fn index(&self) -> u8 {
        self.index
    }

    fn mask(&self) -> u32 {
        1 << self.index
    }
}

/// Transfer configuration shared by simple transfer constructors.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct TransferConfig {
    /// Width of one DMA data item.
    pub width: DataWidth,
}

impl Default for TransferConfig {
    fn default() -> Self {
        Self {
            width: DataWidth::Byte,
        }
    }
}

/// XDMAC transfer data width.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DataWidth {
    /// 8-bit data items.
    Byte,
    /// 16-bit data items.
    HalfWord,
    /// 32-bit data items.
    Word,
}

impl DataWidth {
    fn bits(self) -> u32 {
        match self {
            Self::Byte => CC_DWIDTH_BYTE,
            Self::HalfWord => CC_DWIDTH_HALFWORD,
            Self::Word => CC_DWIDTH_WORD,
        }
    }
}

/// A running or completed DMA transfer.
pub struct Transfer<'a, K> {
    primary: Channel,
    secondary: Option<Channel>,
    _kind: PhantomData<K>,
    _borrow: PhantomData<&'a mut ()>,
}

impl<'a, K> Transfer<'a, K> {
    fn new(primary: Channel) -> Self {
        Self {
            primary,
            secondary: None,
            _kind: PhantomData,
            _borrow: PhantomData,
        }
    }

    fn new_two(primary: Channel, secondary: Channel) -> Self {
        Self {
            primary,
            secondary: Some(secondary),
            _kind: PhantomData,
            _borrow: PhantomData,
        }
    }

    /// Returns the primary channel index.
    pub fn channel(&self) -> u8 {
        self.primary.index
    }

    /// Returns the secondary channel index for two-channel transfers.
    pub fn secondary_channel(&self) -> Option<u8> {
        self.secondary.as_ref().map(Channel::index)
    }

    /// Reads and clears the XDMAC channel status bits for this transfer.
    pub fn status(&mut self) -> TransferStatus {
        let primary = read_channel_status(self.primary.index);
        let secondary = self
            .secondary
            .as_ref()
            .map(|ch| read_channel_status(ch.index));
        TransferStatus { primary, secondary }
    }

    /// Returns `true` when all channels have reported end-of-block.
    pub fn is_complete(&mut self) -> bool {
        self.status().is_complete()
    }

    /// Blocks until the transfer completes or reports an error.
    ///
    /// The transfer still owns its channel or channels after this
    /// method returns. Use [`Transfer::free`] after successful
    /// completion, or [`Transfer::cancel`] to disable and recover
    /// channels after an error.
    pub fn wait(&mut self) -> Result<(), DmaError> {
        loop {
            let status = self.status();
            if status.has_error() {
                return Err(DmaError::Transfer(status));
            }
            if status.is_complete() {
                return Ok(());
            }
        }
    }

    /// Disables the channel or channels and returns them to the caller.
    pub fn cancel(self) -> ChannelSet {
        disable_channel(self.primary.index);
        if let Some(secondary) = &self.secondary {
            disable_channel(secondary.index);
        }
        self.free()
    }

    /// Returns the channel or channels owned by this transfer.
    pub fn free(self) -> ChannelSet {
        ChannelSet {
            primary: self.primary,
            secondary: self.secondary,
        }
    }
}

/// Channel set returned by a completed or cancelled transfer.
pub struct ChannelSet {
    /// Primary channel used by the transfer.
    pub primary: Channel,
    /// Secondary channel used by full-duplex transfers.
    pub secondary: Option<Channel>,
}

/// Status for a one- or two-channel transfer.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct TransferStatus {
    /// Status of the primary channel.
    pub primary: ChannelStatus,
    /// Status of the secondary channel, if present.
    pub secondary: Option<ChannelStatus>,
}

impl TransferStatus {
    /// Returns `true` when every channel has completed its block.
    pub fn is_complete(&self) -> bool {
        self.primary.block_complete
            && self
                .secondary
                .map(|status| status.block_complete)
                .unwrap_or(true)
    }

    /// Returns `true` when any channel has reported an error.
    pub fn has_error(&self) -> bool {
        self.primary.has_error()
            || self
                .secondary
                .map(|status| status.has_error())
                .unwrap_or(false)
    }
}

/// Decoded XDMAC channel interrupt status.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct ChannelStatus {
    /// End of block.
    pub block_complete: bool,
    /// End of linked list.
    pub linked_list_complete: bool,
    /// End of disable.
    pub disabled: bool,
    /// End of flush.
    pub flushed: bool,
    /// Read bus error.
    pub read_bus_error: bool,
    /// Write bus error.
    pub write_bus_error: bool,
    /// Request overflow error.
    pub request_overflow: bool,
}

impl ChannelStatus {
    fn from_bits(bits: u32) -> Self {
        Self {
            block_complete: bits & (1 << 0) != 0,
            linked_list_complete: bits & (1 << 1) != 0,
            disabled: bits & (1 << 2) != 0,
            flushed: bits & (1 << 3) != 0,
            read_bus_error: bits & (1 << 4) != 0,
            write_bus_error: bits & (1 << 5) != 0,
            request_overflow: bits & (1 << 6) != 0,
        }
    }

    /// Returns `true` when any error bit is set.
    pub fn has_error(&self) -> bool {
        self.read_bus_error || self.write_bus_error || self.request_overflow
    }
}

/// Errors reported by the XDMAC abstraction.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DmaError {
    /// No channel is available from the runtime allocator.
    NoChannel,
    /// The channel was not allocated by this manager.
    UnallocatedChannel,
    /// The requested transfer length does not fit in an XDMAC microblock.
    LengthTooLarge,
    /// The destination buffer is too small for the source buffer.
    BufferTooSmall,
    /// Full-duplex TX and RX buffers have different lengths.
    LengthMismatch,
    /// The peripheral is not in the required operating mode.
    InvalidMode,
    /// A transfer reported an error status.
    Transfer(TransferStatus),
}

/// Marker for memory-to-memory transfers.
pub enum MemToMem {}

/// Marker for memory-to-peripheral transfers.
pub enum PeripheralTx {}

/// Marker for peripheral-to-memory transfers.
pub enum PeripheralRx {}

/// Marker for full-duplex peripheral transfers.
pub enum FullDuplex {}

/// XDMAC descriptor view 0.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct DescriptorView0 {
    /// Next descriptor address.
    pub mbr_nda: u32,
    /// Microblock control member.
    pub mbr_ubc: u32,
    /// Destination address member.
    pub mbr_da: u32,
}

/// XDMAC descriptor view 1.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct DescriptorView1 {
    /// Next descriptor address.
    pub mbr_nda: u32,
    /// Microblock control member.
    pub mbr_ubc: u32,
    /// Source address member.
    pub mbr_sa: u32,
    /// Destination address member.
    pub mbr_da: u32,
}

/// XDMAC descriptor view 2.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct DescriptorView2 {
    /// Next descriptor address.
    pub mbr_nda: u32,
    /// Microblock control member.
    pub mbr_ubc: u32,
    /// Source address member.
    pub mbr_sa: u32,
    /// Destination address member.
    pub mbr_da: u32,
    /// Channel configuration register value.
    pub mbr_cfg: u32,
}

/// XDMAC descriptor view 3.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct DescriptorView3 {
    /// Next descriptor address.
    pub mbr_nda: u32,
    /// Microblock control member.
    pub mbr_ubc: u32,
    /// Source address member.
    pub mbr_sa: u32,
    /// Destination address member.
    pub mbr_da: u32,
    /// Channel configuration register value.
    pub mbr_cfg: u32,
    /// Block control member.
    pub mbr_bc: u32,
    /// Data stride member.
    pub mbr_ds: u32,
    /// Source microblock stride member.
    pub mbr_sus: u32,
    /// Destination microblock stride member.
    pub mbr_dus: u32,
}

/// Sealed SPI DMA request metadata.
#[cfg(not(feature = "__pins-64"))]
pub trait SpiDma: generics::Sealed {
    /// XDMAC transmit request line.
    const TX_PERID: u8;
    /// XDMAC receive request line.
    const RX_PERID: u8;
}

/// Sealed USART DMA request metadata.
pub trait UsartDma: generics::Sealed {
    /// XDMAC transmit request line.
    const TX_PERID: u8;
    /// XDMAC receive request line.
    const RX_PERID: u8;
}

#[cfg(not(feature = "__pins-64"))]
impl SpiDma for crate::serial::spi::Spi0 {
    const TX_PERID: u8 = 1;
    const RX_PERID: u8 = 2;
}

#[cfg(all(not(feature = "__pins-64"), feature = "__pins-144"))]
impl SpiDma for crate::serial::spi::Spi1 {
    const TX_PERID: u8 = 3;
    const RX_PERID: u8 = 4;
}

impl UsartDma for crate::serial::usart::Usart0 {
    const TX_PERID: u8 = 7;
    const RX_PERID: u8 = 8;
}

#[cfg(feature = "reconfigurable-system-pins")]
impl UsartDma for crate::serial::usart::Usart1 {
    const TX_PERID: u8 = 9;
    const RX_PERID: u8 = 10;
}

#[cfg(not(feature = "__pins-64"))]
impl UsartDma for crate::serial::usart::Usart2 {
    const TX_PERID: u8 = 11;
    const RX_PERID: u8 = 12;
}

fn validate_len(len: usize) -> Result<(), DmaError> {
    if len > UBLEN_MAX {
        Err(DmaError::LengthTooLarge)
    } else {
        Ok(())
    }
}

fn read_channel_status(index: u8) -> ChannelStatus {
    let xdmac = unsafe { &*XDMAC::ptr() };
    ChannelStatus::from_bits(xdmac.xdmac_chid(index as usize).cis().read().bits())
}

fn disable_channel(index: u8) {
    let xdmac = unsafe { &*XDMAC::ptr() };
    xdmac.gd().write(|w| unsafe { w.bits(1 << index) });
}
