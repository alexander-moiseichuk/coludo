//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! SPI2, the IMU bus (doc/specs/avionics-port.md M5, design decisions): a thin `spi_master` wrapper. The config's
//! `spi` bus 1 is SPI2_HOST (MicroPython's `SPI(1)`); plan B ran it on SCK 48 / MOSI 47 / MISO 46 at 5 MHz, mode 3.
//!
//! - **No DMA:** the data goes through the 16-word buffer (64 B a transaction), so no transfer allocates or syncs
//!   the cache (with DMA, every unaligned buffer would, spi_master.c:1186-1262), and buffers may be anywhere.
//! - **Polling, no interrupt flags, no callbacks:** `spi_device_polling_transmit` busy-waits on the caller's core, no
//!   ISR and no context switch (spi_master.c:1424-1517); the interrupt the first device allocates is pinned to core 1
//!   (`isr_cpu_id`) should a queued transfer ever be used. There is no Rust `post_cb` (esp-idf-hal's SpiDeviceDriver
//!   always installs one), so nothing of ours runs in an interrupt (the §4.7 rule for this bus).
//! - **One full-duplex chip-select frame per transaction:** the P4 has no half-duplex transfer that both writes and
//!   reads, so a register read is one 1+N byte frame (the drivers set the read flag themselves).
//! - **Discard on first use (plan B spibus.py:25, 44-75):** on plan B's bus the first 1..11 framed reads after a
//!   peripheral appeared returned 0x00, so a handle's first transaction first clocks 16 framed reads of register 0
//!   (side-effect-free on both parts) and drops them, errors too: a resync, never a probe.
//!
//! Its owner is M6's control thread; in M5 the SPI parts are set up on main and held.

use board_config::SpiSpecification;
use embedded_hal::spi::{ErrorKind, ErrorType, Operation};
use esp_idf_svc::sys;

/// Framed reads discarded on a handle's first use (plan B spibus.py:25 `_RESYNC_READS`).
pub const DISCARD_READS: usize = 16;

/// The most bytes one transaction moves without DMA: the peripheral's data buffer.
pub const FRAME_BYTES: usize = 64;

/// The register-0 read a discard frame clocks: the read flag, then one byte.
const DISCARD_FRAME: [u8; 2] = [0x80, 0x00];

/// The bytes the hardware reads per word of a transmit buffer: the HAL loads the data registers by whole 32-bit
/// words (spi_ll_write_buffer), so a frame's transmit buffer is backed to the next word.
const WORD_BYTES: usize = 4;

/// Why an SPI transaction failed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SpiError {
    /// The operations need more than [`FRAME_BYTES`] in one frame.
    TooLong,
    /// A delay inside a transaction: not one frame.
    Unsupported,
    /// The driver refused.
    Driver(sys::esp_err_t),
}

impl core::fmt::Display for SpiError {
    fn fmt(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            SpiError::TooLong => write!(formatter, "more than {FRAME_BYTES} bytes in one frame"),
            SpiError::Unsupported => formatter.write_str("not one frame (a delay inside the transaction)"),
            SpiError::Driver(code) => write!(formatter, "driver error {code:#x}"),
        }
    }
}

impl embedded_hal::spi::Error for SpiError {
    fn kind(&self) -> ErrorKind {
        ErrorKind::Other
    }
}

/// SPI2, initialised for the program: never freed, so never opened twice (a second open is refused).
pub struct SpiBus {
    /// The bus's pins, mode and clock, from the board config.
    specification: SpiSpecification,
}

impl SpiBus {
    /// Initialises SPI2 on the config's `buses.spi.1` (plan B spibus.py:131-147): no DMA, a 64-byte transfer limit,
    /// master, no interrupt flags, the interrupt pinned to core 1.
    ///
    /// Errors: the bus is not spi:1 (the only one the P4 frees, SPI2), or the driver refused (the host taken, bad
    /// pins).
    pub fn open(specification: SpiSpecification) -> anyhow::Result<SpiBus> {
        anyhow::ensure!(
            specification.id == 1,
            "no spi:{} on this MCU (spi:1 is SPI2)",
            specification.id
        );
        let mut config = sys::spi_bus_config_t {
            sclk_io_num: specification.sck as i32,
            data4_io_num: -1,
            data5_io_num: -1,
            data6_io_num: -1,
            data7_io_num: -1,
            max_transfer_sz: FRAME_BYTES as i32,
            flags: sys::SPICOMMON_BUSFLAG_MASTER,
            isr_cpu_id: sys::esp_intr_cpu_affinity_t_ESP_INTR_CPU_AFFINITY_1,
            intr_flags: 0,
            ..Default::default()
        };
        config.__bindgen_anon_1.mosi_io_num = specification.mosi as i32;
        config.__bindgen_anon_2.miso_io_num = specification.miso as i32;
        config.__bindgen_anon_3.quadwp_io_num = -1;
        config.__bindgen_anon_4.quadhd_io_num = -1;
        // SAFETY: a filled config; the bus lives for the program.
        sys::esp!(unsafe {
            sys::spi_bus_initialize(
                sys::spi_host_device_t_SPI2_HOST,
                &config,
                sys::spi_common_dma_t_SPI_DMA_DISABLED,
            )
        })?;
        Ok(SpiBus { specification })
    }

    /// A device handle for chip select `chip_select`, at the bus's mode and clock, full duplex, one queued
    /// transaction (polling never queues).
    ///
    /// Errors: the driver refused (all six hardware chip selects taken, a bad pin).
    pub fn handle(&self, chip_select: u8) -> anyhow::Result<SpiHandle> {
        let config = sys::spi_device_interface_config_t {
            mode: self.specification.mode,
            clock_speed_hz: self.specification.clock_hz as i32,
            spics_io_num: chip_select as i32,
            queue_size: 1,
            ..Default::default()
        };
        let mut handle: sys::spi_device_handle_t = core::ptr::null_mut();
        // SAFETY: a filled config and a live output on the initialised host.
        sys::esp!(unsafe { sys::spi_bus_add_device(sys::spi_host_device_t_SPI2_HOST, &config, &mut handle) })?;
        Ok(SpiHandle {
            handle,
            chip_select,
            synced: false,
        })
    }
}

/// Holds chip select `chip_select` high (inactive): an enabled SPI part with no driver yet, whose floating chip select
/// could make it fight the IMU on MISO (plan B spibus.py:42: chip selects idle high).
///
/// Errors: the pin would not configure.
pub fn hold_high(chip_select: u8) -> anyhow::Result<()> {
    let pin = chip_select as sys::gpio_num_t;
    // SAFETY: plain GPIO configuration of a pin the board config gives this part. The level goes first: after the
    // reset the output latch is low, and the select would pulse low as the output turns on.
    unsafe {
        sys::esp!(sys::gpio_reset_pin(pin))?;
        sys::esp!(sys::gpio_set_level(pin, 1))?;
        sys::esp!(sys::gpio_set_direction(pin, sys::gpio_mode_t_GPIO_MODE_OUTPUT))?;
    }
    Ok(())
}

/// One SPI2 device: its handle, and whether its first-use discard ran.
pub struct SpiHandle {
    /// The driver's handle.
    handle: sys::spi_device_handle_t,
    /// The chip-select pin.
    chip_select: u8,
    /// The discard ran.
    synced: bool,
}

// SAFETY: IDF's handle is usable from any task; one task uses a handle at a time (main at setup, the owner later).
unsafe impl Send for SpiHandle {}

impl SpiHandle {
    /// One full-duplex frame of `transmit.len()` bytes, received into `receive`. The memory past `transmit` up to
    /// the next [`WORD_BYTES`] boundary is read too (never sent): the callers' buffers extend that far.
    fn frame(&mut self, transmit: &[u8], receive: &mut [u8]) -> Result<(), SpiError> {
        let mut transaction = sys::spi_transaction_t {
            length: transmit.len() * 8,
            rxlength: receive.len() * 8,
            ..Default::default()
        };
        transaction.__bindgen_anon_1.tx_buffer = transmit.as_ptr().cast();
        transaction.__bindgen_anon_2.rx_buffer = receive.as_mut_ptr().cast();
        // SAFETY: a live handle; both buffers outlive the polled transaction, which completes before the call returns.
        match unsafe { sys::spi_device_polling_transmit(self.handle, &mut transaction) } {
            sys::ESP_OK => Ok(()),
            code => Err(SpiError::Driver(code)),
        }
    }

    /// The first-use discard: 16 framed register-0 reads, dropped, errors too (plan B spibus.py:44-75).
    fn discard(&mut self) {
        self.synced = true;
        let mut transmit = [0u8; WORD_BYTES];
        transmit[..DISCARD_FRAME.len()].copy_from_slice(&DISCARD_FRAME);
        for _ in 0..DISCARD_READS {
            let mut answer = [0u8; DISCARD_FRAME.len()];
            let _ = self.frame(&transmit[..DISCARD_FRAME.len()], &mut answer);
        }
    }
}

impl Drop for SpiHandle {
    /// Frees the chip select's slot on the bus (six at most).
    fn drop(&mut self) {
        // SAFETY: a handle this bus added; no transaction is in flight (polling completes inside each call).
        if let Err(error) = sys::esp!(unsafe { sys::spi_bus_remove_device(self.handle) }) {
            log::warn!("spi:1 :: CS GPIO{} would not leave the bus: {error}", self.chip_select);
        }
    }
}

impl ErrorType for SpiHandle {
    type Error = SpiError;
}

impl embedded_hal::spi::SpiDevice for SpiHandle {
    /// Every operation in ONE chip-select frame of at most [`FRAME_BYTES`]: writes and transfers send their bytes,
    /// reads send zeros, and what comes back fills the reads and transfers. A delay is not one frame.
    fn transaction(&mut self, operations: &mut [Operation<'_, u8>]) -> Result<(), SpiError> {
        if !self.synced {
            self.discard();
        }
        let mut transmit = [0u8; FRAME_BYTES];
        let mut length = 0;
        for operation in operations.iter() {
            let bytes = match operation {
                Operation::Write(bytes) => bytes.len(),
                Operation::Read(buffer) => buffer.len(),
                Operation::Transfer(read, write) => read.len().max(write.len()),
                Operation::TransferInPlace(buffer) => buffer.len(),
                Operation::DelayNs(_) => return Err(SpiError::Unsupported),
            };
            if length + bytes > FRAME_BYTES {
                return Err(SpiError::TooLong);
            }
            match operation {
                Operation::Write(bytes) => transmit[length..length + bytes.len()].copy_from_slice(bytes),
                Operation::Transfer(_, write) => transmit[length..length + write.len()].copy_from_slice(write),
                Operation::TransferInPlace(buffer) => transmit[length..length + buffer.len()].copy_from_slice(buffer),
                Operation::Read(_) | Operation::DelayNs(_) => {}
            }
            length += bytes;
        }
        if length == 0 {
            return Ok(());
        }
        let mut receive = [0u8; FRAME_BYTES];
        self.frame(&transmit[..length], &mut receive[..length])?;
        let mut offset = 0;
        for operation in operations.iter_mut() {
            match operation {
                Operation::Write(bytes) => offset += bytes.len(),
                Operation::Read(buffer) => {
                    let taken = buffer.len();
                    buffer.copy_from_slice(&receive[offset..offset + taken]);
                    offset += taken;
                }
                Operation::Transfer(read, write) => {
                    let taken = read.len().max(write.len());
                    read.copy_from_slice(&receive[offset..offset + read.len()]);
                    offset += taken;
                }
                Operation::TransferInPlace(buffer) => {
                    let taken = buffer.len();
                    buffer.copy_from_slice(&receive[offset..offset + taken]);
                    offset += taken;
                }
                Operation::DelayNs(_) => {}
            }
        }
        Ok(())
    }
}
