//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The I2C ports, on IDF's new `i2c_master` driver (doc/specs/avionics-port.md M5, design decisions). Each port is
//! created exactly once, by its owner thread on core 1 (crate::owner in the binary), so its interrupt runs on core 1;
//! a second `i2c_new_master_bus` on a live port would tear its pins down (i2c_master.c:1058, 1170-1173). Port 0
//! (SDA 7 / SCL 8) also carries the camera's SCCB, whose own 400 kHz device handle esp_video adds to the bus
//! published here ([`port0`]). The legacy driver (esp-idf-hal's I2cDriver) must never touch either port.
//!
//! [`I2cPort`] is plan B's i2cbus.Bus on that driver: an `embedded_hal::i2c::I2c` for the drivers crate, one device
//! handle per address at the port's speed, and plan B's wedge rule -- 4 consecutive failures across the port, cured,
//! counting on from zero (i2cbus.py:22, 138-184). What counts is a wire failure: `ESP_ERR_INVALID_STATE` (a NAK, an
//! SCL timeout, lost arbitration; the driver cannot tell them apart). A bus-lock timeout (`ESP_ERR_TIMEOUT`: the
//! camera can hold the lock for its 500 ms SCCB timeout) is not the bus's fault, nor is a caller error, a probe or a
//! scan. The cure is `i2c_master_bus_reset` (an FSM reset and 9 SCL pulses); the handles survive it, and are made
//! again only when the live speed differs from the configured one (after a bustune), as plan B's `reclock()`.

use std::sync::atomic::{AtomicUsize, Ordering};

use embedded_hal::i2c::{ErrorKind, ErrorType, NoAcknowledgeSource, Operation};
use esp_idf_svc::sys;

/// The transfer timeout, ms: plan B's MicroPython passes 50 (machine_i2c.c:36).
pub const TRANSFER_TIMEOUT_MS: i32 = 50;

/// Consecutive failures across a port that make a wedge (plan B i2cbus.py:22 `_BUS_FAIL_LIMIT`).
pub const WEDGE_FAILURES: u32 = 4;

/// The general-call address and its software-reset command (I2C spec 3.1.14).
pub const GENERAL_CALL: (u8, u8) = (0x00, 0x06);

/// The addresses a scan probes: MicroPython's `I2C.scan` range (the reserved ones at both ends left out).
const SCAN_ADDRESSES: core::ops::RangeInclusive<u8> = 0x08..=0x77;

/// How many device handles a port keeps at most: its devices, the general-call one and a few touched addresses.
const MOST_HANDLES: usize = 16;

/// How often a handle's removal is tried: it can fail while another handle's transfer is on the wire
/// (i2c_master.c:1213-1217, a status check outside the bus lock).
const REMOVE_ATTEMPTS: u32 = 3;

/// Port 0's bus handle once its owner created it, as a plain number so it can live in a static (0 = not yet).
static PORT0: AtomicUsize = AtomicUsize::new(0);

/// Port 0's master bus, for the camera's SCCB (crate::video): created by port 0's owner at boot, never here.
///
/// Errors: the owner has not created it (yet).
pub fn port0() -> anyhow::Result<sys::i2c_master_bus_handle_t> {
    match PORT0.load(Ordering::Acquire) {
        0 => anyhow::bail!("i2c port 0 is not created: its owner thread has not run"),
        handle => Ok(handle as sys::i2c_master_bus_handle_t),
    }
}

/// Why an I2C transaction failed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum I2cError {
    /// A wire failure: no acknowledge, an SCL timeout or lost arbitration (`ESP_ERR_INVALID_STATE`). Counted.
    NotAcknowledged,
    /// The bus lock was not had within the timeout (`ESP_ERR_TIMEOUT`): another handle (the camera) held the bus.
    BusBusy,
    /// Any other driver answer: a caller error, no memory, an unsupported transaction shape.
    Driver(sys::esp_err_t),
}

impl core::fmt::Display for I2cError {
    fn fmt(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            I2cError::NotAcknowledged => formatter.write_str("not acknowledged (no reply, or the bus failed)"),
            I2cError::BusBusy => formatter.write_str("bus busy (another device held the bus)"),
            I2cError::Driver(code) => write!(formatter, "driver error {code:#x}"),
        }
    }
}

impl embedded_hal::i2c::Error for I2cError {
    fn kind(&self) -> ErrorKind {
        match self {
            I2cError::NotAcknowledged => ErrorKind::NoAcknowledge(NoAcknowledgeSource::Unknown),
            I2cError::BusBusy | I2cError::Driver(_) => ErrorKind::Other,
        }
    }
}

/// A port's counters, for `i2c touch`, `bustune` and `inspect`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct PortFigures {
    /// The current run of consecutive failures (0..WEDGE_FAILURES).
    pub failures: u32,
    /// Wedges cured since boot.
    pub wedges: u32,
    /// General calls sent on this port since boot (any caller's).
    pub general_calls: u32,
    /// The speed the handles run at, Hz.
    pub speed_hz: u32,
    /// The speed the running config asks for, Hz.
    pub configured_hz: u32,
}

/// One I2C port: its bus, its device handles and plan B's wedge counter.
pub struct I2cPort {
    /// The port number (0 or 1).
    id: u8,
    /// The bus.
    bus: sys::i2c_master_bus_handle_t,
    /// One device handle per address, all at `speed_hz`.
    handles: Vec<(u8, sys::i2c_master_dev_handle_t)>,
    /// The speed the running config asks for, Hz.
    configured_hz: u32,
    /// The speed the handles run at, Hz (differs from `configured_hz` while a bustune tries another).
    speed_hz: u32,
    /// The current run of consecutive failures.
    failures: u32,
    /// Wedges cured since boot.
    wedges: u32,
    /// General calls that reached the wire since boot (answered or NAKed; one the port refused never counts).
    general_calls: u32,
    /// A general call reached the wire since [`I2cPort::take_general_call`] last asked.
    general_called: bool,
}

// SAFETY: the handles are IDF's, usable from any task; one task uses a port at a time (its owner, or main on loan).
unsafe impl Send for I2cPort {}

impl I2cPort {
    /// Creates port `id`'s bus on pins `sda`/`scl` at `clock_hz`, with plan B's MicroPython settings: the XTAL clock,
    /// a glitch filter of 7, internal pull-ups on (beside the board's), the synchronous driver (async refuses
    /// probes). Call it on the port's owner thread (core 1), once: the bus's interrupt stays on the creating core.
    /// Port 0's handle is published for the camera ([`port0`]).
    ///
    /// Errors: the driver refused the bus (the port taken, bad pins, no memory).
    pub fn create(id: u8, sda: u8, scl: u8, clock_hz: u32) -> anyhow::Result<I2cPort> {
        let mut config = sys::i2c_master_bus_config_t {
            i2c_port: id as sys::i2c_port_num_t,
            sda_io_num: sda as sys::gpio_num_t,
            scl_io_num: scl as sys::gpio_num_t,
            glitch_ignore_cnt: 7,
            intr_priority: 0,
            trans_queue_depth: 0,
            ..Default::default()
        };
        config.__bindgen_anon_1.clk_source = sys::soc_periph_i2c_clk_src_t_I2C_CLK_SRC_DEFAULT;
        config.flags.set_enable_internal_pullup(1);
        let mut bus: sys::i2c_master_bus_handle_t = core::ptr::null_mut();
        // SAFETY: a filled config and a live output; the bus lives for the program.
        sys::esp!(unsafe { sys::i2c_new_master_bus(&config, &mut bus) })?;
        if id == 0 {
            PORT0.store(bus as usize, Ordering::Release);
        }
        Ok(I2cPort {
            id,
            bus,
            handles: Vec::with_capacity(MOST_HANDLES),
            configured_hz: clock_hz,
            speed_hz: clock_hz,
            failures: 0,
            wedges: 0,
            general_calls: 0,
            general_called: false,
        })
    }

    /// The port number.
    pub fn id(&self) -> u8 {
        self.id
    }

    /// Sets the speed the running config asks for (the post-layout `freq`), and runs at it.
    ///
    /// Errors: a handle could not be made again at that speed.
    pub fn configure(&mut self, clock_hz: u32) -> Result<(), I2cError> {
        self.configured_hz = clock_hz;
        self.retune(clock_hz)
    }

    /// Runs every handle at `clock_hz` (a bustune rung; not saved: [`I2cPort::configure`] or a wedge cure restores
    /// the configured speed). Each handle is added again at the new speed, then the old one removed: IDF has no
    /// speed change in place.
    ///
    /// Errors: a handle could not be added (the old ones stay).
    pub fn retune(&mut self, clock_hz: u32) -> Result<(), I2cError> {
        if clock_hz == self.speed_hz {
            return Ok(());
        }
        let mut fresh = Vec::with_capacity(MOST_HANDLES);
        for &(address, _) in &self.handles {
            match self.add(address, clock_hz) {
                Ok(handle) => fresh.push((address, handle)),
                Err(error) => {
                    for (_, handle) in fresh {
                        self.remove(handle);
                    }
                    return Err(error);
                }
            }
        }
        for (_, handle) in core::mem::replace(&mut self.handles, fresh) {
            self.remove(handle);
        }
        self.speed_hz = clock_hz;
        Ok(())
    }

    /// Whether `address` acknowledges (a START, the address, a STOP, at IDF's fixed 100 kHz). Never counted toward a
    /// wedge, as plan B's scan was not (layout.py:191-213).
    ///
    /// Errors: the bus was busy or broken (not an answer either way).
    pub fn probe(&mut self, address: u8) -> Result<bool, I2cError> {
        // SAFETY: the bus lives for the program; the call takes the bus lock itself.
        match unsafe { sys::i2c_master_probe(self.bus, address as u16, TRANSFER_TIMEOUT_MS) } {
            sys::ESP_OK => Ok(true),
            sys::ESP_ERR_NOT_FOUND => Ok(false),
            sys::ESP_ERR_TIMEOUT => Err(I2cError::BusBusy),
            code => Err(I2cError::Driver(code)),
        }
    }

    /// What acknowledges on the port, probed as MicroPython's `scan` probes ([`SCAN_ADDRESSES`]), for the layout
    /// (plan B layout.py `_scan`). An address whose probe fails reads as absent, as MicroPython's scan reads it, so
    /// one glitch or long clock stretch costs one address, not the bus's votes. Never counted toward a wedge.
    ///
    /// Errors: [`WEDGE_FAILURES`] probes failed in a row (a stuck or shorted bus, each failure a timeout): the layout
    /// reads that as no votes from this bus, as plan B's scan of such a bus found nothing, only sooner.
    pub fn scan(&mut self) -> Result<layout::Addresses, I2cError> {
        let mut found = layout::Addresses::EMPTY;
        let mut failures = 0;
        for address in SCAN_ADDRESSES {
            match self.probe(address) {
                Ok(true) => found.insert(address),
                Ok(false) => {}
                Err(error) => {
                    failures += 1;
                    if failures >= WEDGE_FAILURES {
                        return Err(error);
                    }
                    continue;
                }
            }
            failures = 0;
        }
        Ok(found)
    }

    /// Sends the general call's software reset (0x00, 0x06): every part that honours it resets. Counted toward a
    /// wedge, its NAK included (nothing on the bus honours it), as plan B's `Bus.writeto` counted it
    /// (i2cbus.py:228-243). The caller re-arms the port's members (crate::owner).
    ///
    /// Errors: the call was not acknowledged, or the bus was busy.
    pub fn general_call_reset(&mut self) -> Result<(), I2cError> {
        embedded_hal::i2c::I2c::write(self, GENERAL_CALL.0, &[GENERAL_CALL.1])
    }

    /// Whether a general call went out on this port since the last ask (from any caller: a driver's own call too).
    pub fn take_general_call(&mut self) -> bool {
        core::mem::take(&mut self.general_called)
    }

    /// Starts the failure run from zero (plan B test_i2cbus.py:55), so `i2c touch` counts toward a wedge from a known
    /// place.
    pub fn restart_run(&mut self) {
        self.failures = 0;
    }

    /// The port's counters.
    pub fn figures(&self) -> PortFigures {
        PortFigures {
            failures: self.failures,
            wedges: self.wedges,
            general_calls: self.general_calls,
            speed_hz: self.speed_hz,
            configured_hz: self.configured_hz,
        }
    }

    /// A device handle for `address` at `clock_hz`.
    fn add(&self, address: u8, clock_hz: u32) -> Result<sys::i2c_master_dev_handle_t, I2cError> {
        let config = sys::i2c_device_config_t {
            dev_addr_length: sys::i2c_addr_bit_len_t_I2C_ADDR_BIT_LEN_7,
            device_address: address as u16,
            scl_speed_hz: clock_hz,
            scl_wait_us: 0,
            ..Default::default()
        };
        let mut handle: sys::i2c_master_dev_handle_t = core::ptr::null_mut();
        // SAFETY: a filled config and a live output on the program's bus.
        match unsafe { sys::i2c_master_bus_add_device(self.bus, &config, &mut handle) } {
            sys::ESP_OK => Ok(handle),
            code => Err(I2cError::Driver(code)),
        }
    }

    /// Removes a device handle, trying again while another transfer is on the wire; one that will not go stays (it
    /// holds no hardware state, only a little memory).
    fn remove(&self, handle: sys::i2c_master_dev_handle_t) {
        for _ in 0..REMOVE_ATTEMPTS {
            // SAFETY: a handle this port added and no longer uses.
            if unsafe { sys::i2c_master_bus_rm_device(handle) } == sys::ESP_OK {
                return;
            }
            std::thread::sleep(std::time::Duration::from_millis(1));
        }
        log::warn!("i2c:{} :: a device handle would not be removed (kept)", self.id);
    }

    /// The handle for `address`, made on its first use at the port's speed.
    fn handle(&mut self, address: u8) -> Result<sys::i2c_master_dev_handle_t, I2cError> {
        if let Some(&(_, handle)) = self.handles.iter().find(|(known, _)| *known == address) {
            return Ok(handle);
        }
        if self.handles.len() >= MOST_HANDLES {
            return Err(I2cError::Driver(sys::ESP_ERR_NO_MEM));
        }
        let handle = self.add(address, self.speed_hz)?;
        self.handles.push((address, handle));
        Ok(handle)
    }

    /// Counts a transaction's outcome toward plan B's wedge rule, and cures a wedge at the 4th failure in a row.
    fn count(&mut self, outcome: &Result<(), I2cError>) {
        match outcome {
            Ok(()) => self.failures = 0,
            Err(I2cError::NotAcknowledged) => {
                self.failures += 1;
                if self.failures >= WEDGE_FAILURES {
                    self.cure();
                }
            }
            Err(I2cError::BusBusy | I2cError::Driver(_)) => {}
        }
    }

    /// The wedge cure: the bus reset (FSM and 9 SCL pulses), the configured speed again if a bustune left another,
    /// plan B's log line (i2cbus.py:182-183), and a fresh count.
    fn cure(&mut self) {
        // SAFETY: the bus lives for the program. The reset takes no bus lock: the camera's SCCB is quiet while it
        // streams, and its register traffic is pre-flight only (avionics-port.md §4.2).
        if let Err(error) = sys::esp!(unsafe { sys::i2c_master_bus_reset(self.bus) }) {
            log::warn!("i2c:{} :: bus reset failed: {error}", self.id);
        }
        if self.speed_hz != self.configured_hz
            && let Err(error) = self.retune(self.configured_hz)
        {
            log::warn!("i2c:{} :: back to the configured speed failed: {error}", self.id);
        }
        self.wedges += 1;
        self.failures = 0;
        log::warn!(
            "i2c:{} :: bus wedged after {WEDGE_FAILURES} failures -- clocked SDA free and re-inited",
            self.id
        );
    }
}

/// An IDF transfer's answer as an [`I2cError`].
fn outcome(code: sys::esp_err_t) -> Result<(), I2cError> {
    match code {
        sys::ESP_OK => Ok(()),
        sys::ESP_ERR_INVALID_STATE => Err(I2cError::NotAcknowledged),
        sys::ESP_ERR_TIMEOUT => Err(I2cError::BusBusy),
        code => Err(I2cError::Driver(code)),
    }
}

impl ErrorType for I2cPort {
    type Error = I2cError;
}

impl embedded_hal::i2c::I2c for I2cPort {
    /// The drivers' shapes, straight onto the driver: [Write] transmit, [Read] receive, [Write, Read]
    /// transmit_receive, writes only (at most 6) one multi-buffer transmit. Any other shape, or an empty buffer,
    /// answers `ESP_ERR_NOT_SUPPORTED`, uncounted.
    fn transaction(&mut self, address: u8, operations: &mut [Operation<'_>]) -> Result<(), I2cError> {
        let unsupported = Err(I2cError::Driver(sys::ESP_ERR_NOT_SUPPORTED));
        let empty = operations.iter().any(|operation| match operation {
            Operation::Write(bytes) => bytes.is_empty(),
            Operation::Read(bytes) => bytes.is_empty(),
        });
        if operations.is_empty() || empty {
            return unsupported;
        }
        let general_call = address == GENERAL_CALL.0
            && matches!(&*operations, [Operation::Write(bytes)] if bytes[..] == [GENERAL_CALL.1]);
        let handle = self.handle(address)?;
        let code = match operations {
            [Operation::Write(bytes)] => {
                // SAFETY: a live handle and a buffer valid for the call.
                unsafe { sys::i2c_master_transmit(handle, bytes.as_ptr(), bytes.len(), TRANSFER_TIMEOUT_MS) }
            }
            [Operation::Read(bytes)] => {
                // SAFETY: as above.
                unsafe { sys::i2c_master_receive(handle, bytes.as_mut_ptr(), bytes.len(), TRANSFER_TIMEOUT_MS) }
            }
            [Operation::Write(written), Operation::Read(read)] => {
                // SAFETY: as above.
                unsafe {
                    sys::i2c_master_transmit_receive(
                        handle,
                        written.as_ptr(),
                        written.len(),
                        read.as_mut_ptr(),
                        read.len(),
                        TRANSFER_TIMEOUT_MS,
                    )
                }
            }
            writes if writes.len() <= 6 && writes.iter().all(|operation| matches!(operation, Operation::Write(_))) => {
                let mut buffers: Vec<sys::i2c_master_transmit_multi_buffer_info_t> = writes
                    .iter()
                    .map(|operation| match operation {
                        Operation::Write(bytes) => sys::i2c_master_transmit_multi_buffer_info_t {
                            write_buffer: bytes.as_ptr(),
                            buffer_size: bytes.len(),
                        },
                        Operation::Read(_) => unreachable!("only writes reach here"),
                    })
                    .collect();
                // SAFETY: a live handle and buffers valid for the call.
                unsafe {
                    sys::i2c_master_multi_buffer_transmit(
                        handle,
                        buffers.as_mut_ptr(),
                        buffers.len(),
                        TRANSFER_TIMEOUT_MS,
                    )
                }
            }
            _ => return unsupported,
        };
        let result = outcome(code);
        // a general call counts once it reached the wire, answered or not: one the port refused (bus busy, a driver
        // error) reset nothing, as the drivers' wire rule reads it
        let on_the_wire = result
            .as_ref()
            .err()
            .is_none_or(|error| embedded_hal::i2c::Error::kind(error) != ErrorKind::Other);
        if general_call && on_the_wire {
            self.general_calls += 1;
            self.general_called = true;
        }
        self.count(&result);
        result
    }
}
