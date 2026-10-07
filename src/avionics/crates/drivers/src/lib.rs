//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The device logic of the src/glider drivers (src/glider/drivers/*.py), one module per plan B file and named
//! after it: the register init tables as data, the conversions, the CRCs, the calibration and tare math, and the
//! re-arm a general call makes necessary. Everything talks through embedded-hal 1.0 (`I2c`, `DelayNs`,
//! `OutputPin`), so the same code runs over the firmware's `I2cPort` and over the scripted [`mock`] bus the host
//! tests use.
//!
//! What is NOT here: the run loops, the databoard channels, the telemetry streams and the CC surface. Plan B's
//! Task owned all of them; in Rust they belong to the bus owner thread and its per-part glue (firmware
//! `sensors/<part>.rs`, doc/specs/avionics-port.md §1.2). A driver here is a passive object the owner calls
//! between its own transactions, and nothing in it reads a clock: time arrives as an argument or as a
//! `DelayNs` wait.
//!
//! Shared by every driver:
//!  * [`device`] -- the [`device::Device`] trait every bus part implements (setup and the general-call
//!    [`device::Device::rearm`]), the [`device::Fault`] a conversation ends in, and [`device::Strikes`], the
//!    consecutive-failure counter (src/glider task.py);
//!  * [`commons`] -- the Sensirion CRC-8 and the wire-level id diagnosis (src/glider commons.py);
//!  * [`mock`] -- the scripted bus, delay and pin the host tests drive the drivers with;
//!  * [`axes`] -- the optional per-sensor `axes` key: a part mounted other than per its marks (new, avionics-port.md
//!    §7.2 Q7);
//!  * [`debounce`] -- a two-level input read once per tick on a µs clock, with the edges its interrupt stamped or
//!    polled, debounced for the latch it stands behind: the separation switch's pads, and any other contact (a
//!    remove-before-flight pin, a breakwire).
//!
//! The parts, each named after its plan B module:
//!  * [`sdp810`] -- the pitot: dynamic pressure and airspeed;
//!  * [`ina226`] -- the power monitor: bus voltage, current and power;
//!  * [`vl53l1x`] and [`vl53l4cx`] -- the two laser rangers that share one socket at 0x29;
//!  * [`separation`] -- the stage-separation switch: its latch over [`debounce`], the separation moment and the
//!    pads' wobble;
//!  * [`lsm6dso32`] -- the primary IMU on SPI: accel and gyro `rate`, its write-then-verify bring-up;
//!  * [`bmi323`] -- the SEN0697's IMU on I2C: the same `accel` and `rate`, behind the LSM6DSO32;
//!  * [`icp10111`] -- the primary barometer: the TDK polynomial, the two-phase measurement, the general call;
//!  * [`bmp581`] -- the SEN0697's backup barometer;
//!  * [`bmm350`] -- the SEN0697's magnetometer and its hard/soft-iron calibration;
//!  * [`sg90`] and [`mg90s`] -- the fin servos' degree-to-pulse map.

pub mod axes;
pub mod bmi323;
pub mod bmm350;
pub mod bmp581;
pub mod commons;
pub mod debounce;
pub mod device;
pub mod icp10111;
pub mod ina226;
pub mod lsm6dso32;
pub mod mg90s;
pub mod mock;
pub mod sdp810;
pub mod separation;
pub mod sg90;
pub mod vl53l1x;
pub mod vl53l4cx;
