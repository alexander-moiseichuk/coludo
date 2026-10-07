//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The buses the parts hang off (doc/specs/avionics-port.md M5): the I2C ports, each created by its owner thread on
//! core 1 (crate::owner in the binary).

pub mod i2c;
