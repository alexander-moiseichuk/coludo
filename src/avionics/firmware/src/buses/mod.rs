//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The buses the parts hang off (doc/specs/avionics-port.md M5): the I2C ports, each created by its owner thread on
//! core 1 (crate::owner in the binary), and SPI2, the IMU bus (M6's control thread owns it).

pub mod i2c;
pub mod spi;
