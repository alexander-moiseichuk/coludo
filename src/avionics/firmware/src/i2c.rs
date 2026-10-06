//! The I2C buses, on IDF's new `i2c_master` driver. Port 0 (SDA GPIO7 / SCL GPIO8, 2.2 k pull-ups on the board)
//! carries the camera's SCCB and the flight sensors: one bus, owned here, handed to everyone. The legacy driver
//! (esp-idf-hal's I2cDriver) must never touch a port this module owns.

use std::sync::OnceLock;

use esp_idf_svc::sys;

/// Port 0's bus handle, as a plain number so it can live in a static.
static PORT0: OnceLock<usize> = OnceLock::new();

/// Port 0's master bus, created on first use.
pub fn port0() -> anyhow::Result<sys::i2c_master_bus_handle_t> {
    if let Some(handle) = PORT0.get() {
        return Ok(*handle as sys::i2c_master_bus_handle_t);
    }
    let mut config = sys::i2c_master_bus_config_t {
        i2c_port: 0,
        sda_io_num: 7,
        scl_io_num: 8,
        glitch_ignore_cnt: 7,
        ..Default::default()
    };
    config.__bindgen_anon_1.clk_source = sys::soc_periph_i2c_clk_src_t_I2C_CLK_SRC_DEFAULT;
    let mut handle: sys::i2c_master_bus_handle_t = core::ptr::null_mut();
    // SAFETY: a filled config and a live output; the bus lives for the program.
    sys::esp!(unsafe { sys::i2c_new_master_bus(&config, &mut handle) })?;
    let _ = PORT0.set(handle as usize);
    Ok(handle)
}
