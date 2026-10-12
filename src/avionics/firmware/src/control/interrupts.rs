//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The GPIO interrupts the control thread serves: the service it installs on core 1, the second IMU's INT1 and the
//! INA226's ALERT, counted.

use super::*;

/// The second IMU's INT1 edges since its owner last took them ([`take_second_edges`]): the BMI323's data-ready line,
/// evidence only, its reads paced by the owner's tick (M9).
pub(super) static SECOND_EDGES: AtomicU32 = AtomicU32::new(0);
/// The second IMU's INT1 data-ready interrupt: count the edge. Integer only.
pub(super) unsafe extern "C" fn second_edge(_: *mut core::ffi::c_void) {
    SECOND_EDGES.fetch_add(1, Ordering::Relaxed);
}

/// Arms the second IMU's INT1 (the BMI323's, M9) on GPIO `gpio`, counting into [`take_second_edges`]. Only once the
/// control thread runs ([`running`]): it installed the GPIO interrupt service on core 1 before it ran.
///
/// Errors: the pin would not configure, or its handler would not attach.
pub fn arm_second_interrupt(gpio: u8) -> Result<(), sys::EspError> {
    arm_interrupt(gpio, second_edge)
}

/// The INA226's ALERT trips since boot (plan B ina226._on_alert): its open-drain line fell, the shunt current past
/// `alert_ma`.
pub(super) static ALERT_EDGES: AtomicU32 = AtomicU32::new(0);

/// The INA226's ALERT interrupt: count the trip. Integer only.
pub(super) unsafe extern "C" fn alert_edge(_: *mut core::ffi::c_void) {
    ALERT_EDGES.fetch_add(1, Ordering::Relaxed);
}

/// Arms the INA226's ALERT on GPIO `gpio`: falling edges, the internal pull-up holding the open-drain, active-low line
/// high (plan B `Pin(gpio, Pin.IN, Pin.PULL_UP)`, IRQ_FALLING), counting into [`alert_edges`]. Only once the control
/// thread runs ([`running`]): it installed the GPIO interrupt service on core 1 before it ran.
///
/// Errors: the pin would not configure, or its handler would not attach.
pub fn arm_alert_interrupt(gpio: u8) -> Result<(), sys::EspError> {
    let config = sys::gpio_config_t {
        pin_bit_mask: 1u64 << gpio,
        mode: sys::gpio_mode_t_GPIO_MODE_INPUT,
        pull_up_en: sys::gpio_pullup_t_GPIO_PULLUP_ENABLE,
        pull_down_en: sys::gpio_pulldown_t_GPIO_PULLDOWN_DISABLE,
        intr_type: sys::gpio_int_type_t_GPIO_INTR_NEGEDGE,
        ..Default::default()
    };
    // SAFETY: plain GPIO configuration of the pin the board config gives the ALERT; the handler is a static function.
    unsafe {
        sys::esp!(sys::gpio_config(&config))?;
        sys::esp!(sys::gpio_isr_handler_add(
            i32::from(gpio),
            Some(alert_edge),
            core::ptr::null_mut()
        ))?;
    }
    Ok(())
}

/// The INA226's ALERT trips since boot.
pub fn alert_edges() -> u32 {
    ALERT_EDGES.load(Ordering::Relaxed)
}

/// The second IMU's INT1 edges since the last take.
pub fn take_second_edges() -> u32 {
    SECOND_EDGES.swap(0, Ordering::Relaxed)
}

/// Arms a rising-edge interrupt on GPIO `gpio` that counts with `handler`, through the service
/// [`install_gpio_service`] installed. The internal pull-down holds a broken wire at 0,
/// where plan B's `Pin(gpio, Pin.IN)` left it floating.
pub(super) fn arm_interrupt(
    gpio: u8,
    handler: unsafe extern "C" fn(*mut core::ffi::c_void),
) -> Result<(), sys::EspError> {
    let config = sys::gpio_config_t {
        pin_bit_mask: 1u64 << gpio,
        mode: sys::gpio_mode_t_GPIO_MODE_INPUT,
        pull_up_en: sys::gpio_pullup_t_GPIO_PULLUP_DISABLE,
        pull_down_en: sys::gpio_pulldown_t_GPIO_PULLDOWN_ENABLE,
        intr_type: sys::gpio_int_type_t_GPIO_INTR_POSEDGE,
        ..Default::default()
    };
    // SAFETY: plain GPIO configuration of the pin the board config gives INT1; the handler is a static function.
    unsafe {
        sys::esp!(sys::gpio_config(&config))?;
        sys::esp!(sys::gpio_isr_handler_add(
            i32::from(gpio),
            Some(handler),
            core::ptr::null_mut()
        ))?;
    }
    Ok(())
}

/// Installs the GPIO interrupt service, once, before any GPIO interrupt is armed: on this thread, so every GPIO
/// interrupt it dispatches runs on core 1.
pub(super) fn install_gpio_service() -> Result<(), sys::EspError> {
    // SAFETY: no GPIO interrupt is armed yet; ESP_INTR_FLAG 0, a level-1 interrupt allocated on this core.
    match unsafe { sys::gpio_install_isr_service(0) } {
        // installed already, by a component before this thread: its interrupts land where it installed them
        sys::ESP_OK | sys::ESP_ERR_INVALID_STATE => Ok(()),
        code => sys::esp!(code),
    }
}
