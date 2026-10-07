//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! Who this board is and what it runs on: the id, this boot's count, the persistent store, the vitals.

use std::sync::Mutex;

use esp_idf_svc::nvs::{EspDefaultNvsPartition, EspNvs, NvsDefault};
use esp_idf_svc::sys;
use firmware::thermal::Thermometer;

/// The firmware's name on the wire: the hub shows it as the board's version.
pub const FIRMWARE_VERSION: &str = concat!("avionics-", env!("CARGO_PKG_VERSION"));
/// The NVS namespace src/glider uses too, so the boot count and the calibrations carry across a reflash.
const NAMESPACE: &str = "coludo";

/// This board: its id, this boot's id, its vitals.
pub struct Board {
    /// The id the hub registers: the config's `board.id`.
    pub id: String,
    /// This boot's NVS count (src/glider main._next_boot); None when the store would not count it.
    pub boot_id: Option<i32>,
    /// The prefix this boot's capture files carry (crate::recorder::session_prefix): a valid `recorder.session`
    /// label the card's layout does not reserve (`videos`, `recordings`, `video`), else the boot id as `%06u`, else
    /// the legacy date form.
    pub session: String,
    /// The die temperature sensor; None when it would not start (health then reports no temperature). Behind
    /// a lock: IDF does not promise its read is thread-safe, and the console and the link both ask.
    pub thermometer: Mutex<Option<Thermometer>>,
    /// Whether the microSD mounted at boot (firmware::sd).
    pub sd_mounted: bool,
}

impl Board {
    /// The board of this boot: `boot_id` from [`count_boot`], which ran first because the boot id names the
    /// recorder's files, and the `session` prefix those files carry.
    pub fn new(
        id: String,
        boot_id: Option<i32>,
        session: String,
        thermometer: Option<Thermometer>,
        sd_mounted: bool,
    ) -> Self {
        Self {
            id,
            boot_id,
            session,
            thermometer: Mutex::new(thermometer),
            sd_mounted,
        }
    }

    /// The die temperature in °C, rounded to 0.1 (what health reports).
    pub fn celsius(&self) -> Option<f32> {
        let celsius = self.thermometer.lock().unwrap().as_ref()?.celsius()?;
        Some((celsius * 10.0).round() / 10.0)
    }

    /// One line naming the firmware, the silicon and this boot -- the first thing the console prints.
    pub fn banner(&self) -> String {
        let mut chip = sys::esp_chip_info_t::default();
        // SAFETY: esp_chip_info fills the struct it is handed and keeps no reference to it.
        unsafe { sys::esp_chip_info(&mut chip) };
        format!(
            "{FIRMWARE_VERSION} board {} boot {} reset {} chip rev v{}.{} cores {} cpu {} MHz psram {} KB heap {} KB die {:?} C",
            self.id,
            self.session,
            reset_reason(),
            chip.revision / 100,
            chip.revision % 100,
            chip.cores,
            cpu_mhz(),
            heap_free(sys::MALLOC_CAP_SPIRAM) / 1024,
            heap_free(sys::MALLOC_CAP_INTERNAL) / 1024,
            self.celsius(),
        )
    }
}

/// Count this boot: NVS `coludo`/`boot` + 1 in the namespace plan B shares, committed before anything records
/// (src/glider main._next_boot), since the boot id names the recorder's files.
///
/// Only a key that was never written starts the count at 1. Any other read error says nothing about how far
/// the count got, and restarting it would reuse ids -- so it, and a count the store will not take, give None.
///
/// Errors: the namespace would not open (the `nvs` partition is missing or unusable).
pub fn count_boot(partition: EspDefaultNvsPartition) -> anyhow::Result<Option<i32>> {
    let store = EspNvs::new(partition, NAMESPACE, true)?;
    Ok(next_boot(&store))
}

/// The count after this boot's, stored; None when the store would not read or keep it.
fn next_boot(store: &EspNvs<NvsDefault>) -> Option<i32> {
    let boot = store.get_i32("boot").ok()?.unwrap_or(0) + 1;
    store.set_i32("boot", boot).ok()?; // esp-idf-svc commits on every set
    Some(boot)
}

/// Milliseconds since boot.
pub fn uptime_ms() -> i64 {
    // SAFETY: a plain read of the monotonic system timer.
    unsafe { sys::esp_timer_get_time() / 1000 }
}

/// Free heap bytes with the given capabilities (internal RAM, PSRAM).
pub fn heap_free(capabilities: u32) -> usize {
    // SAFETY: a plain query of the heap allocator.
    unsafe { sys::heap_caps_get_free_size(capabilities) }
}

/// The running CPU clock in MHz.
fn cpu_mhz() -> u32 {
    // SAFETY: a plain read of the ROM's clock bookkeeping.
    unsafe { sys::esp_rom_get_cpu_ticks_per_us() }
}

/// Why the chip last reset, as IDF names it: the banner, the provenance line and the recorder's session-start
/// record carry it.
pub fn reset_reason() -> &'static str {
    // SAFETY: a plain read of the reset cause latched at boot.
    match unsafe { sys::esp_reset_reason() } {
        sys::esp_reset_reason_t_ESP_RST_POWERON => "power-on",
        sys::esp_reset_reason_t_ESP_RST_SW => "software",
        sys::esp_reset_reason_t_ESP_RST_PANIC => "panic",
        sys::esp_reset_reason_t_ESP_RST_INT_WDT => "interrupt-watchdog",
        sys::esp_reset_reason_t_ESP_RST_TASK_WDT => "task-watchdog",
        sys::esp_reset_reason_t_ESP_RST_WDT => "watchdog",
        sys::esp_reset_reason_t_ESP_RST_BROWNOUT => "brownout",
        sys::esp_reset_reason_t_ESP_RST_EXT => "external",
        sys::esp_reset_reason_t_ESP_RST_DEEPSLEEP => "deep-sleep",
        sys::esp_reset_reason_t_ESP_RST_SDIO => "sdio",
        sys::esp_reset_reason_t_ESP_RST_USB => "usb",
        sys::esp_reset_reason_t_ESP_RST_JTAG => "jtag",
        sys::esp_reset_reason_t_ESP_RST_EFUSE => "efuse",
        sys::esp_reset_reason_t_ESP_RST_PWR_GLITCH => "power-glitch",
        sys::esp_reset_reason_t_ESP_RST_CPU_LOCKUP => "cpu-lockup",
        _ => "unknown",
    }
}
