//! Who this board is and what it runs on: the id, this boot's count, the persistent store, the vitals.

use std::sync::Mutex;

use esp_idf_svc::nvs::{EspDefaultNvsPartition, EspNvs, NvsDefault};
use esp_idf_svc::sys;
use firmware::thermal::Thermometer;

/// The id the hub registers this board under, until board config lands (src/glider: `board.id`).
const BOARD_ID: &str = "taster";
/// The firmware's name on the wire: the hub shows it as the board's version.
pub const FIRMWARE_VERSION: &str = concat!("avionics-", env!("CARGO_PKG_VERSION"));
/// The NVS namespace src/glider uses too, so the boot count and the calibrations carry across a reflash.
const NAMESPACE: &str = "coludo";

/// This board: its id, this boot's id, and the NVS namespace behind both.
pub struct Board {
    /// The id the hub registers.
    pub id: &'static str,
    /// This boot's NVS count (src/glider main._next_boot); None when the store would not count it.
    pub boot_id: Option<i32>,
    /// The `coludo` NVS namespace (boot count, Wi-Fi credentials).
    pub store: Mutex<EspNvs<NvsDefault>>,
    /// The die temperature sensor; None when it would not start (health then reports no temperature). Behind
    /// a lock: IDF does not promise its read is thread-safe, and the console and the link both ask.
    pub thermometer: Mutex<Option<Thermometer>>,
    /// Whether the microSD mounted at boot (firmware::sd).
    pub sd_mounted: bool,
}

impl Board {
    /// Open the store, count this boot, keep the thermometer.
    pub fn open(
        partition: EspDefaultNvsPartition,
        thermometer: Option<Thermometer>,
        sd_mounted: bool,
    ) -> anyhow::Result<Self> {
        let store = EspNvs::new(partition, NAMESPACE, true)?;
        let boot_id = count_boot(&store);
        Ok(Self {
            id: BOARD_ID,
            boot_id,
            store: Mutex::new(store),
            thermometer: Mutex::new(thermometer),
            sd_mounted,
        })
    }

    /// The die temperature in °C, rounded to 0.1 (what health reports).
    pub fn celsius(&self) -> Option<f32> {
        let celsius = self.thermometer.lock().unwrap().as_ref()?.celsius()?;
        Some((celsius * 10.0).round() / 10.0)
    }

    /// The prefix this boot's capture files carry: the boot id as `%06u`, None when the boot was not counted.
    pub fn session(&self) -> Option<String> {
        self.boot_id.map(|boot| format!("{boot:06}"))
    }

    /// One line naming the firmware, the silicon and this boot -- the first thing the console prints.
    pub fn banner(&self) -> String {
        let mut chip = sys::esp_chip_info_t::default();
        // SAFETY: esp_chip_info fills the struct it is handed and keeps no reference to it.
        unsafe { sys::esp_chip_info(&mut chip) };
        format!(
            "{FIRMWARE_VERSION} board {} boot {} reset {} chip rev v{}.{} cores {} cpu {} MHz psram {} KB heap {} KB die {:?} C",
            self.id,
            self.session().unwrap_or_else(|| "uncounted".into()),
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

    /// The Wi-Fi network to join, as the console stored it: (ssid, password).
    pub fn credentials(&self) -> Option<(String, String)> {
        let store = self.store.lock().unwrap();
        let mut ssid = [0u8; 33];
        let mut password = [0u8; 65];
        let ssid = store.get_str(SSID_KEY, &mut ssid).ok()??.to_string();
        let password = store.get_str(PASSWORD_KEY, &mut password).ok()??.to_string();
        Some((ssid, password))
    }

    /// Store the Wi-Fi network to join; the link picks it up on its next dial.
    pub fn set_credentials(&self, ssid: &str, password: &str) -> anyhow::Result<()> {
        let store = self.store.lock().unwrap();
        store.set_str(SSID_KEY, ssid)?;
        store.set_str(PASSWORD_KEY, password)?;
        Ok(())
    }
}

/// NVS key of the Wi-Fi network name.
const SSID_KEY: &str = "wifi_ssid";
/// NVS key of the Wi-Fi password. Never in the image, never in git: the console sets it.
const PASSWORD_KEY: &str = "wifi_password";

/// Count this boot: NVS `coludo`/`boot` + 1, committed before anything records (src/glider main._next_boot).
///
/// Only a key that was never written starts the count at 1. Any other read error says nothing about how far
/// the count got, and restarting it would reuse ids -- so it, and a count the store will not take, give None.
fn count_boot(store: &EspNvs<NvsDefault>) -> Option<i32> {
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

/// Why the chip last reset, as IDF names it.
fn reset_reason() -> &'static str {
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
        _ => "unknown",
    }
}
