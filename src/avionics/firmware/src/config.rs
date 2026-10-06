//! The board's settings: the built-in default config, and the `cfg` NVS partition (partitions.csv) that holds what
//! src/glider kept as files on its VFS -- board.config and launch.config in namespace `config`, the Wi-Fi passwords
//! (plan B's `<ssid>.creds`) in namespace `creds`. tools/provision.sh writes the whole partition before an airframe
//! is sealed; the CC commands and the console keep it current after that.
//!
//! Documents are JSON text in blobs: NVS strings stop at one 4 KB page, and a board config is ~10 KB.

use std::sync::OnceLock;

use esp_idf_svc::nvs::{EspCustomNvsPartition, EspNvs, NvsCustom};
use esp_idf_svc::sys;
use serde_json::Value;

/// The partition's label in partitions.csv.
const PARTITION: &str = "cfg";
/// The namespace of the documents.
const DOCUMENTS: &str = "config";
/// The namespace of the Wi-Fi passwords.
const CREDENTIALS: &str = "creds";
/// The key of board.config (src/glider config.load's path).
const BOARD_KEY: &str = "board";
/// The key of launch.config (src/glider mission.LAUNCH_PATH).
const LAUNCH_KEY: &str = "launch";

/// Plan B's built-in board config (src/glider config_default.default(), generated into
/// src/avionics/golden/config_default.json), stamped with this build's version: the fixture pins
/// board.firmware_version to 'dev', and plan B's load() restamps a saved config from the running firmware.
pub fn defaults() -> &'static Value {
    static DEFAULTS: OnceLock<Value> = OnceLock::new();
    DEFAULTS.get_or_init(|| {
        let fixture: Value = serde_json::from_str(include_str!("../../golden/config_default.json"))
            .expect("golden/config_default.json is generated, so it parses");
        let mut document = fixture["document"].clone();
        document["board"]["firmware_version"] = crate::board::FIRMWARE_VERSION.into();
        document
    })
}

/// Take the `cfg` partition once (esp-idf-svc refuses a second take, and dropping the last handle deinitialises
/// it). A partition another firmware left foreign is erased and formatted, so this fails only on a partition table
/// without `cfg`.
pub fn partition() -> anyhow::Result<EspCustomNvsPartition> {
    Ok(EspCustomNvsPartition::take(PARTITION)?)
}

/// The documents: board.config and launch.config.
pub struct Store {
    /// The `config` namespace.
    nvs: EspNvs<NvsCustom>,
}

impl Store {
    /// Open the documents' namespace.
    pub fn open(partition: EspCustomNvsPartition) -> anyhow::Result<Self> {
        Ok(Self {
            nvs: EspNvs::new(partition, DOCUMENTS, true)?,
        })
    }

    /// The saved board.config text; None when there is none (the board then runs the default).
    pub fn board(&self) -> Option<String> {
        self.text(BOARD_KEY)
    }

    /// Save board.config. NVS writes the new value before it drops the old one, so a cut keeps one of them.
    pub fn save_board(&self, text: &str) -> anyhow::Result<()> {
        self.nvs.set_blob(BOARD_KEY, text.as_bytes())?;
        Ok(())
    }

    /// Forget board.config, so the next boot runs the default; whether there was one.
    pub fn remove_board(&self) -> anyhow::Result<bool> {
        Ok(self.nvs.remove(BOARD_KEY)?)
    }

    /// The saved launch.config text; None when there is none (the mission then starts from its default).
    pub fn launch(&self) -> Option<String> {
        self.text(LAUNCH_KEY)
    }

    /// Save launch.config.
    pub fn save_launch(&self, text: &str) -> anyhow::Result<()> {
        self.nvs.set_blob(LAUNCH_KEY, text.as_bytes())?;
        Ok(())
    }

    /// The blob under `key` as UTF-8 text; None when absent, unreadable or not text.
    fn text(&self, key: &str) -> Option<String> {
        let length = self.nvs.blob_len(key).ok()??;
        let mut buffer = vec![0u8; length];
        let bytes = self.nvs.get_blob(key, &mut buffer).ok()??;
        String::from_utf8(bytes.to_vec()).ok()
    }
}

/// The Wi-Fi passwords, one per network (plan B: `<ssid>.creds` beside board.config). Never in the image, never in
/// git: tools/provision.sh or the console `wifi` command writes them. Cheap to clone: each call opens its own NVS
/// handle, so the link and the console share it without a lock (NVS serialises internally).
#[derive(Clone)]
pub struct Credentials {
    /// The `cfg` partition.
    partition: EspCustomNvsPartition,
}

impl Credentials {
    /// The passwords on this partition.
    pub fn new(partition: EspCustomNvsPartition) -> Self {
        Self { partition }
    }

    /// The stored password for `ssid`; None when none was stored.
    pub fn password(&self, ssid: &str) -> Option<String> {
        let nvs = EspNvs::new(self.partition.clone(), CREDENTIALS, false).ok()?;
        let mut buffer = [0u8; 65]; // a WPA2 passphrase is at most 63 characters
        nvs.get_str(&key(ssid), &mut buffer).ok()?.map(str::to_string)
    }

    /// Store the password for `ssid`; the link uses it on its next attempt at that network.
    pub fn set_password(&self, ssid: &str, password: &str) -> anyhow::Result<()> {
        let nvs = EspNvs::new(self.partition.clone(), CREDENTIALS, true)?;
        nvs.set_str(&key(ssid), password)?;
        Ok(())
    }
}

/// The NVS key of a network's password: `w` and the zlib CRC-32 of the SSID in 8 hex digits. NVS keys stop at 15
/// characters and an SSID may run to 32; tools/provision.sh computes the same key.
fn key(ssid: &str) -> String {
    // SAFETY: the ROM CRC reads exactly `len` bytes of the slice. Seeded with 0 it is zlib's crc32.
    let checksum = unsafe { sys::esp_rom_crc32_le(0, ssid.as_ptr(), ssid.len() as u32) };
    format!("w{checksum:08x}")
}
