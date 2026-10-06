//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The board's settings: the built-in default config, and the `cfg` NVS partition (partitions.csv) that holds what
//! src/glider kept as files on its VFS -- board.config and launch.config in namespace `config`, the Wi-Fi passwords
//! (plan B's `<ssid>.creds`) in namespace `creds`. tools/provision.sh writes the whole partition before an airframe
//! is sealed; the CC commands and the console keep it current after that.
//!
//! Documents are JSON text in blobs: NVS strings stop at one 4 KB page, and a board config is ~10 KB.

use std::sync::OnceLock;

use esp_idf_svc::handle::RawHandle;
use esp_idf_svc::nvs::{EspCustomNvsPartition, EspNvs, NvsCustom};
use esp_idf_svc::sys;
use firmware::gcl::{Command, Gate, Kind, Parameter, Reach};
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
/// The longest SSID 802.11 allows, bytes.
const SSID_LIMIT: usize = 32;
/// The longest password the radio takes, bytes: a 64-hex-digit raw key (a passphrase stops at 63).
const PASSWORD_LIMIT: usize = 64;

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

    /// Save board.config, replacing the old one only once the new one is written (plan B: atomic_write_json).
    pub fn save_board(&self, text: &str) -> anyhow::Result<()> {
        replace_blob(&self.nvs, BOARD_KEY, text.as_bytes())
    }

    /// Forget board.config, so the next boot runs the default; whether there was one.
    pub fn remove_board(&self) -> anyhow::Result<bool> {
        Ok(self.nvs.remove(BOARD_KEY)?)
    }

    /// The saved launch.config text; None when there is none (the mission then starts from its default).
    pub fn launch(&self) -> Option<String> {
        self.text(LAUNCH_KEY)
    }

    /// Save launch.config, replacing the old one only once the new one is written.
    pub fn save_launch(&self, text: &str) -> anyhow::Result<()> {
        replace_blob(&self.nvs, LAUNCH_KEY, text.as_bytes())
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

    /// The stored password for `ssid`; None when none (or an empty one) was stored -- plan B falls back to the
    /// config's password on an empty `<ssid>.creds` line too.
    pub fn password(&self, ssid: &str) -> Option<String> {
        let nvs = EspNvs::new(self.partition.clone(), CREDENTIALS, false).ok()?;
        let mut buffer = [0u8; PASSWORD_LIMIT + 1];
        let password = nvs.get_str(&key(ssid), &mut buffer).ok()??;
        (!password.is_empty()).then(|| password.to_string())
    }

    /// Store the password for `ssid`; the link uses it on its next attempt at that network. Refuses what the
    /// radio could never use: an SSID outside 1..=32 bytes, a password over 64 (tools/provision.sh's limits).
    pub fn set_password(&self, ssid: &str, password: &str) -> anyhow::Result<()> {
        anyhow::ensure!(
            (1..=SSID_LIMIT).contains(&ssid.len()),
            "an SSID is 1 to {SSID_LIMIT} bytes"
        );
        anyhow::ensure!(
            password.len() <= PASSWORD_LIMIT,
            "a password is at most {PASSWORD_LIMIT} bytes"
        );
        let nvs = EspNvs::new(self.partition.clone(), CREDENTIALS, true)?;
        let key = std::ffi::CString::new(key(ssid))?;
        let value = std::ffi::CString::new(password)?;
        // SAFETY: valid C strings on an open handle. IDF's set writes the new entry before it erases the old one
        // (esp-idf-svc's set_str erases first, so a cut would lose the password).
        sys::esp!(unsafe { sys::nvs_set_str(nvs.handle(), key.as_ptr(), value.as_ptr()) })?;
        sys::esp!(unsafe { sys::nvs_commit(nvs.handle()) })?;
        Ok(())
    }
}

/// Write `bytes` under `key` with IDF's own update, which writes the new entry before it erases the old one, so a
/// cut keeps one of the two. (esp-idf-svc's set_blob erases the key first: a cut there loses the document, and
/// the next boot would run the default without saying so.) The recorder's session-start record goes through it
/// too (crate::recorder).
pub fn replace_blob(nvs: &EspNvs<NvsCustom>, key: &str, bytes: &[u8]) -> anyhow::Result<()> {
    let key = std::ffi::CString::new(key)?;
    // SAFETY: a valid C string and a byte slice of the given length on an open handle.
    sys::esp!(unsafe { sys::nvs_set_blob(nvs.handle(), key.as_ptr(), bytes.as_ptr().cast(), bytes.len()) })?;
    sys::esp!(unsafe { sys::nvs_commit(nvs.handle()) })?;
    Ok(())
}

/// The NVS key of a network's password: `w` and the zlib CRC-32 of the SSID in 8 hex digits. NVS keys stop at 15
/// characters and an SSID may run to 32; tools/provision.sh computes the same key.
fn key(ssid: &str) -> String {
    // SAFETY: the ROM CRC reads exactly `len` bytes of the slice. Seeded with 0 it is zlib's crc32.
    let checksum = unsafe { sys::esp_rom_crc32_le(0, ssid.as_ptr(), ssid.len() as u32) };
    format!("w{checksum:08x}")
}

impl Credentials {
    /// The credential store's console command (the Glider Command Language), registered at boot. Console only:
    /// over CC the password would land in the hub's log.
    pub fn register_commands(&self) {
        let credentials = self.clone();
        firmware::gcl::register(vec![Command {
            owner: "credentials",
            words: "wifi",
            about: "store a network's password (the network list itself is the board config's)",
            gate: Gate::Setting,
            reach: Reach::Console,
            parameters: &[
                Parameter {
                    name: "ssid",
                    kind: Kind::Text,
                    default: None,
                    about: "the network, 1..32 bytes",
                },
                Parameter {
                    name: "password",
                    kind: Kind::Text,
                    default: None,
                    about: "its password, at most 64 bytes (tools/avionics_console.py --wifi reads it from the creds file)",
                },
            ],
            bare_words: &[],
            run: Box::new(move |arguments| {
                match credentials.set_password(arguments.text("ssid"), arguments.text("password")) {
                    Ok(()) => cc_protocol::build("ok", &[arguments.text("ssid")], &[]),
                    Err(error) => cc_protocol::build("err", &["store", &error.to_string()], &[]),
                }
            }),
        }]);
    }
}
