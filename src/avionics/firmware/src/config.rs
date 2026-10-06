//! The board's settings store: the `cfg` NVS partition (partitions.csv), namespace `config`. It holds what
//! src/glider kept as files on its VFS -- launch.config now, board.config with the config store (port plan M2).
//! Values are JSON text in blobs: NVS strings stop at one 4 KB page, and a launch.config with sites can pass it.

use esp_idf_svc::nvs::{EspCustomNvsPartition, EspNvs, NvsCustom};

/// The partition's label in partitions.csv.
const PARTITION: &str = "cfg";
/// The namespace inside it.
const NAMESPACE: &str = "config";
/// The key of launch.config (src/glider mission.LAUNCH_PATH).
const LAUNCH_KEY: &str = "launch";

/// The open settings store.
pub struct Store {
    /// The `config` namespace on the `cfg` partition.
    nvs: EspNvs<NvsCustom>,
}

impl Store {
    /// Open the store. A partition another firmware left foreign is erased and formatted (esp-idf-svc), so this
    /// fails only on a partition table without `cfg`.
    pub fn open() -> anyhow::Result<Self> {
        let partition = EspCustomNvsPartition::take(PARTITION)?;
        Ok(Self {
            nvs: EspNvs::new(partition, NAMESPACE, true)?,
        })
    }

    /// The stored launch.config text; None when there is none (the mission then starts from its default).
    pub fn launch(&self) -> Option<String> {
        self.text(LAUNCH_KEY)
    }

    /// Store launch.config. NVS writes the new value before it drops the old one, so a cut keeps one of them.
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
