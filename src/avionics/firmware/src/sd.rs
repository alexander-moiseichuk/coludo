//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The microSD card (components/sd_card): mount once at boot, name the card by its FAT volume serial, report free
//! space, a write test an operator can run from the console, and the card's files from the console or the hub: `ls`,
//! `df`, `rm` (the flight recorder's console has the same three, src/avionics/recorder).
//!
//! The card holds one or two FAT volumes (doc/specs/avionics-port.md "SD and flash file layout"): partition 1, the
//! recordings, at [`MOUNT`] (`/sdcard`), and on a two-partition card partition 2, the videos, at [`videos_mount`]
//! (`/sdcard/videos`, crates/recorder layout's `videos` directory under the card's root). A one-partition card keeps
//! `videos/` as a plain directory on its one volume. Paths on the card come from crates/recorder's layout, never
//! from here.

use std::ffi::CString;
use std::fs::File;
use std::io::Write;
use std::sync::atomic::{AtomicU32, Ordering};

use esp_idf_svc::sys;
use recorder::layout;

use crate::gcl::{self, Command, Gate, Kind, Parameter, Reach, Word};

/// The last measured free space in MB of the recordings volume, the one the ring_saver writes (u32::MAX = unknown):
/// health's `sd_free_mb`. Not both volumes together: a full recordings partition stops the recordings while the
/// videos partition may still hold gigabytes, and a sum would hide that. The videos partition's figure is in the boot
/// log and after each `video stop` ([`describe`]). Health reads this, never the card: f_getfree waits for the volume
/// lock a writer holds and can scan megabytes of FAT -- it stalled the hub link for seconds.
static FREE_MEGABYTES: AtomicU32 = AtomicU32::new(u32::MAX);

/// Where the card's root is mounted: partition 1 (or a one-partition card's only volume).
pub const MOUNT: &str = "/sdcard";

/// How the card's volumes are mounted (components/sd_card `sd_card_layout_t`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Layout {
    /// One volume at [`MOUNT`]: a superfloppy, one FAT partition, or a second partition that is not FAT (an M4-era
    /// card). The videos directory is a plain directory on it.
    OnePartition,
    /// Partition 1 at [`MOUNT`], partition 2 at [`videos_mount`].
    TwoPartitions,
    /// Partition 1 at [`MOUNT`] alone: partition 2 is FAT by its type but would not mount, or the table is unsound
    /// (the two partitions overlap, one passes the card's end, or a file system runs past its partition; the boot log
    /// says which). Video stays off: on partition 1 it would fill the recordings volume (a few GB) and stop the
    /// ring_saver.
    VideosFailed,
}

impl Layout {
    /// How the boot log names it.
    pub fn name(self) -> &'static str {
        match self {
            Layout::OnePartition => "one partition",
            Layout::TwoPartitions => "two partitions",
            Layout::VideosFailed => "two partitions, the videos one not mounted",
        }
    }
}

/// Where the videos partition is mounted on a two-partition card: the layout's videos directory under [`MOUNT`]
/// (14 bytes, within IDF's ESP_VFS_PATH_MAX of 15). IDF's VFS takes the longest matching prefix, so it shadows that
/// directory of partition 1. vfs_fat keeps it cut to 13 bytes, so its by-path calls (`esp_vfs_fat_info`) never find
/// it: nothing here asks them by path ([`measure_free`] goes by the FatFs drive).
pub fn videos_mount() -> String {
    format!("{MOUNT}/{}", layout::VIDEOS)
}

/// Mount the card (never formats): partition 1 at [`MOUNT`] and, when the MBR's partitions 1 and 2 are both FAT and
/// sound, partition 2 at [`videos_mount`]. False when there is no usable card (a sector 0 that will not read
/// included); the rest of the board runs on.
pub fn mount() -> bool {
    let (Ok(base), Ok(videos)) = (CString::new(MOUNT), CString::new(videos_mount())) else {
        return false;
    };
    // SAFETY: mounts once; two valid C strings, copied by the VFS before the call returns.
    unsafe { sys::sd_card_mount(base.as_ptr(), videos.as_ptr()) == sys::ESP_OK }
}

/// How the card is mounted; None without a card.
pub fn layout() -> Option<Layout> {
    // SAFETY: a plain read of the mount's outcome.
    match unsafe { sys::sd_card_layout() } {
        sys::sd_card_layout_t_SD_CARD_LAYOUT_ONE_PARTITION => Some(Layout::OnePartition),
        sys::sd_card_layout_t_SD_CARD_LAYOUT_TWO_PARTITIONS => Some(Layout::TwoPartitions),
        sys::sd_card_layout_t_SD_CARD_LAYOUT_VIDEOS_FAILED => Some(Layout::VideosFailed),
        _ => None,
    }
}

/// Whether the videos partition is mounted at [`videos_mount`]: a two-partition card whose partition 2 mounted.
pub fn videos_mounted() -> bool {
    layout() == Some(Layout::TwoPartitions)
}

/// The mounted card's FAT volume serial number (`blkid`'s UUID `XXXX-XXXX`), partition 1's, read from its boot
/// sector: what the session-start record names the card by, so the next boot knows whether the card is the one this
/// boot saved to. None when no card is mounted or the boot sector would not read.
pub fn volume_serial() -> Option<u32> {
    let mut serial = 0u32;
    // SAFETY: a live output; the call reads the mounted card's boot sector under FatFs's own volume lock.
    (unsafe { sys::sd_card_volume_serial(&mut serial) } == sys::ESP_OK).then_some(serial)
}

/// The videos partition's FAT volume serial number, for the boot log; None unless it is mounted, or when its boot
/// sector would not read.
pub fn videos_serial() -> Option<u32> {
    let mut serial = 0u32;
    // SAFETY: a live output; the call reads the videos volume's boot sector under FatFs's own volume lock.
    (unsafe { sys::sd_card_videos_serial(&mut serial) } == sys::ESP_OK).then_some(serial)
}

/// A FAT volume serial as `blkid` shows it: `1A2B-3C4D`.
pub fn serial_name(serial: u32) -> String {
    format!("{:04X}-{:04X}", serial >> 16, serial & 0xFFFF)
}

/// One mounted volume's size and free space, in whole megabytes.
#[derive(Clone, Copy, Debug)]
pub struct Space {
    /// Its size.
    pub total_megabytes: u32,
    /// What is free on it.
    pub free_megabytes: u32,
}

/// The card's volumes as [`measure_free`] found them.
#[derive(Clone, Copy, Debug)]
pub struct Volumes {
    /// The volume at [`MOUNT`] (partition 1); None when it would not answer.
    pub recordings: Option<Space>,
    /// The videos partition; None on a one-partition card, or when it would not answer.
    pub videos: Option<Space>,
}

/// One volume's size and free space now; None when it is not mounted or FatFs would not tell. Asked by its FatFs
/// drive (components/sd_card `sd_card_space`), not by its path: IDF's `esp_vfs_fat_info` looks a volume up by the
/// base path vfs_fat keeps, cut to 13 characters, so it never finds `/sdcard/videos`.
fn space(volume: sys::sd_card_volume_t) -> Option<Space> {
    let (mut total, mut free) = (0u64, 0u64);
    // SAFETY: two live outputs; the call takes the volume's FatFs lock.
    let status = unsafe { sys::sd_card_space(volume, &mut total, &mut free) };
    (status == sys::ESP_OK).then_some(Space {
        total_megabytes: (total >> 20) as u32,
        free_megabytes: (free >> 20) as u32,
    })
}

/// Measure both volumes' free space now (slow: each volume's lock and possibly a FAT scan) and remember the
/// recordings volume's, which `health` reports as `sd_free_mb`. Call off the hot paths -- after mounting, after a
/// write burst.
///
/// Returns each volume as measured, for the caller to log ([`describe`]).
pub fn measure_free() -> Volumes {
    let volumes = Volumes {
        recordings: space(sys::sd_card_volume_t_SD_CARD_VOLUME_RECORDINGS),
        videos: if videos_mounted() {
            space(sys::sd_card_volume_t_SD_CARD_VOLUME_VIDEOS)
        } else {
            None
        },
    };
    let free = volumes.recordings.map(|recordings| recordings.free_megabytes);
    FREE_MEGABYTES.store(free.unwrap_or(u32::MAX), Ordering::Relaxed);
    volumes
}

/// The volumes as one log text: `recordings 4092 MB, 4000 MB free; videos 11000 MB, 10990 MB free`, or the one
/// volume's figures alone on a one-partition card.
pub fn describe(volumes: &Volumes) -> String {
    let text = |space: Option<Space>| {
        space.map_or_else(
            || "size unknown".to_string(),
            |space| format!("{} MB, {} MB free", space.total_megabytes, space.free_megabytes),
        )
    };
    match layout() {
        Some(Layout::TwoPartitions) => format!(
            "recordings {}; videos {}",
            text(volumes.recordings),
            text(volumes.videos)
        ),
        _ => text(volumes.recordings),
    }
}

/// The last measured free space of the recordings volume (partition 1, or a one-partition card's only volume) in
/// whole megabytes; never touches the card. None until measured.
pub fn free_megabytes() -> Option<u32> {
    let megabytes = FREE_MEGABYTES.load(Ordering::Relaxed);
    (megabytes != u32::MAX).then_some(megabytes)
}

/// What a write test measured.
pub struct WriteTest {
    /// Sustained rate including the final fsync, MB/s.
    pub rate: f32,
    /// The slowest single write, ms.
    pub worst_write_ms: f32,
}

/// Write `megabytes` to `<mount>/COLUDO/sdtest.bin` in 128 KB chunks from a cache-aligned PSRAM buffer (the
/// video/mover path), then remove it. `kilobytes_per_second` paces the writes (the video case: ~1000 at 8 Mbit/s);
/// None writes flat out.
pub fn write_test(megabytes: usize, kilobytes_per_second: Option<u32>) -> anyhow::Result<WriteTest> {
    const CHUNK: usize = 128 << 10;
    let micros = || unsafe { sys::esp_timer_get_time() };
    std::fs::create_dir_all(format!("{MOUNT}/COLUDO"))?;
    let path = format!("{MOUNT}/COLUDO/sdtest.bin");
    // SAFETY: an aligned PSRAM allocation, checked, freed before returning.
    let buffer = unsafe { sys::heap_caps_aligned_alloc(64, CHUNK, sys::MALLOC_CAP_SPIRAM | sys::MALLOC_CAP_8BIT) };
    anyhow::ensure!(!buffer.is_null(), "no 128 KB PSRAM buffer");
    // SAFETY: CHUNK bytes were just allocated.
    let data = unsafe { core::slice::from_raw_parts_mut(buffer as *mut u8, CHUNK) };
    data.fill(0x5a);
    let result = (|| {
        let mut file = File::create(&path)?;
        let mut worst = 0i64;
        let start = micros();
        // one 128 KB chunk per period at the paced rate
        let period = kilobytes_per_second.map(|rate| (CHUNK as i64 / 1024) * 1_000_000 / rate.max(1) as i64);
        for index in 0..(megabytes << 20) / CHUNK {
            if let Some(period) = period {
                let due = start + index as i64 * period;
                let now = micros();
                if due > now {
                    std::thread::sleep(std::time::Duration::from_micros((due - now) as u64));
                }
            }
            let before = micros();
            file.write_all(data)?;
            worst = worst.max(micros() - before);
        }
        file.sync_all()?;
        let seconds = (micros() - start) as f32 / 1e6;
        Ok(WriteTest {
            rate: megabytes as f32 / seconds * 1.048_576,
            worst_write_ms: worst as f32 / 1000.0,
        })
    })();
    let _ = std::fs::remove_file(&path);
    // SAFETY: allocated above, freed once.
    unsafe { sys::heap_caps_free(buffer) };
    let volumes = measure_free();
    log::info!("sd :: after the write test: {}", describe(&volumes));
    result
}

/// One directory's entries: `[name, bytes]` for a file, `[name/, null]` for a directory, as `ls` answers them.
///
/// Args:
/// * `relative` - the directory from the card's root (`/` or empty for the root).
///
/// Returns the JSON reply's `entries`. Errors: `..` in the path, or the directory would not read (the error says
/// why: a missing one, a file).
fn list(relative: &str) -> Result<Vec<serde_json::Value>, String> {
    let relative = relative.trim_matches('/');
    if relative.split('/').any(|part| part == "..") {
        return Err("no .. in a path".into());
    }
    let path = if relative.is_empty() {
        MOUNT.to_string()
    } else {
        format!("{MOUNT}/{relative}")
    };
    let entries = std::fs::read_dir(&path).map_err(|error| format!("{path}: {error}"))?;
    let mut listed = Vec::new();
    let mut videos_listed = false;
    for entry in entries.flatten() {
        let name = entry.file_name().to_string_lossy().into_owned();
        videos_listed |= name.eq_ignore_ascii_case(layout::VIDEOS); // FAT folds case
        match entry.metadata() {
            Ok(metadata) if metadata.is_dir() => listed.push(serde_json::json!([format!("{name}/"), null])),
            Ok(metadata) => listed.push(serde_json::json!([name, metadata.len()])),
            Err(error) => listed.push(serde_json::json!([name, error.to_string()])),
        }
    }
    // the videos partition is mounted at the root's `videos`, a mount point partition 1 does not list
    if relative.is_empty() && videos_mounted() && !videos_listed {
        listed.push(serde_json::json!([format!("{}/", layout::VIDEOS), null]));
    }
    Ok(listed)
}

/// Removes a file, or a directory with all it holds, from the card.
///
/// Args:
/// * `relative` - the path from the card's root.
/// * `session` - this boot's session: its directories are refused, and so is any directory that holds one (the
///   root, `videos`): the ring_saver and the camera write there now.
///
/// Returns the path removed, as the card names it. Errors: the path is refused, or it would not go (a file a writer
/// holds open answers busy).
fn remove(relative: &str, session: &str) -> Result<String, String> {
    let relative = relative.trim_matches('/');
    if relative.is_empty() || relative.split('/').any(|part| part.is_empty() || part == "..") {
        return Err(format!("{relative:?} is the card's root, or not a path on it"));
    }
    // ancestor-or-self either way, in any case (FAT folds case)
    let related = |live: &str| {
        let (path, live) = (relative.to_ascii_lowercase(), live.to_ascii_lowercase());
        path == live || live.starts_with(&format!("{path}/")) || path.starts_with(&format!("{live}/"))
    };
    for kind in [layout::RECORDINGS, layout::VIDEOS] {
        let live = layout::directory(session, kind);
        let live = if kind == layout::VIDEOS {
            live
        } else {
            live.split('/').next().unwrap_or("").to_string()
        };
        if related(&live) {
            return Err(format!(
                "{relative} holds or is in this session's {live}, which is being written"
            ));
        }
    }
    let path = format!("{MOUNT}/{relative}");
    let removed = match std::fs::metadata(&path) {
        Ok(metadata) if metadata.is_dir() => std::fs::remove_dir_all(&path),
        Ok(_) => std::fs::remove_file(&path),
        Err(error) => Err(error),
    };
    removed.map_err(|error| format!("{path}: {error}"))?;
    let volumes = measure_free();
    log::info!("sd :: removed {path}: {}", describe(&volumes));
    Ok(relative.to_string())
}

/// `rm`'s word: nothing removed comes back.
const RM_CONFIRM: Word = Word {
    name: "confirm",
    about: "required: what it removes is gone for good",
};

/// The card's console commands (the Glider Command Language, crate::gcl), registered when the card mounts.
///
/// Args:
/// * `session` - this boot's session, which `rm` refuses.
pub fn register_commands(session: String) {
    gcl::register(vec![
        Command {
            owner: "sd",
            words: "ls",
            about: "a directory of the card: [name, bytes] per file, [name/, null] per directory",
            gate: Gate::Open,
            reach: Reach::Everywhere,
            parameters: &[Parameter {
                name: "path",
                kind: Kind::Text,
                default: Some("/"),
                about: "the directory from the card's root: <session>/recordings, videos/<session>",
            }],
            bare_words: &[],
            run: Box::new(|arguments| {
                let path = arguments.text("path");
                match list(path).and_then(|entries| {
                    serde_json::to_string(&serde_json::json!({"path": path, "entries": entries}))
                        .map_err(|error| error.to_string())
                }) {
                    Ok(json) => cc_protocol::build("ok", &[&json], &[]),
                    Err(error) => cc_protocol::build("err", &["ls", &error], &[]),
                }
            }),
        },
        Command {
            owner: "sd",
            words: "df",
            about: "the card's size and free space per volume (recordings; videos on a two-partition card)",
            gate: Gate::Open,
            reach: Reach::Everywhere,
            parameters: &[],
            bare_words: &[],
            run: Box::new(|_| {
                let volumes = measure_free();
                let mut named = vec![("layout", layout().map_or("none", Layout::name).replace(' ', "_"))];
                for (name, space) in [("recordings", volumes.recordings), ("videos", volumes.videos)] {
                    if let Some(space) = space {
                        named.push((name, format!("{}/{}", space.free_megabytes, space.total_megabytes)));
                    }
                }
                let named: Vec<(&str, &str)> = named.iter().map(|(name, value)| (*name, value.as_str())).collect();
                cc_protocol::build("ok", &["df"], &named)
            }),
        },
        Command {
            owner: "sd",
            words: "rm",
            about: "remove a file, or a directory with all it holds; never this session's own directories",
            gate: Gate::Setting,
            reach: Reach::Everywhere,
            parameters: &[Parameter {
                name: "path",
                kind: Kind::Text,
                default: None,
                about: "the path from the card's root: <session>, videos/<session>, a file",
            }],
            bare_words: &[RM_CONFIRM],
            run: Box::new(move |arguments| match remove(arguments.text("path"), &session) {
                Ok(path) => cc_protocol::build("ok", &["rm"], &[("path", &path)]),
                Err(error) => cc_protocol::build("err", &["rm", &error], &[]),
            }),
        },
        Command {
            owner: "sd",
            words: "sdtest",
            about: "write to the card and report the sustained rate and the worst single write (the ring sizes this)",
            gate: Gate::Setting,
            reach: Reach::Everywhere,
            parameters: &[
                Parameter {
                    name: "megabytes",
                    kind: Kind::Integer { low: 1, high: 1024 },
                    default: Some("16"),
                    about: "how much to write",
                },
                Parameter {
                    name: "rate",
                    kind: Kind::Integer { low: 0, high: 100_000 },
                    default: Some("0"),
                    about: "pace the writes at this many KB/s (the video case is ~1000 at 8 Mbit/s); 0 writes flat out",
                },
            ],
            bare_words: &[],
            run: Box::new(|arguments| {
                let rate = arguments.integer("rate");
                let pace = (rate > 0).then_some(rate as u32);
                match write_test(arguments.integer("megabytes") as usize, pace) {
                    Ok(test) => cc_protocol::build(
                        "ok",
                        &["sdtest"],
                        &[
                            ("rate_mbs", &format!("{:.2}", test.rate)),
                            ("worst_ms", &format!("{:.1}", test.worst_write_ms)),
                        ],
                    ),
                    Err(error) => cc_protocol::build("err", &["sdtest", &error.to_string()], &[]),
                }
            }),
        },
    ]);
}
