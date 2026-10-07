//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The microSD card (components/sd_card): mount once at boot, name the card by its FAT volume serial, report free
//! space, and a write test an operator can run from the console.

use std::fs::File;
use std::io::Write;
use std::sync::atomic::{AtomicU32, Ordering};

use esp_idf_svc::sys;

use crate::gcl::{self, Command, Gate, Kind, Parameter, Reach};

/// The last measured free space in MB (u32::MAX = unknown). Health reads this, never the card: f_getfree waits
/// for the volume lock a writer holds and can scan megabytes of FAT -- it stalled the hub link for seconds.
static FREE_MEGABYTES: AtomicU32 = AtomicU32::new(u32::MAX);

/// Where the card is mounted.
pub const MOUNT: &str = "/sdcard";

/// The ring_saver's directory on the card: one directory per session and the global session.csv, as recorderd laid
/// out the Luckfox's /userdata/recordings (doc/specs/avionics-port.md "SD and flash file layout"). `GET
/// /sd/recordings/<session>/` serves a session to tools/board_pull.sh.
pub const RECORDINGS: &str = "/sdcard/recordings";

/// Mount the card (never formats). False when there is no usable card; the rest of the board runs on.
pub fn mount() -> bool {
    // SAFETY: mounts once; a valid C string.
    unsafe { sys::sd_card_mount(c"/sdcard".as_ptr()) == sys::ESP_OK }
}

/// The mounted card's FAT volume serial number (`blkid`'s UUID `XXXX-XXXX`), read from its boot sector: what the
/// session-start record names the card by, so the next boot knows whether the card is the one this boot saved to.
/// None when no card is mounted or the boot sector would not read.
pub fn volume_serial() -> Option<u32> {
    let mut serial = 0u32;
    // SAFETY: a live output; the call reads the mounted card's boot sector under FatFs's own volume lock.
    (unsafe { sys::sd_card_volume_serial(&mut serial) } == sys::ESP_OK).then_some(serial)
}

/// Measure the free space now (slow: the volume lock and possibly a FAT scan) and remember it. Call off the hot
/// paths -- after mounting, after a write burst.
pub fn measure_free() -> Option<u32> {
    let (mut total, mut free) = (0u64, 0u64);
    // SAFETY: a valid C string and two live outputs.
    let status = unsafe { sys::esp_vfs_fat_info(c"/sdcard".as_ptr(), &mut total, &mut free) };
    let megabytes = (status == sys::ESP_OK).then_some((free >> 20) as u32);
    FREE_MEGABYTES.store(megabytes.unwrap_or(u32::MAX), Ordering::Relaxed);
    megabytes
}

/// The last measured free space in whole megabytes; never touches the card. None until measured.
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
    measure_free();
    result
}

/// The card's console command (the Glider Command Language, crate::gcl), registered when the card mounts.
pub fn register_commands() {
    gcl::register(vec![Command {
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
    }]);
}
