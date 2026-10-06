//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! S1/S2: the microSD on SDMMC slot 0 (shared host with the C6 on slot 1). Mounts it (never formats), prints the
//! card, then writes 32 MB per chunk size from an internal-RAM buffer and from a PSRAM buffer, timing every write
//! (the worst case sizes the PSRAM ring in front of the card) and the sustained rate including the final fsync,
//! and repeats into a preallocated contiguous file (the raw-like path). Reads a sample back to verify. Touches
//! only its own files under /sdcard/COLUDO and removes them. Prints `sd;<group>;<name>;<value>;<unit>`.

use std::fs::{File, OpenOptions};
use std::io::{Read, Seek, SeekFrom, Write};
use std::time::Duration;

use esp_idf_svc::sys;
use firmware::thermal::Thermometer;

/// Bytes written per chunk size and buffer kind.
const TOTAL: usize = 32 << 20;
/// Where the card is mounted.
const BASE: &str = "/sdcard";

fn main() {
    esp_idf_svc::sys::link_patches();
    let thermometer = esp_idf_svc::hal::peripherals::Peripherals::take()
        .ok()
        .and_then(|peripherals| Thermometer::new(peripherals.temp_sensor).ok());
    let die = |when: &str| {
        let celsius = thermometer.as_ref().and_then(Thermometer::celsius).unwrap_or(f32::NAN);
        println!("sd;thermal;die_{when};{celsius:.1};C");
    };
    std::thread::sleep(Duration::from_millis(300));
    die("start");
    let start = micros();
    // SAFETY: mounts once; the path is a valid C string.
    let mounted = unsafe { sys::sd_card_mount(c"/sdcard".as_ptr()) };
    report("mount", "result", mounted as f32, "esp_err");
    report("mount", "time", (micros() - start) as f32 / 1000.0, "ms");
    if mounted != sys::ESP_OK {
        println!("sd;done");
        idle();
    }
    // SAFETY: prints the mounted card's identity.
    unsafe { sys::sd_card_print_info() };
    let directory = format!("{BASE}/COLUDO");
    let _ = std::fs::create_dir_all(&directory);
    let internal = sys::MALLOC_CAP_INTERNAL | sys::MALLOC_CAP_DMA | sys::MALLOC_CAP_8BIT;
    let psram = sys::MALLOC_CAP_SPIRAM | sys::MALLOC_CAP_8BIT;
    for chunk in [4usize << 10, 32 << 10, 128 << 10, 512 << 10] {
        for (memory, caps) in [("internal", internal), ("psram", psram)] {
            if memory == "internal" && chunk > (128 << 10) {
                continue; // no 512 KB block of internal RAM
            }
            let path = format!("{directory}/bench_{memory}_{}k.bin", chunk >> 10);
            write_test(&format!("{memory}_{}k", chunk >> 10), &path, chunk, caps, false);
            let _ = std::fs::remove_file(&path);
        }
    }
    // the raw-like path: one preallocated contiguous file, rewritten in place
    let path = format!("{directory}/bench_contiguous.bin");
    write_test("contiguous_psram_512k", &path, 512 << 10, psram, true);
    let _ = std::fs::remove_file(&path);
    die("end");
    println!("sd;done");
    idle();
}

fn idle() -> ! {
    loop {
        std::thread::sleep(Duration::from_secs(3600));
    }
}

fn report(group: &str, name: &str, value: f32, unit: &str) {
    println!("sd;{group};{name};{value:.3};{unit}");
}

fn micros() -> i64 {
    // SAFETY: a plain read of the system timer.
    unsafe { sys::esp_timer_get_time() }
}

/// Write TOTAL bytes to `path` in `chunk`-byte writes from a `caps` buffer; report rate and latency spread.
fn write_test(label: &str, path: &str, chunk: usize, caps: u32, contiguous: bool) {
    // SAFETY: an aligned allocation, checked, filled, freed at the end of this function.
    let buffer = unsafe { sys::heap_caps_aligned_alloc(64, chunk, caps) } as *mut u8;
    if buffer.is_null() {
        report(label, "skipped_no_buffer", chunk as f32, "bytes");
        return;
    }
    // SAFETY: `chunk` bytes were just allocated.
    let data = unsafe { core::slice::from_raw_parts_mut(buffer, chunk) };
    for (index, byte) in data.iter_mut().enumerate() {
        *byte = (index as u32).wrapping_mul(2_654_435_761).to_le_bytes()[3];
    }
    let file = if contiguous {
        let (base, full) = (
            std::ffi::CString::new(BASE).unwrap(),
            std::ffi::CString::new(path).unwrap(),
        );
        let start = micros();
        // SAFETY: valid C strings; allocates the file's clusters up front.
        let made = unsafe { sys::esp_vfs_fat_create_contiguous_file(base.as_ptr(), full.as_ptr(), TOTAL as u64, true) };
        report(label, "preallocate", (micros() - start) as f32 / 1000.0, "ms");
        if made != sys::ESP_OK {
            report(label, "preallocate_failed", made as f32, "esp_err");
            // SAFETY: allocated above.
            unsafe { sys::heap_caps_free(buffer.cast()) };
            return;
        }
        OpenOptions::new().write(true).open(path)
    } else {
        File::create(path)
    };
    let Ok(mut file) = file else {
        report(label, "open_failed", 0.0, "-");
        // SAFETY: allocated above.
        unsafe { sys::heap_caps_free(buffer.cast()) };
        return;
    };
    let mut latencies: Vec<u32> = Vec::with_capacity(TOTAL / chunk);
    let start = micros();
    let mut failed = false;
    for _ in 0..TOTAL / chunk {
        let before = micros();
        if file.write_all(data).is_err() {
            failed = true;
            break;
        }
        latencies.push((micros() - before) as u32);
    }
    let synced = micros();
    let _ = file.sync_all();
    let end = micros();
    if failed {
        report(label, "write_failed_after", latencies.len() as f32, "chunks");
    }
    let seconds = (end - start) as f32 / 1e6;
    report(label, "rate", (latencies.len() * chunk) as f32 / seconds / 1e6, "MB/s");
    report(label, "fsync", (end - synced) as f32 / 1000.0, "ms");
    latencies.sort_unstable();
    let at = |rank: usize| {
        latencies
            .get((latencies.len().max(1) - 1) * rank / 1000)
            .copied()
            .unwrap_or(0) as f32
    };
    report(label, "write_p50", at(500) / 1000.0, "ms");
    report(label, "write_p99", at(990) / 1000.0, "ms");
    report(label, "write_max", at(1000) / 1000.0, "ms");
    // verify a chunk from the middle of the file
    let mut back = vec![0u8; chunk];
    drop(file);
    let verified = File::open(path)
        .and_then(|mut reader| {
            reader.seek(SeekFrom::Start((TOTAL / 2 / chunk * chunk) as u64))?;
            reader.read_exact(&mut back)
        })
        .is_ok()
        && back == data;
    report(label, "readback_matches", verified as u8 as f32, "bool");
    // SAFETY: allocated above, freed once.
    unsafe { sys::heap_caps_free(buffer.cast()) };
}
