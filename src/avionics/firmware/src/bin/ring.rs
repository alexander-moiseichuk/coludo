//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! G1 (doc/specs/avionics-port.md, risk R1): does the GD 0xC84019 NOR take many small programs into one 256 B
//! page? The flash ring appends 40-200 B frames at 8-byte-aligned offsets, one esp_partition_write_raw each, so a
//! page is programmed several times over (only 1 -> 0 bit changes into erased bytes). If the part cannot, frames
//! must be padded to whole pages (about twice the flash per second). Runs on the `log` partition above 16 MB (the
//! 4-byte-addressed half) and prints `ring;<group>;<name>;<value>;<unit>`:
//!  * frames: 10 000 frames of 40-200 B, each header + payload in ONE program at an 8-aligned offset, then every
//!    byte read back -- the frames and the 0xFF gaps between them;
//!  * dense: 32 programs of 8 B into each of 64 pages, read back (the worst case the ring can produce);
//!  * the per-frame program latency spread and the 64 KB read that checks a block is erased before use.

use std::time::Duration;

use esp_idf_svc::sys;

/// Where the test region starts inside the `log` partition: 16 MB in, past the 3-byte address limit.
const REGION: u32 = 16 << 20;
/// The region's size: 2 MB, erased first (10 000 frames at ~128 B average need ~1.3 MB).
const REGION_SIZE: u32 = 2 << 20;
/// Frames in the frames test.
const FRAMES: usize = 10_000;
/// Frame size bounds, bytes (a header is 40, a 50 ms frame of a quiet stream is ~200).
const FRAME_BYTES: (usize, usize) = (40, 200);
/// The frames' alignment (the frame format's).
const ALIGN: usize = 8;

fn main() {
    esp_idf_svc::sys::link_patches();
    std::thread::sleep(Duration::from_millis(300));
    let partition = log_partition();
    // SAFETY: erases a range inside the partition the lookup returned.
    let start = micros();
    unsafe {
        sys::esp!(sys::esp_partition_erase_range(
            partition,
            REGION as usize,
            REGION_SIZE as usize
        ))
    }
    .expect("erase");
    report("setup", "erase_2mb", (micros() - start) as f32 / 1000.0, "ms");
    frames(partition);
    dense(partition);
    erased_check(partition);
    println!("ring;done");
    loop {
        std::thread::sleep(Duration::from_secs(3600));
    }
}

/// 10 000 frames appended at 8-aligned offsets, one program each, then the whole span read back.
fn frames(partition: *const sys::esp_partition_t) {
    let mut seed: u32 = 0x1234_5678;
    let mut offset = 0usize;
    let mut placed: Vec<(usize, usize, u32)> = Vec::with_capacity(FRAMES); // (offset, length, pattern seed)
    let mut latencies: Vec<f32> = Vec::with_capacity(FRAMES);
    let mut pages_touched_twice = 0usize;
    for _ in 0..FRAMES {
        let length = FRAME_BYTES.0 + (next(&mut seed) as usize % (FRAME_BYTES.1 - FRAME_BYTES.0 + 1));
        let pattern = next(&mut seed);
        let frame = fill(length, pattern);
        if !offset.is_multiple_of(256) {
            pages_touched_twice += 1; // this program lands in a page an earlier one already programmed
        }
        let start = micros();
        // SAFETY: writes `length` bytes inside the erased test region.
        unsafe {
            sys::esp!(sys::esp_partition_write_raw(
                partition,
                REGION as usize + offset,
                frame.as_ptr().cast(),
                length
            ))
        }
        .expect("program");
        latencies.push((micros() - start) as f32);
        placed.push((offset, length, pattern));
        offset = (offset + length).next_multiple_of(ALIGN);
    }
    report("frames", "count", FRAMES as f32, "frames");
    report("frames", "bytes_spanned", offset as f32, "bytes");
    report(
        "frames",
        "programs_into_a_used_page",
        pages_touched_twice as f32,
        "programs",
    );
    report_spread("frames", "program", &mut latencies, "us");
    // read the whole span back in 32 KB chunks and check every byte: frames match, gaps stay 0xFF
    let mut expected = vec![0xFFu8; offset];
    for &(at, length, pattern) in &placed {
        expected[at..at + length].copy_from_slice(&fill(length, pattern));
    }
    let mut actual = vec![0u8; offset];
    for chunk in (0..offset).step_by(32 << 10) {
        let length = (32 << 10).min(offset - chunk);
        // SAFETY: reads inside the test region into a buffer of that length.
        unsafe {
            sys::esp!(sys::esp_partition_read(
                partition,
                REGION as usize + chunk,
                actual[chunk..].as_mut_ptr().cast(),
                length
            ))
        }
        .expect("read");
    }
    let bad_frames = placed
        .iter()
        .filter(|&&(at, length, _)| actual[at..at + length] != expected[at..at + length])
        .count();
    let bad_gap_bytes = (0..offset)
        .filter(|&index| expected[index] == 0xFF && actual[index] != 0xFF)
        .count();
    report("frames", "frames_corrupt", bad_frames as f32, "frames");
    report("frames", "gap_bytes_not_erased", bad_gap_bytes as f32, "bytes");
    report(
        "frames",
        "bytes_differing",
        (0..offset).filter(|&i| actual[i] != expected[i]).count() as f32,
        "bytes",
    );
}

/// The worst case: 32 programs of 8 B into each of 64 pages, read back.
fn dense(partition: *const sys::esp_partition_t) {
    const PAGES: usize = 64;
    let base = REGION as usize + (REGION_SIZE as usize - PAGES * 256); // the region's erased tail
    let mut seed: u32 = 0x9abc_def0;
    let mut expected = vec![0u8; PAGES * 256];
    for (index, slot) in expected.chunks_mut(8).enumerate() {
        let bytes = fill(8, next(&mut seed));
        slot.copy_from_slice(&bytes);
        // SAFETY: 8 bytes inside the erased tail of the test region.
        unsafe {
            sys::esp!(sys::esp_partition_write_raw(
                partition,
                base + index * 8,
                bytes.as_ptr().cast(),
                8
            ))
        }
        .expect("program");
    }
    let mut actual = vec![0u8; PAGES * 256];
    // SAFETY: reads the tail just written into a buffer of that length.
    unsafe {
        sys::esp!(sys::esp_partition_read(
            partition,
            base,
            actual.as_mut_ptr().cast(),
            actual.len()
        ))
    }
    .expect("read");
    let differing = actual.iter().zip(&expected).filter(|(a, b)| a != b).count();
    report("dense", "programs_per_page", 32.0, "programs");
    report("dense", "pages", PAGES as f32, "pages");
    report("dense", "bytes_differing", differing as f32, "bytes");
}

/// The writer's check before it uses a block: read 64 KB and confirm all 0xFF.
fn erased_check(partition: *const sys::esp_partition_t) {
    let mut buffer = vec![0u8; 64 << 10];
    let mut times = Vec::new();
    for block in 0..8 {
        let start = micros();
        // SAFETY: reads 64 KB inside the partition into a buffer of that length.
        unsafe {
            sys::esp!(sys::esp_partition_read(
                partition,
                REGION as usize + (REGION_SIZE as usize - (block + 1) * (64 << 10)),
                buffer.as_mut_ptr().cast(),
                buffer.len()
            ))
        }
        .expect("read");
        let _all_erased = buffer.iter().all(|&byte| byte == 0xFF);
        times.push((micros() - start) as f32 / 1000.0);
    }
    report_spread("erased_check", "read_and_scan_64kb", &mut times, "ms");
}

/// A deterministic byte pattern of `length` bytes from `pattern`, never all 0xFF.
fn fill(length: usize, pattern: u32) -> Vec<u8> {
    let mut state = pattern | 1;
    (0..length)
        .map(|_| {
            let byte = (next(&mut state) >> 24) as u8;
            if byte == 0xFF { 0x5A } else { byte }
        })
        .collect()
}

/// xorshift32.
fn next(state: &mut u32) -> u32 {
    *state ^= *state << 13;
    *state ^= *state >> 17;
    *state ^= *state << 5;
    *state
}

fn log_partition() -> *const sys::esp_partition_t {
    // SAFETY: a lookup in the static partition table.
    let partition = unsafe {
        sys::esp_partition_find_first(
            sys::esp_partition_type_t_ESP_PARTITION_TYPE_DATA,
            sys::esp_partition_subtype_t_ESP_PARTITION_SUBTYPE_ANY,
            c"log".as_ptr(),
        )
    };
    assert!(!partition.is_null(), "no `log` partition");
    partition
}

fn report(group: &str, name: &str, value: f32, unit: &str) {
    println!("ring;{group};{name};{value:.3};{unit}");
}

fn report_spread(group: &str, name: &str, values: &mut [f32], unit: &str) {
    values.sort_by(f32::total_cmp);
    let at = |per_mille: usize| values[(values.len() - 1) * per_mille / 1000];
    report(group, &format!("{name}_p50"), at(500), unit);
    report(group, &format!("{name}_p99"), at(990), unit);
    report(group, &format!("{name}_max"), at(1000), unit);
}

fn micros() -> i64 {
    // SAFETY: a plain read of the system timer.
    unsafe { sys::esp_timer_get_time() }
}
