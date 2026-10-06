//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! B3b: PSRAM bandwidth as DMA sees it -- the camera, ISP and H.264 encoder move frames with DMA, not the CPU
//! (CPU copies are latency-bound: bench.rs B3, 57 MB/s for 1 MB). Times AXI-GDMA async memcpy over every route
//! at three burst sizes; prints `dma;<route>_burst<n>;<MB/s>;MB/s`.
//!
//! The completion callback runs in an ISR, so it only bumps an atomic counter -- integer only (a float in an ISR
//! aborts the chip).

use std::sync::atomic::{AtomicU32, Ordering};
use std::time::Duration;

use esp_idf_svc::sys;
use firmware::thermal::Thermometer;

fn main() {
    esp_idf_svc::sys::link_patches();
    let thermometer = esp_idf_svc::hal::peripherals::Peripherals::take()
        .ok()
        .and_then(|peripherals| Thermometer::new(peripherals.temp_sensor).ok());
    let die = |when: &str| {
        let celsius = thermometer.as_ref().and_then(Thermometer::celsius).unwrap_or(f32::NAN);
        println!("dma;thermal;die_{when};{celsius:.1};C");
    };
    std::thread::sleep(Duration::from_millis(300));
    die("start");
    let hertz = cycle_hertz();
    let psram = sys::MALLOC_CAP_SPIRAM | sys::MALLOC_CAP_8BIT;
    let internal = sys::MALLOC_CAP_INTERNAL | sys::MALLOC_CAP_DMA | sys::MALLOC_CAP_8BIT;
    for burst in [16usize, 32, 64, 128] {
        for (route, source_caps, target_caps, chunk, count) in [
            ("psram_to_psram", psram, psram, 1usize << 20, 16u32),
            ("internal_to_psram", internal, psram, 64 << 10, 64),
            ("psram_to_internal", psram, internal, 64 << 10, 64),
            ("internal_to_internal", internal, internal, 64 << 10, 64),
        ] {
            match copy_rate(hertz, burst, source_caps, target_caps, chunk, count, 1) {
                Some(rate) => println!("dma;{route}_burst{burst};{rate:.1};MB/s"),
                None => println!("dma;{route}_burst{burst};nan;failed"),
            }
        }
    }
    // is the ceiling the engine or the memory? several engines at once, PSRAM to PSRAM, burst 64
    for engines in [2usize, 3] {
        match copy_rate(hertz, 64, psram, psram, 1 << 20, 16, engines) {
            Some(rate) => println!("dma;psram_to_psram_burst64_engines{engines};{rate:.1};MB/s"),
            None => println!("dma;psram_to_psram_burst64_engines{engines};nan;failed"),
        }
    }
    die("end");
    println!("dma;done");
    loop {
        std::thread::sleep(Duration::from_secs(3600));
    }
}

#[inline(always)]
fn cycles() -> u32 {
    let count: u32;
    // SAFETY: reads the cycle CSR.
    unsafe { core::arch::asm!("rdcycle {0}", out(reg) count) };
    count
}

fn cycle_hertz() -> u64 {
    // SAFETY: plain timer reads.
    let (start_cycles, start_micros) = (cycles(), unsafe { sys::esp_timer_get_time() });
    while unsafe { sys::esp_timer_get_time() } - start_micros < 100_000 {}
    let elapsed = cycles().wrapping_sub(start_cycles) as u64;
    elapsed * 1_000_000 / (unsafe { sys::esp_timer_get_time() } - start_micros) as u64
}

/// Completion ISR: count the finished transaction.
unsafe extern "C" fn finished(
    _handle: sys::async_memcpy_handle_t,
    _event: *mut sys::async_memcpy_event_t,
    counter: *mut core::ffi::c_void,
) -> bool {
    // SAFETY: the counter outlives every transaction (the caller waits for all of them).
    unsafe { (*(counter as *const AtomicU32)).fetch_add(1, Ordering::Release) };
    false
}

/// MB/s summed over `engines` AXI-GDMA engines, each doing `count` back-to-back copies of `chunk` bytes between
/// its own buffers at the same time; None when an engine or a buffer is refused.
fn copy_rate(
    hertz: u64,
    burst: usize,
    source_caps: u32,
    target_caps: u32,
    chunk: usize,
    count: u32,
    engines: usize,
) -> Option<f32> {
    let config = sys::async_memcpy_config_t {
        backlog: 8,
        __bindgen_anon_1: sys::async_memcpy_config_t__bindgen_ty_1 { dma_burst_size: burst },
        ..Default::default()
    };
    let done: Vec<AtomicU32> = (0..engines).map(|_| AtomicU32::new(0)).collect();
    let mut handles = Vec::new();
    let mut buffers = Vec::new();
    // SAFETY: engines, buffers and counters are all released before returning, after every transaction finished.
    unsafe {
        let mut ready = true;
        for _ in 0..engines {
            let mut handle: sys::async_memcpy_handle_t = core::ptr::null_mut();
            if sys::esp_async_memcpy_install_gdma_axi(&config, &mut handle) != sys::ESP_OK {
                ready = false;
                break;
            }
            handles.push(handle);
            let source = sys::heap_caps_aligned_alloc(64, chunk, source_caps) as *mut u8;
            let target = sys::heap_caps_aligned_alloc(64, chunk, target_caps) as *mut u8;
            buffers.push((source, target));
            if source.is_null() || target.is_null() {
                ready = false;
                break;
            }
            core::ptr::write_bytes(source, 0x5a, chunk);
        }
        let mut rate = None;
        if ready {
            let mut issued = vec![0u32; engines];
            let start = cycles();
            while issued.iter().any(|sent| *sent < count) {
                for engine in 0..engines {
                    let in_flight = issued[engine] - done[engine].load(Ordering::Acquire);
                    if issued[engine] < count && in_flight < 8 {
                        let (source, target) = buffers[engine];
                        let counter = &done[engine] as *const AtomicU32 as *mut core::ffi::c_void;
                        if sys::esp_async_memcpy(
                            handles[engine],
                            target.cast(),
                            source.cast(),
                            chunk,
                            Some(finished),
                            counter,
                        ) == sys::ESP_OK
                        {
                            issued[engine] += 1;
                        }
                    }
                }
            }
            while done.iter().any(|finished| finished.load(Ordering::Acquire) < count) {}
            let elapsed = cycles().wrapping_sub(start);
            let intact = buffers
                .iter()
                .all(|(_, target)| **target == 0x5a && *target.add(chunk - 1) == 0x5a);
            if intact {
                let bytes = chunk as f32 * count as f32 * engines as f32;
                rate = Some(bytes / (elapsed as f32 / hertz as f32) / 1e6);
            }
        }
        for (source, target) in buffers {
            sys::heap_caps_free(source.cast());
            sys::heap_caps_free(target.cast());
        }
        for handle in handles {
            sys::esp_async_memcpy_uninstall(handle);
        }
        rate
    }
}
