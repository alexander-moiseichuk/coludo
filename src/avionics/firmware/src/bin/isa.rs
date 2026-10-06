//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! What the ESP32-P4 HP core really implements, and what Rust gets out of it: ISA extensions, atomics and
//! locks (internal RAM and PSRAM, across cores), and the PIE SIMD / hardware-loop kernels of esp-dsp.
//! Results print as `isa;<group>;<name>;<value>;<unit>`.
//!
//! Each extension probe executes one instruction by its raw encoding (so the assembler needs no feature
//! flag) and checks the result. An unimplemented one traps and the chip reboots; a crumb in RTC no-init
//! memory names the probe that was running, and the next boot reports it as `trapped` and carries on.

use std::hint::black_box;
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use esp_idf_svc::hal::cpu::Core;
use esp_idf_svc::hal::task::thread::ThreadSpawnConfiguration;
use esp_idf_svc::sys;

/// [magic, 1 + index of the probe running]: survives a panic reset in RTC memory.
#[unsafe(link_section = ".rtc_noinit")]
static mut CRUMB: [u32; 2] = [0; 2];
/// Marks the crumb as written by this program, not left-over RTC noise.
const MAGIC: u32 = 0xC01D_15A0;

/// One extension probe: its name and the check (true = the instruction ran and gave the right answer).
type Probe = (&'static str, fn() -> bool);

const PROBES: &[Probe] = &[
    ("zcb_c_zext_b", zcb),
    ("zcmp_push_pop", zcmp),
    ("zba_sh1add", zba),
    ("zbb_cpop", zbb),
    ("zbs_bset", zbs),
    ("zbc_clmul", zbc),
    ("zicond_czero_eqz", zicond),
];

fn main() {
    esp_idf_svc::sys::link_patches();
    std::thread::sleep(Duration::from_millis(300));
    identity();
    // SAFETY: the crumb is touched by this thread only.
    let resume = unsafe { if CRUMB[0] == MAGIC { CRUMB[1] as usize } else { 0 } };
    if resume > 0 {
        report("extension", PROBES[resume - 1].0, 0.0, "trapped");
    }
    for (index, (name, probe)) in PROBES.iter().enumerate().skip(resume) {
        // SAFETY: as above.
        unsafe { CRUMB = [MAGIC, index as u32 + 1] };
        let correct = probe();
        report(
            "extension",
            name,
            correct as u8 as f64,
            if correct { "supported" } else { "wrong-result" },
        );
    }
    // SAFETY: as above.
    unsafe { CRUMB = [0, 0] };
    let hertz = cycle_hertz();
    atomics(hertz);
    locks(hertz);
    dsp(hertz);
    println!("isa;done");
    loop {
        std::thread::sleep(Duration::from_secs(3600));
    }
}

fn report(group: &str, name: &str, value: f64, unit: &str) {
    println!("isa;{group};{name};{value:.3};{unit}");
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

/// The machine ID CSRs and `misa` (the single-letter extensions).
fn identity() {
    let (isa, vendor, architecture, implementation): (u32, u32, u32, u32);
    // SAFETY: reads of machine information CSRs; IDF runs the app in machine mode.
    unsafe {
        core::arch::asm!("csrr {0}, misa", out(reg) isa);
        core::arch::asm!("csrr {0}, mvendorid", out(reg) vendor);
        core::arch::asm!("csrr {0}, marchid", out(reg) architecture);
        core::arch::asm!("csrr {0}, mimpid", out(reg) implementation);
    }
    let letters: String = (0..26)
        .filter(|bit| isa & (1 << bit) != 0)
        .map(|bit| (b'a' + bit as u8) as char)
        .collect();
    println!("isa;identity;misa;0x{isa:08x};{letters}");
    println!("isa;identity;mvendorid;0x{vendor:08x};-");
    println!("isa;identity;marchid;0x{architecture:08x};-");
    println!("isa;identity;mimpid;0x{implementation:08x};-");
}

/// Zcb `c.zext.b a0` (0x9d61): 0x1234 -> 0x34.
fn zcb() -> bool {
    let mut value: u32 = black_box(0x1234);
    // SAFETY: one register-only instruction on a0.
    unsafe { core::arch::asm!(".2byte 0x9d61", inout("a0") value) };
    value == 0x34
}

/// Zcmp `cm.push {ra}, -16` (0xb842) then `cm.pop {ra}, 16` (0xba42): sp comes back where it was.
fn zcmp() -> bool {
    let (before, after): (u32, u32);
    // SAFETY: the pair stores ra below sp and reloads it; sp and ra end where they started.
    unsafe {
        core::arch::asm!("mv {0}, sp", ".2byte 0xb842", ".2byte 0xba42", "mv {1}, sp",
            out(reg) before, out(reg) after, out("ra") _)
    };
    before == after
}

/// Run one R-type instruction (raw `word`) as `a0 = op(a0, a1)`.
macro_rules! r_type {
    ($word:literal, $first:expr, $second:expr) => {{
        let mut first: u32 = black_box($first);
        let second: u32 = black_box($second);
        // SAFETY: one register-only instruction on a0, a1.
        unsafe { core::arch::asm!(concat!(".4byte ", $word), inout("a0") first, in("a1") second) };
        first
    }};
}

/// Zba `sh1add a0, a0, a1`: (5 << 1) + 100.
fn zba() -> bool {
    r_type!("0x20b52533", 5, 100) == 110
}

/// Zbb `cpop a0, a0`: popcount(0xf0f0) = 8.
fn zbb() -> bool {
    r_type!("0x60251513", 0xf0f0, 0) == 8
}

/// Zbs `bset a0, a0, a1`: 0 | 1 << 5.
fn zbs() -> bool {
    r_type!("0x28b51533", 0, 5) == 32
}

/// Zbc `clmul a0, a0, a1`: 0b11 clmul 0b11 = 0b101.
fn zbc() -> bool {
    r_type!("0x0ab51533", 3, 3) == 5
}

/// Zicond `czero.eqz a0, a0, a1`: a1 == 0 -> 0.
fn zicond() -> bool {
    r_type!("0x0eb55533", 42, 0) == 0
}

/// Spawn the next std thread pinned to `core`.
fn pin_next_thread(core: Core) {
    ThreadSpawnConfiguration {
        pin_to_core: Some(core),
        priority: 10,
        ..Default::default()
    }
    .set()
    .expect("thread");
}

/// A 64-byte-aligned block of `capabilities` memory holding a counter and a lock word, freed on drop.
struct Shared {
    pointer: *mut u32,
}

unsafe impl Send for Shared {}
unsafe impl Sync for Shared {}

impl Shared {
    fn new(capabilities: u32) -> Self {
        // SAFETY: a plain allocation, checked, zeroed.
        let pointer = unsafe { sys::heap_caps_aligned_calloc(64, 1, 64, capabilities) } as *mut u32;
        assert!(!pointer.is_null());
        Self { pointer }
    }

    /// The atomic word at `index`.
    fn atomic(&self, index: usize) -> &AtomicU32 {
        // SAFETY: inside the 64-byte block, aligned, and only ever used atomically or under the lock.
        unsafe { AtomicU32::from_ptr(self.pointer.add(index)) }
    }

    /// The plain word at `index` (guarded by the spin lock).
    fn plain(&self, index: usize) -> *mut u32 {
        // SAFETY: inside the block.
        unsafe { self.pointer.add(index) }
    }
}

impl Drop for Shared {
    fn drop(&mut self) {
        // SAFETY: allocated by heap_caps_aligned_calloc, freed once.
        unsafe { sys::heap_caps_free(self.pointer.cast()) };
    }
}

/// Run `body` on both cores at once; returns the slower side's cycles.
fn both_cores(body: impl Fn() + Send + Sync + 'static) -> u32 {
    let body = Arc::new(body);
    let handles: Vec<_> = [Core::Core0, Core::Core1]
        .into_iter()
        .map(|core| {
            pin_next_thread(core);
            let body = body.clone();
            std::thread::spawn(move || {
                let start = cycles();
                body();
                cycles().wrapping_sub(start)
            })
        })
        .collect();
    ThreadSpawnConfiguration::default().set().expect("thread");
    handles.into_iter().map(|handle| handle.join().unwrap()).max().unwrap()
}

/// Atomics on internal RAM and on PSRAM, from both cores at once: AMO (`fetch_add` = amoadd.w) and LR/SC
/// (`compare_exchange` spin lock around a plain counter). The totals must come out exact.
fn atomics(hertz: u64) {
    const COUNT: u32 = 200_000;
    let nanoseconds = |cycles: u32, operations: u32| cycles as f64 * 1e9 / hertz as f64 / operations as f64;
    for (capabilities, label) in [
        (sys::MALLOC_CAP_INTERNAL | sys::MALLOC_CAP_8BIT, "internal"),
        (sys::MALLOC_CAP_SPIRAM | sys::MALLOC_CAP_8BIT, "psram"),
    ] {
        let shared = Arc::new(Shared::new(capabilities));
        let counter = shared.clone();
        let elapsed = both_cores(move || {
            for _ in 0..COUNT {
                counter.atomic(0).fetch_add(1, Ordering::AcqRel);
            }
        });
        let total = shared.atomic(0).load(Ordering::Acquire);
        report(
            "atomics",
            &format!("fetch_add_two_cores_{label}_exact"),
            (total == 2 * COUNT) as u8 as f64,
            "bool",
        );
        report(
            "atomics",
            &format!("fetch_add_two_cores_{label}"),
            nanoseconds(elapsed, COUNT),
            "ns/op",
        );

        let locked = shared.clone();
        let elapsed = both_cores(move || {
            for _ in 0..COUNT {
                while locked
                    .atomic(1)
                    .compare_exchange_weak(0, 1, Ordering::Acquire, Ordering::Relaxed)
                    .is_err()
                {
                    core::hint::spin_loop();
                }
                // SAFETY: the spin lock (word 1) is held.
                unsafe { *locked.plain(2) += 1 };
                locked.atomic(1).store(0, Ordering::Release);
            }
        });
        // SAFETY: both threads have joined.
        let total = unsafe { *shared.plain(2) };
        report(
            "atomics",
            &format!("spinlock_two_cores_{label}_exact"),
            (total == 2 * COUNT) as u8 as f64,
            "bool",
        );
        report(
            "atomics",
            &format!("spinlock_two_cores_{label}"),
            nanoseconds(elapsed, COUNT),
            "ns/op",
        );
    }
    let single = AtomicU32::new(0);
    let start = cycles();
    for _ in 0..COUNT {
        single.fetch_add(1, Ordering::AcqRel);
    }
    report(
        "atomics",
        "fetch_add_uncontended_internal",
        nanoseconds(cycles().wrapping_sub(start), COUNT),
        "ns/op",
    );
}

/// std Mutex (a FreeRTOS mutex with priority inheritance underneath): uncontended and contended.
fn locks(hertz: u64) {
    const COUNT: u32 = 100_000;
    let nanoseconds = |cycles: u32, operations: u32| cycles as f64 * 1e9 / hertz as f64 / operations as f64;
    let mutex = Mutex::new(0u32);
    let start = cycles();
    for _ in 0..COUNT {
        *mutex.lock().unwrap() += 1;
    }
    report(
        "locks",
        "mutex_uncontended",
        nanoseconds(cycles().wrapping_sub(start), COUNT),
        "ns/op",
    );
    let shared = Arc::new(Mutex::new(0u32));
    let counter = shared.clone();
    let elapsed = both_cores(move || {
        for _ in 0..COUNT {
            *counter.lock().unwrap() += 1;
        }
    });
    report(
        "locks",
        "mutex_two_cores_exact",
        (*shared.lock().unwrap() == 2 * COUNT) as u8 as f64,
        "bool",
    );
    report("locks", "mutex_two_cores", nanoseconds(elapsed, COUNT), "ns/op");

    // where the std Mutex time goes: the raw FreeRTOS mutex it sits on (priority inheritance, no pthread layer)
    // SAFETY: a FreeRTOS mutex created, taken/given by this task only, and leaked (one per run).
    unsafe {
        let semaphore = sys::xQueueCreateMutex(1); // queueQUEUE_TYPE_MUTEX (a macro, not bound)
        let start = cycles();
        for _ in 0..COUNT {
            sys::xQueueSemaphoreTake(semaphore, u32::MAX);
            sys::xQueueGenericSend(semaphore, core::ptr::null(), 0, 0); // queueSEND_TO_BACK
        }
        report(
            "locks",
            "freertos_mutex_uncontended",
            nanoseconds(cycles().wrapping_sub(start), COUNT),
            "ns/op",
        );
    }
    // the cross-core critical section (portMUX spinlock + this core's interrupts off): for a few-instruction
    // section shared with an ISR or the other core
    let section = esp_idf_svc::hal::interrupt::IsrCriticalSection::new();
    let mut value = 0u32;
    let start = cycles();
    for _ in 0..COUNT {
        let _guard = section.enter();
        value = black_box(value + 1);
    }
    report(
        "locks",
        "isr_critical_section",
        nanoseconds(cycles().wrapping_sub(start), COUNT),
        "ns/op",
    );
}

/// Best-of-5 cycles of `body`, in microseconds.
fn time_micros(hertz: u64, mut body: impl FnMut()) -> f64 {
    let mut best = u32::MAX;
    for _ in 0..5 {
        let start = cycles();
        body();
        best = best.min(cycles().wrapping_sub(start));
    }
    best as f64 * 1e6 / hertz as f64
}

/// esp-dsp's P4 kernels (PIE SIMD + hardware loops, `_arp4`) against its portable C (`_ansi`) and plain Rust.
fn dsp(hertz: u64) {
    const LENGTH: usize = 1024;
    let first: Vec<f32> = (0..LENGTH).map(|index| (index as f32 * 0.01).sin()).collect();
    let second: Vec<f32> = (0..LENGTH).map(|index| (index as f32 * 0.02).cos()).collect();
    let mut result = 0.0f32;
    let rust = time_micros(hertz, || {
        result = black_box(first.iter().zip(&second).map(|(a, b)| a * b).sum::<f32>());
    });
    let expected = result;
    report("dsp", "dotprod_f32_1024_rust", rust, "us");
    for (name, kernel) in [
        (
            "dotprod_f32_1024_ansi",
            sys::dsps_dotprod_f32_ansi as unsafe extern "C" fn(_, _, _, _) -> _,
        ),
        ("dotprod_f32_1024_arp4", sys::dsps_dotprod_f32_arp4),
    ] {
        let micros = time_micros(hertz, || {
            result = 0.0;
            // SAFETY: two live arrays of LENGTH floats and one output.
            unsafe { kernel(first.as_ptr(), second.as_ptr(), &mut result, LENGTH as i32) };
        });
        report("dsp", name, micros, "us");
        report(
            "dsp",
            &format!("{name}_agrees"),
            ((result - expected).abs() < 1e-2) as u8 as f64,
            "bool",
        );
    }

    let first: Vec<i16> = (0..LENGTH).map(|index| ((index * 37) % 2000) as i16 - 1000).collect();
    let second: Vec<i16> = (0..LENGTH).map(|index| ((index * 91) % 2000) as i16 - 1000).collect();
    let mut wide = 0i64;
    let rust = time_micros(hertz, || {
        wide = black_box(first.iter().zip(&second).map(|(a, b)| *a as i64 * *b as i64).sum());
    });
    report("dsp", "dotprod_s16_1024_rust", rust, "us");
    // Q15: dest = sum(src1 * src2) >> (15 - shift); shift 0 keeps the sum in i16 range for these inputs
    let mut reference = 0i16;
    for (name, kernel) in [
        (
            "dotprod_s16_1024_ansi",
            sys::dsps_dotprod_s16_ansi as unsafe extern "C" fn(_, _, _, _, _) -> _,
        ),
        ("dotprod_s16_1024_arp4", sys::dsps_dotprod_s16_arp4),
    ] {
        let mut narrow = 0i16;
        let micros = time_micros(hertz, || {
            narrow = 0;
            // SAFETY: two live arrays of LENGTH i16 and one output.
            unsafe { kernel(first.as_ptr(), second.as_ptr(), &mut narrow, LENGTH as i32, 0) };
        });
        report("dsp", name, micros, "us");
        if name.ends_with("ansi") {
            reference = narrow; // the portable C is the reference the SIMD kernel must reproduce
            report(
                "dsp",
                "dotprod_s16_ansi_vs_rust_q15",
                (narrow as i64 - (wide >> 15)) as f64,
                "lsb",
            );
        } else {
            report(
                "dsp",
                &format!("{name}_agrees_with_ansi"),
                ((narrow - reference).abs() <= 1) as u8 as f64,
                "bool",
            );
        }
    }

    // a 2nd-order low-pass over 1024 samples: the sensor-filter workload
    let input: Vec<f32> = (0..LENGTH).map(|index| (index as f32 * 0.3).sin()).collect();
    let mut output = vec![0.0f32; LENGTH];
    for (name, kernel) in [
        (
            "biquad_f32_1024_ansi",
            sys::dsps_biquad_f32_ansi as unsafe extern "C" fn(_, _, _, _, _) -> _,
        ),
        ("biquad_f32_1024_arp4", sys::dsps_biquad_f32_arp4),
    ] {
        let mut coefficients = [0.0675f32, 0.1349, 0.0675, -1.1430, 0.4128];
        let mut state = [0.0f32; 2];
        let micros = time_micros(hertz, || {
            state = [0.0; 2];
            // SAFETY: live input/output of LENGTH samples, 5 coefficients, 2 state words.
            unsafe {
                kernel(
                    input.as_ptr(),
                    output.as_mut_ptr(),
                    LENGTH as i32,
                    coefficients.as_mut_ptr(),
                    state.as_mut_ptr(),
                )
            };
        });
        report("dsp", name, micros, "us");
    }

    // 4x4 matrix product: the attitude/navigation workload
    let a: Vec<f32> = (0..16).map(|index| index as f32 * 0.5).collect();
    let b: Vec<f32> = (0..16).map(|index| 1.0 - index as f32 * 0.25).collect();
    let mut c = [0.0f32; 16];
    let rust = time_micros(hertz, || {
        for row in 0..4 {
            for column in 0..4 {
                c[row * 4 + column] = (0..4).map(|inner| a[row * 4 + inner] * b[inner * 4 + column]).sum();
            }
        }
        black_box(&c);
    });
    report("dsp", "matrix_4x4_rust", rust, "us");
    for (name, kernel) in [
        (
            "matrix_4x4_ansi",
            sys::dspm_mult_f32_ansi as unsafe extern "C" fn(_, _, _, _, _, _) -> _,
        ),
        ("matrix_4x4_arp4", sys::dspm_mult_f32_arp4),
    ] {
        let micros = time_micros(hertz, || {
            // SAFETY: 4x4 inputs and output.
            unsafe { kernel(a.as_ptr(), b.as_ptr(), c.as_mut_ptr(), 4, 4, 4) };
        });
        report("dsp", name, micros, "us");
    }
}
