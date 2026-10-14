//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! Peak throughput of the ESP32-P4 HP cores, the headline numbers: EEMBC CoreMark (Espressif's port, run as
//! shipped), f32 / f64 FLOPS, 32-bit integer OPS, the PIE SIMD int16 MAC rate through esp-dsp, and streaming
//! memory bandwidth -- each on one core, then on both cores at once. Results print as
//! `peak;<group>;<name>;<value>;<unit>`; CoreMark prints its own report first.
//!
//! The arithmetic kernels keep eight (or sixteen) independent accumulator chains, so the pipeline waits on its
//! own results as little as possible: that is throughput, unlike bench.rs, which times dependent (latency-bound)
//! operations. Eight f32 chains do not hide the FPU latency; sixteen reach ~1 FMA per 1.44 cycles.

use std::hint::black_box;
use std::sync::Arc;
use std::time::Duration;

use esp_idf_svc::hal::cpu::Core;
use esp_idf_svc::hal::task::thread::ThreadSpawnConfiguration;
use esp_idf_svc::sys;
use firmware::thermal::Thermometer;

unsafe extern "C" {
    /// EEMBC CoreMark's `main()`, renamed by components/coremark_entry.
    fn coremark_main() -> i32;
}

fn main() {
    esp_idf_svc::sys::link_patches();
    let thermometer = esp_idf_svc::hal::peripherals::Peripherals::take()
        .ok()
        .and_then(|peripherals| Thermometer::new(peripherals.temp_sensor).ok());
    let die = |when: &str| {
        let celsius = thermometer.as_ref().and_then(Thermometer::celsius).unwrap_or(f32::NAN);
        println!("peak;thermal;die_{when};{celsius:.1};C");
    };
    std::thread::sleep(Duration::from_millis(300));
    die("start");
    let hertz = cycle_hertz();
    report("meta", "cpu_clock", hertz as f32 / 1e6, "MHz");
    // SAFETY: CoreMark runs once, single-threaded, on this task; it only prints and returns.
    unsafe { coremark_main() };
    for (name, unit, kernel) in KERNELS {
        let one = run_on(&[Core::Core1], hertz, *kernel);
        let both = run_on(&[Core::Core0, Core::Core1], hertz, *kernel);
        report("one_core", name, one, unit);
        report("two_cores", name, both, unit);
    }
    die("end");
    println!("peak;done");
    loop {
        std::thread::sleep(Duration::from_secs(3600));
    }
}

fn report(group: &str, name: &str, value: f32, unit: &str) {
    println!("peak;{group};{name};{value:.1};{unit}");
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

/// A kernel: does its work and returns how many operations (FLOP, OP, MAC or bytes) it performed.
type Kernel = fn() -> u64;

/// (name, unit of the rate, kernel). Rates print in millions per second.
const KERNELS: &[(&str, &str, Kernel)] = &[
    ("f32_fma", "MFLOPS", f32_fma),
    ("f32_fma_16_chains", "MFLOPS", f32_fma_wide),
    ("i32_add_xor_16_chains", "MOPS", i32_add_xor_wide),
    ("f32_add", "MFLOPS", f32_add),
    ("f32_mul", "MFLOPS", f32_mul),
    ("f64_fma_software", "MFLOPS", f64_fma),
    ("i32_mul_add", "MOPS", i32_mul_add),
    ("i32_add_xor", "MOPS", i32_add_xor),
    ("int16_mac_pie_esp_dsp", "MMAC/s", int16_mac_pie),
    ("f32_dot_esp_dsp", "MFLOPS", f32_dot_dsp),
    ("read_internal_128k", "MB/s", read_internal),
    ("read_psram_8m", "MB/s", read_psram),
];

/// Run `kernel` on each of `cores` at the same time (one pinned thread each): the summed rate in millions/s.
fn run_on(cores: &[Core], hertz: u64, kernel: Kernel) -> f32 {
    let kernel = Arc::new(kernel);
    let handles: Vec<_> = cores
        .iter()
        .map(|core| {
            ThreadSpawnConfiguration {
                pin_to_core: Some(*core),
                priority: 10,
                ..Default::default()
            }
            .set()
            .expect("thread");
            let kernel = kernel.clone();
            std::thread::spawn(move || {
                kernel(); // warm the caches and take the FPU / PIE first-use trap outside the timing
                let start = cycles();
                let operations = kernel();
                (operations, cycles().wrapping_sub(start))
            })
        })
        .collect();
    ThreadSpawnConfiguration::default().set().expect("thread");
    handles
        .into_iter()
        .map(|handle| handle.join().unwrap())
        .map(|(operations, elapsed)| operations as f32 * hertz as f32 / elapsed as f32 / 1e6)
        .sum()
}

/// Eight independent f32 chains of `acc = acc * m + c` as fused multiply-adds: 2 FLOP each.
fn f32_fma() -> u64 {
    const ROUNDS: u64 = 1_000_000;
    let (multiplier, addend) = (black_box(0.999_999f32), black_box(1e-7f32));
    let mut chains = black_box([1.0f32, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7]);
    for _ in 0..ROUNDS {
        for chain in chains.iter_mut() {
            *chain = chain.mul_add(multiplier, addend);
        }
    }
    black_box(chains);
    ROUNDS * 8 * 2
}

/// Sixteen independent FMA chains: does doubling the chains hide more FPU latency?
fn f32_fma_wide() -> u64 {
    const ROUNDS: u64 = 500_000;
    let (multiplier, addend) = (black_box(0.999_999f32), black_box(1e-7f32));
    let mut chains: [f32; 16] = black_box(core::array::from_fn(|index| 1.0 + index as f32 * 0.1));
    for _ in 0..ROUNDS {
        for chain in chains.iter_mut() {
            *chain = chain.mul_add(multiplier, addend);
        }
    }
    black_box(chains);
    ROUNDS * 16 * 2
}

/// Sixteen independent integer chains (add, shift, xor): the integer issue rate with less loop overhead.
fn i32_add_xor_wide() -> u64 {
    const ROUNDS: u64 = 1_000_000;
    let key = black_box(0x9e37_79b9u32);
    let mut chains: [u32; 16] = black_box(core::array::from_fn(|index| index as u32 + 1));
    for _ in 0..ROUNDS {
        for chain in chains.iter_mut() {
            *chain = chain.wrapping_add(key) ^ (*chain >> 3);
        }
    }
    black_box(chains);
    ROUNDS * 16 * 3
}

/// Eight independent f32 add chains: 1 FLOP each.
fn f32_add() -> u64 {
    const ROUNDS: u64 = 1_000_000;
    let addend = black_box(1e-7f32);
    let mut chains = black_box([1.0f32, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7]);
    for _ in 0..ROUNDS {
        for chain in chains.iter_mut() {
            *chain += addend;
        }
    }
    black_box(chains);
    ROUNDS * 8
}

/// Eight independent f32 multiply chains: 1 FLOP each.
fn f32_mul() -> u64 {
    const ROUNDS: u64 = 1_000_000;
    let multiplier = black_box(0.999_999_9f32);
    let mut chains = black_box([1.0f32, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7]);
    for _ in 0..ROUNDS {
        for chain in chains.iter_mut() {
            *chain *= multiplier;
        }
    }
    black_box(chains);
    ROUNDS * 8
}

/// Eight independent f64 chains of mul + add (software floating point): 2 FLOP each. Only here to price the rule
/// "no f64 anywhere in the firmware".
fn f64_fma() -> u64 {
    const ROUNDS: u64 = 20_000;
    let (multiplier, addend) = (black_box(0.999_999f64), black_box(1e-7f64));
    let mut chains = black_box([1.0f64, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7]);
    for _ in 0..ROUNDS {
        for chain in chains.iter_mut() {
            *chain = *chain * multiplier + addend;
        }
    }
    black_box(chains);
    ROUNDS * 8 * 2
}

/// Eight independent 32-bit chains of `acc = acc * k + c`: 2 OP each.
fn i32_mul_add() -> u64 {
    const ROUNDS: u64 = 2_000_000;
    let (multiplier, addend) = (black_box(1_664_525u32), black_box(1_013_904_223u32));
    let mut chains = black_box([1u32, 2, 3, 4, 5, 6, 7, 8]);
    for _ in 0..ROUNDS {
        for chain in chains.iter_mut() {
            *chain = chain.wrapping_mul(multiplier).wrapping_add(addend);
        }
    }
    black_box(chains);
    ROUNDS * 8 * 2
}

/// Eight independent 32-bit chains of `acc = (acc + k) ^ (acc >> 3)`: 3 OP each (add, shift, xor).
fn i32_add_xor() -> u64 {
    const ROUNDS: u64 = 2_000_000;
    let key = black_box(0x9e37_79b9u32);
    let mut chains = black_box([1u32, 2, 3, 4, 5, 6, 7, 8]);
    for _ in 0..ROUNDS {
        for chain in chains.iter_mut() {
            *chain = chain.wrapping_add(key) ^ (*chain >> 3);
        }
    }
    black_box(chains);
    ROUNDS * 8 * 3
}

/// A buffer in `capabilities` memory, freed on drop.
struct Buffer(*mut u8);

impl Buffer {
    fn new(bytes: usize, capabilities: u32) -> Self {
        // SAFETY: a plain allocation, checked.
        let pointer = unsafe { sys::heap_caps_aligned_calloc(64, 1, bytes, capabilities) } as *mut u8;
        assert!(!pointer.is_null(), "allocation of {bytes} bytes failed");
        Self(pointer)
    }
}

impl Drop for Buffer {
    fn drop(&mut self) {
        // SAFETY: allocated above, freed once.
        unsafe { sys::heap_caps_free(self.0.cast()) };
    }
}

/// esp-dsp's PIE kernel (Q15 dot product, internal RAM): MACs.
fn int16_mac_pie() -> u64 {
    const LENGTH: usize = 2048;
    const REPEATS: u64 = 500;
    let internal = sys::MALLOC_CAP_INTERNAL | sys::MALLOC_CAP_8BIT;
    let (first, second) = (Buffer::new(LENGTH * 2, internal), Buffer::new(LENGTH * 2, internal));
    let mut result = 0i16;
    for _ in 0..REPEATS {
        // SAFETY: two live arrays of LENGTH i16 and one output.
        unsafe {
            sys::dsps_dotprod_s16_arp4(first.0.cast(), second.0.cast(), &mut result, LENGTH as i32, 0);
        }
    }
    black_box(result);
    REPEATS * LENGTH as u64
}

/// esp-dsp's P4 f32 dot product (hardware loops, FPU; internal RAM): 2 FLOP per element.
fn f32_dot_dsp() -> u64 {
    const LENGTH: usize = 2048;
    const REPEATS: u64 = 200;
    let internal = sys::MALLOC_CAP_INTERNAL | sys::MALLOC_CAP_8BIT;
    let (first, second) = (Buffer::new(LENGTH * 4, internal), Buffer::new(LENGTH * 4, internal));
    let mut result = 0.0f32;
    for _ in 0..REPEATS {
        // SAFETY: two live arrays of LENGTH f32 and one output.
        unsafe { sys::dsps_dotprod_f32_arp4(first.0.cast(), second.0.cast(), &mut result, LENGTH as i32) };
    }
    black_box(result);
    REPEATS * LENGTH as u64 * 2
}

/// Stream-read `bytes` of `capabilities` memory `repeats` times, four words at a time: bytes read.
fn read_stream(bytes: usize, repeats: u64, capabilities: u32) -> u64 {
    let buffer = Buffer::new(bytes, capabilities);
    let words = buffer.0.cast::<u32>();
    let mut sums = [0u32; 4];
    for _ in 0..repeats {
        for index in (0..bytes / 4).step_by(4) {
            // SAFETY: index + 3 < bytes / 4, inside the live buffer.
            unsafe {
                sums[0] = sums[0].wrapping_add(words.add(index).read_volatile());
                sums[1] = sums[1].wrapping_add(words.add(index + 1).read_volatile());
                sums[2] = sums[2].wrapping_add(words.add(index + 2).read_volatile());
                sums[3] = sums[3].wrapping_add(words.add(index + 3).read_volatile());
            }
        }
    }
    black_box(sums);
    bytes as u64 * repeats
}

fn read_internal() -> u64 {
    read_stream(128 << 10, 64, sys::MALLOC_CAP_INTERNAL | sys::MALLOC_CAP_8BIT)
}

fn read_psram() -> u64 {
    read_stream(8 << 20, 2, sys::MALLOC_CAP_SPIRAM | sys::MALLOC_CAP_8BIT)
}
