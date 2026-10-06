//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! Our own flight code, case by case, against MicroPython: each case is a faithful port of the src/glider
//! function it is named after, with the same inputs as src/glider/test/bench_cases.py, which times the real
//! MicroPython modules. Built once per rustc flag set (doc/benches/avionics-20261005/README.md), so the
//! table compares languages AND optimisation levels. Prints `cases;<name>;<nanoseconds per call>;ns`.
//!
//! Timing is best-of-5 runs (the one fewest interrupts landed in), loop and call included -- on the
//! MicroPython side the call overhead IS part of what a call costs, so it is included there too.

use std::fmt::Write as _;
use std::hint::black_box;
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
        println!("cases;thermal;die_{when};{celsius:.1};C");
    };
    std::thread::sleep(Duration::from_millis(300));
    die("start");
    let hertz = cycle_hertz();
    println!(
        "cases;meta;variant;{};-",
        option_env!("CASES_VARIANT").unwrap_or("release")
    );
    checks();
    primitives(hertz);
    control(hertz);
    navigation(hertz);
    recorder(hertz);
    infrastructure(hertz);
    die("end");
    println!("cases;done");
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

/// Nanoseconds per call of `body` (best of 5 runs of `count`), printed as a case line.
fn case(hertz: u64, name: &str, count: u32, mut body: impl FnMut()) {
    let mut best = u32::MAX;
    for _ in 0..5 {
        let start = cycles();
        for _ in 0..count {
            body();
        }
        best = best.min(cycles().wrapping_sub(start));
    }
    println!("cases;{name};{:.1};ns", best as f32 * 1e9 / hertz as f32 / count as f32);
}

/// The same inputs through fresh objects, printed as `check;<name>;<value>` -- bench_cases.py prints the
/// same lines from the real MicroPython modules, so the port is proven equivalent on the board.
fn checks() {
    let check = |name: &str, value: String| println!("check;{name};{value}");
    check("clamp_int", clamp_int(0, 5, 10).to_string());
    check("wrap180", wrap180(200).to_string());
    check("bank_demand", format!("{:.3}", bank_demand(90, 1.5, 30.0)));
    check("sensirion_crc8", sensirion_crc8(0xbe, 0xef).to_string());
    check("atan2_cd", atan2_cd(-3000, -4000).to_string());
    let mut text = String::new();
    to_str(-12345, &mut text);
    check("to_str", text);
    let mut roll = Pid::new(1.5, 0.2, 0.1, 20, 30);
    let outputs: Vec<String> = (0..5)
        .map(|step| roll.step(500 + step * 900, 10, Some(-300)).to_string())
        .collect();
    check("pid_steps", outputs.join(","));
    let mut mixer = Mixer::new();
    mixer.actuate(50, -30, 10);
    let angles: Vec<String> = mixer.fins.iter().map(|fin| (fin.0.angle()).to_string()).collect();
    check("mixer_angles", angles.join(","));
    let table = fin_limit_table();
    check(
        "fin_limits",
        [0.0f32, 5.0, 15.0, 30.0, 90.0]
            .map(|speed| fin_deflection_limit(&table, speed).to_string())
            .join(","),
    );
    let (top_left, bottom_right, position) = (
        (48.1010f32, 11.5000f32),
        (48.1000f32, 11.5027f32),
        (48.1050f32, 11.4950f32),
    );
    let (bearing, waypoint, on_target) = geometry32::steer(position, top_left, bottom_right);
    check(
        "steer_f32",
        format!("{bearing:.2},{:.5},{:.5},{}", waypoint.0, waypoint.1, on_target as u8),
    );
    let zone = geometry32::zone(top_left, bottom_right);
    check(
        "approach_f32",
        format!("{:.2}", geometry32::approach_to(position, zone, 120.0, 0.5, 30.0)),
    );
    let mut row = String::from("@000135_flight@123456789");
    for field in 0..19 {
        let _ = write!(row, ";{}", field * 137 - 900);
    }
    row.push('\n');
    let mut line = [0u8; 320];
    let size = wrap(row.as_bytes(), &mut line, &crc_table()).unwrap();
    check("recorder_wrap", String::from_utf8_lossy(&line[..size - 1]).into_owned());
}

// ---- commons.py / fixed.py ----------------------------------------------------------------------------

/// commons.clamp_int
fn clamp_int(low: i32, value: i32, high: i32) -> i32 {
    if value < low {
        low
    } else if value > high {
        high
    } else {
        value
    }
}

/// commons.wrap180 (Python `%`: the result takes the divisor's sign)
fn wrap180(degrees: i32) -> i32 {
    if (-180..=180).contains(&degrees) {
        degrees
    } else {
        (degrees + 180).rem_euclid(360) - 180
    }
}

/// commons.between
fn between(low: f32, value: f32, high: f32) -> f32 {
    if value < low {
        low
    } else if value > high {
        high
    } else {
        value
    }
}

/// commons.magnitude_sq
fn magnitude_sq(x: f32, y: f32, z: f32) -> f32 {
    x * x + y * y + z * z
}

/// commons.bank_demand
fn bank_demand(heading_error: i32, gain: f32, limit: f32) -> f32 {
    between(-limit, gain * heading_error as f32, limit)
}

/// commons.sensirion_crc8 (polynomial 0x31, seed 0xFF)
fn sensirion_crc8(byte0: u8, byte1: u8) -> u8 {
    let mut crc = 0xffu8;
    for current in [byte0, byte1] {
        crc ^= current;
        for _ in 0..8 {
            crc = if crc & 0x80 != 0 { (crc << 1) ^ 0x31 } else { crc << 1 };
        }
    }
    crc
}

/// commons._FIN_LIMIT: max fin deflection (degrees) per whole m/s of airspeed, 0..=80
fn fin_limit_table() -> [i32; 81] {
    core::array::from_fn(|speed| {
        if speed == 0 {
            45
        } else {
            (12500.0 / (speed * speed) as f32).round().clamp(5.0, 45.0) as i32
        }
    })
}

/// commons.fin_deflection_limit
fn fin_deflection_limit(table: &[i32; 81], speed_ms: f32) -> i32 {
    table[(speed_ms as i32).clamp(0, 80) as usize]
}

/// fixed._ATAN_CD: atan(2^-i) in centidegrees
const ATAN_CD: [i32; 12] = [4500, 2657, 1404, 713, 358, 179, 90, 45, 22, 11, 6, 3];
/// fixed._NORM_MIN
const NORM_MIN: i32 = 1 << 14;

/// fixed._cordic_vec
fn cordic_vec(mut y: i32, mut x: i32) -> i32 {
    if x != 0 || y != 0 {
        while x < NORM_MIN && -NORM_MIN < y && y < NORM_MIN {
            x <<= 1;
            y <<= 1;
        }
    }
    let mut z = 0;
    for (shift, atan) in ATAN_CD.iter().enumerate() {
        if y < 0 {
            (x, y, z) = (x - (y >> shift), y + (x >> shift), z - atan);
        } else {
            (x, y, z) = (x + (y >> shift), y - (x >> shift), z + atan);
        }
    }
    z
}

/// fixed.atan2_cd
fn atan2_cd(y: i32, x: i32) -> i32 {
    if x >= 0 {
        return if x != 0 || y != 0 { cordic_vec(y, x) } else { 0 };
    }
    let base = cordic_vec(y, -x);
    if y >= 0 { 18000 - base } else { -18000 - base }
}

/// fixed.to_str, into a reused buffer
fn to_str(scaled: i32, out: &mut String) {
    out.clear();
    let magnitude = scaled.unsigned_abs();
    let _ = write!(
        out,
        "{}{}.{:02}",
        if scaled < 0 { "-" } else { "" },
        magnitude / 100,
        magnitude % 100
    );
}

fn primitives(hertz: u64) {
    case(hertz, "clamp_int", 100_000, || {
        black_box(clamp_int(black_box(0), black_box(5), black_box(10)));
    });
    case(hertz, "wrap180", 100_000, || {
        black_box(wrap180(black_box(200)));
    });
    case(hertz, "between", 100_000, || {
        black_box(between(black_box(0.0), black_box(5.0), black_box(10.0)));
    });
    case(hertz, "magnitude_sq", 100_000, || {
        black_box(magnitude_sq(black_box(3.0), black_box(4.0), black_box(12.0)));
    });
    case(hertz, "bank_demand", 100_000, || {
        black_box(bank_demand(black_box(90), black_box(1.5), black_box(30.0)));
    });
    case(hertz, "sensirion_crc8", 100_000, || {
        black_box(sensirion_crc8(black_box(0xbe), black_box(0xef)));
    });
    case(hertz, "atan2_cd", 50_000, || {
        black_box(atan2_cd(black_box(-3000), black_box(-4000)));
    });
    let mut text = String::with_capacity(16);
    case(hertz, "to_str", 50_000, || {
        to_str(black_box(-12345), &mut text);
        black_box(&text);
    });
}

// ---- pid.py / mixer.py / the control step -----------------------------------------------------------------

/// pid._KU: gain scale
const KU: i32 = 100;

/// pid.Pid: fixed-point, error in centidegrees, gains ×100, derivative on measurement (gyro rate)
struct Pid {
    kp: i32,
    ki: i32,
    kd: i32,
    anti_windup_shift: u32,
    output_limit: i32,
    integral_limit_ms: i32,
    integral: i32,
    integral_ms: i32,
    previous: Option<i32>,
}

impl Pid {
    fn new(kp: f32, ki: f32, kd: f32, integral_limit: i32, output_limit: i32) -> Self {
        Self {
            kp: (kp * KU as f32) as i32,
            ki: (ki * KU as f32) as i32,
            kd: (kd * KU as f32) as i32,
            anti_windup_shift: 2,
            output_limit: output_limit * 100,
            integral_limit_ms: (integral_limit * 100 * 1000).min(268_435_456),
            integral: 0,
            integral_ms: 0,
            previous: None,
        }
    }

    /// pid.Pid.step (Python floor division -> div_euclid for the signed divisions)
    fn step(&mut self, error: i32, dt_ms: i32, rate: Option<i32>) -> i32 {
        let accumulated = clamp_int(
            -self.integral_limit_ms,
            self.integral_ms + error * dt_ms,
            self.integral_limit_ms,
        );
        self.integral_ms = accumulated;
        let mut integral = accumulated.div_euclid(1000);
        self.integral = integral;
        let derivative = match (rate, self.previous) {
            (Some(rate), _) => -rate,
            (None, Some(previous)) if dt_ms > 0 => ((error - previous) * 1000).div_euclid(dt_ms),
            _ => 0,
        };
        self.previous = Some(error);
        let output = (self.kp * error + self.ki * integral + self.kd * derivative).div_euclid(KU);
        let limited = clamp_int(-self.output_limit, output, self.output_limit);
        if limited != output && self.ki != 0 {
            let unwound = integral - (((output - limited) * KU).div_euclid(self.ki) >> self.anti_windup_shift);
            integral = if integral > 0 { unwound.max(0) } else { unwound.min(0) };
            self.integral = integral;
            self.integral_ms = integral * 1000;
        }
        limited
    }
}

/// The PID as Rust would write it: plain f32 degrees and seconds, the same law (derivative on the measured
/// rate, clamped integral, back-calculation anti-windup). fixed.py's centidegree integers exist only to dodge
/// MicroPython's boxed floats with the GC off in flight; Rust has neither problem.
struct PidF32 {
    kp: f32,
    ki: f32,
    kd: f32,
    integral_limit: f32,
    output_limit: f32,
    integral: f32,
}

impl PidF32 {
    fn new(kp: f32, ki: f32, kd: f32, integral_limit: f32, output_limit: f32) -> Self {
        Self {
            kp,
            ki,
            kd,
            integral_limit,
            output_limit,
            integral: 0.0,
        }
    }

    /// error in degrees, dt in seconds, rate in degrees per second
    fn step(&mut self, error: f32, dt: f32, rate: f32) -> f32 {
        self.integral = (self.integral + error * dt).clamp(-self.integral_limit, self.integral_limit);
        let output = self.kp * error + self.ki * self.integral - self.kd * rate;
        let limited = output.clamp(-self.output_limit, self.output_limit);
        if limited != output && self.ki != 0.0 {
            let unwound = self.integral - (output - limited) / self.ki * 0.25;
            self.integral = if self.integral > 0.0 {
                unwound.max(0.0)
            } else {
                unwound.min(0.0)
            };
        }
        limited
    }
}

/// A servo the mixer drives (bench_flight's FakeFin: it stores the angle).
trait Fin {
    fn set_angle(&mut self, angle: i32);
    fn settle(&mut self);
    fn angle(&self) -> i32;
}

struct FakeFin {
    angle: i32,
}

impl Fin for FakeFin {
    fn set_angle(&mut self, angle: i32) {
        self.angle = black_box(angle);
    }
    fn settle(&mut self) {}
    fn angle(&self) -> i32 {
        self.angle
    }
}

/// mixer.Mixer after bind(): (fin, base, roll gain, pitch gain, yaw gain) per surface
struct Mixer {
    limit: i32,
    fins: Vec<(Box<dyn Fin>, i32, i32, i32, i32)>,
}

impl Mixer {
    /// The default surfaces: rudder (yaw), two elevons (pitch, ±roll); neutral 90.
    fn new() -> Self {
        let fin = || -> Box<dyn Fin> { Box::new(FakeFin { angle: 90 }) };
        Self {
            limit: 45,
            fins: vec![(fin(), 90, 0, 0, 1), (fin(), 90, 1, 1, 0), (fin(), 90, -1, 1, 0)],
        }
    }

    /// mixer.Mixer.actuate (between() runs on the integer sum, as in Python)
    fn actuate(&mut self, roll: i32, pitch: i32, yaw: i32) {
        let limit = self.limit;
        for (fin, base, roll_gain, pitch_gain, yaw_gain) in self.fins.iter_mut() {
            fin.set_angle(*base + clamp_int(-limit, *roll_gain * roll + *pitch_gain * pitch + *yaw_gain * yaw, limit));
            fin.settle();
        }
    }
}

fn control(hertz: u64) {
    let mut roll = Pid::new(1.5, 0.2, 0.1, 20, 30);
    let mut pitch = Pid::new(1.5, 0.2, 0.1, 20, 30);
    let mut yaw = Pid::new(1.0, 0.0, 0.05, 20, 30);
    case(hertz, "pid_step", 50_000, || {
        black_box(roll.step(black_box(500), 10, Some(black_box(-300))));
    });
    case(hertz, "pid_step_x3", 20_000, || {
        black_box(roll.step(black_box(500), 10, Some(black_box(-300))));
        black_box(pitch.step(black_box(-250), 10, Some(black_box(120))));
        black_box(yaw.step(black_box(80), 10, Some(black_box(-40))));
    });
    let mut mixer = Mixer::new();
    case(hertz, "mixer_actuate", 50_000, || {
        mixer.actuate(black_box(5), black_box(-3), black_box(1));
    });
    let table = fin_limit_table();
    // one glide step: airspeed governor, the bank demand, three PIDs, the fused mixer
    case(hertz, "control_step", 20_000, || {
        let speed = magnitude_sq(black_box(18.0), black_box(2.5), black_box(-1.2)).sqrt();
        mixer.limit = fin_deflection_limit(&table, speed);
        let bank = bank_demand(black_box(40), 1.5, 30.0);
        let roll_error = (bank * 100.0) as i32 - black_box(1250);
        let roll_out = roll.step(roll_error, 10, Some(black_box(-300)));
        let pitch_out = pitch.step(black_box(-250), 10, Some(black_box(120)));
        let yaw_out = yaw.step(black_box(80), 10, Some(black_box(-40)));
        mixer.actuate(
            roll_out.div_euclid(100),
            pitch_out.div_euclid(100),
            yaw_out.div_euclid(100),
        );
    });

    // the same, written the Rust way: f32 throughout (no MicroPython twin -- MicroPython had to avoid it)
    let mut roll = PidF32::new(1.5, 0.2, 0.1, 20.0, 30.0);
    let mut pitch = PidF32::new(1.5, 0.2, 0.1, 20.0, 30.0);
    let mut yaw = PidF32::new(1.0, 0.0, 0.05, 20.0, 30.0);
    case(hertz, "pid_step_f32", 50_000, || {
        black_box(roll.step(black_box(5.0), 0.01, black_box(-3.0)));
    });
    case(hertz, "control_step_f32", 20_000, || {
        let speed = magnitude_sq(black_box(18.0), black_box(2.5), black_box(-1.2)).sqrt();
        mixer.limit = fin_deflection_limit(&table, speed);
        let bank = bank_demand(black_box(40), 1.5, 30.0);
        let roll_out = roll.step(bank - black_box(12.5), 0.01, black_box(-3.0));
        let pitch_out = pitch.step(black_box(-2.5), 0.01, black_box(1.2));
        let yaw_out = yaw.step(black_box(0.8), 0.01, black_box(-0.4));
        mixer.actuate(roll_out as i32, pitch_out as i32, yaw_out as i32);
    });
}

// ---- navigation.py -------------------------------------------------------------------------------------

/// navigation.py's flat-earth geometry in f32 -- MicroPython here is single precision too. Lat/lon precision is
/// kept by working in metres from a local reference ("accumulate metres, convert once"), never by f64.
mod geometry32 {
    /// commons.M_PER_DEG
    const M_PER_DEG: f32 = 111_320.0;

    pub type Point = (f32, f32);

    pub fn offset(lat1: f32, lon1: f32, lat2: f32, lon2: f32) -> (f32, f32) {
        let lat_mid = ((lat1 + lat2) / 2.0).to_radians();
        let dlon = (lon2 - lon1 + 180.0).rem_euclid(360.0) - 180.0;
        (dlon * M_PER_DEG * lat_mid.cos(), (lat2 - lat1) * M_PER_DEG)
    }

    fn compass(east: f32, north: f32) -> f32 {
        east.atan2(north).to_degrees().rem_euclid(360.0)
    }

    pub fn bearing(lat1: f32, lon1: f32, lat2: f32, lon2: f32) -> f32 {
        let (east, north) = offset(lat1, lon1, lat2, lon2);
        compass(east, north)
    }

    fn distance(lat1: f32, lon1: f32, lat2: f32, lon2: f32) -> f32 {
        let (east, north) = offset(lat1, lon1, lat2, lon2);
        (east * east + north * north).sqrt()
    }

    /// navigation.zone: (target, gate_a, gate_b)
    pub fn zone(top_left: Point, bottom_right: Point) -> (Point, Point, Point) {
        let ((lat_t, lon_l), (lat_b, lon_r)) = (top_left, bottom_right);
        let (lat_c, lon_c) = ((lat_t + lat_b) / 2.0, (lon_l + lon_r) / 2.0);
        let lat_span = (lat_t - lat_b).abs() * M_PER_DEG;
        let lon_span = (lon_r - lon_l).abs() * M_PER_DEG * lat_c.to_radians().cos();
        if lon_span >= lat_span {
            ((lat_c, lon_c), (lat_c, lon_l), (lat_c, lon_r))
        } else {
            ((lat_c, lon_c), (lat_t, lon_c), (lat_b, lon_c))
        }
    }

    fn inside(position: Point, top_left: Point, bottom_right: Point) -> bool {
        let ((lat, lon), (lat_t, lon_l), (lat_b, lon_r)) = (position, top_left, bottom_right);
        lat_t.min(lat_b) <= lat && lat <= lat_t.max(lat_b) && lon_l.min(lon_r) <= lon && lon <= lon_l.max(lon_r)
    }

    /// navigation.steer_to: (bearing, waypoint, is the target leg)
    pub fn steer_to(
        position: Point,
        top_left: Point,
        bottom_right: Point,
        (target, gate_a, gate_b): (Point, Point, Point),
    ) -> (f32, Point, bool) {
        let (lat, lon) = position;
        let (waypoint, on_target) = if inside(position, top_left, bottom_right) {
            (target, true)
        } else if distance(lat, lon, gate_a.0, gate_a.1) <= distance(lat, lon, gate_b.0, gate_b.1) {
            (gate_a, false)
        } else {
            (gate_b, false)
        };
        (bearing(lat, lon, waypoint.0, waypoint.1), waypoint, on_target)
    }

    /// navigation.steer (the zone recomputed per call, as in Python)
    pub fn steer(position: Point, top_left: Point, bottom_right: Point) -> (f32, Point, bool) {
        steer_to(position, top_left, bottom_right, zone(top_left, bottom_right))
    }

    /// navigation.approach_to
    pub fn approach_to(
        position: Point,
        (target, gate_a, gate_b): (Point, Point, Point),
        heading: f32,
        cross_gain: f32,
        intercept_max: f32,
    ) -> f32 {
        let mut centreline = bearing(gate_a.0, gate_a.1, gate_b.0, gate_b.1);
        if ((centreline - heading + 180.0).rem_euclid(360.0) - 180.0).abs() > 90.0 {
            centreline = (centreline + 180.0).rem_euclid(360.0);
        }
        let (east, north) = offset(target.0, target.1, position.0, position.1);
        let radians = centreline.to_radians();
        let cross = east * radians.cos() - north * radians.sin();
        let intercept = (-cross_gain * cross).clamp(-intercept_max, intercept_max);
        (centreline + intercept).rem_euclid(360.0)
    }
}

fn navigation(hertz: u64) {
    // a made-up zone (not a real site): 200 m x 100 m, the glider 600 m out
    let (top_left, bottom_right, position) = ((48.1010, 11.5000), (48.1000, 11.5027), (48.1050, 11.4950));
    case(hertz, "steer_f32", 20_000, || {
        black_box(geometry32::steer(black_box(position), top_left, bottom_right));
    });
    let zone32 = geometry32::zone(top_left, bottom_right);
    case(hertz, "approach_f32", 20_000, || {
        black_box(geometry32::approach_to(black_box(position), zone32, 120.0, 0.5, 30.0));
    });
}

// ---- recorder.py ------------------------------------------------------------------------------------------

/// recorder._crc_table: the reflected IEEE CRC-32 table (zlib's)
fn crc_table() -> [u32; 256] {
    core::array::from_fn(|index| {
        let mut value = index as u32;
        for _ in 0..8 {
            value = if value & 1 != 0 {
                (value >> 1) ^ 0xEDB8_8320
            } else {
                value >> 1
            };
        }
        value
    })
}

const HEX: &[u8; 16] = b"0123456789abcdef";

/// recorder._wrap: `[@routing@]payload\n` -> `[@routing@]{OPEN};payload;<CLOSE>\n`; the line's length, or None
fn wrap(record: &[u8], line: &mut [u8], table: &[u32; 256]) -> Option<usize> {
    let mut end = record.len();
    if end > 0 && record[end - 1] == b'\n' {
        end -= 1;
    }
    if end + 23 > line.len() {
        return None;
    }
    let mut payload = 0;
    if end > 0 && record[0] == b'@' {
        if let Some(close) = record[1..end].iter().position(|byte| *byte == b'@') {
            payload = close + 2;
        }
    }
    let covered = if payload > 0 { 1 } else { 0 };
    let mut crc = u32::MAX;
    for byte in &record[covered..end] {
        crc = table[((crc ^ *byte as u32) & 0xff) as usize] ^ (crc >> 8);
    }
    let number = record[payload..end]
        .iter()
        .take_while(|byte| byte.is_ascii_digit())
        .fold(0u32, |number, byte| {
            number.wrapping_mul(10).wrapping_add((byte - b'0') as u32)
        });
    line[..payload].copy_from_slice(&record[..payload]);
    let mut out = payload;
    let put_hex = |line: &mut [u8], out: &mut usize, value: u32| {
        for shift in (0..8).rev() {
            line[*out] = HEX[((value >> (shift * 4)) & 15) as usize];
            *out += 1;
        }
    };
    line[out] = b'{';
    out += 1;
    put_hex(line, &mut out, !crc);
    line[out..out + 2].copy_from_slice(b"};");
    out += 2;
    line[out..out + end - payload].copy_from_slice(&record[payload..end]);
    out += end - payload;
    line[out..out + 2].copy_from_slice(b";<");
    out += 2;
    put_hex(line, &mut out, crc ^ number);
    line[out..out + 2].copy_from_slice(b">\n");
    Some(out + 2)
}

fn recorder(hertz: u64) {
    // a 20-field integer telemetry row, as recorder.Telemetry.push formats it ('@<session>_<file>@%u;%s;...')
    let fields: [i32; 19] = core::array::from_fn(|index| index as i32 * 137 - 900);
    let mut row = String::with_capacity(256);
    case(hertz, "telemetry_row_format", 20_000, || {
        row.clear();
        let _ = write!(row, "@000135_flight@{}", black_box(123_456_789u32));
        for field in black_box(&fields) {
            let _ = write!(row, ";{field}");
        }
        row.push('\n');
        black_box(&row);
    });
    let table = crc_table();
    let mut line = [0u8; 320];
    let record = row.as_bytes().to_vec();
    case(hertz, "recorder_wrap", 20_000, || {
        black_box(wrap(black_box(&record), &mut line, &table));
    });
}

// ---- bench_asyncio.py's micro-operations ------------------------------------------------------------------

fn infrastructure(hertz: u64) {
    case(hertz, "ticks_us", 100_000, || {
        // SAFETY: a plain read of the system timer.
        black_box(unsafe { sys::esp_timer_get_time() });
    });
    case(hertz, "alloc_bytes256", 20_000, || {
        black_box(vec![0u8; black_box(256)]);
    });
    let mut buffer = [0u8; 64];
    case(hertz, "pack_into_16B", 100_000, || {
        // struct.pack_into('<IiiI', buf, 16, ...): four little-endian words at an offset
        let (a, b, c, d) = black_box((123_456u32, -42i32, 7i32, 0xdead_beefu32));
        buffer[16..20].copy_from_slice(&a.to_le_bytes());
        buffer[20..24].copy_from_slice(&b.to_le_bytes());
        buffer[24..28].copy_from_slice(&c.to_le_bytes());
        buffer[28..32].copy_from_slice(&d.to_le_bytes());
        black_box(&buffer);
    });
    case(hertz, "yield", 10_000, || {
        std::thread::yield_now();
    });
}
