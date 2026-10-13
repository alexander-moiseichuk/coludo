//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The fin servos (M13; plan B drivers/sg90.py, mg90s.py, the logic in crates/drivers sg90): one LEDC channel per
//! servo on one 50 Hz timer at 14-bit duty (1.22 us a step). Every servo pin of the config is driven LOW from the
//! first moment of the boot ([`hold_low`]): a floating fin pin picks up the others' pulses and every servo twitches
//! (the bench gotcha). At bring-up's end [`open`] gives each pin its channel with no pulse. On a cold boot the control
//! thread then centres the fins ([`Fins::centre`], stepped each tick), a group at a time, `fins.concurrency` per
//! group, each group given its travel time before the next: three servos starting together draw their peaks
//! together, which a 1 A bench supply does not carry. The centring never holds the control thread's start (it took
//! ~0.9 s at concurrency 1: the fast warm boot, operator 2026-10-11); a warm start skips it, the restored law taking
//! the fins from its first tick, and a launch during it hands the fins to the law.
//!
//! From there the control thread owns the fins: the flight loop's compare-and-set [`Fins::set_angle`] (M13 stage 4)
//! and the operator's `update <servo> {trim|angle}` through its mailbox ([`Fins::update`]). A write is two LEDC
//! register updates; the row in `<name>.csv` (`angle`, `pulse_us`, `done`) goes through the recorder's lock-free
//! producer ring. The executor reads each fin's state through atomics (`inspect`).

use std::sync::OnceLock;
use std::sync::atomic::{AtomicI32, Ordering};

use drivers::sg90::{Command, PERIOD_US, PROBE_SAMPLE_MS, PROBE_SAMPLES, PROBE_SETTLE_MS, ProbeVerdict, Servo};
use esp_idf_svc::sys;
use recorder::{Cell, Rows};
use serde_json::Value;

use crate::devices::ServoMember;
use firmware::time::uptime_us;

/// The servos' PWM frame rate, Hz: plan B's `PWM(freq=50)`.
const FREQUENCY_HZ: u32 = 50;

/// The duty resolution, bits: 2^14 steps over the 20 ms frame, 1.22 us a step.
const RESOLUTION_BITS: u32 = 14;

/// The LEDC timer every servo channel runs on.
const TIMER: sys::ledc_timer_t = sys::ledc_timer_t_LEDC_TIMER_0;

/// The LEDC speed mode (the ESP32-P4 has the low-speed one only).
const MODE: sys::ledc_mode_t = sys::ledc_mode_t_LEDC_LOW_SPEED_MODE;

/// How many fins [`inspection`] can answer for: the airframe's three, and a spare.
const MAXIMUM_FINS: usize = 4;

/// The implementations that are servos.
const SERVO_DRIVERS: [&str; 2] = ["sg90", "mg90s"];

/// Each fin's state for `inspect`, by its index: the commanded angle, the pulse driven (us) and the trim.
static STATE: [[AtomicI32; 3]; MAXIMUM_FINS] = [const { [const { AtomicI32::new(0) }; 3] }; MAXIMUM_FINS];
/// The fins' names, by index, once [`open`] ran.
static NAMES: OnceLock<Vec<&'static str>> = OnceLock::new();

/// One fin: its servo logic, its channel and its stream.
struct Fin {
    /// The entry's name.
    name: &'static str,
    /// The command logic: clamp, trim, pulse map, compare-and-set (crates/drivers sg90).
    servo: Servo,
    /// The angle commanded at boot, degrees: the config's `angle`, else the range's neutral.
    start_degrees: f64,
    /// Half the command range, degrees: the boot centring's worst-case travel from an unknown horn.
    half_range: i32,
    /// Its LEDC channel.
    channel: sys::ledc_channel_t,
    /// Its stream: a refused row logged once a run (recorder::Rows).
    rows: Rows<'static>,
    /// Its pulse is off (DONE, [`Fins::rest`]): the next command drives it again even when unchanged.
    resting: bool,
}

/// The fins, owned by one thread: bring-up's main, then the control thread.
pub struct Fins {
    /// In config order.
    fins: Vec<Fin>,
    /// The probe sweep's half-range, whole degrees: the board config's `fins.mixer.limit_deg` (None sweeps the
    /// servos' own ranges, plan B's).
    probe_limit: Option<i32>,
    /// The boot centring under way, None once done (or never begun: a warm start).
    centring: Option<Centring>,
}

/// The boot centring's progress (crate::fins module notes).
struct Centring {
    /// Fins per group, at least 1.
    concurrency: usize,
    /// The group issued last, its moves recorded done once it settled.
    group: core::ops::Range<usize>,
    /// When that group's travel ends, µs on the monotonic timer.
    settled_us: u64,
}

/// One fin's probe sweep, for the probe's lines (crate::commands, plan B sg90.probe) and its verdict.
#[derive(Clone, Copy, Debug)]
pub struct Swept {
    /// The fin.
    pub name: &'static str,
    /// The stops, (label, degrees): min, max, neutral.
    pub stops: [(&'static str, i32); 3],
    /// How many stops it reached.
    pub reached: usize,
    /// A fresh INA226 power baseline was taken.
    pub watching: bool,
    /// That baseline, mW.
    pub baseline_mw: i32,
    /// The fresh power readings taken during the sweep.
    pub sampled: u32,
    /// The largest reading over the baseline, mW (0 when none rose).
    pub peak_rise_mw: i32,
    /// The verdict; None when the fin was not swept.
    pub verdict: Option<ProbeVerdict>,
}

/// Where a fin's sweep stands.
#[derive(Clone, Copy, Debug)]
enum Phase {
    /// The next fin starts at the next step.
    Start,
    /// At neutral, the power settling to its baseline until this time, µs.
    Settle(u64),
    /// At stop `stop`, `samples` taken, the next due at `next_us`.
    Stop { stop: usize, samples: u32, next_us: u64 },
}

/// The probe sweep in progress (plan B sg90.probe, as a control-thread state machine): one fin at a time, each to
/// its stops with the power sampled while it travels; stepped every control tick, never waiting.
pub struct Sweep {
    /// The fins to sweep, by index.
    order: Vec<usize>,
    /// The fin under way, an index into `order`.
    position: usize,
    /// Where it stands.
    phase: Phase,
    /// One per fin in `order`.
    swept: Vec<Swept>,
}

/// Drives every servo pin of `document` (sensors and components with a servo driver, enabled or not) LOW as an
/// output, before anything else touches the pins: a floating fin pin makes every servo twitch.
pub fn hold_low(document: &Value) {
    for entry in board_config::entries(document) {
        if !SERVO_DRIVERS.contains(&entry.implementation) {
            continue;
        }
        let Some(gpio) = board_config::pin(document, &entry, "pin") else {
            continue;
        };
        let config = sys::gpio_config_t {
            pin_bit_mask: 1u64 << gpio,
            mode: sys::gpio_mode_t_GPIO_MODE_OUTPUT,
            ..Default::default()
        };
        // SAFETY: plain GPIO driver calls on a pin the config gives to this servo alone.
        unsafe {
            let _ = sys::gpio_set_level(i32::from(gpio), 0);
            let _ = sys::gpio_config(&config);
            let _ = sys::gpio_set_level(i32::from(gpio), 0);
        }
    }
}

/// Gives each servo its LEDC channel (no pulse yet); the fins, for the control thread, which centres them on a cold
/// boot ([`Fins::centre`]). `probe_limit` bounds the probe sweep ([`Sweep`]).
///
/// Errors: the LEDC timer or a channel would not configure (the fins are then crashed by the caller).
pub fn open(members: Vec<ServoMember>, probe_limit: Option<i32>) -> anyhow::Result<Fins> {
    let timer = sys::ledc_timer_config_t {
        speed_mode: MODE,
        duty_resolution: RESOLUTION_BITS,
        timer_num: TIMER,
        freq_hz: FREQUENCY_HZ,
        clk_cfg: sys::soc_periph_ledc_clk_src_legacy_t_LEDC_AUTO_CLK,
        ..Default::default()
    };
    // SAFETY: a plain LEDC driver call; the timer is this module's alone.
    sys::esp!(unsafe { sys::ledc_timer_config(&timer) })?;
    let mut fins = Vec::with_capacity(members.len());
    for (index, member) in members.into_iter().enumerate() {
        let channel = sys::ledc_channel_t_LEDC_CHANNEL_0 + index as sys::ledc_channel_t;
        let config = sys::ledc_channel_config_t {
            gpio_num: i32::from(member.gpio),
            speed_mode: MODE,
            channel,
            intr_type: sys::ledc_intr_type_t_LEDC_INTR_DISABLE,
            timer_sel: TIMER,
            duty: 0,
            hpoint: 0,
            ..Default::default()
        };
        // SAFETY: a plain LEDC driver call on the pin hold_low drove low; the channel is this module's alone.
        sys::esp!(unsafe { sys::ledc_channel_config(&config) })?;
        fins.push(Fin {
            name: member.name,
            servo: member.servo,
            start_degrees: member.start_degrees,
            half_range: member.half_range,
            channel,
            rows: crate::recorder::rows(member.name),
            resting: false,
        });
    }
    let _ = NAMES.set(fins.iter().map(|fin| fin.name).collect());
    Ok(Fins {
        fins,
        probe_limit,
        centring: None,
    })
}

impl Fins {
    /// Begins the boot centring, `concurrency` fins a group (at least 1), stepped by [`Fins::step_centring`].
    pub fn centre(&mut self, concurrency: usize) {
        self.centring = Some(Centring {
            concurrency: concurrency.max(1),
            group: 0..0,
            settled_us: 0,
        });
    }

    /// Whether the boot centring is under way: the operator's fin requests wait for its end.
    pub fn centring(&self) -> bool {
        self.centring.is_some()
    }

    /// One control tick of the boot centring at `now_us`: each fin to its configured angle (`angle`, else the
    /// range's neutral), trim included, a group at a time. The horn's start is unknown, so a group is given a
    /// half-range travel with the settle margin before the next is issued; each move is recorded issued, then
    /// completed. `may_move` false (the stage left SETTING: a launch) ends it where it is, the law's from then.
    pub fn step_centring(&mut self, now_us: u64, may_move: bool) {
        let Some(centring) = &self.centring else {
            return;
        };
        if !may_move {
            self.centring = None;
            return;
        }
        if now_us < centring.settled_us {
            return;
        }
        let (settled, concurrency) = (centring.group.clone(), centring.concurrency);
        for index in settled.clone() {
            let command = self.fins[index].servo.command();
            self.record(index, command, true);
        }
        let first = settled.end;
        if first >= self.fins.len() {
            self.centring = None;
            return;
        }
        let group = first..(first + concurrency).min(self.fins.len());
        let mut longest_ms = 0;
        for index in group.clone() {
            let fin = &mut self.fins[index];
            let command = fin.servo.apply(fin.start_degrees);
            longest_ms = longest_ms.max(fin.servo.travel_ms(fin.half_range.max(1), true));
            self.write(index, command, false);
            log::info!(
                "{} :: centred at {} deg ({} us)",
                self.fins[index].name,
                command.angle,
                command.pulse_us
            );
        }
        self.centring = Some(Centring {
            concurrency,
            group,
            settled_us: now_us + u64::try_from(longest_ms).unwrap_or(0) * 1000,
        });
    }

    /// Drives `command`'s pulse on fin `index`'s channel and records it (`done` false: issued).
    fn write(&mut self, index: usize, command: Command, done: bool) {
        let fin = &mut self.fins[index];
        fin.resting = false;
        let fin = &self.fins[index];
        let duty = (i64::from(command.pulse_us) << RESOLUTION_BITS) / i64::from(PERIOD_US);
        // SAFETY: plain LEDC driver calls on this fin's configured channel.
        unsafe {
            let _ = sys::ledc_set_duty(MODE, fin.channel, duty as u32);
            let _ = sys::ledc_update_duty(MODE, fin.channel);
        }
        self.record(index, command, done);
    }

    /// The `<name>.csv` row of `command` and the fin's state for `inspect`.
    fn record(&mut self, index: usize, command: Command, done: bool) {
        let fin = &mut self.fins[index];
        if let Some(state) = STATE.get(index) {
            state[0].store(command.angle, Ordering::Relaxed);
            state[1].store(command.pulse_us, Ordering::Relaxed);
            state[2].store(fin.servo.trim(), Ordering::Relaxed);
        }
        let row = command.row(done).map(Cell::I32);
        let now_us = uptime_us();
        fin.rows.push(now_us, &row);
    }

    /// The flight loop's command for fin `index` (plan B `set_angle`): compare-and-set, nothing written when the
    /// clamped angle is the one already commanded.
    pub fn set_angle(&mut self, index: usize, degrees: f64) {
        let Some(fin) = self.fins.get_mut(index) else {
            return;
        };
        match fin.servo.set_angle(degrees) {
            Some(command) => self.write(index, command, false),
            // the angle held, but its pulse was cut (DONE): drive it again
            None if fin.resting => {
                let command = fin.servo.command();
                self.write(index, command, false);
            }
            None => {}
        }
    }

    /// At DONE (M11b): every fin's pulse off, so the servos draw no holding current while the beacon runs on the
    /// battery; the next command drives a fin again.
    pub fn rest(&mut self) {
        for fin in &mut self.fins {
            // SAFETY: plain LEDC driver calls on this fin's configured channel.
            unsafe {
                let _ = sys::ledc_set_duty(MODE, fin.channel, 0);
                let _ = sys::ledc_update_duty(MODE, fin.channel);
            }
            fin.resting = true;
        }
        log::info!("fins :: at rest, no pulse (DONE)");
    }

    /// The operator's `update <servo> {trim|angle}` (plan B `update`): immediate and ungated; a new trim alone
    /// re-drives the current angle so the fin moves to its new centre.
    ///
    /// Returns: the keys applied, a subset of ["trim", "angle"]; None for a name that is no fin.
    ///
    /// Errors: the refusal's text (a key that is not a number).
    pub fn update(&mut self, name: &str, properties: &Value) -> Option<Result<Vec<&'static str>, String>> {
        let index = self.fins.iter().position(|fin| fin.name == name)?;
        Some(match self.fins[index].servo.update(properties) {
            Ok((applied, command)) => {
                if let Some(command) = command {
                    self.write(index, command, false);
                }
                Ok(applied)
            }
            Err(refusal) => Err(refusal.to_string()),
        })
    }
}

impl Fins {
    /// The index of the fin named `name`.
    pub fn index(&self, name: &str) -> Option<usize> {
        self.fins.iter().position(|fin| fin.name == name)
    }

    /// A probe sweep of every fin, or of the one named `only`; None when `only` names no fin.
    pub fn sweep(&self, only: Option<&str>) -> Option<Sweep> {
        let order: Vec<usize> = (0..self.fins.len())
            .filter(|&index| only.is_none_or(|only| self.fins[index].name == only))
            .collect();
        if order.is_empty() {
            return None;
        }
        let swept = order
            .iter()
            .map(|&index| Swept {
                name: self.fins[index].name,
                stops: self.fins[index].servo.probe_targets(self.probe_limit),
                reached: 0,
                watching: false,
                baseline_mw: 0,
                sampled: 0,
                peak_rise_mw: 0,
                verdict: None,
            })
            .collect();
        Some(Sweep {
            order,
            position: 0,
            phase: Phase::Start,
            swept,
        })
    }

    /// One control tick of `sweep` at `now_us`: `power` the INA226's fresh reading (mW), `may_move` whether a fin
    /// may move now (SETTING, disarmed, no launch), checked before each fin -- when it may not, the fins left are not
    /// swept. Returns the fins' sweeps once it is over.
    pub fn step_sweep(
        &mut self,
        sweep: &mut Sweep,
        now_us: u64,
        power: Option<f32>,
        may_move: bool,
    ) -> Option<Vec<Swept>> {
        while let Some(&index) = sweep.order.get(sweep.position) {
            let swept = &mut sweep.swept[sweep.position];
            match sweep.phase {
                Phase::Start => {
                    if !may_move {
                        sweep.position = sweep.order.len(); // the fins left stay unswept: verdict None
                        break;
                    }
                    let command = self.fins[index].servo.apply(f64::from(swept.stops[2].1));
                    self.write(index, command, false);
                    swept.watching = power.is_some();
                    sweep.phase = if swept.watching {
                        Phase::Settle(now_us + u64::from(PROBE_SETTLE_MS) * 1000)
                    } else {
                        Phase::Stop {
                            stop: 0,
                            samples: 0,
                            next_us: now_us,
                        }
                    };
                }
                Phase::Settle(until_us) => {
                    if now_us < until_us {
                        return None;
                    }
                    match power {
                        Some(power) => swept.baseline_mw = power as i32,
                        None => swept.watching = false, // the INA went stale while settling: no verdict
                    }
                    sweep.phase = Phase::Stop {
                        stop: 0,
                        samples: 0,
                        next_us: now_us,
                    };
                }
                Phase::Stop { .. } => {
                    self.step_stop(sweep, index, now_us, power);
                    return None;
                }
            }
        }
        Some(std::mem::take(&mut sweep.swept))
    }

    /// `sweep`'s tick at a stop of fin `index` (Phase::Stop): its command at the stop's first sample, then
    /// [`PROBE_SAMPLES`] samples of `power` a [`PROBE_SAMPLE_MS`] apart; then the next stop, or after the last the
    /// fin's verdict and the next fin, at the next tick.
    fn step_stop(&mut self, sweep: &mut Sweep, index: usize, now_us: u64, power: Option<f32>) {
        let Phase::Stop { stop, samples, next_us } = sweep.phase else {
            return;
        };
        let sample_us = u64::from(PROBE_SAMPLE_MS) * 1000;
        let swept = &mut sweep.swept[sweep.position];
        if now_us < next_us {
            return;
        }
        if samples == 0 {
            let command = self.fins[index].servo.apply(f64::from(swept.stops[stop].1));
            self.write(index, command, false);
        }
        if swept.watching
            && let Some(power) = power
        {
            swept.sampled += 1;
            swept.peak_rise_mw = swept.peak_rise_mw.max(power as i32 - swept.baseline_mw);
        }
        let samples = samples + 1;
        if samples < PROBE_SAMPLES {
            sweep.phase = Phase::Stop {
                stop,
                samples,
                next_us: next_us + sample_us,
            };
            return;
        }
        swept.reached = stop + 1;
        if stop + 1 < swept.stops.len() {
            sweep.phase = Phase::Stop {
                stop: stop + 1,
                samples: 0,
                next_us: next_us + sample_us,
            };
            return;
        }
        swept.verdict = Some(
            self.fins[index]
                .servo
                .probe_verdict(swept.watching, swept.sampled, swept.peak_rise_mw),
        );
        sweep.position += 1;
        sweep.phase = Phase::Start; // the next fin starts at the next tick
    }
}

/// How many fins a sweep of `only` (every fin when None) covers; None when no fin runs, or `only` names none.
pub fn count(only: Option<&str>) -> Option<usize> {
    let names = NAMES.get()?;
    let count = names
        .iter()
        .filter(|name| only.is_none_or(|only| **name == only))
        .count();
    (count > 0).then_some(count)
}

/// Fin `name`'s commanded angle, whole degrees, once a pulse was driven: None before its first write (a warm start
/// skips the centring, so a fin waits for the restored law) and for a name that is no fin.
pub fn commanded(name: &str) -> Option<i32> {
    inspection(name).and_then(|[angle, pulse_us, _]| (pulse_us != 0).then_some(angle))
}

/// Fin `name`'s state for `inspect` (plan B sg90 inspect): the commanded angle, the pulse driven (us), the trim; None
/// for a name that is no fin.
pub fn inspection(name: &str) -> Option<[i32; 3]> {
    let index = NAMES.get()?.iter().position(|fin| *fin == name)?;
    let state = STATE.get(index)?;
    Some(state.each_ref().map(|value| value.load(Ordering::Relaxed)))
}
