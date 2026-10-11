//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The fin servos (M13; plan B drivers/sg90.py, mg90s.py, the logic in crates/drivers sg90): one LEDC channel per
//! servo on one 50 Hz timer at 14-bit duty (1.22 us a step). Every servo pin of the config is driven LOW from the
//! first moment of the boot ([`hold_low`]): a floating fin pin picks up the others' pulses and every servo twitches
//! (the bench gotcha). At bring-up's end [`open`] gives each pin its channel with no pulse, then centres the fins a
//! group at a time, `fins.concurrency` per group, each group given its travel time before the next: three servos
//! starting together draw their peaks together, which a 1 A bench supply does not carry.
//!
//! From there the control thread owns the fins: the flight loop's compare-and-set [`Fins::set_angle`] (M13 stage 4)
//! and the operator's `update <servo> {trim|angle}` through its mailbox ([`Fins::update`]). A write is two LEDC
//! register updates; the row in `<name>.csv` (`angle`, `pulse_us`, `done`) goes through the recorder's lock-free
//! producer ring. The executor reads each fin's state through atomics (`inspect`).

use std::sync::OnceLock;
use std::sync::atomic::{AtomicI32, Ordering};
use std::time::Duration;

use drivers::sg90::{Command, PERIOD_US, Servo};
use esp_idf_svc::sys;
use recorder::{Cell, Telemetry};
use serde_json::Value;

use crate::devices::ServoMember;

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
    /// `<name>.csv`, with a recorder.
    telemetry: Option<Telemetry<'static>>,
    /// A refused row was logged since the last accepted one.
    overflowing: bool,
}

/// The fins, owned by one thread: bring-up's main, then the control thread.
pub struct Fins {
    /// In config order.
    fins: Vec<Fin>,
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

/// Gives each servo its LEDC channel (no pulse yet), then centres the fins `concurrency` at a time, each group
/// given its travel time; the fins, for the control thread.
///
/// Errors: the LEDC timer or a channel would not configure (the fins are then crashed by the caller).
pub fn open(members: Vec<ServoMember>, concurrency: usize) -> anyhow::Result<Fins> {
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
            telemetry: crate::recorder::get()
                .zip(crate::recorder::device_stream(member.name))
                .and_then(|(recorder, stream)| Telemetry::new(recorder, stream)),
            overflowing: false,
        });
    }
    let _ = NAMES.set(fins.iter().map(|fin| fin.name).collect());
    let mut owned = Fins { fins };
    owned.centre(concurrency.max(1));
    Ok(owned)
}

impl Fins {
    /// The boot centring: each fin to its configured angle (`angle`, else the range's neutral), trim included, a group
    /// of `concurrency` at a time. The horn's start is unknown, so each group waits a half-range travel with the
    /// settle margin before the next starts; each move is recorded issued, then completed.
    fn centre(&mut self, concurrency: usize) {
        for first in (0..self.fins.len()).step_by(concurrency) {
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
            std::thread::sleep(Duration::from_millis(u64::try_from(longest_ms).unwrap_or(0)));
            for index in group {
                let command = self.fins[index].servo.command();
                self.record(index, command, true);
            }
        }
    }

    /// Drives `command`'s pulse on fin `index`'s channel and records it (`done` false: issued).
    fn write(&mut self, index: usize, command: Command, done: bool) {
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
        // SAFETY: a plain read of the monotonic timer.
        let now_us = unsafe { sys::esp_timer_get_time() } as u64;
        crate::sensors::record(fin.name, &mut fin.telemetry, &mut fin.overflowing, now_us, &row);
    }

    /// The flight loop's command for fin `index` (plan B `set_angle`): compare-and-set, nothing written when the
    /// clamped angle is the one already commanded.
    #[expect(dead_code, reason = "the flight loop's, M13 stage 4")]
    pub fn set_angle(&mut self, index: usize, degrees: f64) {
        if let Some(command) = self.fins.get_mut(index).and_then(|fin| fin.servo.set_angle(degrees)) {
            self.write(index, command, false);
        }
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

/// Fin `name`'s state for `inspect` (plan B sg90 inspect): the commanded angle, the pulse driven (us), the trim; None
/// for a name that is no fin.
pub fn inspection(name: &str) -> Option<[i32; 3]> {
    let index = NAMES.get()?.iter().position(|fin| *fin == name)?;
    let state = STATE.get(index)?;
    Some(state.each_ref().map(|value| value.load(Ordering::Relaxed)))
}
