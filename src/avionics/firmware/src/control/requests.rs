//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The control thread's requests: the executor's flight-word slot and the mailbox of probes, bus tunes and fin updates,
//! served on the tick.

use super::*;

/// The executor's request slot (§4.3, step 4): its `stage`, `arm` and `disarm` reach the flight word's writer
/// through it.
/// The executor's alone: another requesting thread gets a slot of its own.
pub(super) static EXECUTOR_SLOT: RequestSlot = RequestSlot::new();

/// How long the executor waits for the tick to take its request before withdrawing it (§4.4): two hundred ticks at
/// the default 100 Hz, and two at the slowest rate a config may set (1 Hz), so a thread that took none has stalled.
pub(super) const REQUEST_DEADLINE: Duration = Duration::from_secs(2);

/// How often the executor looks for its answer: one FreeRTOS tick (CONFIG_FREERTOS_HZ is 1000).
pub(super) const REQUEST_POLL: Duration = Duration::from_millis(1);

/// What the control thread's mailbox asks between its tick's work (§4.3, step 4), each answer going back through its
/// sender: the outcome, or the stage's refusal outside SETTING (a value: the executor makes its text).
pub(super) enum Request {
    /// Probe the IMU: None healthy, else why.
    ProbeImu(SyncSender<Result<Option<String>, ChangeRefused>>),
    /// Run the IMU's SPI handle at `clock_hz` and probe it (a bustune rung): the verdict and the clock it runs at.
    BustuneSpi {
        clock_hz: u32,
        reply: SyncSender<Result<(Option<String>, u32), ChangeRefused>>,
    },
    /// Skip the IMU's reads for `duration_ms` (`imu mute`, M9): whether an IMU took it.
    Mute {
        duration_ms: u32,
        reply: SyncSender<Result<bool, ChangeRefused>>,
    },
    /// The operator's `update <servo> {trim|angle}` on fin `name` (M13): the keys applied, the servo's refusal, or
    /// None for a name that is no fin.
    Fin {
        name: String,
        properties: Value,
        reply: SyncSender<Result<FinAnswer, ChangeRefused>>,
    },
    /// Sweep every fin, or the one named, through its probe stops (M13): answered when the sweep is over.
    ProbeFins {
        only: Option<String>,
        reply: SyncSender<SweepAnswer>,
    },
}

/// A fin update's answer: None for a name that is no fin, else the keys applied or the servo's refusal.
pub(super) type FinAnswer = Option<Result<Vec<&'static str>, String>>;

/// A probe sweep's answer: each fin's sweep, or why none ran.
pub(super) type SweepAnswer = Result<Vec<crate::fins::Swept>, String>;

/// The probe sweep under way and where its answer goes.
pub(super) type Sweeping = Option<(crate::fins::Sweep, SyncSender<SweepAnswer>)>;

/// The control thread's mailbox, once it runs.
pub(super) static MAILBOX: OnceLock<SyncSender<Request>> = OnceLock::new();

/// How many mailbox requests a tick serves at most (§4.3: two), after the request slot's one.
pub(super) const REQUESTS_PER_TICK: usize = 2;

/// How long a probe waits for the control thread's answer: a tick and a frame, with a wide margin.
pub(super) const PROBE_DEADLINE: Duration = Duration::from_secs(2);

/// The refusal of a fin request during the boot centring.
pub(super) const CENTRING: &str = "the fins are still centring after the boot";

/// Serves at most [`REQUESTS_PER_TICK`] requests waiting in the mailbox, each refused unless `stage` is SETTING: the
/// executor checked its gate once, and a probe sweep then takes seconds, during which a launch may be detected. A
/// refusal is the stage as a value, so off the pad this thread makes no text (§3.1: text only off the flight path).
pub(super) fn serve(
    requests: &Receiver<Request>,
    mut imu: Option<&mut Imu>,
    bus: Option<&SpiBus>,
    mut fins: Option<&mut crate::fins::Fins>,
    sweeping: &mut Sweeping,
    stage: Stage,
) {
    for _ in 0..REQUESTS_PER_TICK {
        let request = match requests.try_recv() {
            Ok(request) => request,
            Err(TryRecvError::Empty | TryRecvError::Disconnected) => return,
        };
        let refusal = stage.change_refused(false);
        match request {
            Request::ProbeImu(reply) => {
                let answer = match refusal {
                    Some(refusal) => Err(refusal),
                    None => Ok(imu.as_deref_mut().and_then(Imu::probe)),
                };
                let _ = reply.send(answer);
            }
            Request::BustuneSpi { clock_hz, reply } => {
                let answer = match (refusal, imu.as_deref_mut(), bus) {
                    (Some(refusal), ..) => Err(refusal),
                    (None, Some(imu), Some(bus)) => Ok(imu.retune(bus, clock_hz)),
                    (None, ..) => Ok((Some("no SPI IMU".to_string()), 0)),
                };
                let _ = reply.send(answer);
            }
            Request::Mute { duration_ms, reply } => {
                let answer = match (refusal, imu.as_deref_mut()) {
                    (Some(refusal), _) => Err(refusal),
                    (None, Some(imu)) => {
                        imu.mute(uptime_us(), duration_ms);
                        Ok(true)
                    }
                    (None, None) => Ok(false),
                };
                let _ = reply.send(answer);
            }
            Request::Fin {
                name,
                properties,
                reply,
            } => {
                let answer = match refusal {
                    Some(refusal) => Err(refusal),
                    None if fins.as_deref().is_some_and(crate::fins::Fins::centring) => {
                        Ok(Some(Err(CENTRING.to_string())))
                    }
                    None => Ok(fins.as_deref_mut().and_then(|fins| fins.update(&name, &properties))),
                };
                let _ = reply.send(answer);
            }
            Request::ProbeFins { only, reply } => {
                if let Some(refusal) = refusal {
                    let _ = reply.send(Err(refusal.to_string()));
                } else if fins.as_deref().is_some_and(crate::fins::Fins::centring) {
                    let _ = reply.send(Err(CENTRING.to_string()));
                } else if sweeping.is_some() {
                    let _ = reply.send(Err("a fin probe sweep is under way".to_string()));
                } else {
                    match fins.as_deref().and_then(|fins| fins.sweep(only.as_deref())) {
                        Some(sweep) => *sweeping = Some((sweep, reply)),
                        None => {
                            let _ = reply.send(Ok(Vec::new()));
                        }
                    }
                }
            }
        }
    }
}

/// The fins' probe sweep (plan B sg90.probe, M13): every fin, or the one named `only`, one at a time through its
/// stops on the control thread, the INA226's power sampled as each travels. SETTING and disarmed only; a launch, an
/// arm or a stage change before a fin leaves the fins left unswept.
///
/// Returns: each fin's sweep; None when no fin runs, or `only` names none.
///
/// Errors: the control thread is not running, busy, refused outside SETTING, already sweeping, or did not answer.
pub fn probe_fins(only: Option<&str>) -> Option<Result<Vec<crate::fins::Swept>, String>> {
    let count = crate::fins::count(only)?;
    let Some(mailbox) = MAILBOX.get() else {
        return Some(Err("the control thread is not running".to_string()));
    };
    let (reply, answer) = sync_channel(1);
    if mailbox
        .try_send(Request::ProbeFins {
            only: only.map(str::to_string),
            reply,
        })
        .is_err()
    {
        return Some(Err("the control thread is busy".to_string()));
    }
    // each fin: the settle, three stops of the sampling window, with a wide margin
    let deadline = PROBE_DEADLINE + Duration::from_secs(3) * count as u32;
    Some(match answer.recv_timeout(deadline) {
        Ok(answer) => answer,
        Err(_) => Err("the control thread did not finish the fin sweep".to_string()),
    })
}

/// The operator's `update <servo> {trim|angle}` (M13): posted to the control thread, the fins' owner.
///
/// Returns: the keys applied, or the servo's refusal (a key that is not a number).
///
/// Errors: no fin by that name, the control thread is not running, is busy, refused outside SETTING, or did not
/// answer in time.
pub fn update_fin(name: &str, properties: &Value) -> Result<Result<Vec<&'static str>, String>, String> {
    let mailbox = MAILBOX
        .get()
        .ok_or_else(|| "the control thread is not running".to_string())?;
    let (reply, answer) = sync_channel(1);
    mailbox
        .try_send(Request::Fin {
            name: name.to_string(),
            properties: properties.clone(),
            reply,
        })
        .map_err(|_| "the control thread is busy".to_string())?;
    match answer.recv_timeout(PROBE_DEADLINE) {
        Ok(Ok(Some(answer))) => Ok(answer),
        Ok(Ok(None)) => Err(format!("{name} is no fin on the control thread")),
        Ok(Err(refusal)) => Err(refusal.to_string()),
        Err(_) => Err("the control thread did not answer".to_string()),
    }
}

/// Applies an operator `request` (the executor's `stage`, `arm`, `disarm`) to the flight word through its one writer, this
/// thread: posted to the executor's slot, served at the next tick, withdrawn when not taken in [`REQUEST_DEADLINE`].
/// While the thread is not running nothing commits, so a request that would change nothing (a disarm of a disarmed
/// board) is answered with the flight as it stands, at once, and any other is not applied. The executor's alone.
///
/// Returns: the answer: the flight committed, or why an arm was refused.
///
/// Errors: why there is no answer, saying whether the request was applied.
pub fn request(request: sequencer::Request) -> Result<sequencer::Answer, String> {
    if !RUNNING.load(Ordering::Acquire) {
        let flight = FLIGHT.load();
        return match flight.apply(request) {
            Ok(after) if after == flight => Ok(Ok(flight)),
            _ => Err("the control thread is not running: not applied".to_string()),
        };
    }
    EXECUTOR_SLOT
        .request(request, REQUEST_DEADLINE, REQUEST_POLL)
        .map_err(|unanswered| unanswered.to_string())
}
