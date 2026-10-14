//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The attitude filter on the board: the pure helpers its caller needs around [`super::Attitude::tick`].
//!  * [`assemble`] -- what one step reads from the databoard (plan B `_tick`'s reads): the fused `rate`, `accel` and
//!    `mag` through the marks map, the GNSS `course` and `speed`, the fused `attitude`'s winner with its rank, and
//!    whether a source below the filter is fresh to take over from it;
//!  * [`Pace`] -- which control ticks step the filter: every `period_ms` on the uptime clock (plan B's task slept its
//!    period between ticks);
//!  * [`on_backup`] -- the health flag `attitude-backup` (plan B cc_client `_on_backup`): whether a channel's fused
//!    winner is outranked by an enabled provider of it.
//!
//! No clock and no lock: the times are arguments and the databoard's fused reads are lock-free, so the host tests
//! drive them over a real databoard.

use databoard::Parameter;
use serde_json::Value;

use super::{Fused, Orientation, Readings};

/// The databoard quantities one filter step reads, as the firmware's topology holds them.
#[derive(Clone, Copy)]
pub struct Quantities<'board> {
    /// `rate`: the gyro in degrees per second, in the marks frame (the LSM6DSO32, backed up by the BMI323); None when
    /// no provider of it came up.
    pub rate: Option<&'board Parameter<[f32; 3]>>,
    /// `accel`: the accelerometer's specific force in g, in the marks frame; None when no provider of it came up.
    pub acceleration: Option<&'board Parameter<[f32; 3]>>,
    /// `mag`: the calibrated magnetometer in the marks frame (the BMM350); None when no provider of it came up.
    pub magnetic: Option<&'board Parameter<[f32; 3]>>,
    /// `course`: the GNSS ground-track course in degrees, the yaw's absolute reference; None when no provider of it
    /// came up.
    pub course: Option<&'board Parameter<f32>>,
    /// `speed`: the GNSS ground speed in m/s, which gates the course; None when no provider of it came up.
    pub speed: Option<&'board Parameter<f32>>,
    /// `attitude`: the fused attitude the filter publishes into, its samples packed as [`Orientation::to_sample`]
    /// packs them. Never absent: the filter's own registration made it, and a filter with none does not run (plan B's
    /// setup fails).
    pub attitude: &'board Parameter<[f32; 3]>,
}

/// The readings of one filter step at `now_us` (plan B `_tick`'s reads): the fused `rate`, `accel` and `mag` mapped
/// from the marks frame ([`Readings::from_marks`]); the GNSS `course` and ground `speed`, the yaw's pull toward the
/// track (without them the heading is the gyro's alone, relative to how the board sat at power-up, and the
/// magnetometer's offset is never learned); the fused `attitude`'s fresh winner with the rank its source registered
/// at; whether a source ranked below `rank`, the filter's own ([`super::Settings::rank`]), is fresh; and the
/// caller's `ballistic`, the stage is BOOSTING ([`Readings::ballistic`]).
///
/// Each is the databoard's fused read: the first source fresh within the quantity's shared window, in rank order,
/// never extrapolated and, for a vector, never reconciled. So the BMI323 carries `rate` and `accel` the moment the
/// LSM6DSO32 goes stale, and back. A reading that is not finite is handed on as the fused read answers it:
/// [`super::Attitude::tick`] takes it as absent.
///
/// `now_us` is read after the tick's pushes: the databoard takes a push stamped after it as not fresh.
pub fn assemble(quantities: &Quantities, rank: i32, now_us: u64, ballistic: bool) -> Readings {
    let fresh = |parameter: Option<&Parameter<[f32; 3]>>| Some(parameter?.read(now_us)?.value);
    let scalar = |parameter: Option<&Parameter<f32>>| Some(parameter?.read(now_us)?.value);
    let attitude = quantities.attitude;
    let fused = attitude.read(now_us).and_then(|winner| {
        Some(Fused {
            orientation: Orientation::from_sample(winner.value),
            rank: attitude.rank(winner.source)?,
        })
    });
    Readings {
        fused,
        backup_fresh: attitude.fresh_below(rank, now_us),
        ballistic,
        course: scalar(quantities.course),
        ground_speed: scalar(quantities.speed),
        ..Readings::from_marks(
            fresh(quantities.rate),
            fresh(quantities.acceleration),
            fresh(quantities.magnetic),
        )
    }
}

/// Which control ticks step the filter (plan B's task slept `period_ms` between its ticks): every `period_ms` on the
/// uptime clock, judged at each tick's wake within half a tick.
///
/// So a wake that comes a little early still takes its step, and the steps stay on their grid through the ticks'
/// jitter: 20 ms at a 10 ms tick is every second tick, never the third. A period at or below the tick steps on every
/// tick, the fastest the tick can run it. A wake more than half a tick past its due time (a stall, or the first wake,
/// due at 0) starts a new grid a period on from itself, so the grid's edges fall between the wakes, never on one,
/// and no burst of catch-up steps follows: the next step comes at least a period less half a tick later. Two steps
/// are never closer than a period less a tick.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Pace {
    /// The step period, microseconds (`period_ms`).
    period_us: u64,
    /// Half the control tick, microseconds: how early a wake may come and still step, and how late and still keep
    /// the grid.
    slack_us: u64,
    /// When the next step is due, microseconds of uptime: 0 until the first, so the first tick steps.
    next_due_us: u64,
}

impl Pace {
    /// The pace of a filter stepping every `period_ms` on a control tick of `tick_us`.
    pub fn new(period_ms: u32, tick_us: u64) -> Pace {
        Pace {
            period_us: u64::from(period_ms) * 1000,
            slack_us: tick_us / 2,
            next_due_us: 0,
        }
    }

    /// Whether the filter steps on the tick that woke at `now_us`. When it does, the next step is due a period on:
    /// from this step's due time while the wake was on time within half a tick, else from the wake itself.
    pub fn due(&mut self, now_us: u64) -> bool {
        if now_us + self.slack_us < self.next_due_us {
            return false;
        }
        let anchor_us = if now_us > self.next_due_us + self.slack_us {
            now_us
        } else {
            self.next_due_us
        };
        self.next_due_us = anchor_us + self.period_us;
        true
    }
}

/// Whether `channel` is flying on a BACKUP: whether its fused winner `source` is outranked by an ENABLED provider of
/// it (plan B cc_client `_on_backup`, the health flag `attitude-backup`).
///
/// "On the backup" is degraded only where a better-ranked source exists to have fallen off, so the answer comes from
/// the config's ranks, not from a name: the attitude filter was the BNO055's backup until 2026-10-05 and is now the
/// primary, with the SEN0253's BNO055 as ITS backup where one is fitted. A provider configured but disabled (the
/// layout unfitted it) does not count: it was never going to publish. An enabled one counts whether or not it came
/// up, so the BNO055 standing in for a filter that failed its setup is a backup. A source the config does not
/// declare (the HITL sim, `hitl`) is no fallback, and neither is nothing fresh at all (`source` None, a different
/// problem).
///
/// `config` is the active board config as the bring-up read it, after the layout fitted it. Its providers are the
/// enabled devices with their ranks as the bring-up registers them (crates/board_config `directory` and `provides`),
/// so the flag ranks exactly as the databoard does: a priority is Python's int and saturates past i32, and a name
/// given twice takes the later entry's rank, as plan B's walk does. Plan B answers otherwise only on a document it
/// raises on or validate() refuses: its walk counts a provider with no name, which never comes up, and compares a
/// float priority, which the registration reads as 0.
///
/// True also when `config` is not an object and something is fresh, as plan B answers a config it cannot read: an
/// unknown never silently hides a real fallback.
pub fn on_backup(config: &Value, channel: &str, source: Option<&str>) -> bool {
    let Some(source) = source else {
        return false;
    };
    if !config.is_object() {
        return true;
    }
    let mut best: Option<i32> = None;
    let mut flying = None;
    for entry in board_config::directory(config) {
        let Some(provide) = board_config::provides(&entry)
            .into_iter()
            .find(|provide| provide.quantity == channel)
        else {
            continue;
        };
        best = Some(best.map_or(provide.priority, |best| best.min(provide.priority)));
        if entry.name == source {
            flying = Some(provide.priority);
        }
    }
    flying.zip(best).is_some_and(|(flying, best)| flying > best)
}

#[cfg(test)]
mod tests;
