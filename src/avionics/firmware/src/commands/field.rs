//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The CC-less field agent on the executor (M11; crates/mission field, plan B tasks/field.py). The executor owns the
//! mission the agent changes, so it ticks the agent itself, every `period_ms`, between command lines: the site by
//! GPS on the first fresh fix, and the auto-arm once the dwell is complete. The auto-arm keeps the CC `arm`'s passive
//! gates (the operator, 2026-10-11): the devices, the stage, the mission and the recorder's ring gate, and not the
//! probe sweep, which would move the fins with nobody there. A refusal logs every problem and restarts the dwell.

use std::time::{Duration, Instant};

use mission::field::{Field, SitePick};

use super::Context;
use crate::devices::FieldMember;
use crate::status::FLIGHT;

/// The field agent once its entry came up.
pub struct Agent {
    /// The entry's name: its log lines'.
    name: &'static str,
    /// The decisions (crates/mission field).
    field: Field,
    /// Its pass period.
    period: Duration,
    /// When its next pass is due.
    due: Instant,
}

impl Agent {
    /// The agent of `member`, its first pass a period from now (plan B sleeps before its first tick).
    pub fn new(member: FieldMember) -> Agent {
        let period = Duration::from_millis(u64::from(member.settings.period_ms));
        Agent {
            name: member.name,
            field: Field::new(member.settings),
            period,
            due: Instant::now() + period,
        }
    }

    /// How long until its next pass.
    pub fn wait(&self) -> Duration {
        self.due.saturating_duration_since(Instant::now())
    }
}

/// The agent's pass when it is due: the decisions, their log lines, and the auto-arm through the passive gates.
pub fn tick(context: &mut Context) {
    let Some(mut agent) = context.field.take() else {
        return;
    };
    let now = Instant::now();
    if now >= agent.due {
        agent.due = now + agent.period;
        pass(&mut agent, context);
    }
    context.field = Some(agent);
}

/// One pass of `agent` over `context`'s mission.
fn pass(agent: &mut Agent, context: &mut Context) {
    // SAFETY: a plain read of the monotonic timer.
    let now_us = unsafe { esp_idf_svc::sys::esp_timer_get_time() } as u64;
    let flight = FLIGHT.load();
    let fix = crate::gnss::position(context.topology);
    // a fresh reading only: a dead accelerometer must never look still
    let accel = context
        .topology
        .vector("accel")
        .and_then(|parameter| parameter.read(now_us))
        .map(|reading| reading.value);
    let tick = agent.field.tick(
        now_us / 1000,
        flight.stage == sequencer::Stage::Setting,
        flight.armed,
        fix,
        accel,
        &mut context.mission,
    );
    match tick.site {
        Some(SitePick::Site(site)) => log::info!("{} :: site by GPS: {site}", agent.name),
        Some(SitePick::Fallback) => log::info!(
            "{} :: no site within {:.0}m -> fallback zone (bearing {:.0})",
            agent.name,
            context.mission.max_range_m(),
            agent.field.settings().fallback_bearing_deg
        ),
        None => {}
    }
    if tick.arm {
        let dwell_s = agent.field.settings().arm_dwell_ms / 1000;
        match super::control::auto_arm(context) {
            Ok(()) => log::info!("{} :: auto-armed: stationary with a fix for {dwell_s} s", agent.name),
            Err(problems) => {
                log::warn!("{} :: auto-arm refused: {problems}; the dwell restarts", agent.name);
                agent.field.refused();
            }
        }
    }
}
