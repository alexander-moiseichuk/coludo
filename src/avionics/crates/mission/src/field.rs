//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The CC-less field agent (plan B tasks/field.py; doc/specs/coludo.md "Field operation without CC"), disabled by
//! default. On the pad (SETTING) it makes at most two decisions:
//!  1. the site by GPS: on the first fresh fix the mission adopts the nearest known site within `max_range_m`
//!     ([`Mission::select_site`]), else the synthesized fallback zone at the clear-sector bearing
//!     ([`Mission::fallback_zone`]);
//!  2. the auto-arm (opt-in): once the board sat still (a fresh accel inside the still band) with a fresh fix for
//!     `auto_arm_dwell_s`, with a zone set and not armed, [`Field::tick`] asks for the arm. The caller runs the arm's
//!     gates and arms.
//!
//! The port's own: plan B armed the Controller directly, past every gate of the CC `arm`. Here the caller keeps the
//! passive gates (the operator, 2026-10-11) and, when one refuses, calls [`Field::refused`], which restarts the dwell:
//! the agent asks again after another whole dwell, where plan B's direct arm never failed.
//!
//! A knob plan B would run with but that makes no sense refuses the entry, naming the key ([`Settings::from_config`]).

use control::navigation::Position;
use serde_json::{Map, Value};

use crate::Mission;

/// The field entry's knobs (plan B field.py setup), each with plan B's default.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Settings {
    /// The first fresh fix picks the site (`site_select`, true).
    pub site_select: bool,
    /// The fallback zone's bearing from the pad, degrees: the operator's clear sector (`fallback_bearing_deg`, 0).
    pub fallback_bearing_deg: f32,
    /// The gap from the pad to the fallback zone's near edge, m (`fallback_near_m`, 50).
    pub fallback_near_m: f32,
    /// The fallback zone's wide side, facing the pad, m (`fallback_width_m`, 100).
    pub fallback_width_m: f32,
    /// How far the fallback zone runs away from the pad, m (`fallback_depth_m`, 90).
    pub fallback_depth_m: f32,
    /// Arm once still with a fix for the dwell (`auto_arm`, false: arming is normally a human act).
    pub auto_arm: bool,
    /// The dwell, ms (`auto_arm_dwell_s` x 1000, truncated as plan B's int(); 60 s).
    pub arm_dwell_ms: u64,
    /// The still band's half-width around 1 g (`still_g`, 0.3, the stage machine's).
    pub still_g: f32,
    /// The pass period, ms (`period_ms`, 1000).
    pub period_ms: u32,
}

impl Settings {
    /// The knobs from the entry's fields; a missing key takes plan B's default.
    ///
    /// Errors: `<key>: <value> <why>` for a key of the wrong type, a negative distance or dwell, a still band
    /// outside 0..1 g, or a period under 1 ms.
    pub fn from_config(fields: &Map<String, Value>) -> Result<Settings, String> {
        let flag = |key: &str, default: bool| match fields.get(key) {
            None => Ok(default),
            Some(value) => value
                .as_bool()
                .ok_or_else(|| format!("{key}: {value} is not true or false")),
        };
        let number = |key: &str, default: f64, least: f64, most: f64| match fields.get(key) {
            None => Ok(default),
            Some(value) => value
                .as_f64()
                .filter(|number| (least..=most).contains(number))
                .ok_or_else(|| format!("{key}: {value} is not a number from {least} to {most}")),
        };
        let period_ms = match fields.get("period_ms") {
            None => 1000,
            Some(value) => value
                .as_u64()
                .and_then(|period| u32::try_from(period).ok())
                .filter(|period| *period >= 1)
                .ok_or_else(|| format!("period_ms: {value} is not a whole number from 1"))?,
        };
        Ok(Settings {
            site_select: flag("site_select", true)?,
            fallback_bearing_deg: number("fallback_bearing_deg", 0.0, -360.0, 360.0)? as f32,
            fallback_near_m: number("fallback_near_m", 50.0, 0.0, f64::from(f32::MAX))? as f32,
            fallback_width_m: number("fallback_width_m", 100.0, 0.0, f64::from(f32::MAX))? as f32,
            fallback_depth_m: number("fallback_depth_m", 90.0, 0.0, f64::from(f32::MAX))? as f32,
            auto_arm: flag("auto_arm", false)?,
            arm_dwell_ms: (number("auto_arm_dwell_s", 60.0, 0.0, 1.0e9)? * 1000.0) as u64,
            still_g: number("still_g", 0.3, 0.0, 1.0)? as f32,
            period_ms,
        })
    }
}

/// What the site decision did on a tick, for the log line.
#[derive(Clone, Debug, PartialEq)]
pub enum SitePick {
    /// A known site was in range: its name.
    Site(String),
    /// None was: the fallback zone was synthesized.
    Fallback,
}

/// One tick's decisions.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Tick {
    /// The site decision, on the tick that made it.
    pub site: Option<SitePick>,
    /// The dwell is complete: the caller runs the arm's gates and arms, or calls [`Field::refused`].
    pub arm: bool,
}

/// The field agent's state.
#[derive(Clone, Debug, PartialEq)]
pub struct Field {
    /// The knobs.
    settings: Settings,
    /// The site decision was made (or is off).
    site_done: bool,
    /// When the stationary-with-a-fix dwell began, ms; None while it is not running.
    still_since_ms: Option<u64>,
}

impl Field {
    /// The agent with `settings`, nothing decided yet.
    pub fn new(settings: Settings) -> Field {
        Field {
            settings,
            site_done: !settings.site_select,
            still_since_ms: None,
        }
    }

    /// The knobs.
    pub fn settings(&self) -> &Settings {
        &self.settings
    }

    /// One decision step at `now_ms` (plan B `_tick`), only in SETTING (`setting`): the site by the first fresh
    /// `fix`, then, with the auto-arm on, unarmed and a zone set, the dwell over a fresh `accel` (g) inside the still
    /// band. A missing fix or accel, or one outside the band, restarts the dwell.
    pub fn tick(
        &mut self,
        now_ms: u64,
        setting: bool,
        armed: bool,
        fix: Option<Position>,
        accel: Option<[f32; 3]>,
        mission: &mut Mission,
    ) -> Tick {
        let mut tick = Tick::default();
        if !setting {
            return tick;
        }
        let Some(fix) = fix else {
            self.still_since_ms = None; // arming needs a live fix through the whole dwell
            return tick;
        };
        if !self.site_done {
            self.site_done = true;
            tick.site = Some(match mission.select_site(fix) {
                Some(name) => SitePick::Site(name),
                None => {
                    let settings = &self.settings;
                    mission.fallback_zone(
                        fix,
                        settings.fallback_bearing_deg,
                        settings.fallback_near_m,
                        settings.fallback_width_m,
                        settings.fallback_depth_m,
                    );
                    SitePick::Fallback
                }
            });
        }
        if !self.settings.auto_arm || armed || mission.zone.is_none() {
            return tick; // no zone: a blind flight is a human decision
        }
        let Some([x, y, z]) = accel else {
            self.still_since_ms = None; // no fresh accel cannot claim stillness
            return tick;
        };
        let squared = x * x + y * y + z * z;
        let (low, high) = (1.0 - self.settings.still_g, 1.0 + self.settings.still_g);
        if !(low * low < squared && squared < high * high) {
            self.still_since_ms = None; // moving: carried or handled
            return tick;
        }
        match self.still_since_ms {
            None => self.still_since_ms = Some(now_ms),
            Some(since) => tick.arm = now_ms.saturating_sub(since) >= self.settings.arm_dwell_ms,
        }
        tick
    }

    /// The arm the dwell asked for was refused: the dwell restarts, so the agent asks again after another whole one.
    pub fn refused(&mut self) {
        self.still_since_ms = None;
    }
}
