//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The mission (src/glider mission.py): the per-launch identity the operator sets before a flight. It holds a
//! launch id, the launch site and its position (a known origin), the landing zone, and the known sites for
//! CC-less field operation, as the launch.config document. Unlike the board config (hardware, stable across
//! flights) it changes every launch.
//!
//! This crate is the document and its rules, with no storage and no clock. [`Mission::load`] takes the stored
//! text and [`Mission::persisted`] gives the document to store. [`Mission::inspect`] takes the clock text and
//! epoch from its caller, and the live GNSS fix is an argument too (`fix`: a FRESH fix or None, as plan B's
//! databoard read gives it). A time set is the caller's job: [`epoch_request`] says whether an update asks for
//! one, by plan B's `set_time` rule, and [`TimeSet`] holds the momentary inputs riding with it for the
//! session.csv row.
//!
//! Positions are [`Position`] integers (1e-7 degree; see `control::navigation`). A JSON number is an f64, so a
//! coordinate is f64 at the JSON boundary and nowhere else (decision D18): it is read, range-checked and
//! rounded to units on the way in, and divided back out on the way out. Metres are f32.
//!
//! Deliberate differences from plan B:
//!  * `sites` survive. Plan B's persisted() leaves them out, so every `set-config launch` silently erased the
//!    CC-less site list at the next boot. Here [`Mission::persisted`] keeps them, and [`Mission::merge`] (the
//!    `set-config launch` path) replaces them when its payload carries a `sites` list.
//!  * `launch_id` and `site` are text and `altitude` is metres. Plan B stores whatever JSON value arrives;
//!    here an update of another type is ignored (and not listed as changed), and a stored one loads as unset
//!    ('' or None). A site entry whose `name` is not text is named as if it had none (`site<n>`), where plan B
//!    stored the Python repr of the value.
//!  * Metres and coordinates always go out as JSON floats. Plan B echoes the JSON number it was given, so an
//!    altitude, coordinate or zone corner sent as `5` reads back `5` there and `5.0` here: the same value of
//!    another JSON kind. Keeping the kind would mean storing it beside every number, for no reader that needs
//!    it.
//!  * A position is rounded to the nearest 1e-7 degree (1.1 cm) on the way in. Plan B keeps the f64 it was
//!    given: an update that moves a coordinate by less than half a unit is listed as changed there and not
//!    here, and `persisted`, `inspect` and `get-config launch` echo 7 decimals where plan B echoes all of them
//!    (CC's `assist` sends NMEA-derived degrees with about 15 significant digits).
//!  * Nothing is memoized. Plan B caches the zone geometry by identity for its hot path; here it is a little
//!    integer and f32 work, done when asked.
//!
//! Not yet ported from plan B's Mission: `freeze_launch` (the arm freeze of the launch point, M7),
//! `select_site` and `fallback_zone` (CC-less field operation, M11), and `zone_aspect`/`endgame_heading` (the
//! endgame pattern for guidance, M13; `control::navigation::zone_aspect` already exists). A milestone that
//! needs one ports it here first.

use control::navigation;
use serde_json::{Map, Value, json};
use std::ops::RangeInclusive;

/// A position in 1e-7 degree units, re-exported so a mission's caller needs no `control` of its own.
pub use control::navigation::Position;

/// The default range limit, in metres, from the launch point to any zone point (mission.py
/// `_DEFAULT_MAX_RANGE_M`). The real figure is the board config's `field.max_range_m`: a glide-range property
/// of the airframe.
pub const DEFAULT_MAX_RANGE_M: f32 = 200.0;

/// 2000-01-01T00:00:00Z in Unix seconds. A time set before it is refused: no real flight clock is that early,
/// and the board RTC counts from it (mission.py `_EPOCH_OFFSET`).
const EPOCH_2000: i64 = 946_684_800;

/// A time set's `utc_offset` in minutes spans the zones that exist, UTC-12:00 to UTC+14:00. Past them it is a
/// bad value: the session.csv cell stays empty, while the time set itself still counts.
const UTC_OFFSET_MINUTES: RangeInclusive<i64> = -720..=840;

/// A latitude's limit in degrees, either side of the equator.
const LATITUDE_LIMIT: f64 = 90.0;

/// A longitude's limit in degrees, either side of Greenwich.
const LONGITUDE_LIMIT: f64 = 180.0;

/// Seconds in a UTC day (no leap seconds, as Unix time and the board RTC count).
const SECONDS_PER_DAY: i64 = 86_400;

/// The launch.config a board without one starts from: HPRC (Homestead Public Rocketry Club), the documented
/// test pad and landing zone (mission.py `_HPRC_DEFAULT`). A fresh board then has a valid pad and zone for HPRC
/// field tests. It is JSON text so it goes through the same validation as a stored document.
const HPRC_DEFAULT: &str = r#"{"site": "HPRC", "latitude": 25.514379, "longitude": -80.391795, "altitude": 2.0,
    "zone": [[25.514944, -80.392972], [25.514583, -80.391111]]}"#;

/// A landing zone: two diagonally opposite corners, as the operator set them.
///
/// The corner order is kept as given. The geometry does not depend on it (navigation::zone), but the operator
/// chose it, and the stored document gives it back unchanged.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Zone {
    /// The first corner (launch.config's top-left).
    pub top_left: Position,
    /// The diagonally opposite corner (launch.config's bottom-right).
    pub bottom_right: Position,
}

impl Zone {
    /// A zone as launch.config carries it, `[[latitude, longitude], [latitude, longitude]]` (mission.py
    /// `_zone`): exactly two corners, each two in-range numbers. None for any other shape or value.
    fn from_json(value: &Value) -> Option<Zone> {
        let [top_left, bottom_right] = value.as_array()?.as_slice() else {
            return None;
        };
        Some(Zone {
            top_left: position(top_left)?,
            bottom_right: position(bottom_right)?,
        })
    }

    /// The zone as launch.config writes it, corners in decimal degrees.
    fn to_json(self) -> Value {
        json!([self.top_left.degrees(), self.bottom_right.degrees()])
    }
}

/// A known launch location for CC-less field operation (doc/specs/coludo.md "Field operation without CC"): the
/// board picks the nearest one whose pad is within range of its GNSS fix.
#[derive(Clone, Debug, PartialEq)]
pub struct Site {
    /// The site's name, which becomes the mission's `site` when it is picked.
    pub name: String,
    /// The launch pad.
    pub pad: Position,
    /// The site's landing zone.
    pub zone: Zone,
}

impl Site {
    /// The site as launch.config writes it, `{name, pad: [lat, lon], zone: [[lat, lon], [lat, lon]]}`: the
    /// shape [`sites`] reads back.
    fn to_json(&self) -> Value {
        json!({"name": self.name, "pad": self.pad.degrees(), "zone": self.zone.to_json()})
    }
}

/// The landing zone resolved against the launch point (mission.py `geometry`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Geometry {
    /// The launch point the distances are measured from.
    pub origin: Position,
    /// The zone's centre, where the glider lands.
    pub target: Position,
    /// The midpoints of the zone's two short sides, its entrances.
    pub gates: [Position; 2],
    /// The distances in metres from the origin to the target, gate_a and gate_b, each rounded to 0.1 m as plan
    /// B reports them.
    pub distances_m: [f32; 3],
    /// The farthest of the three distances.
    pub max_distance_m: f32,
    /// Whether every point is within the range limit; out of range fails the pre-flight probe.
    pub in_range: bool,
}

/// The momentary inputs that ride with a time set (`update mission {epoch, ...}`) into its session.csv row:
/// written to the row, never stored (mission.py `_index_time`). An invalid one is None and becomes an empty
/// cell; it never refuses the row or the time set.
#[derive(Clone, Debug, PartialEq)]
pub struct TimeSet {
    /// The setter's offset from UTC in minutes, None outside the zones that exist (-720..=840) or when not an
    /// integer.
    pub utc_offset: Option<i32>,
    /// CC's own position. Never the launch pad, which only `latitude`/`longitude` set.
    pub cc_position: Option<Position>,
    /// What set the time ('cc-auto', 'dashboard'), as given. None when it is not text, the empty cell plan B's
    /// Recorder makes of it. Vetting the text for the cell is the Recorder's job.
    pub source: Option<String>,
}

impl TimeSet {
    /// The time-set inputs of an `update mission` request, each validated as plan B's `_index_time` does.
    pub fn from_properties(properties: &Map<String, Value>) -> TimeSet {
        TimeSet {
            utc_offset: properties
                .get("utc_offset")
                .and_then(Value::as_i64)
                .filter(|minutes| UTC_OFFSET_MINUTES.contains(minutes))
                .and_then(|minutes| i32::try_from(minutes).ok()),
            cc_position: properties.get("cc_position").and_then(position),
            source: text(properties.get("source")),
        }
    }
}

/// The operator-set launch identity: the launch.config document.
///
/// The fields are public to read. Change them through [`Mission::update`] or [`Mission::merge`], which
/// validate as plan B does; a field set directly skips that.
#[derive(Clone, Debug, PartialEq)]
pub struct Mission {
    /// The launch id for this flight; '' when unset.
    pub launch_id: String,
    /// The site name; 'fallback' once a CC-less board has synthesized its own zone.
    pub site: String,
    /// The launch pad latitude in 1e-7 degree; None when unset.
    pub latitude_e7: Option<i32>,
    /// The launch pad longitude in 1e-7 degree; None when unset.
    pub longitude_e7: Option<i32>,
    /// The launch-site elevation in metres; None when unset.
    pub altitude: Option<f32>,
    /// The landing zone; None when unset.
    pub zone: Option<Zone>,
    /// The known launch locations for CC-less operation.
    pub sites: Vec<Site>,
    /// The range limit in metres from the launch point to any zone point (the board config's
    /// `field.max_range_m`).
    max_range_m: f32,
}

impl Mission {
    /// The mission from the stored launch.config text (mission.py `Mission(path, max_range_m)`).
    ///
    /// Never fails. No text, text that is not JSON, or JSON that is not an object gives the HPRC default, so the
    /// board stays usable. An object, even `{}`, is the mission: whatever it lacks or holds invalid is unset.
    pub fn load(stored: Option<&str>, max_range_m: f32) -> Mission {
        let document = stored
            .and_then(|text| serde_json::from_str::<Value>(text).ok())
            .and_then(|value| match value {
                Value::Object(document) => Some(document),
                _ => None,
            })
            .unwrap_or_else(|| serde_json::from_str(HPRC_DEFAULT).expect("the HPRC default is a JSON object"));
        Mission {
            launch_id: text(document.get("launch_id")).unwrap_or_default(),
            site: text(document.get("site")).unwrap_or_default(),
            latitude_e7: coordinate(document.get("latitude"), LATITUDE_LIMIT),
            longitude_e7: coordinate(document.get("longitude"), LONGITUDE_LIMIT),
            altitude: document.get("altitude").and_then(metres),
            zone: document.get("zone").and_then(Zone::from_json),
            sites: document.get("sites").map(sites).unwrap_or_default(),
            max_range_m,
        }
    }

    /// Apply the editable mission fields of an `update mission` request (mission.py `update`, less the clock).
    ///
    /// Takes launch_id, site, latitude, longitude, altitude and zone; anything else is ignored, `sites`
    /// included. An out-of-range coordinate, a null latitude or longitude (the pad cannot be cleared) and an
    /// invalid zone are ignored; a null altitude clears it.
    ///
    /// `epoch` is not applied here: this crate owns no clock. The caller asks [`epoch_request`], sets the clock,
    /// and appends 'epoch' to the list, which is where plan B lists it.
    ///
    /// Returns the names actually changed, in plan B's order (launch_id, site, latitude, longitude, altitude,
    /// zone). An unchanged or ignored field is not listed.
    pub fn update(&mut self, properties: &Map<String, Value>) -> Vec<&'static str> {
        let outcomes = [
            (
                "launch_id",
                replace(&mut self.launch_id, text(properties.get("launch_id"))),
            ),
            ("site", replace(&mut self.site, text(properties.get("site")))),
            (
                "latitude",
                replace(
                    &mut self.latitude_e7,
                    coordinate(properties.get("latitude"), LATITUDE_LIMIT).map(Some),
                ),
            ),
            (
                "longitude",
                replace(
                    &mut self.longitude_e7,
                    coordinate(properties.get("longitude"), LONGITUDE_LIMIT).map(Some),
                ),
            ),
            (
                "altitude",
                replace(&mut self.altitude, properties.get("altitude").and_then(altitude_update)),
            ),
            (
                "zone",
                replace(
                    &mut self.zone,
                    properties.get("zone").and_then(Zone::from_json).map(Some),
                ),
            ),
        ];
        outcomes
            .into_iter()
            .filter_map(|(name, changed)| changed.then_some(name))
            .collect()
    }

    /// Apply a `set-config launch` payload: [`Mission::update`], plus `sites`.
    ///
    /// The rule for `sites` (plan B has none, since its set-config dropped them): a payload whose `sites` is a
    /// JSON list replaces the stored list with its valid entries, validated as at load (an invalid entry is
    /// dropped, so `[]` clears the list). A payload without `sites`, or with one that is not a list, keeps them,
    /// as a merge keeps any field the payload leaves out. 'sites' is listed last when the list changed.
    pub fn merge(&mut self, properties: &Map<String, Value>) -> Vec<&'static str> {
        let mut changed = self.update(properties);
        let replacement = properties.get("sites").filter(|value| value.is_array()).map(sites);
        if replace(&mut self.sites, replacement) {
            changed.push("sites");
        }
        changed
    }

    /// Whether the operator set the launch position (both coordinates): the dashboard's 'launchpad_set'.
    pub fn position_set(&self) -> bool {
        self.latitude_e7.is_some() && self.longitude_e7.is_some()
    }

    /// The launch origin (mission.py `launch_point`): the operator-set position when both coordinates are set,
    /// else the live GNSS `fix`, else None.
    pub fn launch_point(&self, fix: Option<Position>) -> Option<Position> {
        match (self.latitude_e7, self.longitude_e7) {
            (Some(latitude_e7), Some(longitude_e7)) => Some(Position {
                latitude_e7,
                longitude_e7,
            }),
            _ => fix,
        }
    }

    /// The landing zone resolved against the launch point (mission.py `geometry`); None when the zone or the
    /// launch point is unset.
    pub fn geometry(&self, fix: Option<Position>) -> Option<Geometry> {
        let zone = self.zone?;
        let origin = self.launch_point(fix)?;
        let (target, gate_a, gate_b) = navigation::zone(zone.top_left, zone.bottom_right);
        let distances_m = [target, gate_a, gate_b].map(|point| tenths(navigation::distance(origin, point)));
        let max_distance_m = distances_m.into_iter().fold(0.0, f32::max);
        Some(Geometry {
            origin,
            target,
            gates: [gate_a, gate_b],
            distances_m,
            max_distance_m,
            in_range: max_distance_m <= self.max_range_m,
        })
    }

    /// The pre-flight self-test (mission.py `probe`): a launch position is known (set, or the live fix) and, when
    /// a landing zone is set, all three of its points are within range.
    ///
    /// Returns None when it passes, else plan B's message naming what failed.
    pub fn probe(&self, fix: Option<Position>) -> Option<String> {
        if self.launch_point(fix).is_none() {
            return Some("launch/zone: launch position not set (CC or GNSS)".to_owned());
        }
        let geometry = self.geometry(fix)?;
        (!geometry.in_range).then(|| {
            format!(
                "launch/zone: landing zone out of range ({:.0} m > {:.0} m)",
                geometry.max_distance_m, self.max_range_m
            )
        })
    }

    /// The operator-facing snapshot (`inspect mission`; mission.py `inspect`), with plan B's keys and meanings.
    ///
    /// `launch_point` is the origin the board would fly from right now, and `launch_source` says where it came
    /// from: 'set' (CC-set, or frozen at arm), 'gnss' (the live fix), or null. `target`, `gates`, `distances_m`
    /// and `in_range` are present only when the zone resolves against an origin. `clock` and `epoch` are the
    /// caller's board clock, passed through.
    pub fn inspect(&self, fix: Option<Position>, clock: &str, epoch: i64) -> Value {
        let origin = self.launch_point(fix);
        let launch_source = if self.position_set() {
            Some("set")
        } else {
            origin.map(|_| "gnss")
        };
        let mut snapshot = json!({
            "launch_id": self.launch_id,
            "site": self.site,
            "latitude": self.latitude_e7.map(navigation::degrees_from_e7),
            "longitude": self.longitude_e7.map(navigation::degrees_from_e7),
            "altitude": self.altitude.map(json_metres),
            "launch_point": origin.map(Position::degrees),
            "launch_source": launch_source,
            "zone": self.zone.map(Zone::to_json),
            "clock": clock,
            "epoch": epoch,
        });
        if let Some(geometry) = self.geometry(fix) {
            let [target, gate_a, gate_b] = geometry.distances_m.map(json_metres);
            snapshot["target"] = json!(geometry.target.degrees());
            snapshot["gates"] = json!(geometry.gates.map(Position::degrees));
            snapshot["distances_m"] = json!({"target": target, "gate_a": gate_a, "gate_b": gate_b});
            snapshot["in_range"] = json!(geometry.in_range);
        }
        snapshot
    }

    /// The document as launch.config stores it: launch_id, site, latitude, longitude, altitude, zone and
    /// sites. No computed geometry and no clock. `get-config launch` answers it, and the dashboard's editor sends
    /// it back through `set-config launch`.
    ///
    /// `sites` is the deliberate addition to plan B's persisted() (see the crate notes). Coordinates go out in
    /// decimal degrees, so 25.514379 prints as 25.514379 (an f64 made at the JSON boundary; see
    /// `navigation::degrees_from_e7`).
    pub fn persisted(&self) -> Value {
        json!({
            "launch_id": self.launch_id,
            "site": self.site,
            "latitude": self.latitude_e7.map(navigation::degrees_from_e7),
            "longitude": self.longitude_e7.map(navigation::degrees_from_e7),
            "altitude": self.altitude.map(json_metres),
            "zone": self.zone.map(Zone::to_json),
            "sites": self.sites.iter().map(Site::to_json).collect::<Vec<Value>>(),
        })
    }
}

/// The epoch an `update mission` request asks the clock to be set to, by plan B's `set_time` rule: an integer
/// (a bool, a float or text is not) of Unix seconds no earlier than 2000-01-01. None when there is no such
/// request.
///
/// The caller sets its clock from it and lists 'epoch' as changed. A time set also writes the boot's
/// session.csv row from [`TimeSet::from_properties`].
pub fn epoch_request(properties: &Map<String, Value>) -> Option<i64> {
    properties
        .get("epoch")
        .and_then(Value::as_i64)
        .filter(|epoch| *epoch >= EPOCH_2000)
}

/// A Unix epoch as the board clock's text, 'YYYY-MM-DDTHH:MM:SS' in UTC (mission.py `clock`, which reads the
/// board's UTC RTC).
pub fn clock_text(epoch: i64) -> String {
    let (year, month, day) = civil_from_days(epoch.div_euclid(SECONDS_PER_DAY));
    let seconds = epoch.rem_euclid(SECONDS_PER_DAY);
    format!(
        "{year:04}-{month:02}-{day:02}T{:02}:{:02}:{:02}",
        seconds / 3600,
        seconds % 3600 / 60,
        seconds % 60
    )
}

/// The proleptic Gregorian (year, month, day) of a count of days since 1970-01-01.
///
/// Howard Hinnant's civil_from_days: shift the year to start in March, so the leap day ends it, then split into
/// 400-year eras of 146097 days, which repeat exactly. Exact for every i64 day count this crate can be given,
/// with no date library.
fn civil_from_days(days: i64) -> (i64, i64, i64) {
    let shifted = days + 719_468; // days since 0000-03-01
    let era = shifted.div_euclid(146_097);
    let day_of_era = shifted.rem_euclid(146_097); // 0..=146096
    let year_of_era = (day_of_era - day_of_era / 1460 + day_of_era / 36_524 - day_of_era / 146_096) / 365; // 0..=399
    let day_of_year = day_of_era - (365 * year_of_era + year_of_era / 4 - year_of_era / 100); // 0..=365, from 1 March
    let month_from_march = (5 * day_of_year + 2) / 153; // 0..=11
    let day = day_of_year - (153 * month_from_march + 2) / 5 + 1;
    let month = if month_from_march < 10 {
        month_from_march + 3
    } else {
        month_from_march - 9
    };
    (year_of_era + era * 400 + i64::from(month <= 2), month, day)
}

/// Store `value` in `field` when there is one and it differs, and say whether it did: plan B lists a field as
/// changed only then.
fn replace<T: PartialEq>(field: &mut T, value: Option<T>) -> bool {
    match value {
        Some(value) if value != *field => {
            *field = value;
            true
        }
        _ => false,
    }
}

/// A JSON text value, None for any other value or none.
fn text(value: Option<&Value>) -> Option<String> {
    value.and_then(Value::as_str).map(str::to_owned)
}

/// A JSON number of metres as f32 (a JSON number is f64 only at this boundary); None for anything else.
fn metres(value: &Value) -> Option<f32> {
    value.as_f64().map(|number| number as f32)
}

/// An `altitude` update: a number sets it, null clears it (Some(None)), anything else is ignored (None).
fn altitude_update(value: &Value) -> Option<Option<f32>> {
    if value.is_null() {
        Some(None)
    } else {
        metres(value).map(Some)
    }
}

/// A coordinate in 1e-7 degree when `value` is a JSON number at most `limit` degrees either side of zero
/// (mission.py `_number`; a JSON bool is not a number), else None.
fn coordinate(value: Option<&Value>, limit: f64) -> Option<i32> {
    value
        .and_then(Value::as_f64)
        .filter(|degrees| (-limit..=limit).contains(degrees))
        .map(navigation::e7_from_degrees)
}

/// A position as JSON carries one, `[latitude, longitude]` in decimal degrees: exactly two in-range numbers.
/// None otherwise. Zone corners, site pads and CC's position all take this rule in plan B.
fn position(value: &Value) -> Option<Position> {
    let [latitude, longitude] = value.as_array()?.as_slice() else {
        return None;
    };
    Some(Position {
        latitude_e7: coordinate(Some(latitude), LATITUDE_LIMIT)?,
        longitude_e7: coordinate(Some(longitude), LONGITUDE_LIMIT)?,
    })
}

/// The valid entries of a launch.config `sites` list (mission.py `_sites`): each an object with a valid `pad`
/// and `zone`. An invalid entry is dropped, never raised, because a typo in one site must not cost the list. A
/// value that is not a list gives none. An entry without a text `name` is named `site<n>`, n counting the valid
/// entries before it.
fn sites(value: &Value) -> Vec<Site> {
    let mut valid: Vec<Site> = Vec::new();
    for entry in value.as_array().into_iter().flatten().filter_map(Value::as_object) {
        let pad = entry.get("pad").and_then(position);
        let zone = entry.get("zone").and_then(Zone::from_json);
        let (Some(pad), Some(zone)) = (pad, zone) else {
            continue;
        };
        let name = text(entry.get("name")).unwrap_or_else(|| format!("site{}", valid.len()));
        valid.push(Site { name, pad, zone });
    }
    valid
}

/// Metres rounded to 0.1 m, ties to even, as plan B's `round(distance, 1)` reports them.
fn tenths(metres: f32) -> f32 {
    (metres * 10.0).round_ties_even() / 10.0
}

/// An f32 of metres as a JSON number, written with its shortest decimal: 37.1 goes out as 37.1, not as the
/// 37.099998474121094 its f64 widening prints. The f64 is made only for JSON (decision D18's boundary).
fn json_metres(metres: f32) -> Value {
    metres
        .to_string()
        .parse::<f64>()
        .ok()
        .and_then(serde_json::Number::from_f64)
        .map_or(Value::Null, Value::Number)
}

#[cfg(test)]
mod tests;
