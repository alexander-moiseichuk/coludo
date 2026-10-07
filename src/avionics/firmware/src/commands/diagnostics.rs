//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! probe / verify / calibrate / detect / bustune -- the self-tests and the pre-flight check (src/glider cc_client
//! `_register_diagnostics`). The I2C parts are probed on their bus owners (crate::owner); the SPI part's probe and
//! the calibrations arrive with the control thread and their drivers (port plan M6-M10), and until then the answers
//! say so instead of passing.

use std::collections::BTreeMap;

use cc_protocol::Message;
use serde::Serialize;
use serde_json::json;

use super::{Code, Context, Fault, Reply};

/// What a probe or a bustune of the SPI bus answers until SPI2's owner exists.
const NO_SPI_OWNER: &str = "comes with the control thread, SPI2's owner (port plan M6)";

/// The I2C parts probed on both ports' owners, the one named or every one, in port order (plan B's
/// `Inspector.probe_all` over the device tasks), and why an owner did not answer, under its port's name.
fn probe_parts(name: Option<&str>) -> (Vec<crate::owner::Verdict>, Vec<(&'static str, String)>) {
    let mut verdicts = Vec::new();
    let mut errors = Vec::new();
    for (port, label) in [(0, "i2c0"), (1, "i2c1")] {
        match crate::owner::probe_port(port, name) {
            Ok(probed) => verdicts.extend(probed),
            Err(error) => {
                log::warn!("probe :: {error}");
                errors.push((label, error));
            }
        }
    }
    (verdicts, errors)
}

/// `probe [name|all]`: {name: null | problem} for the objects with a self-test, as plan B answers it
/// (cc_client.py:729-780). `probe all` lists every device that came up (null, or its crash) and every one that never
/// did (its failure line), and, when there are alternatives, their not-fitted lines in a second map; `probe
/// <alternative>` answers null for it, then that map. The SPI LSM6DSO32's probe waits for SPI2's owner (M6): `probe
/// all` lists it as up, `probe imu_lsm6dso32` says so.
pub fn probe(message: &Message, context: &mut Context) -> Reply {
    let table = context.devices;
    let target = message.args.first().map_or("all", String::as_str);
    let json = |value: serde_json::Value| value.to_string();
    if target == "all" {
        let mut results: BTreeMap<&str, Option<String>> = BTreeMap::new();
        results.insert("mission", probe_mission(context));
        let (verdicts, errors) = probe_parts(None);
        for (name, verdict) in verdicts {
            results.insert(name, table.probed(name, verdict.as_deref()));
        }
        // an owner that did not answer leaves its parts unprobed: say so under its port's name
        for (label, error) in errors {
            results.insert(label, Some(error));
        }
        // the rest that came up have no probe of their own here (plan B's base Task.probe: healthy unless crashed)
        for (name, _) in table.tasks() {
            results.entry(name).or_insert_with(|| table.probed(name, None));
        }
        for (name, line) in table.failures() {
            results.entry(name).or_insert(Some(line));
        }
        let alternatives: BTreeMap<&str, String> = table.alternatives().into_iter().collect();
        let first = json(json!(results));
        return Ok(if alternatives.is_empty() {
            cc_protocol::build("ok", &[&first], &[])
        } else {
            cc_protocol::build("ok", &[&first, &json(json!(alternatives))], &[])
        });
    }
    if target == "mission" {
        return super::ok_json(&json!({"mission": probe_mission(context)}));
    }
    let entry = table.find(target);
    if entry.is_some_and(|entry| entry.implementation == "lsm6dso32" && entry.active()) {
        return Err(Fault::new(
            Code::Unsupported,
            format!("{target}'s probe {NO_SPI_OWNER}"),
        ));
    }
    let (verdicts, errors) = if entry.is_some_and(|entry| entry.socket.is_some()) {
        probe_parts(Some(target))
    } else {
        (Vec::new(), Vec::new())
    };
    if let Some((name, verdict)) = verdicts.into_iter().next() {
        return super::ok_json(&json!({name: table.probed(name, verdict.as_deref())}));
    }
    if entry.is_some_and(|entry| entry.active()) {
        if !errors.is_empty() {
            let errors: Vec<String> = errors.into_iter().map(|(_, error)| error).collect();
            return Err(Fault::new(Code::Internal, errors.join("; ")));
        }
        return super::ok_json(&json!({target: table.probed(target, None)}));
    }
    if let Some(line) = table.failure(target) {
        return super::ok_json(&json!({target: line}));
    }
    if let Some(line) = table.unfitted(target) {
        return Ok(cc_protocol::build(
            "ok",
            &[&json(json!({target: null})), &json(json!({target: line}))],
            &[],
        ));
    }
    Err(Fault::new(Code::BadArguments, format!("no probe for {target}")))
}

/// The failing probes of the I2C parts, named as plan B's verify and arm name them (`_named`), and an owner that did
/// not answer, under its port's name.
pub fn probe_problems(context: &Context) -> Vec<(&'static str, String)> {
    let (verdicts, errors) = probe_parts(None);
    verdicts
        .into_iter()
        .filter_map(|(name, verdict)| Some((name, context.devices.probed(name, Some(&verdict?))?)))
        .chain(errors)
        .collect()
}

/// `verify`: the pre-flight check -- devices, problems, and the flight-readiness verdict. Never passes until the
/// flight loop exists.
pub fn verify(_message: &Message, context: &mut Context) -> Reply {
    let devices = context.devices;
    let mut problems: BTreeMap<&str, String> = devices.failures().into_iter().collect();
    problems.insert("flight", super::control::NOT_PORTED.into());
    if let Some(problem) = super::stage_problem() {
        problems.insert("stage", problem);
    }
    match super::outside_setting() {
        Some(airborne) => {
            problems.insert("probe", format!("not run: {airborne} (the probes sweep the fins)"));
        }
        None => {
            if let Some(problem) = probe_mission(context) {
                problems.insert("mission", problem);
            }
            problems.extend(probe_problems(context));
        }
    }
    let readiness = json!({"flight": "not ported yet: fins hold neutral -- ballistic"});
    super::ok_json(&json!({
        "pass": problems.is_empty(),
        "devices": devices.devices().into_iter().collect::<BTreeMap<_, _>>(),
        "problems": problems,
        "alternatives": devices.alternatives().into_iter().collect::<BTreeMap<_, _>>(),
        "ready": false,
        "readiness": readiness,
    }))
}

/// The mission's self-test (a launch position is set, the zone is in range), logged as plan B's mission.probe logs
/// it, so an operator's log window shows the check and its verdict.
pub fn probe_mission(context: &Context) -> Option<String> {
    log::info!("mission :: probe: launch position ...");
    // plan B logs the position as soon as it resolves, before the zone range check can fail
    if let Some(origin) = context.mission.launch_point(None) {
        let [latitude, longitude] = origin.degrees();
        log::info!("mission :: probe: position ok ({latitude:.5}, {longitude:.5})");
    }
    let problem = context.mission.probe(None);
    if let Some(problem) = &problem {
        log::warn!("mission :: probe FAILED: {problem}");
    }
    problem
}

/// `calibrate`: what needs calibrating (nothing yet); `calibrate <device>` runs one (no device has one yet).
pub fn calibrate(message: &Message, _context: &mut Context) -> Reply {
    let Some(target) = message.args.first() else {
        return super::ok_json(&json!({}));
    };
    if let Some(airborne) = super::outside_setting() {
        // a baro calibrate mid-air re-zeroes elevation: the ground, to the stage machine
        return Err(Fault::new(Code::Unsafe, airborne));
    }
    Err(Fault::new(Code::BadArguments, format!("no calibrate for {target}")))
}

/// The `detect` payload, in plan B's key order.
#[derive(Serialize)]
struct Detected<'a> {
    /// The revision the buses look like now, null when undecided.
    detected: Option<&'static str>,
    /// The revision this boot runs as (health's `layout`).
    applied: &'a str,
    /// The scan's evidence.
    detail: String,
}

/// `detect`: scan both I2C buses, on their owners, and answer the revision they look like beside the one this boot
/// applied, changing nothing (plan B cc_client `detect`): what is wired now, which takes effect only at the next
/// boot. Scans with the running (post-layout) config's bus specs, as plan B scans the controller's.
pub fn detect(_message: &Message, context: &mut Context) -> Reply {
    let (revision, detail) = layout::detect(&context.config.document, |bus, _| {
        crate::owner::scan(bus).inspect_err(|error| log::warn!("detect :: {error}: no votes from this bus"))
    });
    super::ok_json(&Detected {
        detected: revision.map(layout::Revision::name),
        applied: &context.layout,
        detail,
    })
}

/// `bustune <kind> <id> <freq>`: run sensor bus `<kind>:<id>` at `<freq>` Hz in place, never saved (the CC sweep
/// saves the chosen clock with set-config and a reboot), and answer each part's probe (plan B
/// controller.bustune, cc_client.py:855-870): {kind, id, freq, devices: {name: 'ok' | '<driver> -- <why>'}, all_ok}.
///
/// The verdicts are the parts that came up on that bus, probed now (operator, 2026-10-07): plan B also counted every
/// other config entry on the bus as down -- disabled ones included, and here the parts whose drivers are not ported
/// yet -- so its sweep never found a ceiling on a bus with one.
pub fn bustune(message: &Message, context: &mut Context) -> Reply {
    let [kind, id, freq, ..] = message.args.as_slice() else {
        return Err(Fault::new(Code::BadArguments, "bustune <kind> <id> <freq>"));
    };
    let freq: i64 = freq
        .trim()
        .parse()
        .map_err(|_| Fault::new(Code::BadArguments, "freq must be an int (Hz)"))?;
    if kind != "i2c" && kind != "spi" {
        return super::ok_json(&json!({"error": format!("bus kind '{kind}' is not tunable (i2c/spi only)")}));
    }
    let defined = context.config.document["buses"][kind.as_str()]
        .get(id.as_str())
        .is_some_and(|bus| bus.is_object());
    if !defined {
        return super::ok_json(&json!({"error": format!("bus {kind}:{id} not defined")}));
    }
    if kind == "spi" {
        return Err(Fault::new(Code::Unsupported, format!("spi bustune {NO_SPI_OWNER}")));
    }
    let port: u8 = id
        .parse()
        .map_err(|_| Fault::new(Code::Internal, format!("i2c port {id} has no owner")))?;
    let clock_hz = u32::try_from(freq)
        .ok()
        .filter(|clock_hz| *clock_hz > 0)
        .ok_or_else(|| Fault::new(Code::Internal, format!("{freq} Hz is no clock")))?;
    let verdicts = crate::owner::bustune(port, clock_hz)
        .map_err(|error| Fault::new(Code::Internal, error))?
        .map_err(|error| {
            Fault::new(
                Code::Internal,
                format!("i2c:{port} would not run at {clock_hz} Hz: {error}"),
            )
        })?;
    let devices: BTreeMap<&str, String> = verdicts
        .into_iter()
        .map(|(name, verdict)| {
            // 'ok' is the contract the CC sweep reads; anything else names the part, as probe and verify do
            let line = verdict.map_or_else(
                || "ok".to_string(),
                |why| context.devices.probed(name, Some(&why)).unwrap_or(why),
            );
            (name, line)
        })
        .collect();
    let all_ok = !devices.is_empty() && devices.values().all(|verdict| verdict == "ok");
    super::ok_json(&json!({"kind": kind, "id": id, "freq": freq, "devices": devices, "all_ok": all_ok}))
}
