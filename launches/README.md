# Launches

One folder per launch day, `YYYYMMDD`, holding the flight manifest and **the board configs as they
flew**; after the day, also its flight data, plots and video. The next launch has its dated folder from
the moment its date is set.

Configs live here rather than in a shared `configs/` because a profile is only meaningful beside the
airframe it was written for: the masses, the motor, the fin setting and the electronics fitted all
change from launch to launch, and a config that outlives them is a config nobody can trust. Regenerate
into a launch with `python3 tools/make_telemetry_config.py --launch <YYYYMMDD>` (default: the newest).
It writes the v0.1 profiles only (7C, 7D, 7D control) — keyless `layout` (declared v0.1) and the
VL53L4CX as their one enabled laser; 7E/7F are hand-kept. **Every board enables exactly one laser
entry**: an unfitted one fails `verify` and `arm`.

**Never regenerate into a flown launch:** the default is the newest dated folder, now `20261205` (TMS-8, whose
config is hand-kept for the Rust avionics: the generator's v0.1 profiles do not belong there either). Pass
`--launch` explicitly; a flown folder's configs are the record.

| launch | status |
|---|---|
| [20261205](20261205/TMS-8/README.md) | **draft** — TMS-8's board config (Rust avionics, the lean boost law): the visual fin trims, `final_approach_agl` and the manifest pending |
| [20261003](20261003/README.md) | **flown** — [TMS-7](20261003/TMS-7/README.md) booster test: success, full flight logged; TMS-7C looped in the boost and crashed ([onboard video](20261003/TMS-7C/video.mp4)); TMS-7D and TMS-7E never separated; TMS-7F not assembled in time |
| ~~20260905~~ | **cancelled — weather** |

Motor masses used throughout (`doc/hardware.md`): **F15-4 98.8 g** (measured), **E16 ~82.5 g** loaded
(approximate). Airframe masses in each manifest are **without** the motor; the totals add it.
