# Launches

One folder per launch day, `YYYYMMDD`, holding the flight manifest and **the board configs as they
flew**.

Configs live here rather than in a shared `configs/` because a profile is only meaningful beside the
airframe it was written for: the masses, the motor, the fin setting and the electronics fitted all
change from launch to launch, and a config that outlives them is a config nobody can trust. Regenerate
into a launch with `python3 tools/make_telemetry_config.py --launch <YYYYMMDD>` (default: the newest).

| launch | status |
|---|---|
| [20261003](20261003/) | **planned** — five airframes + one booster-only test |
| ~~20260905~~ | **cancelled — weather** |

Motor masses used throughout, both measured (`doc/hardware.md`): **F15-4 98.8 g**, **E16 ~82.5 g**
loaded. Airframe masses in each manifest are **without** the motor; the totals add it.
