# TMS-7 attitude — BNO055 backup + loiter board provenance (one HITL round)

The attitude-redundancy capture set (plan item 4), flown on the **measured TMS-7 v3 weights** with the
**tuned loiter + endgame-spiral law** on the real ESP32-P4 (on-board HITL). One round closes two
things at once — the flights are the same tuned-law glides, differing only in whether the BNO055 is
killed mid-glide:

- **Loiter board provenance** (4 nominal cases) — the loiter law had only *host* provenance
  (`TMS-7-loiter`); these confirm it on-device, both motors, both weights.
- **Attitude backup** (2 drop cases) — the sim's `attitude` is cut mid-glide (a BNO055 death), so the
  priority-1 complementary-filter backup (`tasks/attitude.py`) must carry the glide to a controlled
  landing. Calm, 5 % noise, `inject_hz=25`, quality-2 polar (the worst-case floor).

## The six cases

| case | stack | miss | in-zone | max-from-pad | glide | what it proves |
|---|---|---|---|---|---|---|
| [E16 full](e16_full.html) | 450 g | 61 m | **yes** | 120 m | 36.4 s | loiter on-board (E16, full) |
| [E16 light](e16_half.html) | 400 g | 53 m | **yes** | 120 m | 41.7 s | loiter on-board (E16, light) |
| [F15 full](f15_full.html) | 467 g | 55 m | **yes** | 122 m | 64.1 s | loiter on-board (F15, full) |
| [F15 light](f15_half.html) | 417 g | 52 m | **yes** | 122 m | 72.2 s | loiter on-board (F15, light) |
| [E16 + BNO055 drop](e16_full_drop.html) | 450 g | **15 m** | **yes** | 122 m | 33.7 s | backup flies E16 to landing |
| [F15 + BNO055 drop](f15_full_drop.html) | 467 g | 62 m | no (near) | 122 m | ~62 s | backup flies F15 to landing |

🎬 **[`tms7_attitude.mp4`](tms7_attitude.mp4)** — the follow-cam movie of all six, FHD 30 FPS, each
segment titled with what is flying and (for the drop cases) when the BNO055 dies.

## Attitude backup — validated, and it flies the glider home

The backup derives (heading, roll, pitch) from the LSM6DSO32 gyro `rate` + accel gravity vector and
provides `attitude` at **priority 1**; the databoard's timeout handoff (40 ms) swaps to it the instant
the BNO055 (priority 0) stops — no `flight.py` change. It **mirrors** the BNO055 while that is fresh
(so the handoff has no transient) and **free-runs** the filter only once it is lost.

The direct backup-vs-truth numbers come from `tools/attitude_soak.py` (which logs both): after the
drop the source flips `hitl → attitude` seamlessly and the backup tracks truth to **~1° roll /
~0.5° pitch through the whole loiter**, flying to DONE on the backup. The two drop cases here are the
capture-set confirmation: with the sole attitude source dead, both still land under control —
**E16 15 m in-zone**, F15 62 m near the zone. Without the backup a mid-glide attitude loss goes to
stale attitude → neutral fins → ballistic; landing near the zone *is* the proof the backup carried it.

**The finding that shaped the filter — the coordinated-turn illusion.** The first dropout run
*diverged*: the estimate rolled toward wings-level (roll error 2° → 21°) while the glider banked
harder into the loiter. In a coordinated turn the accelerometer reads gravity+centripetal **down the
body axis**, so its roll/pitch look level at any bank, and the load-factor band (0.7–1.3 g) does not
catch a moderate turn (24° bank is only 1.09 g). The fix is a **yaw-rate gate** (`turn_gate` 4 °/s):
past it the filter trusts the gyro alone (which integrates the true bank), and the accel gravity
vector only re-anchors roll/pitch in straight-ish flight (cancelling gyro drift there). Heading is
gyro-z only — it drifts (no magnetometer) — so nav heading degrades gracefully while roll/pitch stay
solid and the glider holds bank + pitch. Roll/pitch use an **integer-CORDIC `atan2` + `isqrt` in
`fixed.py`** (viper, ~0.17°, zero float boxed but the heading the channel format requires).

## Loiter law — confirmed on the board, with a provenance caveat

All four nominal cases land **in-zone**, both motors, both weights (miss 52–61 m, time aloft 36–72 s,
apogee-to-touchdown contained inside ~122 m of the pad) — the tuned loiter orbit + endgame spiral
works on-device, not just on the host. Provenance caveat: the host `TMS-7-loiter` set landed
**17–18 m** (near the midpoint, objective #3) on the same calm/quality-2 setup; on the board at
`inject_hz=25` the misses are **52–61 m** — still comfortably in-zone (objective #2), but the tighter
midpoint accuracy does not fully reproduce at the coarser on-board sensor rate + the endgame's
phase-luck. Board reality: **in-zone yes, near-midpoint not always** — a real datum for the endgame
tuning, exactly what board provenance is for.

## Regenerate

```bash
cd src/glider && ./deploy.sh              # deploy the firmware (incl. tasks/attitude.py + hitl drop flag)
mpremote connect /dev/ttyACM0 cp tools/hitl_run.py :
for case in "E16 e16_full 285 0" "E16 e16_half 235 0" "F15 f15_full 285 0" "F15 f15_half 235 0" \
            "E16 e16_full_drop 285 6.0" "F15 f15_full_drop 285 6.0"; do
  set -- $case
  PORT=/dev/ttyACM0 tools/hitl_collect.sh "$1" "$2" 0.05 0.0 210.0 False /tmp/hitl/att "$3" 25 0 False "$4"
done
# reports/SVGs: flight_report.py / flight_svg.py per case (pad 25.514379,-80.391795;
#   zone 25.514944,-80.392972,25.514583,-80.391111). movie: flight_video.py <out> "LABEL" <capture> ...
# direct backup-vs-truth trace: printf 'import attitude_soak\nattitude_soak.soak("F15", 6.0)\n' | mpremote run
```
