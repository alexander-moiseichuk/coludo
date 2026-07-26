# TMS-7 physics refresh — drag polar, gusts, fault injection (host)

The sim gained three things it was missing (findings §27.20–22), and this is the canonical matrix
re-flown on the result. Host virtual-flight provenance (`tools/virtual_flight.py` — the real
`guidance`/`governor`/`pid`/`mixer` over `sim_model.py`, 5 % noise, calm, seed 1), E16/F15 × full/light
glider, at the worst-case **quality 2** polar and the realistic **quality 5**.

## What changed in the model

- **Drag polar (§27.22).** Sink was a function of bank only; now it follows the AIRSPEED polar too —
  profile drag ∝ v³, induced ∝ 1/v, normalised to exactly 1.0 at the 14 m/s trim. Minimum sink sits at
  **~0.76 × trim (~10.6 m/s)**, below best-glide speed, as on any real glider. Flying well off trim now
  costs energy in *both* directions, which is what turns an airspeed-gated bank into a real trade-off
  (it unblocks the turn-radius #5.2 work). **Calm runs at trim are unchanged** — the shape was the gap,
  not the magnitude.
- **Gusts / turbulence (§27.21).** `gust` (1-σ, m/s) adds an Ornstein-Uhlenbeck disturbance to the steady
  wind, correlated over `gust_tau` — a glider integrates gusts, so the slow wander matters, not per-step
  jitter. **Default 0 (calm), so every earlier study reproduces exactly.**
- **Fault injection (§27.20).** `sim_model.Faults('gnss@30,pitot@45')` degrades a channel at a chosen
  time. Dead channels publish `None`; a faulted **pitot RAILS** instead, because a saturated
  differential-pressure cell *under-reads* — the more dangerous failure, since a low airspeed *loosens*
  the fin cap.

Knobs: `VF_GUST`, `VF_GUST_TAU`, `VF_FAULT`, `VF_GLIDER_G` (+ the existing `VF_QUALITY`, `VF_SEED`).

## Canonical matrix — touchdown miss from zone centre

| case | motor | glider | quality 2 (floor) | quality 5 (realistic) | report |
|---|---|---|---|---|---|
| `e16_full` | E16 | 270 g | 85.8 m ✗ | 88.3 m ✗ | [report](e16_full.html) |
| `e16_light` | E16 | 215 g | 38.9 m ✗ | **19.9 m ✓ in-zone** | [report](e16_light.html) |
| `f15_full` | F15 | 270 g | 109.7 m ✗ | **27.0 m ✓ in-zone** | [report](f15_full.html) |
| `f15_light` | F15 | 215 g | 114.4 m ✗ | 56.0 m ✗ | [report](f15_light.html) |

[**Top-down plan**, all four at q5](plan_canonical.svg) · [**combined video** (FHD, 310 s)](physics_refresh.mp4).

Consistent with the endgame study: **the polar dominates the landing**. Nothing reaches the zone at the
quality-2 floor; at the realistic quality 5 half the matrix lands in-zone. The lighter E16 and the full
F15 are the two that convert their glide into accuracy.

## Gust robustness (new capability)

F15, quality 5, calm baseline = 27.0 m:

| gust (1-σ) | miss | in-zone |
|---|---|---|
| 0 m/s (calm) | 27.0 m | ✓ |
| 1.5 m/s | 31.0 m | ✓ |
| 3.0 m/s | 39.0 m | ✓ |

**The converging endgame degrades gracefully** — a 3 m/s gust field costs ~12 m of accuracy and still
lands in-zone. That was the open question §27.21 raised: the `ov` result was previously proven only in
steady air, so its robustness margin was unknown. It holds.

## Fault matrix (new capability)

F15, quality 5, fault injected at t = 30 s:

| fault | miss | in-zone | reading |
|---|---|---|---|
| `pitot@30` (rails) | **27.0 m** | ✓ | **identical to baseline** — the saturation guard drops the governor back to the accel backbone at zero cost |
| `baro@30` (dead) | 100.2 m | ✗ | the endgame band is elevation-driven; losing it costs the descent plan |
| `gnss@30` (dead) | 720.2 m | ✗ | guidance falls to the blind tier — position is the one irreplaceable input |

The pitot row is the useful confirmation: the saturation fallback built for the SDP810 fusion behaves
exactly as designed under a real injected failure. The GNSS row quantifies what "GNSS is
flight-critical for accuracy" actually costs.

## On real hardware (board HITL)

The host runs above are the matrix; this is the same flight flown **on the board** — real drivers, real
servos physically moving, real INA226 on the servo rail, real MCU memory.
[**Report**](hitl_f15_full.html) · [**video** (FHD, 62 s)](hitl_f15_full.mp4).

| measured on the board (3 runs) | value | before the pitot fix |
|---|---|---|
| touchdown | **118.6–124.5 m**, spread 6 m | 85–225 m, bimodal |
| fin activity | 1869–1914 moves, **7736–8068°** (134–139 °/s) | ~1000 **or** ~5000–7000° |
| servo energy | **30.7–37.2 J** (0.53–0.64 W avg) | 3.6 J or 27.8 J |
| `fin_cap` | **5° at boost speed → 45° at trim** — the real 1/v² schedule | pinned 14 or 45 |
| airspeed estimate | ~50 m/s boost → **14.1 m/s glide** (sim trim = 14.0) | **0.00 all flight** |

### The bistability had a root cause, and it was a real flight bug

Repeated identical board flights used to land in one of two families — ~1000° of fin travel touching
down ~220 m out, or ~5000–7000° landing 85–180 m. Neither was correct, and the explanation is a hole in
the governor rather than anything about the endgame.

**The HITL sim does not simulate `airspeed`.** It is absent from `hitl.py`'s provided channels and from
`config_hitl._SIM_SENSORS`, so the **real SDP810 kept publishing its still-air ~0 m/s** into the fused
channel on every on-board flight. The pitot band was bounded only above (`pitot_max_ms`, a railed cell),
so a near-zero reading sailed through and was blended at gain 0.5. **The glider flew a simulated 14 m/s
while the governor believed 0.00 for the entire flight.**

The cap then sat at the unconfident floor (14°) until the boost transient happened to push `predict()`
past the 5 m/s confidence threshold *before* the pitot dragged it back — a **race**. Win it and the cap
jumped to the full 45° computed off that bogus zero; lose it and the flight ran on a crippled 14°. That
race was the two families.

`pitot_min_ms` (3.0 m/s ≈ 5.5 Pa) closes it — see the commit. It is not a sim-only fix: a **blocked,
iced or disconnected pitot reads exactly there in real flight**, and feeding it in opens fin authority
to maximum at high dynamic pressure, the precise failure the governor exists to prevent.

Two consequences worth carrying:

- **Servo energy is ~35 J per 56 s flight (0.6 W average), not 3.6 J.** The old figure came from
  flights whose authority was accidentally capped at 14°. This is a battery-sizing input — and it is
  the **servo rail only**: the INA226 sits on the aft power bus, which is to be rewired to service the
  main MCU as well, so the measured total becomes a steady 0.5 W or more with this servo work on top.
- **Board HITL captures recorded before this fix are not a valid basis for a control-tuning or
  power-budget claim.** Host results are unaffected — the host sim publishes realistic airspeeds, so
  the floor never bites, and the full matrix (8 cases, gusts, faults) is bit-identical before and after.

### Delta check after the §23.4 / §23.5 / stale-agl changes

Those three (warm-start airspeed seed, PID back-calculation, and a landing detector that no longer
trusts a stale AGL) were re-flown separately: **the host matrix is bit-identical** — all eight canonical
cases, the gust sweep and the fault matrix reproduce to 0.1 m. Back-calculation only engages on a
saturated fin, which these flights never reach; the AGL fix is a no-op on the host, where the sim
publishes AGL at every altitude rather than only inside a real laser's ~4 m range.

Leak on the board captures is **233–262 KB/s → OOM ~125–140 s**. It scales with control activity — every
fin move costs a servo-telemetry row — so it is a property of the flight, not only of the firmware.

This run is what validated the capture-pipeline work end to end: `flight.csv` (542 rows), the three
per-servo streams and `airspeed_sdp810.csv` all came off a real board, and the fin/authority/airspeed
panels render from them.

### Memory: where the GC-off bytes actually go

Three fixes landed, each measured on the board rather than guessed:

- **`Telemetry.push` 240 → 176 B.** The wire line is now formatted and encoded **once** (`tlm()` used to
  re-copy the whole row to prepend the session prefix), and telemetry rows carry **centi-unit integers
  instead of floats** — a float in a row is heap-boxed on MicroPython *and* then `str()`-formatted.
- **Bus read 320 → 80 B (−75 %).** `i2cbus`/`spibus` took an `asyncio.Lock` per operation; the lock alone
  cost **288 of those 320 B**. Every locked section is ONE synchronous, non-yielding `machine.I2C` call,
  and MicroPython's loop is cooperative, so the lock protected nothing — and being released *between*
  calls, it never gave multi-step atomicity either. `Bus.transaction()` is now the explicit escape hatch
  for a sequence that genuinely must not interleave.
- **A per-tick raised exception.** `flight._record()` pushed telemetry every tick; with the Recorder not
  running that RAISED and was caught every tick. `Telemetry.due()` now reports not-due when there is no
  ring to write to. In `bench_flight` this alone was **720 of the 816 B** it attributed to a control step
  — the real control path is **96 B/step (22 KB/s at 100 Hz, OOM ~1450 s)** and was never the problem.

#### How much of a HITL leak is the simulator?

The sim is float physics, and every intermediate is a boxed float — so it is the standing suspect. It is
now measured two independent ways (`make bench-hitl`, and a flown A/B where each arm changes exactly one
rate). F15, noise 0.05, calm, 285 g, ~56 s airborne:

| arm | leak | isolates |
|---|---|---|
| `inject_hz` 50 (default) | 224, 255 KB/s | baseline — two runs, the spread is real (~±16) |
| `inject_hz` 10 | 193, 194 KB/s | −40 Hz of `_publish` → **46 KB/s** (~1150 B each) |
| `inject_hz` 10 + `sim_hz` 25 | 168 KB/s | −25 Hz of `glide_step` → **26 KB/s** (~1040 B each) |

At the default rates that is publish ~58 + physics ~52 = **~110 KB/s, 46 % of the ~240 KB/s baseline,
none of which a real flight runs**. The per-call bench agrees in shape but reads ~40 % high (145 KB/s) —
a tight bench loop takes branches a real flight skips, so it is an upper bound and the flown A/B is the
number to quote.

**A real flight's leak is therefore ~130 KB/s → OOM ~250 s**, not the ~240 KB/s / ~135 s a HITL capture
shows. `board_health` carries the OOM prediction and the safe-height elimination logic as the standing
mitigation, and a dedicated soak still belongs before any powered flight — but the headline HITL figure
should not be read as a flight number.

One trap worth recording: the first `sim_hz` arm measured *nothing*, because `hitl_run` imports
`config_hitl` **from the board**. Editing the host copy without deploying changes nothing.

## Regenerate

```bash
SP=/tmp/canon; mkdir -p $SP
for q in 2 5; do for case in "E16 270 e16_full" "E16 215 e16_light" "F15 270 f15_full" "F15 215 f15_light"; do
  set -- $case
  VF_QUALITY=$q VF_GLIDER_G=$2 VF_SEED=1 python3 tools/virtual_flight.py --motor $1 --noise 0.05 \
      -o $SP/${3}_q$q.txt
done; done
PV=~/.local/share/pipx/venvs/plotly/bin/python
for n in e16_full e16_light f15_full f15_light; do
  $PV tools/flight_report.py --cdn $SP/${n}_q5.txt -o doc/sims/TMS-7-physics_refresh/$n.html; done
# gust sweep / fault matrix
VF_GUST=3.0 VF_QUALITY=5 VF_SEED=1 python3 tools/virtual_flight.py --motor F15 --noise 0.05 -o $SP/gust3.txt
VF_FAULT=gnss@30 VF_QUALITY=5 VF_SEED=1 python3 tools/virtual_flight.py --motor F15 --noise 0.05 -o $SP/f_gnss.txt

# top-down plan (all four at q5)
python3 tools/flight_svg.py $SP/{e16_full,e16_light,f15_full,f15_light}_q5.txt --overlay \
    -o doc/sims/TMS-7-physics_refresh/plan_canonical.svg --labels "e16_full,e16_light,f15_full,f15_light" \
    --title "TMS-7 physics refresh -- canonical matrix, quality 5"

# BOARD hitl -- fly it 3x, not once (see the bistability note above), then report the representative run
for n in 1 2 3; do
  ./tools/hitl_collect.sh F15 f15_full_r$n 0.05 0.0 210.0 False /tmp/hitlnew 270 0
  pkill -9 mpremote; uhubctl -l 1-3 -p 1 -a cycle; sleep 8   # the CDC wedges under rapid mpremote
done
python3 tools/flight_kpi.py /tmp/hitlnew/f15_full_r*.txt          # compare the spread before picking one
$PV tools/flight_report.py --cdn /tmp/hitlnew/f15_full_r1.txt -o doc/sims/TMS-7-physics_refresh/hitl_f15_full.html

# videos (flight_video.py <out.mp4> <LABEL> <capture> [<LABEL> <capture>] ...)
python3 tools/flight_video.py doc/sims/TMS-7-physics_refresh/hitl_f15_full.mp4 \
    "HITL F15 full (board)" /tmp/hitlnew/f15_full_r1.txt
python3 tools/flight_video.py doc/sims/TMS-7-physics_refresh/physics_refresh.mp4 \
    "E16 full q5" $SP/e16_full_q5.txt   "E16 light q5" $SP/e16_light_q5.txt \
    "F15 full q5" $SP/f15_full_q5.txt   "F15 light q5" $SP/f15_light_q5.txt
```
