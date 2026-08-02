# TMS-7 noise tolerance — which sensor is it actually sensitive to?

Every main sensor driven to **100 % noise individually**, F15 full (270 g), quality-2 polar, calm,
seed 1, host sim. A global noise sweep answers "how much can it take"; only a per-channel sweep
answers "which sensor does it depend on", and those turn out to be very different questions here.

Per-channel noise is a new knob: `VF_NOISE_<CHANNEL>` overrides `--noise` for one channel
(`ACCEL`, `HEADING`, `ROLL`, `PITCH`, `ALTITUDE`, `RATE`).

[baseline](baseline.html) · [roll_100](roll_100.html) · [altitude_100](altitude_100.html) ·
[accel_100](accel_100.html) · [**overlay plan**](plan_noise.svg)

## Results

| channel @100 % | miss | in-zone | apogee | duration | verdict |
|---|---|---|---|---|---|
| *baseline (5 %)* | 119 m | no | 268 m | 48.8 s | — |
| `RATE` (gyro) | 119 m | no | 268 m | 48.8 s | **no effect** |
| `HEADING` | 118 m | no | 268 m | 48.8 s | **no effect** |
| `ROLL` | 120 m | no | 268 m | 48.7 s | **no effect** |
| `PITCH` | 114 m | no | 268 m | 46.7 s | negligible |
| `ALTITUDE` | 43 m | *yes* | **168 m** | **32.8 s** | ⚠ **stage machine — early apogee** |
| `ACCEL` | — | — | — | — | ⚠ **stage machine — NEVER LAUNCHED** |

Both stage-machine failures have since been **fixed** — see "The fixes" below. `ACCEL` at 100 % now
flies identically to baseline; `ALTITUDE` is improved at every level but bounded, for a reason worth
reading.

## The headline contradicts the hypothesis on record

`plan.md` predicted: *"the bank loop reads attitude, so heavy noise degrades the orbit — a filter /
rate-limit on the steering input"*. **That is not what happens.** Attitude, gyro rate and heading can
be driven to 100 % noise — twenty times the nominal level — and the landing moves by **at most 5 m**.
The control law is already robust to exactly the inputs it was thought to be fragile to, and a filter
on the steering input would be solving a problem that does not exist.

The fragility is in the **SEQUENCER**, not `guidance`.

**`ACCEL` at 100 % — the glider never leaves the pad.** Its stage timeline is `0.0s → SETTING` and
nothing more, against the baseline's `SETTING → BOOSTING(0.1s) → GLIDING(7.1s) → LANDING(48.5s)`.
Launch detection is a `launch_g` threshold on accel magnitude, so noise on that channel prevents the
2.5 g trigger from being recognised and the flight never starts. The "49 m miss" `flight_metrics`
reports is the distance from the pad, not a landing — **the run is not a flight and must not be read
as one**.

**`ALTITUDE` at 100 % — apogee fires 2.8 s early.** GLIDING is entered at 4.3 s instead of 7.1 s, so
the glider separates at **168 m instead of 268 m** and flies 32.8 s instead of 48.8 s. It happens to
land 43 m out and in-zone, but that is a *shorter, lower arc reaching a nearer part of the field* —
not better control. Reading it as an improvement would be exactly backwards.

## What this means for the work queued against it

The queued task was "high-noise robustness ≥50 %: a filter / rate-limit on the steering input".
**That task should be re-scoped.** The steering input does not need protecting. What needs protecting
is stage detection:

1. **Launch detect against accel noise.** Today a noisy accel channel can hide the launch entirely.
   The baro backup (`launch_alt_m` 10 m) exists but did not save this run — worth checking why, since
   it is precisely the redundancy for this case.
2. **Apogee detect against baro noise.** `apogee_drop_m` 5.0 m with a noisy baro fires early. The
   detector already has a sustain window (`launch_ms`); the drop threshold may simply need to scale
   with observed noise, or the peak needs filtering before the comparison.

Both are `sequencer` changes, both are cheap, and both matter more than anything in the control law —
because a missed launch or an early apogee costs the whole flight, while 100 % attitude noise costs
5 m.

## The fixes

Both detectors now live in `commons` and are called by the board AND the host sim, so this study grades
the flight code rather than a copy of it. That sharing was not incidental — it is what made the fix
measurable at all (see the first caveat below, which used to describe a live problem).

**Launch — a LEAKY dwell (`commons.dwell_step`). Fully fixed.**

| `ACCEL` @100 % | before | after | clean baseline |
|---|---|---|---|
| apogee | *never launched* | **268 m** | 268 m |
| duration | — | **48.9 s** | 48.8 s |
| miss | — | **121 m** | 119 m |

The old dwell reset its start on any single sample below `launch_g`, so an unbroken `launch_ms` run
never happened on a noisy channel. Now a dip *drains* the accumulated credit instead of wiping it: the
condition must hold `launch_ms` of NET time. At 100 % accel noise the flight is now indistinguishable
from the clean one.

The unit matters, and getting it wrong cost a revert. A leaky count of *samples* is equally
noise-tolerant and was tried first, but it silently redefines `launch_ms` into a sample count — the
trigger's timing then drifts with tick rate and a documented config knob quietly stops doing anything.
`test_sequencer` caught it on the second tick. Time in, time out.

**Apogee — smooth the PEAK, not just the descent. Improved, and bounded.**

The peak is a running *maximum*, and a maximum has no noise immunity: one high sample is latched
forever and every later reading is judged against that spike. The drop band only ever protected the
descending side. A first-order IIR (`commons.APOGEE_SMOOTH`, weight 1/4, one scalar of state so it
allocates nothing with GC off) now smooths elevation before the comparison.

| altitude noise | apogee before | apogee after | true apogee |
|---|---|---|---|
| 15 % | 180 m | **215 m** | 268 m |
| 25 % | 177 m | **180 m** | 268 m |
| 50 % | 168 m | **178 m** | 268 m |
| 100 % | 168 m | **177 m** | 268 m |

Better at every level, never worse. **Read the apogee column, not `miss`** — a truer apogee means a
longer flight with more energy to dissipate, and since the clean baseline itself misses by 121 m, the
short early-apogee arcs land *nearer* by accident. That is the same trap this study already warned
about; it applies to its own fix.

Heavier smoothing was measured and rejected: at 8/16/32 the noisy case does not improve at all (171 m,
flat) while the clean case picks up ~20 m of detection lag. 1/4 is the knee.

**Why 100 % altitude stays at 177 m, and why that is correct.** `sim_model.noisy` scales the
perturbation by `abs(value) + 1`, so 100 % on a proportional channel makes the baro uniform-random over
`[0, 2×altitude]` — not a noisy sensor but one carrying **zero information**. No filter recovers a
signal that is not there. What actually flies the vehicle in that case is the burnout timeout, which is
exactly the fallback it was built to be, and 177 m is that fallback working. The honest reading is that
the 100 % altitude case tests *redundancy*, not *filtering* — and it passes.

Note also that even 15 % baro noise still costs 53 m of apogee. Real hardware is nowhere near that (an
icp10111 resolves ~±0.1 m), but it does say the detector is more noise-sensitive than the drop band
alone suggests.

## Caveats

- **The "before" numbers graded a COPY of the board's launch detector.** `virtual_flight.py` had its
  own `if accel_m > launch_g`, so the original sweep measured the host's detector and reported it as
  the board's — the result happened to be right, but the study could not have known. Both detectors are
  now shared through `commons`, which is why the "after" numbers mean what they say. The lesson
  generalises: **a sim that reimplements the flight code grades itself.**
- **GNSS position is never noised** in either sim (board parity — `tasks/hitl._publish` does the same),
  so this sweep says nothing about position-error tolerance. A *noisy* GNSS remains untested. A *dead*
  one was priced at 720 m by the earlier fault matrix and has since been addressed separately — the
  no-fix path now dead-reckons (`guidance._reckon`) instead of freezing at the launch point, since the
  receiver is expected to lose lock through the boost. That number wants re-measuring. Building it
  turned up a trap worth knowing about generally: **board floats are single-precision**, so advancing a
  latitude near 48 deg by a 0.2 m step changes it by exactly nothing (~0.4 m ULP). Dead reckoning
  accumulates metres and converts once; anything that integrates small increments into a large absolute
  coordinate on this hardware is silently wrong, and worst precisely when the increments are small.
- One seed per case. The attitude channels are so insensitive that seed choice is unlikely to matter
  there, but the two stage-machine failures should be repeated across seeds before being quantified.

## Reproduce

```bash
for ch in ACCEL HEADING ROLL PITCH ALTITUDE RATE; do
    lc=$(echo $ch | tr A-Z a-z)
    env VF_QUALITY=2 VF_GLIDER_G=270 VF_SEED=1 VF_NOISE_$ch=1.0 \
        python3 tools/virtual_flight.py --motor F15 --noise 0.05 -o captures/noise/${lc}_100.txt
done
python3 tools/flight_metrics.py captures/noise --sort name
```
