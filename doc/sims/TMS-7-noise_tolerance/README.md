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

## Caveats

- **Host sim, quality-2 polar.** The board's stage machine is the same code (`tasks/sequencer.py`),
  but the host reimplements the *driving* of it, so confirm on-board before acting.
- **GNSS position is never noised** in either sim (board parity — `tasks/hitl._publish` does the same),
  so this sweep says nothing about position-error tolerance. The earlier fault matrix priced a *dead*
  GNSS at 720 m; a *noisy* one is untested.
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
