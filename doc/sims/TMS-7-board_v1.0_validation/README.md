# TMS-7 — board v1.0 validation: does the rewire change anything?

**90 board flights** (3 combos × 10 scenarios × 3 runs) plus a **70 000-frame bus sweep**, flown on the
breadboard after it was rewired from the v0.1 bus layout to
[v1.0](../../hardware.md#main-board-v10--the-target-layout): four devices moved bus, `i2c:1` dropped to
100 kHz, the ADXL375 came off.

Two questions, and they want opposite answers. **The buses should get better** — that is what the rewire
was for. **The aircraft should not change at all** — the board is meant to fly the same while weighing
less, and a layout change that moved control behaviour would be cause to investigate rather than
celebrate.

Both came out that way.

## 1. The buses: 0.50 % → 0.000 %

`test/diag_bus_stability.py`, 10 000 reads per device across every bus:

| device | bus | good | corrupt | bus err |
|---|---|---|---|---|
| `baro_icp10111` | i2c:1 | **10000** | **0** | 0 |
| `airspeed_sdp810` | i2c:1 | 10000 | 0 | 0 |
| `laser_agl` | i2c:1 | 10000 | 0 | 0 |
| `imu_bno055` | i2c:0 | 10000 | 0 | 0 |
| `baro_bmp280` | i2c:0 | 10000 | 0 | 0 |
| `power_ina226` | i2c:0 | 10000 | 0 | 0 |
| `imu_lsm6dso32` | spi:1 | 10000 | 0 | 0 |

The headline is the ICP-10111. On v0.1 it corrupted **2 frames in 400 (0.50 %)** — single-bit flips in
its own CRC byte with plausible data either side, measured on the bench with the board idle. On v1.0 it
is **0 in 10 000**, where the old rate would have produced about **50 events**. A 400-frame sample could
not have said this: zero happens ~13 % of the time even at 0.50 %, which is why the frame count went up
by 25×.

**The mechanism is the clock, not isolation.** The ICP still shares a bus with the pitot and the laser —
what changed is that the bus runs at **100 kHz instead of 400 kHz**. Slower edges on a long unterminated
harness is the textbook fix for single-bit corruption in transit, and it costs nothing here because
nothing on that bus exceeds 50 Hz.

Two detectors were needed, because **five of the seven devices have no checksum**: the CRC parts are
validated by their own drivers, and everything else is read at an *identity register* whose value cannot
legitimately change, so any deviation is a bit that did not survive the wire. NAK and timeout are counted
separately from corruption throughout — a device that stops answering is a connector, one that answers
wrongly is signal integrity, and conflating them would let a loose plug read as a bus needing a slower
clock.

## 2. Detection: the board recognises itself, both ways

`layout.detect()` scans both buses before any driver binds and votes on which revision it is looking at.
Same firmware, same config, no edit:

| | i2c:0 | i2c:1 | verdict |
|---|---|---|---|
| before rewire | `0x18 0x25 0x28 0x29 0x63 0x76` | `0x40` | **v0.1, 4–0** |
| after rewire | `0x18 0x28 0x40 0x76` | `0x25 0x29 0x63` | **v1.0, 4–0** |

and then applied itself: `power_ina226 i2c:1→0 · airspeed_sdp810 i2c:0→1 · baro_icp10111 i2c:0→1 ·
laser_agl i2c:0→1 · i2c:1 400000→100000 Hz · accel_adxl375 not fitted → disabled · laser int_pin and
xshut_pin dropped`.

`diag_devices` then reported **21 up, 0 down** with every probe passing, and the board suite **58/58**.

## 3. The aircraft: unchanged, which is the point

Median miss per run, against the three-run [preflight](../TMS-7-preflight/) baseline:

| combo | preflight | v1.0 | bands |
|---|---|---|---|
| `e16_full` | 71.0 / 74.0 / 76.6 | 73.7 / 75.7 / 80.3 | overlap |
| `f15_full` | 88.0 / 90.8 / 90.9 | 70.9 / 77.8 / 83.3 | **disjoint** |
| `f15_half` | 37.1 / 39.3 / 46.3 | 42.8 / 45.1 / 49.0 | overlap |

`f15_full`'s band is disjoint and lower, and **it would be wrong to call that an improvement.** The
median moved because *variance exploded* on two scenarios, not because flights got consistently closer:

| scenario | preflight | spread | v1.0 | spread |
|---|---|---|---|---|
| `f15_full` `wind00` | 92.5 / 89.5 / 90.6 | **3.0** | 49.6 / 88.9 / 100.0 | **50.4** |
| `f15_full` `noise25` | 95.5 / 92.2 / 86.8 | 8.7 | 80.0 / 77.8 / 24.6 | **55.4** |
| `f15_half` `noise05` | 27.3 / 23.3 / 30.3 | 7.0 | 30.3 / 35.3 / 118.4 | **88.1** |

A median that falls because its inputs became less predictable is not a better aircraft. Caveat on the
claim in the other direction too: **comparing variances from three samples each is statistically weak**,
so the contrast is stark but not established.

**In-zone counts, which are the decision-relevant number:**

| combo | preflight | v1.0 |
|---|---|---|
| `e16_full` | 1/10, 1/10, 1/10 | **0/10, 0/10, 0/10** |
| `f15_full` | 0/10, 0/10, 0/10 | 0/10, 0/10, 2/10 |
| `f15_half` | **7/10, 7/10, 7/10** | 7/10, 7/10, 6/10 |

`f15_half` — the combo that matters, and the one the airframe is heading toward on mass — reproduces:
same band, essentially the same seven scenarios in zone.

**One open item, recorded rather than dismissed.** `e16_full` lost its single in-zone landing in **all
three runs**. It was `wind03`, previously the most reproducible scenario in the whole study at a **1.9 m**
spread (54.1 / 56.0 / 55.6), now 73.4 / 56.1 / 64.9. A consistent 3/3 change on the tightest scenario
deserves an explanation before the first *guided* flight. It blocks nothing today — 7C and 7D fly
fins-fixed — and one candidate mechanism is that removing the ADXL375 took `accel`'s priority-1 provider
away, so a stale LSM6DSO32 now falls to the BNO055 at a **40 ms** window instead of 20 ms, which feeds
the governor's airspeed and hence endgame timing.

## 4. Panel comparison

`tools/hitl_compare.py` across the three runs (`comparison.json`):

* **Channel dropouts: 0.** No sensor went missing in 90 flights.
* **Physically impossible values: 3**, all in rows of correct width — two gyro ranges at ~102 000 against
  a [0, 40 000] limit and one airspeed at 282 322 cm/s. Bit flips inside a field, invisible to every
  structural guard; only the physical-bounds check catches them. Same class and rate as the preflight
  study's residual.
* **Wide but expected:** `health load max` at 74–78 % and `health mem_free min` vary genuinely run to
  run, exactly as the preflight study found. Memory is not reproducible at this granularity.

## 5. Verdict

**The rewire is validated.** Bus integrity improved measurably and by a mechanism that makes sense;
detection works in both directions with no operator action; every device is up; the suite is green; and
the aircraft flies the same to within its own endgame phase variance.

**And flight-neutral is the correct result**, because the board's payoff is not accuracy but **mass**:
the deleted power and GNSS harnesses, their connectors and a shorter nose are expected at **30–50 g**,
taking the glider from 287.5 g toward 237–257 g. On this airframe
[50 g is worth 45 m of median miss and seven in-zone landings](../TMS-7-preflight/) — far more than any
control change available. The accuracy comes from the scale.

## 6. r4 — the code-polish re-fly (2026-09-16)

A 22-finding code review touched the control station, the new nose logger and **one glider file**:
`config.py`, where `config_id` stopped hashing `board.firmware_version` (load() restamps it from the
running firmware, so a byte-identical config was reporting a different id after every deploy). Nothing
in the flight path changed. r4 asks whether the aircraft noticed anyway.

**It did not**, which is the answer that was wanted:

| combo | r1–r3 medians | **r4** | r1–r3 in-zone | **r4 in-zone** |
|---|---|---|---|---|
| `e16_full` | 73.7 / 75.7 / 80.3 | **80.6** | 0/10 ×3 | **1/10** |
| `f15_full` | 70.9 / 77.8 / 83.3 | **83.2** | 0, 0, 2 /10 | **0/10** |
| `f15_half` | 42.8 / 45.1 / 49.0 | **39.6** | 7, 7, 6 /10 | **7/10** |

Every median lands inside or at the edge of its band, and `f15_half` — the combo that matters, and the
one the airframe is heading toward on mass — reproduces **7/10** exactly.

**The open item from §3 did not reproduce.** `e16_full` had lost its only in-zone landing on `wind03` in
all three previous runs; in r4 it is back (57 m, in zone). One observation against three does not settle
it, but it argues the cause is VARIABLE rather than a fixed consequence of the rewire — the more
reassuring of the two explanations, and the one that keeps this off the critical path for a fins-fixed
flight.

**Do not read `f15_half`'s 39.6 as an improvement.** §3 makes the argument against exactly that move: a
median can fall because variance grew. With one run there is no across-run spread to check, so the only
honest statement is that it is not WORSE.

**Two provenance limits, recorded rather than smoothed over:**

* One capture (`e16_full/wind06`) is stamped `build UNKNOWN` — the board did not answer the post-flight
  query. Its data is complete; only its stamp is missing.
* **The build stamp cannot see uncommitted work.** It reads the last commit, and r4 flew that commit
  plus an uncommitted working tree. The stamp exists to answer "did these runs fly the same build", and
  a dirty tree is precisely the case it cannot distinguish. Worth fixing before it misleads someone.

`config_id` also moved once, by design: taster reports `a2bd31537ad3` where it reported `e5fa26f98e9c`.
It is stable across deploys from here.

### What r4 cost, and the guards that came out of it

The first attempt produced **30 captures and zero data**, and reported `OK ... DONE` for every flight.
The recorder's filesystem was **100 % full** — 53.5 GB of camera `.mjpeg` had filled `/userdata` — so
each session got its files created and no rows written. Two combos were then lost to a CDC wedge, which
`hitl_matrix.sh` reported and gave up on.

Both failure modes now fail loudly instead:

* `hitl_collect.sh` refuses to start below 50 MB free on the recorder, and **fails when a pulled stream
  is 0 bytes** — `ls | wc -l` counts an empty file exactly like a full one, which is how thirty flights
  looked successful.
* `hitl_matrix.sh` runs `board_unwedge.py` and retries once rather than losing a ten-flight combo.
* `tools/preflight.py` gates on **1 GB** of recorder free space.

The same full-disk signature — filenames merged into data rows, stats lines cut mid-token — is what the
[postaudit regression](../TMS-7-postaudit_regression/) recorded as UART corruption "believed to be on the
receiving side and unresolved". That does not prove the historical case had this cause, but partial
writes against a full disk explain the signature better than line noise on a soldered wire, and the note
is worth revisiting rather than left standing.

## Layout

```
r1/ r2/ r3/        per-run plotly HTML + SVG, prefixed by combo
r4/                the code-polish re-fly (§5), same shape
comparison.json    the full panel comparison across the first three runs
```

## Reproducing

```bash
tools/deploy.sh
mpremote connect $PORT run src/glider/test/diag_bus_stability.py     # 10 000 frames per device
export SCENARIOS='noise05 noise10 noise25 wind00 wind03 wind06 wind09 wind12 corner_spike corner_stress'
for r in r1 r2 r3 r4; do    # r4 is the code-polish re-fly; see section 5
  GLIDER_G=285 bash tools/hitl_matrix.sh E16 /tmp/v10/$r/e16_full
  GLIDER_G=285 bash tools/hitl_matrix.sh F15 /tmp/v10/$r/f15_full
  GLIDER_G=235 bash tools/hitl_matrix.sh F15 /tmp/v10/$r/f15_half
done
```

Count the artefacts. A short run prints "matrix done" like any other.
