# TMS-7F — the first v1.1 airframe

Main board **v1.1**: the SEN0697 replaces the SEN0253, so this is the first airframe where the board
fuses its own attitude and carries a magnetometer. Built 2026-09-20 with the last LSM6DSO32 in stock
and the VL53L1X laser moved across from the taster.

| | |
|---|---|
| board | **main board v1.1** — detected at boot, no config edit |
| attitude | **SEN0697** = BMI323 `0x69` + BMP581 `0x47` + **BMM350 `0x15`** |
| 6-DoF | LSM6DSO32 on `spi:1` — still the primary `accel`; the BMI323 is the SECOND gyro source |
| laser | **VL53L1X** (`0x29`) — the L4CX was out of stock; Adafruit parts due in 1–2 weeks |
| config | [`tms7f.config`](tms7f.config) — passive (`flight` off), `fins.concurrency` **1** |
| status | **soldered, not yet brought up** |

## What v1.1 changes

`layout` recognises the board at boot and applies all of it — nothing is hand-edited per board:

* `imu_bmi323`, `mag_bmm350`, `baro_bmp581` **enabled**; `imu_bno055`, `baro_bmp280`, `accel_adxl375`
  **disabled**. One firmware, three revisions.
* Bus map as v1.0: ICP-10111, pitot and both laser entries on `i2c:1` at 100 kHz, INA226 on `i2c:0`.

Redundancy moves in both directions, and it is the real difference between this airframe and 7E:

| channel | v1.0 | **v1.1** |
|---|---|---|
| `attitude` | BNO055 p0 + the board's filter p1 | **the filter ALONE — no p0 provider** |
| `rate` | 1 source | **2** (LSM6DSO32 + BMI323) |
| `mag` | none | **BMM350** |

**`attitude` is single-source here.** If the complementary filter misbehaves there is nothing behind
it, where a v1.0 board still had the BNO055. No round has yet flown the filter as the sole attitude
source with a healthy GNSS — see the open item in
[TMS-7-board_v1.1_sen0697](../../../doc/sims/TMS-7-board_v1.1_sen0697/).

## The two AGL thresholds are deliberately NOT what the defaults say

Both are set from what the fitted laser actually does, not from its spec sheet.

**`land_agl_m` stays at 5.0** — the default, kept on purpose. The measured VL53L1X ceiling is **2.07 m
indoors** (ambient 26, pale targets, no sun), which is the lucky case rather than the safe one. This
threshold gates BOTH sources — laser first, barometric elevation as fallback — so lowering it to a
laser-reachable 2 m would also drop the BARO trigger to 2 m, and baro is what will actually fire. That
trades a reliable path for a lucky one: same precision, ~1 s less of LANDING stage at a 2–3 m/s sink.

**`final_approach_agl` is 0 — OFF.** That gate has NO fallback (`agl_source is not None`), so the
laser's acquisition range is the de facto threshold: with a part that acquires below ~2 m, configuring
4 m or 8 m gives identical behaviour, namely engaging ~0.5–0.7 s before touchdown. What it does on
engaging is track the strip centreline with `land_bank_gain` 3.0 up to `land_bank_limit` 45°. A
guidance mode that switches on that late and commands bank is worse than one that never switches.

So **the laser contributes to no control decision on this airframe.** It still records AGL telemetry
for the last metre or two, which is worth having for post-flight analysis of the flare.

Revisit both when the VL53L4CX arrives AND has been measured against open ground in sunlight — not
before. If the laser is to matter, the change that buys it is giving the final-approach gate the same
elevation fallback the landing trigger has; that alters flight behaviour and wants a HITL round.

## Bring-up sequence (the one that worked for 7E)

```
tools/deploy.sh
mpremote connect $PORT run src/glider/test/diag_devices.py     # 21+ devices, fin probe moves fins
cd src/glider/test && make test                                # expect 63/63
mpremote connect $PORT cp launches/20261003/TMS-7F/tms7f.config :board.config && mpremote reset
```

Then, unique to this airframe: **calibrate the BMM350** over CC (`calibrate mag_bmm350`) — a LEVEL full
circle, twice round. The code is written and tested but has never met a real magnetic field; it refuses
a partial turn on purpose. `fins.concurrency` starts at 1 until this board's servos have drawn current.
