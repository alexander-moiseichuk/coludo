# TMS-7E — the first v1.0-board airframe

Same airframe idea as [TMS-7D](../TMS-7D/), rebuilt on **main board v1.0** instead of v0.1. This is the
first airframe to fly the consolidated board, and the reason that board exists: the payoff is **mass**,
not accuracy — 30–50 g of deleted harness and a shorter nose, worth roughly 45 m of median miss at the
measured 0.9 m/gram.

| | |
|---|---|
| board | **main board v1.0** — detected at boot, no config edit |
| config | [`tms7e.config`](tms7e.config) — passive (`flight` off), `fins.concurrency` **3** |
| status | **built, bench bring-up done** — 21 devices up, 0 down; engines not yet fitted |

## What the v1.0 board changes against 7D's v0.1

`layout.py` recognises the board at boot and applies all of this itself — the config carries the
resulting map, and nothing is hand-edited per board:

* **`power_ina226` moves to `i2c:0`**, and the **ICP-10111, SDP810 and laser move to `i2c:1`**, which
  drops to **100 kHz**. That bus split is what took ICP-10111 frame corruption from 0.50 % to 0.000 %.
* **No ADXL375.** The LSM6DSO32's ±32 g already covers the 8–12 g boost and the measured HITL peak is
  4.3 g; the backstop never recorded anything the primary could not. `layout` disables it.
* The laser's `int_pin` and `xshut_pin` are freed — the driver treats both as optional.

The LSM6DSO32 **stays** on `spi:1`: it is the primary `accel` and the only gyro `rate`, so it is
flight-critical here exactly as on 7D.

## Bench bring-up (2026-09-20)

Deployed, `layout` detected **v1.0 on votes 6-4-2**, and 21 devices came up with none down: BNO055,
ICP-10111, BMP280, SDP810, laser, INA226, GNSS, separation, recorder, three servos and the radios.
Board suite **62/62** (after the fix below). Recorder reachable over adb with 54 GB free. The board
answers CC as `TMS-7E` on the panda network, reporting `layout: v1.0` itself.

The BNO055 is already calibrated (`[0, 3, 3, 3]` — the leading `sys` lags and means nothing). One
calibration is still outstanding: the **pitot tare** (`calibrate airspeed_sdp810`), which needs still
air.

**Servos: two of three verified, `servo_yaw` OUTSTANDING.** With the rail unpowered (191 mV) all three
read "no current draw", which proves nothing. Powered through the battery simulator at **4.076 V** the
elerons both pass their probe; `servo_yaw` still shows no rise, while idle draw of 22 mA says all three
parts *are* powered (~7 mA standby each). So yaw has supply but does not respond — its signal path or
its own driver. The test that separates them is to move its lead onto a proven channel.

**`fins.concurrency` is 3**, matching 7D, set on the battery simulator after the bench bring-up. It
started at 1: a concurrency-3 profile on a current-limited supply is the most likely cause of the servo
failure on 2026-08-29 (measured MG90S peak 0.79 A each, so three at once is ~2.4 A). But the value
does not protect a weak supply: it gates `servo.move()` only, and boot centring moves all three fins
together at every boot regardless. On an unknown or current-limited supply, disable the servos. 7C
flies at 1, with its servos disabled.

**RESOLVED: the `test_spibus` failure was software, and bench-only.** It failed in three consecutive
full suite runs (*LSM6DSO32 WHO_AM_I never read 0x6C*) while passing standalone, which looked like
marginal SPI wiring. Bisected to **`test_pins.py`**: it creates and DEINITIALISES `SPI(1)` under the
LSM6DSO32, which leaves the part out of step — and an MCU reset never clears it, because the reset does
not reach the chip.

The mechanism is the opposite of the obvious one: **reading never recovers the part**, however long you
read. Writing `CTRL3_C` pins the interface and fixes it at once, and a write issued before the part is
listening is lost. The old `setup()` wrote once and then read five times, so it spent its whole budget
reading a chip it had already failed to pin. Driver and test now pin-then-verify in a loop.

**It is not a flight risk**, measured rather than assumed: interrupting the driver mid-traffic and
rebooting recovered on the first read **12/12**, and a bus retune at 1/8/5 MHz **4/4**. Only creating
and destroying the peripheral reproduces it, which only this test does. Suite now 62/62, three runs.
