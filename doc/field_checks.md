# Field checks for the Rust stack (M7b, M11, M11b, M20)

What the bench cannot prove: a real GNSS fix, the beacons' range and the batteries' time. Each check here closes an
open item in `doc/specs/avionics-port.md`. The walk test in `doc/field_test.md` is plan B's and stays as it is.

Run them on a field day before the first passive flight, on the airframe that will fly (TMS-7F, or 7C once
validated). The taster can do the GNSS and range checks too; it is USB-powered, so it cannot do the battery
time. None of the checks needs a motor.

> **Safety.** Arming moves nothing without the flight loop. But a config with `flight` enabled moves the fins in
> GLIDING. Keep the operator console open (hub port 1235) to disarm.

## Kit

- [ ] The airframe with its main board, a charged LiPo and the flight recorder box (charged, card in, switch off).
- [ ] The CC host (the panda laptop) running the hub, with the USB GPS plugged in (`--gps-device` finds it by
      default). The hub console's `gps <board>` compares the board's fix against the host's.
- [ ] A phone with a Wi-Fi analyzer app (it shows SSID, channel and signal in dBm), and a tape or a range finder
      for distances.
- [ ] Open sky for the GNSS: no building or tree line within ~20 m of the pad.

## Before leaving (bench)

- [ ] Firmware on the board: `firmware/flash.sh` over USB, or `tools/ota.py --console` (both boards on one build).
- [ ] A clean card: `tools/board_clean.py <board>` (it keeps the running session, `sessions.csv` and
      `entry.gcl`). The same for the box: `tools/recorder_console.py ls`, `rm` the old sessions.
- [ ] No entry file: `entry show` answers empty (while one is present, `verify` and `arm` refuse).
- [ ] The INA226 entry gives `battery_mah` (the pack's capacity), so the beacon page shows the hours left.
- [ ] `verify` passes on the hub. `ready` may still list the field items (no zone yet, and so on).

## 1. GNSS: first fix and the reference (M11)

The NEO-6M's module needs open sky. With the small passive patch the 10-03 fix never came; an active antenna is
fitted since (`doc/benches/gnss-20261004`).

- [ ] Power on in SETTING. Note the time of power-on.
- [ ] On the hub console: `gps <board>` every ~30 s until the board shows a fix. Note the time to first fix
      (TTFF) and the satellites used.
- [ ] Once both fixes hold, note their distance apart (`gps <board>` shows both). Expected: within ~5-10 m.
- [ ] `inspect gnss`: `lines` rising, no `inv format` events in `gnss_events.csv` (a refused configuration
      raises one per family).
- [ ] Record the antenna's sky: `gnss_sky.csv` (C/N0 of the four best satellites, every ~10 s). Expected above
      35 dB-Hz with the active antenna.

**Pass:** a fix within 2 minutes of a cold start, and within ~10 m of the host's.

## 2. gnss_calib: the pad drift (M11)

The drift is the GNSS speed a still board reads on the pad. The flight subtracts it from the wind.

- [ ] Board still on the pad with a fix, in SETTING. `inspect gnss_calib` every ~30 s for 2 minutes: `samples`
      rises about one a second; `drift_e` and `drift_n` stay small (a still board: below ~0.3 m/s).
- [ ] `arm`, and check that `samples` restarts from 0. Then `inspect gnss_calib` again after 10 s: it averages
      from the arm on.
- [ ] `disarm`.

**Pass:** samples accumulate, the restart at arm happens, and the drift stays below ~0.3 m/s.

## 3. The launch-point freeze (M11)

- [ ] Do not use `assist` before this check. `health`: `launchpad_set` false, and `launchpad` follows the live fix.
- [ ] `arm`. `health`: `launchpad_set` true, and `launchpad` equals the fix at the arm.
- [ ] Carry the airframe 20 m away. `launchpad` does not move; `position` does.
- [ ] `disarm`. Bring it back.

**Pass:** the point pins at the arm and stays pinned.

## 4. field: CC-less site pick and auto-arm (M11)

Only when the airframe will fly CC-less. It needs a config with `field` enabled: `site_select` true,
`auto_arm` true, `auto_arm_dwell_s` 60, `still_g` 0.3, and at least one site in the launch config.

- [ ] Reboot into the field config. With a fix, the log names the site it picked (or the fallback zone, when no
      site is in range: `cc-less-fallback` in `degraded`).
- [ ] Leave the airframe still on the pad. After the dwell it arms itself: the log line, then `health.armed` true.
- [ ] `disarm`. Move the airframe during the dwell: it restarts, and no arm comes until a whole still dwell.
- [ ] Negative: power it with a gate failing (for example a sensor unplugged). The auto-arm logs each problem and
      waits another dwell. It never arms past `arm`'s passive gates.
- [ ] Put the normal config back.

**Pass:** the site is picked, the arm follows one still dwell, a move restarts it, and a failing gate is never
armed past.

## 5. The beacons' range (M11b, M20)

There are two beacons on two radios: `<BOARD>_glider` on channel 3 (the main board, from DONE on) and
`<BOARD>_beacon` on channel 6 (the flight recorder box, from the rod trigger on). The box is on air 60 s, then
silent 60 s.

- [ ] Box switch on, connected to the main board. In SETTING, `stage done` on the console: the main board beacons
      within ~2 s, and the box once it reads the stage past SETTING (within its 5 s report).
- [ ] At the pad, a phone joins each SSID. The page opens by itself (a captive portal) and shows the fix with a
      working map link, every elevation source and the battery.
- [ ] Walk away in a straight line. At 25, 50, 100, 150 and 200 m, note each SSID's signal (dBm) and whether the
      page still loads. Wait up to a minute at each point for the box's on-minute.
- [ ] Repeat with the airframe lying in grass. Then hang it about 2 m up in a bush or a tree, as after a landing.

| distance | glider dBm | glider page | box dBm | box page | placement |
|---|---|---|---|---|---|
| 25 m | | | | | open |
| 50 m | | | | | open |
| 100 m | | | | | open |
| 150 m | | | | | open |
| 200 m | | | | | open |
| ... | | | | | grass / tree |

**Pass:** both SSIDs visible to at least 100 m in the open, and the page loads at the distance a search starts
from (~50 m).

## 6. The battery time (M11b, M20)

Each battery runs from full charge until its beacon disappears. The two runs can go at the same time.

- [ ] **The main board at DONE, on its LiPo** (never USB: above 4.35 V the page reads USB and shows no hours).
      With the pack fully charged, `stage done`. Note the time. The beacon page shows the voltage, the current
      and the hours left.
- [ ] **The box on its 120 mAh LiPo:** fully charged and beaconing (it keeps beaconing after the main board is
      off: it replays its last report at boot).
- [ ] Log both SSIDs every minute from the laptop, until both are gone:
      `while sleep 60; do date +%T; nmcli -f SSID,SIGNAL dev wifi list --rescan yes | grep -E '_glider|_beacon'; done | tee beacons.log`
- [ ] Afterwards, pull the main board's session (`tools/board_pull.sh <board>`). `power_ina226.csv` at 1/s is the
      discharge curve.

**Expected:** the main board ~1.5-2 h on 800 mAh. The box ~2.1-2.4 h on 120 mAh with the 60 s on/off cycle, against
~1.2 h always on (estimates from the ESP32-C6 datasheet: `doc/flight_recorder.md`).

## 7. The camera's pad run (M7b)

The segments were proven on the bench and in HITL at 20 s. The 10-minute run at the default 300 s is left to the
pad.

- [ ] In SETTING on the pad: `video start`. After 11 minutes, `video stop`.
- [ ] Pull the session's video (`tools/board_pull.sh <board> <session>`): two whole 300 s segments and a partial
      third, each playable (`ffprobe` shows the frame count at ≥ 20 fps).

## After the day

- [ ] Pull each session from the main board (`tools/board_pull.sh`) and from the box
      (`tools/recorder_console.py pull <session>`). Every file pulled is checked against the card's size and CRC-32.
- [ ] Note the results in `doc/specs/avionics-port.md` (M11's "Open, outdoors", M11b's battery time, M20's range
      and battery time, M7b's pad run) and file the numbers in the session's notes.
