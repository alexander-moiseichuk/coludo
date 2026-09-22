# TMS-7 nose logger — ESP32-C6 + SEN0697

A standalone flight recorder for **TMS-7**, the booster-only vertical test (119.5 g airframe +
98.8 g F15 = **218.3 g liftoff**). TMS-7 carries no electronics and returns no data; this payload
makes the flight yield **peak boost acceleration** and **apogee altitude** for the cost of a few grams
in the nose.

It talks to nothing. No radio in flight, no link to the main board — a sandwich of
**ESP32-C6 SuperMini + LiPo + SEN0697**, recording from power-up to power-off and saving to flash as it
goes. USB is the development and read-out path; the status LED is the liveness check.

## The part

**SEN0697** (DFRobot Fermion, *Gravity 3126*) carries three Bosch dice on one board:

| chip | channels | range | max ODR |
|---|---|---|---|
| **BMI323** | accel x/y/z, gyro x/y/z, temperature | **±16 g**, ±2000 °/s | 6.4 kHz |
| **BMM350** | mag x/y/z | ~±2000 µT | 400 Hz |
| **BMP581** | pressure, temperature | 30–125 kPa | 240 Hz |

**The BMM350 is not wired into this mission.** A magnetometer on a vertical missile reports roll
orientation at best, and adopting it means owning hard- and soft-iron calibration next to a booster —
the exact cost that made `doc/hardware.md` reject this family for the glider's attitude role. Skipping
it saves a driver and buys nothing away.

### The one number to watch

Expected peak specific force is `25 N / (0.225 kg × 9.81) ≈ **11.3 g**`, against the BMI323's ±16 g
ceiling — **1.4× margin**, with no measured ignition transient anywhere in this project to bound the
spike. If it rails you still learn *"peak exceeded 16 g"*, which answers "did the booster survive"
but not "by how much". If the exact peak is the point, add an **ADXL375** (±200 g, already owned,
driver already written) beside it.

## Mechanical

PCB is **24 × 19 mm**, mounting holes **19 mm apart, 2 mm diameter**. That is within a millimetre or two
of a C6 SuperMini's footprint, so the sandwich stacks squarely — board, cell, board — rather than
needing an offset. Plan the stack height around the LiPo, which will be the thickest layer.

## Wiring

The SEN0697 breaks out two sides. Either works; they differ only in whether you take the interrupts.

**Verified against the ESP32-C6 datasheet (ch. 3 Boot Configurations) and ESP-IDF, 2026-09-15:**

| pin | status | why |
|---|---|---|
| **GPIO12, GPIO13** | **do not use** | native USB D−/D+. ESP-IDF: *"GPIO12 and GPIO13 are used by USB-JTAG by default. If they are reconfigured to operate as normal GPIOs, USB-JTAG functionality will be disabled."* This is the dev cable, the console and the flash read-out path. |
| **GPIO9** | **do not use** | the only pin that actually gates boot. Held LOW at reset the chip enters the ROM serial bootloader instead of running flash. It is the BOOT button, and it has an internal pull-up. |
| **GPIO8** | avoid, but **not** for the usual reason | Table 3-3 says SPI boot is `GPIO8 = any value, GPIO9 = 1` — GPIO8 is *ignored* in normal boot, so a sensor INT idling low here does **not** stop the board booting. What it does cost: BOOT pulls GPIO9 low, and `GPIO8 = 0` + `GPIO9 = 0` is documented as *"invalid and will trigger unexpected behavior"* — so you permanently lose the button route into the bootloader. On a soldered payload in a nose cone that is your last hardware recovery path. On the SuperMini it is also the **WS2812 RGB LED** data line. |
| **GPIO15** | free at chip level, **but it is the status LED** | appears only in Table 3-7 (JTAG signal source) and is *"Ignored"* with factory eFuses. Harmless electrically — but on the C6 SuperMini it drives the plain status LED, which is the liveness indicator this payload wants. |
| **GPIO10, GPIO11** | **not present** | ESP-IDF: on SiP-flash variants *"the SPI0/1 pins and GPIO10 ~ GPIO11 are not led out"*. The SuperMini is an ESP32-C6FH4 with in-package flash. |
| **GPIO24–GPIO30** | **not present** | SPI flash pins; absent on the SiP package. |
| **GPIO16, GPIO17** | avoid | default UART0 TX/RX. |

Usable set on this package: **GPIO0–GPIO9 and GPIO12–GPIO23**, minus the rows above. GPIO14 *does* exist
here — it is led out only on the SiP-flash variants, which is what a SuperMini is.

### What to solder — five wires, all on the left header

Direct to the ESP32-C6 pads, skipping the Fermion breakout, which is the compact build.

| SEN0697 | → ESP32-C6 | why this pin |
|---|---|---|
| 3V3 | **3V3** | board is 3.3 V, draws single-digit mA |
| GND | **GND** | |
| SCL | **GPIO20** | clean, adjacent |
| SDA | **GPIO19** | clean, adjacent |
| INT1 | **GPIO18** | BMI323 FIFO watermark — the only interrupt this mission needs |

**INT2, INT3 and INT4 are deliberately not wired.** INT2 is the BMI323's second channel, INT3 belongs
to the unused magnetometer, INT4 to a barometer that polls comfortably at ≤240 Hz. Leaving them off is
not a compromise for compactness — it is what keeps **GPIO8, GPIO9 and GPIO15 free**, which buys back
three things worth more than the interrupts: the BOOT-button recovery path, the plain status LED, and
the WS2812. Two independent indicators means the armed / recording / saved states can be signalled in
both blink pattern *and* colour, readable from outside the airframe without a USB cable.

### The FIFO is polled, and INT1 is spare

Both sensors' FIFOs are drained every 10 ms, so the sample rate is set by the sensor rather than by how
fast MicroPython can loop — this project measured a ~10 ms scheduling floor on the ESP32 asyncio port,
which alone would cap a Python-rate loop near 100 Hz and miss a boost transient.

Draining needs no interrupt: the loop reads `FIFO_FILL_LEVEL` each tick and takes whatever is queued.
**`main.py` never touches GPIO18.** The wire is still worth having — one conductor, and it leaves a
watermark interrupt available if a later build wants to sleep between drains — but nothing today depends
on it, and a harness without it behaves identically.

> Confirm your **SuperMini's** silkscreen against the GPIO numbers above before soldering — clone
> variants differ, and the C6 SuperMini is not a DFRobot board.

### Battery and arming switch

| from | to | note |
|---|---|---|
| LiPo **+** | switch pole → **`B+`** pad | switch goes in the **positive** line |
| LiPo **−** | **`B−`** pad | direct, no switch |

**Switch the positive line, never ground.** USB carries its own ground, so a switch in `B−` does not
isolate the cell while the dev cable is plugged in — you get a half-powered board and a confusing bench
session. In `B+` the behaviour is clean: switch OFF and the board runs from USB alone, switch ON and it
runs from the cell. Note the charger is downstream of the switch, so the cell only charges with the
switch ON.

**A slide switch can vibrate open under boost.** That loses the flight, and it loses it silently. Either
use a screw/twist switch (standard rocketry practice for exactly this reason), or secure the slide
switch mechanically once armed — a wrap of tape or a shrink sleeve over the actuator is enough. Whatever
you choose has to be reachable with the airframe closed, since the arming order is: switch on, confirm
the LED, *then* close up.

### Pull-ups and power

- **The wiki does not say whether I²C pull-ups are fitted.** Measure SDA→3V3 and SCL→3V3 with the board
  unpowered before assuming; if they read open, add 4.7 kΩ each. Operating voltage is **3.3 V**.
- This SuperMini variant **does** carry a charger: `B+` / `B−` solder pads on the underside plus a green
  charge-indicator LED driven by the charger IC (not by a GPIO) — on while charging, off with a battery
  connected, blinking with no battery. Wire the cell to `B+`/`B−`, never to the 3V3 rail.
- Add a **bulk capacitor (100–470 µF)** across 3V3. The flight record lives in RAM until landing, so a
  brownout is not a glitch — it is total data loss.

## Expected I²C addresses

From the DFRobot wiki (https://wiki.dfrobot.com/sen0697/). Note every default is the **upper**
address of its pair — a scan that finds `0x68 / 0x14 / 0x46` means the straps are pulled the other way,
not that something is broken.

| chip | default | alternate |
|---|---|---|
| BMI323 | **`0x69`** | `0x68` |
| BMM350 | **`0x15`** | `0x14` |
| BMP581 | **`0x47`** | `0x46` |

## Development

**Connect the C6 straight to the PC, not through a hub.** On 2026-09-15 the replacement bench hub dropped
bytes in both directions: raw-REPL tools (mpremote, ampy, rshell) could not enter raw mode, file writes
did not commit, and the board's own echo came back with characters missing. Plugged in directly, the
same transfer verified by SHA-256 and survived a soft reset on the first try.

The C6's USB is native (USB Serial/JTAG), which changes two things you might otherwise assume:

- **The baud setting is ignored.** Data moves at USB speed whatever the port says; "slower" means pacing
  the writes, not a lower baud.
- **DTR/RTS are wired to reset and the BOOT strap** — that is how esptool auto-resets. A tool that drops
  DTR while RTS is still high resets the chip on connect. That also wipes whatever is in RAM, which is
  why the firmware saves to flash rather than relying on a read-out over USB.

Interactive: `rshell -p /dev/ttyACM0`, then `repl`. First run after soldering:

```
ampy -p /dev/ttyACM0 -d 2 run src/logger/scan.py      # -d 2: let the board settle after the port opens
```

**Recovery** if a bad `main.py` ever wedges USB at every boot: hold BOOT, tap RST — the ROM download
mode — and reflash MicroPython with esptool. GPIO9 is left unwired precisely so this path stays open.
**A reflash erases the filesystem, logs included** — pull any flight off first (below); it is the last
resort, never the way to stop the logger.

## Firmware — `main.py`

Runs at boot and records until power-off. Nothing is filtered: a flight can be a start and a drop inside
one save window, so every sample is kept.

| | |
|---|---|
| rate | **50 Hz**, both sensors sampling on their own clocks into their own FIFOs |
| drain | every 10 ms — twice per frame period, so a frame never waits |
| record | 24 bytes, raw: `uint32 ms · int16 ax ay az gx gy gz · int32 pressure · int32 temperature` |
| autosave | a new file every **750 records** (~15 s, 18 KB) |
| still | **1 Hz** after 5 s of stillness; the first frame that moves restores 50 Hz |
| BOOT | saves the partial segment immediately — **3 blinks**, and sampling never pauses for them |
| LED (GPIO15) | toggles every ~0.5 s while recording |
| files | `bBBBBB_sSSSS.bin` — boot number (monotonic, kept in NVS) + segment; they sort chronologically |
| space | 2 MB filesystem, 1.94 MB free → **~28 minutes** at full rate, far longer with idle stretches |
| full flash | the **oldest** logs are deleted — only as many as the new segment needs, counted from their own sizes; `main.py` and `boot.py` are never touched. A save that still fails drops that one segment; recording continues |

Each file opens with a 20-byte header, `<4sHHHHII`: magic `CLG3`, record size, sample period (ms), boot,
segment, record count, and the MCU clock at the save.

**Why the FIFOs matter.** Programming flash freezes the single core — measured at 647–1434 ms per 72 KB
save, and 1221–2004 ms once the flash was full and each save also had to delete a file. Through that
freeze the BMI323 keeps queueing accel + gyro (~3.4 s deep at 50 Hz) and the BMP581 keeps queueing
pressure (32 frames, 0.64 s), and the next drain collects it all. Measured after the change: **0 ms of
extra gap at every save join**, across 11 saves, one of them containing 23 g impacts.

Temperature is read live rather than through the BMP581's FIFO: putting it there halves that FIFO from
32 frames to 16, and temperature moves far too slowly to need buffering.

**Timestamps come from the sensor's clock** — frame N is `t0 + N × 20 ms`, evenly spaced by construction,
with no MCU jitter. The header's MCU clock is the cross-check: a lost frame would show as the sensor
timeline falling a further 20 ms behind, and on the bench it sat a constant ~70 ms ahead instead.

**Stillness is judged frame to frame**, never against a stored resting pose — a pose keeps reading
"moved" after the airframe is set down in a new orientation, and the logger would never settle. The
thresholds are 0.05 g of frame-to-frame change, 5 dps on any axis, or ~2 m of climb against the mark a
second ago; resting noise measures ~5 LSB and ~1 dps, well clear of them.

**What you can still lose:**

- **Up to one segment at power-off** — whatever is in RAM since the last save. Press BOOT first if that
  tail matters; after a flight it is post-landing idle. While still, a segment takes minutes to fill.
- Nothing at start-up: the gyro reports `0x8000` until it has started, and `_setup()` waits for valid data
  before flushing the FIFO and beginning.
- **The flight itself, if the logger stays on after recovery.** Carrying the airframe back is motion, so
  it records at full rate, and a full flash deletes the **oldest** segments — ~28 minutes of handling
  overwrites the flight. **Switch it OFF at recovery.**

### Reading the logs off — the read-out procedure

The hardware watchdog (8 s) is fed only by the sampling loop, so a tool that stops that loop with Ctrl-C
used to let it fire mid-copy — and the reboot started recording, deleting the oldest segments once the
flash was full: the flight. Recording now starts at once but the watchdog is armed only **4 s after
boot**, and a Ctrl-C saves what is in RAM (only if it fits — it never deletes to make room) and stops:

1. **Power it on, plugged straight into the PC**, and run the copy, e.g.
   `rshell -p /dev/ttyACM0 cp '/pyboard/b*.bin' launches/<date>/<airframe>/logger/`.
2. If the tool connected within the 4 s window (a connect that resets the chip always does), the
   logger stops cleanly: `stopped for read-out`, **LED solid**, and the copy runs.
3. If the watchdog was already armed, the logger saves, flags read-out mode in RTC memory and resets
   itself — the tool loses the port once (it re-enumerates). **Run the same command again**: the board
   comes back in read-out mode — `READ-OUT MODE`, **LED solid**, not recording, no watchdog.
4. **A power-cycle returns it to recording** — RTC memory does not survive power-off. A software reset
   does not, so read-out mode holds across as many tool connects as the copy takes.

## Where the data lives

Logs belong to the **flight**, not to this directory: pull them into
`launches/<date>/<airframe>/logger/` (e.g. `launches/20261003/TMS-7/logger/`) so a capture sits beside
the airframe's mass, motor and configuration. Bench captures that prove something about the firmware go
in the commit that changes it; routine desk recordings are not worth keeping, and CSVs never are —
`decode.py` regenerates them from the `.bin` at any time.

## Reading the data — `decode.py`

```
python3 src/logger/decode.py launches/20261003/TMS-7/logger/*.bin
```

Writes a CSV beside each file, one row per sample. A file cut short (the header counts more records than
it holds) is decoded as far as it goes and says `truncated: N of M`; one that is not a logger file at all
is reported `SKIPPED` and the rest still decode.

`index, t_s, dt_ms, ax_g, ay_g, az_g, a_g, gx_dps, gy_dps, gz_dps, pressure_pa, temp_c, alt_rel_m, tick_ms`

- `0x8000` — the BMI323's "no sample yet" — becomes an **empty cell**, never −2000 dps. Before that rule
  existed, two start-up samples made a handheld shake summarise as a 2000 dps peak.
- `alt_rel_m` is height above the file's first sample, **hypsometric with the measured temperature**. The
  standard-atmosphere formula assumes 15 °C, and height per pascal scales with absolute temperature, so a
  30 °C pad reads ~5% low — about 15 m on a 300 m apogee. The BMP581 die sits beside the MCU and reads a
  few degrees warm, which is still far closer than 15 °C.

## Bench results — 2026-09-15

| | |
|---|---|
| heap free after boot | 332 KB |
| sample rate | **50.0 Hz**, flat 20 ms from the sensor clock (the earlier polled build: 99.9 Hz, 10–11 ms) |
| accel scale | per-sample \|a\| median **1.000 g** at rest — ±16 g at 2048 LSB/g confirmed against gravity |
| pressure noise | 4 Pa spread while still (~0.3 m) |
| hand shake | peak 2.7 g, 549 dps |
| gyro bias | **≤1 dps** at rest, 29 °C |

## Carried over from the static-burn logger

`tools/c3_burn_logger.py` (ESP32-C3, static burn + separation ground test) is not the base for this code.
Of its three rules, two hold and one was overtaken:

- **Rotate files** — holds. A restart or brownout never overwrites an earlier capture.
- **Decimate while idle** — holds, and is why ten minutes on the ground costs ~14 KB rather than ~720 KB.
  That logger dropped to 1 row/s while waiting for ignition; this one keys on stillness itself, so it
  applies on the pad, under the chute and after landing without having to know which is which.
- **Never write flash while sampling** — overtaken rather than traded away. It was the right rule for a
  build that polled its sensors, and this one records from power-up precisely because a missed BOOT press
  must not cost the flight. The FIFOs removed the conflict instead of splitting the difference: the save
  still freezes the core, and no samples are lost to it.
