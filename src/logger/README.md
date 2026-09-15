# TMS-7 nose logger — ESP32-C6 + SEN0697

A standalone flight recorder for **TMS-7**, the booster-only vertical test (119.5 g airframe +
98.8 g F15 = **218.3 g liftoff**). TMS-7 carries no electronics and returns no data; this payload
makes the flight yield **peak boost acceleration** and **apogee altitude** for the cost of a few grams
in the nose.

It talks to nothing. No radio in flight, no link to the main board — a sandwich of
**ESP32-C6 SuperMini + LiPo + SEN0697**, recording to RAM and flushing to flash after landing. USB is
the development and read-out path, and the only liveness check.

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

### Why the FIFO interrupt is the one that matters

The plan runs the BMI323's own FIFO at high ODR and drains it in blocks, so the sample rate is set by
the sensor rather than by how fast MicroPython can loop — this project measured a ~10 ms scheduling
floor on the ESP32 asyncio port, which alone would cap a Python-rate loop near 100 Hz and miss a boost
transient. Draining a FIFO decouples the two, and it needs exactly one line: the BMI323's
**FIFO-watermark interrupt**, which the DFRobot wiki assigns to **INT1**.

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

## Bring-up order

1. Solder the sandwich, USB in.
2. `mpremote connect /dev/ttyACM0 run scan.py` — every expected address answers, chip IDs read back.
3. Only then write the sampling path.

## Notes carried over from the earlier static-burn logger

`tools/c3_burn_logger.py` (ESP32-C3, static burn + separation ground test) is **not** the base for this
code, but three of its conclusions were paid for already and hold here:

- **Never write flash while sampling.** Buffer every row in RAM; write the whole file at once, at a
  point where a multi-millisecond erase stall cannot collide with a peak.
- **Latch a flight state.** Decimate hard while idle on the pad, switch to full rate on the launch
  spike, and flush once the event is over — not at power-off.
- **Rotate files.** A restart or brownout must never overwrite an earlier capture.

Two things change for a flight rather than a static burn: the flight window must close on **landing**,
not 10 s after the g-spikes stop (which in the air is apogee), and `time.ticks_us()` wraps at
**17.9 minutes** of uptime — clear of a 5–10 minute power-on window, but the stamp is raw uptime, so
it is a real edge if the pad wait ever runs long.
