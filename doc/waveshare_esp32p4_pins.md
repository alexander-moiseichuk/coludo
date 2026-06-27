# WaveShare ESP32-P4-WIFI6 — pin map

Pin reference for the Coludo Main Controller board
([WaveShare ESP32-P4-WIFI6](https://www.waveshare.com/esp32-p4-wifi6.htm)), derived from the
official [board schematic](https://files.waveshare.com/wiki/ESP32-P4-WIFI6/ESP32-P4-WIFI6-datasheet.pdf)
(net-by-net), to seed the `buses`/`pins` sections of `board.config` (see
[`../specs/board-config.md`](../specs/board-config.md)). The ESP32-P4 has 55 GPIOs (GPIO0–54);
PSRAM is in-package (not pinned out). The 40-pin header is Pico-style and the **GPIO number is
silk-printed on each header pin**, so locating a pin is trivial.

## Reserved / occupied GPIOs — do NOT reuse

| GPIO(s) | Used by | Note |
|---|---|---|
| **6, 14, 15, 16, 17, 18, 19, 54** | **ESP32-C6 Wi-Fi** (SDIO data/clk/cmd + control/enable) | Using any of these **breaks Wi-Fi** — the C6 co-processor link |
| **37 (TX), 38 (RX)** | USB-UART console / REPL (CH343) | Also strapping pins; this is the `mpremote` link |
| 9, 10, 11, 12, 13 | ES8311 audio codec (I²S MCLK/SCLK/LRCK/ADC/DAC) | Free only if audio is unused (routed to the codec chip) |
| 53 | Speaker amplifier (PA) enable | Free only if audio is unused |
| 39, 40, 41, 42, 43, 44 | microSD slot (SDIO 4-bit: D0/D1/D2/D3/CLK/CMD) | Free only if the onboard TF card is unused |
| 24, 25 | USB (second USB PHY / USB-JTAG) | |
| 0, 34, 35, 36 | Boot strapping / BOOT button / test points (TP1/TP2) | Avoid; 35 = BOOT key |
| 45 | 3V3 load-switch control (Q1) | Verify before repurposing |
| — | Flash (dedicated pins), PSRAM (in-package) | Not on the GPIO range |

## I²C (shared bus)

The board's default I²C is **SDA = GPIO7, SCL = GPIO8**, broken out on the header and also used
by the ES8311 codec. Our sensors share this bus (distinct addresses), so no extra pins are needed.

Addresses on I²C0 (no conflicts): BNO055 `0x28`, VL53L4CX `0x29`, ICP-10111 `0x63`, BMP280 `0x76`.
The two **raw high-rate IMUs share SPI1** instead — ADXL375 and LSM6DSO32, each on its own
chip-select (see SPI below) — so the fast accel/gyro reads never wait on the shared I²C bus. The
"i²c PCB" carries BNO055 + the baros + laser; the "spi PCB" carries ADXL375 + LSM6DSO32.

## Free header GPIOs

Available on the 2×20 header for our use (none tied to an onboard peripheral):

```
GPIO2  GPIO3  GPIO4  GPIO5  GPIO20 GPIO21 GPIO22 GPIO23 GPIO26 GPIO27
GPIO28 GPIO29 GPIO30 GPIO31 GPIO32 GPIO33 GPIO46 GPIO47 GPIO48 GPIO49
GPIO50 GPIO51 GPIO52
```

All ESP32-P4 HP GPIOs support LEDC PWM (servos) and pin interrupts (separation switch) via the
GPIO matrix, so any free pin works for those roles. Per the Espressif GPIO reference there are
**no input-only pins** (every GPIO0–54 is bidirectional), and **ADC** is available on
GPIO16–23 and GPIO49–54 — so a spare free pin in 20–23 or 49–52 can double as an analog input
(e.g. battery-voltage sense). Touch is on GPIO2–15.

## Recommended Coludo pin map

A conflict-free starting assignment (drop into `board.config`):

| Bus | Function | GPIO | Device.pin | Note |
|---|---|---|---|---|
| `i2c.0` | SDA | 7 | bno055.sda · icp10111.sda · bmp280.sda · vl53l4cx.sda | shared bus |
| `i2c.0` | SCL | 8 | bno055.scl · icp10111.scl · bmp280.scl · vl53l4cx.scl | shared bus |
| `spi.1` | SCK | 48 | adxl375.scl · lsm6dso32.scl | shared clock, mode 3, 5 MHz |
| `spi.1` | MOSI | 47 | adxl375.sdi · lsm6dso32.sdi | shared |
| `spi.1` | MISO | 46 | adxl375.sdo · lsm6dso32.sdo | shared |
| `spi.1` | CS | 49 | adxl375.cs | `adxl375_cs`, active low |
| `spi.1` | CS | 50 | lsm6dso32.cs | `lsm6dso32_cs`, active low |
| `gpio` | INT | 4 | adxl375.int1 | `adxl375_int`, DATA_READY → sampling |
| `gpio` | INT | 28 | lsm6dso32.int1 | `lsm6dso32_int1`, data-ready → sampling |
| `gpio` | enable | 5 | vl53l4cx.xshut | `laser_xshut`, active low |
| `gpio` | INT | 3 | vl53l4cx.gpio1 | `laser_int`, data-ready |
| `uart.1` | TX | 20 | luckfox.rx | Recorder sink, 921600 baud |
| `uart.1` | RX | 21 | luckfox.tx | optional (one-way link needs only TX) |
| `uart.2` | TX | 22 | atgm336h.rx | GNSS config out (`$PCAS…`) |
| `uart.2` | RX | 23 | atgm336h.tx | NMEA in |
| `pwm` | signal | 26 | servo_yaw.sig | LEDC, yaw (vertical fin) |
| `pwm` | signal | 27 | servo_eleron_left.sig | LEDC |
| `pwm` | signal | 32 | servo_eleron_right.sig | LEDC |
| `gpio` | switch | 33 | separation.pad | `PULL_DOWN`, IRQ (HIGH=nested, LOW=separated) |
| `gpio` | LED | 2 | led.anode | external LED + resistor (no onboard user LED) |
| — | spare | 29, 30, 31, 51, 52 | — | future sensors, 2nd I²C, battery sense (29 = LSM6DSO32 INT2, unwired) |

### Caveats

- **No onboard user LED** — the only LED is a hardwired power indicator. The status-LED role
  needs an external LED on a free GPIO (GPIO2 above).
- I²C is **shared with the ES8311 codec**; harmless (different addresses) as long as we don't
  also drive GPIO9–13/53 for audio.
- If a future revision needs the microSD or audio, GPIO39–44 / GPIO9–13 / GPIO53 come back into
  play — keep them out of the free pool then.
- Never assign GPIO6/14–19/54 (Wi-Fi) or GPIO37/38 (console) in `board.config`.

## Per-device wiring — device.pin → connection

The reverse of the pin map above: grab one device, walk down its pins, wire each where the **→**
column says (a GPIO, the **3V3** rail, the **5V/BEC** rail, **GND**, or **NC** = leave unconnected).
Breakout silk varies, so each pin lists the aliases you may see printed. `NC` pins stay open.

The two **raw IMUs (ADXL375 + LSM6DSO32) share SPI1** — same SCK/MOSI/MISO, one chip-select each.
SPI mode 3 (CPOL=CPHA=1) suits both; bus runs at 5 MHz (ADXL375's ceiling; LSM6DSO32 does 10 MHz, so
no limit hit). On a SPI part, **CS is the real active-low select** (no "tie it high" as in I²C) and
**SDO is plain MISO** (not an address pin) — so neither IMU needs an address strap.

| Device | Pin (aliases) | → | Note |
|---|---|---|---|
| **adxl375** (SPI, ±200 g) | VIN / VDD | 3V3 | breakout takes 3–5 V |
| | GND | GND | |
| | SCL / SCK | GPIO48 | `spi.1` SCK (shared) |
| | SDI / SDA / MOSI | GPIO47 | `spi.1` MOSI (shared) |
| | SDO / MISO | GPIO46 | `spi.1` MISO (shared) |
| | CS / NCS | GPIO49 | `adxl375_cs`, active low |
| | INT1 | GPIO4 | `adxl375_int`, DATA_READY |
| | INT2 | NC | unused |
| **lsm6dso32** (SPI, 6-DoF) | VIN / VDD | 3V3 | breakout 3–5 V; bare chip 1.71–3.6 V |
| | GND | GND | |
| | SCL / SPC / SCK | GPIO48 | `spi.1` SCK (shared) |
| | SDA / SDI / MOSI | GPIO47 | `spi.1` MOSI (shared) |
| | SDO / MISO | GPIO46 | `spi.1` MISO — data out in SPI, **not** an address pin |
| | CS / NCS | GPIO50 | `lsm6dso32_cs`, active low (LOW = SPI mode, which we want) |
| | INT1 / I1 | GPIO28 | `lsm6dso32_int1`, data-ready |
| | INT2 / I2 | **NC** | not wired (polling INT1); GPIO29 stays in the spare pool |
| | SDX / SCX | NC | sensor-hub aux master, unused |
| | DEN | NC | external sync stamp, unused |
| | 3V / 3Vo | NC | regulator output, do not drive |
| **bno055** (I²C, fused IMU) | VIN | 3V3 | |
| | GND | GND | |
| | SDA | GPIO7 | `i2c.0` (shared) |
| | SCL | GPIO8 | `i2c.0` (shared) |
| | ADR / ADDR | GND | addr **0x28** (3V3 → 0x29, which collides with the laser — keep low) |
| | PS0 / PS1 | GND | I²C mode select (both low); fixed on Adafruit boards |
| | RST / INT | NC | unused |
| **icp10111** (I²C, baro) | VIN | 3V3 | DFRobot SEN0517 |
| | GND | GND | |
| | SDA | GPIO7 | `i2c.0` (shared); fixed addr **0x63**, no INT |
| | SCL | GPIO8 | `i2c.0` (shared) |
| **bmp280** (I²C, baro) | VIN / VCC | 3V3 | |
| | GND | GND | |
| | SDA / SDI | GPIO7 | `i2c.0` (shared) |
| | SCL / SCK | GPIO8 | `i2c.0` (shared) |
| | SDO | GND | addr **0x76** (3V3 → 0x77) |
| | CSB | 3V3 | tie HIGH to select I²C (LOW = SPI) |
| **vl53l4cx** (I²C, laser AGL) | VIN | 3V3 | |
| | GND | GND | |
| | SDA | GPIO7 | `i2c.0` (shared), addr **0x29** |
| | SCL | GPIO8 | `i2c.0` (shared) |
| | XSHUT | GPIO5 | `laser_xshut`, enable/reset (active low) |
| | GPIO1 / INT | GPIO3 | `laser_int`, data-ready |
| **atgm336h** (UART, GNSS) | VCC | 3V3 | |
| | GND | GND | |
| | TX | GPIO23 | → board `uart.2` RX (NMEA in) |
| | RX | GPIO22 | ← board `uart.2` TX (`$PCAS…` config out) |
| | PPS | NC | unused |
| **luckfox** (UART, Recorder) | RX | GPIO20 | ← board `uart.1` TX (log/telemetry sink, 921600) |
| | TX | GPIO21 | → board `uart.1` RX (optional) |
| | GND | GND | common ground with the MCU |
| **servo ×3** (SG90, fins) | VCC (red) | 5V / BEC | **own servo rail, NOT 3V3** |
| | GND (brown) | GND | common with the MCU |
| | signal (orange) | GPIO26 / 27 / 32 | yaw / left-eleron / right-eleron PWM |
| **separation** (copper pads) | hot pad | 3V3 | routed to the sense pin while nested |
| | sense pad | GPIO33 | `separation_switch`, internal `PULL_DOWN`: **HIGH=nested, LOW=separated** |
| **status LED** (external) | anode | GPIO2 | via series resistor |
| | cathode | GND | |

**LSM6DSO32 config** (drop into `board.config` when the `lsm6dso32` driver lands tomorrow):

```python
'pins': { ..., 'lsm6dso32_cs': 50, 'lsm6dso32_int1': 28 },   # INT2 not wired (GPIO29 free)
'sensors': [ ...,
    { 'name': 'imu_lsm6dso32', 'driver': 'lsm6dso32',
      'bus': 'spi', 'id': 1,             # shares ADXL375's SPI1 (mode 3, 5 MHz)
      'cs_pin': 'lsm6dso32_cs',          # its own chip-select (GPIO50)
      'int_pin': 'lsm6dso32_int1',       # INT1 data-ready paces the sampling
      'telemetry_us': 10000,             # ~100 Hz raw 6-DoF, decimated in imu_lsm6dso32.csv
      'enabled': True,
      'provides': { 'accel': {'priority': 0, 'timeout_ms': 20},   # PRIMARY: +-32 g covers the ~12 g
                    'rate':  {'priority': 0, 'timeout_ms': 20} } },  # boost without clipping AND has
]                                                                    # fine low-g resolution for the
```                                                                  # glide integrator; gyro = sole rate

# This makes LSM6DSO32 the lead accel. Bump the existing entries to match: adxl375 accel -> priority 1
# (the >+-32 g high-g backstop), bno055 accel -> priority 2 (fused fallback). The +-200 g ADXL375 is
# ~50x coarser at 1 g, so it should NOT lead the glide-phase airspeed integrator.

## Firmware peripheral defaults — do not rely on them

This is a *generic* ESP32-P4 MicroPython build, so `machine` bus constructors return default
pins unrelated to this board's wiring. **Always pass explicit pins** (which is what `board.config`
does):

| constructor | firmware default pins | problem |
|---|---|---|
| `I2C(0)` | scl=18, sda=19 | **GPIO18/19 are the C6 Wi-Fi SDIO lines** — the default fights Wi-Fi |
| `I2C(1)` | scl=9, sda=8 | GPIO9 is a codec I²S pin |
| `SPI(1)` | sck=9, mosi=8, miso=10 | codec / I²C pins |
| `SPI(2)` | sck=43, mosi=44, miso=39 | **the microSD pins** |

Two hardware I²C controllers exist (`I2C(0)`/`I2C(1)`; `I2C(2)` **hard-crashes** the board — see
the [benchmark findings](benches/WaveShare_esp32p4-micropython-findings.md)). Two hardware SPI controllers
exist (`SPI(1)`/`SPI(2)`; `SPI(0)`/`SPI(3)` raise `ValueError`). Any controller remaps to
arbitrary GPIOs via the matrix.

**Validated on hardware** (MicroPython v1.28.0): the recommended map all constructs cleanly —
`I2C(0, scl=8, sda=7)`, `UART(1, tx=20, rx=21, 921600)` (comes up ~922190 baud, within UART
tolerance), `UART(2, tx=22, rx=23, 9600)`, `PWM` on 26/27/32 at 50 Hz, `Pin(33, IN, PULL_DOWN)`
reading LOW when the pads are open (= separated; HIGH while nested), and `Pin(2, OUT)` for the status LED.

## Sources

- Board schematic (authoritative for this board's net routing):
  [ESP32-P4-WIFI6 schematic PDF](https://files.waveshare.com/wiki/ESP32-P4-WIFI6/ESP32-P4-WIFI6-datasheet.pdf)
  and the [WaveShare wiki](https://docs.waveshare.com/ESP32-P4-WIFI6).
- ESP32-P4 GPIO categories (strapping 34–38, USB-JTAG 24/25, ADC/touch, no input-only pins):
  [Espressif ESP-IDF GPIO reference](https://docs.espressif.com/projects/esp-idf/en/stable/esp32p4/api-reference/peripherals/gpio.html).
- Chip-level pin overview: [esp32pins.com — ESP32-P4](https://esp32pins.com/boards/esp32-p4/).
