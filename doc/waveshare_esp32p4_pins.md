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

Addresses on I²C0 (no conflicts): BNO055 `0x28`, VL53L4CX `0x29`, ICP-10111 `0x63`, LSM6DSO32
`0x6A` (SA0/SDO low; `0x6B` if SA0 high), BMP280 `0x76`. The high-g ADXL375 is the one IMU **off**
this bus — it has its own SPI1 (clean high-rate reads). So the "i²c PCB" carries BNO055 + LSM6DSO32
+ the baros + laser; the "spi PCB" carries the ADXL375.

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

| Function | GPIO | Notes |
|---|---|---|
| I²C0 SDA | 7 | sensors: BNO055, LSM6DSO32, ICP-10111, BMP280, laser AGL (shared) |
| I²C0 SCL | 8 | |
| LSM6DSO32 INT1 | 28 | 6-DoF data-ready (`lsm6dso32_int1`) — drives the IMU sampling |
| LSM6DSO32 INT2 | 29 | optional second interrupt (`lsm6dso32_int2`); leave NC if unused |
| SPI1 SCK | 48 | ADXL375 (its own bus, mode 3, 5 MHz — off I²C for clean high-rate reads) |
| SPI1 MOSI | 47 | ADXL375 SDA/SDI |
| SPI1 MISO | 46 | ADXL375 SDO |
| ADXL375 CS | 49 | SPI chip-select (`adxl375_cs`), active low |
| ADXL375 INT1 | 4 | DATA_READY (`adxl375_int`) — drives interrupt sampling |
| VL53L4CX XSHUT | 5 | laser enable/reset (`laser_xshut`), active low |
| VL53L4CX GPIO1 | 3 | laser data-ready interrupt (`laser_int`) |
| UART → Recorder TX | 20 | to Luckfox RX, 921600 baud (logs/telemetry sink) |
| UART → Recorder RX | 21 | optional (one-way link only needs TX) |
| UART ↔ GNSS TX | 22 | to ATGM336H RX (send `$PCAS…` config) |
| UART ↔ GNSS RX | 23 | from ATGM336H TX (NMEA in) |
| Servo — yaw (vertical fin) | 26 | LEDC PWM |
| Servo — left eleron | 27 | LEDC PWM (`servo_eleron_left`) |
| Servo — right eleron | 32 | LEDC PWM (`servo_eleron_right`) |
| Separation switch | 33 | input, `PULL_UP`, IRQ (LOW=nested, HIGH=separated) |
| Status LED (external) | 2 | **no onboard user LED** — wire an external LED+resistor |
| Spare / expansion | 30, 31, 50, 51, 52 | future sensors, second I²C, etc. |

### Caveats

- **No onboard user LED** — the only LED is a hardwired power indicator. The status-LED role
  needs an external LED on a free GPIO (GPIO2 above).
- I²C is **shared with the ES8311 codec**; harmless (different addresses) as long as we don't
  also drive GPIO9–13/53 for audio.
- If a future revision needs the microSD or audio, GPIO39–44 / GPIO9–13 / GPIO53 come back into
  play — keep them out of the free pool then.
- Never assign GPIO6/14–19/54 (Wi-Fi) or GPIO37/38 (console) in `board.config`.

## LSM6DSO32 — 6-DoF IMU: per-pin wiring (assembly sheet)

The LSM6DSO32 (±32 g accel + ±2000 dps gyro, the g12 telemetry IMU) goes on the **shared I²C0 bus**
— it joins the BNO055/baro/laser "i²c PCB", while the high-g ADXL375 stays alone on SPI1. I²C is the
right call for tomorrow's bring-up: at ≤200 Hz the board only needs a ~12-byte burst read per sample
(~0.3 ms on the 400 kHz bus), assembly is two wires onto an existing bus, and the address (`0x6A`)
is free. (If a later launch wants raw multi-kHz logging, move it to SPI1 — see the alternative below.)

**Wire every pad as follows.** Breakout silk varies (Adafruit 4692, DFRobot, generic GY-boards), so
each row lists the aliases you may see printed; a 6-DoF that does both I²C and SPI gates the *mode* on
the **CS** pin, which is the one non-obvious connection — it MUST be tied HIGH for I²C.

| Device pad (silk aliases) | What it is | Wire to (MCU / rail) | Notes |
|---|---|---|---|
| `VIN` / `VCC` / `VDD` | power in | **3V3 rail** | Adafruit VIN accepts 3–5 V (onboard reg + level-shift); bare-chip VDD is 1.71–3.6 V — 3V3 is safe for both |
| `3V` / `3Vo` | regulated 3.3 V **out** | **leave NC** | breakout output, do not drive |
| `GND` | ground | **GND rail** | common ground with the MCU |
| `SCL` / `SPC` | I²C clock (SPI clock) | **GPIO8** (I²C0 SCL) | shared with the other I²C sensors |
| `SDA` / `SDI` / `SDIO` | I²C data (SPI MOSI) | **GPIO7** (I²C0 SDA) | shared with the other I²C sensors |
| `SDO` / `SA0` | **I²C address LSB** (SPI MISO) | **GND** | GND → `0x6A` (our default); 3V3 → `0x6B`. Tie it explicitly even if the breakout has a pull |
| `CS` / `CS̅` / `NCS` | chip / mode select | **3V3 rail** | **must be HIGH for I²C.** LOW would switch the part into SPI mode. Tie/pull to 3V3 |
| `INT1` / `I1` | interrupt 1 (data-ready / FIFO / wake) | **GPIO28** (`lsm6dso32_int1`) | primary — drives interrupt-paced sampling |
| `INT2` / `I2` | interrupt 2 | **GPIO29** (`lsm6dso32_int2`) or NC | optional; leave open if the driver polls one INT |
| `SDX` / `OCS` | sensor-hub **aux-master** SDA (Mode 2) | **leave NC** | for mastering an external slave; unused |
| `SCX` | sensor-hub aux-master SCL | **leave NC** | unused |
| `DEN` | data-enable / external sync stamp | **leave NC** | edge-stamp trigger; unused |

So the minimum buildable set is **seven wires**: VIN→3V3, GND→GND, SCL→GPIO8, SDA→GPIO7, SDO→GND,
CS→3V3, INT1→GPIO28 (INT2→GPIO29 optional). SDX/SCX/DEN/3Vo stay unconnected.

**Config (drop into `board.config` when the `lsm6dso32` driver lands).** The bus already exists; add
the INT pin name(s) and a sensor entry:

```python
# buses.i2c.0 unchanged — shared bus
'pins': { ..., 'lsm6dso32_int1': 28, 'lsm6dso32_int2': 29 },
'sensors': [ ...,
    { 'name': 'imu_lsm6dso32', 'driver': 'lsm6dso32',
      'bus': 'i2c', 'id': 0,
      'addr': 0x6A,                    # SA0/SDO low; 0x6B if SA0 tied high
      'int_pin': 'lsm6dso32_int1',     # INT1 data-ready paces the sampling
      'telemetry_us': 10000,           # ~100 Hz raw 6-DoF, decimated in imu_lsm6dso32.csv
      'enabled': True,
      'provides': { 'accel': {'priority': 2, 'timeout_ms': 20},   # behind adxl375(0), bno055(1)
                    'rate':  {'priority': 0, 'timeout_ms': 20} } },  # gyro: the only rate provider
]
```

**SPI alternative (only if a launch needs raw multi-kHz).** Share ADXL375's SPI1 with a *second*
chip-select: `CS`→a free pin (e.g. GPIO50, `lsm6dso32_cs`), `SCL`→GPIO48 (SCK), `SDA`→GPIO47 (MOSI),
`SDO`→GPIO46 (MISO), VIN/GND as above, INT1→GPIO28. Then the config entry is `'bus': 'spi', 'id': 1,
'cs_pin': 'lsm6dso32_cs'`. Not needed for tomorrow.

## Reverse wiring table — MCU pin → device pad (whole board)

Assembly check, going header-pin by header-pin: every MCU connection and which device pad lands on it.
Power rails first, then GPIOs in number order.

| MCU pin / rail | Goes to (device : pad) |
|---|---|
| **3V3** | BNO055 VIN · **LSM6DSO32 VIN** · **LSM6DSO32 CS (HIGH = I²C mode)** · ICP-10111 VIN · BMP280 VIN · VL53L4CX VIN · ADXL375 VIN · ATGM336H VCC · external-LED anode (via resistor) |
| **GND** | every device GND · **LSM6DSO32 SDO/SA0 (= addr 0x6A)** · separation-switch pad return · LED cathode |
| **GPIO7** (I²C0 SDA) | BNO055 SDA · **LSM6DSO32 SDA/SDI** · ICP-10111 SDA · BMP280 SDA · VL53L4CX SDA |
| **GPIO8** (I²C0 SCL) | BNO055 SCL · **LSM6DSO32 SCL/SPC** · ICP-10111 SCL · BMP280 SCL · VL53L4CX SCL |
| **GPIO48** (SPI1 SCK) | ADXL375 SCL/SCK |
| **GPIO47** (SPI1 MOSI) | ADXL375 SDA/SDI |
| **GPIO46** (SPI1 MISO) | ADXL375 SDO |
| **GPIO49** | ADXL375 CS |
| **GPIO4** | ADXL375 INT1 |
| **GPIO5** | VL53L4CX XSHUT (enable/reset) |
| **GPIO3** | VL53L4CX GPIO1 (data-ready INT) |
| **GPIO28** | **LSM6DSO32 INT1** |
| **GPIO29** | **LSM6DSO32 INT2** (optional) |
| **GPIO20** (UART1 TX) | Recorder (Luckfox) RX |
| **GPIO21** (UART1 RX) | Recorder TX (optional — one-way link only needs TX) |
| **GPIO22** (UART2 TX) | ATGM336H RX (`$PCAS…` config out) |
| **GPIO23** (UART2 RX) | ATGM336H TX (NMEA in) |
| **GPIO26** | servo — yaw fin (signal) |
| **GPIO27** | servo — left eleron (signal) |
| **GPIO32** | servo — right eleron (signal) |
| **GPIO33** | separation switch (pad: HIGH=nested, LOW=separated) |
| **GPIO2** | external status LED (anode via resistor) |

Servos take their **own 5 V/BEC power and ground** (not the 3V3 sensor rail) — only the signal pin
lands on the MCU; tie the servo ground to the MCU GND.

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
tolerance), `UART(2, tx=22, rx=23, 9600)`, `PWM` on 26/27/32 at 50 Hz, `Pin(33, IN, PULL_UP)`
reading HIGH (= separated), and `Pin(2, OUT)` for the status LED.

## Sources

- Board schematic (authoritative for this board's net routing):
  [ESP32-P4-WIFI6 schematic PDF](https://files.waveshare.com/wiki/ESP32-P4-WIFI6/ESP32-P4-WIFI6-datasheet.pdf)
  and the [WaveShare wiki](https://docs.waveshare.com/ESP32-P4-WIFI6).
- ESP32-P4 GPIO categories (strapping 34–38, USB-JTAG 24/25, ADC/touch, no input-only pins):
  [Espressif ESP-IDF GPIO reference](https://docs.espressif.com/projects/esp-idf/en/stable/esp32p4/api-reference/peripherals/gpio.html).
- Chip-level pin overview: [esp32pins.com — ESP32-P4](https://esp32pins.com/boards/esp32-p4/).
