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
| I²C0 SDA | 7 | sensors: BNO055, ICP-10111, BMP280, laser AGL (shared) |
| I²C0 SCL | 8 | |
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
| Spare / expansion | 28, 29, 30, 31, 50, 51, 52 | future sensors, second I²C, etc. |

### Caveats

- **No onboard user LED** — the only LED is a hardwired power indicator. The status-LED role
  needs an external LED on a free GPIO (GPIO2 above).
- I²C is **shared with the ES8311 codec**; harmless (different addresses) as long as we don't
  also drive GPIO9–13/53 for audio.
- If a future revision needs the microSD or audio, GPIO39–44 / GPIO9–13 / GPIO53 come back into
  play — keep them out of the free pool then.
- Never assign GPIO6/14–19/54 (Wi-Fi) or GPIO37/38 (console) in `board.config`.

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
