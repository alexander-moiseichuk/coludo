# Board probe rounds — constructable buses per board

Output of [`tools/board_probe.py`](../tools/board_probe.py) run on each physical board. One section
per board (a "round"), since the constructable UART/SPI/I2C ids and their default pins are
board/port-specific — only the *performance* numbers are shared (see `doc/benches/`).

Re-run after a board swap:

```
mpremote connect /dev/ttyACM0 run tools/board_probe.py
```

> The probe **streams** results and probes I2C **last**: `I2C(2)` hard-crashes this port (Core 1
> "Interrupt wdt timeout", followed by a reboot) — a fault no `try/except` can catch. Everything
> printed before the crash is valid; the reboot is the expected end of the run.

These are the *defaults* the MicroPython port hands out per id — overlapping pins across UART/SPI/I2C
ids (e.g. GPIO 9 appears as `UART(1).rx`, `SPI(1).sck`, and `I2C(1).scl`) are just unconfigured
defaults, not a wiring claim. The authoritative wiring is `board.json` (see
[`tools/board_pinmap.py`](../tools/board_pinmap.py)).

## WaveShare ESP32-P4-WIFI6 — round 2026-06-17

| | |
|---|---|
| USB | `1a86:55d3` (WCH CH343) |
| `machine` | Generic ESP32P4 module with WIFI module of external ESP32C6 with ESP32P4 |
| `release` | 1.28.0 |
| `unique_id` | `e8f60ae0eca4` |
| `freq` | 360 MHz |
| `Pin.board` | `[]` (port exposes no named board pins) |

| bus | id | default pins | note |
|---|---|---|---|
| UART | 0 | — | `ESP_ERR_INVALID_STATE` — in use by the REPL |
| UART | 1 | tx=10 rx=9 | |
| UART | 2 | tx=17 rx=16 | |
| UART | 3 | tx=0 rx=0 | placeholder defaults |
| UART | 4 | tx=0 rx=0 | placeholder defaults |
| UART | 5 | tx=5 rx=4 | default baud 57600 (others 115200) |
| SPI | 0 | — | does not exist |
| SPI | 1 | sck=9 mosi=8 miso=10 | |
| SPI | 2 | sck=43 mosi=44 miso=39 | |
| SPI | 3–5 | — | do not exist |
| I2C | 0 | scl=18 sda=19 | |
| I2C | 1 | scl=9 sda=8 | |
| I2C | 2 | — | **HARD CRASH** → Core 1 WDT timeout, board reboots |

## DFRobot FireBeetle 2 ESP32-P4 — round pending

Not captured this round — the FireBeetle 2 is disconnected (WaveShare is back on `/dev/ttyACM0`).
USB id when present is native Espressif `303a:1001` (see the
[DFRobot bench report](benches/DFRobot_esp32p4-micropython-findings.md)). Reconnect it and re-run
the command above to fill this section.
