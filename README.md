# coludo

This repository is to represent and control the rocket powered glider.
Since I do not know C/C++, it will be less complex than [HPR Rocket Flight Computer](https://github.com/SparkyVT/HPR-Rocket-Flight-Computer) this project will be written with MicroPython.
The composion of [hardware components](doc/hardware.md) is in progress with weight/power consumption restrictions in the account.

**Board revisions.** One firmware runs three of them and decides which it is on at boot, by I²C scan
([`layout.py`](src/glider/layout.py)) — no config edit when a board is rewired:

| rev | attitude module | also |
| --- | --- | --- |
| **v0.1** | sen0253 (BNO055 `0x28` + BMP280 `0x76`) | ADXL375 ±200 g on SPI |
| **v1.0** | sen0253 | ADXL375 removed; the ICP-10111, pitot and laser move to their own 100 kHz bus |
| **v1.1** | **sen0697** (BMI323 `0x69` + BMP581 `0x47` + **BMM350 `0x15`**) | first board with a magnetometer; the board fuses attitude itself |

The v1.1 change is measured in [TMS-7-board_v1.1_sen0697](doc/sims/TMS-7-board_v1.1_sen0697/):
the aircraft flies the same, and the magnetometer keeps it near the landing zone through a GNSS
blackout instead of letting it wander.

The old glider version TMS-4 can be ![found in here](https://github.com/alexander-moiseichuk/coludo/blob/main/doc/photos/TMS-4%20with%20electronics.jpg)

The glider version TMS-7 ![avaliable there](https://github.com/alexander-moiseichuk/coludo/blob/main/doc/photos/TMS-7_glider_static_burn.jpg)

## Where things live

**Specifications — [`doc/specs/`](doc/specs/)**
- [Architecture overview](doc/specs/coludo.md) — the **main description**: flight lifecycle (Setting → Boosting → Gliding → Landing), flight controller, sensors, telemetry and logging. Authoritative for flight behaviour.
- [Board configuration](doc/specs/board-config.md) — the controller's config schema, the three config layers, and the save/reboot activation lifecycle.
- [Control Center ↔ board protocol](doc/specs/cc-protocol.md) — the wire protocol between the ground station and the boards, plus the browser bridge.

**Documentation — [`doc/`](doc/)**
- [Hardware](doc/hardware.md) — parts list, weights, power budget, and candidate build configurations.
- [Data sources](doc/datasources.md) — every measured quantity with its primary source, backups and rates, per board revision, and which channels lose redundancy where.
- [WaveShare ESP32-P4-WIFI6 pin map](doc/waveshare_esp32p4_pins.md) — reserved vs free GPIOs and the recommended `board.config` pin assignment.
- [Tasks & plans](doc/plan.md) — required hardware checklist and the phased development roadmap.
- [Development & testing guide](doc/skills.md) — tooling (`ampy`/`mpremote`/`rshell`, `mpy-cross`), source layout, the `panda` test network, and the testing rules.
- [Flight simulations](doc/sims/) — closed-loop host + on-board HITL flights (noise/wind/corner sweeps) with interactive reports.
- [Benchmarks](doc/benches/) — board performance logs (BeagleBone, RPi4, StarFive).
- [Photos](doc/photos/) — build and electronics photos.
- [Videos](doc/videos/) — flight footage (e.g. the TMS-6 campaign).

**Models — [`models/`](models/)** — 3D-printable STL parts for the booster and glider prototypes (TMS-1 … TMS-7).

**Source — [`src/`](src/)**
- [`src/glider/`](src/glider/) — Main Controller flight firmware (MicroPython), with tests in `src/glider/test/`. *(implemented)*
- [`src/control/`](src/control/) — Control Center ground station (Python) + browser dashboard. *(implemented)*
- [`src/camera/`](src/camera/) — Recorder module (Luckfox Pico): 2304×1296 video + UART telemetry/log sink. *(implemented)*
- [`src/logger/`](src/logger/) — standalone nose logger (ESP32-C6 + SEN0697 + 120 mAh LiPo): a self-contained
  payload that records boost acceleration and apogee altitude for a flight the main controller is not on. *(implemented)*

**Tools — [`tools/`](tools/)** — various tools which helps in development and setup.

