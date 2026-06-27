# coludo

This repository is to represent and control the rocket powered glider.
Since I do not know C/C++, it will be less complex than [HPR Rocket Flight Computer](https://github.com/SparkyVT/HPR-Rocket-Flight-Computer) this project will be written with MicroPython.
The composion of [hardware components](doc/hardware.md) is in progress with weight/power consumption restrictions in the account.

The way the glider looks can be ![found in here](https://github.com/alexander-moiseichuk/coludo/blob/main/doc/photos/TMS-4%20with%20electronics.jpg)

## Where things live

**Specifications — [`specs/`](specs/)**
- [Architecture overview](specs/coludo.md) — the **main description**: flight lifecycle (Setting → Boosting → Gliding → Landing), flight controller, sensors, telemetry and logging. Authoritative for flight behaviour.
- [Board configuration](specs/board-config.md) — the controller's config schema, the three config layers, and the save/reboot activation lifecycle.
- [Control Center ↔ board protocol](specs/cc-protocol.md) — the wire protocol between the ground station and the boards, plus the browser bridge.

**Documentation — [`doc/`](doc/)**
- [Hardware](doc/hardware.md) — parts list, weights, power budget, and candidate build configurations.
- [WaveShare ESP32-P4-WIFI6 pin map](doc/waveshare_esp32p4_pins.md) — reserved vs free GPIOs and the recommended `board.config` pin assignment.
- [Tasks & plans](doc/plan.md) — required hardware checklist and the phased development roadmap.
- [Development & testing guide](doc/skills.md) — tooling (`ampy`/`mpremote`/`rshell`, `mpy-cross`), source layout, the `panda` test network, and the testing rules.
- [Flight simulations](doc/sims/) — closed-loop host + on-board HITL flights (noise/wind/corner sweeps) with interactive reports.
- [Benchmarks](doc/benches/) — board performance logs (BeagleBone, RPi4, StarFive).
- [Photos](doc/photos/) — build and electronics photos.
- [Videos](doc/videos/) — flight footage (e.g. the TMS-6 campaign).

**Models — [`models/`](models/)** — 3D-printable STL parts for the booster and glider prototypes (TMS-1 … TMS-7).

**Source — [`src/`](src/)**
- [`src/glider/`](src/glider/) — Main Controller flight firmware (MicroPython), with tests in `src/glider/test/`. *(planned)*
- [`src/control/`](src/control/) — Control Center ground station (Python). *(planned)*
- [`src/camera/`](src/camera/) — Recorder module (Luckfox Pico): 2304×1296 video + UART telemetry/log sink. *(implemented)*

**Tools — [`tools/`](tools/)** — various tools which helps in development and setup.

