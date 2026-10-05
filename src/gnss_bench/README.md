# GNSS bench checker

[`main.py`](main.py) checks two GNSS modules in the field. They get **our flight init** plus slow diagnostics,
and the checker records what they do. It runs on an **ESP32-C6 SuperMini** with **MicroPython ≥ 1.29**. It is
not flight firmware: `tools/deploy.sh` never touches it. The first result is
[doc/benches/gnss-20261004](../../doc/benches/gnss-20261004/README.md).

## Wiring

| module | module TX → C6 | module RX ← C6 | UART | baud |
|---|---|---|---|---|
| ATGM336H | GPIO4 | GPIO5 | `UART(2)`, the C6's LP UART (fixed pins) | 9600 |
| NEO-6M | GPIO7 | GPIO6 | `UART(1)` | 9600 |

Power the C6 and both modules together. A module that powers up with the C6 gets up to 5 s to start talking
before it is configured anyway.

* **LED:** the SuperMini's RGB LED (WS2812, GPIO8). Solid **green** while it works; solid **red** once a save
  failed.
* **No button.** The SuperMini's BOOT button never registered on GPIO9 on this unit, so the checker needs
  none.

## A session per power-on

Every power-on starts session N, numbered on from the highest `336.N` on flash. Each module gets the flight
init once it talks, as `drivers/atgm336h.py` (hz 10) and `drivers/neo6mv2.py` (hz 5) send it. Then come the
diagnostics: GSV, GSA and the ATGM's antenna text every ~10 s, which leave the RMC/GGA rates as they are.
After that:

* every 10 s, a `status` line per module: RMC valid, GSA mode, GGA quality, satellites used and HDOP,
  satellites in view, the best four C/N0, the antenna text, UTC and the position;
* at a module's first 3D fix, a `fix3d` record with the last 100 NMEA lines;
* every 30 s, the moment files are rewritten with each module's last 100 lines, both at one instant, so
  pulling the battery loses at most the last 30 s.

## Files

| file | what |
|---|---|
| `336.N` / `neo6.N` | the ATGM / NEO in session N: `start`, `talking` (or `silent`), the `tx` commands sent, the first 10 NMEA lines, the `status` lines, the `fix3d` record |
| `336.N.x` / `neo6.N.x` | the last moment: `moment`, the situation, the last 100 lines |

A saved NMEA line is `<uptime ms>;<line verbatim>`. [`main.py`](main.py)'s header has the full record format.
The fixed status lines, the `fix3d` records and the moment files hold **the position at metre precision**.

## Install, read out, report

Connect the C6 **directly to the PC, never through a USB hub**: the hub corrupts mpremote transfers.

```
mpremote connect /dev/ttyACM0 cp src/gnss_bench/main.py :main.py          # install
mpremote connect /dev/ttyACM0 ls                                           # which sessions are there
mpremote connect /dev/ttyACM0 cp :336.5 :336.5.x :neo6.5 :neo6.5.x <dir>/  # read out (every N)
```

The read-out power-on starts a session too: the newest one, with almost no status lines. mpremote's Ctrl-C
ends it. To start numbering from 1 again, remove the old session files.

```
python3 tools/gnss_bench_report.py <dir> --label 1=none 5=small ... [-o summary.csv]
```

The report gives one row per session and module: the run, the 3D-fix time, when the RMC turned valid
(between two readings: the status lines and the `fix3d` record's situation), valid status lines, satellites
in view and used, the best C/N0 and the best top-4 mean, the antenna text, and the CEP50 and farthest fix
about the median. It never prints the position. Then it compares the two modules on the same satellites,
from one read-out of their moment files.

The session files hold the bench site's position: keep them local, out of git. To pin them in a write-up
without publishing them, print a keyed digest per file:

```
python3 tools/gnss_bench_report.py --provenance <dir>
```

Each line is `<file>;<HMAC-SHA256>`, keyed with `<dir>/.pepper`: 32 random bytes made on first use,
owner-only, never printed, and git-ignored by name. A plain hash would let anyone confirm a guessed position.
