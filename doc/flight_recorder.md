# Flight recorder (M19/M20): composition and pinout

The flight recorder is a small separate box that keeps a copy of everything the glider records and is the glider's
second recovery beacon. The data must survive when the main board and its SD card are both damaged. It has its own
battery and card, is mounted apart from the main board, and has only two wires to it: TX and GND. A short or a
brown-out on the main board cannot take it down. It replaced the Luckfox recorder in October 2026, and the boxes
can be swapped.

Firmware: `src/avionics/recorder` (Rust, ESP-IDF 5.5.5, package `flight_recorder`). Design and status:
`doc/specs/avionics-port.md` M19 (the main board's uart_saver) and M20 (this box and its beacon).

## Composition

| part | what it is | notes |
|---|---|---|
| MCU | ESP32-C6 SuperMini (a clone) | Wi-Fi for the beacon, USB-Serial-JTAG console, LiPo charging pads on the back |
| card | a 3.3 V-only micro-SD SPI module (no regulator; 10 kOhm pull-ups and two capacitors on board) | on the C6's **3V3**, soldered pin to pin on its right header (see Pinout); since 2026-10-13 |
| battery | 1-cell LiPo, 120 mAh planned (300 mAh for longer searches) | on the C6's B+/B- pads: its charger fills it from USB |
| switch | a power switch in the battery lead | cuts the battery off when the box is stored |
| supply in flight | a USB cable from the main board's supply | runs the box and charges the cell while the main board has power; the cell carries on after |

## Pinout

| from | to | signal |
|---|---|---|
| main board (P4) GPIO20 | C6 GPIO17 (its RX pin) | UART1, the uart_saver's lines, **921600 baud 8N1**, one way |
| main board GND | C6 GND | the common ground, the only other wire |
| C6 3V3 | card 3V3 | the module has no regulator |
| C6 GPIO20 | card CS | SPI2 |
| C6 GPIO19 | card MOSI | SPI2 |
| C6 GPIO18 | card SCK | SPI2 |
| C6 GPIO15 | card MISO | SPI2 |
| C6 GPIO14 | (nothing) | the header pin facing the module's GND |
| C6 GND | card GND | its own wire: GPIO14 is no ground |

The first five are the C6's right header in the module's own pin order, so the module solders on pin to pin. GPIO15
is a strapping pin only once EFUSE_JTAG_SEL_ENABLE is burned, and the module's pull-up then selects USB-JTAG, the
default. The C6's GPIO20 is the card's CS. It isn't the main board's GPIO20, which is that board's UART TX: two
different chips that happen to share a pin number.

## Power

- **The card runs from the C6's 3V3** (2026-10-13): the module has no regulator, so the LiPo needs no boost. It
  mounts at 20 MHz SPI with no extra capacitor. The first module (WWZMDiB) carried its own 3.3 V regulator and wanted
  5 V: on 3.3 V the card answered CMD0 and CMD8 but sat in ACMD41's idle state until the 3 s timeout.
- **The module must always be powered.** With its 3V3 lead off, the card is fed through its protection diodes from
  the bus lines: the C6 then failed USB enumeration (-71), and once up the card answered its init but its SCR read
  failed CRC with the same bytes on every try.
- **Battery time (estimate, ESP32-C6 datasheet v1.5).** The beacon's access point keeps the receiver on: about
  80-85 mA. The operator's 60 s on / 60 s off cycle averages ~41 mA plus the off minute's draw, so a 120 mAh cell
  lasts ~2.1-2.4 h, against ~1.2 h always on. Measuring it is open: a LiPo run timed to the last beacon.

## Firmware and tools

- Flash only with its own script, from a direct USB port. Through the hub, long transfers corrupt. The image is
  ~1.5 MB; IDF's default 1 MB slot boot-loops.
  `PORT=/dev/ttyACMn src/avionics/recorder/flash.sh src/avionics/target/riscv32imac-esp-espidf/release/flight_recorder`
- Find its port by the sysfs USB serial (10:BD:A3:AC:E1:F4 on the bench one). Never esptool-probe every port: its
  connect resets any ESP32 it reaches. Open the console with DTR and RTS both held (pyserial `dtr=True, rts=True`);
  releasing them one at a time resets the board.
- The console: `tools/recorder_console.py ls|df|rm|get|pull` lists, measures, removes and pulls the card's files.
  Each pull is checked by size and CRC-32.
- A status line every 5 s on the console: bytes and lines in, good and damaged, card errors, the beacon's state.

## What it does

- **Files:** each line where the main board's ring_saver files it, byte for byte: `<session>/recordings/...`,
  `sessions.csv`, `recorder.log`, `damaged.log` for a line that fails its check. A session closed by a reboot or a
  panic matched the main board's card in every file (20/20, 21/21). A line sent again after a main-board reboot is
  dropped (every file's lines rise in uptime).
- **Beacon:** the main board reports in `beacon.csv` every 5 s. From the rod trigger (or any stage past SETTING) to
  power-off the box is `<BOARD>_beacon` on Wi-Fi channel 6, 60 s on and 60 s off, 20 dBm. Its page is the glider's
  own: the last fix, a map link, every elevation source, the battery. The last report is kept on the card and
  replayed at boot, so a box that resets after an impact beacons again 0.6 s later. It never beacons in a HITL
  flight.

## Photos

To add (the operator): the assembled box from both sides, the SD module's wiring, the battery, boost and switch.
