#!/bin/bash
# Flash the avionics firmware: the cargo runner (`cargo run --release`), or by hand with the .elf path.
#
# Uses the bootloader esp-idf-sys built for THIS sdkconfig (rev < 3 silicon) -- espflash's bundled one
# targets the v3.x line and would not boot the fleet's v1.3 -- and partitions.csv, whose nvs sits where
# src/glider keeps it. PORT overrides the serial port (default /dev/ttyACM0), BAUD the flashing rate.
set -euo pipefail
elf=${1:?usage: flash.sh <elf>}
here=$(cd "$(dirname "$0")" && pwd)
build=$(dirname "$elf")
# the newest one: cargo keeps a build dir per esp-idf-sys configuration (build/esp-idf-sys/<hash>/out/...)
bootloader=$(find "$build/build" -path '*esp-idf-sys*/out/build/bootloader/bootloader.bin' -printf '%T@ %p\n' |
    sort -n | tail -1 | cut -d' ' -f2)
[ -n "$bootloader" ] || { echo "no bootloader.bin under $build/build -- build first" >&2; exit 1; }
exec espflash flash --chip esp32p4 --port "${PORT:-/dev/ttyACM0}" --baud "${BAUD:-921600}" --bootloader "$bootloader" \
    --partition-table "$here/partitions.csv" "$elf"
