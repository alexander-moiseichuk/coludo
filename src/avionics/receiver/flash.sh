#!/bin/bash
# Coludo project, copyright under MIT license, Alexander Moiseichuk
#
# Flash the receiver onto the ESP32-C6 SuperMini: the bootloader esp-idf-sys built for THIS sdkconfig (4 MB flash),
# partitions.csv's table, and the app in the factory slot. esptool, not espflash: espflash's transfers failed on this
# board. The board must sit on a direct USB port of the host (through a hub every long transfer corrupts).
# PORT names the serial port: find the C6 by its USB serial in /sys/class/tty/ttyACM*/device/../serial, never by
# probing every port with esptool (its connect resets any ESP32 it reaches).
set -euo pipefail
elf=${1:?usage: PORT=/dev/ttyACMn flash.sh <elf>}
port=${PORT:?PORT=/dev/ttyACMn: the C6, by its USB serial}
here=$(cd "$(dirname "$0")" && pwd)
build=$(dirname "$elf")
idf_build=$(find "$build/build" -path '*esp-idf-sys*/out/build/bootloader/bootloader.bin' -printf '%T@ %p\n' |
    sort -n | tail -1 | cut -d' ' -f2 | xargs -r dirname | xargs -r dirname)
[ -n "$idf_build" ] || { echo "no bootloader.bin under $build/build -- build first" >&2; exit 1; }
idf_path=$(grep -m1 '^IDF_PATH:' "$idf_build/CMakeCache.txt" 2>/dev/null | cut -d= -f2 || true)
idf_path=${idf_path:-$HOME/.espressif/esp-idf/v5.5.5}
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT
esptool --chip esp32c6 elf2image --flash-size 4MB -o "$work/app.bin" "$elf" >/dev/null
python3 "$idf_path/components/partition_table/gen_esp32part.py" -q "$here/partitions.csv" "$work/partitions.bin"
factory=$(awk -F, '$1 ~ /^factory/ {gsub(/ /, "", $4); print $4}' "$here/partitions.csv")
esptool --chip esp32c6 --port "$port" write-flash --flash-size 4MB \
    0x0 "$idf_build/bootloader/bootloader.bin" 0x8000 "$work/partitions.bin" "$factory" "$work/app.bin"
