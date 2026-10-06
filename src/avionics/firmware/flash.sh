#!/bin/bash
# Flash the avionics firmware: the cargo runner (`cargo run --release`), or by hand with an .elf path.
#
# The same steps ESP-IDF itself takes: esptool elf2image makes the app image (stamped for chip revisions
# v1.0..v1.99, as the sdkconfig selects -- the fleet's v1.3), IDF's gen_esp32part.py the partition table, and
# esptool writes them with the bootloader esp-idf-sys built for THIS sdkconfig. espflash is not used: it
# rejects custom partition subtypes and cannot package an XIP-from-PSRAM image.
# PORT overrides the serial port (default /dev/ttyACM0), BAUD the flashing rate (default 921600).
set -euo pipefail
elf=${1:?usage: flash.sh <elf>}
here=$(cd "$(dirname "$0")" && pwd)
build=$(dirname "$elf")
# the newest: cargo keeps one build dir per esp-idf-sys configuration (build/esp-idf-sys/<hash>/out/...)
idf_build=$(find "$build/build" -path '*esp-idf-sys*/out/build/bootloader/bootloader.bin' -printf '%T@ %p\n' |
    sort -n | tail -1 | cut -d' ' -f2 | xargs -r dirname | xargs -r dirname)
[ -n "$idf_build" ] || { echo "no bootloader.bin under $build/build -- build first" >&2; exit 1; }
idf_path=$(grep -m1 '^IDF_PATH:' "$idf_build/CMakeCache.txt" 2>/dev/null | cut -d= -f2 || true)
idf_path=${idf_path:-$HOME/.espressif/esp-idf/v5.5.5}
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT
esptool --chip esp32p4 elf2image --flash-mode qio --flash-freq 80m --flash-size 16MB \
    --min-rev-full 100 --max-rev-full 199 --elf-sha256-offset 0xb0 -o "$work/app.bin" "$elf" >/dev/null
python3 "$idf_path/components/partition_table/gen_esp32part.py" -q "$here/partitions.csv" "$work/partitions.bin"
esptool --chip esp32p4 --port "${PORT:-/dev/ttyACM0}" --baud "${BAUD:-921600}" write-flash \
    0x2000 "$idf_build/bootloader/bootloader.bin" 0x8000 "$work/partitions.bin" 0x10000 "$work/app.bin"
