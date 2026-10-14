#!/bin/bash
# Coludo project, copyright under MIT license, Alexander Moiseichuk
#
# Flash a MicroPython ESP32-P4 firmware image WITHOUT wiping NVS.
#
# The official images (ESP32_GENERIC_P4-*.bin, written at 0x2000) are merged: bootloader, partition table, and
# app, with the gap between them filled with 0xFF -- and that gap covers nvs (0x9000..0xEFFF). Writing the image
# whole therefore erases NVS on every reflash: the boot counter restarts at 1, so session ids repeat and a new
# boot's capture files collide with old ones on the logger, and NVS calibrations (the pitot zero, ...) are lost.
# This writes the three parts only and leaves nvs untouched.
#
#   tools/flash_micropython.sh ~/src/images/esp32p4/ESP32_GENERIC_P4-C6_WIFI-20260824-v1.29.0.bin [PORT]
#
# Then tools/deploy.sh as usual. The image must target the board's chip revision (the fleet's P4 is v1.3: an
# image whose minimal revision is v3.x will not boot -- `esptool image-info <image>` shows it).
set -euo pipefail
image=${1:?usage: flash_micropython.sh <ESP32_GENERIC_P4 image.bin> [port]}
port=${2:-/dev/ttyACM0}
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT
python3 - "$image" "$work" <<'EOF'
import struct
import sys

image, work = sys.argv[1], sys.argv[2]
data = open(image, 'rb').read()
BASE = 0x2000  # the image is written from the P4's bootloader offset
table = data[0x8000 - BASE : 0x9000 - BASE]
if table[:2] != b'\xaa\x50' or data[0x10000 - BASE] != 0xE9:
    sys.exit('%s: not a merged ESP32-P4 image (no partition table at 0x8000 or app at 0x10000)' % image)
entries = {}
for offset in range(0, len(table), 32):
    entry = table[offset : offset + 32]
    if entry[:2] != b'\xaa\x50':
        break
    start, size = struct.unpack('<II', entry[4:12])
    entries[entry[12:28].rstrip(b'\0').decode()] = (start, size)
if entries.get('nvs', (0, 0))[0] != 0x9000 or entries.get('factory', (0, 0))[0] != 0x10000:
    sys.exit('%s: unexpected partition layout %s -- refusing to guess' % (image, entries))
open(work + '/bootloader.bin', 'wb').write(data[: 0x8000 - BASE])
open(work + '/partitions.bin', 'wb').write(table)
open(work + '/app.bin', 'wb').write(data[0x10000 - BASE :])
EOF
esptool --chip esp32p4 --port "$port" --baud 921600 write-flash \
    0x2000 "$work/bootloader.bin" 0x8000 "$work/partitions.bin" 0x10000 "$work/app.bin"
