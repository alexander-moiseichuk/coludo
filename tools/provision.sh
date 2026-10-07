#!/bin/bash
# Coludo project, copyright under MIT license, Alexander Moiseichuk
#
# Provision a Rust (src/avionics) board BEFORE the airframe is sealed: write its `cfg` NVS partition with the board
# config, launch.config and the Wi-Fi passwords. The Rust firmware keeps there what plan B (src/glider) kept as
# files on its VFS, and the first Rust boot carries none of them over: a sealed, Wi-Fi-only board without them has
# no network to join (the risk table of doc/specs/avionics-port.md).
#
#   tools/provision.sh [--port /dev/ttyACM0] [--launch launch.config] [--ssid NAME]... [--dry-run] <board.config>
#
# What the partition then holds, and nothing else:
#   config/board    blob: the board config file's bytes
#   config/launch   blob: the launch.config file's bytes (only with --launch)
#   creds/w<crc32>  string per --ssid: the password from <NAME>.creds as plan B's wifi driver reads it on
#                   MicroPython -- its first line up to a '\n' (a lone '\r' does not end it), stripped of ASCII
#                   whitespace only (a no-break space stays) -- keyed 'w' + the 8 lowercase hex digits of
#                   zlib.crc32(NAME as UTF-8), because an NVS key is at most 15 characters and an SSID up to 32 bytes
#
# Refused before anything is built: a board config plan B's own config.validate rejects (run on CPython through
# src/avionics/golden/shims.py) or the Rust board's validate rejects beyond it (a device's debounce_ms outside
# 0..=1000 ms), a board or launch file the Rust board's serde_json cannot read (NaN, a number past the f64 range, a
# lone surrogate escape), all of which would boot the default config, a launch file that is not a JSON object, a
# missing or empty creds file, and two SSIDs whose keys collide. The image is built by ESP-IDF's
# nvs_partition_gen.py, read back with IDF's own NVS parser and compared with what was asked before anything is
# flashed.
#
# The passwords: the CSV, the value files and the image live in a mktemp directory a trap removes on every exit
# path, because the image carries them. No password is ever printed, logged or kept, and nothing is written into
# the repo (no bytecode either).
#
# The write: the offset and size are partitions.csv's cfg row (0x620000, 0x100000 today). The board's own partition
# table is read first and must hold that same cfg partition: a board still on MicroPython has none, and its
# filesystem (plan B's files) fills the flash past its app. Then only that range is written, with the esptool
# firmware/flash.sh uses: the app, the shared nvs and the other partitions are never touched. Writing REPLACES
# everything on cfg -- a launch.config or board config saved over CC since, a password not in this image -- and the
# script says so before it writes.
#
#   --dry-run    build and verify the image and list its keys (never the values); flash nothing
#   PORT, BAUD   the serial port (/dev/ttyACM0) and the rate (921600) when not given, as in flash.sh
#   CREDS_DIR    where the <NAME>.creds files are (default src/glider: the files plan B's deploy pushes)
#   IDF_PATH, IDF_PYTHON_ENV_PATH   ESP-IDF (default ~/.espressif/esp-idf/v5.5.5) and its Python environment
#                (default ~/.espressif/python_env/idf5.5_py*_env), which carries nvs_partition_gen
#
# Exit codes: 0 done (or a dry run), 1 refused or failed (esptool's own failures too: it exits 2), 2 a usage error.
set -euo pipefail
usage='usage: provision.sh [--port PORT] [--launch launch.config] [--ssid NAME]... [--dry-run] <board.config>'
here=$(cd "$(dirname "$0")" && pwd)
root=$(dirname "$here")
port=${PORT:-/dev/ttyACM0}
baud=${BAUD:-921600}
creds=${CREDS_DIR:-$root/src/glider}
launch=''
ssids=()
dry_run=0
board=''

misuse() {
    # a usage error: say what was wrong, and how it is called
    echo "provision.sh: $1" >&2
    echo "$usage" >&2
    exit 2
}

fail() {
    # a refusal or a missing tool: the reason, exit 1
    echo "provision.sh: $1" >&2
    exit 1
}

while [ $# -gt 0 ]; do
    case $1 in
        --port | --launch | --ssid)
            [ $# -ge 2 ] || misuse "$1 needs a value"
            case $1 in
                --port) port=$2 ;;
                --launch) launch=$2 ;;
                --ssid) ssids+=("$2") ;;
            esac
            shift 2
            ;;
        --dry-run) dry_run=1; shift ;;
        -h | --help) echo "$usage"; exit 0 ;;
        -*) misuse "unknown option $1" ;;
        *) [ -z "$board" ] || misuse "one board config only ($board, $1)"; board=$1; shift ;;
    esac
done
[ -n "$board" ] || misuse 'no board config'
export PYTHONDONTWRITEBYTECODE=1  # nothing lands in the repo (src/glider, golden) or in ESP-IDF

idf_path=${IDF_PATH:-$HOME/.espressif/esp-idf/v5.5.5}
if [ -z "${IDF_PYTHON_ENV_PATH:-}" ]; then
    environments=("$HOME"/.espressif/python_env/idf5.5_py*_env)
    IDF_PYTHON_ENV_PATH=${environments[-1]}
fi
idf_python=$IDF_PYTHON_ENV_PATH/bin/python
generator=$idf_path/components/nvs_flash/nvs_partition_generator/nvs_partition_gen.py
parser=$idf_path/components/nvs_flash/nvs_partition_tool
[ -f "$generator" ] && [ -f "$parser/nvs_parser.py" ] || fail "no ESP-IDF NVS tools under $idf_path (set IDF_PATH)"
"$idf_python" -c 'import esp_idf_nvs_partition_gen' 2>/dev/null ||
    fail "$idf_python cannot run nvs_partition_gen (set IDF_PYTHON_ENV_PATH to the ESP-IDF Python environment)"

partitions=$root/src/avionics/firmware/partitions.csv
row=$(awk -F'[[:space:]]*,[[:space:]]*' '$1 == "cfg" && $2 == "data" && $3 == "nvs" { print $4, $5 }' "$partitions")
[ -n "$row" ] || fail "no cfg data/nvs partition in $partitions"
read -r offset size <<<"$row"

umask 077
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT

python3 - "$root/src/avionics/golden" "$board" "$launch" <<'EOF'
"""
Plan B's verdict on the board config, the Rust board's rule beyond it, and the launch file's shape: the files must be
what the Rust board can boot.

This step runs alone in its interpreter: shims.install() is process-wide (MicroPython's clock and modules), so
nothing else shares it. Each file is read as the Rust board reads its blob: UTF-8 text, strict JSON as serde_json
takes it. CPython also takes NaN and Infinity, a number past the f64 range (1e400 as inf, a 400-digit integer as an
int) and a lone surrogate escape ("\\ud800"); serde_json refuses all of them, and the board would then boot its
default config. Prints the board id and the config_id plan B's `set-config board` would reply; exits 1 with the
reasons on a refusal.
"""

import json
import math
import sys

golden: str = sys.argv[1]  # src/avionics/golden: shims.py
board_path: str = sys.argv[2]
launch_path: str = sys.argv[3]  # '' without --launch
sys.path.insert(0, golden)
import shims  # noqa: E402 -- the golden directory joins the path just above

shims.install()
import config  # noqa: E402 -- plan B: importable once the shims are installed


"""
The longest debounce_ms the Rust board's validate takes (crates/board_config validate.rs LONGEST_DEBOUNCE_MS), its one
rule beyond plan B's: the board refuses a stored config that breaks it and runs its default config instead.
"""
_LONGEST_DEBOUNCE_MS: int = 1000


def _debounce_errors(board: dict) -> list:
    """
    The Rust board's validate messages on the devices' debounce_ms, worded as it words them.

    Args:
        board - a board config plan B's validate passed: its sensors and components are lists of named objects.

    Returns:
        A message per device whose debounce_ms is not an int (a bool is none) from 0 to _LONGEST_DEBOUNCE_MS; [] for
        none.
    """
    errors = []
    for section in ('sensors', 'components'):
        for device in board.get(section) or []:
            if 'debounce_ms' not in device:
                continue
            debounce = device['debounce_ms']
            if isinstance(debounce, bool) or not isinstance(debounce, int) or not 0 <= debounce <= _LONGEST_DEBOUNCE_MS:
                errors.append("%s '%s'.debounce_ms must be an int from 0 to %d ms (got %r)"
                              % (section, device['name'], _LONGEST_DEBOUNCE_MS, debounce))
    return errors


def _reject_constant(name: str) -> None:
    """json.loads hook for NaN and Infinity: not JSON, and the Rust board's serde_json refuses them."""
    raise ValueError('%s is not JSON' % name)


def _finite(text: str) -> float:
    """
    json.loads hook for a number with a fraction or an exponent: CPython reads 1e400 as inf, serde_json refuses it.

    Args:
        text - the number as written.

    Returns:
        Its value.

    Raises:
        ValueError - the value is past the f64 range.
    """
    value = float(text)
    if not math.isfinite(value):
        raise ValueError('%s is not a finite number' % text)
    return value


def _integer(text: str) -> int:
    """
    json.loads hook for an integer: serde_json reads one past 64 bits as an f64, and refuses it past the f64 range.

    Args:
        text - the integer as written.

    Returns:
        Its value, exact (plan B's validate sees the int CPython would give it).

    Raises:
        ValueError - the value is past the f64 range (309 digits or more).
    """
    _finite(text)
    return int(text)


def _document(path: str) -> object:
    """
    The JSON document in the file at `path`.

    Args:
        path - the file.

    Returns:
        The parsed document.

    Raises:
        SystemExit - the file is unreadable, not UTF-8, or not JSON serde_json reads; the message names it.
    """
    try:
        with open(path, 'rb') as handle:
            text = handle.read().decode('utf-8')
        document = json.loads(text, parse_constant=_reject_constant, parse_float=_finite, parse_int=_integer)
        json.dumps(document, ensure_ascii=False).encode('utf-8')  # raises on a lone surrogate: not text, in any key
        return document
    except OSError as error:
        raise SystemExit('provision.sh: %s: %s' % (path, error.strerror)) from None
    except UnicodeDecodeError:
        raise SystemExit('provision.sh: %s: not UTF-8 text' % path) from None
    except UnicodeEncodeError:
        raise SystemExit('provision.sh: %s: not JSON (a \\u escape of a lone surrogate)' % path) from None
    except ValueError as error:
        raise SystemExit('provision.sh: %s: not JSON (%s)' % (path, error)) from None


board: object = _document(board_path)  # a dict once validate passes it
errors: list
try:
    errors = config.validate(board)
except Exception as error:  # plan B's load() reads a raising validate as an invalid file too
    errors = ['validation raised %r' % error]
if errors:
    raise SystemExit('provision.sh: %s: plan B config.validate refuses it:\n  %s' % (board_path, '\n  '.join(errors)))
errors = _debounce_errors(board)
if errors:
    raise SystemExit("provision.sh: %s: the Rust board's validate refuses it:\n  %s"
                     % (board_path, '\n  '.join(errors)))
if launch_path and not isinstance(_document(launch_path), dict):
    raise SystemExit('provision.sh: %s: a launch.config must be a JSON object' % launch_path)
print('board config %s: valid, board.id %s, config_id %s' % (board_path, board['board']['id'], config.config_id(board)))
EOF

python3 - "$work" "$offset" "$size" "$idf_python" "$generator" "$parser" "$creds" "$board" "$launch" \
    "${ssids[@]}" <<'EOF'
"""
Build the cfg image with ESP-IDF's nvs_partition_gen.py, read it back with IDF's NVS parser, and list its keys.

Every value is a file of its own in the work directory and the CSV names the files (`file` rows), so no value -- a
password least of all -- passes through a CSV cell, an argument or a message. Messages name keys, SSIDs and paths,
never a value. Exits 1 with the reason on a refusal.
"""

import csv
import json
import os
import subprocess
import sys
import zlib
from typing import NoReturn

work: str = sys.argv[1]  # the mktemp directory
offset: str = sys.argv[2]  # the cfg partition's, as partitions.csv writes it
size: str = sys.argv[3]
idf_python: str = sys.argv[4]  # ESP-IDF's Python, which carries the generator's dependencies
generator: str = sys.argv[5]  # nvs_partition_gen.py
parser: str = sys.argv[6]  # the nvs_partition_tool directory
creds: str = sys.argv[7]  # the directory of the <NAME>.creds files
board_path: str = sys.argv[8]
launch_path: str = sys.argv[9]  # '' without --launch
ssids: list = sys.argv[10:]  # one per --ssid, in order
sys.path.insert(0, parser)
import nvs_parser  # noqa: E402 -- ESP-IDF's nvs_partition_tool joins the path just above

_SSID_LIMIT: int = 32  # bytes in an 802.11 SSID, and all the firmware's Wi-Fi configuration holds
_PASSWORD_LIMIT: int = 64  # bytes the firmware reads back and its Wi-Fi configuration holds: a passphrase, a PSK
_WHITESPACE: str = ' \t\n\r\v\f'  # what MicroPython's str.strip() removes; CPython's also takes U+00A0, U+3000...
_CSV: str = 'cfg.csv'  # the generator's input, in the work directory
_IMAGE: str = 'cfg.bin'  # the generator's output, in the work directory


def _refuse(message: str) -> NoReturn:
    """
    Stop with `message`, which never carries a value.

    Args:
        message - the reason.

    Raises:
        SystemExit - always (exit 1).
    """
    raise SystemExit('provision.sh: ' + message)


def _read(path: str) -> bytes:
    """The bytes of a file the validation step has already read (it refused the unreadable ones)."""
    with open(path, 'rb') as handle:
        return handle.read()


def _password(ssid: str) -> bytes:
    """
    The password for `ssid`, as plan B's wifi driver reads it on MicroPython: the first line of <ssid>.creds,
    stripped. MicroPython's readline stops at b'\\n' only and its strip() takes ASCII whitespace only, so the file is
    read as bytes: CPython's text mode would also end the line at a lone '\\r', and its strip() would also take a
    no-break space -- the sealed board would then hold another password than plan B used.

    Args:
        ssid - the network name.

    Returns:
        The password, UTF-8 encoded.

    Raises:
        SystemExit - the file is missing, unreadable or not UTF-8, its first line is empty (plan B would fall back
        to the config's password: leave the --ssid out for that), or the password is longer than the firmware holds.
    """
    path = os.path.join(creds, ssid + '.creds')
    try:
        with open(path, 'rb') as handle:
            line = handle.readline()
        password = line.decode('utf-8').strip(_WHITESPACE)
    except OSError as error:
        _refuse('--ssid %s: no password: %s: %s' % (ssid, path, error.strerror))
    except UnicodeDecodeError:
        _refuse('%s: not UTF-8 text' % path)
    if not password:
        _refuse('%s: no password on its first line' % path)
    encoded = password.encode('utf-8')
    if len(encoded) > _PASSWORD_LIMIT:
        _refuse('%s: the password is longer than the %d bytes the firmware holds' % (path, _PASSWORD_LIMIT))
    return encoded


def _credentials() -> dict:
    """
    One creds entry per --ssid.

    Args:
        (none -- reads the --ssid list)

    Returns:
        {key: (ssid, password)}, keyed 'w' + the 8 lowercase hex digits of zlib.crc32(ssid as UTF-8).

    Raises:
        SystemExit - an SSID is empty, holds '/' (it names a file), is longer than 32 bytes or is given twice, or
        two SSIDs share a key (a CRC-32 collision: NVS could hold only one of them).
    """
    names = {}  # key -> ssid: every key is checked before any creds file is read
    for ssid in ssids:
        encoded = ssid.encode('utf-8')
        if not ssid or '/' in ssid or len(encoded) > _SSID_LIMIT:
            _refuse('--ssid %r: an SSID is 1..%d bytes, without /' % (ssid, _SSID_LIMIT))
        key = 'w%08x' % zlib.crc32(encoded)
        if key in names:
            _refuse('--ssid %s given twice' % ssid if names[key] == ssid else
                    '--ssid %s and --ssid %s share the key %s (a CRC-32 collision): NVS can hold only one'
                    % (names[key], ssid, key))
        names[key] = ssid
    return {key: (ssid, _password(ssid)) for key, ssid in names.items()}


def _tried_networks() -> set:
    """
    The SSIDs the board tries, for the notes on missing and unused passwords: plan B's drivers/wifi.py setup() rule,
    which the board_config crate's view follows too. A note the operator acts on before the seal must be right.

    `wifi.networks` when it is a non-empty list, else the top-level `wifi.ssid` alone; a bare string is {'ssid':
    name}; an entry without an ssid, with a falsy `enabled` or with policy 'disabled' is dropped. None at all when
    the section's own policy is 'disabled': the radio is never touched. (Validation has made every entry a string or
    an object with a string ssid.)

    Args:
        (none -- reads the board config)

    Returns:
        The SSIDs.
    """
    with open(board_path, encoding='utf-8') as handle:
        wifi = json.load(handle).get('wifi') or {}
    if wifi.get('policy', 'auto') == 'disabled':
        return set()
    names = set()
    for entry in wifi.get('networks') or ([wifi['ssid']] if wifi.get('ssid') else []):
        entry = {'ssid': entry} if isinstance(entry, str) else entry
        if entry.get('ssid') and entry.get('enabled', True) and entry.get('policy', 'auto') != 'disabled':
            names.add(entry['ssid'])
    return names


def _build(wanted: dict) -> bytes:
    """
    Write the value files and the CSV, and run nvs_partition_gen.py on them.

    Args:
        wanted - {(namespace, key): (encoding, value)}: 'binary' for a blob, 'string' for a string.

    Returns:
        The image, exactly `size` bytes.

    Raises:
        SystemExit - the generator failed (its output, which names keys, never values, is in the message) or wrote
        an image of another size.
    """
    rows = [('key', 'type', 'encoding', 'value')]
    for namespace in ('config', 'creds'):
        keys = sorted(key for space, key in wanted if space == namespace)
        if keys:
            rows.append((namespace, 'namespace', '', ''))
        for key in keys:
            encoding, value = wanted[namespace, key]
            with open(os.path.join(work, key), 'wb') as handle:
                handle.write(value)
            rows.append((key, 'file', encoding, os.path.join(work, key)))
    with open(os.path.join(work, _CSV), 'w', newline='') as handle:
        csv.writer(handle).writerows(rows)
    result = subprocess.run([idf_python, generator, 'generate', _CSV, _IMAGE, size], cwd=work, capture_output=True,
                            text=True)
    if result.returncode != 0:
        _refuse('nvs_partition_gen.py failed (exit %d):%s' % (result.returncode, result.stdout + result.stderr))
    image = _read(os.path.join(work, _IMAGE))
    if len(image) != int(size, 0):
        _refuse('nvs_partition_gen.py wrote %d bytes for a %s partition' % (len(image), size))
    return image


def _decode(image: bytes) -> dict:
    """
    What the image holds, read back with ESP-IDF's NVS parser, every page and entry CRC checked.

    Args:
        image - the partition image.

    Returns:
        {(namespace, key): (encoding, value)}, in _build's terms: a string without its NUL, a blob's chunks joined.

    Raises:
        SystemExit - a CRC fails, a key repeats, a string lacks its NUL, or a blob's chunks do not add up.
    """
    namespaces = {}
    written = []
    for page in nvs_parser.NVS_Partition('cfg', bytearray(image)).pages:
        if page.header['status'] == 'Empty':
            continue
        if page.header['crc']['original'] != page.header['crc']['computed']:
            _refuse('the image fails its page CRC (page %d)' % page.header['page_index'])
        for entry in page.entries:
            if entry.state != 'Written':
                continue
            checks = entry.metadata['crc']
            data_intact = not entry.children or checks['data_original'] == checks['data_computed']
            if checks['original'] != checks['computed'] or not data_intact:
                _refuse('the image fails an entry CRC (key %s)' % entry.key)
            if entry.metadata['namespace'] == 0:
                namespaces[entry.data['value']] = entry.key
            else:
                written.append(entry)
    found = {}
    chunks = {}
    indexes = {}
    for entry in written:
        name = (namespaces.get(entry.metadata['namespace']), entry.key)
        kind = entry.metadata['type']
        data = b''.join(bytes(child.raw) for child in entry.children)[:entry.data.get('size', 0)]
        if kind == 'blob_data':
            chunks[name, entry.metadata['chunk_index']] = data
            continue
        if name in found or name in indexes:
            _refuse('the image holds %s/%s twice' % name)
        if kind == 'blob_index':
            indexes[name] = entry.data
        elif kind == 'string' and data.endswith(b'\0'):
            found[name] = ('string', data[:-1])
        else:
            _refuse('the image holds %s/%s as an unexpected %s' % (name + (kind,)))
    for name, index in indexes.items():
        parts = [chunks.pop((name, index['chunk_start'] + chunk), None) for chunk in range(index['chunk_count'])]
        value = b''.join(part for part in parts if part is not None)
        if None in parts or len(value) != index['size']:
            _refuse('the blob %s/%s does not add up from its chunks' % name)
        found[name] = ('binary', value)
    if chunks:
        _refuse('the image holds blob chunks no index names')
    return found


wanted: dict = {('config', 'board'): ('binary', _read(board_path))}  # {(namespace, key): (encoding, value)}
if launch_path:
    wanted['config', 'launch'] = ('binary', _read(launch_path))
credentials: dict = _credentials()
for key, (_ssid, password) in credentials.items():
    wanted['creds', key] = ('string', password)
found: dict = _decode(_build(wanted))
if found != wanted:
    differing = sorted(name for name in set(found) | set(wanted) if found.get(name) != wanted.get(name))
    _refuse('the image does not read back as built: %s' % ', '.join('%s/%s' % name for name in differing))
print('cfg image verified: %s bytes for the cfg partition at %s; it holds' % (size, offset))
for namespace, key in sorted(found):
    encoding, value = found[namespace, key]
    what = ('blob, %d bytes' % len(value) if encoding == 'binary' else 'string: the password for %s'
            % credentials[key][0])
    print('  %-16s %s' % ('%s/%s' % (namespace, key), what))
tried: set = _tried_networks()
given: set = {ssid for ssid, _password in credentials.values()}
for ssid in sorted(tried - given):
    print('note: the board tries the network %s, and the image holds no password for it' % ssid)
for ssid in sorted(given - tried):
    print('note: the board never tries the network %s: its password is stored, never used' % ssid)
EOF

if [ "$dry_run" = 1 ]; then
    echo 'dry run: nothing flashed'
    exit 0
fi

echo "reading the partition table of the board on $port"
esptool --chip esp32p4 --port "$port" --baud "$baud" read-flash 0x8000 0xc00 "$work/table.bin" ||
    fail "could not read the partition table on $port (esptool failed, see above): nothing was written"
python3 - "$work/table.bin" "$offset" "$size" "$port" <<'EOF'
"""
Refuse unless the board's own partition table holds the cfg partition partitions.csv describes.

The write lands at a fixed offset whatever the board runs: on a board still on MicroPython it would land in the
filesystem that fills the flash past its app, and on an older Rust table in whatever is there now. Exits 1 with the
table's layout on a refusal.
"""

import struct
import sys

table_path: str = sys.argv[1]  # what read-flash gave
offset: int = int(sys.argv[2], 0)  # partitions.csv's cfg row
size: int = int(sys.argv[3], 0)
port: str = sys.argv[4]
_MAGIC: bytes = b'\xaa\x50'  # a partition table entry starts with it; the MD5 entry and the 0xFF tail do not
_ENTRY: int = 32  # bytes per entry: magic, type, subtype, offset, size, label (16), flags
_DATA_NVS: tuple = (0x01, 0x02)  # (type, subtype) of an NVS data partition

with open(table_path, 'rb') as handle:
    table: bytes = handle.read()
partitions: dict = {}  # label -> (type, subtype, offset, size)
for start in range(0, len(table) - _ENTRY + 1, _ENTRY):
    entry = table[start : start + _ENTRY]
    if entry[:2] != _MAGIC:
        break
    kind, subtype, entry_offset, entry_size = struct.unpack('<BBII', entry[2:12])
    partitions[entry[12:28].rstrip(b'\0').decode('ascii', 'replace')] = (kind, subtype, entry_offset, entry_size)
if partitions.get('cfg') != _DATA_NVS + (offset, size):
    layout = ', '.join('%s 0x%x+0x%x' % (label, value[2], value[3]) for label, value in partitions.items())
    raise SystemExit('provision.sh: the board on %s has no cfg NVS partition at 0x%x+0x%x (its table: %s) -- flash '
                     'the Rust firmware first (src/avionics/firmware/flash.sh); on a MicroPython board that range '
                     'is its filesystem' % (port, offset, size, layout or 'none'))
EOF

echo "writing cfg at $offset on $port: this REPLACES everything on the partition -- a launch.config or board config"
echo 'saved over CC since, a Wi-Fi password not in this image -- and touches nothing else'
esptool --chip esp32p4 --port "$port" --baud "$baud" write-flash "$offset" "$work/cfg.bin" ||
    fail "writing cfg on $port failed: the partition may be partly written -- run provision.sh again before sealing"
