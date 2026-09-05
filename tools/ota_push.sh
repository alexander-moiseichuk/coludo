#!/usr/bin/env bash
#
# Push one or more modules to a board over WiFi, through the CC hub -- no USB, no wipe.
#
# The counterpart to deploy.sh for the case deploy.sh cannot serve: an airframe that is packed, or
# on the rail, where opening it to reach the USB port is the expensive part. deploy.sh remains the
# way to install a whole firmware; this is for the last-moment fix to a file or two.
#
# WHAT IT DOES FOR YOU: compiles .py to .mpy with the same toolchain and flags as deploy.sh (a board
# running mismatched bytecode is a debugging cycle nobody enjoys), and derives the DEVICE path from
# the source's place in the tree -- src/glider/drivers/bno055.py becomes drivers/bno055.mpy. Passing
# a bare basename is the one mistake this exists to prevent: the file lands in the root, the real
# module in drivers/ is untouched, and the push reports success.
#
# WHAT IT WILL NOT DO: reboot unless asked (-r). The running firmware holds the OLD module until it
# restarts -- MicroPython caches imports, and live instances and running tasks keep their bound
# methods -- so an install without a reboot leaves the board running a mixture with nothing to show
# which half is which. It also cannot rescue a board whose new module breaks the boot; the checksum
# stops a CORRUPT file being installed, but an intact-and-wrong one imports and fails, and that is a
# USB recovery. Push what has been through `make test`.
#
# Usage: tools/ota_push.sh [--host HOST] [--port PORT] [--reboot] [--dry-run]
#                          <board> [file[:device-path] ...]
#   --host, -H  CC hub host (default 127.0.0.1; the panda AP is 192.168.102.1)
#   --port, -p  operator port (default 1235)
#   --reboot    reboot the board once every file in THIS invocation is installed
#   --dry-run, -n   show what would be pushed where, touch nothing
#   --help, -h  this text
#
# REBOOT IS A SEPARATE STEP on purpose. A fix is often several files, and they may be pushed across
# several invocations -- so rebooting per invocation would restart the board into a half-updated
# tree, repeatedly. Push everything, then reboot once. With no files, --reboot just reboots.
#
# Examples:
#   tools/ota_push.sh TMS-7C src/glider/drivers/bno055.py        # -> drivers/bno055.mpy
#   tools/ota_push.sh TMS-7C src/glider/pid.py src/glider/mixer.py
#   tools/ota_push.sh --reboot TMS-7C                            # ...then reboot once, separately
#   tools/ota_push.sh --reboot TMS-7C src/glider/governor.py     # or push and reboot together
#   tools/ota_push.sh TMS-7C launches/20261003/TMS-7C/tms7c.config:board.config

set -u

TOOLS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$TOOLS/.." && pwd)"
GLIDER="$ROOT/src/glider"
HOST=127.0.0.1
PORT=1235
REBOOT=0
DRYRUN=0

if [ -t 1 ]; then G=$'\e[32m'; R=$'\e[31m'; Y=$'\e[33m'; N=$'\e[0m'; else G=; R=; Y=; N=; fi
die()  { echo "${R}$*${N}" >&2; exit 1; }
warn() { echo "${Y}$*${N}" >&2; }

USAGE="usage: ota_push.sh [--host HOST] [--port PORT] [--reboot] [--dry-run] <board> [file[:device-path] ...]"

# Hand-rolled rather than getopts, which cannot do long options -- and --reboot wants to be spelled
# out, being the one flag here that restarts a flight computer.
while [ "$#" -gt 0 ]; do
    case "$1" in
        --host|-H) HOST="${2:-}"; [ -n "$HOST" ] || die "$1 needs a value"; shift 2 ;;
        --port|-p) PORT="${2:-}"; [ -n "$PORT" ] || die "$1 needs a value"; shift 2 ;;
        --reboot)  REBOOT=1; shift ;;
        --dry-run|-n) DRYRUN=1; shift ;;
        --help|-h) echo "$USAGE"; exit 0 ;;
        --) shift; break ;;
        -*) die "unknown option $1
$USAGE" ;;
        *) break ;;
    esac
done
[ "$#" -ge 1 ] || die "$USAGE"
BOARD="$1"; shift
# No files is legitimate ONLY with --reboot: that is the "push a few times, then reboot once" step.
[ "$#" -ge 1 ] || [ "$REBOOT" = 1 ] || die "nothing to push (and no --reboot)
$USAGE"

command -v nc >/dev/null || die "nc not found (needed to reach the CC operator port)"

# Same cross-compiler and flags as deploy.sh: bytecode built by a different mpy-cross can load and
# then misbehave, which is far harder to diagnose than a refusal.
MPYX="$TOOLS/mpy-cross.v1.29.0"
if [ ! -x "$MPYX" ]; then
    command -v mpy-cross >/dev/null || die "no mpy-cross ($MPYX missing and none on PATH)"
    MPYX=mpy-cross
    warn "warning: using mpy-cross from PATH, not the repo's pinned $TOOLS/mpy-cross.v1.29.0"
fi

# ---------------------------------------------------------------- the link

ask() {
    # One operator exchange, read to the end of the reply LINE.
    #
    # nc cannot do this correctly: it has no idea when a reply is complete, so it either waits for
    # the peer to close (the hub keeps operator sessions open, so that hangs until the timeout) or
    # guesses with -q/-W and truncates. A `list` reply spans several TCP segments, and a `push` of a
    # large module is many board round trips inside one command -- both break a guess. The protocol
    # is newline-delimited, so reading until '\n' is the only stop condition that is actually right.
    OTA_HOST="$HOST" OTA_PORT="$PORT" OTA_LINE="$1" python3 -c '
import os, socket, sys
sock = socket.create_connection((os.environ["OTA_HOST"], int(os.environ["OTA_PORT"])), timeout=20)
sock.settimeout(300)                      # a big push is many board round trips inside one command
sock.sendall((os.environ["OTA_LINE"] + "\n").encode())
buf = b""
try:
    while b"\n" not in buf:
        chunk = sock.recv(4096)
        if not chunk:
            break                          # peer closed before a full line: report what arrived
        buf += chunk
except (socket.timeout, OSError):
    pass
sock.close()
sys.stdout.write(buf.decode("utf-8", "replace").split("\n")[0])
' 2>/dev/null
}

reply="$(ask 'list')"
[ -n "$reply" ] || die "no answer from the CC hub at $HOST:$PORT -- is it running?"
case "$reply" in
    *"\"$BOARD\""*) ;;
    *) die "board $BOARD is not on the hub. Seen: $(echo "$reply" | head -c 200)" ;;
esac
case "$reply" in
    *"\"$BOARD\", \"online\": false"*) warn "warning: $BOARD is listed but marked offline" ;;
esac

# ---------------------------------------------------------------- per file

# Derive the DEVICE path from where a source lives in the tree, so a module in a package keeps its
# package. Anything outside src/glider keeps its basename, which is right for a config or creds file.
device_path() {
    local src="$1" abs rel
    abs="$(cd "$(dirname "$src")" && pwd)/$(basename "$src")"
    case "$abs" in
        "$GLIDER"/*) rel="${abs#"$GLIDER"/}"; echo "${rel%.py}.mpy" ;;
        *)           echo "$(basename "$src")" ;;
    esac
}

status=0
for spec in "$@"; do
    src="${spec%%:*}"
    dest="${spec#*:}"
    [ "$dest" = "$spec" ] && dest=""            # no ':' in the spec -> derive it
    [ -f "$src" ] || { warn "skip $src: not a file"; status=1; continue; }
    [ -n "$dest" ] || dest="$(device_path "$src")"

    payload="$src"
    if [ "${src%.py}" != "$src" ]; then         # a .py source is compiled, never pushed raw
        payload="$(mktemp "${TMPDIR:-/tmp}/ota_XXXXXX.mpy")"
        if ! "$MPYX" -march=rv32imc -O3 "$src" -o "$payload"; then
            rm -f "$payload"; warn "skip $src: mpy-cross failed"; status=1; continue
        fi
    fi

    printf '%-46s -> %-28s ' "$src" "$dest"
    if [ "$DRYRUN" = 1 ]; then
        echo "${Y}(dry run)${N}"
        [ "$payload" = "$src" ] || rm -f "$payload"
        continue
    fi

    out="$(ask "push $BOARD $payload $dest")"
    [ "$payload" = "$src" ] || rm -f "$payload"
    case "$out" in
        *"from cc ok"*) echo "${G}ok${N}  $(echo "$out" | sed 's/.*from cc ok //' | head -c 120)" ;;
        "")             echo "${R}FAILED${N} (no reply -- link or hub timeout)"; status=1 ;;
        *)              echo "${R}FAILED${N} $(echo "$out" | head -c 200)"; status=1 ;;
    esac
done

# ---------------------------------------------------------------- finish

if [ "$status" != 0 ]; then
    warn "one or more pushes failed -- NOT rebooting, so the board keeps running what it has"
    exit "$status"
fi
if [ "$REBOOT" = 1 ] && [ "$DRYRUN" != 1 ]; then
    echo "rebooting $BOARD to load the new modules..."
    ask "$BOARD reboot" >/dev/null
    echo "${G}done${N} -- give it ~20 s to come back on the hub"
elif [ "$DRYRUN" != 1 ] && [ "$#" -gt 0 ]; then
    echo "${Y}REBOOT REQUIRED${N}: the board still runs the old modules."
    echo "  push anything else first, then:  $0 --reboot $BOARD"
fi
