#!/usr/bin/env bash
# install.sh — clean install onto a board: WIPE the filesystem (keeping only boot.py), then push the
# runtime firmware (top-level modules + the drivers/ and tasks/ packages), gated by ruff + mpy-cross.
# Secrets and per-device config (*.creds, *.config, board.json) are deliberately NOT pushed -- and the
# wipe removes any old ones -- so you deploy those individually afterwards. For the dev iterate/test
# loop use deploy.sh instead (it also pushes test/ and *.creds). Lives in src/glider.
#
# Usage:  ./install.sh          Env:  PORT (default /dev/ttyACM0)

set -u
HERE="$(cd "$(dirname "$0")" && pwd)"   # = src/glider
PORT="${PORT:-/dev/ttyACM0}"
if [ -t 1 ]; then G=$'\e[32m'; R=$'\e[31m'; Y=$'\e[33m'; N=$'\e[0m'; else G=; R=; Y=; N=; fi
command -v mpremote >/dev/null || { echo "${R}mpremote not found${N}"; exit 2; }
have_ruff=1; command -v ruff >/dev/null      || { have_ruff=0; echo "${Y}warning: ruff not found${N}"; }
have_mpy=1;  command -v mpy-cross >/dev/null || { have_mpy=0;  echo "${Y}warning: mpy-cross not found${N}"; }
tmp="$(mktemp -d)"; trap 'rm -rf "$tmp"' EXIT

# runtime files: top-level modules + the packages (NO test/, NO *.creds / *.config)
# stamp the firmware version as YYYY.MM.DD.commit (commit date, so the same commit -> the same
# version) for config_default to report; version.py is gitignored
printf "VERSION = '%s'\n" "$(git -C "$HERE" show -s --date=format:'%Y.%m.%d' --format='%cd.%h' --abbrev=12 HEAD 2>/dev/null || echo dev)" > "$HERE/version.py"

mods=("$HERE"/*.py)
[ -e "${mods[0]}" ] || { echo "${R}no modules to install${N}"; exit 1; }
files=("${mods[@]}")
for f in "$HERE"/drivers/*.py "$HERE"/tasks/*.py; do [ -e "$f" ] && files+=("$f"); done

# 1) gate every file before touching the board
for f in "${files[@]}"; do
    if [ "$have_ruff" = 1 ] && ! ruff check "$f"; then echo "${R}ruff failed: $f${N}"; exit 1; fi
    if [ "$have_mpy" = 1 ] && ! mpy-cross -O3 "$f" -o "$tmp/c.mpy" 2>"$tmp/err"; then
        echo "${R}mpy-cross failed: $f${N}"; sed 's/^/    /' "$tmp/err"; exit 1
    fi
done

# 2) wipe the board filesystem, keeping boot.py. Retried: a deployed main.py auto-runs on the
#    soft-reset and can briefly block raw-REPL entry; a retry lands once it yields, and after the
#    first successful wipe main.py is gone so it cannot interfere again.
WIPE='
import os
def rm(p):
    for e in os.ilistdir(p):
        c = (p + "/" + e[0]) if p != "/" else "/" + e[0]
        if e[1] & 0x4000:
            rm(c); os.rmdir(c)
        else:
            os.remove(c)
for e in os.ilistdir("/"):
    if e[0] == "boot.py":
        continue
    p = "/" + e[0]
    if e[1] & 0x4000:
        rm(p); os.rmdir(p)
    else:
        os.remove(p)
print("wiped")
'
echo "wiping board (keeping boot.py)..."
wiped=0
for _ in 1 2 3 4 5; do
    if mpremote connect "$PORT" exec "$WIPE" 2>/dev/null | grep -q wiped; then wiped=1; break; fi
    sleep 1.5
done
[ "$wiped" = 1 ] || { echo "${R}wipe failed (board busy?)${N}"; exit 1; }

# 3) push the runtime in one batched session (packages via cp -r; main.py auto-runs on each reset)
cmd=(cp "${mods[@]}" :)
for pkg in drivers tasks; do [ -d "$HERE/$pkg" ] && cmd+=(+ cp -r "$HERE/$pkg" :); done
for _ in 1 2 3 4 5; do
    if mpremote connect "$PORT" "${cmd[@]}" >/dev/null 2>&1; then
        echo "  ${G}installed${N} ${#mods[@]} modules + drivers/ tasks/  —  now deploy *.creds / *.config / board.json separately"
        exit 0
    fi
    sleep 1.5
done
echo "${R}install push failed${N}"; exit 1
