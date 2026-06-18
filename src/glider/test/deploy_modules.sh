#!/usr/bin/env bash
# Copy the glider modules + the driver/task packages to the board so on-board tests and tools can
# import them. Done in ONE mpremote session (a single soft-reset): a deployed main.py auto-runs on
# every soft-reset, so per-file copies would each relaunch it and race the next copy. cp -r merges
# onto existing package dirs (no nesting). Usage:  deploy_modules.sh [PORT]  (default /dev/ttyACM0)

set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PORT="${1:-${PORT:-/dev/ttyACM0}}"
SRC="$HERE/.."

mods=("$SRC"/*.py)
[ -e "${mods[0]}" ] || { echo "no modules to deploy"; exit 0; }

# one chained command: copy every module to root, then each package recursively
cmd=(cp "${mods[@]}" :)
for pkg in drivers tasks; do [ -d "$SRC/$pkg" ] && cmd+=(+ cp -r "$SRC/$pkg" :); done

for _ in 1 2 3 4 5; do
    if mpremote connect "$PORT" "${cmd[@]}" >/dev/null 2>&1; then
        echo "  deployed ${#mods[@]} modules + drivers/ tasks/"
        exit 0
    fi
    sleep 1.5
done
echo "  ERROR: module deploy failed after retries"
exit 1
