#!/usr/bin/env bash
# Copy the glider Python modules (src/glider/*.py) to the attached board so on-board tests and
# tools can `import` them. Usage:  deploy_modules.sh [PORT]   (PORT default /dev/ttyACM0)

set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PORT="${1:-${PORT:-/dev/ttyACM0}}"

mods=("$HERE"/../*.py)
[ -e "${mods[0]}" ] || { echo "no modules to deploy"; exit 0; }

mpremote connect "$PORT" reset >/dev/null 2>&1 || true
sleep 2
for m in "${mods[@]}"; do
    n="$(basename "$m")"
    if mpremote connect "$PORT" cp "$m" ":$n" >/dev/null 2>&1; then
        echo "  deployed $n"
    else
        echo "  WARN: failed to deploy $n"
    fi
done
