#!/usr/bin/env bash
# Run the on-board MicroPython tests via tools/boardrun.py (pyserial paste mode) -- the reliable
# non-interactive counterpart to test/run_tests.sh (which drives mpremote). Modules must already be
# deployed (tools/deploy_board.sh). Each test is run with a fresh soft-reset for isolation; a test
# PASSES if its output has an 'ok:' line and no Traceback/Error/FAIL.
#
# Usage:  run_tests_board.sh [test_name ...]    # names like test_flight (default: all test/test_*.py)
# Env:    PORT (default /dev/ttyACM0)  TIMEOUT secs (default 60)
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PORT="${PORT:-/dev/ttyACM0}"
TIMEOUT="${TIMEOUT:-60}"
VPY="$HOME/.local/share/pipx/venvs/mpremote/bin/python"; [ -x "$VPY" ] || VPY=python3
LOGDIR="/tmp/coludo-tests"; mkdir -p "$LOGDIR"

if [ "$#" -gt 0 ]; then
    names=("$@")
else
    names=(); for f in "$ROOT"/src/glider/test/test_*.py; do names+=("$(basename "$f" .py)"); done
fi

pass=0; fail=0; failed=()
echo "== coludo board tests ==  port=$PORT  timeout=${TIMEOUT}s  ($(${VPY} -c 'import serial' 2>/dev/null && echo pyserial-ok))"
for name in "${names[@]}"; do
    printf "%-32s " "$name"
    log="$LOGDIR/$name.log"
    "$VPY" "$ROOT/tools/boardrun.py" "$PORT" runfile "test/$name.py" "$TIMEOUT" > "$log" 2>&1
    rc=$?
    if [ "$rc" -eq 0 ] && grep -qa 'ok:' "$log"; then
        echo "PASS"; pass=$((pass+1))
    else
        echo "FAIL (rc=$rc)  -> $log"; grep -aE 'Traceback|Error|FAIL|assert' "$log" | head -3 | sed 's/^/    /'
        fail=$((fail+1)); failed+=("$name")
    fi
done
echo "------------------------------------------------"
echo "total: $((pass+fail))   pass: $pass   fail: $fail"
[ "$fail" -gt 0 ] && { printf '  x %s\n' "${failed[@]}"; exit 1; }
exit 0
