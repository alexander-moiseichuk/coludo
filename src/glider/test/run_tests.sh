#!/usr/bin/env bash
# Iterate over the MicroPython tests in this directory: compile each with mpy-cross -O3,
# run it on the connected board with mpremote, and print a pass/fail report.
#
# Test convention:
#   * A test file is named  test_*.py  (so e.g. bench_asyncio.py is not picked up).
#   * It PASSES if it compiles and runs to completion without raising (mpremote exit 0).
#   * It FAILS on a compile error, an uncaught exception / failed `assert` (non-zero exit),
#     a timeout, or if its output contains "FAIL" or "Traceback".
#   Tests should use `assert` for checks and may print "ok ...".
#
# Usage:  run_tests.sh [test_file ...]      # default: all test_*.py here
# Env:    PORT (default /dev/ttyACM0)  TIMEOUT secs (default 60)
#         RESET_WAIT secs (default 2)  NORESET=1 to skip the per-test board reset

set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PORT="${PORT:-/dev/ttyACM0}"
TIMEOUT="${TIMEOUT:-60}"
RESET_WAIT="${RESET_WAIT:-2}"
LOGDIR="/tmp/coludo-tests"
mkdir -p "$LOGDIR"

if [ -t 1 ]; then G=$'\e[32m'; R=$'\e[31m'; Y=$'\e[33m'; B=$'\e[1m'; N=$'\e[0m'; else G=; R=; Y=; B=; N=; fi

command -v mpremote >/dev/null || { echo "${R}error: mpremote not found${N}"; exit 2; }
# mpy-cross gate: prefer the repo-built RV32 cross so @micropython.native/viper tests (test_native_gate)
# compile-check with the right -march -- without it mpy-cross errors 'invalid arch' on the native emitter.
# -march is harmless on plain bytecode. Mirrors ../deploy.sh.
MPYX="$HERE/../../../tools/mpy-cross.v1.29.0"; MARCH=(-march=rv32imc); have_mpycross=1
if [ ! -x "$MPYX" ]; then
    if command -v mpy-cross >/dev/null; then MPYX=mpy-cross
    else have_mpycross=0; echo "${Y}warning: mpy-cross not found - skipping compile step${N}"; fi
fi

if [ "$#" -gt 0 ]; then tests=("$@"); else tests=("$HERE"/test_*.py); fi
if [ ! -e "${tests[0]}" ]; then echo "${Y}no tests found in $HERE (test_*.py)${N}"; exit 0; fi

# deploy the glider modules so on-board tests can import them (config, ...)
echo "deploying modules to $PORT..."
bash "$HERE/deploy_modules.sh" "$PORT" || echo "${Y}warning: module deploy had issues${N}"

tmpmpy="$(mktemp "$LOGDIR/compile_XXXX.mpy")"
trap 'rm -f "$tmpmpy"' EXIT

pass=0; fail=0; failed=()
echo "${B}== coludo glider tests ==${N}  port=$PORT  timeout=${TIMEOUT}s"
for t in "${tests[@]}"; do
    name="$(basename "$t")"
    printf "%-34s " "$name"

    if [ "$have_mpycross" = 1 ]; then
        if ! err="$("$MPYX" -O3 "${MARCH[@]}" "$t" -o "$tmpmpy" 2>&1)"; then
            echo "${R}FAIL${N} (compile)"; printf '%s\n' "$err" | sed 's/^/    /'
            fail=$((fail+1)); failed+=("$name (compile)"); continue
        fi
    fi

    if [ "${NORESET:-0}" != 1 ]; then
        timeout 15 mpremote connect "$PORT" reset >/dev/null 2>&1 || true
        sleep "$RESET_WAIT"
    fi

    log="$LOGDIR/$name.log"
    out="$(timeout "$TIMEOUT" mpremote connect "$PORT" run "$t" 2>&1)"; rc=$?
    printf '%s\n' "$out" > "$log"

    if [ "$rc" -eq 124 ]; then
        echo "${R}FAIL${N} (timeout ${TIMEOUT}s)  -> $log"
        fail=$((fail+1)); failed+=("$name (timeout)")
    elif [ "$rc" -ne 0 ] || printf '%s' "$out" | grep -qE 'FAIL|Traceback'; then
        echo "${R}FAIL${N} (rc=$rc)  -> $log"; printf '%s\n' "$out" | tail -6 | sed 's/^/    /'
        fail=$((fail+1)); failed+=("$name")
    else
        echo "${G}PASS${N}"; pass=$((pass+1))
    fi
done

echo "------------------------------------------------"
echo "${B}total:${N} $((pass+fail))   ${G}pass: $pass${N}   ${R}fail: $fail${N}"
if [ "$fail" -gt 0 ]; then printf '  %sx %s\n' "${R}" "${failed[@]}${N}"; exit 1; fi
exit 0
