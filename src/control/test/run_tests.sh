#!/usr/bin/env bash
# Run the host (CPython) Control tests: python3 each test_*.py; pass = exit 0. Control is host
# code, so (unlike the glider tests) these run on the PC, not the board.
#
# Usage:  run_tests.sh [test_file ...]

set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
PY="${PYTHON:-python3}"

if [ -t 1 ]; then G=$'\e[32m'; R=$'\e[31m'; B=$'\e[1m'; N=$'\e[0m'; else G=; R=; B=; N=; fi
command -v "$PY" >/dev/null || { echo "${R}error: $PY not found${N}"; exit 2; }

if [ "$#" -gt 0 ]; then tests=("$@"); else tests=("$HERE"/test_*.py); fi
[ -e "${tests[0]}" ] || { echo "${R}no test_*.py found in $HERE${N}"; exit 0; }

pass=0; fail=0; failed=()
echo "${B}== coludo control tests ==${N}  ($("$PY" --version 2>&1))"
for t in "${tests[@]}"; do
    name="$(basename "$t")"
    printf '%-28s ' "$name"
    if out="$("$PY" "$t" 2>&1)"; then
        echo "${G}PASS${N}"
        pass=$((pass + 1))
    else
        echo "${R}FAIL${N}"
        printf '%s\n' "$out" | sed 's/^/    /'
        fail=$((fail + 1)); failed+=("$name")
    fi
done
echo '----------------------------------------'
echo "${B}total:${N} $((pass + fail))   ${G}pass: $pass${N}   ${R}fail: $fail${N}"
[ "$fail" -eq 0 ] || { printf '  %s\n' "${failed[@]}"; exit 1; }
exit 0
