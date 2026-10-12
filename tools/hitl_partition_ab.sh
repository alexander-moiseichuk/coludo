#!/bin/bash
# Coludo project, copyright under MIT license, Alexander Moiseichuk
#
# The card-layout A/B under a HITL flight (doc/specs/avionics-port.md "SD and flash file layout"): <runs> HITL
# flights per board, the boards taking turns, each with video through the flight and a card cleared first
# (tools/hitl_host.py --video --clean: every earlier session goes, for good). After each run the session's
# recorder.log comes off the board, then tools/partition_ab.py writes runs.csv and prints the table by layout.
#
#   tools/hitl_partition_ab.sh <runs> <outdir> <board>:<ip>:<serial>...
#   tools/hitl_partition_ab.sh 3 /tmp/ab taster:192.168.102.152:/dev/ttyACM0 TMS-7F:192.168.102.162:/dev/ttyACM1
#
# HITL_FLAGS adds hitl_host.py flags to every run (`--no-power` on a board whose servo rail is off: the fins are
# commanded and recorded as ever, arm's probe passes open-loop; give it every board, so their streams match).
#
# The hub must run (the boards' CC goes through it). A refused arm is tried once more (an attitude filter not yet fed
# by the sim was seen once); a run that then ends without DONE or TIMEOUT stops the A/B.
set -euo pipefail
runs=${1:?usage: hitl_partition_ab.sh <runs> <outdir> <board>:<ip>:<serial>...}
outdir=${2:?usage: hitl_partition_ab.sh <runs> <outdir> <board>:<ip>:<serial>...}
shift 2
[ $# -gt 0 ] || { echo 'name at least one <board>:<ip>:<serial>' >&2; exit 1; }
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
mkdir -p "$outdir"
for run in $(seq 1 "$runs"); do
  for spec in "$@"; do
    IFS=: read -r board ip serial <<<"$spec"
    d="$outdir/${board}_$run"
    # resuming into the same outdir: a run already flown and pulled is kept
    if [ -s "$d/recorder.log" ] && grep -qE '^(DONE|TIMEOUT)' "$d/host.log"; then
      echo "KEPT $board run $run"
      continue
    fi
    mkdir -p "$d"
    for attempt in 1 2; do
      # shellcheck disable=SC2086 # HITL_FLAGS is a list of flags
      python3 -u "$ROOT/tools/hitl_host.py" --ip "$ip" --hub "$board" --serial "$serial" --motor F15 --glider-g 285 \
        --video --clean ${HITL_FLAGS:-} >"$d/host.log" 2>&1 || true
      grep -q '^ARM REFUSED' "$d/host.log" || break
      echo "  $board run $run: arm refused on attempt $attempt ($(grep -m1 '^ARM REFUSED' "$d/host.log" | cut -c1-120))"
    done
    if ! grep -qE '^(DONE|TIMEOUT)' "$d/host.log"; then
      # a board that did not come back from the HITL reboot (its C6 link once failed at boot): reset it by its serial
      # port, then fly the run again; hitl_host restores the configs the failed run left from its backup
      echo "  $board run $run: no DONE or TIMEOUT ($(tail -1 "$d/host.log" | cut -c1-120)): reset, once more"
      python3 -c "import serial, time; s = serial.Serial(); s.port = '$serial'; s.dtr, s.rts = False, True; s.open()
time.sleep(0.2); s.dtr = True; s.close()"
      sleep 45
      # shellcheck disable=SC2086 # HITL_FLAGS is a list of flags
      python3 -u "$ROOT/tools/hitl_host.py" --ip "$ip" --hub "$board" --serial "$serial" --motor F15 --glider-g 285 \
        --video --clean ${HITL_FLAGS:-} >"$d/host.log" 2>&1 || true
    fi
    grep -qE '^(DONE|TIMEOUT)' "$d/host.log" || { echo "FAIL $board run $run: no DONE or TIMEOUT ($d/host.log)"; exit 1; }
    session=$(grep -oE '^SESSION [0-9A-Za-z_-]+' "$d/host.log" | awk '{print $2}' | head -1)
    curl -sf --noproxy '*' -m 120 -o "$d/recorder.log" "http://$ip:8000/sd/$session/recordings/recorder.log" ||
      { echo "FAIL $board run $run: recorder.log of $session did not come off the board"; exit 1; }
    echo "OK $board run $run session=$session $(grep -oE '^(DONE|TIMEOUT [0-9]+)' "$d/host.log" | head -1)"
  done
done
python3 "$ROOT/tools/partition_ab.py" "$outdir"/*_*/ -o "$outdir"
