#!/bin/bash
# tools/flight_pull.sh -- pull ONE real-flight recorder session off the Luckfox and render it in one
# command: adb-pull every stream, assemble the interleaved capture, and produce the interactive report
# (HTML) + SVG + KPIs. The FIELD counterpart to hitl_collect.sh -- that one also FLIES a HITL sim;
# here the flight already happened, so this is just the pull + assemble + report chain (plan CC item 11,
# which was hand-run per stream). Needs `adb` to the Luckfox and (for the HTML) the plotly venv.
#
# Usage: flight_pull.sh [session] [outdir]
#   session : recorder session id (default: the LATEST session on the Luckfox)
#   outdir  : output directory (default: /tmp/flights/<session>)
# Env: PAD=lat,lon  ZONE=tl_lat,tl_lon,br_lat,br_lon  (optional -- drawn on the SVG; default HPRC)
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
REC=/userdata/recordings
PAD=${PAD:-25.514379,-80.391795}
ZONE=${ZONE:-25.514944,-80.392972,25.514583,-80.391111}
PLY=${PLY:-$HOME/.local/share/pipx/venvs/plotly/bin/python}

command -v adb >/dev/null || { echo "error: adb not found (need the Luckfox recorder)"; exit 2; }

ses=${1:-}
if [ -z "$ses" ]; then                       # default: the newest session (by its health stream)
  # Any stream identifies the session, not `health` specifically: a profile with board_health
  # disabled produced NO *_health.csv, so this returned empty and reported "no recorder session"
  # on a board that had just recorded a full flight. hitl_collect.sh already keys on any *_*.csv.
  # The prefix is the boot id (`000123`, current firmware) or the older leading YYYYMMDD_HHMMSS_<tag>,
  # so strip from the LAST underscore-group on
  # (a literal-anchored sed, not `_*` which is a regex meaning "zero or more underscores").
  # MATCH the session prefix, do not strip the suffix. The prefix has a known shape,
  # while stream names do NOT have a known underscore count: stripping the
  # last group turned `..._servo_eleron_left.csv` into `..._servo_eleron`. Anchoring on the prefix is
  # correct for every stream name, however many underscores it carries. The two prefix shapes cannot
  # both match one name (a boot id is followed by a stream's lowercase letter, a date by a digit).
  # The Luckfox spins corrupted names off into junk files, and the newest file of all is often one of
  # those -- and a junk name can match either shape (`20000101_000005_1927_gl.csv`), even repeat an OLD
  # session's id. So the session is the one whose SECOND-newest file is the newest: walking the listing
  # newest first, the first prefix met twice. A session writes many streams, all of them until it ends;
  # a junk name is one file, and counting over the whole listing let one junk name lift an old session.
  # Strip the ANSI colour codes and the CR the Luckfox `ls` emits BEFORE that $-anchored match (as the
  # stream listing below does): stripped after, `.csv\r` never matched and a recorder that had just
  # captured a flight was reported as holding no session at all. LC_ALL=C: a junk name need not be
  # UTF-8, and a UTF-8 `.` matches no such byte.
  ses=$(adb shell "ls -t $REC/*_*.csv 2>/dev/null" \
        | LC_ALL=C sed -e "s/\x1b\[[0-9;]*m//g" -e "s/\r//g" \
        | LC_ALL=C sed -nE "s|.*/||; s|^([0-9]{8}_[0-9]{6}_[A-Za-z0-9]+)_.*\.csv$|\1|p; s|^([0-9]{6,})_[a-z].*\.csv$|\1|p" \
        | LC_ALL=C awk '++count[$0] == 2 {print; exit}')
  [ -z "$ses" ] && { echo "error: no recorder session found on the Luckfox"; exit 1; }
  echo "latest session: $ses"
fi
out=${2:-/tmp/flights/$ses}
mkdir -p "$out"; rm -f "$out"/*.csv

# Pull EVERY stream this session wrote -- never a hardcoded list. The old fixed STREAMS omitted
# airspeed_sdp810, flight, checkpoint and the per-servo servo_*.csv, so a pulled capture was missing
# data the board HAD recorded and nothing said so: it assembles into a file that looks like a whole
# flight and every downstream tool renders it as one. hitl_collect.sh already learned this.
# The Luckfox shell does not expand a glob here and its `ls` emits ANSI colour codes + CR, so strip
# both before matching or every name silently fails to match. recorder_wire picks the session's own
# files: under a label, an older label that held '_' can share the prefix (`hitl_f15_*` beside `hitl_*`).
# One name per line, read whole: a junk name can hold a space, or bytes that are not UTF-8.
expected=$(adb shell "ls $REC/" 2>/dev/null \
           | LC_ALL=C sed -e "s/\x1b\[[0-9;]*m//g" -e "s/\r//g" | python3 "$ROOT/tools/recorder_wire.py" files "$ses")
n=0; missing=''
while IFS= read -r name; do
  [ -n "$name" ] || continue
  if adb pull "$REC/$name" "$out/" >/dev/null 2>&1; then n=$((n + 1)); else missing="$missing $name"; fi
done <<< "$expected"
want=$(echo "$expected" | grep -c . || true)
[ "$n" -eq 0 ] && { echo "error: session $ses has no streams on the Luckfox"; exit 1; }
# VERIFY the pull, do not just count it. A PARTIAL pull is the dangerous case: it still assembles.
if [ "$n" -ne "$want" ]; then
  echo "error: pulled $n of $want streams for $ses -- capture would be INCOMPLETE" >&2
  echo "  missing:$missing" >&2
  exit 1
fi
echo "pulled $n streams (all the session wrote)"

cap="$out/$ses.txt"
python3 "$ROOT/tools/assemble_capture.py" "$ses" "$out" "$cap" >/dev/null || { echo "assemble failed"; exit 1; }
echo "assembled $cap"

# Every render runs even if an earlier one failed -- the capture is already safe -- but a failure is
# REPORTED, with its stderr kept, and the pull exits non-zero. These used to go to /dev/null behind
# `|| true`: a missing report looked like one that had simply not been asked for, and the exit said ok.
rendered=0
if python3 "$ROOT/tools/flight_svg.py" "$cap" -o "$out/$ses.svg" --title "Coludo flight $ses" \
     --pad "$PAD" --zone "$ZONE" >/dev/null 2>"$out/svg.err"; then
  echo "svg    $out/$ses.svg"
else
  echo "svg    FAILED -- see $out/svg.err" >&2; rendered=1
fi
if [ ! -x "$PLY" ]; then
  echo "report SKIPPED -- no plotly python at $PLY (set PLY)" >&2; rendered=1
elif "$PLY" "$ROOT/tools/flight_report.py" "$cap" -o "$out/$ses.html" --cdn >/dev/null 2>"$out/report.err"; then
  echo "report $out/$ses.html"
else
  echo "report FAILED -- see $out/report.err" >&2; rendered=1
fi
echo "--- KPIs ---"
python3 "$ROOT/tools/flight_kpi.py" "$ses:$cap" 2>"$out/kpi.err" \
  || { echo "KPIs   FAILED -- see $out/kpi.err" >&2; rendered=1; }
exit "$rendered"
