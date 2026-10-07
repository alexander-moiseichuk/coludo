#!/bin/bash
# Coludo project, copyright under MIT license, Alexander Moiseichuk
#
# Pull one recorder session off a Rust (src/avionics) board's microSD over Wi-Fi, the counterpart of flight_pull.sh
# (which pulls from the Luckfox over adb): every file the ring_saver wrote for the session, and its video, through
# the board's file server (firmware/src/files.rs, port 8000), checked against the board's own listing.
#
#   tools/board_pull.sh <host>[:port] [session] [outdir]
#
#   host     the board's address or name (taster, 192.168.102.57); the port defaults to 8000
#   session  a boot id (000276), a dated prefix or a recorder.session label. Default: the newest boot-id session on
#            the card (the largest number, in either layout below); a label is never the default, so a labelled run
#            needs this argument
#   outdir   where the flight goes (default /tmp/flights/<session>), as doc/specs/avionics-port.md "SD and flash file
#            layout" reassembles it on the host:
#              <outdir>/recordings/   the session's files: <session>_<stream>.csv, recorder.log, session.csv
#              <outdir>/videos/       its .h264 files, when the card holds any
#
# The board serves /sd/<path>: a file, or a directory as one `<name>;<bytes>` line per entry. The card is laid out
# session first (crates/recorder layout.rs): the session's recordings in /sd/<session>/recordings/, its video in
# /sd/videos/<session>/ (the videos partition on a two-partition card, a plain directory on a one-partition one),
# the global index /sd/sessions.csv. A card the M4 firmware wrote keeps them in /sd/recordings/<session>/ and
# /sd/video/<session>/ (or /sd/videos/<session>/), read when the session is not in the layout's place. A session in
# both places is refused: the boot that ran when an M4 card's board was reflashed has its bulk in the old place and
# only the tail the new firmware saved in the new one, and either alone would pass for the whole session.
# `videos`, `recordings` and `video` (in any case) name the root's other directories, never a session (the layout's
# RESERVED): an earlier boot labelled so is saved under _<label> and pulled as `_<label>`; an M4 card's own session
# under such a label (recordings/recordings/) is pulled by hand.
#
# Verified, never only counted:
#   * every file arrived whole: exactly the bytes the board's X-Coludo-Bytes header says the file held when the board
#     opened it. The board sends a file chunked, and on a read error mid-file still closes the reply as if it were
#     whole, so that header is the only proof. The listing is no proof: the default session is usually the running
#     boot, whose saver appends to every stream each second, so a file cut short can still come out longer than it
#     was listed. A board whose firmware sends no such header is refused: its files cannot be checked;
#   * no file came out shorter than listed (it would have shrunk). One LONGER than listed grew while it was pulled:
#     the board is still saving this session (it is the running boot, or its saver is draining the flash); the pull
#     is good up to the moment of each file's open, and says so;
#   * every file the listings name is pulled, and no directory pulled from gained a file meanwhile (the saver started
#     a new stream, the camera a new video): each is listed again at the end, and a listing that does not come then
#     fails the pull as the first one would.
# A file the board is writing at that moment answers 503 (FatFs's lock), which curl tries again, a second apart; a
# video still recording stays busy until `video stop`.
#
# FLAGGED, not failed: a stream (.csv) of 0 bytes, a file the saver created that holds nothing, which no tool reads
# as a stream: a cut before the saver's first fsync of it, or a card that failed then.
#
# Nothing replaces an earlier pull unless this one is complete: the files land in a staging directory inside
# outdir, verified there, and only then take the place of <outdir>/recordings and <outdir>/videos.
#
# Then (as flight_pull.sh does with a Luckfox dump):
#   python3 tools/assemble_capture.py <session> <outdir>/recordings <outdir>/<session>.txt
#   python3 tools/recorder_flight.py <outdir>/recordings --session <session> -o <launch folder>
#
# Needs curl 7.84 or later (--write-out %header). Exit codes: 0 done, 1 the pull failed (unreachable, no such
# session, a file missing, short or unchecked, a listing that did not come), 2 a usage error, 3 done but a stream was
# flagged.
set -euo pipefail
usage='usage: board_pull.sh <host>[:port] [session] [outdir]'
port=8000
size_header=x-coludo-bytes # files.rs SIZE_HEADER, lower case as curl's %header{} takes it

misuse() {
    # a usage error: say what was wrong (its arguments, joined by a space), and how it is called
    echo "board_pull.sh: $*" >&2
    echo "$usage" >&2
    exit 2
}

fail() {
    # the pull failed: the reason (its arguments, joined by a space), exit 1
    echo "board_pull.sh: $*" >&2
    exit 1
}

[ $# -ge 1 ] && [ $# -le 3 ] || misuse 'name the board'
case $1 in
    -h | --help) echo "$usage"; exit 0 ;;
    -*) misuse "unknown option $1" ;;
esac
host=$1
[[ $host == *:* ]] || host="$host:$port"
command -v curl >/dev/null || fail 'curl is not installed'
curl_version=$(curl --version | head -1 | cut -d' ' -f2)
[ "$(printf '7.84.0\n%s\n' "$curl_version" | sort -V | head -1)" = 7.84.0 ] ||
    fail "curl $curl_version is too old: the size check needs 7.84 or later (--write-out %header)"
base="http://$host/sd"

get() {
    # curl as every request here makes it, its own arguments after: no proxy (the board is on the local network),
    # the board's 503 tried again a second apart, a stalled transfer given up
    curl --noproxy '*' --silent --show-error --location --connect-timeout 5 --retry 5 --retry-delay 1 \
        --speed-limit 1024 --speed-time 30 "$@"
}

listing() {
    # the board's listing of the directory $1 under /sd, one `<name>;<bytes>` line per entry; empty when there is no
    # such directory (404). Anything else fails the pull (no board, still busy after the retries, another error), so
    # a listing that did not come is never taken for an empty one
    local reply status
    reply=$(get --output - --write-out '\n%{http_code}' "$base/$1") ||
        fail "no answer from $base/$1: is the board up, and its card mounted?"
    status=${reply##*$'\n'}
    case $status in
        200) printf '%s' "${reply%$'\n'*}" ;;
        404) ;;
        *) fail "$base/$1 answered $status, not its listing" ;;
    esac
}

names() {
    # the names a listing ($1) holds, one a line, sorted bytewise for comm; a name may itself hold ';', so the size
    # is what follows the last one
    sed 's/;[^;]*$//' <<<"$1" | LC_ALL=C sort
}

session=${2:-}
if [ -z "$session" ]; then
    # the newest boot-id session: a directory at the card's root, or one in an M4 card's recordings/
    root=$(listing '')
    legacy=$(listing recordings)
    session=$({ names "$root"; names "$legacy"; } | grep -E '^[0-9]{6,}$' | sort -n | tail -1) || true
    [ -n "$session" ] || fail "no boot-id session on the card at $base/ (a labelled session must be named)"
    echo "newest session: $session"
fi
[[ $session =~ ^[A-Za-z0-9._-]+$ ]] || misuse "a session is letters, digits, '.', '_' and '-', not $session"
# the names the card's root holds for other things (crates/recorder layout RESERVED, in any case): the videos
# directory (a mount point on a two-partition card) and an M4 card's recordings/ and video/. The firmware refuses them
# as a label, and its saver files an earlier boot labelled so under _<label>, which is pulled by that name
case ${session,,} in
    videos | recordings | video)
        misuse "$session names a directory at the card's root, not a session: an earlier boot labelled so is" \
            "filed as _$session, pull that"
        ;;
esac
outdir=${3:-/tmp/flights/$session}
mkdir -p "$outdir"
staging=$(mktemp -d "$outdir/.pull.XXXXXX")
trap 'rm -rf "$staging"' EXIT

# every directory the session may have files in, with its listing now: the recordings and the video, each in the
# layout's place first, then in an M4 card's
declare -A first
for directory in "$session/recordings" "recordings/$session" "videos/$session" "video/$session"; do
    first[$directory]=$(listing "$directory")
done
recordings_directory=''
for directory in "$session/recordings" "recordings/$session"; do
    [ -z "${first[$directory]}" ] || { recordings_directory=$directory; break; }
done
[ -n "$recordings_directory" ] ||
    fail "the card holds no session $session ($base/$session/recordings, nor $base/recordings/$session)"
[ -z "${first[$session/recordings]}" ] || [ -z "${first[recordings/$session]}" ] ||
    fail "the card holds session $session in both layouts ($base/$session/recordings and" \
        "$base/recordings/$session): an M4 card whose tail the new firmware saved; either alone is not the" \
        "session, so pull both by hand"
recordings=${first[$recordings_directory]}
videos_directory=''
for directory in "videos/$session" "video/$session"; do
    [ -z "${first[$directory]}" ] || { videos_directory=$directory; break; }
done

flagged=0
grown=0
pulled=0
pull() {
    # pull every file a listing ($1, under the board path $2) names into the staging directory $3, and check each
    # against the size the board opened it at and its listed size
    local line name listed sent bytes
    mkdir -p "$3"
    while IFS= read -r line; do
        [ -n "$line" ] || continue
        name=${line%;*}
        listed=${line##*;}
        [[ $listed =~ ^[0-9]+$ ]] || fail "the listing of $2 holds a line that is not <name>;<bytes>: $line"
        sent=$(get --fail --output "$3/$name" --write-out "%header{$size_header}" "$base/$2/$name") ||
            fail "$2/$name did not come off the board"
        [[ $sent =~ ^[0-9]+$ ]] || fail "$2/$name came without $size_header, its size as the board opened it:" \
            "the board's firmware predates that header, and a file cut short cannot be told from a whole one"
        bytes=$(stat -c %s "$3/$name")
        if [ "$bytes" != "$sent" ]; then
            fail "$2/$name came off as $bytes bytes, but the board opened it at $sent: it came off short"
        elif [ "$bytes" -lt "$listed" ]; then
            fail "$2/$name is $bytes bytes, but the board listed $listed: it shrank since"
        elif [ "$bytes" -gt "$listed" ]; then
            grown=$((grown + 1))
        fi
        if [ "$bytes" = 0 ] && [[ $name == *.csv ]]; then
            echo "FLAG: $2/$name holds 0 bytes: a stream no tool can read" >&2
            flagged=$((flagged + 1))
        fi
        pulled=$((pulled + 1))
    done <<<"$1"
}

pull "$recordings" "$recordings_directory" "$staging/recordings"
files=$pulled
[ -z "$videos_directory" ] || pull "${first[$videos_directory]}" "$videos_directory" "$staging/videos"
# the listings again: a file the first ones did not name is missing from this pull, which must not pass for complete
gained=()
for directory in "${!first[@]}"; do
    again=$(listing "$directory")
    while IFS= read -r name; do
        gained+=("$directory/$name")
    done < <(LC_ALL=C comm -13 <(names "${first[$directory]}") <(names "$again"))
done
[ ${#gained[@]} = 0 ] || fail "the session gained files while it was pulled (${gained[*]}): pull it again"
rm -rf "$outdir/recordings" "$outdir/videos"
mv "$staging/recordings" "$outdir/recordings"
[ ! -d "$staging/videos" ] || mv "$staging/videos" "$outdir/videos"
echo "pulled $files recording files of session $session into $outdir/recordings"
if [ -n "$videos_directory" ]; then
    echo "pulled $((pulled - files)) video files into $outdir/videos"
else
    echo "no video for session $session"
fi
[ "$grown" = 0 ] || echo "$grown files grew while pulled: the board is still saving this session"
[ "$flagged" = 0 ] || exit 3
