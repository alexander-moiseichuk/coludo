"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Host (CPython) test for tools/board_pull.sh, which pulls a recorder session off a Rust board's microSD over HTTP and
checks it against the board's own listing. No board is contacted: a local http.server on 127.0.0.1 stands in for
firmware/src/files.rs, serving a fixture tree laid out like the card under /sd (doc/specs/avionics-port.md "SD and
flash file layout": <session>/recordings/..., the global sessions.csv, videos/<session>/...; and beside it the M4
firmware's layout of an older card, recordings/<session>/..., recordings/session.csv, video/<session>/...), a
directory as one `<name>;<bytes>` line per entry, as the board answers.

The stand-in sends each file's size as of its open in X-Coludo-Bytes, as files.rs does, and can misbehave as the board
can: answer 503 with Retry-After for a file a saver batch holds (FatFs's lock), send less than that size in a reply
that still ends cleanly (the board's chunked reply after a read error), send a file that grew since its listing (a
live session), send no X-Coludo-Bytes (a board before that header), list a file only from the second listing on (a
stream or a video that started during the pull), and fail a listing the second time.

Positive: the newest boot-id session by default across both layouts (never a label, never a shorter id), every file
byte for byte, the video under videos/<session>/, an M4 card's session from recordings/<session>/ with its video under
video/<session>/ (or videos/<session>/), a named label session without video, a busy file asked again, a grown file
noted, and an http_proxy in the environment ignored. Negative: a short file fails the pull and leaves the earlier pull
of that session as it was, also one that grew past its listing before it was cut, a reply without X-Coludo-Bytes fails
it, a 0-byte stream is flagged (exit 3) with the files still pulled, a stream or a video that appeared during the pull
fails it, as does a second listing that did not come, and an unknown session, a session in both layouts, a card with
no boot-id session and none named, a refused connection, a listing that is not <name>;<bytes>, a name the card's
root holds for something else (`videos`, `recordings`, `video`, in any case) named as a session and usage errors fail
before anything is replaced; an earlier boot so labelled comes off as `_<label>`, where the saver files it. The
refusal of a session in both layouts carries its whole reason. No staging directory outlives a run.
"""

import http.server
import os
import shutil
import subprocess
import tempfile
import threading
import time

HERE: str = os.path.dirname(os.path.abspath(__file__))
ROOT: str = os.path.normpath(os.path.join(HERE, '..', '..', '..'))
SCRIPT: str = os.path.join(ROOT, 'tools', 'board_pull.sh')

_SCRATCH = tempfile.TemporaryDirectory(prefix='coludo-board-pull-')
CARD: str = os.path.join(_SCRATCH.name, 'card')  # the fixture tree, as /sdcard holds it

"""A session's files as the ring_saver writes them: wrapped lines, recorder.log and the session's session.csv."""
SESSION_FILES: dict = {
    '000123_health.csv': b'{00000000};uptime;temp;mem_free;load;oom_s;land_s;leak_kbps;rescues;rescue_ms;<00000000>\n'
                         b'{00000000};1000000;41.5;9000000;12;None;None;0;0;0;<00000000>\n',
    '000123_imu_lsm6dso32.csv': b'{00000000};uptime;ax;ay;az;<00000000>\n' * 400,
    'recorder.log': b'{00000000};1000 main :: boot: board taster;<00000000>\n',
    'session.csv': b'{00000000};uptime;boot;session;utc;utc_offset;board;firmware;config_id;source;cc_lat;cc_lon;'
                   b'<00000000>\n',
}


class _Board(http.server.SimpleHTTPRequestHandler):
    """The board's file server for the fixture card: GET /sd/<path>, a file or a `<name>;<bytes>` listing."""

    busy: dict = {}  # card path -> how many more GETs answer 503
    short: dict = {}  # card path -> the bytes a GET sends of it (its X-Coludo-Bytes still says the whole file)
    grown: dict = {}  # card path -> bytes it grew by between its listing and its open
    unsized: set = set()  # card paths answered without X-Coludo-Bytes, as a board before that header
    appears: dict = {}  # card directory -> a file listed only from its second listing on
    garbage: dict = {}  # card directory -> a line its listing carries that is not <name>;<bytes>
    failing: dict = {}  # card directory -> the status every listing after its first answers
    listed: dict = {}  # card directory -> listings served

    def log_message(self, *arguments: object) -> None:
        """Quiet: the test reads the script's output, not the server's."""

    def do_GET(self) -> None:
        """Serve /sd/<path> as files.rs does."""
        relative = self.path.split('?', 1)[0]
        if not relative.startswith('/sd/') or '..' in relative.split('/'):
            self.send_error(404)
            return
        relative = relative[len('/sd/'):].strip('/')
        path = os.path.join(CARD, relative)
        if os.path.isdir(path):
            count = _Board.listed[relative] = _Board.listed.get(relative, 0) + 1
            if relative in _Board.failing and count > 1:
                self.send_error(_Board.failing[relative])
                return
            names = sorted(os.listdir(path))
            lines = ['%s;%d' % (name, os.path.getsize(os.path.join(path, name))) for name in names]
            if relative in _Board.appears and count > 1:
                lines.append('%s;10' % _Board.appears[relative])
            if relative in _Board.garbage:
                lines.append(_Board.garbage[relative])
            self._answer(200, ''.join(line + '\n' for line in lines).encode())
        elif os.path.isfile(path):
            if _Board.busy.get(relative, 0) > 0:
                _Board.busy[relative] -= 1
                self.send_response(503)
                self.send_header('Retry-After', '1')
                self.end_headers()
                self.wfile.write(b'busy: the file is being written\n')
                return
            with open(path, 'rb') as handle:
                body = handle.read() + b'x' * _Board.grown.get(relative, 0)
            size = None if relative in _Board.unsized else len(body)  # the size as of the open
            self._answer(200, body[:_Board.short.get(relative, len(body))], size)
        else:
            self.send_error(404)

    def _answer(self, status: int, body: bytes, size: int | None = None) -> None:
        """
        A reply with no Content-Length, ended by the close, as the board's chunked reply ends after any error.

        Args:
            status - the HTTP status.
            body - what is sent.
            size - the X-Coludo-Bytes header, the file's size as the board opened it; None sends none.
        """
        self.send_response(status)
        self.send_header('Content-Type', 'application/octet-stream')
        if size is not None:
            self.send_header('X-Coludo-Bytes', str(size))
        self.end_headers()
        self.wfile.write(body)


def _session(directory: str, session: str) -> None:
    """A session's files as the ring_saver writes them, in `directory` under the card."""
    os.makedirs(os.path.join(CARD, directory))
    for name, content in SESSION_FILES.items():
        with open(os.path.join(CARD, directory, name.replace('000123', session)), 'wb') as handle:
            handle.write(content)


def _video(directory: str) -> None:
    """One .h264 file in `directory` under the card."""
    os.makedirs(os.path.join(CARD, directory))
    with open(os.path.join(CARD, directory, '5321.h264'), 'wb') as handle:
        handle.write(b'\x00\x00\x00\x01video' * 5000)


def _card() -> None:
    """
    The fixture card: two boot-id sessions, a label and a shorter id in the layout's place, the global index and one
    video under videos/; beside them an M4-era session under recordings/, with its index and its video under video/.
    """
    shutil.rmtree(CARD, ignore_errors=True)
    for session in ('000122', '000123', 'flightlabel', '12345'):
        _session(os.path.join(session, 'recordings'), session)
    with open(os.path.join(CARD, 'sessions.csv'), 'wb') as handle:
        handle.write(SESSION_FILES['session.csv'])
    _video(os.path.join('videos', '000123'))
    _session(os.path.join('recordings', '000099'), '000099')
    with open(os.path.join(CARD, 'recordings', 'session.csv'), 'wb') as handle:
        handle.write(SESSION_FILES['session.csv'])
    _video(os.path.join('video', '000099'))
    _Board.busy, _Board.short, _Board.grown, _Board.appears, _Board.garbage, _Board.listed = {}, {}, {}, {}, {}, {}
    _Board.unsized, _Board.failing = set(), {}


def _serve() -> str:
    """Start the stand-in on a free local port; returns its `host:port`."""
    server = http.server.ThreadingHTTPServer(('127.0.0.1', 0), _Board)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return '127.0.0.1:%d' % server.server_address[1]


def _run(*arguments: str) -> subprocess.CompletedProcess:
    """
    Run board_pull.sh with a proxy set in its environment that does not exist: the pull must never use one.

    Args:
        arguments - the script's arguments.

    Returns:
        The finished run.
    """
    environment = dict(os.environ, http_proxy='http://127.0.0.1:9', HTTP_PROXY='http://127.0.0.1:9',
                       all_proxy='http://127.0.0.1:9')
    return subprocess.run([SCRIPT, *arguments], capture_output=True, text=True, env=environment, timeout=120)


def _tree(directory: str) -> dict:
    """Every file under `directory` as {relative path: bytes}."""
    files = {}
    for here, _, names in os.walk(directory):
        for name in names:
            path = os.path.join(here, name)
            with open(path, 'rb') as handle:
                files[os.path.relpath(path, directory)] = handle.read()
    return files


def _card_files(*parts: str) -> dict:
    """The fixture card's files under `parts`, as {name: bytes}."""
    return _tree(os.path.join(CARD, *parts))


def _no_staging(outdir: str) -> None:
    """No staging directory of a run is left in `outdir`."""
    leftovers = [name for name in os.listdir(outdir) if name.startswith('.pull.')] if os.path.isdir(outdir) else []
    assert leftovers == [], leftovers


def test_newest_session_whole(host: str) -> None:
    """With no session named, the newest boot id comes off whole, its video under videos/."""
    _card()
    outdir = os.path.join(_SCRATCH.name, 'newest')
    result = _run(host, '', outdir)  # an empty session is the default, as when left out
    assert result.returncode == 0, (result.returncode, result.stdout, result.stderr)
    assert 'newest session: 000123' in result.stdout, result.stdout  # not the label, nor the shorter 12345
    assert _tree(os.path.join(outdir, 'recordings')) == _card_files('000123', 'recordings')
    assert _tree(os.path.join(outdir, 'videos')) == _card_files('videos', '000123')
    assert 'pulled 4 recording files of session 000123' in result.stdout, result.stdout
    assert 'pulled 1 video files' in result.stdout and 'grew' not in result.stdout, result.stdout
    _no_staging(outdir)


def test_an_m4_card(host: str) -> None:
    """
    A session the M4 firmware wrote comes off its recordings/<session>/ with its video under video/ (or videos/),
    and the newest boot id is taken across both layouts; a session in both is refused.
    """
    _card()
    outdir = os.path.join(_SCRATCH.name, 'm4')
    result = _run(host, '000099', outdir)
    assert result.returncode == 0, (result.returncode, result.stdout, result.stderr)
    assert _tree(os.path.join(outdir, 'recordings')) == _card_files('recordings', '000099')
    assert _tree(os.path.join(outdir, 'videos')) == _card_files('video', '000099')
    os.rename(os.path.join(CARD, 'video', '000099'), os.path.join(CARD, 'videos', '000099'))
    result = _run(host, '000099', outdir)
    assert result.returncode == 0, (result.returncode, result.stdout, result.stderr)
    assert _tree(os.path.join(outdir, 'videos')) == _card_files('videos', '000099')
    # an M4 card that holds the newest boot: the default finds it there
    _session(os.path.join('recordings', '000124'), '000124')
    result = _run(host, '', outdir)
    assert result.returncode == 0, (result.returncode, result.stdout, result.stderr)
    assert 'newest session: 000124' in result.stdout, result.stdout
    assert _tree(os.path.join(outdir, 'recordings')) == _card_files('recordings', '000124')
    assert 'no video for session 000124' in result.stdout, result.stdout
    # negative: a session in both layouts (an M4 card whose running boot's tail the new firmware saved) is refused,
    # since either copy alone would pass for the whole session, and the earlier pull stays as it was
    before = _tree(outdir)
    _session(os.path.join('recordings', '000122'), '000122')
    result = _run(host, '000122', outdir)
    assert result.returncode == 1, (result.returncode, result.stdout, result.stderr)
    # the whole reason, every part of it: the two places, and what to do
    assert ('the card holds session 000122 in both layouts (http://{0}/sd/000122/recordings and '
            'http://{0}/sd/recordings/000122): an M4 card whose tail the new firmware saved; either alone is not the '
            'session, so pull both by hand\n'.format(host)) in result.stderr, result.stderr
    assert _tree(outdir) == before, 'a refused pull touched the earlier one'
    _no_staging(outdir)


def test_label_session_without_video(host: str) -> None:
    """A named label session comes off whole; it has no video, and an earlier pull's videos/ goes with it."""
    _card()
    outdir = os.path.join(_SCRATCH.name, 'label')
    os.makedirs(os.path.join(outdir, 'videos'))
    with open(os.path.join(outdir, 'videos', 'stale.h264'), 'wb') as handle:
        handle.write(b'stale')
    result = _run(host, 'flightlabel', outdir)
    assert result.returncode == 0, (result.returncode, result.stdout, result.stderr)
    assert _tree(os.path.join(outdir, 'recordings')) == _card_files('flightlabel', 'recordings')
    assert 'no video for session flightlabel' in result.stdout, result.stdout
    assert not os.path.exists(os.path.join(outdir, 'videos')), 'a stale video stayed beside the new pull'


def test_an_escaped_earlier_label(host: str) -> None:
    """
    An earlier boot labelled `videos` (the M4 firmware allowed it), which the saver files under _videos/recordings/,
    comes off by that name, its files still named by the label.
    """
    _card()
    _session(os.path.join('_videos', 'recordings'), 'videos')
    outdir = os.path.join(_SCRATCH.name, 'escaped')
    result = _run(host, '_videos', outdir)
    assert result.returncode == 0, (result.returncode, result.stdout, result.stderr)
    pulled = _tree(os.path.join(outdir, 'recordings'))
    assert pulled == _card_files('_videos', 'recordings') and 'videos_health.csv' in pulled, sorted(pulled)
    assert 'no video for session _videos' in result.stdout, result.stdout


def test_busy_and_grown_files(host: str) -> None:
    """A file the saver holds (503) is asked again; a file longer than listed is kept and noted."""
    _card()
    _Board.busy['000123/recordings/recorder.log'] = 2
    _Board.grown['000123/recordings/000123_health.csv'] = 7
    outdir = os.path.join(_SCRATCH.name, 'busy')
    started = time.monotonic()
    result = _run(host, '000123', outdir)
    assert result.returncode == 0, (result.returncode, result.stdout, result.stderr)
    assert time.monotonic() - started >= 2, 'two 503s with Retry-After 1 must cost two seconds'
    assert _Board.busy['000123/recordings/recorder.log'] == 0
    pulled = _tree(os.path.join(outdir, 'recordings'))
    assert pulled['recorder.log'] == SESSION_FILES['recorder.log']
    assert pulled['000123_health.csv'] == SESSION_FILES['000123_health.csv'] + b'x' * 7
    assert '1 files grew while pulled' in result.stdout, result.stdout


def test_short_file_keeps_the_earlier_pull(host: str) -> None:
    """A file shorter than listed fails the pull, and the earlier pull of the session stays as it was."""
    _card()
    outdir = os.path.join(_SCRATCH.name, 'short')
    assert _run(host, '000123', outdir).returncode == 0
    before = _tree(outdir)
    _Board.short['000123/recordings/000123_imu_lsm6dso32.csv'] = 100
    result = _run(host, '000123', outdir)
    assert result.returncode == 1, (result.returncode, result.stdout, result.stderr)
    assert ('000123/recordings/000123_imu_lsm6dso32.csv came off as 100 bytes, but the board opened it at %d: it came '
            'off short' % len(SESSION_FILES['000123_imu_lsm6dso32.csv'])) in result.stderr, result.stderr
    assert _tree(outdir) == before, 'a failed pull touched the earlier one'
    _no_staging(outdir)


def test_cut_short_after_it_grew(host: str) -> None:
    """
    A file that grew after the listing and then came off short fails the pull, though it is longer than listed.

    The running boot's saver appends every second, so by its open the board's health.csv holds 2.5 times what the
    listing said, and a read error cuts the reply at twice the listing: only X-Coludo-Bytes tells. A board that sends
    no X-Coludo-Bytes cannot be checked, and is refused.
    """
    _card()
    listed = len(SESSION_FILES['000123_health.csv'])
    _Board.grown['000123/recordings/000123_health.csv'] = listed * 3 // 2
    _Board.short['000123/recordings/000123_health.csv'] = listed * 2
    outdir = os.path.join(_SCRATCH.name, 'cut')
    result = _run(host, '', outdir)  # the default session, the one still being saved
    assert result.returncode == 1, (result.returncode, result.stdout, result.stderr)
    assert ('000123/recordings/000123_health.csv came off as %d bytes, but the board opened it at %d: it came off '
            'short' % (listed * 2, listed * 5 // 2)) in result.stderr, result.stderr
    assert not os.path.exists(os.path.join(outdir, 'recordings')), 'a short pull took the place of none'
    _no_staging(outdir)
    _card()
    _Board.unsized.add('000123/recordings/recorder.log')
    result = _run(host, '000123', outdir)
    assert result.returncode == 1, (result.returncode, result.stdout, result.stderr)
    assert '000123/recordings/recorder.log came without x-coludo-bytes' in result.stderr, result.stderr
    assert not os.path.exists(os.path.join(outdir, 'recordings')), 'an unchecked pull took the place of none'
    _no_staging(outdir)


def test_late_files_and_lost_listings(host: str) -> None:
    """
    A video that started during the pull fails it, as a stream does; so does a listing that does not come the
    second time (a 503 the retries did not outlast, or another error), which must never pass for an empty one.
    """
    _card()
    outdir = os.path.join(_SCRATCH.name, 'late')
    _Board.appears['videos/000123'] = '7000.h264'
    result = _run(host, '000123', outdir)
    assert result.returncode == 1, (result.returncode, result.stdout, result.stderr)
    assert 'the session gained files while it was pulled (videos/000123/7000.h264)' in result.stderr, result.stderr
    assert not os.path.exists(os.path.join(outdir, 'recordings')), 'an incomplete pull took the place of none'
    for session, directory, status in (('000123', '000123/recordings', 500), ('000123', 'videos/000123', 503),
                                       ('000099', 'video/000099', 500)):  # the last an M4 card's video
        _card()
        _Board.failing[directory] = status
        started = time.monotonic()
        result = _run(host, session, outdir)
        assert result.returncode == 1, (directory, result.returncode, result.stdout, result.stderr)
        assert '/sd/%s answered %d, not its listing' % (directory, status) in result.stderr, result.stderr
        assert status != 503 or time.monotonic() - started >= 5, 'a 503 is tried again before the pull gives up'
        assert not os.path.exists(os.path.join(outdir, 'recordings')), 'an unchecked pull took the place of none'
        _no_staging(outdir)


def test_a_name_holding_a_semicolon(host: str) -> None:
    """A file name may hold ';' (FAT allows it): its size is what follows the listing line's last ';'."""
    _card()
    with open(os.path.join(CARD, '000123', 'recordings', 'odd;name.txt'), 'wb') as handle:
        handle.write(b'twelve bytes')
    outdir = os.path.join(_SCRATCH.name, 'semicolon')
    result = _run(host, '000123', outdir)
    assert result.returncode == 0, (result.returncode, result.stdout, result.stderr)
    assert _tree(os.path.join(outdir, 'recordings'))['odd;name.txt'] == b'twelve bytes'


def test_empty_stream_flagged(host: str) -> None:
    """A 0-byte stream is flagged (exit 3), and everything is still pulled; an empty recorder.log is not a stream."""
    _card()
    open(os.path.join(CARD, '000123', 'recordings', '000123_sequencer.csv'), 'wb').close()
    outdir = os.path.join(_SCRATCH.name, 'flagged')
    result = _run(host, '000123', outdir)
    assert result.returncode == 3, (result.returncode, result.stdout, result.stderr)
    assert 'FLAG: 000123/recordings/000123_sequencer.csv holds 0 bytes' in result.stderr, result.stderr
    assert result.stderr.count('FLAG:') == 1, result.stderr
    assert _tree(os.path.join(outdir, 'recordings')) == _card_files('000123', 'recordings')


def test_failures(host: str) -> None:
    """A file that appeared during the pull, an unknown session, no board, a bad listing and usage errors."""
    _card()
    outdir = os.path.join(_SCRATCH.name, 'failures')
    _Board.appears['000123/recordings'] = '000123_late.csv'
    result = _run(host, '000123', outdir)
    assert result.returncode == 1 and 'the session gained files while it was pulled' in result.stderr, result.stderr
    assert not os.path.exists(os.path.join(outdir, 'recordings')), 'an incomplete pull took the place of none'
    _no_staging(outdir)
    _card()
    result = _run(host, '000777', outdir)
    assert result.returncode == 1 and 'the card holds no session 000777' in result.stderr, result.stderr
    _Board.garbage['000123/recordings'] = 'no size here'
    result = _run(host, '000123', outdir)
    assert result.returncode == 1, (result.returncode, result.stdout, result.stderr)
    assert 'the listing of 000123/recordings holds a line that is not <name>;<bytes>: no size here' in result.stderr
    assert not os.path.exists(os.path.join(outdir, 'recordings')), 'an incomplete pull took the place of none'
    probe = http.server.ThreadingHTTPServer(('127.0.0.1', 0), _Board)
    closed = '127.0.0.1:%d' % probe.server_address[1]
    probe.server_close()  # nothing listens there now: the connection is refused
    result = _run(closed, '000123', outdir)
    assert result.returncode == 1, (result.returncode, result.stdout, result.stderr)
    assert 'no answer from http://%s/sd/000123/recordings' % closed in result.stderr, result.stderr
    result = _run(closed, '', outdir)
    assert result.returncode == 1 and 'no answer from http://%s/sd/' % closed in result.stderr, result.stderr
    for arguments, reason in (((), 'name the board'), (('a', 'b', 'c', 'd'), 'name the board'),
                              ((host, '../000123', outdir), "a session is letters, digits, '.', '_' and '-'"),
                              ((host, 'videos', outdir), "videos names a directory at the card's root, not a "
                               'session: an earlier boot labelled so is filed as _videos, pull that'),
                              ((host, 'Videos', outdir), "Videos names a directory at the card's root"),
                              ((host, 'recordings', outdir), "recordings names a directory at the card's root"),
                              ((host, 'VIDEO', outdir), 'is filed as _VIDEO, pull that'),
                              (('--bogus',), 'unknown option --bogus')):
        result = _run(*arguments)
        assert result.returncode == 2 and reason in result.stderr, (arguments, result.returncode, result.stderr)
    # a card with labelled sessions only: no default, in either layout
    for session in ('000122', '000123', '12345', os.path.join('recordings', '000099')):
        shutil.rmtree(os.path.join(CARD, session))
    result = _run(host, '', outdir)
    assert result.returncode == 1, (result.returncode, result.stdout, result.stderr)
    assert 'no boot-id session on the card' in result.stderr, result.stderr
    _no_staging(outdir)


host = _serve()
test_newest_session_whole(host)
test_an_m4_card(host)
test_label_session_without_video(host)
test_an_escaped_earlier_label(host)
test_busy_and_grown_files(host)
test_short_file_keeps_the_earlier_pull(host)
test_cut_short_after_it_grew(host)
test_late_files_and_lost_listings(host)
test_a_name_holding_a_semicolon(host)
test_empty_stream_flagged(host)
test_failures(host)
print('ok: board_pull.sh -- against a local stand-in for the board: the newest boot-id session whole with its video, '
      'an M4 card\'s layout still read, an escaped earlier label, busy files asked again, short files (also grown '
      'ones), unsized replies, late files and lost listings refused without touching the earlier pull, 0-byte '
      'streams flagged, and bad input and reserved names refused')
