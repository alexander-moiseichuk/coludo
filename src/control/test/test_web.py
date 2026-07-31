"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Host (CPython) test for the operator dashboard (src/control/web.py): request parsing, routing, and the
MALFORMED-INPUT paths §26 hardened.

web.py is the surface you actually stare at in the field, it was untested (findings §27.11), and §26
found four ways to hang or crash it -- a garbage request line, a non-numeric Content-Length, a handler
exception with no response, and an unguarded json.loads. A hung dashboard mid-test is the worst time to
discover any of them, so each is pinned here. Drives `_handle` over in-memory streams; no socket, no
event loop server. Run by `make test` / `make check`.
"""

import asyncio
import json
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_HERE))
import web  # noqa: E402


class _Reader:
    """asyncio.StreamReader stand-in over a fixed request buffer."""

    def __init__(self, data: bytes):
        self._lines = data.splitlines(True)
        self._rest = b''

    async def readline(self) -> bytes:
        return self._lines.pop(0) if self._lines else b''

    async def readexactly(self, count: int) -> bytes:
        body = b''.join(self._lines)
        self._lines = []
        if len(body) < count:
            raise asyncio.IncompleteReadError(body, count)
        return body[:count]


class _Writer:
    """asyncio.StreamWriter stand-in that just accumulates what was sent."""

    def __init__(self):
        self.sent = b''
        self.closed = False

    def write(self, data: bytes) -> None:
        self.sent += data

    async def drain(self) -> None:
        pass

    def close(self) -> None:
        self.closed = True

    def get_extra_info(self, _name):
        return ('127.0.0.1', 9999)


class _Hub:
    """Minimal hub: enough registry surface for the routes under test."""

    def __init__(self):
        self.boards = {}
        self.streams = {}

    def board_rows(self):
        return [{'id': 'taster', 'online': True}]


def _request(raw: bytes):
    """Drive one request through Web._handle and return the raw response bytes."""
    server = web.Web(_Hub(), log=lambda *a: None)
    writer = _Writer()
    asyncio.run(server._handle(_Reader(raw), writer))
    assert writer.closed, 'the handler must always close its writer'
    return writer.sent


def test_routes():
    """The GET routes answer, and an unknown path is a clean 404 rather than a hang."""
    assert b'200 OK' in _request(b'GET / HTTP/1.1\r\n\r\n')
    boards = _request(b'GET /api/boards HTTP/1.1\r\n\r\n')
    assert b'200 OK' in boards and b'taster' in boards
    assert json.loads(boards.split(b'\r\n\r\n', 1)[1]) == [{'id': 'taster', 'online': True}]
    assert b'404' in _request(b'GET /nope HTTP/1.1\r\n\r\n')


def test_dashboard_carries_the_imu_calibration_column():
    """
    The IMU calibration must survive the whole path board -> health -> /api/boards -> table.

    An uncalibrated BNO055 is invisible to probe(), to self-test and to the config gate, yet NDOF
    fusion only converges with MOTION -- so a still glider reaches launch with a frozen attitude. It
    only becomes actionable if the operator can SEE it, so the field is a column rather than a
    footnote, and the page ships the formatter + the guided button that watches mag climb to 3.
    """
    page = _request(b'GET / HTTP/1.1\r\n\r\n')
    # the live operator surface must never be cached: a stale page shows stale controls, and the
    # failure is SILENT -- an edited dashboard that simply never appears in the browser
    assert b'Cache-Control: no-store' in page
    assert b'<th>calibration</th>' in page, 'the dashboard lost its calibration column'
    assert b'fmtImu' in page and b'calibrateBoard' in page, 'the calibrate action is not served'
    # the sweep must be GENERIC -- a hardcoded device name means a new one is silently skipped
    assert b"'calibrate', []" in page, 'the button must sweep, not name one device'
    # one device at a time, gated on the OPERATOR confirming -- a timed pause races them, and a tare
    # captured while the airframe is still being set down is worse than no tare
    assert b'confirm(' in page, 'each device must wait for an explicit OK'
    # the button COUNTS DOWN off the heartbeat, so the row clears itself without an extra round trip
    assert b'pendingCalibration' in page and b'calibrate ${pending.length}' in page
    assert b'names[0]' in page, 'one device per press, not a loop that holds the operator'
    # PLAIN WORDS, not "M3/3": the operator reads this in a field, without a datasheet. And the report
    # stays reachable when everything is fine -- "no button" must not mean "no information".
    assert b'calibWord' in page and b'calibrationReport' in page
    assert b"'not calibrated'" in page and b"'OK'" in page
    # the report rides the TOOLTIP on the status itself, so it stays reachable with nothing to click
    assert b'title="${report}"' in page, 'the report must stay reachable when nothing is outstanding'
    # every colspan must match the header width, or the empty-state row misaligns the table
    assert b'colspan="13"' in page and b'colspan="12"' not in page

    # a NOT-READY board must be obvious on the ROW, not buried in a cell an operator has to read
    assert b'function notReady' in page and b'tr.notready' in page
    assert b'\xe2\x9d\x97' in page or b'&#10071;' in page or b'notready' in page  # the ❗ marker

    """
    The server must FORWARD the field rather than dropping it with the rest of `health`. Asserted by
    RUNNING board_rows() over a stub board, not by grepping server.py's source for a literal: the old
    string match broke on any reformat or rename while the behaviour was still correct, and -- worse
    -- would have passed on a line that had been commented out.
    """
    import server as server_module

    class _StubBoard:
        id, online, info = 'glider-01', True, {}
        cache = {'health': {'imu_calibration': {'sys': 3, 'gyr': 3, 'acc': 2, 'mag': 0},
                            'calibration': {'imu_bno055': 'make a figure 8'}}}

    hub = server_module.Server.__new__(server_module.Server)  # no sockets: board_rows is pure
    hub.boards = {'glider-01': _StubBoard()}
    row = server_module.Server.board_rows(hub)[0]
    assert row['imu_calibration'] == {'sys': 3, 'gyr': 3, 'acc': 2, 'mag': 0}, \
        'board_rows dropped imu_calibration'
    assert row['calibration'] == {'imu_bno055': 'make a figure 8'}, \
        'board_rows dropped the calibration instructions'



def test_malformed_request_line_does_not_hang():
    """
    §26.4: `method, path, _ = line.split(' ', 2)` raised ValueError on a garbage line, which no except
    clause caught -- the writer closed with NO response, so the client hung.
    """
    for raw in (b'GARBAGE\r\n\r\n',                  # no spaces at all -> ValueError on unpack
                b'GET\r\n\r\n',                      # one token
                b'\xff\xfe\x00 / HTTP/1.1\r\n\r\n',  # undecodable -> UnicodeDecodeError
                b'\r\n'):                            # empty request line
        _request(raw)  # must return (writer closed) rather than raise or block


def test_bad_content_length_still_routes():
    """§26.5: a non-numeric Content-Length raised ValueError; now it means 'no body' and routing goes on."""
    response = _request(b'GET /api/boards HTTP/1.1\r\nContent-Length: abc\r\n\r\n')
    assert b'200 OK' in response and b'taster' in response


def test_handler_fault_answers_500():
    """
    §26.10: an unexpected handler exception produced NO response at all, hanging the browser.

    Forced here by giving the hub a board_rows() that raises -- the 500 path must answer, and the status
    line must read the real reason (the _REASON table gained 500 for exactly this).
    """
    server = web.Web(_Hub(), log=lambda *a: None)
    server.hub.board_rows = lambda: (_ for _ in ()).throw(RuntimeError('boom'))
    writer = _Writer()
    asyncio.run(server._handle(_Reader(b'GET /api/boards HTTP/1.1\r\n\r\n'), writer))
    assert b'500 Internal Server Error' in writer.sent, writer.sent[:80]
    assert writer.closed


def test_post_with_bad_json_is_answered():
    """A POST carrying non-JSON must get a response (not a hang) -- json.loads is guarded (§26.8/9)."""
    body = b'{not json'
    raw = b'POST /api/cmd HTTP/1.1\r\nContent-Length: %d\r\n\r\n%s' % (len(body), body)
    response = _request(raw)
    assert response, 'a malformed POST body must still produce a response'


test_routes()
test_dashboard_carries_the_imu_calibration_column()
test_malformed_request_line_does_not_hang()
test_bad_content_length_still_routes()
test_handler_fault_answers_500()
test_post_with_bad_json_is_answered()
print('ok: web -- routing + 404, IMU calibration column + calibrate action, not-ready row flag, '
      'malformed request line, bad Content-Length, handler fault -> 500, bad JSON POST answered')
