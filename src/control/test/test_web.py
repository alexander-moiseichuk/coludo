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

_ZONE: str = 'Asia/Kolkata'  # the browser's zone for 'sync time': half-hour, no DST, never UTC by accident
_ZONE_OFFSET: int = 330  # its offset EAST of UTC in minutes, the sign utc_offset carries


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


def test_hud_is_served_and_offline_safe():
    """
    The walk-test HUD (findings §27.19) is served at /hud and must stay FIELD-USABLE.

    Two things are asserted rather than assumed. It renders the live flight vitals the walk test is
    actually watching -- attitude, fins, airspeed, the authority cap -- so a rename on the board side
    that empties the page fails here instead of at the launch site. And it references NOTHING external:
    the field has no internet, so a CDN font or library would turn the HUD into a blank screen exactly
    where it is needed. The dashboard follows the same rule.
    """
    page = _request(b'GET /hud HTTP/1.1\r\n\r\n')
    assert b'200 OK' in page, 'the HUD route is not served'
    body = page.split(b'\r\n\r\n', 1)[1]
    for field in (b'airspeed', b'fin_cap', b'attitude', b'fins', b'heading_error', b'degraded'):
        assert field in body, 'the HUD stopped reading %r from the vitals' % field
    assert b'http://' not in body and b'https://' not in body, (
        'the HUD must stay self-contained -- the field has no internet')
    assert b'/events' in body and b'/api/boards' in body, 'the HUD needs the live feed + its fallback'


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
    # the count COUNTS DOWN off the heartbeat, so the cell clears itself without an extra round trip
    assert b'pendingCalibration' in page and b'(${pending.length})' in page
    """
    The calibration cell is STATUS; the ACTION is a bound button in the board actions bar.

    This pinned the in-row `calibrate ${pending.length}` button, and that button outlived the refactor
    that moved every action out of the table -- bindBoardRows stopped binding per-row controls, so the
    button still rendered, still looked live, and was wired to NOTHING. The test passed throughout,
    because it only ever checked that the markup was emitted. So assert the binding, not the markup,
    and forbid the shape that failed: no button may be rendered into a row at all.
    """
    assert b'id="actcalib"' in page, 'the calibrate action must live in the board actions bar'
    assert b"['actcalib', calibrateBoard]" in page, 'the calibrate button must be BOUND to a handler'
    assert b'class="calib"' not in page, 'no per-row calibrate button: the table is status + selection'
    row_template = page.split(b'data-board="${escAttr(b.id)}"')[1].split(b'</tr>')[0]
    assert b'<button' not in row_template, 'no button belongs in a table row -- nothing binds them'
    assert b'names[0]' in page, 'one device per press, not a loop that holds the operator'
    # PLAIN WORDS, not "M3/3": the operator reads this in a field, without a datasheet. And the report
    # stays reachable when everything is fine -- "no button" must not mean "no information".
    assert b'calibWord' in page and b'calibrationReport' in page
    assert b"'not calibrated'" in page and b"'OK'" in page
    # the report rides the TOOLTIP on the status itself, so it stays reachable with nothing to click
    assert b'title="${report}"' in page, 'the report must stay reachable when nothing is outstanding'
    """
    ONE board selection for the whole page.

    Every section used to carry its own "board id" box, all written by selectBoard(). Seven copies of
    one fact is seven chances to disagree, and nothing showed which box a button was about to read --
    so a command could go to a board the operator was no longer looking at. The table is the only
    place a board is chosen now, and each section heading names its target.
    """
    assert b'placeholder="board id"' not in page, 'per-section board boxes must be gone'
    assert page.count(b'function boardId(') == 1, 'exactly one reader for the selection'
    assert b"row.classList.toggle('chosen'" in page, 'the chosen row must be marked in the table'
    assert b'markChosen();' in page, 'and re-marked after each heartbeat re-render'

    """
    Every colspan must match the header width, or the empty-state row misaligns the table.

    DERIVED from the header rather than hardcoded: this assertion previously pinned colspan="13", so
    adding a column made a correct page fail a test that was only ever meant to catch a mismatch. A
    test that has to be edited whenever the thing it guards legitimately changes teaches people to
    edit it without reading it.
    """
    import re as _re
    head = _re.search(rb'<thead>.*?</thead>', page, _re.S)
    assert head, 'no table header found'
    columns = len(_re.findall(rb'<th[ >]', head.group(0)))
    spans = {int(n) for n in _re.findall(rb'colspan="(\d+)"', page)}
    assert spans == {columns}, 'colspan %s does not match the %d-column header' % (sorted(spans), columns)

    # a NOT-READY board must be obvious on the ROW, not buried in a cell an operator has to read
    assert b'function notReady' in page and b'tr.notready' in page
    assert b'\xe2\x9d\x97' in page or b'&#10071;' in page or b'notready' in page  # the ❗ marker

    """
    The server must FORWARD the field rather than dropping it with the rest of `health`. Asserted by
    RUNNING board_rows() over a stub board, not by grepping server.py's source for a literal: the old
    string match broke on any reformat or rename while the behaviour was still correct, and -- worse
    -- would have passed on a line that had been commented out.
    """
    import time as _time

    import server as server_module

    class _StubBoard:
        id, online, info = 'glider-01', True, {}
        last_seen = _time.monotonic()
        cache = {'health': {'imu_calibration': {'sys': 3, 'gyr': 3, 'acc': 2, 'mag': 0},
                            'calibration': {'imu_bno055': 'make a figure 8'}}}

    hub = server_module.Server.__new__(server_module.Server)  # no sockets: board_rows is pure
    hub.boards = {'glider-01': _StubBoard()}
    hub.heartbeat_s = server_module.HEARTBEAT_S
    row = server_module.Server.board_rows(hub)[0]
    assert row['imu_calibration'] == {'sys': 3, 'gyr': 3, 'acc': 2, 'mag': 0}, \
        'board_rows dropped imu_calibration'
    assert row['calibration'] == {'imu_bno055': 'make a figure 8'}, \
        'board_rows dropped the calibration instructions'
    assert row['stale'] is False and row['health_age'] < 1.0, \
        'a board seen just now must not be flagged stale'

    """
    NEGATIVE: a board last seen long ago is still `online` (offline takes _MISSED_BEATS), so without
    the flag the row would present handshake-age `stage` as live. The flag is the only thing that
    distinguishes them.
    """
    _StubBoard.last_seen = _time.monotonic() - (server_module.HEARTBEAT_S * 5)
    stale_row = server_module.Server.board_rows(hub)[0]
    assert stale_row['online'] is True, 'the stub is still online -- that is the point'
    assert stale_row['stale'] is True, 'a board silent for 5 heartbeats must be flagged stale'



_HUD_HARNESS = r"""
// Run the HUD's script under a stub DOM, feed it /events frames, print the cells it rendered.
const page = require('fs').readFileSync(process.argv[2], 'utf8');
const script = [...page.matchAll(/<script>([\s\S]*?)<\/script>/g)].map((m) => m[1]).join('\n');
const cells = {};
const cell = (id) => cells[id] || (cells[id] = {
  textContent: '', innerHTML: '', value: '', dataset: {}, children: [], style: {},
  classList: { toggle() {} }, addEventListener() {}, width: 200, height: 200,
  getContext: () => new Proxy({}, { get: () => () => {} }) });
let feed = null;
new Function('document', 'EventSource', 'fetch', 'setInterval', script)(
  { getElementById: cell, body: { classList: { toggle() {} } } },
  class { constructor() { feed = this; } }, async () => ({ json: async () => [] }), () => 0);
const out = [];
for (const frame of JSON.parse(process.argv[3])) {
  feed.onmessage({ data: JSON.stringify(frame) });
  out.push({ status: cell('status').textContent, reach: cell('reach').textContent,
             uptime: cell('uptime').textContent, degraded: cell('degraded').style.display === 'none'
               ? null : cell('degraded').textContent });
}
console.log(JSON.stringify(out));
"""


def test_hud_renders_an_events_frame():
    """
    The HUD must RENDER the /events frame, which wraps the rows as {cc, boards}.

    It took the frame for the list: every SSE message refreshed the 'live' clock and then threw, so the
    HUD read live on data it never drew, and the 2 s poll was the only thing painting it. Its cells were
    wrong too -- reach printed [object Object], uptime showed milliseconds as seconds, and the nominal
    empty `degraded` list lit the amber pill. Run under node with a stub DOM (skipped without node).
    """
    import shutil
    import subprocess
    import tempfile
    node = shutil.which('node')
    if node is None:
        print('   (node not found -- HUD render check skipped)')
        return
    page = _request(b'GET /hud HTTP/1.1\r\n\r\n').split(b'\r\n\r\n', 1)[1]
    directory = tempfile.mkdtemp()
    with open(os.path.join(directory, 'hud.html'), 'wb') as handle:
        handle.write(page)
    with open(os.path.join(directory, 'harness.js'), 'w') as handle:
        handle.write(_HUD_HARNESS)
    row = {'id': 'TMS-7C', 'online': True, 'stale': False, 'health_age': 0.4, 'stage': 'setting',
           'uptime': 125000, 'degraded': [], 'flight': {'reach': {'reachable': True, 'margin_m': 42}}}
    troubled = dict(row, degraded=['attitude-backup'],
                    flight={'reach': {'reachable': False, 'margin_m': -7}})
    frames = [{'cc': {}, 'boards': [row]}, {'cc': {}, 'boards': [troubled]}, {'cc': {}, 'boards': 'junk'}]
    done = subprocess.run([node, os.path.join(directory, 'harness.js'), os.path.join(directory, 'hud.html'),
                           json.dumps(frames)], capture_output=True)
    assert done.returncode == 0, done.stderr.decode('utf-8', 'replace')
    nominal, degraded, junk = json.loads(done.stdout)
    assert nominal['status'].startswith('live — TMS-7C'), nominal
    assert nominal['reach'] == 'zone ✓ +42 m' and nominal['uptime'] == '125 s', nominal
    assert nominal['degraded'] is None, 'an empty degraded list is NOMINAL -- no amber pill'
    assert degraded['degraded'] == 'attitude-backup' and degraded['reach'] == 'zone ✗ -7 m', degraded
    assert junk == degraded, 'a frame with no board list must leave the last good render alone'


_DASHBOARD_HARNESS = r"""
// Load the dashboard script under a stub DOM; drive selectBoard / updateObject / calibrateBoard /
// syncTime and the row formatters; print what landed where.
const page = require('fs').readFileSync(process.argv[2], 'utf8');
const script = [...page.matchAll(/<script>([\s\S]*?)<\/script>/g)].map((m) => m[1]).join('\n');
const cells = {};
const cell = (id) => cells[id] || (cells[id] = {
  textContent: '', innerHTML: '', value: '', dataset: {}, children: [], style: {},
  classList: { toggle() {}, add() {}, remove() {} }, addEventListener() {}, appendChild() {},
  querySelectorAll: () => [] });
const replies = [];                                     // /api/cmd bodies, consumed in order
const reply = async () => replies.shift() || '{}';
const posted = [];                                      // request bodies the page sent, in order
const sources = {};                                     // EventSource by url, to feed the page frames
const api = new Function('document', 'EventSource', 'fetch', 'setInterval', 'setTimeout', 'confirm',
  script + '\nreturn { selectBoard, calibrateBoard, updateObject, syncTime, fmtFlight, fmtPad };')(
  { getElementById: cell, querySelector: () => cell('query'), querySelectorAll: () => [],
    createElement: () => cell('new') },
  class { constructor(url) { sources[url] = this; } },
  async (url, options) => { posted.push(options && options.body);
                            return { status: 200, text: reply, json: async () => JSON.parse(await reply()) }; },
  () => 0, () => 0, () => true);
(async () => {
  const out = {};
  api.selectBoard('A');
  cell('inspresult').innerHTML = 'cards of A';
  api.selectBoard('A');
  out.same = cell('inspresult').innerHTML;
  api.selectBoard('B');
  out.switched = cell('inspresult').innerHTML;
  cell('insp-mission').value = '{}';
  await api.updateObject('A', 'mission');
  out.staleCard = cell('tick-mission').textContent;
  cell('actmsg').textContent = 'B calibrate imu: STILL OUTSTANDING';
  replies.push(JSON.stringify({ status: 'ok', args: [JSON.stringify({ baro: 'tare', imu: 'figure 8' })] }),
               JSON.stringify({ status: 'ok', args: ['{}'] }),
               JSON.stringify({ status: 'ok', args: [JSON.stringify({ imu: 'figure 8' })] }));
  await api.calibrateBoard('B');
  out.calibrated = cell('actmsg').textContent;
  out.flight = api.fmtFlight({ degraded: ['<i>x</i>'] });
  out.pad = api.fmtPad({ launchpad: [1, 2], site: '<i>y</i>' });
  // sync time acts only on a board in SETTING, and carries CC's fix from the last /events frame
  const frame = async (gps, stage) => {
    sources['/events'].onmessage({ data: JSON.stringify({ cc: { gps }, boards: [{ id: 'A', online: true, stage }] }) });
    await new Promise((resolve) => setImmediate(resolve));      // let the absent-roster fetch settle
    posted.length = 0;
  };
  const button = () => ({ disabled: cell('actsynctime').disabled, title: cell('actsynctime').title });
  api.selectBoard('A');
  out.sync = [];
  for (const gps of [{ usable: true, latitude: 25.5144, longitude: -80.3918 }, { usable: false }]) {
    await frame(gps, 'setting');
    replies.push(JSON.stringify({ status: 'ok', args: [JSON.stringify({ changed: ['epoch'] })] }));
    await api.syncTime('A');
    out.sync.push({ sent: JSON.parse(posted[0]), shown: cell('actmsg').textContent, button: button() });
  }
  out.unset = [];   // an ok that changed nothing, with no `changed` or one not a list, and a non-ok naming epoch
  for (const [status, changed] of [['ok', []], ['ok', undefined], ['ok', 'epoch'], ['ok', { epoch: 1 }],
                                   ['err', ['epoch']]]) {
    replies.push(JSON.stringify({ status, args: [JSON.stringify({ changed })] }));
    await api.syncTime('A');
    out.unset.push(cell('actmsg').textContent);
  }
  out.hubError = [];   // the hub's own failures carry no status or args: 404 no online board, 502 offline
  for (const body of [{ error: "no online board 'A'" }, { board: 'A', error: 'offline' }]) {
    replies.push(JSON.stringify(body));
    await api.syncTime('A');
    out.hubError.push(cell('actmsg').textContent);
  }
  out.refused = [];                     // any other stage: the button disables, a direct call sends nothing
  for (const stage of ['done', 'gliding', null]) {
    await frame({ usable: false }, stage);
    await api.syncTime('A');
    out.refused.push({ stage, posted: posted.length, shown: cell('actmsg').textContent, button: button() });
  }
  await frame({ usable: false }, 'setting');
  api.selectBoard('B');                 // a selected board missing from the frame
  out.absentBoard = button();
  api.selectBoard('');                  // nothing selected
  out.noBoard = button();
  // the browser's own clock: 2020-01-01 exactly is valid and sent as is, a second earlier sends nothing
  api.selectBoard('A');
  const realNow = Date.now;
  out.browserClock = [];
  for (const now of [1577836800000, 1577836799000]) {
    Date.now = () => now;
    posted.length = 0;
    replies.length = 0;
    replies.push(JSON.stringify({ status: 'ok', args: [JSON.stringify({ changed: ['epoch'] })] }));
    await api.syncTime('A');
    out.browserClock.push({ posted: posted.map((body) => JSON.parse(body)), shown: cell('actmsg').textContent });
  }
  Date.now = realNow;
  console.log(JSON.stringify(out));
})();
"""


def test_dashboard_actions_follow_the_selection():
    """
    What the dashboard shows and sends must belong to the SELECTED board.

    The inspect cards outlived a selection change: the heading named the new board while each card's
    update button still wrote to the old one. A guided calibrate reported success under the status
    table while every other outcome went to the actions bar, so a stale STILL OUTSTANDING stayed on
    screen. Board-reported `degraded` / `site` went into the markup raw. 'sync time' sets the selected
    board's RTC only in SETTING, from the browser's clock in a pinned zone. Run under node with a stub
    DOM (skipped without node); the negative cases are the same board re-selected, which keeps its
    cards, a sync in any other stage, of an absent board or from a browser clock before 2020, a reply
    that set no clock, and the hub's own failure (no online board, offline), which has no args.
    """
    import shutil
    import subprocess
    import tempfile
    node = shutil.which('node')
    if node is None:
        print('   (node not found -- dashboard behaviour check skipped)')
        return
    page = _request(b'GET / HTTP/1.1\r\n\r\n').split(b'\r\n\r\n', 1)[1]
    directory = tempfile.mkdtemp()
    with open(os.path.join(directory, 'index.html'), 'wb') as handle:
        handle.write(page)
    with open(os.path.join(directory, 'harness.js'), 'w') as handle:
        handle.write(_DASHBOARD_HARNESS)
    zone = dict(os.environ, TZ=_ZONE)  # the browser's timezone, pinned so a UTC host cannot hide a sign slip
    done = subprocess.run([node, os.path.join(directory, 'harness.js'), os.path.join(directory, 'index.html')],
                          capture_output=True, env=zone)
    assert done.returncode == 0, done.stderr.decode('utf-8', 'replace')
    out = json.loads(done.stdout)
    assert out['same'] == 'cards of A', 're-selecting the same board must keep its cards'
    assert out['switched'] == '', 'the cards of the board being left must go'
    assert out['staleCard'].startswith('not sent'), out['staleCard']
    assert out['calibrated'] == 'B calibrate: baro done — 1 left (imu)', out['calibrated']
    assert '<i>' not in out['flight'] and '&lt;i&gt;x' in out['flight'], out['flight']
    assert '<i>' not in out['pad'] and '&lt;i&gt;y' in out['pad'], out['pad']
    # sync time in SETTING: UTC epoch (not local) + the browser's offset EAST of UTC + CC's fix, 'dashboard'
    import time
    with_fix, without_fix = out['sync']
    for sync in (with_fix, without_fix):
        assert sync['sent']['board'] == 'A' and sync['sent']['command'] == 'update', sync
        object_name, payload = sync['sent']['params']
        mission = json.loads(payload)
        assert object_name == 'mission' and sorted(mission) == ['cc_position', 'epoch', 'source', 'utc_offset']
        assert type(mission['epoch']) is int and abs(mission['epoch'] - time.time()) < 5, mission
        assert mission['utc_offset'] == _ZONE_OFFSET and mission['source'] == 'dashboard', mission
        assert sync['shown'] == 'A sync time: ok', sync
        assert sync['button']['disabled'] is False and "THIS BROWSER's clock" in sync['button']['title'], sync
    assert json.loads(with_fix['sent']['params'][1])['cc_position'] == [25.5144, -80.3918], with_fix
    assert json.loads(without_fix['sent']['params'][1])['cc_position'] is None, without_fix
    assert len(out['unset']) == 5, out['unset']
    for shown in out['unset']:
        assert shown.startswith('A sync time: NOT set'), 'only an ok whose `changed` LIST names epoch is a set clock'
    assert out['hubError'] == ['A sync time: NOT set {"error":"no online board \'A\'"}',
                               'A sync time: NOT set {"board":"A","error":"offline"}'], out['hubError']
    # any other stage, or a board not connected: disabled, says why, and a direct call sends nothing
    for refused in out['refused']:
        assert refused['posted'] == 0 and refused['button']['disabled'] is True, refused
        reason = 'A sync time: NOT sent -- A is %s, ' % (refused['stage'] or 'unknown')
        assert refused['shown'].startswith(reason), refused
        assert 'only in stage setting (A is ' in refused['button']['title'], refused
        assert "THIS BROWSER's clock" in refused['button']['title'], refused
    assert out['absentBoard']['disabled'] is True, out['absentBoard']
    assert 'B is not connected' in out['absentBoard']['title'], out['absentBoard']
    assert out['noBoard']['disabled'] is True, out['noBoard']
    assert out['noBoard']['title'].startswith('only in stage setting (no board selected) -- '), out['noBoard']
    # the browser's clock before 2020 is unset: nothing sent, and the message says why
    valid, unset = out['browserClock']
    assert [json.loads(sent['params'][1])['epoch'] for sent in valid['posted']] == [1577836800], valid
    assert valid['shown'] == 'A sync time: ok', valid
    assert unset['posted'] == [], unset
    assert unset['shown'] == ("A sync time: NOT sent -- this browser's clock reads 2019-12-31T23:59:59Z, "
                              'before 2020'), unset


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


def test_board_timeout_answers_json_504():
    """
    A board that does not answer in time is a JSON 504, not the generic text/plain 500.

    board.exchange() raises TimeoutError after giving the link up. That fell to the catch-all, and every
    dashboard action then threw on res.json() and left its 'probing...' on screen. Negative: a board
    that does answer still comes back 200 with its reply.
    """
    class _Reply:
        command, args = 'ok', ['{}']

    class _Board:
        id, online = 'taster', True

        def __init__(self, fault):
            self._fault = fault

        async def command(self, command, *params):
            if self._fault:
                raise TimeoutError()
            return _Reply()

    body = json.dumps({'board': 'taster', 'command': 'probe', 'params': []}).encode()
    raw = b'POST /api/cmd HTTP/1.1\r\nContent-Length: %d\r\n\r\n%s' % (len(body), body)
    for fault, status, key in ((True, b'504 Gateway Timeout', 'error'), (False, b'200 OK', 'status')):
        server = web.Web(_Hub(), log=lambda *a: None)
        server.hub.boards['taster'] = _Board(fault)
        writer = _Writer()
        asyncio.run(server._handle(_Reader(raw), writer))
        assert writer.sent.startswith(b'HTTP/1.1 ' + status), writer.sent[:80]
        assert b'application/json' in writer.sent.split(b'\r\n\r\n', 1)[0], 'every /api reply is JSON'
        assert key in json.loads(writer.sent.split(b'\r\n\r\n', 1)[1]), writer.sent
    # and the page's cmd() survives a body that is NOT json (a hub fault, a proxy page) instead of throwing
    page = _request(b'GET / HTTP/1.1\r\n\r\n')
    assert b'await res.text()' in page.split(b'async function cmd(', 1)[1].split(b'\n}\n', 1)[0]


def test_post_with_bad_json_is_answered():
    """A POST carrying non-JSON must get a response (not a hang) -- json.loads is guarded (§26.8/9)."""
    body = b'{not json'
    raw = b'POST /api/cmd HTTP/1.1\r\nContent-Length: %d\r\n\r\n%s' % (len(body), body)
    response = _request(raw)
    assert response, 'a malformed POST body must still produce a response'


def test_every_rendered_control_is_bound():
    """
    Every control the page RENDERS must be referenced by its script.

    The suite asserted a binding for exactly one button, which is how fifteen dead controls shipped in a
    single refactor: in the served bytes, a <button> with no listener is indistinguishable from a live
    one. Deriving the list from the markup instead of naming them means a control cannot be added, or
    orphaned by a rename, without this failing.
    """
    import re as _re
    page = _request(b'GET / HTTP/1.1\r\n\r\n').decode('utf-8', 'replace')
    markup, separator, script = page.partition('<script>')
    assert separator, 'the dashboard served no <script> block'
    ids = _re.findall(r'<(?:button|select)[^>]*\bid="([^"]+)"', markup)
    assert len(ids) >= 12, 'expected many controls in the markup, found %d -- has the page changed shape?' % len(ids)
    unbound = [name for name in ids if "'%s'" % name not in script and '"%s"' % name not in script]
    assert not unbound, 'rendered but never referenced by the script: %s' % unbound


def test_dashboard_script_is_valid_javascript():
    """
    The page's script must PARSE. A syntax error anywhere in it kills the whole dashboard.

    Not hypothetical: a Python-style `#` comment was once pasted into the JS, and every control on the
    page died at once while the server still served 200 OK and every other assertion here still passed
    -- these tests check bytes, and bytes cannot tell valid JS from a parse error. Skipped when node is
    unavailable, so this adds a check where one can run without inventing a hard dependency.
    """
    import os
    import re as _re
    import shutil
    import subprocess
    import tempfile
    node = shutil.which('node')
    if node is None:
        print('   (node not found -- JS syntax check skipped)')
        return
    page = _request(b'GET / HTTP/1.1\r\n\r\n').decode('utf-8', 'replace')
    blocks = _re.findall(r'<script>(.*?)</script>', page, _re.S)
    assert blocks, 'the dashboard served no <script> block at all'
    path = os.path.join(tempfile.mkdtemp(), 'page.js')
    with open(path, 'w', encoding='utf-8') as handle:
        handle.write('\n'.join(blocks))
    done = subprocess.run([node, '--check', path], capture_output=True)
    assert done.returncode == 0, 'dashboard JS does not parse:\n' + done.stderr.decode('utf-8', 'replace')


test_routes()
test_hud_is_served_and_offline_safe()
test_hud_renders_an_events_frame()
test_dashboard_actions_follow_the_selection()
test_dashboard_carries_the_imu_calibration_column()
test_dashboard_script_is_valid_javascript()
test_every_rendered_control_is_bound()
test_malformed_request_line_does_not_hang()
test_bad_content_length_still_routes()
test_handler_fault_answers_500()
test_board_timeout_answers_json_504()
test_post_with_bad_json_is_answered()
print('ok: web -- routing + 404, IMU calibration column + calibrate action, not-ready row flag, '
      'malformed request line, bad Content-Length, handler fault -> 500, bad JSON POST answered')
