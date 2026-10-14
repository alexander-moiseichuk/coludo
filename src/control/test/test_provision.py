"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Host (CPython) test for tools/provision.sh, which writes a Rust board's cfg NVS partition -- the board config,
launch.config and the Wi-Fi passwords -- before the airframe is sealed. A sealed, Wi-Fi-only board provisioned
wrong cannot be reached to fix it, so the image is decoded here with ESP-IDF's own nvs_tool.py, independently of the
script's own read-back: every key, both blobs byte for byte, every password, and IDF's integrity check over every
page the image uses. The creds keys are pinned as literal vectors, never recomputed with the script's own formula.

No board and no serial port is involved. Every run passes a --port that does not exist, and a fake esptool first on
PATH stands in for the real one: it answers read-flash with a partition table built here by IDF's gen_esp32part.py
and keeps a copy of what write-flash was given. Every password is made up, in a fake creds directory (CREDS_DIR).
Every run, refusals included, is checked to leave the script's temporary directory gone and no password in its
output.

The positives beyond the round trip: a password read as plan B reads it on MicroPython (a lone '\r' and a no-break
space stay), JSON that serde_json takes at its edges (a surrogate pair, a 301-digit integer, 1e300), and the notes
following plan B's network rule, and a debounce_ms at either end of the Rust board's range. The negatives: a board
config plan B's config.validate refuses, one the Rust board's validate refuses beyond it (a sensor's or a component's
debounce_ms outside its range, or not an int), a board or launch file serde_json refuses (NaN, 1e400, a 401-digit
integer, a lone surrogate) or that is not JSON or not UTF-8, a launch file that is not a JSON object, a creds file
missing, empty, not UTF-8 or past 64 bytes, an SSID empty, past 32 bytes, holding '/' or given twice, two SSIDs
whose CRC-32 keys collide, a board whose partition table has no NVS cfg where partitions.csv puts it (a MicroPython
board's filesystem lies there), esptool failing (it exits 2, never a usage error), no ESP-IDF, and usage errors.
Needs ESP-IDF (IDF_PATH, default ~/.espressif/esp-idf/v5.5.5) and its Python environment, as the script does.
"""

import base64
import json
import os
import stat
import subprocess
import sys
import tempfile
import zlib

HERE: str = os.path.dirname(os.path.abspath(__file__))
ROOT: str = os.path.normpath(os.path.join(HERE, '..', '..', '..'))
SCRIPT: str = os.path.join(ROOT, 'tools', 'provision.sh')
BOARD: str = os.path.join(ROOT, 'launches', '20261003', 'TMS-7F', 'tms7f.config')  # the sealed board's file
BOARD_CONFIG_ID: str = 'f80fcbb3231e'  # its id, a test vector of the port plan's config store milestone
PARTITIONS: str = os.path.join(ROOT, 'src', 'avionics', 'firmware', 'partitions.csv')
IDF: str = os.environ.get('IDF_PATH') or os.path.expanduser('~/.espressif/esp-idf/v5.5.5')
NVS_TOOL: str = os.path.join(IDF, 'components', 'nvs_flash', 'nvs_partition_tool', 'nvs_tool.py')
GEN_ESP32PART: str = os.path.join(IDF, 'components', 'partition_table', 'gen_esp32part.py')


def _cfg_row() -> tuple:
    """partitions.csv's cfg row as (offset text, size): the test follows the table, as the script does."""
    with open(PARTITIONS) as table:
        for row in table:
            fields = [field.strip() for field in row.split('#', 1)[0].split(',')]
            if fields[0] == 'cfg':
                return fields[3], int(fields[4], 0)
    raise AssertionError('partitions.csv has no cfg row')


CFG_OFFSET, CFG_SIZE = _cfg_row()  # today 0x620000, 1 MB
NVS_PAGE: int = 0x1000  # bytes per NVS page

"""
The NVS key of each network's password, as literals: zlib.crc32 of the SSID, which is esp_rom_crc32_le seeded with 0
in firmware/src/config.rs key(). Pinned here so a drift of the script's formula (seed, case, encoding) fails.
"""
KEYS: dict = {'coludo': 'wdb960ca1', 'panda': 'wc5079ef3', 'field': 'w5bf54558', 'uejgtcuo': 'wfbe81776'}

"""
Made-up passwords. The first creds file has padding and a second line, which plan B's wifi driver strips and
ignores; the password keeps its comma, quotes and inner spaces, which must survive the CSV the script hands the
generator. 'field' is read as MicroPython reads it: its line ends at the '\n' only (the lone '\r' stays inside) and
only ASCII whitespace is stripped (the no-break space and the unit separator stay; CPython's strip() takes both).
"""
CREDS_FILES: dict = {
    'coludo': '  made-up, "pass" #1  \nsecond line, never read\n',
    'panda': 'made-up-panda-password\n',
    'field': 'made-up\rfield\u00a0\x1f \t\nsecond line, never read\n',
    'uejgtcuo': 'made-up-collision-one\n',
    'iiwucoup': 'made-up-collision-two\n',
    'workshop': 'made-up-workshop-password\n',
    'empty': '\nmade-up-on-the-second-line\n',
    'long': 'made-up-' + 'x' * 57 + '\n',  # 65 bytes: one past what the firmware holds
    'binary': b'made-up-\xff\n',  # not UTF-8
}
PASSWORDS: dict = {'coludo': 'made-up, "pass" #1', 'panda': 'made-up-panda-password',
                   'field': 'made-up\rfield\u00a0\x1f'}
SECRETS: tuple = ('made-up', 'second line')  # no run may print any of it

"""Two SSIDs with one zlib.crc32 (found by a seeded search over 8 lowercase letters), so one NVS key."""
COLLIDING: tuple = ('uejgtcuo', 'iiwucoup')

"""A launch file, with JSON at serde_json's edges it still takes: a surrogate pair, a 301-digit integer, 1e300."""
LAUNCH: dict = {'launch_id': 'provision-test', 'site': 'HPRC', 'latitude': 25.514379, 'longitude': -80.391795,
                'altitude': 2.0, 'note': 'rocket \U0001f680', 'serial': 10 ** 300, 'ceiling': 1e300}

"""
A MicroPython-like table, without cfg (filesystem there), a Rust table with cfg elsewhere, and one whose
cfg sits at the right offset and size but is not NVS.
"""
MICROPYTHON_TABLE: str = ('nvs, data, nvs, 0x9000, 0x6000\nphy_init, data, phy, 0xf000, 0x1000\n'
                          'factory, app, factory, 0x10000, 0x3f0000\nvfs, data, fat, 0x400000, 0xc00000\n')
MOVED_TABLE: str = ('nvs, data, nvs, 0x9000, 0x6000\nphy_init, data, phy, 0xf000, 0x1000\n'
                    'factory, app, factory, 0x10000, 0x400000\ncfg, data, nvs, 0x7f0000, 0x10000\n')
FAT_CFG_TABLE: str = ('nvs, data, nvs, 0x9000, 0x6000\nphy_init, data, phy, 0xf000, 0x1000\n'
                      'factory, app, factory, 0x10000, 0x400000\ncfg, data, fat, %s, %#x\n' % (CFG_OFFSET, CFG_SIZE))

"""
The esptool stand-in: logs its arguments, answers read-flash with $FAKE_TABLE, keeps write-flash's image. The command
$FAKE_FAIL names fails as esptool fails (no port, a lost link): a fatal error and exit 2.
"""
FAKE_ESPTOOL: str = '''#!%s
import os
import shutil
import sys

arguments = sys.argv[1:]
with open(os.environ['FAKE_LOG'], 'a') as log:
    log.write(' '.join(arguments) + '\\n')
if os.environ.get('FAKE_FAIL') in arguments:
    sys.stderr.write('A fatal error occurred: fake esptool fails %%s\\n' %% os.environ['FAKE_FAIL'])
    sys.exit(2)
if 'read-flash' in arguments:
    shutil.copy(os.environ['FAKE_TABLE'], arguments[arguments.index('read-flash') + 3])
elif 'write-flash' in arguments:
    shutil.copy(arguments[arguments.index('write-flash') + 2], os.environ['FAKE_IMAGE'])
else:
    sys.exit('fake esptool: an unexpected command')
''' % sys.executable

_SCRATCH: tempfile.TemporaryDirectory = tempfile.TemporaryDirectory()  # everything a run writes; gone at exit
_OUTPUTS: list = []  # every run's stdout and stderr, for the password check


def _path(*parts: str) -> str:
    """A path inside the scratch directory."""
    return os.path.join(_SCRATCH.name, *parts)


def _write(path: str, content: str | bytes) -> str:
    """
    Write `content` to `path`, byte for byte: a str as UTF-8, no newline translated.

    Args:
        path - the file.
        content - its text, or its bytes (a file that is not UTF-8).

    Returns:
        The path.
    """
    with open(path, 'wb') as handle:
        handle.write(content.encode('utf-8') if isinstance(content, str) else content)
    return path


def _table(name: str, csv_text: str | None) -> str:
    """
    A binary partition table, built by IDF's gen_esp32part.py as firmware/flash.sh builds its own.

    Args:
        name - the scratch file name.
        csv_text - the table as partition CSV; None for the firmware's partitions.csv.

    Returns:
        The table's path.
    """
    source = PARTITIONS if csv_text is None else _write(_path(name + '.csv'), csv_text)
    subprocess.run([sys.executable, GEN_ESP32PART, '-q', source, _path(name + '.bin')], check=True,
                   env=dict(os.environ, PYTHONDONTWRITEBYTECODE='1'))
    return _path(name + '.bin')


def _setup() -> None:
    """The fake creds directory, the launch file, the fake esptool and the script's own temporary directory."""
    for missing in (NVS_TOOL, GEN_ESP32PART):
        assert os.path.isfile(missing), 'no ESP-IDF tool %s: set IDF_PATH' % missing
    os.makedirs(_path('creds'))
    for ssid, content in CREDS_FILES.items():
        _write(_path('creds', ssid + '.creds'), content)
    assert zlib.crc32(COLLIDING[0].encode()) == zlib.crc32(COLLIDING[1].encode()), 'the pair must collide'
    assert len(CREDS_FILES['long'].rstrip('\n').encode()) == 65, 'one byte past the firmware'
    _write(_path('launch.config'), json.dumps(LAUNCH) + '\n')
    os.makedirs(_path('bin'))
    esptool = _write(_path('bin', 'esptool'), FAKE_ESPTOOL)
    os.chmod(esptool, stat.S_IRWXU)
    os.makedirs(_path('tmp'))


def _run(*arguments: str, table: str | None = None, overrides: dict | None = None) -> subprocess.CompletedProcess:
    """
    Run provision.sh against the fake board, then check the run left nothing behind.

    Args:
        arguments - the script's arguments; --port is always added, naming a port that does not exist.
        table - the partition table the fake esptool's read-flash answers with; None for none.
        overrides - environment variables set last: FAKE_FAIL, or an IDF_PATH without ESP-IDF.

    Returns:
        The finished run (stdout, stderr, returncode), its output also kept for the password check.
    """
    environment = {key: value for key, value in os.environ.items()
                   if key not in ('PORT', 'BAUD', 'CREDS_DIR', 'FAKE_FAIL')}
    environment.update(PATH=_path('bin') + os.pathsep + os.environ['PATH'], TMPDIR=_path('tmp'),
                       CREDS_DIR=_path('creds'), FAKE_LOG=_path('esptool.log'), FAKE_IMAGE=_path('flashed.bin'),
                       FAKE_TABLE=table or _path('no-table'))
    environment.update(overrides or {})
    result = subprocess.run([SCRIPT, '--port', _path('no-such-port'), *arguments], capture_output=True, text=True,
                            env=environment, cwd=_SCRATCH.name)
    _OUTPUTS.append(result.stdout + result.stderr)
    assert os.listdir(_path('tmp')) == [], 'the work directory (it holds the passwords) outlived the run'
    return result


def _esptool_calls() -> list:
    """What the fake esptool was asked since the last call: one argument list per invocation (then forgotten)."""
    try:
        with open(_path('esptool.log')) as handle:
            calls = [line.split() for line in handle]
    except FileNotFoundError:
        return []
    os.remove(_path('esptool.log'))
    return calls


def _decode(image: str) -> dict:
    """
    The image as ESP-IDF's nvs_tool.py reads it, after its integrity check passes on every page the image uses.

    The check must have checked something: a 'CRC32: OK' line per page that is not erased (its header is not all
    0xFF), counted from the image itself, so a check that printed nothing cannot pass. nvs_tool's minimal dump lists a
    multi-page blob as its chunks in page order, which is chunk order in an image the generator wrote fresh, so the
    chunks are joined as listed.

    Args:
        image - the partition image.

    Returns:
        {'namespace/key': bytes for a blob, str for a string}.
    """
    environment = dict(os.environ, PYTHONDONTWRITEBYTECODE='1')
    check = subprocess.run([sys.executable, NVS_TOOL, '-d', 'none', '-i', '--color', 'never', image],
                           capture_output=True, text=True, check=True, env=environment).stdout
    problems = [line for line in check.splitlines()
                if line.strip() and line.strip() != 'Page Empty' and not line.endswith('CRC32: OK')]
    assert problems == [], problems
    with open(image, 'rb') as handle:
        content = handle.read()
    used = sum(content[start : start + 32] != b'\xff' * 32 for start in range(0, len(content), NVS_PAGE))
    checked = [line for line in check.splitlines() if line.startswith('Page no. ') and line.endswith('CRC32: OK')]
    assert used >= 1 and len(checked) == used, (used, checked)
    dump = subprocess.run([sys.executable, NVS_TOOL, '-d', 'minimal', '-f', 'json', '--color', 'never', image],
                          capture_output=True, text=True, check=True, env=environment).stdout
    found = {}
    for entry in json.loads(dump):
        name = '%s/%s' % (entry['namespace'], entry['key'])
        if entry['encoding'] == 'string':
            assert name not in found, name
            found[name] = entry['data']
        else:
            assert entry['encoding'] == 'blob_data', entry['encoding']
            found[name] = found.get(name, b'') + base64.b64decode(entry['data'])
    return found


def test_dry_run() -> None:
    """A dry run validates, builds and lists every key -- never a value -- and asks esptool nothing."""
    result = _run('--dry-run', '--launch', 'launch.config', '--ssid', 'coludo', '--ssid', 'panda', BOARD)
    assert result.returncode == 0, result.stdout + result.stderr
    lines = result.stdout.splitlines()
    assert lines[0] == 'board config %s: valid, board.id TMS-7F, config_id %s' % (BOARD, BOARD_CONFIG_ID), lines
    verified = 'cfg image verified: %#x bytes for the cfg partition at %s; it holds' % (CFG_SIZE, CFG_OFFSET)
    assert lines[1] == verified, lines
    expected = sorted(['  config/board     blob, %d bytes' % os.path.getsize(BOARD),
                       '  config/launch    blob, %d bytes' % os.path.getsize(_path('launch.config'))]
                      + ['  %-16s string: the password for %s' % ('creds/' + KEYS[ssid], ssid)
                         for ssid in ('coludo', 'panda')])
    assert lines[2:-1] == expected, lines  # both networks of the board file have a password: no note
    assert lines[-1] == 'dry run: nothing flashed', lines
    assert result.stderr == '', result.stderr
    assert _esptool_calls() == [], 'a dry run reached esptool'


def _notes(*arguments: str) -> list:
    """
    The notes of a dry run that must succeed.

    Args:
        arguments - the script's arguments, --dry-run added.

    Returns:
        Its 'note: ' lines, in order.
    """
    result = _run('--dry-run', *arguments)
    assert result.returncode == 0, result.stdout + result.stderr
    return [line for line in result.stdout.splitlines() if line.startswith('note: ')]


def _missing(ssid: str) -> str:
    """The note on a network the board tries with no password in the image."""
    return 'note: the board tries the network %s, and the image holds no password for it' % ssid


def _unused(ssid: str) -> str:
    """The note on a stored password the board never uses."""
    return 'note: the board never tries the network %s: its password is stored, never used' % ssid


def test_notes() -> None:
    """
    A network the board tries without a password in the image, and a password for one it never tries, are noted; the
    networks follow plan B's wifi rule (a non-empty list wins over the top-level ssid, a disabled entry is dropped).
    """
    assert _notes('--ssid', 'panda', '--ssid', 'workshop', BOARD) == [_missing('coludo'), _unused('workshop')]
    result = _run('--dry-run', BOARD)  # the board config alone: no launch key, no creds namespace
    assert result.returncode == 0, result.stdout + result.stderr
    lines = result.stdout.splitlines()
    assert lines[2:] == ['  config/board     blob, %d bytes' % os.path.getsize(BOARD), _missing('coludo'),
                         _missing('panda'), 'dry run: nothing flashed'], lines
    listed = _variant('listed.config', {'wifi': {'ssid': 'coludo', 'networks': [
        'panda', {'ssid': 'workshop', 'enabled': False}, {'ssid': 'field', 'policy': 'disabled'}, '']}})
    assert _notes('--ssid', 'coludo', '--ssid', 'workshop', '--ssid', 'field', listed) == [
        _missing('panda'), _unused('coludo'), _unused('field'), _unused('workshop')]
    alone = _variant('alone.config', {'wifi': {'ssid': 'coludo', 'networks': []}})  # an empty list: the ssid
    assert _notes('--ssid', 'coludo', alone) == []
    assert _notes('--ssid', 'panda', alone) == [_missing('coludo'), _unused('panda')]
    off = _variant('off.config', {'wifi': {'policy': 'disabled', 'ssid': 'coludo', 'networks': ['coludo']}})
    assert _notes('--ssid', 'coludo', off) == [_unused('coludo')]  # the radio is never touched


def test_flash_round_trip() -> None:
    """The flash path checks the board's table, warns, writes only cfg, and IDF reads the written image back whole."""
    result = _run('--launch', 'launch.config', '--ssid', 'coludo', '--ssid', 'panda', '--ssid', 'field', BOARD,
                  table=_table('rust', None))
    assert result.returncode == 0, result.stdout + result.stderr
    calls = _esptool_calls()
    assert len(calls) == 2, calls
    port = ['--chip', 'esp32p4', '--port', _path('no-such-port'), '--baud', '921600']
    assert calls[0][:6] == port and calls[0][6:9] == ['read-flash', '0x8000', '0xc00'], calls[0]
    assert calls[1][:6] == port and calls[1][6:8] == ['write-flash', CFG_OFFSET] and len(calls[1]) == 9, calls[1]
    assert 'REPLACES everything on the partition' in result.stdout, 'the write must be announced'
    assert result.stdout.index('REPLACES') > result.stdout.index('cfg image verified'), 'announced after the check'
    assert os.path.getsize(_path('flashed.bin')) == CFG_SIZE, 'the image must be exactly the partition'
    found = _decode(_path('flashed.bin'))
    with open(BOARD, 'rb') as handle:
        board = handle.read()
    with open(_path('launch.config'), 'rb') as handle:
        launch = handle.read()
    expected = {'config/board': board, 'config/launch': launch}
    for ssid in ('coludo', 'panda', 'field'):
        expected['creds/' + KEYS[ssid]] = PASSWORDS[ssid]
    assert found == expected, sorted(found)
    assert result.stdout.count('note: ') == 1 and _unused('field') in result.stdout, result.stdout
    os.remove(_path('flashed.bin'))


def test_board_without_cfg_refused() -> None:
    """A board whose own table lacks partitions.csv's NVS cfg (MicroPython, an older Rust table) is never written."""
    for name, table in (('micropython', MICROPYTHON_TABLE), ('moved', MOVED_TABLE), ('fat', FAT_CFG_TABLE)):
        result = _run('--ssid', 'panda', BOARD, table=_table(name, table))
        assert result.returncode == 1, (name, result.stdout, result.stderr)
        assert 'has no cfg NVS partition at %s+%#x' % (CFG_OFFSET, CFG_SIZE) in result.stderr, result.stderr
        assert 'flash the Rust firmware first' in result.stderr, result.stderr
        calls = _esptool_calls()
        assert [call[6] for call in calls] == ['read-flash'], (name, calls)
        assert 'REPLACES' not in result.stdout and not os.path.exists(_path('flashed.bin')), name


def test_esptool_failure() -> None:
    """
    esptool failing (exit 2, as with no board on the port) is a failure with a reason (exit 1), never a usage error;
    a failed write says the partition may be partly written and the script must run again before the seal.
    """
    port = _path('no-such-port')
    table = _table('rust', None)
    cases = (('read-flash', ['read-flash'], 'could not read the partition table on %s' % port),
             ('write-flash', ['read-flash', 'write-flash'], 'writing cfg on %s failed: the partition may be partly '
              'written -- run provision.sh again before sealing' % port))
    for failing, commands, reason in cases:
        result = _run('--ssid', 'panda', BOARD, table=table, overrides={'FAKE_FAIL': failing})
        assert result.returncode == 1, (failing, result.returncode, result.stdout, result.stderr)
        assert reason in result.stderr and 'fake esptool fails ' + failing in result.stderr, result.stderr
        assert [call[6] for call in _esptool_calls()] == commands, failing
        assert not os.path.exists(_path('flashed.bin')), failing


def _variant(name: str, change: dict) -> str:
    """
    A copy of the board file with some sections replaced.

    Args:
        name - the scratch file name.
        change - {section: new value}.

    Returns:
        The copy's path.
    """
    with open(BOARD) as handle:
        document = json.load(handle)
    document.update(change)
    return _write(_path(name), json.dumps(document, indent=1))


def test_refusals() -> None:
    """Each refusal exits with its reason, before anything is flashed, and leaves nothing behind."""
    with open(BOARD) as handle:
        text = handle.read()
    buses = json.loads(text)['buses']
    buses['i2c']['0']['scl'] = buses['i2c']['0']['sda']
    invalid = _variant('invalid.config', {'wifi': {'mode': 'ap', 'ssid': 'coludo'}, 'buses': buses})
    assert '"rev": 1,' in text, 'the strict JSON cases edit this field'
    edits = {'nan': '"rev": NaN,', 'huge': '"rev": 1e400,', 'long': '"rev": 1%s,' % ('0' * 400),
             'surrogate': '"rev": 1, "note": "\\ud800",'}
    strict = {name: _write(_path(name + '.config'), text.replace('"rev": 1,', edit, 1)) for name, edit in edits.items()}
    binary = _write(_path('binary.config'), b'{"board": "\xff"}\n')
    _write(_path('list.launch'), '[1, 2]\n')
    _write(_path('garbage.launch'), 'garbage\n')
    _write(_path('surrogate.launch'), '{"site": "\\udc00"}\n')
    cases = (
        ((invalid,), 1, ["plan B config.validate refuses it:\n  wifi.mode must be 'sta'\n",
                         'GPIO%d used by both i2c:0.sda and i2c:0.scl' % buses['i2c']['0']['sda']]),
        ((strict['nan'],), 1, ['nan.config: not JSON (NaN is not JSON)']),
        ((strict['huge'],), 1, ['huge.config: not JSON (1e400 is not a finite number)']),
        ((strict['long'],), 1, ['long.config: not JSON (1%s is not a finite number)' % ('0' * 400)]),
        ((strict['surrogate'],), 1, ['surrogate.config: not JSON (a \\u escape of a lone surrogate)']),
        ((binary,), 1, ['binary.config: not UTF-8 text']),
        (('--launch', 'list.launch', BOARD), 1, ['list.launch: a launch.config must be a JSON object']),
        (('--launch', 'absent.launch', BOARD), 1, ['absent.launch: No such file or directory']),
        (('--launch', 'garbage.launch', BOARD), 1, ['garbage.launch: not JSON (Expecting value']),
        (('--launch', 'surrogate.launch', BOARD), 1, ['surrogate.launch: not JSON (a \\u escape of a lone surrogate)']),
        (('--ssid', 'nowhere', BOARD), 1, ['--ssid nowhere: no password: %s: No such file or directory'
                                           % _path('creds', 'nowhere.creds')]),
        (('--ssid', 'empty', BOARD), 1, ['empty.creds: no password on its first line']),
        (('--ssid', 'long', BOARD), 1, ['long.creds: the password is longer than the 64 bytes the firmware holds']),
        (('--ssid', 'binary', BOARD), 1, ['binary.creds: not UTF-8 text']),
        (('--ssid', 'panda', '--ssid', 'panda', BOARD), 1, ['--ssid panda given twice']),
        (('--ssid', COLLIDING[0], '--ssid', COLLIDING[1], BOARD), 1,
         ['--ssid %s and --ssid %s share the key %s (a CRC-32 collision)' % (COLLIDING + (KEYS[COLLIDING[0]],))]),
        (('--ssid', 'a/b', BOARD), 1, ["--ssid 'a/b': an SSID is 1..32 bytes, without /"]),
        (('--ssid', 'x' * 33, BOARD), 1, ["--ssid '%s': an SSID is 1..32 bytes, without /" % ('x' * 33)]),
        (('--ssid', '', BOARD), 1, ["--ssid '': an SSID is 1..32 bytes, without /"]),
        ((), 2, ['no board config']),
        (('--ssid',), 2, ['--ssid needs a value']),
        (('--bogus', BOARD), 2, ['unknown option --bogus']),
        ((BOARD, BOARD), 2, ['one board config only']),
    )
    table = _table('rust', None)  # a board that WOULD take the write
    for arguments, code, reasons in cases:
        result = _run(*arguments, table=table)
        assert result.returncode == code, (arguments, result.returncode, result.stdout, result.stderr)
        for reason in reasons:
            assert reason in result.stderr, (arguments, reason, result.stderr)
        assert 'cfg image verified' not in result.stdout, arguments
        assert _esptool_calls() == [], arguments
    result = _run(BOARD, table=table, overrides={'IDF_PATH': _path('no-idf')})
    assert result.returncode == 1, (result.returncode, result.stdout, result.stderr)
    assert 'no ESP-IDF NVS tools under %s (set IDF_PATH)' % _path('no-idf') in result.stderr, result.stderr
    assert result.stdout == '' and _esptool_calls() == [], result.stdout


def _debounced(name: str, section: str, debounce: object) -> str:
    """
    A copy of the board file with one device debouncing `debounce` ms.

    Args:
        name - the scratch file name.
        section - 'components' for the separation switch, 'sensors' for the first sensor.
        debounce - that device's debounce_ms.

    Returns:
        The copy's path.
    """
    with open(BOARD) as handle:
        devices = json.load(handle)[section]
    device = next(device for device in devices if section == 'sensors' or device['name'] == 'separation')
    device['debounce_ms'] = debounce
    return _variant(name, {section: devices})


def test_debounce_bound() -> None:
    """
    The Rust board's validate refuses a debounce_ms outside 0 to 1000 ms, or one that is not an int, on any sensor or
    component, where plan B's takes any value; such a stored config would boot the board's default config, so
    provision.sh refuses it as the board would, in the board's words. At either end of the range, the file is valid.
    """
    for name, debounce in (('bound.config', 1000), ('zero.config', 0)):
        result = _run('--dry-run', _debounced(name, 'components', debounce))
        assert result.returncode == 0, result.stdout + result.stderr
        assert result.stdout.startswith('board config %s: valid, board.id TMS-7F' % _path(name)), result.stdout
    table = _table('rust', None)  # a board that WOULD take the write
    separation, sensor = "components 'separation'", "sensors 'accel_adxl375'"
    for name, section, debounce, label, got in (
            ('past.config', 'components', 1001, separation, '1001'),
            ('seconds.config', 'components', 20000, separation, '20000'),
            ('negative.config', 'components', -1, separation, '-1'),
            ('float.config', 'components', 20.0, separation, '20.0'),
            ('bool.config', 'components', True, separation, 'True'),
            ('text.config', 'components', '20', separation, "'20'"),
            ('null.config', 'components', None, separation, 'None'),
            ('sensor.config', 'sensors', 5000, sensor, '5000')):
        result = _run(_debounced(name, section, debounce), table=table)
        assert result.returncode == 1, (name, result.returncode, result.stdout, result.stderr)
        reason = ("%s: the Rust board's validate refuses it:\n  %s.debounce_ms must be an int from 0 to 1000 ms "
                  '(got %s)\n' % (_path(name), label, got))
        assert reason in result.stderr, (name, result.stderr)
        assert 'cfg image verified' not in result.stdout and _esptool_calls() == [], name


def test_no_password_printed() -> None:
    """No run, the refusals included, printed any part of a password or a creds file."""
    assert len(_OUTPUTS) >= 37, len(_OUTPUTS)  # the check must have seen every run
    for output in _OUTPUTS:
        for secret in SECRETS:
            assert secret not in output, output


_setup()
test_dry_run()
test_notes()
test_flash_round_trip()
test_board_without_cfg_refused()
test_esptool_failure()
test_refusals()
test_debounce_bound()
test_no_password_printed()
print('ok: provision.sh -- the cfg image reads back whole through IDF (keys, blobs, passwords), writes only cfg on a '
      'board whose table has it, and refuses invalid input without leaking a password or its work directory')
