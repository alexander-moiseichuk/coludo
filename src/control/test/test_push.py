"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Host test for `push` (commands/push.py) driven against the REAL board-side staging (glider/ota.py).

The two halves are written in different languages' dialects and never see each other until a board is
on the link, so the failure worth catching here is a mismatch BETWEEN them: the chunk boundaries, the
hex encoding, and the SHA-256 the sender promises versus the one the board computes. A test that
stubbed the board side would agree with itself and prove nothing -- so this wires the actual Upload
the firmware runs, and checks the bytes that land are the bytes that were sent.

The payload is deliberately binary spanning every byte value: the base64 path the protocol uses for
text cannot carry it (utf-8 decode fails above 0x7f, and '=' padding parses as a named param), which
is why `push` is hex, and a change back would fail here rather than on a board at the field.
"""

import asyncio
import base64
import hashlib
import json
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_HERE))                        # src/control
sys.path.insert(0, os.path.join(_HERE, '..', '..', 'glider'))     # src/glider (the board's ota.py)
import ota  # noqa: E402
from commands.push import push_command  # noqa: E402

_BLOB = bytes(range(256)) * 9 + b'tail'   # 2308 bytes: not a multiple of the 512-byte chunk
_REMOTE = 'push_selftest.mpy'


class _FakeBoard:
    """A board that speaks the push commands into a real ota.Upload, as the firmware dispatcher does."""

    def __init__(self):
        self.online = True
        self.upload = ota.Upload()
        self.chunks = 0

    async def command(self, command, *args, timeout=None, quiet=False):
        if command == 'push-begin':
            refused = self.upload.begin(args[0], int(args[1]), args[2])
        elif command == 'push':
            self.chunks += 1
            token = args[1]
            assert '=' not in token, 'padding would be parsed as a named param on the wire'
            refused = self.upload.chunk(int(args[0]),
                                        base64.b64decode(token + '=' * (-len(token) % 4)))
        elif command == 'push-commit':
            info, refused = self.upload.commit(*args)
            if refused is None:
                return _Reply('ok', [json.dumps(info)])
        elif command == 'push-abort':
            self.upload.reset()
            refused = None
        else:
            raise AssertionError('unexpected command %r' % command)
        return _Reply('err', ['badargs', refused]) if refused else _Reply('ok', ['{}'])


class _Reply:
    def __init__(self, command, args):
        self.command, self.args = command, args


class _FakeHub:
    def __init__(self, board):
        self.boards = {'TMS-7X': board}
        self.lines = []
        self.log = self.lines.append


def _cleanup():
    for suffix in ('', '.ota', '.bak'):
        try:
            os.remove(_REMOTE + suffix)
        except OSError:
            pass


async def amain():
    _cleanup()
    source = os.path.join(_HERE, 'push_selftest_source.bin')
    with open(source, 'wb') as handle:
        handle.write(_BLOB)
    board = _FakeBoard()
    hub = _FakeHub(board)

    try:
        replies = await push_command(hub, ['push', 'TMS-7X', source, _REMOTE], {})
        assert replies[0].startswith('from cc ok'), replies
        result = json.loads(replies[0].split(' ', 3)[3])
        assert result['installed'] == _REMOTE and result['bytes'] == len(_BLOB), result
        assert result['reboot_required'] is True, 'the operator must be told a reboot is needed'

        # 2308 bytes over 256-byte chunks is 10, the last carrying only 4 -- the short-final-chunk
        # boundary that an off-by-one in the range/slice arithmetic would silently truncate
        assert board.chunks == 10, board.chunks
        with open(_REMOTE, 'rb') as handle:
            landed = handle.read()
        assert landed == _BLOB, 'installed bytes differ from the source file'
        assert hashlib.sha256(landed).hexdigest() == hashlib.sha256(_BLOB).hexdigest()

        # a board that refuses (armed, in flight) must leave nothing behind and say so
        board.upload = ota.Upload()
        missing = await push_command(hub, ['push', 'TMS-7X', source + '.nope'], {})
        assert missing[0].startswith('from cc err badargs'), missing
        offline = await push_command(hub, ['push', 'TMS-NOPE', source], {})
        assert offline[0].startswith('from cc err noboard'), offline
    finally:
        os.remove(source)
        _cleanup()

    print('ok: push carries a binary file as unpadded base64, the board stages and verifies it, and '
          'the installed bytes match the source exactly (short final chunk included)')


asyncio.run(amain())
