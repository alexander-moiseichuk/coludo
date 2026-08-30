"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

On-board test for the OTA staging path (ota.py): filename safety, chunk ordering, the SHA-256 gate,
and the atomic install with its .bak. Writes only to files named ota_selftest.* in the board's working
directory and removes them again. Run by `make test`.
"""

import asyncio
import binascii
import hashlib
import os

import ota

_NAME = 'ota_selftest.mpy'
_BODY = bytes(range(256)) * 4      # 1024 bytes, every byte value -- binary that base64/utf-8 would maul


def _cleanup():
    for suffix in ('', '.ota', '.bak'):
        try:
            os.remove(_NAME + suffix)
        except OSError:
            pass


async def amain():
    _cleanup()
    sha = binascii.hexlify(hashlib.sha256(_BODY).digest()).decode()
    upload = ota.Upload()

    """
    A path arriving off the network is a write-anywhere primitive unless it is checked -- but the check
    must still admit a SUBDIRECTORY, because drivers/, tasks/ and test/ are where most of the firmware
    lives and a bare-leaf-only rule could not update any of it.
    """
    for bad in ('../main.py', '/main.py', 'a/../../x.mpy', '.hidden.mpy', 'noext', 'x' * 100 + '.mpy',
                'drivers//x.mpy', 'drivers/.hidden.mpy'):
        assert upload.begin(bad, len(_BODY), sha) is not None, 'accepted unsafe path %r' % bad

    nested = 'drivers/' + _NAME  # drivers/ exists on the board; this is the case bare-leaf-only broke
    assert upload.begin(nested, len(_BODY), sha) is None, 'refused a legitimate subdirectory path'
    assert upload.chunk(0, _BODY[:32]) is None

    """
    Starting a DIFFERENT upload must not strand the first one's staging file. A board with a few
    megabytes and no shell has no way to reclaim it, so the leak would only surface much later as an
    unrelated write failure.
    """
    assert upload.begin(_NAME, len(_BODY), sha) is None
    try:
        os.stat(nested + '.ota')
        raise AssertionError('a superseded transfer left its staging file on the flash')
    except OSError:
        pass
    upload.discard()
    try:
        os.stat(_NAME + '.ota')
        raise AssertionError('discard left the staging file behind')
    except OSError:
        pass

    assert upload.begin(_NAME, 0, sha) is not None, 'accepted a zero size'
    assert upload.begin(_NAME, 999999999, sha) is not None, 'accepted a size past the cap'
    assert upload.begin(_NAME, len(_BODY), 'tooshort') is not None, 'accepted a malformed digest'

    # nothing above opened a transfer, so a chunk now has nothing to append to
    assert upload.chunk(0, b'x') is not None, 'accepted a chunk with no upload in progress'

    assert upload.begin(_NAME, len(_BODY), sha) is None
    assert upload.chunk(1, _BODY[:16]) is not None, 'accepted an out-of-order chunk'
    assert upload.chunk(0, _BODY[:16]) is None
    assert upload.chunk(1, _BODY) is not None, 'accepted a chunk that overruns the declared size'

    # an incomplete transfer must not install anything
    info, refused = upload.commit()
    assert info is None and refused is not None, 'committed an incomplete transfer'

    """
    The digest gate: a transfer that is the right LENGTH but wrong CONTENT must not reach the module
    that the next boot imports. This is the case a plain byte count cannot catch.
    """
    corrupt = bytearray(_BODY)
    corrupt[500] ^= 0xFF
    assert upload.begin(_NAME, len(_BODY), sha) is None
    assert upload.chunk(0, bytes(corrupt)) is None
    info, refused = upload.commit()
    assert info is None and 'sha mismatch' in refused, refused
    try:
        os.stat(_NAME)
        raise AssertionError('a corrupt upload was installed')
    except OSError:
        pass  # correct: nothing was put in place
    try:
        os.stat(_NAME + '.ota')
        raise AssertionError('a failed verify left its staging file on the flash')
    except OSError:
        pass  # correct: the debris of a failed push is not kept

    # the happy path, in two chunks, over a file that already exists -> the old one is kept as .bak
    with open(_NAME, 'wb') as handle:
        handle.write(b'previous version')
    assert upload.begin(_NAME, len(_BODY), sha) is None
    assert upload.chunk(0, _BODY[:600]) is None
    assert upload.status()['received'] == 600
    assert upload.chunk(1, _BODY[600:]) is None

    # a commit aimed at a different transfer must fail rather than install what happens to be staged
    info, refused = upload.commit('some_other.mpy', sha)
    assert info is None and 'is staged' in refused, refused
    info, refused = upload.commit(_NAME, 'f' * 64)
    assert info is None and 'digest' in refused, refused

    info, refused = upload.commit(_NAME, sha)
    assert refused is None, refused
    assert info['installed'] == _NAME and info['bytes'] == len(_BODY) and info['backup'] is True
    with open(_NAME, 'rb') as handle:
        assert handle.read() == _BODY, 'installed bytes differ from what was sent'
    with open(_NAME + '.bak', 'rb') as handle:
        assert handle.read() == b'previous version', 'the outgoing version was not kept'
    assert upload.status()['uploading'] is None, 'commit must clear the in-progress upload'

    _cleanup()
    print('ok: ota accepts subdirectory paths and rejects traversal, out-of-order and overrunning '
          'chunks, a wrong digest and a mis-aimed commit; discards staging on failure; installs a '
          'verified file atomically and keeps the previous version as .bak')


asyncio.run(amain())
