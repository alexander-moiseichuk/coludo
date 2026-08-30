"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Over-the-air module updates: stage a .mpy over the CC link, verify it, install it atomically.

Deploying by USB means opening the airframe. Once the glider is packed -- and on a launch day, once it
is on the rail -- that is the difference between shipping a fix and flying the bug. This carries a
module over the link the board already holds to Control, so a change reaches a sealed airframe.

The transfer is deliberately three steps (`push-begin`, `push`, `push-commit`) rather than one
command per file. Nothing touches the live module until the LAST byte has arrived and the SHA-256 of
what landed matches what the sender promised: a link that drops mid-transfer leaves a half-written
file in staging, which is inert, instead of a half-written module that the next boot would try to
import. The previous version is kept as `.bak` by the same commit, so a bad-but-valid module can be
put back over the link without opening anything.

REBOOT IS REQUIRED for an installed module to take effect -- MicroPython caches imports, and the
running firmware holds the old code until it restarts. `push-commit` says so in its reply.

What this deliberately does NOT do: recover a board whose new module breaks the boot. The staging and
the checksum stop a CORRUPT file from ever being installed, but a module that is intact and wrong
will import and fail, and the board is then a USB recovery. Treat an OTA push as a real deployment,
not a scratchpad -- push what has been through `make test` on the bench board.
"""

import binascii
import os

try:
    import hashlib
except ImportError:  # no hashlib -> the digest gate cannot run, and begin() refuses the upload
    hashlib = None

import recorder

try:
    from micropython import const
except ImportError:  # CPython (tooling / off-board checks)
    from commons import const


_MAX_BYTES = const(262144)   # 256 KB: far above any module here, low enough to not fill the VFS
_STAGE_SUFFIX = '.ota'       # the staging sibling; inert to the boot, since nothing imports '*.ota'
_BACKUP_SUFFIX = '.bak'      # the outgoing version, kept so a bad push can be reversed over the link
_ALLOWED_SUFFIX = ('.mpy', '.py', '.config', '.creds')


def _safe_name(name: str) -> str:
    """
    Reject any filename that could write outside the board's module directory.

    The name arrives from the network, and `open()` on this VFS will follow whatever it is given --
    so a name carrying a separator or a parent reference is a write-anywhere primitive, not a module
    update. Only a bare leaf name with a known suffix is accepted.

    Args:
        name - the requested filename.

    Returns:
        The reason it was rejected, or None when it is safe to write.
    """
    if not name or len(name) > 64:
        return 'name missing or too long'
    if '/' in name or '\\' in name or name.startswith('.') or '..' in name:
        return 'name must be a bare filename (no path)'
    for suffix in _ALLOWED_SUFFIX:
        if name.endswith(suffix):
            return None
    return 'name must end in one of %s' % (', '.join(_ALLOWED_SUFFIX),)


class Upload:
    """
    The one in-progress upload.

    One at a time by design: two concurrent pushes over a single line-oriented link would interleave
    their chunks with no way to tell them apart, and there is no case for it -- the operator pushes a
    module, then pushes the next.
    """

    def __init__(self):
        self.reset()

    def reset(self) -> None:
        """Drop any in-progress upload (also the `push-abort` path); the staged file is left to be overwritten."""
        self.name: str = None
        self.size: int = 0
        self.sha: str = ''
        self.received: int = 0
        self.next_seq: int = 0
        self._digest = None

    def begin(self, name: str, size: int, sha: str) -> str:
        """
        Open staging for a new upload, replacing any upload already in progress.

        Args:
            name - the destination filename (a bare leaf, checked here).
            size - the total byte count the sender will send.
            sha - the SHA-256 hex digest of the complete file.

        Returns:
            None when staging is open, else the reason it was refused.
        """
        if hashlib is None:
            return 'no hashlib on this firmware -- refusing an unverifiable upload'
        bad = _safe_name(name)
        if bad is not None:
            return bad
        if size <= 0 or size > _MAX_BYTES:
            return 'size %d outside 1..%d' % (size, _MAX_BYTES)
        if len(sha) != 64:
            return 'sha must be the 64-char hex SHA-256 of the file'
        self.reset()
        self.name, self.size, self.sha = name, size, sha
        self._digest = hashlib.sha256()
        try:
            open(name + _STAGE_SUFFIX, 'wb').close()  # truncate any leftover staging from a dropped push
        except OSError as error:
            self.reset()
            return 'cannot open staging: %s' % error
        return None

    def chunk(self, seq: int, data: bytes) -> str:
        """
        Append one chunk, in order.

        Strictly sequential: a gap means a chunk was lost, and appending past it would produce a file
        that is the right LENGTH with a hole in it. The digest is computed as the bytes arrive, so a
        reordered or duplicated chunk fails the commit rather than silently corrupting the module.

        The staging file is reopened per chunk rather than held open across the transfer: an upload
        that is simply abandoned -- the operator walks away, the link drops -- must not leave a file
        handle open on a board that will not be rebooted before it flies.

        Args:
            seq - the chunk index, counting from 0.
            data - the raw chunk bytes.

        Returns:
            None when the chunk was appended, else the reason it was refused.
        """
        if self.name is None:
            return 'no upload in progress -- push-begin first'
        if seq != self.next_seq:
            return 'out of order: expected chunk %d, got %d' % (self.next_seq, seq)
        if self.received + len(data) > self.size:
            return 'overrun: %d bytes for a %d-byte file' % (self.received + len(data), self.size)
        try:
            with open(self.name + _STAGE_SUFFIX, 'ab') as handle:
                handle.write(data)
        except OSError as error:
            return 'staging write failed: %s' % error
        self._digest.update(data)
        self.received += len(data)
        self.next_seq += 1
        return None

    def commit(self) -> tuple:
        """
        Verify the staged file and install it, keeping the outgoing version as `.bak`.

        The digest is checked BEFORE anything is moved, so a truncated or corrupted transfer cannot
        reach the module the next boot imports.

        Args:
            (none)

        Returns:
            (info, None) once installed -- info carries the name, size and whether a backup was kept;
            (None, reason) when the upload was incomplete or the digest did not match.
        """
        if self.name is None:
            return None, 'no upload in progress'
        if self.received != self.size:
            return None, 'incomplete: %d of %d bytes' % (self.received, self.size)
        actual = binascii.hexlify(self._digest.digest()).decode()
        if actual != self.sha:
            self.reset()
            return None, 'sha mismatch: got %s, expected %s' % (actual[:16], self.sha[:16])
        name, staged, backup = self.name, self.name + _STAGE_SUFFIX, self.name + _BACKUP_SUFFIX
        kept = _replace(name, staged, backup)
        recorder.Recorder.log('ota', 'installed %s (%d bytes, sha %s) -- reboot to load it'
                                     % (name, self.size, self.sha[:12]))
        info = {'installed': name, 'bytes': self.size, 'sha': self.sha[:12],
                'backup': kept, 'reboot_required': True}
        self.reset()
        return info, None

    def status(self) -> dict:
        """What is staged right now, for the operator and for a resumed session to orient itself."""
        if self.name is None:
            return {'uploading': None}
        return {'uploading': self.name, 'received': self.received, 'size': self.size,
                'chunks': self.next_seq}


def _replace(name: str, staged: str, backup: str) -> bool:
    """
    Move the staged file over the live one, keeping the outgoing version.

    Same remove-then-rename fallback as commons.atomic_write_json, for a VFS that will not rename onto
    an existing file. The backup is best-effort: failing to keep the OLD module must not block
    installing the new one, since the caller can always push again.

    Args:
        name - the live module path.
        staged - the verified staging path.
        backup - where to keep the outgoing version.

    Returns:
        True when the previous version was kept as `.bak`.
    """
    kept = False
    try:
        os.stat(name)
    except OSError:
        os.rename(staged, name)  # nothing there to preserve: a new file
        return False
    try:
        try:
            os.remove(backup)
        except OSError:
            pass
        os.rename(name, backup)
        kept = True
    except OSError:
        pass  # could not keep a backup -- proceed, the new module is still verified
    try:
        os.rename(staged, name)
    except OSError:
        os.remove(name)
        os.rename(staged, name)
    return kept
