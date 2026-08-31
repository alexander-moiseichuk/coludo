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


def _safe_path(path: str) -> str:
    """
    Reject any destination that could write outside the board's own module tree.

    The path arrives from the network and `open()` follows whatever it is given, so this is the only
    thing standing between a module update and a write-anywhere primitive. What it must still ALLOW is
    a subdirectory: `drivers/`, `tasks/` and `test/` are where most of the firmware lives, and a rule
    that accepted only bare leaf names could not update any of it.

    So: relative only, no `..` in any component, no component starting with a dot, and a known suffix.
    That admits `drivers/bno055.mpy` and refuses `../main.py`, `/etc/passwd` and `a/../../x.mpy`.

    Args:
        path - the requested destination path, relative to the board's working directory.

    Returns:
        The reason it was rejected, or None when it is safe to write.
    """
    if not path or len(path) > 96:
        return 'path missing or too long'
    if path[0] == '/' or '\\' in path:
        return 'path must be relative (no leading / and no backslashes)'
    parts = path.split('/')
    for part in parts:
        if not part or part == '..' or part[0] == '.':
            return 'path must not contain empty, dotted or parent components'
    for suffix in _ALLOWED_SUFFIX:
        if path.endswith(suffix):
            return None
    return 'path must end in one of %s' % (', '.join(_ALLOWED_SUFFIX),)


class Upload:
    """
    The one in-progress upload.

    One at a time by design: two concurrent pushes over a single line-oriented link would interleave
    their chunks with no way to tell them apart, and there is no case for it -- the operator pushes a
    module, then pushes the next.
    """

    def __init__(self):
        self.reset()

    def discard(self) -> None:
        """Remove the staging file and forget the upload -- the failure path, so nothing is left behind."""
        if self.name is not None:
            try:
                os.remove(self.name + _STAGE_SUFFIX)
            except OSError:
                pass  # never created, or already gone: either way there is nothing to clean up
        self.reset()

    def reset(self) -> None:
        """Forget any in-progress upload (the `push-abort` path); see discard() to also drop the staging file."""
        self.name: str = None
        self.size: int = 0
        self.sha: str = ''
        self.received: int = 0
        self.next_seq: int = 0
        self._digest = None

    def begin(self, name: str, size: int, sha: str) -> str:
        """
        Open staging for a new upload, DISCARDING any upload already in progress.

        Discarding rather than merely forgetting: an operator who starts one push and then starts a
        different one -- a typo, a change of mind, a retry under another name -- would otherwise strand
        the first file's `.ota` on the flash with nothing that ever removes it. On a board with a few
        megabytes and no shell, a leak with no reclaim path is the kind that is only noticed when a
        write fails much later, for an unrelated reason.

        Args:
            name - the destination path, relative to the board's working directory (checked here).
            size - the total byte count the sender will send.
            sha - the SHA-256 hex digest of the complete file.

        Returns:
            None when staging is open, else the reason it was refused.
        """
        if hashlib is None:
            return 'no hashlib on this firmware -- refusing an unverifiable upload'
        bad = _safe_path(name)
        if bad is not None:
            return bad
        if size <= 0 or size > _MAX_BYTES:
            return 'size %d outside 1..%d' % (size, _MAX_BYTES)
        if len(sha) != 64:
            return 'sha must be the 64-char hex SHA-256 of the file'
        self.discard()  # a previous transfer's staging file must not outlive it
        self.name, self.size, self.sha = name, size, sha
        self._digest = hashlib.sha256()
        _make_parents(name)  # a push may be the first thing to put a module in a new package
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

    def commit(self, path: str = None, sha: str = None) -> tuple:
        """
        Verify the staged file and install it, keeping the outgoing version as `.bak`.

        The digest is checked BEFORE anything is moved, so a truncated or corrupted transfer cannot
        reach the module the next boot imports. A failed check DELETES the staging file: a board with
        a few megabytes of flash must not accumulate the debris of every abandoned push, and a stale
        `.ota` left next to a module is exactly the sort of thing a later reader mistakes for state.

        The commit restates the path and digest it believes it is finishing. They are optional so an
        operator can still type `push-commit` by hand, but when given they must match what the upload
        was opened with -- which is what turns a commit sent against the wrong transfer into an error
        rather than an install of whatever happened to be staged.

        Args:
            path - the destination the caller believes it is committing (optional, checked).
            sha - the digest the caller believes it sent (optional, checked).

        Returns:
            (info, None) once installed -- info carries the name, size and whether a backup was kept;
            (None, reason) when the upload was incomplete, mismatched, or did not verify.
        """
        if self.name is None:
            return None, 'no upload in progress'
        if path is not None and path != self.name:
            return None, 'commit is for %s, but %s is staged' % (path, self.name)
        if sha is not None and sha != self.sha:
            return None, 'commit digest does not match the one push-begin was given'
        if self.received != self.size:
            return None, 'incomplete: %d of %d bytes' % (self.received, self.size)
        actual = binascii.hexlify(self._digest.digest()).decode()
        if actual != self.sha:
            self.discard()
            return None, 'sha mismatch: got %s, expected %s -- staging discarded' % (
                actual[:16], self.sha[:16])
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


def orphans() -> list:
    """
    Staging files left behind by a transfer whose in-RAM state did not survive.

    A push interrupted by a REBOOT is the case nothing else cleans up: `discard` needs an upload in
    memory to know what to remove, and after a restart there is none -- while the `.ota` is still on
    the flash. Since the board has no shell, an orphan that cannot be listed is an orphan that cannot
    be removed, so this is what makes `sweep` usable rather than a guess.

    Scans the working directory and one level down, which is the whole of the module tree
    (`drivers/`, `tasks/`, `test/`); the staging suffix cannot occur deeper because a push cannot
    create directories.

    Args:
        (none)

    Returns:
        The paths of every staging file found, without the suffix stripped.
    """
    found = []
    for entry in _listdir(''):
        if entry.endswith(_STAGE_SUFFIX):
            found.append(entry)
            continue
        for nested in _listdir(entry):        # empty for a plain file, so this needs no isdir()
            if nested.endswith(_STAGE_SUFFIX):
                found.append('%s/%s' % (entry, nested))
    return found


def _listdir(path: str) -> list:
    """Entries of `path`, or empty when it is a file, is missing, or cannot be read."""
    try:
        return os.listdir(path) if path else os.listdir()
    except OSError:
        return []


def _make_parents(path: str) -> str:
    """
    Create the destination's parent directories, as `mkdir -p` would.

    A push to a directory the board does not have yet would otherwise fail at the staging open() with
    ENOENT, which reads as a broken transfer rather than a missing directory -- and there is no way to
    create one over the link, so the push could never succeed. That case arrives the first time a
    module is added to a package the running firmware predates.

    Safe only because _safe_path has already run: the path is relative with no `..`, so every
    directory made here is inside the module tree. Existing directories are not an error; a directory
    that cannot be made is left for the open() to report, since that carries the real errno.

    Args:
        path - the destination path, already checked.

    Returns:
        None (always) -- failures surface at the open() that follows.
    """
    parts = path.split('/')[:-1]
    built = ''
    for part in parts:
        built = part if not built else built + '/' + part
        try:
            os.mkdir(built)
        except OSError:
            pass  # already there, or cannot be made -- the staging open() reports the latter
    return None


def sweep(path: str) -> str:
    """
    Remove one staging file by PATH, whether or not an upload is open.

    The counterpart to discard(), which can only remove the staging of a transfer it still holds in
    memory. This addresses the file directly, so a `.ota` orphaned by a reboot -- the usual way a push
    goes stale -- can be cleared and the push restarted, on a board with no shell and, by assumption,
    no USB attached.

    Args:
        path - the destination path whose staging file should be removed.

    Returns:
        None when the staging file is gone, else the reason it was refused.
    """
    bad = _safe_path(path)
    if bad is not None:
        return bad
    try:
        os.remove(path + _STAGE_SUFFIX)
    except OSError:
        return 'no staging file for %s' % path
    return None


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
