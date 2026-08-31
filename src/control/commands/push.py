"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

`push <board> <file> [name]` -- send a module to a board over WiFi and install it.

The counterpart to the board's push-begin / push / push-commit. Reads a local file (a .mpy from
tools/deploy.sh, a .config, a .creds), carries it over the link the board already holds, and installs
it only once the board has confirmed the SHA-256 of what landed. Nothing on the board is touched
until that check passes, so a link that drops mid-push costs a retry and nothing else.

Deploying by USB means opening the airframe; this does not. It is still a real deployment -- push
what has been through `make test` on the bench board, and reboot the board afterwards, because
MicroPython holds the old module until it restarts.
"""

import base64
import hashlib
import json
import os

from . import command

_CHUNK = 256      # raw bytes per chunk -> ~344-char base64 lines, comfortable for readline()
_PROGRESS_EVERY = 8   # log a progress line every N chunks, so a big push is not silent


@command('push', 'push a module file to a board over WiFi and install it (push <board> <file> [name])')
async def push_command(hub, tokens, session) -> list:
    if len(tokens) < 2:
        return ['from cc err badargs push <board> <file> [name]']
    """
    Three-or-more tokens is always `push <board> <file> [name]`; two is `push <file>` against the
    selected board.

    Resolving tokens[1] by whether it HAPPENS to name a live board reads better until someone
    mistypes the board -- then the name is silently taken as a filename and the reply is
    'push-needs-a-board', which points at the wrong thing entirely. The arity is unambiguous, so
    it decides, and a bad board name is reported as a bad board name.
    """
    if len(tokens) >= 3:
        target, path = tokens[1], tokens[2]
        remote = tokens[3] if len(tokens) >= 4 else os.path.basename(path)
    else:
        target, path = session.get('selected'), tokens[1]
        remote = os.path.basename(path)
    if not target:
        return ['from cc err badargs push-needs-a-board']
    board = hub.boards.get(target)
    if board is None or not board.online:
        return ['from cc err noboard %s' % target]
    try:
        with open(path, 'rb') as handle:
            blob = handle.read()
    except OSError as error:
        return ['from cc err badargs cannot read %s: %s' % (path, error)]
    if not blob:
        return ['from cc err badargs %s is empty' % path]

    digest = hashlib.sha256(blob).hexdigest()
    total = (len(blob) + _CHUNK - 1) // _CHUNK
    hub.log('push %s -> %s:%s (%d bytes, %d chunks, sha %s)'
            % (path, target, remote, len(blob), total, digest[:12]))

    started = await board.command('push-begin', remote, str(len(blob)), digest)
    if started is None:
        return ['from cc err offline %s' % target]
    if started.command != 'ok':
        return ['from cc err push-begin %s' % ' '.join(str(arg) for arg in started.args)]

    for seq in range(total):
        piece = blob[seq * _CHUNK:(seq + 1) * _CHUNK]
        # unpadded base64: '=' is the one character of the alphabet the protocol would misread, as a
        # key=value separator; length alone determines the padding, so the board restores it exactly
        token = base64.b64encode(piece).decode().rstrip('=')
        sent = await board.command('push', str(seq), token, quiet=True)
        if sent is None:
            return ['from cc err offline %s (chunk %d of %d -- nothing was installed)'
                    % (target, seq, total)]
        if sent.command != 'ok':
            # leave no half-transfer behind: the board would otherwise refuse the next begin's
            # sequence numbers until something reset it
            await board.command('push-abort', quiet=True)
            return ['from cc err push %s (chunk %d of %d -- nothing was installed)'
                    % (' '.join(str(arg) for arg in sent.args), seq, total)]
        if seq and seq % _PROGRESS_EVERY == 0:
            hub.log('  push %s: %d/%d chunks' % (remote, seq, total))

    # restate what is being finished: a commit that reaches the wrong open transfer must fail loudly
    installed = await board.command('push-commit', remote, digest)
    if installed is None:
        return ['from cc err offline %s (during commit -- nothing was installed)' % target]
    if installed.command != 'ok':
        return ['from cc err push-commit %s' % ' '.join(str(arg) for arg in installed.args)]
    result = json.loads(installed.args[0]) if installed.args else {}
    result['from'] = path
    hub.log('push %s -> %s:%s INSTALLED -- reboot the board to load it' % (path, target, remote))
    return ['from cc ok %s' % json.dumps(result)]
