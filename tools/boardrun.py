#!/usr/bin/env python3
"""Minimal pyserial driver for the MicroPython REPL -- a reliable, non-interactive alternative to
mpremote/rshell for the ESP32-P4 glider board.

WHY: in a non-interactive shell (no controlling TTY) mpremote's raw-paste path intermittently wedges
the CDC into uninterruptible D-state, and rshell's `repl` needs a TTY (termios). This driver speaks
the plain REPL **paste mode** the way rshell does for a human -- small, paced writes, no raw-paste --
which is reliable here. It needs no TTY and never raw-pastes.

Usage:
  boardrun.py PORT reset                 # Ctrl-D soft-reboot, print the boot banner
  boardrun.py PORT eval  "<expr>"        # print(repr(<expr>))
  boardrun.py PORT exec  "<statements>"  # run statements as-is
  boardrun.py PORT runfile "<board_path>" [timeout]   # soft-reset, then exec(open(path).read())
  boardrun.py PORT mkdir "<board_dir>"   # create a board directory (ok if it exists)
  boardrun.py PORT put "<local>" "<board_path>"   # upload a file (base64 over paste mode)

Exit: 0 on success; 1 if the captured output contains a Traceback / error, or on timeout.
"""
import base64
import sys
import time

import serial

_PROMPT = b'>>> '
_BAUD = 115200


def _drain(port, quiet=0.25):
    """Read until the board has been quiet for `quiet` seconds; return what was read."""
    end = time.time() + quiet
    buffer = b''
    while time.time() < end:
        waiting = port.in_waiting
        if waiting:
            buffer += port.read(waiting)
            end = time.time() + quiet
        else:
            time.sleep(0.01)
    return buffer


def _read_until(port, marker, timeout):
    """Read until `marker` appears or `timeout` elapses; return (data, found)."""
    end = time.time() + timeout
    buffer = b''
    while time.time() < end:
        waiting = port.in_waiting
        if waiting:
            buffer += port.read(waiting)
            if marker in buffer:
                return buffer, True
        else:
            time.sleep(0.01)
    return buffer, False


def _interrupt(port):
    """Break any running program and get a clean prompt."""
    port.write(b'\r\x03\x03')  # Enter + Ctrl-C twice
    _drain(port, 0.2)


def _ensure_prompt(port, timeout=8):
    """Get the board to a known idle '>>> ' prompt, tolerating a reboot. pyserial asserts DTR/RTS on
    open, which reboots this board; firing the paste sequence before it settles was the intermittent
    failure. Send Ctrl-C and wait for the prompt, retrying until `timeout` -- catches both a running
    program and a still-booting board."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        port.write(b'\r\x03')  # Enter + Ctrl-C
        _, found = _read_until(port, _PROMPT, 0.5)
        if found:
            _drain(port, 0.1)
            return True
    return False


def _soft_reset(port):
    """Ctrl-D soft-reboot (clears all imported modules -> fresh state); return the boot output."""
    port.write(b'\r\x04')
    time.sleep(0.3)
    buffer, _ = _read_until(port, _PROMPT, 8)
    return buffer


def _write_paced(port, data, chunk=128):
    """Write `data` in small chunks, draining the board's echo between them. Pacing keeps the host USB
    write buffer from filling faster than the board drains it -- an over-full buffer blocks write() in
    uninterruptible D-state (un-killable), the failure mode that wedged the bulk-transfer path."""
    for start in range(0, len(data), chunk):
        port.write(data[start:start + chunk])
        port.flush()
        time.sleep(0.004)
        pending = port.in_waiting
        if pending:
            port.read(pending)  # discard the paste-mode echo; real output comes after Ctrl-D


def _paste_exec(port, code, timeout):
    """Run `code` via REPL paste mode (paced writes); return (output, prompt_returned)."""
    _ensure_prompt(port)
    port.write(b'\x05')  # Ctrl-E -> enter paste mode
    _read_until(port, b'===', 3)
    _write_paced(port, code.encode() + b'\r')
    port.write(b'\x04')  # Ctrl-D -> execute the pasted block
    return _read_until(port, _PROMPT, timeout)


def _put(port, local_path, board_path):
    """Upload one local file to the board (base64 over paste mode); return True on success."""
    with open(local_path, 'rb') as handle:
        blob = base64.b64encode(handle.read()).decode()
    code = ("import ubinascii as _u\n_f=open('%s','wb')\n"
            "_f.write(_u.a2b_base64('%s'))\n_f.close()\nprint('PUT_OK')" % (board_path, blob))
    output, returned = _paste_exec(port, code, 30)
    text = output.decode('utf-8', 'replace')
    if returned and 'PUT_OK' in text and 'Traceback' not in text:
        return True
    sys.stderr.write('put %s FAILED: %s\n' % (board_path, text[-200:]))
    return False


def main(argv):
    """Dispatch a single board command; return the process exit code."""
    if len(argv) < 3:
        sys.stderr.write(__doc__)
        return 2
    port_name, command = argv[1], argv[2]
    # open with DTR/RTS DEASSERTED: pyserial asserts both by default, which on this board's CH343
    # drives EN/IO0 -> a reset (and risks the download stub = no REPL). Setting them before open
    # keeps the board running its REPL across our connect -- the intermittent-hang fix.
    port = serial.Serial()
    port.port = port_name
    port.baudrate = _BAUD
    port.timeout = 0.1
    port.dtr = False
    port.rts = False
    port.open()
    try:
        _ensure_prompt(port)  # tolerate the DTR/RTS reboot that pyserial's open triggers
        if command == 'reset':
            sys.stdout.write(_soft_reset(port).decode('utf-8', 'replace'))
            return 0
        if command == 'eval':
            code = 'print(repr(%s))' % argv[3]
        elif command == 'exec':
            code = argv[3]
        elif command == 'runfile':
            _soft_reset(port)  # isolate each run with a fresh module state
            code = 'exec(open("%s").read())' % argv[3]
        elif command == 'mkdir':
            code = ("import os\ntry:\n os.mkdir('%s')\nexcept OSError:\n pass\nprint('MKDIR_OK')"
                    % argv[3])
            output, returned = _paste_exec(port, code, 15)
            text = output.decode('utf-8', 'replace')
            return 0 if (returned and 'MKDIR_OK' in text) else 1
        elif command == 'put':
            return 0 if _put(port, argv[3], argv[4]) else 1
        elif command == 'putmany':
            # one port session for a whole tree: manifest lines are "<local>\t<board_path>"
            with open(argv[3]) as manifest:
                pairs = [line.rstrip('\n').split('\t') for line in manifest if line.strip()]
            for board_dir in sorted({r.rsplit('/', 1)[0] for _, r in pairs if '/' in r}):
                _paste_exec(port, "import os\ntry:\n os.mkdir('%s')\nexcept OSError:\n pass" % board_dir, 15)
            good = 0
            for index, (local, remote) in enumerate(pairs, 1):
                ok = _put(port, local, remote)
                good += ok
                sys.stdout.write('  [%d/%d] %s %s\n' % (index, len(pairs), 'ok ' if ok else 'ERR', remote))
                sys.stdout.flush()
            failed = len(pairs) - good
            sys.stdout.write('putmany: %d ok, %d failed\n' % (good, failed))
            return 0 if failed == 0 else 1
        else:
            sys.stderr.write('unknown command: %s\n' % command)
            return 2
        timeout = int(argv[4]) if len(argv) > 4 else 60
        output, returned = _paste_exec(port, code, timeout)
        text = output.decode('utf-8', 'replace')
        sys.stdout.write(text)
        if not returned:
            sys.stderr.write('\n[boardrun: TIMEOUT after %ds]\n' % timeout)
            return 1
        failed = ('Traceback' in text) or ('FAIL' in text) or ('Error' in text and 'ok:' not in text)
        return 1 if failed else 0
    finally:
        port.close()


if __name__ == '__main__':
    sys.exit(main(sys.argv))
