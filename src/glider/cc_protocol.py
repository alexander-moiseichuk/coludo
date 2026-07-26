"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

CC <-> board line protocol (doc/specs/cc-protocol.md).

One newline-delimited message per line:  <command> <board-id> [params...]. Tokens are
whitespace-separated, so there is NO quoting or escaping. A param value is one of:
  * bare token    -> a simple value with no spaces (e.g. 3000, taster, 192.168.10.1)
  * base64:<data> -> anything else: spaces, quotes, JSON, binary
Both sides know each command's schema, so the parser does not guess types: a bare token is returned
as a str and the receiver converts numerics itself (it knows `ms` is an int). Named params are
key=value; everything else is positional. The command is lowercased; values keep their case. parse()
handles requests and responses (ok/err/pong/iam) alike.
"""

import binascii

_PREFIX = 'base64:'
_SAFE = 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789._-/+:'


class _Msg:
    """
    A parsed protocol message (request or response).

    A board receives `command params` (Control has stripped the routing board id), so args are the
    positional params and named are the key=value params.
    """

    def __init__(self, command, args: list, named: dict, line: str):
        self.command = command  # first token, lowercased (None for an empty line)
        self.args: list = args  # positional params
        self.named: dict = named  # dict of key=value params
        self.line: str = line

    def __repr__(self) -> str:
        return '_Msg(%r, args=%r, named=%r)' % (self.command, self.args, self.named)


def _is_simple(s: str) -> bool:
    if not s or s[: len(_PREFIX)] == _PREFIX:
        return False
    for char in s:
        if char not in _SAFE:
            return False
    return True


def encode(v) -> str:
    """
    Encode a value into one whitespace-free wire token.

    Args:
        v - the value to encode (bool / int / str / other via str()).

    Returns:
        The wire token: bare when already safe, else 'base64:'-prefixed.
    """
    if isinstance(v, bool):
        return 'true' if v else 'false'
    if isinstance(v, int):
        return str(v)
    text = v if isinstance(v, str) else str(v)
    if _is_simple(text):
        return text
    return _PREFIX + binascii.b2a_base64(text.encode()).rstrip().decode()


def decode(tok: str) -> str:
    """
    Decode a wire token back to a str.

    Args:
        tok - the wire token.

    Returns:
        The decoded string (base64-decoded when 'base64:'-prefixed, else the token as-is).
    """
    if tok[: len(_PREFIX)] == _PREFIX:
        return binascii.a2b_base64(tok[len(_PREFIX) :]).decode()
    return tok


def parse(line: str) -> _Msg:
    """
    Parse a protocol line into a _Msg (works for requests and responses).

    Args:
        line - the raw newline-stripped protocol line.

    Returns:
        A _Msg; its command is None for an empty line.
    """
    toks = line.split()
    if not toks:
        return _Msg(None, [], {}, line)
    command = toks[0].lower()
    args = []
    named = {}
    for token in toks[1:]:
        if token[: len(_PREFIX)] == _PREFIX:
            args.append(decode(token))  # encoded positional (may contain '=')
        else:
            eq_at = token.find('=')
            if eq_at > 0:
                named[token[:eq_at]] = decode(token[eq_at + 1 :])
            else:
                args.append(decode(token))
    return _Msg(command, args, named, line)


def build(command: str, args=(), named=None) -> str:
    """
    Build a protocol line from a command and its params.

    Args:
        command - the command (or response) keyword.
        args - positional param values, encoded as needed.
        named - key -> value params, emitted as key=value (encoded), or None.

    Returns:
        The assembled single-line protocol string.
    """
    parts = [command]
    for value in args:
        parts.append(encode(value))
    if named:
        for key in named:
            parts.append('%s=%s' % (key, encode(named[key])))
    return ' '.join(parts)
