# CC <-> board line protocol (specs/cc-protocol.md).
#
# One newline-delimited message per line:  <command> <board-id> [params...]
# Tokens are whitespace-separated, so there is NO quoting or escaping. A param value is one of:
#   * bare token    -> a simple value with no spaces (e.g. 3000, glider1, 192.168.10.1)
#   * base64:<data> -> anything else: spaces, quotes, JSON, binary
# Both sides know each command's schema, so the parser does not guess types: a bare token is
# returned as a str and the receiver converts numerics itself (it knows `ms` is an int). Named
# params are key=value; everything else is positional. The command is lowercased; values keep
# their case. parse() handles requests and responses (ok/err/pong/iam) alike.

import binascii

_PREFIX = 'base64:'
_SAFE = 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789._-/+:'


class _Msg:
    # A board receives `command params` (Control has stripped the routing board id), so args are
    # the positional params and named are key=value params.
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
    for c in s:
        if c not in _SAFE:
            return False
    return True


def encode(v) -> str:
    """Encode a value into one whitespace-free wire token."""
    if isinstance(v, bool):
        return 'true' if v else 'false'
    if isinstance(v, int):
        return str(v)
    s = v if isinstance(v, str) else str(v)
    if _is_simple(s):
        return s
    return _PREFIX + binascii.b2a_base64(s.encode()).rstrip().decode()


def decode(tok: str) -> str:
    """Decode a wire token back to a str (base64-decoded if prefixed, else as-is)."""
    if tok[: len(_PREFIX)] == _PREFIX:
        return binascii.a2b_base64(tok[len(_PREFIX) :]).decode()
    return tok


def parse(line: str) -> _Msg:
    """Parse a protocol line into a _Msg (works for requests and responses)."""
    toks = line.split()
    if not toks:
        return _Msg(None, [], {}, line)
    command = toks[0].lower()
    args = []
    named = {}
    for t in toks[1:]:
        if t[: len(_PREFIX)] == _PREFIX:
            args.append(decode(t))  # encoded positional (may contain '=')
        else:
            eq = t.find('=')
            if eq > 0:
                named[t[:eq]] = decode(t[eq + 1 :])
            else:
                args.append(decode(t))
    return _Msg(command, args, named, line)


def build(command: str, args=(), named=None) -> str:
    """Build a protocol line; values are encoded as needed."""
    parts = [command]
    for a in args:
        parts.append(encode(a))
    if named:
        for k in named:
            parts.append('%s=%s' % (k, encode(named[k])))
    return ' '.join(parts)
