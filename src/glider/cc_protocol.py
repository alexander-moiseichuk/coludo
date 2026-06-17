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
    def __init__(self, command, args, named, line):
        self.command = command  # first token, lowercased (None for an empty line)
        self.args = args  # positional values (board-id is args[0] by convention)
        self.named = named  # dict of key=value params
        self.line = line

    @property
    def board(self):
        """The board-id by convention (first positional). None for whoami/list/etc."""
        return self.args[0] if self.args else None

    @property
    def params(self):
        """Positional params after the board-id."""
        return self.args[1:]

    def __repr__(self):
        return '_Msg(%r, args=%r, named=%r)' % (self.command, self.args, self.named)


def _is_simple(s):
    if not s or s[: len(_PREFIX)] == _PREFIX:
        return False
    for c in s:
        if c not in _SAFE:
            return False
    return True


def encode(v):
    """Encode a value into one whitespace-free wire token."""
    if isinstance(v, bool):
        return 'true' if v else 'false'
    if isinstance(v, int):
        return str(v)
    s = v if isinstance(v, str) else str(v)
    if _is_simple(s):
        return s
    return _PREFIX + binascii.b2a_base64(s.encode()).rstrip().decode()


def decode(tok):
    """Decode a wire token back to a str (base64-decoded if prefixed, else as-is)."""
    if tok[: len(_PREFIX)] == _PREFIX:
        return binascii.a2b_base64(tok[len(_PREFIX) :]).decode()
    return tok


def parse(line):
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


def build(command, args=(), named=None):
    """Build a protocol line; values are encoded as needed."""
    parts = [command]
    for a in args:
        parts.append(encode(a))
    if named:
        for k in named:
            parts.append('%s=%s' % (k, encode(named[k])))
    return ' '.join(parts)
