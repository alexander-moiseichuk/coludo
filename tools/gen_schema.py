"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Generate doc/telemetry.md — the TELEMETRY SCHEMA contract — by parsing the sources with `ast`.

Stream names and field lists were declared ad-hoc across ~17 `recorder.Telemetry(...)` sites, documented
nowhere, and rediscovered by every tool through field names (findings §27.14). §27.1 was the first bill
for that: the sim recorded a fused `fins.csv` while a real board records one stream per servo, so every
fin-aware tool came back empty on a board capture and nothing could have told us.

Parses (never imports) the board tree, the HITL task and the host sim, so `machine`/`network` being
absent on the host is irrelevant — the same approach gen_graph uses.

  python3 tools/gen_schema.py           # write doc/telemetry.md
  python3 tools/gen_schema.py --check   # fail when it is stale (the local gate runs this)
"""

import argparse
import ast
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.join(_HERE, '..')
_DOC = os.path.join(_ROOT, 'doc', 'telemetry.md')
_SOURCES = (
    ('board', os.path.join(_ROOT, 'src', 'glider')),
    ('board', os.path.join(_ROOT, 'src', 'glider', 'drivers')),
    ('board', os.path.join(_ROOT, 'src', 'glider', 'tasks')),
    ('host sim', os.path.join(_ROOT, 'tools')),
)


def _literal(node):
    """A source-level constant, or None when the expression is not a plain literal."""
    try:
        return ast.literal_eval(node)
    except (ValueError, SyntaxError, TypeError):
        return None


def _streams_in(path: str) -> list:
    """
    Every `Telemetry(name, fields)` construction in one file.

    Returns:
        [(stream_name, [field, ...] | None)]; the name is None when it is built at runtime
        (e.g. '%s.csv' % self.name -- a per-device stream whose name comes from the config).
    """
    try:
        with open(path) as handle:
            tree = ast.parse(handle.read(), filename=path)
    except (OSError, SyntaxError):
        return []
    found = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        target = node.func
        name = target.attr if isinstance(target, ast.Attribute) else getattr(target, 'id', '')
        if name != 'Telemetry' or not node.args:
            continue
        stream = _literal(node.args[0])
        fields = _literal(node.args[1]) if len(node.args) > 1 else None
        if isinstance(fields, (list, tuple)):
            fields = [str(field) for field in fields]
        elif fields is not None:
            fields = None
        found.append((stream, fields))
    return found


def collect() -> list:
    """(origin, module, stream, fields) for every declared telemetry stream, sorted for a stable doc."""
    rows = []
    for origin, directory in _SOURCES:
        if not os.path.isdir(directory):
            continue
        for entry in sorted(os.listdir(directory)):
            if not entry.endswith('.py'):
                continue
            path = os.path.join(directory, entry)
            for stream, fields in _streams_in(path):
                rows.append((origin, entry[:-3], stream, fields))
    return sorted(rows, key=lambda row: (row[2] or '~runtime', row[1]))


def render(rows: list) -> str:
    """The doc text."""
    out = ['# Telemetry schema',
           '',
           '> **GENERATED from the sources by `tools/gen_schema.py` — do not hand-edit.** Regenerate '
           'after changing any `recorder.Telemetry(...)` declaration (`python3 tools/gen_schema.py`); '
           '`--check` fails the local gate if it is stale.',
           '',
           'Every stream a capture can contain, and the fields in each. A recorder capture interleaves '
           '`@<session>_<file>@<row>` telemetry rows with plain log lines; `tools/flight_telemetry.py` '
           'demuxes them, and every renderer resolves streams **by role** (the fields they carry) rather '
           'than by file name — a capture\'s file names track the fitted hardware, so a fallback flight '
           'names them differently.',
           '',
           '## Streams',
           '',
           '| stream | origin | declared in | fields |',
           '|---|---|---|---|']
    for origin, module, stream, fields in rows:
        name = '`%s`' % stream if stream else '_per-device_ (`<name>.csv`)'
        columns = ', '.join('`%s`' % field for field in fields) if fields else '_runtime_'
        out.append('| %s | %s | `%s.py` | %s |' % (name, origin, module, columns))
    out += [
        '',
        '## Shapes that differ between the sim and the board',
        '',
        'These are the traps — a renderer written against one shape silently finds nothing in the other:',
        '',
        '- **Fins.** The HITL sim records ONE fused `fins.csv` with a column per surface; a real board '
        'records one stream PER SERVO (`servo_<surface>.csv`, column `angle`, from `drivers/sg90.py`). '
        '`flight_telemetry` rebuilds the fused shape from the per-servo streams when a capture has none, '
        'forward-filling each surface (a servo holds its last commanded angle), so both render '
        'identically — including on captures recorded before that existed.',
        '- **Per-device streams.** Any `Telemetry(\'%s.csv\' % self.name, ...)` takes its file name from '
        'the CONFIG, so the same driver appears under whatever the board named it.',
        '',
        '## Contract',
        '',
        '`tools/preflight.py` gates on this: it parses the sim\'s own capture header and requires every '
        'stream a renderer depends on to still resolve. A rename that would break the tools fails the '
        'gate first — that is the standing guard for findings §27.1.',
        '']
    return '\n'.join(out)


def main() -> int:
    parser = argparse.ArgumentParser(description='Generate the telemetry schema doc from the sources.')
    parser.add_argument('--check', action='store_true', help='fail when doc/telemetry.md is stale')
    args = parser.parse_args()
    text = render(collect())
    if args.check:
        existing = ''
        if os.path.exists(_DOC):
            with open(_DOC) as handle:
                existing = handle.read()
        if existing != text:
            print('doc/telemetry.md is STALE -- run: python3 tools/gen_schema.py', file=sys.stderr)
            return 1
        print('telemetry schema doc up to date')
        return 0
    with open(_DOC, 'w') as handle:
        handle.write(text)
    print('wrote', os.path.relpath(_DOC, _ROOT))
    return 0


if __name__ == '__main__':
    sys.exit(main())
