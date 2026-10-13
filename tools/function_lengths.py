#!/usr/bin/env python3
"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

The Rust avionics' function lengths (doc/specs/avionics-refactor.md, R4 and R5): every `fn` from its signature line to
its closing brace, measured on the code alone (tools/gen_avionics_docs.py `blank`: a `{` inside a format string or a
comment is no brace). Test code (`#[cfg(test)]` modules, `tests/`) and the bench binaries (`src/bin/`) are not
measured.

    python3 tools/function_lengths.py                      # the firmware's functions over 60 lines, longest first
    python3 tools/function_lengths.py --over 80 src/avionics/crates
    python3 tools/function_lengths.py --check              # exit 1 on a firmware function over 80 lines that the
                                                           # allow-list (tools/function_lengths.allow) does not name
"""

import argparse
import glob
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gen_avionics_docs import AVIONICS, blank  # noqa: E402

FIRMWARE: str = os.path.join(AVIONICS, 'firmware', 'src')
ALLOW: str = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'function_lengths.allow')
LIMIT: int = 80
_FN: re.Pattern = re.compile(r'\bfn\s+(\w+)')
_TEST_MODULE: re.Pattern = re.compile(r'#\[cfg\(test\)\]\s*mod\s+\w+\s*\{')


def _closing(code: str, start: int) -> int:
    """
    The index of the brace closing the one at `start`.

    Args:
        code - blanked source text.
        start - the index of an opening brace.

    Returns:
        The index of its closing brace, or the text's length when it never closes.
    """
    depth = 0
    for index in range(start, len(code)):
        if code[index] == '{':
            depth += 1
        elif code[index] == '}':
            depth -= 1
            if depth == 0:
                return index
    return len(code)


def functions(text: str) -> list:
    """
    Every function in one Rust source file, test modules left out.

    Args:
        text - the file's text.

    Returns:
        (name, first line, length in lines) for each function with a body, in file order; lines count from 1.
    """
    code = blank(text)
    skipped = [(match.end() - 1, _closing(code, match.end() - 1)) for match in _TEST_MODULE.finditer(code)]
    found = []
    for match in _FN.finditer(code):
        if any(begin <= match.start() <= end for begin, end in skipped):
            continue
        # the body's brace: the first `{` after the signature, unless a `;` ends it first (a trait's declaration)
        body = code.find('{', match.end())
        semicolon = code.find(';', match.end())
        if body < 0 or 0 <= semicolon < body:
            continue
        end = _closing(code, body)
        first = code.count('\n', 0, match.start()) + 1
        last = code.count('\n', 0, end) + 1
        found.append((match.group(1), first, last - first + 1))
    return found


def measure(root: str) -> list:
    """
    Every function under `root`, the bench binaries and integration tests left out.

    Args:
        root - a directory of Rust sources.

    Returns:
        (path relative to src/avionics, name, first line, length) for each, longest first.
    """
    out = []
    for path in sorted(glob.glob(os.path.join(root, '**', '*.rs'), recursive=True)):
        relative = os.path.relpath(path, AVIONICS)
        if f'{os.sep}bin{os.sep}' in relative or f'{os.sep}tests{os.sep}' in relative or relative.endswith('tests.rs'):
            continue
        with open(path, encoding='utf-8') as handle:
            text = handle.read()
        out.extend((relative, name, first, length) for name, first, length in functions(text))
    return sorted(out, key=lambda item: -item[3])


def allowed(path: str = ALLOW) -> set:
    """
    The allow-list: `<path relative to src/avionics>::<function>` per line, each with its reason after a `#`.

    Args:
        path - the allow-list file.

    Returns:
        The allowed functions; empty when the file is missing.
    """
    if not os.path.exists(path):
        return set()
    with open(path, encoding='utf-8') as handle:
        return {line.split('#', 1)[0].strip() for line in handle if line.split('#', 1)[0].strip()}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[1])
    parser.add_argument('root', nargs='?', default=FIRMWARE, help='the sources to measure (default: the firmware)')
    parser.add_argument('--over', type=int, default=60, help='list the functions longer than this')
    parser.add_argument('--check', action='store_true', help=f'fail on a function over {LIMIT} lines not allowed')
    arguments = parser.parse_args()
    measured = measure(arguments.root)
    if arguments.check:
        allow = allowed()
        long = [item for item in measured if item[3] > LIMIT and f'{item[0]}::{item[1]}' not in allow]
        for path, name, first, length in long:
            print(f'{path}:{first} {name}: {length} lines (over {LIMIT}; split it, or allow it with its reason)')
        stale = allow - {f'{path}::{name}' for path, name, _, length in measured if length > LIMIT}
        for entry in sorted(stale):
            print(f'{entry}: allowed but no longer over {LIMIT} lines -- drop it from {os.path.basename(ALLOW)}')
        return 1 if long or stale else 0
    over = [item for item in measured if item[3] > arguments.over]
    for path, name, first, length in over:
        print(f'{length:4d}  {path}:{first} {name}')
    print(f'{len(measured)} functions, {len(over)} over {arguments.over} lines')
    return 0


if __name__ == '__main__':
    sys.exit(main())
