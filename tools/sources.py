"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

The source tree, as the doc generators see it. `gen_docs`, `gen_graph` and `gen_schema` each walked
src/glider with their own ROOT, their own directory list and their own skip set -- so a new module (or
a new package directory) had to be taught to all three independently, and the three skip sets had
already drifted apart into entries that matched nothing. This module is the single answer to "which
files are the sources?"; the generators keep only what genuinely differs between them (what they parse
out of each file, and which trees they care about).

Parsing, never importing: the firmware imports machine/network/esp32, which do not exist on the host,
so every generator reads the sources with `ast` instead. parse() is that shared step.
"""

import ast
import os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
GLIDER_DIRS: tuple = ('src/glider', 'src/glider/drivers', 'src/glider/tasks')
CONTROL_DIRS: tuple = ('src/control', 'src/control/commands')
"""
Not sources for documentation purposes: tests and benchmarks (they document themselves), package
markers, the generated version stamp, and the generators themselves. One set, because the three
generators' sets were the same set with different dead entries -- gen_docs skipped 'gen_docs' and
gen_graph skipped all of 'gen_', but neither ever scans tools/, so neither prefix could match.
"""
SKIP_PREFIXES: tuple = ('test_', 'itest_', 'bench_', 'example_', 'gen_', '__init__', 'version')


def modules(directories, skip_prefixes: tuple = SKIP_PREFIXES):
    """
    Walk `directories` and yield the source files in them, in a stable order.

    SYMLINKS ARE SKIPPED, deliberately: cc_protocol.py is one file symlinked into src/control so the
    hub and the firmware share it, and following it would document the same module twice under two
    names. Directories are repo-relative so a generator can print the path it read.

    Args:
        directories - repo-relative directory paths (e.g. GLIDER_DIRS).
        skip_prefixes - filename prefixes to leave out; pass () to take every .py.

    Returns:
        Yields (relative directory, filename, absolute path) per source file.
    """
    for relative in directories:
        directory = os.path.join(ROOT, relative)
        if not os.path.isdir(directory):
            continue
        for name in sorted(os.listdir(directory)):
            if not name.endswith('.py') or name.startswith(skip_prefixes):
                continue
            path = os.path.join(directory, name)
            if not os.path.islink(path):
                yield relative, name, path


def parse(path: str) -> ast.Module:
    """
    The parsed syntax tree of one source file.

    Args:
        path - absolute path to a .py file.

    Returns:
        Its ast.Module; the filename is carried into the tree so a SyntaxError names the file.
    """
    with open(path, 'r') as handle:
        return ast.parse(handle.read(), filename=path)
