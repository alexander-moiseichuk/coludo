#!/usr/bin/env python3
# Generate doc/api.md from the module sources by *parsing* them (stdlib `ast`) -- never importing,
# so it works for the firmware too (which imports machine/network/esp32, absent on the host) and
# needs no third-party tools. Module descriptions come from the leading `#` comment header; class,
# method and function descriptions from their docstrings. Public surface only (skips _-internals).
#
#   python3 tools/gen_docs.py        # writes doc/api.md

import ast
import os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SOURCES = [
    ('src/glider', 'glider firmware (MicroPython)'),
    ('src/glider/drivers', 'glider HAL drivers — `drivers/`'),
    ('src/glider/tasks', 'glider subsystem tasks — `tasks/`'),
    ('src/control', 'control (CPython)'),
    ('src/control/commands', 'control operator commands — `commands/`'),
]
SKIP_PREFIXES = ('test_', 'itest_', 'bench_', 'gen_docs', '__init__')


def module_header(source: str) -> str:
    """The leading block of `#` comment lines (our module header), as plain text."""
    lines = []
    for line in source.splitlines():
        stripped = line.strip()
        if stripped.startswith('#'):
            lines.append(stripped[1:].strip())
        elif stripped == '' and lines:
            break
        elif stripped == '':
            continue
        else:
            break
    return '\n'.join(lines).strip()


def signature(node: ast.AST) -> str:
    """Render a def's parameter list (dropping a leading self/cls) and return annotation."""
    args = node.args
    parts = []
    positional = list(getattr(args, 'posonlyargs', [])) + list(args.args)
    first_default = len(positional) - len(args.defaults)
    for index, arg in enumerate(positional):
        if index == 0 and arg.arg in ('self', 'cls'):
            continue
        piece = arg.arg
        if arg.annotation is not None:
            piece += ': ' + ast.unparse(arg.annotation)
        if index >= first_default:
            piece += '=' + ast.unparse(args.defaults[index - first_default])
        parts.append(piece)
    if args.vararg:
        parts.append('*' + args.vararg.arg)
    elif args.kwonlyargs:
        parts.append('*')
    for index, arg in enumerate(args.kwonlyargs):
        piece = arg.arg
        if args.kw_defaults[index] is not None:
            piece += '=' + ast.unparse(args.kw_defaults[index])
        parts.append(piece)
    if args.kwarg:
        parts.append('**' + args.kwarg.arg)
    returns = ' -> ' + ast.unparse(node.returns) if node.returns else ''
    return '%s(%s)%s' % (node.name, ', '.join(parts), returns)


def decorator_label(node: ast.AST) -> str:
    names = {d.id for d in node.decorator_list if isinstance(d, ast.Name)}
    for kind in ('classmethod', 'staticmethod', 'property'):
        if kind in names:
            return ' _(%s)_' % kind
    return ''


def summary(node: ast.AST) -> str:
    doc = ast.get_docstring(node)
    return doc.strip().split('\n')[0] if doc else ''


def is_public(name: str) -> bool:
    return not name.startswith('_')


def render_module(path: str, out: list) -> None:
    with open(path) as handle:
        source = handle.read()
    tree = ast.parse(source)
    name = os.path.basename(path)
    out.append('## `%s`\n' % name)
    test = os.path.join(os.path.dirname(path), 'test', 'test_%s' % name)
    if os.path.exists(test):
        out.append('_Tested by `test/test_%s`._\n' % name)
    header = module_header(source)
    if header:
        out.append(header + '\n')

    for node in tree.body:
        if isinstance(node, ast.ClassDef) and is_public(node.name):
            bases = ', '.join(ast.unparse(b) for b in node.bases)
            out.append('### `class %s%s`\n' % (node.name, '(%s)' % bases if bases else ''))
            doc = ast.get_docstring(node)
            if doc:
                out.append(doc.strip() + '\n')
            methods = [m for m in node.body
                       if isinstance(m, (ast.FunctionDef, ast.AsyncFunctionDef))
                       and (is_public(m.name) or m.name == '__init__')]
            for method in methods:
                label = 'constructor' if method.name == '__init__' else summary(method)
                out.append('- `%s`%s%s' % (signature(method), decorator_label(method),
                                           ' — ' + label if label else ''))
            out.append('')
        elif isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and is_public(node.name):
            out.append('### `%s`\n' % signature(node))
            doc = ast.get_docstring(node)
            if doc:
                out.append(doc.strip() + '\n')


def main() -> None:
    out = ['# Coludo API reference', '',
           '_Generated from module docstrings by `tools/gen_docs.py` — do not edit by hand;'
           ' run `python3 tools/gen_docs.py` to regenerate._', '']
    for rel, title in SOURCES:
        directory = os.path.join(ROOT, rel)
        files = sorted(f for f in os.listdir(directory)
                       if f.endswith('.py')
                       and not f.startswith(SKIP_PREFIXES)
                       and not os.path.islink(os.path.join(directory, f)))
        if not files:
            continue
        out.append('# %s — `%s`\n' % (title, rel))
        for name in files:
            render_module(os.path.join(directory, name), out)
    target = os.path.join(ROOT, 'doc', 'api.md')
    with open(target, 'w') as handle:
        handle.write('\n'.join(out).rstrip() + '\n')
    print('wrote %s (%d modules)' % (target, sum(1 for _ in out if _.startswith('## '))))


if __name__ == '__main__':
    main()
