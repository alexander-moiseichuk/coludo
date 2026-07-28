#!/usr/bin/env python3
"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Generate doc/api.md from the module sources by *parsing* them (stdlib `ast`) -- never importing, so it
works for the firmware too (which imports machine/network/esp32, absent on the host) and needs no
third-party tools. Module descriptions come from the module docstring (minus the shared copyright
line); class, method and function descriptions from their docstrings. Public surface only (skips
_-internals).

    python3 tools/gen_docs.py        # writes doc/api.md
"""

import ast
import os
import sys

import sources

ROOT = sources.ROOT
TITLES = {  # the chapter heading each scanned directory gets (the dirs themselves live in sources)
    'src/glider': 'glider firmware (MicroPython)',
    'src/glider/drivers': 'glider HAL drivers — `drivers/`',
    'src/glider/tasks': 'glider subsystem tasks — `tasks/`',
    'src/control': 'control (CPython)',
    'src/control/commands': 'control operator commands — `commands/`',
}


def module_header(tree: ast.Module) -> str:
    """
    The module's description text: its docstring with the shared copyright line stripped.

    Every module now opens with a docstring whose first line is the 'Coludo project, copyright ...'
    notice; api.md wants the prose that follows, not that boilerplate repeated per module. When the
    copyright line is absent the whole docstring is returned unchanged.

    Args:
        tree - the parsed module AST.

    Returns:
        The description prose (copyright line and its trailing blank removed), or '' when the module
        has no docstring.
    """
    doc = ast.get_docstring(tree)
    if not doc:
        return ''
    lines = doc.splitlines()
    if lines and lines[0].strip().startswith('Coludo project, copyright'):
        lines = lines[1:]
        while lines and lines[0].strip() == '':
            lines.pop(0)
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
    tree = sources.parse(path)
    name = os.path.basename(path)
    out.append('## `%s`\n' % name)
    test = os.path.join(os.path.dirname(path), 'test', 'test_%s' % name)
    if os.path.exists(test):
        out.append('_Tested by `test/test_%s`._\n' % name)
    header = module_header(tree)
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


def main(check: bool = False) -> None:
    out = ['# Coludo API reference', '',
           '_Generated from module docstrings by `tools/gen_docs.py` — do not edit by hand;'
           ' run `python3 tools/gen_docs.py` to regenerate._', '',
           'See [`architecture.md`](architecture.md) for the module dependency graph, class hierarchy,'
           ' and the annotated `Flight._step()` hot-path call tree (`tools/gen_graph.py`).', '']
    chapter = None
    for relative, _name, path in sources.modules(sources.GLIDER_DIRS + sources.CONTROL_DIRS):
        if relative != chapter:  # a directory with no sources never opens a chapter
            out.append('# %s — `%s`\n' % (TITLES[relative], relative))
            chapter = relative
        render_module(path, out)
    target = os.path.join(ROOT, 'doc', 'api.md')
    text = '\n'.join(out).rstrip() + '\n'
    """
    --check makes this gateable like the other three generators. It was the ONLY derived doc `make
    check` did not enforce, and it silently rotted 417 lines (330+/87-) while gen_pinmap / gen_graph /
    gen_schema stayed exact through the same period -- an API reference missing a session's worth of
    modules and signatures is worse than none, because it reads as current.
    """
    if check:
        existing = ''
        if os.path.exists(target):
            with open(target) as handle:
                existing = handle.read()
        if existing != text:
            print('doc/api.md is STALE -- run: python3 tools/gen_docs.py', file=sys.stderr)
            return 1
        print('api doc up to date')
        return 0
    with open(target, 'w') as handle:
        handle.write(text)
    print('wrote %s (%d modules)' % (target, sum(1 for _ in out if _.startswith('## '))))
    return 0


if __name__ == '__main__':
    sys.exit(main('--check' in sys.argv))
