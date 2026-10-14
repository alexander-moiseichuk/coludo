#!/usr/bin/env python3
"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Generate doc/avionics_api.md, the Rust avionics' API reference (src/avionics), as tools/gen_docs.py generates
doc/glider_api.md for plan B and the hub: from the sources' own doc comments, by parsing them -- never building,
so it needs no cargo, no toolchain and no third-party tools.

One chapter per crate (crates/*, firmware, recorder, sitl), in dependency order: the crate's `//!` text and the
crates it uses. Per module, its `//!` text (the shared copyright line stripped), the test that covers it, then each
public item: a type with its full `///` text and its public methods' summaries, a public function with its text, the
public constants as one list. `pub(crate)` and test code are not public API.

    python3 tools/gen_avionics_docs.py           # writes doc/avionics_api.md
    python3 tools/gen_avionics_docs.py --check   # exit 1 when the file on disk is stale
"""

import glob
import os
import re
import sys

ROOT: str = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
AVIONICS: str = os.path.join(ROOT, 'src', 'avionics')
TARGET: str = os.path.join(ROOT, 'doc', 'avionics_api.md')
_COPYRIGHT: str = 'Coludo project, copyright under MIT license'
_ITEM: re.Pattern = re.compile(
    r'^pub\s+(?:(?:const|unsafe|async|extern\s+"C")\s+)*(fn|struct|enum|trait|const|static|type)\s+(\w+)')
_IMPL: re.Pattern = re.compile(r'^(?:unsafe\s+)?impl\b(?:<[^{]*?>)?\s+(?:(?:[\w:]+)(?:<[^{]*>)?\s+for\s+)?([\w:]+)')
_METHOD: re.Pattern = re.compile(r'^(?:pub\s+)?(?:(?:const|unsafe|async)\s+)*fn\s+(\w+)')


def blank(text: str) -> str:
    """
    The text with every comment, string and char literal blanked to spaces (newlines kept), so braces and keywords
    are counted on code alone: a `format!("{name}")` or a `'{'` must not move the depth.

    Args:
        text - a Rust source file.

    Returns:
        The same length of text, code characters in place, everything else a space.
    """
    out = []
    i, size = 0, len(text)
    while i < size:
        c = text[i]
        if text.startswith('//', i):
            end = text.find('\n', i)
            end = size if end < 0 else end
            out.append(' ' * (end - i))
            i = end
        elif text.startswith('/*', i):
            end = text.find('*/', i + 2)
            end = size if end < 0 else end + 2
            out.append(re.sub(r'[^\n]', ' ', text[i:end]))
            i = end
        elif c == 'r' and re.match(r'r#*"', text[i:i + 8]) and (i == 0 or not (text[i - 1].isalnum() or
                                                                             text[i - 1] == '_')):
            hashes = re.match(r'r(#*)"', text[i:]).group(1)
            close = '"' + hashes
            end = text.find(close, i + 2 + len(hashes))
            end = size if end < 0 else end + len(close)
            out.append(re.sub(r'[^\n]', ' ', text[i:end]))
            i = end
        elif c == '"':
            j = i + 1
            while j < size and text[j] != '"':
                j += 2 if text[j] == '\\' else 1
            out.append(re.sub(r'[^\n]', ' ', text[i:j + 1]))
            i = j + 1
        elif c == "'":
            literal = re.match(r"'(\\(x[0-9a-fA-F]{2}|u\{[0-9a-fA-F]+\}|.)|[^\\'\n])'", text[i:])
            if literal:
                out.append(' ' * len(literal.group(0)))
                i += len(literal.group(0))
            else:  # a lifetime or a label
                out.append(c)
                i += 1
        else:
            out.append(c)
            i += 1
    return ''.join(out)


def module_doc(lines: list) -> str:
    """The file's leading `//!` text, the copyright line and the blank after it stripped."""
    doc = []
    for line in lines:
        stripped = line.strip()
        if stripped.startswith('//!'):
            doc.append(stripped[3:][1:] if stripped[3:4] == ' ' else stripped[3:])
        elif stripped and not stripped.startswith('#!['):
            break
    if doc and doc[0].startswith(_COPYRIGHT):
        doc = doc[1:]
        while doc and not doc[0].strip():
            doc.pop(0)
    return '\n'.join(doc).strip()


def doc_above(lines: list, index: int) -> str:
    """The `///` text right above line `index`, attributes between them skipped."""
    doc = []
    j = index - 1
    while j >= 0:
        stripped = lines[j].strip()
        if stripped.startswith('///'):
            doc.append(stripped[3:][1:] if stripped[3:4] == ' ' else stripped[3:])
        elif stripped.startswith('#[') or stripped.startswith('#!['):
            pass
        else:
            break
        j -= 1
    return '\n'.join(reversed(doc)).strip()


def summary(doc: str) -> str:
    """A doc text's first paragraph, on one line."""
    return ' '.join(doc.split('\n\n')[0].split())


def signature(code: list, index: int) -> str:
    """The item starting at line `index` of the blanked code, up to its body or its end, on one line."""
    parts = []
    depth = 0
    for line in code[index:index + 40]:
        for position, c in enumerate(line):
            if c in '([<':
                depth += 1
            elif c in ')]>' and depth:
                depth -= 1
            elif c in '{;' and depth == 0:
                parts.append(line[:position])
                return _tidy(' '.join(parts)).rstrip(' =')
        parts.append(line)
    return _tidy(' '.join(parts))


def _tidy(text: str) -> str:
    """A signature joined from several lines, on one: single spaces, no space inside the parentheses' edges and no
    trailing comma before a closing one (rustfmt's layout of a long parameter list)."""
    text = ' '.join(text.split())
    text = re.sub(r'([(<\[])\s+', r'\1', text)
    return re.sub(r',?\s+([)>\]])', r'\1', text)


def constant(original: list, code: list, index: int) -> str:
    """A `pub const`'s line: its name, type and value as written, on one line (a long value cut)."""
    text = []
    for j in range(index, min(index + 12, len(code))):
        text.append(original[j].strip())
        if ';' in code[j]:
            break
    line = ' '.join(' '.join(text).split())
    line = re.sub(r'^pub\s+', '', line).rstrip(';')
    return line if len(line) <= 120 else line[:117] + '...'


def scan(path: str) -> dict:
    """
    One source file's public surface.

    Returns:
        {doc, types: [(signature, doc, [(method signature, summary)])], functions: [(signature, doc)],
         constants: [(line, summary)], tested: bool (an inline or a sibling `mod tests`)}.
    """
    with open(path, encoding='utf-8') as handle:
        text = handle.read()
    original = text.split('\n')
    code = blank(text).split('\n')
    result = {'doc': module_doc(original), 'types': [], 'functions': [], 'constants': [], 'tested': False}
    types = {}
    depth = 0
    test_depth = None
    impl_target = None
    impl_trait = False
    for index, line in enumerate(code):
        stripped = line.strip()
        at = depth
        if test_depth is None and at == 0 and re.match(r'(pub\s+)?mod\s+tests\b', stripped):
            result['tested'] = True
            if '{' in line:
                test_depth = at
        cfg_test = index and original[index - 1].strip() == '#[cfg(test)]'
        if test_depth is None and not cfg_test:
            if at == 0:
                item = _ITEM.match(stripped)
                if item:
                    kind, name = item.groups()
                    doc = doc_above(original, index)
                    if kind in ('const', 'static'):
                        result['constants'].append((constant(original, code, index), summary(doc)))
                    elif kind == 'fn':
                        result['functions'].append((signature(code, index), doc))
                    else:
                        entry = (signature(code, index), doc, [])
                        types[name] = entry
                        result['types'].append(entry)
                impl = _IMPL.match(stripped)
                if impl and '{' in line:
                    impl_target = impl.group(1).split('::')[-1]
                    impl_trait = ' for ' in stripped
            elif at == 1 and impl_target is not None:
                method = _METHOD.match(stripped)
                # an inherent impl's public methods; a trait impl's methods are the trait's (documented there)
                if method and stripped.startswith('pub') and not impl_trait and impl_target in types:
                    sig = re.sub(r'^pub\s+', '', signature(code, index))
                    types[impl_target][2].append((sig, summary(doc_above(original, index))))
        depth += line.count('{') - line.count('}')
        if test_depth is not None and depth <= test_depth:
            test_depth = None
        if depth == 0:
            impl_target = None
    return result


def trait_methods(path: str, name: str) -> list:
    """A public trait's methods, (signature, summary), from its definition in `path`."""
    with open(path, encoding='utf-8') as handle:
        text = handle.read()
    original = text.split('\n')
    code = blank(text).split('\n')
    methods = []
    depth = 0
    inside = False
    for index, line in enumerate(code):
        stripped = line.strip()
        if depth == 0 and re.match(r'pub\s+trait\s+%s\b' % name, stripped):
            inside = True
        elif inside and depth == 1:
            method = _METHOD.match(stripped)
            if method:
                methods.append((signature(code, index), summary(doc_above(original, index))))
        depth += line.count('{') - line.count('}')
        if inside and depth == 0 and index and '}' in line:
            inside = False
    return methods


def crates() -> list:
    """
    The workspace's packages, in dependency order (a crate after the crates it uses; ties by name).

    Returns:
        [(name, directory, [workspace dependencies], [other dependencies])].
    """
    manifests = {}
    directories = sorted(glob.glob(os.path.join(AVIONICS, 'crates', '*'))) + [
        os.path.join(AVIONICS, name) for name in ('firmware', 'recorder', 'sitl')]
    for directory in directories:
        manifest = os.path.join(directory, 'Cargo.toml')
        if not os.path.exists(manifest):
            continue
        with open(manifest, encoding='utf-8') as handle:
            toml = handle.read()
        manifests[re.search(r'^name\s*=\s*"([^"]+)"', toml, re.MULTILINE).group(1)] = (directory, toml)
    found = {}
    for name, (directory, toml) in manifests.items():
        section = re.search(r'^\[dependencies\]\n(.*?)(?=^\[|\Z)', toml, re.MULTILINE | re.DOTALL)
        local, other = [], []
        for line in (section.group(1) if section else '').split('\n'):
            # `name = ...`, `name.workspace = true` or `name = { workspace = true }`: a workspace package is local
            dependency = re.match(r'^([\w-]+)(?:\.workspace)?\s*=', line.strip())
            if dependency:
                (local if dependency.group(1) in manifests else other).append(dependency.group(1))
        found[name] = (directory, sorted(local), sorted(other))
    ordered, placed = [], set()
    while len(ordered) < len(found):
        ready = sorted(name for name, (_dir, local, _other) in found.items()
                       if name not in placed and all(dependency in placed or dependency not in found
                                                     for dependency in local))
        if not ready:  # a cycle: list the rest by name rather than loop
            ready = sorted(set(found) - placed)
        for name in ready:
            ordered.append((name,) + found[name])
            placed.add(name)
    return ordered


def module_path(crate: str, source: str, path: str) -> str:
    """The module's Rust path (`drivers::bmp280`), from its file under the crate's src/."""
    relative = os.path.relpath(path, source)[:-3].replace(os.sep, '::')
    if relative in ('lib', 'main'):
        return crate
    if relative.endswith('::mod'):
        relative = relative[:-5]
    if relative.startswith('bin::'):
        return '%s (binary %s)' % (crate, relative[5:])
    return '%s::%s' % (crate, relative)


def render(out: list) -> int:
    """Append every crate's chapter to `out`; the number of modules."""
    count = 0
    for name, directory, local, other in crates():
        source = os.path.join(directory, 'src')
        out.append('# `%s` — `%s`\n' % (name, os.path.relpath(directory, ROOT)))
        uses = ', '.join('`%s`' % dependency for dependency in local) or 'no workspace crate'
        out.append('_Uses: %s. External: %s._\n' % (uses, ', '.join(other) or 'none'))
        tests = sorted(os.path.relpath(path, directory) for path in glob.glob(os.path.join(directory, 'tests', '*.rs')))
        if tests:
            out.append('_Integration tests: %s._\n' % ', '.join('`%s`' % test for test in tests))
        files = sorted(glob.glob(os.path.join(source, '**', '*.rs'), recursive=True))
        files = [path for path in files if os.path.basename(path) != 'tests.rs' and
                 os.sep + 'tests' + os.sep not in os.path.relpath(path, source)]
        roots = [path for path in files if os.path.basename(path) in ('lib.rs', 'main.rs') and
                 os.path.dirname(path) == source]
        for path in roots + [path for path in files if path not in roots]:
            module = scan(path)
            out.append('## `%s`\n' % module_path(name, source, path))
            sibling = os.path.join(os.path.splitext(path)[0], 'tests.rs')
            if os.path.exists(sibling):
                out.append('_Tested by `%s`._\n' % os.path.relpath(sibling, directory))
            elif module['tested']:
                out.append('_Tested by its own `mod tests`._\n')
            if module['doc']:
                out.append(module['doc'] + '\n')
            for signature_text, doc, methods in module['types']:
                out.append('### `%s`\n' % signature_text)
                if doc:
                    out.append(doc + '\n')
                if signature_text.startswith('pub trait'):
                    methods = trait_methods(path, signature_text.split()[2].split('<')[0].rstrip(':'))
                for method, method_summary in methods:
                    out.append('- `%s`%s' % (method, ' — ' + method_summary if method_summary else ''))
                if methods:
                    out.append('')
            for signature_text, doc in module['functions']:
                out.append('### `%s`\n' % signature_text)
                if doc:
                    out.append(doc + '\n')
            if module['constants']:
                out.append('### Constants\n')
                for line, line_summary in module['constants']:
                    out.append('- `%s`%s' % (line, ' — ' + line_summary if line_summary else ''))
                out.append('')
            count += 1
    return count


def main(check: bool = False) -> int:
    out = ['# Coludo avionics API reference', '',
           '_Generated from the Rust sources\' doc comments by `tools/gen_avionics_docs.py` — do not edit by hand;'
           ' run `python3 tools/gen_avionics_docs.py` to regenerate._', '',
           'The Rust flight firmware (`src/avionics`, [specs/avionics-port.md](specs/avionics-port.md)): one chapter'
           ' per crate, a crate after the crates it uses. Plan B (MicroPython) and the control hub are in'
           ' [`glider_api.md`](glider_api.md).', '']
    modules = render(out)
    text = '\n'.join(out).rstrip() + '\n'
    if check:
        existing = ''
        if os.path.exists(TARGET):
            with open(TARGET, encoding='utf-8') as handle:
                existing = handle.read()
        if existing != text:
            print('doc/avionics_api.md is STALE -- run: python3 tools/gen_avionics_docs.py', file=sys.stderr)
            return 1
        print('avionics api doc up to date')
        return 0
    with open(TARGET, 'w', encoding='utf-8') as handle:
        handle.write(text)
    print('wrote %s (%d modules)' % (TARGET, modules))
    return 0


if __name__ == '__main__':
    sys.exit(main('--check' in sys.argv))
