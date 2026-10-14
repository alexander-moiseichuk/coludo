#!/usr/bin/env python3
"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Decode the panic log of a Rust (src/avionics) board against the firmware's ELF: each record's header line, then its
program counters -- MEPC, RA and every stack word that lands in code -- through addr2line, as source lines.

    tools/panic_decode.py <source> [--elf ELF] [--addr2line PATH]

    source  a board (taster, 192.168.102.57[:port]: GET /panic on its file server), a record's URL
            (http://board:8000/panic?seq=N), or a file holding records (the /panic text, or a copy of it)
    --elf   the firmware the records came from (default: the release build, src/avionics/target/...); decoding
            another build's record gives wrong lines, so check the header's firmware= against it

The records come from crates/panics (doc/specs/avionics-port.md "Panic log"): a header line `COLUDO-PANIC 1 seq=...`
then the captured text, which for a fatal panic is the ESP-IDF panic handler's output (register dump, stack memory).
An oops has no registers: it is printed as is. Stdlib only.
"""

import argparse
import glob
import os
import re
import subprocess
import sys
import urllib.request

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

"""The release ELF of the firmware, built by `cargo build --release` in src/avionics/firmware."""
_ELF = os.path.join(_ROOT, 'src/avionics/target/riscv32imafc-esp-espidf/release/avionics')

"""Where IDF's toolchain keeps its binutils for the P4's RISC-V cores ({} is the tool: addr2line, objdump)."""
_TOOL_GLOB = os.path.expanduser('~/.espressif/tools/riscv32-esp-elf/*/riscv32-esp-elf/bin/riscv32-esp-elf-{}')

"""addr2line's place in IDF's toolchain."""
_ADDR2LINE_GLOB = _TOOL_GLOB.format('addr2line')

"""A record's first line."""
_HEADER = re.compile(r'^COLUDO-PANIC \d+ .*$', re.M)

"""Where abort() was called (`abort() was called at PC 0x480208dd on core 0`): for an abort, MEPC and RA are inside
IDF's panic_abort, and this is the caller."""
_ABORT = re.compile(r'abort\(\) was called at PC 0x([0-9a-fA-F]{8})')

"""A register in the panic handler's dump: `MEPC    : 0x4802076a`."""
_REGISTER = re.compile(r'\b(MEPC|RA)\s*:\s*0x([0-9a-fA-F]{8})')

"""A line of the panic handler's `Stack memory:` dump: an address, then eight words."""
_STACK_LINE = re.compile(r'^[0-9a-fA-F]{8}:((?:\s+0x[0-9a-fA-F]{8})+)\s*$', re.M)

"""The board's file server port."""
_PORT = 8000


def split_records(text: str) -> list:
    """
    The records of a /panic text.

    Args:
        text - the text, records each starting with a COLUDO-PANIC header line.

    Returns:
        [(header line, body)] in the text's order (newest first, as the board serves them).
    """
    starts = [match.start() for match in _HEADER.finditer(text)]
    records = []
    for index, start in enumerate(starts):
        end = starts[index + 1] if index + 1 < len(starts) else len(text)
        header, _, body = text[start:end].partition('\n')
        records.append((header, body.rstrip('\n')))
    return records


def code_ranges(elf: str, objdump: str) -> list:
    """
    Where the firmware's code lives: its sections flagged CODE (.iram0.text in internal RAM, .flash.text run from
    PSRAM, ...), from the ELF's own section table, so heap and stack words in the same memories are not taken for
    code.

    Args:
        elf - the firmware's ELF.
        objdump - the toolchain's objdump.

    Returns:
        [(start, end)] addresses.
    """
    output = subprocess.run([objdump, '-h', elf], capture_output=True, text=True, check=True).stdout.splitlines()
    ranges = []
    for line, flags in zip(output, output[1:]):
        fields = line.split()
        if 'CODE' in flags and len(fields) >= 4 and fields[0].isdigit():
            start, size = int(fields[3], 16), int(fields[2], 16)
            ranges.append((start, start + size))
    return ranges


def in_code(address: int, ranges: list) -> bool:
    """
    Whether an address can be an instruction's.

    Args:
        address - the address.
        ranges - the code's [(start, end)].

    Returns:
        True inside one of the ranges.
    """
    return any(low <= address < high for low, high in ranges)


def candidates(body: str, ranges: list) -> list:
    """
    The program counters worth decoding in a record's body, in order and once each: where abort() was called, MEPC
    and RA, then the stack words that land in code (return addresses, mostly; some are stale or data that happens to
    look like code).

    Args:
        body - the record's text after its header.
        ranges - the code's [(start, end)] (code_ranges).

    Returns:
        [(label, address)].
    """
    found = []
    seen = set()
    for value in _ABORT.findall(body):
        address = int(value, 16)
        if address not in seen and in_code(address, ranges):
            found.append(('abort', address))
            seen.add(address)
    for name, value in _REGISTER.findall(body):
        address = int(value, 16)
        if address not in seen and in_code(address, ranges):
            found.append((name, address))
            seen.add(address)
    for match in _STACK_LINE.finditer(body):
        for word in match.group(1).split():
            address = int(word, 16)
            if address not in seen and in_code(address, ranges):
                found.append(('stack', address))
                seen.add(address)
    return found


def decode(addresses: list, elf: str, addr2line: str) -> dict:
    """
    Source lines for addresses, through addr2line.

    Args:
        addresses - the addresses.
        elf - the firmware's ELF.
        addr2line - the tool.

    Returns:
        {address: 'function at file:line' (inlined frames joined by ' <- ')}, for those addr2line could place.
    """
    if not addresses:
        return {}
    output = subprocess.run([addr2line, '-pfiaC', '-e', elf] + ['0x%08x' % address for address in addresses],
                            capture_output=True, text=True, check=True).stdout
    decoded = {}
    current = None
    for line in output.splitlines():
        match = re.match(r'^0x([0-9a-f]{8}): (.*)$', line)
        if match:
            current = int(match.group(1), 16)
            decoded[current] = [match.group(2)]
        elif current is not None and line.strip().startswith('(inlined by)'):
            decoded[current].append(line.strip()[len('(inlined by) '):])
    return {address: ' <- '.join(frames) for address, frames in decoded.items() if not frames[0].startswith('??')}


def fetch(source: str) -> str:
    """
    The panic log's text from a board, a URL or a file.

    Args:
        source - see the module notes.

    Returns:
        The text.
    """
    if os.path.exists(source):
        with open(source, encoding='utf-8', errors='replace') as handle:
            return handle.read()
    host = source if ':' in source else '%s:%d' % (source, _PORT)
    url = source if source.startswith('http') else 'http://%s/panic' % host
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    with opener.open(url, timeout=30) as response:
        return response.read().decode('utf-8', errors='replace')


def main() -> int:
    parser = argparse.ArgumentParser(description='Decode a board\'s panic log against the firmware ELF')
    parser.add_argument('source', help='a board, a /panic URL or a file of records')
    parser.add_argument('--elf', default=_ELF, help='the firmware ELF the records came from')
    parser.add_argument('--addr2line', default=None, help='riscv32-esp-elf-addr2line (default: IDF\'s)')
    parser.add_argument('--objdump', default=None, help='riscv32-esp-elf-objdump (default: IDF\'s)')
    arguments = parser.parse_args()
    addr2line = arguments.addr2line or next(iter(sorted(glob.glob(_ADDR2LINE_GLOB))), None)
    objdump = arguments.objdump or next(iter(sorted(glob.glob(_TOOL_GLOB.format('objdump')))), None)
    if not addr2line or not objdump:
        print('panic_decode: no riscv32-esp-elf addr2line/objdump (IDF\'s toolchain); give --addr2line and --objdump',
              file=sys.stderr)
        return 2
    if not os.path.exists(arguments.elf):
        print('panic_decode: no ELF at %s; give --elf' % arguments.elf, file=sys.stderr)
        return 2
    try:
        records = split_records(fetch(arguments.source))
    except OSError as error:  # urllib's URLError is one
        print('panic_decode: cannot read %s: %s' % (arguments.source, error), file=sys.stderr)
        return 2
    if not records:
        print('panic_decode: no records')
        return 1
    try:
        ranges = code_ranges(arguments.elf, objdump)
    except (OSError, subprocess.CalledProcessError) as error:
        print('panic_decode: objdump failed: %s' % error, file=sys.stderr)
        return 2
    for header, body in records:
        print(header)
        found = candidates(body, ranges)
        if not found:
            print(body)  # an oops, or a fatal reset with nothing captured: its text is all there is
        else:
            try:
                decoded = decode([address for _, address in found], arguments.elf, addr2line)
            except (OSError, subprocess.CalledProcessError) as error:
                print('panic_decode: addr2line failed: %s' % error, file=sys.stderr)
                return 2
            for line in body.splitlines()[:3]:
                print('  ' + line)
            for label, address in found:
                if address in decoded:
                    print('  %-5s 0x%08x %s' % (label, address, decoded[address]))
        print()
    return 0


if __name__ == '__main__':
    sys.exit(main())
