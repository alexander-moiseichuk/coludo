"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Host (CPython) test for tools/panic_decode.py, the decoder of a Rust board's panic log (doc/specs/avionics-port.md
"Panic log"). A /panic text written out by hand -- an oops, a fatal panic with the ESP-IDF handler's register dump
and stack memory -- is split into its records, and the program counters worth decoding are picked: MEPC and RA, then
the stack words that land in the build's code sections (.iram0.text, .flash.text), each once, while data words, heap
and rodata addresses in the same memories, a register outside code and an oops's text are left alone. Negative: no
header, a header of another layout, an empty stack line. The addr2line and objdump calls run only when IDF's
toolchain and a built ELF exist (else that part is skipped). Stdlib only.
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', 'tools'))
import panic_decode  # noqa: E402

OOPS = ('COLUDO-PANIC 1 seq=7 fatal=0 cause=test-oops boot=365 session=000365 uptime_ms=11310 '
        'firmware=avionics-0.1.0 config=3aa09342698a recorded=365 length=59 intact=1\n'
        'a deliberate oops from a console command: nothing is broken\n')

FATAL = ('COLUDO-PANIC 1 seq=4 fatal=1 cause=panic boot=361 session=000361 uptime_ms=22117 '
         'firmware=avionics-0.1.0 config=3aa09342698a recorded=362 length=900 intact=1\n'
         "Guru Meditation Error: Core  0 panic'ed (Store access fault). Exception was unhandled.\n"
         '\n'
         'Core  0 register dump:\n'
         'MEPC    : 0x4802076a  RA      : 0x4802072e  SP      : 0x48635870  GP      : 0x4ff15880  \n'
         'TP      : 0x48635c60  T0      : 0x4817bf50  T1      : 0x4ff96410  T2      : 0x4863d340  \n'
         'MSTATUS : 0x00011880  MTVEC   : 0x4ff00003  MCAUSE  : 0x00000007  MTVAL   : 0x00000000  \n'
         '\n'
         'Stack memory:\n'
         '48635870: 0x00000020 0x01010100 0x20202020 0x4863d014 0x00000000 0x00000001 0x4ff961d4 0x00000008\n'
         '48635890: 0x4802076a 0x4ff0dd60 0x80808080 0x4800b660 0x4817becc 0x00000004 0x00000005 0x00000001\n')

"""The code of the build the records came from: .iram0.text and .flash.text, as objdump -h would give them."""
CODE = [(0x4FF00000, 0x4FF15028), (0x48000020, 0x4816E300)]


def test_records_split_newest_first_as_served():
    records = panic_decode.split_records(FATAL + '\n' + OOPS + '\n')
    assert [header.split()[2] for header, _ in records] == ['seq=4', 'seq=7']
    assert records[1][1] == 'a deliberate oops from a console command: nothing is broken'
    assert records[0][1].startswith("Guru Meditation Error")


def test_program_counters_are_the_registers_then_code_words_of_the_stack():
    _, body = panic_decode.split_records(FATAL)[0]
    found = panic_decode.candidates(body, CODE)
    # MEPC and RA first; then the stack's code words, once each (MEPC again on the stack is not repeated); the
    # stack pointer values (0x4863...), heap words (0x4ff96...: internal RAM, but past .iram0.text) and plain data
    # are not code
    assert found[:2] == [('MEPC', 0x4802076A), ('RA', 0x4802072E)]
    stack = [address for label, address in found if label == 'stack']
    assert stack == [0x4FF0DD60, 0x4800B660]
    assert 0x4802076A not in stack
    assert all(panic_decode.in_code(address, CODE) for _, address in found)
    # 0x4817becc: PSRAM, but past .flash.text (rodata)
    for data in (0x00000020, 0x20202020, 0x80808080, 0x4FF961D4, 0x4817BECC):
        assert data not in [address for _, address in found]


def test_an_abort_names_its_caller_first():
    body = ('abort() was called at PC 0x480208dd on core 0\n'
            'MEPC    : 0x4ff09744  RA      : 0x4ff092c4  SP      : 0x486357f0  \n')
    found = panic_decode.candidates(body, CODE)
    assert found == [('abort', 0x480208DD), ('MEPC', 0x4FF09744), ('RA', 0x4FF092C4)]


def test_an_oops_and_foreign_text_have_nothing_to_decode():
    _, body = panic_decode.split_records(OOPS)[0]
    assert panic_decode.candidates(body, CODE) == []
    assert panic_decode.split_records('no header here\nMEPC : 0x4802076a\n') == []
    assert panic_decode.split_records('COLUDO-CRASH 1 seq=1 length=0\n') == []
    assert panic_decode.candidates('Stack memory:\n48635870:\n', CODE) == []


def test_addr2line_places_a_real_record_when_the_toolchain_is_here():
    tools = sorted(__import__('glob').glob(panic_decode._ADDR2LINE_GLOB))
    if not tools or not os.path.exists(panic_decode._ELF):
        print('  (skipped: no IDF addr2line or no built ELF)')
        return
    # the build's own code ranges hold its text section, and an address outside every range is not placed
    objdump = sorted(__import__('glob').glob(panic_decode._TOOL_GLOB.format('objdump')))[-1]
    ranges = panic_decode.code_ranges(panic_decode._ELF, objdump)
    assert any(low <= 0x48100000 < high for low, high in ranges)
    decoded = panic_decode.decode([0x00000004], panic_decode._ELF, tools[-1])
    assert 0x00000004 not in decoded


if __name__ == '__main__':
    for name, test in sorted(globals().items()):
        if name.startswith('test_') and callable(test):
            test()
    print('ok')
