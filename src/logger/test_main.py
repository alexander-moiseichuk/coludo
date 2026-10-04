"""
On-board test of the nose logger's decisions -- run on the C6 with main.py installed, in read-out mode:

    mpremote connect /dev/serial/by-id/usb-Espressif* run src/logger/test_main.py

Positive and negative for every rule that decides what is kept: the launch test and the climb that
confirms it, the stillness test, the flash pattern, the flight record and the room-making that must never
touch a flight. The filesystem cases run inside a scratch directory, so the logs on the board are never at
risk. Values are literals: main's const() names are compiled away on the board.
"""

import os

import main

_SCRATCH: str = 'logger_test'
_checks: list = [0]


def _check(condition: bool, what: str) -> None:
    """One assertion, counted."""
    if not condition:
        raise AssertionError(what)
    _checks[0] += 1


def _boost() -> None:
    """3 g held is a launch, a hand is not; any axis, either sign."""
    g = 2048
    _check(not main._boost(0, 0, 0), 'zero g is no boost')
    _check(not main._boost(0, -g, 0), 'gravity is no boost')
    _check(not main._boost(int(2.9 * g), 0, 0), '2.9 g is under')
    _check(main._boost(int(3.1 * g), 0, 0), '3.1 g is a boost')
    _check(main._boost(0, -32768, 0), 'the -16 g rail is a boost')
    _check(main._boost(int(2 * g), int(2 * g), int(2 * g)), '2 g on each axis is 3.46 g')
    _check(not main._boost(int(1.7 * g), int(1.7 * g), int(1.7 * g)), '1.7 g on each axis is 2.94 g')


def _climbed() -> None:
    """The climb that confirms a launch: ~20 m up from where the 3 g began, never down or less."""
    ground = 101300 * 64
    _check(main._climbed(ground, ground - 241 * 64), '241 Pa up is a climb')
    _check(not main._climbed(ground, ground - 200 * 64), '17 m is not enough')
    _check(not main._climbed(ground, ground + 500 * 64), 'down is no climb')
    _check(not main._climbed(ground, ground), 'a hand on the bench climbs nothing')


def _moved() -> None:
    """The stillness test the landing rule counts on."""
    _check(not main._moved(0, -2048, 0, 5, -5, 5, 1000, 0, -2048, 0, 1000), 'rest noise is still')
    _check(main._moved(101, -2048, 0, 0, 0, 0, 1000, 0, -2048, 0, 1000), '0.05 g of change moves')
    _check(main._moved(0, -2048, 0, 0, 81, 0, 1000, 0, -2048, 0, 1000), '5 dps moves')
    _check(main._moved(0, -2048, 0, 0, 0, 0, 1000 + 1537, 0, -2048, 0, 1000), 'a chute descent moves')


def _lit() -> None:
    """Each flash is `on` ticks lit then `off` dark, counted down."""
    levels = [main._lit(left, 3, 1) for left in range(8, 0, -1)]
    _check(levels == [1, 1, 1, 0, 1, 1, 1, 0], 'pattern %s' % levels)


def _touch(name: str, size: int) -> None:
    """A file of `size` bytes."""
    with open(name, 'wb') as handle:
        handle.write(bytes(size))


def _files() -> list:
    """What the scratch directory holds, sorted."""
    return sorted(os.listdir())


def _flights() -> None:
    """The flight record: parsing, protection, and a bad line costing only itself."""
    with open('flights.txt', 'w') as handle:
        handle.write('4 35\nnot a line\n7 0\n4 40\n')
    flights = main._flights()
    _check(flights == {4: 35, 7: 0}, 'parsed %s' % flights)
    _check(not main._protected('b00004_s0034.bin', flights), 'the segment before a flight is not one')
    _check(main._protected('b00004_s0035.bin', flights), 'the first flight segment is protected')
    _check(main._protected('b00004_s0099.bin', flights), 'the flight runs to the end of its boot')
    _check(not main._protected('b00005_s0040.bin', flights), 'another boot is not the flight')
    _check(main._protected('b00007_s0000.bin', flights), 'a launch in segment 0 protects from 0')
    main._protect(9, 3)
    _check(main._flights().get(9) == 3, 'a launch is appended')
    os.remove('flights.txt')
    _check(main._flights() == {}, 'no record, no flights')


def _room() -> None:
    """Room is made from the oldest unprotected logs only; a flight and anything not a log never go."""
    size = 1000
    for name in ('b00001_s0000.bin', 'b00001_s0001.bin', 'b00001_s0002.bin', 'b00002_s0000.bin',
                 'b00002_s0001.bin', 'keep.txt'):
        _touch(name, size)
    main._protect(2, 0)
    free = main._free()
    _check(main._room(free + 2 * size), 'two old logs could make room')
    _check(_files() == ['b00001_s0000.bin', 'b00001_s0001.bin', 'b00001_s0002.bin', 'b00002_s0000.bin',
                        'b00002_s0001.bin', 'flights.txt', 'keep.txt'], '_room deletes nothing')
    _check(main._make_room(free + 2 * size - 1), 'room made')
    _check(_files() == ['b00001_s0002.bin', 'b00002_s0000.bin', 'b00002_s0001.bin', 'flights.txt', 'keep.txt'],
           'exactly the two oldest went: %s' % _files())
    impossible = main._free() + 100 * 1024 * 1024
    _check(not main._room(impossible), 'no room past the flights')
    _check(not main._make_room(impossible), 'cannot make it either')
    _check(_files() == ['b00002_s0000.bin', 'b00002_s0001.bin', 'flights.txt', 'keep.txt'],
           'the flight and the non-log survive: %s' % _files())
    main.clean()
    _check(_files() == ['keep.txt'], 'clean() takes logs and the record only: %s' % _files())
    os.remove('keep.txt')


def run() -> None:
    """Every case, in a scratch directory that is removed afterwards whatever happens."""
    _boost()
    _climbed()
    _moved()
    _lit()
    home = os.getcwd()
    try:
        os.mkdir(_SCRATCH)
    except OSError:
        pass
    os.chdir(_SCRATCH)
    try:
        for name in os.listdir():
            os.remove(name)
        _flights()
        _room()
    finally:
        for name in os.listdir():
            os.remove(name)
        os.chdir(home)
        os.rmdir(_SCRATCH)
    print('ok: %d checks' % _checks[0])


run()
