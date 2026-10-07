"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Host (CPython) test for the board's judgment of a microSD card's partition table, components/sd_card
partition_table.c (doc/specs/avionics-port.md "SD and flash file layout", How the board mounts them): the plain C
file the firmware builds, compiled here with the host's cc into its harness (components/sd_card/test/
partition_table_host.c) and run over card images. Nothing but this test's own image files is ever read or written.

What decides the board's mode is checked on the card shapes it meets:
* two: a card tools/sd_prepare.sh prepared (the partitions it planned, read back from sfdisk), and two small FAT16
  partitions whose boot sectors give their size in the 16-bit field; a partition 2 that holds no file system yet is
  left to the mount (which fails it on the board: no video that boot);
* one, FatFs's own search: a superfloppy (FAT at sector 0), a card with no table, an MBR with one FAT partition (an
  M4-era card), and a partition 2 of a type that is not FAT;
* broken, partition 1 alone and no video: two FAT partitions that overlap, one that passes the card's end, a file
  system that runs past its partition (an M4-era card whose partition 1 was shrunk in the table and given a
  partition 2 without being formatted again; the FAT16 one, by its 16-bit size), and a partition boot sector that
  will not read;
* unreadable, no card: a sector 0 that will not read (the board then mounts nothing rather than guess one partition
  and let video fill the recordings).
Needs cc, util-linux (sfdisk) and dosfstools (mkfs.fat); without them the test says so and passes.
"""

import json
import os
import shutil
import subprocess
import sys
import tempfile

HERE: str = os.path.dirname(os.path.abspath(__file__))
ROOT: str = os.path.normpath(os.path.join(HERE, '..', '..', '..'))
COMPONENT: str = os.path.join(ROOT, 'src', 'avionics', 'firmware', 'components', 'sd_card')
PREPARE: str = os.path.join(ROOT, 'tools', 'sd_prepare.sh')
SECTOR: int = 512
MIB: int = 1 << 20
MBR_TABLE: int = 446  # the four 16-byte entries; each has its type at 4, its start at 8 and its size at 12

_SCRATCH = tempfile.TemporaryDirectory(prefix='coludo-sd-partition-table-')
HARNESS: str = os.path.join(_SCRATCH.name, 'partition_table_host')


def _build() -> None:
    """Compile partition_table.c and its host harness, warnings as errors."""
    subprocess.run(['cc', '-std=c11', '-D_FILE_OFFSET_BITS=64', '-D_POSIX_C_SOURCE=200809L', '-Wall', '-Wextra',
                    '-Werror', '-pedantic', '-o', HARNESS, os.path.join(COMPONENT, 'test', 'partition_table_host.c'),
                    os.path.join(COMPONENT, 'partition_table.c')], check=True)


def _judge(path: str, *unreadable: int) -> tuple:
    """
    The board's judgment of the card image at `path`.

    Args:
        path - the image; its size is the card's.
        unreadable - sectors whose read fails.

    Returns:
        (verdict, ((start, sectors), (start, sectors)), reason): the verdict `unreadable`, `one`, `two` or `broken`,
        the two partitions as judged ((0, 0) each when the verdict names none), and the reason the board logs.
    """
    result = subprocess.run([HARNESS, path, *map(str, unreadable)], capture_output=True, text=True, check=True)
    verdict, start1, sectors1, start2, sectors2, reason = result.stdout.rstrip('\n').split(';')
    return verdict, ((int(start1), int(sectors1)), (int(start2), int(sectors2))), reason


def _image(name: str, size: int) -> str:
    """A sparse, empty card image of `size` bytes in the scratch directory; returns its path."""
    path = os.path.join(_SCRATCH.name, name)
    with open(path, 'wb') as image:
        image.truncate(size)
    return path


def _table(path: str, *partitions: tuple) -> None:
    """An MBR on the image with `partitions`, each (start, sectors, type as sfdisk takes it), by sfdisk."""
    lines = ''.join('start=%d, size=%d, type=%s\n' % partition for partition in partitions)
    subprocess.run(['sfdisk', '--quiet', path], input='label: dos\nunit: sectors\n' + lines, text=True, check=True,
                   capture_output=True)


def _format(path: str, start: int, sectors: int, fat: int) -> None:
    """A FAT file system (`fat` 16 or 32) at sector `start` of the image, `sectors` long, as sd_prepare makes one."""
    subprocess.run(['mkfs.fat', '-F', str(fat), '-I', '--mbr=n', '--offset', str(start), '-h', str(start), '-s', '1',
                    path, str(sectors // 2)], check=True, capture_output=True)


def _sector(path: str, sector: int) -> bytearray:
    """The 512 bytes of the image's sector `sector`."""
    with open(path, 'rb') as image:
        image.seek(sector * SECTOR)
        return bytearray(image.read(SECTOR))


def _write_sector(path: str, sector: int, data: bytes) -> None:
    """Writes `data`, 512 bytes, over the image's sector `sector`."""
    with open(path, 'r+b') as image:
        image.seek(sector * SECTOR)
        image.write(data)


def _entry(mbr: bytearray, index: int, kind: int | None = None, start: int | None = None,
           sectors: int | None = None) -> bytearray:
    """
    A copy of the MBR `mbr` with entry `index` (0..3) changed where a value is given.

    Args:
        mbr - sector 0.
        index - the entry.
        kind - its type byte.
        start - its first sector.
        sectors - its size in sectors.

    Returns:
        The changed sector 0.
    """
    changed = bytearray(mbr)
    entry = MBR_TABLE + index * 16
    if kind is not None:
        changed[entry + 4] = kind
    if start is not None:
        changed[entry + 8:entry + 12] = start.to_bytes(4, 'little')
    if sectors is not None:
        changed[entry + 12:entry + 16] = sectors.to_bytes(4, 'little')
    return changed


def test_a_prepared_card_has_two() -> None:
    """
    A card sd_prepare.sh prepared is judged two, its partitions where sfdisk reads them; its table changed is judged
    one, or broken, as the board must.
    """
    path = _image('prepared.img', 4 << 30)
    prepared = subprocess.run([PREPARE, '--recordings-gib', '1', path], input='4096\n', capture_output=True,
                              text=True, timeout=120, cwd=_SCRATCH.name)
    assert prepared.returncode == 0, (prepared.stdout, prepared.stderr)
    dump = subprocess.run(['sfdisk', '--json', path], capture_output=True, text=True, check=True)
    planned = tuple((partition['start'], partition['size'])
                    for partition in json.loads(dump.stdout)['partitiontable']['partitions'])
    assert _judge(path) == ('two', planned, 'partitions 1 and 2, both FAT'), _judge(path)
    (start1, sectors1), (start2, sectors2) = planned
    card = (4 << 30) // SECTOR
    mbr = _sector(path, 0)
    cases = (
        # a partition 2 of a type that is not FAT (Linux): one volume, FatFs's own search
        (_entry(mbr, 1, kind=0x83), 'one', 'partitions 1 and 2 are not both FAT'),
        # no signature: no table at all
        (mbr[:510] + b'\x00\x00', 'one', 'no partition table at sector 0'),
        # partition 2 over partition 1, and partition 2 inside partition 1's end
        (_entry(mbr, 1, start=start1), 'broken', 'partitions 1 and 2 overlap'),
        (_entry(mbr, 0, sectors=sectors1 + 1), 'broken', 'partitions 1 and 2 overlap'),
        # partition 2 past the card's end
        (_entry(mbr, 1, sectors=card - start2 + 1), 'broken', "a partition passes the card's end"),
        # partition 1 shrunk in the table, its file system not formatted again: it runs into what follows
        (_entry(mbr, 0, sectors=sectors1 // 2), 'broken', 'the file system on partition 1 runs past it'),
        (_entry(mbr, 1, sectors=sectors2 - 8192), 'broken', 'the file system on partition 2 runs past it'),
    )
    for sector0, verdict, reason in cases:
        _write_sector(path, 0, sector0)
        judged = _judge(path)
        assert judged[0] == verdict and judged[2].startswith(reason), (verdict, reason, judged)
    # partitions in the table's other order, apart and whole: two, partition 1 being the one listed first
    swapped = bytearray(mbr)
    swapped[MBR_TABLE:MBR_TABLE + 16], swapped[MBR_TABLE + 16:MBR_TABLE + 32] = mbr[MBR_TABLE + 16:MBR_TABLE + 32], \
        mbr[MBR_TABLE:MBR_TABLE + 16]
    _write_sector(path, 0, swapped)
    assert _judge(path) == ('two', planned[::-1], 'partitions 1 and 2, both FAT'), _judge(path)
    _write_sector(path, 0, mbr)
    # reads that fail: sector 0 is no card at all, a partition's boot sector is a table that cannot be trusted
    assert _judge(path, 0) == ('unreadable', ((0, 0), (0, 0)), 'sector 0 would not read'), _judge(path, 0)
    assert _judge(path, start1) == ('broken', planned, "partition 1's boot sector would not read")
    assert _judge(path, start2) == ('broken', planned, "partition 2's boot sector would not read")
    # a partition 2 with no file system yet: two by the table, the mount refuses it on the board
    boot2 = _sector(path, start2)
    _write_sector(path, start2, bytes(SECTOR))
    assert _judge(path) == ('two', planned, 'partitions 1 and 2, both FAT'), _judge(path)
    _write_sector(path, start2, boot2)
    assert _judge(path)[0] == 'two', 'the image is as prepared again'


def test_small_fat16_partitions() -> None:
    """
    Two FAT16 partitions (types 0x06 and 0x0e) of 16 and 32 MiB, sized by the boot sector's 16-bit field: two; with
    partition 1 shrunk in the table, its file system runs past it.
    """
    path = _image('fat16.img', 64 * MIB)
    first, second = (2048, 16 * MIB // SECTOR, '6'), (2048 + 16 * MIB // SECTOR, 32 * MIB // SECTOR, 'e')
    _table(path, first, second)
    for start, sectors, _ in (first, second):
        _format(path, start, sectors, 16)
    assert int.from_bytes(_sector(path, first[0])[19:21], 'little') == first[1], 'the 16-bit size field is in use'
    partitions = ((first[0], first[1]), (second[0], second[1]))
    assert _judge(path) == ('two', partitions, 'partitions 1 and 2, both FAT'), _judge(path)
    _write_sector(path, 0, _entry(_sector(path, 0), 0, sectors=first[1] - 2048))
    assert _judge(path)[2] == 'the file system on partition 1 runs past it, into what follows', _judge(path)


def test_one_volume_cards() -> None:
    """A superfloppy, a card with one FAT partition (an M4-era card) and an empty card mount as one volume."""
    superfloppy = _image('superfloppy.img', 64 * MIB)
    subprocess.run(['mkfs.fat', '-F', '32', '-I', '-s', '1', superfloppy], check=True, capture_output=True)
    judged = _judge(superfloppy)
    assert judged[:2] == ('one', ((0, 0), (0, 0))) and 'superfloppy' in judged[2], judged
    single = _image('single.img', 64 * MIB)
    _table(single, (8192, 100000, 'c'))
    _format(single, 8192, 100000, 32)
    assert _judge(single) == ('one', ((0, 0), (0, 0)), 'partitions 1 and 2 are not both FAT'), _judge(single)
    empty = _image('empty.img', 64 * MIB)
    assert _judge(empty) == ('one', ((0, 0), (0, 0)), 'no partition table at sector 0'), _judge(empty)
    # negative: the harness's own reader fails a read past the image's end, as a card past its capacity does
    assert _judge(_image('tiny.img', 100))[0] == 'unreadable'


missing = [tool for tool in ('cc', 'sfdisk', 'mkfs.fat') if shutil.which(tool) is None]
if missing:
    print('note: %s not installed, so the board\'s partition table judgment is not tested' % ', '.join(missing))
    sys.exit(0)
_build()
test_a_prepared_card_has_two()
test_small_fat16_partitions()
test_one_volume_cards()
print('ok: partition_table.c on the host -- an sd_prepare.sh card and two FAT16 partitions are two; a superfloppy, no '
      'table, one partition and a non-FAT partition 2 are one; overlapping partitions, one past the card, a file '
      'system past its partition and an unreadable boot sector are broken (partition 1 alone); an unreadable sector 0 '
      'is no card')
