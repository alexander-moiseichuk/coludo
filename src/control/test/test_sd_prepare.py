"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Host (CPython) test for tools/sd_prepare.sh, the operator's microSD preparation for a Rust board
(doc/specs/avionics-port.md "SD and flash file layout", Card preparation): discard the card, two primary FAT32
partitions over --use (25..100 %, default 75) of the card from 4 MiB, split in halves (METRICS, the recordings, the
first; VIDEOS, the videos, the rest), the rest untouched; or, with --partitions 1 (comparing the layouts), one METRICS
partition over the same area with the same clusters.

The script destroys whatever it is pointed at, so this test points it ONLY at plain image files in its own temporary
directory, sparse so that an 8 GiB "card" costs a few MB: no device node is ever named, and _run() refuses any path
outside that directory before the script sees it. No root is needed: sfdisk and mkfs.fat write an image as they write
a card, and "discarding" an image empties it. How it judges a real block device is tested without one: the script,
sourced, only defines its functions, and its judge_device() then runs against stand-ins for lsblk and blockdev and
a sysfs tree of the test's own (wipefs and blkdiscard stand-ins abort the run, should anything ever reach them).

What a prepared image must hold, read here independently of the script's own read-back: the MBR's two partitions
(start, size, type 0x0c each, partition 2 right behind partition 1) from sfdisk's JSON; the FAT32 boot sector at each
partition's offset (the type string, the signature, the hidden sectors, the cluster size, the label and the volume
serial the script printed, the two serials different); fsck.fat's verdict over each partition; and no data anywhere
past the partitions (SEEK_DATA), so the junk planted at the card's end was discarded and nothing was written into
the spare area. The negatives: a dry run and a wrong or missing confirmation (the device's name included: the
confirmation is the card's size in MiB) leave the image byte for byte as it was; so does another image put in the
card's place while the confirmation waits, or the card's growing then; a missing device, a directory, a card too small
for two halves of 512 MiB and usage errors (a share with a leading zero among them, which the shell's arithmetic would
read as octal; the retired --recordings-gib) are refused before anything is written;
and of the block devices, an eMMC or an unknown kind in an MMC slot, a disk that is not removable (a USB backup SSD),
a partition, a disk over 256 GiB, a mounted partition and one held by a mapper are refused, while an SD card in a slot
or a removable card in a reader is taken, its identity following the kernel's disk sequence number. Needs util-linux
(sfdisk) and dosfstools (mkfs.fat, fsck.fat), as the script does; without them the test says so and passes.
"""

import errno
import hashlib
import json
import os
import re
import select
import shutil
import subprocess
import sys
import tempfile
import time

HERE: str = os.path.dirname(os.path.abspath(__file__))
ROOT: str = os.path.normpath(os.path.join(HERE, '..', '..', '..'))
SCRIPT: str = os.path.join(ROOT, 'tools', 'sd_prepare.sh')
SECTOR: int = 512
ALIGN: int = 8192  # 4 MiB in sectors: where the partition starts, and the grid its end is on
GIB: int = 1 << 30
JUNK: bytes = b'an old filesystem, which the preparation must discard ' * 64

_SCRATCH = tempfile.TemporaryDirectory(prefix='coludo-sd-prepare-')


def _image(name: str, size: int) -> str:
    """
    A sparse card image in the scratch directory, with junk at its start, in its middle and at its end.

    Args:
        name - the file name.
        size - the card's size, bytes.

    Returns:
        The image's path.
    """
    path = os.path.join(_SCRATCH.name, name)
    with open(path, 'wb') as image:
        image.truncate(size)
        for offset in (0, size // 2, size - len(JUNK)):
            image.seek(offset)
            image.write(JUNK)
    return path


def _digest(path: str) -> str:
    """
    The image's SHA-256 over its data segments and where they lie (SEEK_DATA/SEEK_HOLE), and its size: any write
    changes it, even of zeros into a hole, and the holes of a sparse image are never read.
    """
    digest = hashlib.sha256(str(os.path.getsize(path)).encode())
    with open(path, 'rb') as image:
        descriptor = image.fileno()
        offset = 0
        while True:
            try:
                offset = os.lseek(descriptor, offset, os.SEEK_DATA)
            except OSError as error:
                assert error.errno == errno.ENXIO, error
                return digest.hexdigest()
            end = os.lseek(descriptor, offset, os.SEEK_HOLE)
            image.seek(offset)
            digest.update(b'@%d:' % offset + image.read(end - offset))
            offset = end


def _mebibytes(path: str) -> str:
    """The confirmation for the image at `path`: its size in MiB, as the plan's card: line shows it."""
    return str(os.path.getsize(path) >> 20)


def _in_scratch(arguments: tuple) -> None:
    """Every argument that is not an option, read as a path from the scratch directory, lies in it."""
    for argument in arguments:
        if not argument.startswith('-'):
            real = os.path.realpath(os.path.join(_SCRATCH.name, argument))
            assert real.startswith(_SCRATCH.name + os.sep) and not real.startswith('/dev'), argument


def _run(*arguments: str, answer: str | None = None) -> subprocess.CompletedProcess:
    """
    Run sd_prepare.sh, never on anything but this test's own files.

    Args:
        arguments - the script's arguments. It runs in the scratch directory, and every argument that is not an
            option, read as a path from there, must lie in it: an option's value too ('25' is the file ./25).
        answer - what is typed at the confirmation; None types nothing (end of input).

    Returns:
        The finished run.
    """
    _in_scratch(arguments)
    return subprocess.run([SCRIPT, *arguments], input='' if answer is None else answer + '\n', capture_output=True,
                          text=True, timeout=120, cwd=_SCRATCH.name)


def _partitions(path: str) -> list:
    """The image's partitions as sfdisk reads them: [{start, size, type}, ...]."""
    dump = subprocess.run(['sfdisk', '--json', path], capture_output=True, text=True, check=True)
    return json.loads(dump.stdout)['partitiontable']['partitions']


def _boot_sector(path: str, start: int) -> bytes:
    """The 512 bytes at sector `start`, a partition's offset."""
    with open(path, 'rb') as image:
        image.seek(start * SECTOR)
        return image.read(SECTOR)


def _check_prepared(path: str, size: int, use: int, result: subprocess.CompletedProcess, clusters_kib: tuple) -> None:
    """
    Check a prepared image: the table, both boot sectors, fsck.fat over each partition, and nothing past them.

    Args:
        path - the image.
        size - its size, bytes.
        use - the share asked, percent.
        result - the script's run.
        clusters_kib - the cluster sizes the two partitions' sizes call for, KiB: (recordings, videos).
    """
    assert result.returncode == 0, (result.returncode, result.stdout, result.stderr)
    assert os.path.getsize(path) == size, 'the card changed size'
    end = size // SECTOR * use // 100 // ALIGN * ALIGN
    recordings = (end - ALIGN) // 2 // ALIGN * ALIGN  # the first half, on the 4 MiB grid
    planned = ((ALIGN, recordings), (ALIGN + recordings, end - ALIGN - recordings))
    partitions = _partitions(path)
    table = [(partition['start'], partition['size'], partition['type']) for partition in partitions]
    assert table == [(start, sectors, 'c') for start, sectors in planned], partitions
    serials = (re.search(r'METRICS, volume serial ([0-9A-F]{4})-([0-9A-F]{4}) \(the board', result.stdout),
               re.search(r'and FAT32 VIDEOS, volume serial ([0-9A-F]{4})-([0-9A-F]{4})', result.stdout))
    assert all(serials), result.stdout
    assert serials[0].groups() != serials[1].groups(), 'one serial for both volumes'
    for (start, sectors), serial, label, cluster_kib in zip(planned, serials, (b'METRICS    ', b'VIDEOS     '),
                                                            clusters_kib):
        boot = _boot_sector(path, start)
        assert boot[82:90] == b'FAT32   ' and boot[510:512] == b'\x55\xaa', (label, boot[:96])
        assert int.from_bytes(boot[28:32], 'little') == start, (label, 'hidden sectors')
        assert boot[13] * SECTOR == cluster_kib * 1024, (label, boot[13])
        assert int.from_bytes(boot[32:36], 'little') <= sectors, (label, 'the filesystem runs past its partition')
        assert int.from_bytes(boot[67:71], 'little') == int(serial.group(1) + serial.group(2), 16), boot[67:71]
        assert boot[71:82] == label, boot[71:82]
    with open(path, 'rb') as image:
        mbr = image.read(SECTOR)
        assert mbr[510:512] == b'\x55\xaa', 'no MBR'
        assert JUNK not in mbr, 'the old start survived'
        try:
            data = os.lseek(image.fileno(), end * SECTOR, os.SEEK_DATA)
        except OSError as error:
            assert error.errno == errno.ENXIO, error
        else:
            raise AssertionError('data at byte %d, past the partition: the spare area was written' % data)
    for start, sectors in planned:
        partition = path + '.partition'
        subprocess.run(['dd', 'if=' + path, 'of=' + partition, 'bs=4M', 'skip=%d' % (start // ALIGN),
                        'count=%d' % (sectors // ALIGN), 'conv=sparse', 'status=none'], check=True)
        fsck = subprocess.run(['fsck.fat', '-n', partition], capture_output=True, text=True)
        assert fsck.returncode == 0, (start, fsck.stdout, fsck.stderr)
        os.remove(partition)


def test_prepares_three_quarters_of_a_card() -> None:
    """
    A 16 GiB card at the defaults: 75 % in two halves -- 6140 MiB of recordings, 6144 MiB of videos -- 32 KiB clusters
    on both, everything past 12 GiB untouched (and the junk gone).
    """
    path = _image('card16.img', 16 * GIB)
    result = _run(path, answer='16384')
    _check_prepared(path, 16 * GIB, 75, result, (32, 32))
    assert 'card:      %s (a card image of %d bytes), 16384 MiB' % (path, 16 * GIB) in result.stdout, result.stdout
    assert 'EVERYTHING ON %s IS LOST.' % path in result.stdout, result.stdout
    assert '1, the recordings: sectors 8192..12582911, 6140 MiB, label METRICS' in result.stdout, result.stdout
    assert '2, the videos: sectors 12582912..25165823, 6144 MiB, label VIDEOS' in result.stdout, result.stdout
    # prepared again, it is a new card: other volume serials
    again = _run(path, answer='16384')
    _check_prepared(path, 16 * GIB, 75, again, (32, 32))
    first, second = (re.search(r'volume serial (\S+) \(the board', run.stdout).group(1) for run in (result, again))
    assert first != second, 'two preparations gave one serial: the board would take the new card for the old one'


def test_prepares_one_partition_over_the_same_area() -> None:
    """
    --partitions 1 (comparing the layouts on cards prepared the same way): one FAT32 partition, METRICS, from 4 MiB to
    the share asked -- the area the two would cover -- with the same 32 KiB clusters, fsck-clean, nothing past it.
    """
    path = _image('card16one.img', 16 * GIB)
    result = _run('--partitions', '1', path, answer='16384')
    assert result.returncode == 0, (result.returncode, result.stdout, result.stderr)
    end = 16 * GIB // SECTOR * 75 // 100 // ALIGN * ALIGN
    partitions = _partitions(path)
    assert [(partition['start'], partition['size'], partition['type']) for partition in partitions] == \
        [(ALIGN, end - ALIGN, 'c')], partitions
    serial = re.search(r'METRICS, volume serial ([0-9A-F]{4})-([0-9A-F]{4}) \(the board', result.stdout)
    assert serial and 'the recordings and the videos on one volume' in result.stdout, result.stdout
    assert 'one FAT32 partition' in result.stdout and 'VIDEOS' not in result.stdout, result.stdout
    boot = _boot_sector(path, ALIGN)
    assert boot[82:90] == b'FAT32   ' and boot[510:512] == b'\x55\xaa', boot[:96]
    assert boot[13] * SECTOR == 32 * 1024, boot[13]
    assert int.from_bytes(boot[67:71], 'little') == int(serial.group(1) + serial.group(2), 16), boot[67:71]
    assert boot[71:82] == b'METRICS    ', boot[71:82]
    with open(path, 'rb') as image:
        try:
            data = os.lseek(image.fileno(), end * SECTOR, os.SEEK_DATA)
        except OSError as error:
            assert error.errno == errno.ENXIO, error
        else:
            raise AssertionError('data at byte %d, past the partition' % data)
    partition = path + '.partition'
    subprocess.run(['dd', 'if=' + path, 'of=' + partition, 'bs=4M', 'skip=1', 'count=%d' % ((end - ALIGN) // ALIGN),
                    'conv=sparse', 'status=none'], check=True)
    fsck = subprocess.run(['fsck.fat', '-n', partition], capture_output=True, text=True)
    assert fsck.returncode == 0, (fsck.stdout, fsck.stderr)
    os.remove(partition)
    # the same card prepared with two again: the two-partition layout, as before
    again = _run(path, answer='16384')
    _check_prepared(path, 16 * GIB, 75, again, (32, 32))


def test_prepares_a_quarter_with_smaller_clusters() -> None:
    """
    An 8 GiB card at 25 %: halves of about 1 GiB (1020 and 1024 MiB), too few 32 KiB clusters for FAT32, so 8 KiB ones.
    """
    path = _image('card8.img', 8 * GIB)
    result = _run('--use', '25', path, answer='8192')
    _check_prepared(path, 8 * GIB, 25, result, (8, 8))
    assert result.stdout.count('clusters of 8 KiB') == 2, result.stdout


def test_dry_run_and_refused_confirmations_write_nothing() -> None:
    """
    A dry run, a wrong confirmation and no confirmation at all leave the image byte for byte as it was: the device's
    name is no confirmation, only the card's size in MiB is.
    """
    path = _image('untouched.img', 16 * GIB)
    before = _digest(path)
    result = _run('--dry-run', path)
    assert result.returncode == 0 and 'dry run: nothing written' in result.stdout, (result.stdout, result.stderr)
    assert 'to go on' not in result.stdout, 'a dry run must not ask'
    for answer in ('yes', path, os.path.basename(path), '16384 ', '16383', '16 GiB', None):
        result = _run(path, answer=answer)
        assert result.returncode == 1, (answer, result.returncode, result.stdout, result.stderr)
        assert 'nothing written' in result.stderr, (answer, result.stderr)
    assert _digest(path) == before, 'a refused run wrote to the card'


def test_refusals() -> None:
    """Usage errors exit 2, refusals exit 1, and neither writes anything."""
    path = _image('refused.img', 16 * GIB)
    small = _image('small.img', 4 * GIB)  # 25 % is 1024 MiB: 4 MiB, then halves of 508 and 512 MiB
    tiny = _image('tiny.img', 1 * GIB)  # 75 % is 768 MiB: halves of 380 and 384 MiB
    before = (_digest(path), _digest(small), _digest(tiny))
    directory = os.path.join(_SCRATCH.name, 'a-directory')
    os.mkdir(directory)
    cases = (
        ((), 2, 'name the card: there is no default device'),
        (('--use', '24', path), 2, '--use takes 25..100 percent, not 24'),
        (('--use', '101', path), 2, '--use takes 25..100 percent, not 101'),
        (('--use', 'half', path), 2, '--use takes 25..100 percent, not half'),
        (('--use',), 2, '--use needs a value'),
        # a leading zero: the shell's arithmetic reads it as octal (040 would plan 32 %, 08 would not parse)
        (('--use', '040', path), 2, '--use takes 25..100 percent, not 040'),
        (('--use', '08', path), 2, '--use takes 25..100 percent, not 08'),
        # the halves are the operator's split now: the old fixed recordings size is no option
        (('--recordings-gib', '4', path), 2, 'unknown option --recordings-gib'),
        (('--bogus', path), 2, 'unknown option --bogus'),
        (('--partitions', '3', path), 2, '--partitions takes 1 or 2, not 3'),
        (('--partitions', 'one', path), 2, '--partitions takes 1 or 2, not one'),
        (('--partitions',), 2, '--partitions needs a value'),
        ((path, small), 2, 'one device only'),
        ((os.path.join(_SCRATCH.name, 'no-such-card.img'),), 1, 'neither a block device nor a card image file'),
        ((directory,), 1, 'neither a block device nor a card image file'),
        (('--use', '25', small), 1,
         'is too small: 25 % of it leaves 508 MiB for the recordings and 512 MiB for the videos'),
        ((tiny,), 1, 'is too small: 75 % of it leaves 380 MiB for the recordings and 384 MiB for the videos'),
    )
    for arguments, code, reason in cases:
        # the answer that would confirm, should a refusal ever let the run reach the prompt
        target = os.path.join(_SCRATCH.name, arguments[-1]) if arguments else ''
        result = _run(*arguments, answer=_mebibytes(target) if os.path.isfile(target) else None)
        assert result.returncode == code, (arguments, result.returncode, result.stdout, result.stderr)
        assert reason in result.stderr, (arguments, reason, result.stderr)
    assert (_digest(path), _digest(small), _digest(tiny)) == before, 'a refused run wrote to a card'
    assert not os.path.exists(os.path.join(_SCRATCH.name, 'no-such-card.img')), 'a refused run made a file'


def _prompted(path: str) -> subprocess.Popen:
    """
    Start sd_prepare.sh on the image at `path` and wait for its confirmation prompt.

    Returns:
        The running script, waiting for the answer on its stdin.
    """
    _in_scratch((path,))
    script = subprocess.Popen([SCRIPT, path], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                              cwd=_SCRATCH.name)
    shown = b''
    deadline = time.monotonic() + 60
    while not shown.endswith(b'to go on: '):
        assert time.monotonic() < deadline and script.poll() is None, shown
        if select.select([script.stdout], [], [], 1)[0]:
            shown += os.read(script.stdout.fileno(), 4096)
    return script


def test_a_card_swapped_while_the_confirmation_waits() -> None:
    """
    The card checked again after the confirmation: another image in its place, or the same one grown, is refused
    with nothing written, though the answer is the planned card's size.
    """
    path = os.path.join(_SCRATCH.name, 'swapped.img')
    for swap in ('another image', 'grown'):
        _image('swapped.img', 16 * GIB)
        script = _prompted(path)
        if swap == 'another image':
            os.replace(_image('other.img', 16 * GIB), path)  # the reader's next card: the same size
        else:
            os.truncate(path, 17 * GIB)
        before = _digest(path)
        _, errors = script.communicate('16384\n'.encode(), timeout=60)
        assert script.returncode == 1, (swap, script.returncode, errors)
        assert b'is not the card planned any more' in errors and b'nothing written' in errors, (swap, errors)
        assert _digest(path) == before, 'the card put in the planned one\'s place was written (%s)' % swap


def _judge(kname: str, kind: str = 'disk', removable: str = '1', bytes_: int = 32 * GIB, tree: str = '',
           card_type: str | None = None, diskseq: int = 7) -> subprocess.CompletedProcess:
    """
    Source sd_prepare.sh and run its judge_device() on a stand-in block device: lsblk and blockdev answer for it, and
    a sysfs tree of this test's own describes it. No device node is named, and nothing the script could write with
    is real.

    Args:
        kname - the kernel's name for the device (sdc, mmcblk0).
        kind - what lsblk calls it (disk, part).
        removable - lsblk's RM, '1' or '0'.
        bytes_ - its size.
        tree - lsblk's PATH,TYPE,MOUNTPOINT lines for the device and its partitions; '' is the bare device.
        card_type - sysfs device/type of an MMC device (SD, MMC); None leaves the file out.
        diskseq - sysfs diskseq.

    Returns:
        The run: on success its stdout holds `bytes=` and `identity=` lines.
    """
    sysfs = os.path.join(_SCRATCH.name, 'sys')
    shutil.rmtree(sysfs, ignore_errors=True)
    os.makedirs(os.path.join(sysfs, 'block', kname, 'device'))
    with open(os.path.join(sysfs, 'block', kname, 'diskseq'), 'w') as handle:
        handle.write('%d\n' % diskseq)
    if card_type is not None:
        with open(os.path.join(sysfs, 'block', kname, 'device', 'type'), 'w') as handle:
            handle.write(card_type + '\n')
    program = """
        source "$SCRIPT"
        sysfs=$FAKE_SYSFS
        lsblk() {
            case $2 in
                KNAME,TYPE,RM) echo "$FAKE_KNAME $FAKE_TYPE $FAKE_RM" ;;
                PATH,TYPE,MOUNTPOINT) printf '%s\n' "${FAKE_TREE:-fake-card disk }" ;;
                SIZE,TRAN,MODEL) echo '29.7G  usb   Card Reader' ;;
                MAJ:MIN,SERIAL,MODEL) echo '8:32 000000000039 Card\\x20Reader' ;;
                *) echo "lsblk $* was not expected" >&2; exit 99 ;;
            esac
        }
        blockdev() {
            [ "$1" = --getsize64 ] || { echo "blockdev $* was not expected" >&2; exit 99; }
            echo "$FAKE_BYTES"
        }
        wipefs() { echo 'wipefs reached' >&2; exit 99; }
        blkdiscard() { echo 'blkdiscard reached' >&2; exit 99; }
        judge_device fake-card
        echo "bytes=$bytes"
        echo "identity=$identity"
    """
    environment = dict(os.environ, SCRIPT=SCRIPT, FAKE_SYSFS=sysfs, FAKE_KNAME=kname, FAKE_TYPE=kind,
                       FAKE_RM=removable, FAKE_BYTES=str(bytes_), FAKE_TREE=tree)
    return subprocess.run(['bash', '-c', program, 'judge'], capture_output=True, text=True, timeout=60,
                          cwd=_SCRATCH.name, env=environment)


def test_devices_are_judged_by_the_kernel() -> None:
    """Only an SD card in a slot, or removable media in a reader, of at most 256 GiB and in nobody's use passes."""
    taken = (
        dict(kname='sdc'),  # a card in a USB reader
        dict(kname='mmcblk0', removable='0', card_type='SD'),  # a card in a slot, which the kernel may call fixed
    )
    for case in taken:
        result = _judge(**case)
        assert result.returncode == 0, (case, result.stdout, result.stderr)
        assert 'bytes=%d' % (32 * GIB) in result.stdout, (case, result.stdout)
    refused = (
        (dict(kname='mmcblk0', removable='0', card_type='MMC'), 'is MMC storage in an MMC slot, not an SD card'),
        (dict(kname='mmcblk1', card_type=None), 'is unknown storage in an MMC slot, not an SD card'),
        (dict(kname='sdb', removable='0', bytes_=240 * 10**9), 'is not removable media'),  # a USB backup SSD
        (dict(kname='sdc1', kind='part'), 'is a part, not a whole card'),
        (dict(kname='sdc', bytes_=300 * GIB), 'holds 300 GiB'),
        (dict(kname='sdc', tree='fake-card disk \nfake-card1 part /media/card'),
         'fake-card1 is mounted at /media/card'),
        (dict(kname='sdc', tree='fake-card disk \nfake-card1 part \nluks-card crypt '),
         'luks-card holds a crypt device'),
    )
    for case, reason in refused:
        result = _judge(**case)
        assert result.returncode == 1 and reason in result.stderr, (case, result.returncode, result.stderr)
        assert 'reached' not in result.stderr and 'was not expected' not in result.stderr, (case, result.stderr)
    # another card in the same reader: the same name, size, serial and model, only the disk sequence number moves on
    identities = [re.search(r'identity=(.*)', _judge(kname='sdc', diskseq=diskseq).stdout).group(1)
                  for diskseq in (7, 8)]
    assert identities[0] != identities[1] and 'diskseq 7' in identities[0], identities


missing = [tool for tool in ('sfdisk', 'mkfs.fat', 'fsck.fat', 'dd') if shutil.which(tool) is None]
if missing:
    print('note: %s not installed, so sd_prepare.sh is not tested' % ', '.join(missing))
    sys.exit(0)
test_prepares_three_quarters_of_a_card()
test_prepares_one_partition_over_the_same_area()
test_prepares_a_quarter_with_smaller_clusters()
test_dry_run_and_refused_confirmations_write_nothing()
test_refusals()
test_a_card_swapped_while_the_confirmation_waits()
test_devices_are_judged_by_the_kernel()
print('ok: sd_prepare.sh -- on image files only: two FAT32 partitions, METRICS and VIDEOS halving the share asked '
      '(75 % by default), or one METRICS over the same area (fsck clean, the serials printed are the ones '
      'written, nothing past them); dry '
      'runs, wrong confirmations, a card swapped at the prompt, a card too small and bad input write nothing; block '
      'devices judged on stand-ins: eMMC, fixed disks, partitions, big, mounted and held devices refused')
