#!/bin/bash
# Coludo project, copyright under MIT license, Alexander Moiseichuk
#
# Prepare a microSD card for a Rust (src/avionics) board, the operator's pre-cook (doc/specs/avionics-port.md "SD and
# flash file layout", Card preparation). DESTRUCTIVE: everything on the card is gone afterwards.
#
#   tools/sd_prepare.sh [--dry-run] [--use PERCENT] [--partitions 1|2] <device>
#
# What it does, in this order:
#   1. wipes the card's old signatures and discards the whole card (wipefs, blkdiscard), so its controller knows every
#      block is free; a reader that cannot pass a discard on (many USB readers) only costs that knowledge, and the
#      script says so and goes on;
#   2. writes an MBR with TWO primary partitions, both type 0x0c (FAT32, LBA), over PERCENT of the card (--use, 25..100,
#      default 75), from 4 MiB (the card's usual allocation-unit alignment) and rounded down to 4 MiB, split in two
#      halves (the operator, 2026-10-10): partition 1, the recordings, the first half; partition 2, the videos, the
#      rest. The logs use half and the video half: measured -> reported writes every sample (about 50 KB/s of text at
#      the M9 rates, ~180 MB an hour), the default 8 Mbit/s of video ~3.6 GB an hour. --partitions 1 writes ONE
#      partition instead, the recordings and the videos together over the same area, for comparing the layouts on one
#      card prepared the same way (the same discard, alignment and cluster size);
#   3. formats each FAT32, written in place at the partition's offset in the device itself: partition 1 labelled
#      METRICS (the only one with --partitions 1), partition 2 VIDEOS, each with a fresh random volume serial, 32 KiB
#      clusters (fewer when a small partition needs them to stay FAT32);
#   4. reads the table and both boot sectors back and checks them.
# The rest of the card stays unpartitioned and untouched: spare area for the card's controller.
#
# Why two (doc/specs/avionics-port.md "SD and flash file layout"): the firmware mounts partition 1 at /sdcard and
# partition 2 at /sdcard/videos (components/sd_card), so video and the ring_saver each have a volume of their own --
# its own lock, which a 64 KB video write held 23 ms on average, and its own FAT, which a power cut mid-video cannot
# tear for the saved recordings. The firmware finds the partitions by number; the labels are for people. A card with
# one FAT partition (an M4-era card, or one formatted elsewhere) still runs, with videos/ a plain directory on it.
#
# The firmware never formats a card. Partition 1's volume serial, printed at the end, is how the board names the
# card (the session-start record's `card`, the boot log's `sd :: mounted card XXXX-XXXX`): the ring_saver resumes at
# its saved position only on the card it was saved to, so a card prepared again counts as a new card, and gets
# everything still in the board's flash.
#
# Refused, before anything is written:
#   * anything but an explicit device: there is no default;
#   * a block device that is not a whole disk (a partition, a loop, dm, md or zram device);
#   * a block device that is not a card: in an MMC slot, anything the kernel does not call an SD card (soldered eMMC,
#     a board's or a laptop's own storage, reads MMC); anywhere else, anything that is not removable media (a card
#     in a reader is; a USB, SATA or NVMe disk is not, a USB backup SSD included);
#   * a card larger than 256 GiB (no card this board uses), or one that holds a mounted filesystem, active swap, or a
#     mapper/RAID holder on it or on any of its partitions;
#   * a card too small for two halves of at least 512 MiB each (FAT32's least here) at the share asked;
#   * a confirmation that is not the card's size in MiB as the plan's `card:` line shows it: typing it back means
#     reading the line that says which card it is (size, transport, model), where typing the device's name back
#     would only repeat a wrong name;
#   * a card that is not the one planned once the confirmation came: the prompt waits as long as it takes, and by
#     then the name may hold another device, or the reader another card. Every check above runs again on what the
#     name holds then, and its identity (major:minor, the kernel's disk sequence number, which every new medium
#     bumps, size, serial, model) must be the planned one.
# A plain file is taken as a card image (the tests prepare an image this way: no root, no device); it must not be
# attached to a loop device, and its identity is its file system, inode and size. "Discarding" an image empties it
# to a sparse file of its size.
#
#   --dry-run            every check and the plan, then exit without writing or asking
#   --use N              the share of the card to partition, both partitions together, 25..100 percent (default 75)
#   --partitions 1|2     two partitions, the recordings and the videos half each (default), or one over the same area
# --use takes a plain decimal number: a leading zero is refused (the shell's arithmetic would read 010 as octal).
#
# Exit codes: 0 done (or a dry run), 1 refused or failed, 2 a usage error. Sourced rather than run (as the test does),
# it only defines its functions.
set -euo pipefail
usage='usage: sd_prepare.sh [--dry-run] [--use PERCENT] [--partitions 1|2] <device>'
sector=512
align=8192                  # 4 MiB in sectors: each partition's start and its end are multiples of it
smallest=$((512 << 20))     # the smallest partition, bytes
largest=$((256 << 30))      # the largest card, bytes
recordings_label=METRICS    # partition 1, mounted at /sdcard
videos_label=VIDEOS         # partition 2, mounted at /sdcard/videos
sysfs=/sys                  # the kernel's view of the block devices (the test points it at a tree of its own)

misuse() {
    # a usage error: say what was wrong, and how it is called
    echo "sd_prepare.sh: $1" >&2
    echo "$usage" >&2
    exit 2
}

fail() {
    # a refusal or a failure: the reason (its arguments, joined by a space), exit 1
    echo "sd_prepare.sh: $*" >&2
    exit 1
}

judge_device() {
    # the block device $1 as the card to prepare: refused unless it is one (see the header); sets bytes, description
    # and identity
    local tool kname type removable card_type path child mountpoint
    for tool in lsblk wipefs blkdiscard blockdev; do
        command -v "$tool" >/dev/null || fail "$tool is not installed (util-linux)"
    done
    read -r kname type removable < <(lsblk -dnro KNAME,TYPE,RM "$1")
    [ "$type" = disk ] || fail "$1 is a $type, not a whole card: name the card itself (/dev/sdX, /dev/mmcblkN)"
    case $kname in
        mmcblk*)
            # an MMC slot holds an SD card or soldered eMMC, and the kernel says which: SD, or MMC
            card_type=$(cat "$sysfs/block/$kname/device/type" 2>/dev/null) || card_type='unknown'
            [ "$card_type" = SD ] ||
                fail "$1 is $card_type storage in an MMC slot, not an SD card: it looks like built-in eMMC"
            ;;
        *)
            [ "$removable" = 1 ] ||
                fail "$1 is not removable media, as a card in a reader is: it looks like a system or backup disk"
            ;;
    esac
    bytes=$(blockdev --getsize64 "$1")
    [ "$bytes" -le "$largest" ] ||
        fail "$1 holds $((bytes >> 30)) GiB: no card this board uses is larger than $((largest >> 30)) GiB"
    # the card and every partition on it: mounted (swap shows as [SWAP]), or held by a mapper or a RAID
    while read -r path child mountpoint; do
        [ -z "$mountpoint" ] || fail "$path is mounted at $mountpoint: unmount it first"
        case $child in
            disk | part) ;;
            *) fail "$path holds a $child device: it is in use" ;;
        esac
    done < <(lsblk -nro PATH,TYPE,MOUNTPOINT "$1")
    description=$(lsblk -dno SIZE,TRAN,MODEL "$1" | tr -s ' ')
    identity="$(lsblk -dnro MAJ:MIN,SERIAL,MODEL "$1") diskseq $(cat "$sysfs/block/$kname/diskseq" 2>/dev/null ||
        echo none) bytes $bytes"
}

judge_image() {
    # the plain file $1 as a card image: refused when a loop device holds it; sets bytes, description and identity
    if command -v losetup >/dev/null && [ -n "$(losetup -j "$1" 2>/dev/null)" ]; then
        fail "$1 is attached to a loop device: detach it first"
    fi
    bytes=$(stat -c %s "$1")
    description="a card image of $bytes bytes"
    identity=$(stat -c 'file %d:%i bytes %s' "$1")
}

clusters_for() {
    # the cluster size in sectors for a FAT32 volume of $1 sectors: 32 KiB (what the SD Association formats 8-32 GB
    # cards with), halved while FAT32's 65525 clusters would not fit (the margin covers the reserved sectors and the
    # FATs)
    local cluster=64
    while [ $(($1 / cluster)) -lt 66000 ] && [ "$cluster" -gt 8 ]; do
        cluster=$((cluster / 2))
    done
    echo "$cluster"
}

random_serial() {
    # a fresh random FAT volume serial, eight hex digits
    if [ -n "${SRANDOM:-}" ]; then
        printf '%08X' "$SRANDOM"
    else
        printf '%04X%04X' "$((RANDOM & 0xFFFF))" "$(((RANDOM << 1 ^ RANDOM) & 0xFFFF))"
    fi
}

format_partition() {
    # FAT32 in place at sector $2 of the card $1, $3 sectors, $4 sectors a cluster, label $5, serial $6: no partition
    # node and no udev wait, the same on a card and on an image (-I: mkfs.fat would refuse a whole card; --mbr=n:
    # sector 0 is sfdisk's; -h: the boot sector counts the sectors before the partition). Its output is shown only
    # when it fails: it notes, every time, that the card holds more blocks than the partition takes
    local formatted
    if ! formatted=$(mkfs.fat -F 32 -I --mbr=n --offset "$2" -h "$2" -s "$4" -n "$5" -i "$6" "$1" $(($3 / 2)) 2>&1)
    then
        fail "mkfs.fat could not format the $5 partition:
$formatted"
    fi
}

check_boot_sector() {
    # the FAT32 boot sector at sector $2 of the card $1 names serial $3 and counts $2 hidden sectors; else fails
    local boot fat_type signature read_serial hidden
    boot=$(($2 * sector))
    fat_type=$(od -An -c -j $((boot + 82)) -N 8 "$1" | tr -d ' ')
    signature=$(od -An -tx1 -j $((boot + 510)) -N 2 "$1" | tr -d ' ')
    read_serial=$(od -An -tx4 -j $((boot + 67)) -N 4 "$1" | tr -d ' ' | tr 'a-f' 'A-F')
    hidden=$(od -An -tu4 -j $((boot + 28)) -N 4 "$1" | tr -d ' ')
    [ "$fat_type" = FAT32 ] && [ "$signature" = 55aa ] || fail "no FAT32 boot sector at sector $2"
    [ "$read_serial" = "$3" ] || fail "the volume serial at sector $2 read back as $read_serial, not $3"
    [ "$hidden" = "$2" ] || fail "the boot sector at sector $2 counts $hidden hidden sectors, not $2"
}

judge() {
    # $1 as the card to prepare, a block device or a card image: sets kind, bytes, description and identity, or
    # refuses
    if [ -b "$1" ]; then
        kind=device
        judge_device "$1"
    elif [ -f "$1" ]; then
        kind=image
        judge_image "$1"
    else
        fail "$1 is neither a block device nor a card image file"
    fi
}

main() {
    local dry_run=0 use=75 partitions=2 device='' tool total end \
        recordings_size videos_start videos_size recordings_cluster videos_cluster recordings_serial videos_serial \
        shown videos_shown mebibytes answer planned dump
    while [ $# -gt 0 ]; do
        case $1 in
            --dry-run) dry_run=1; shift ;;
            --use)
                [ $# -ge 2 ] || misuse '--use needs a value'
                use=$2
                shift 2
                ;;
            --partitions)
                [ $# -ge 2 ] || misuse '--partitions needs a value'
                partitions=$2
                shift 2
                ;;
            --help | -h) echo "$usage"; exit 0 ;;
            -*) misuse "unknown option $1" ;;
            *)
                [ -z "$device" ] || misuse "one device only (got $device and $1)"
                device=$1
                shift
                ;;
        esac
    done
    [ -n "$device" ] || misuse 'name the card: there is no default device'
    # decimal digits with no leading zero: [ -ge ] reads 010 as ten, but $(( )) reads it as octal eight (and 08 not
    # at all), so a leading zero would plan other sizes than the ones checked and shown
    [[ $use =~ ^[1-9][0-9]*$ ]] && [ "$use" -ge 25 ] && [ "$use" -le 100 ] ||
        misuse "--use takes 25..100 percent, not $use"
    [ "$partitions" = 1 ] || [ "$partitions" = 2 ] || misuse "--partitions takes 1 or 2, not $partitions"
    for tool in sfdisk mkfs.fat od; do
        command -v "$tool" >/dev/null || fail "$tool is not installed (util-linux, dosfstools, coreutils)"
    done
    judge "$device"

    # the plan: the share asked from 4 MiB, split in halves on the 4 MiB grid, the recordings first and the videos the
    # rest (the operator: the logs half, the video half); with one partition, it alone from 4 MiB up to the share
    total=$((bytes / sector))
    end=$((total * use / 100 / align * align))
    if [ "$partitions" = 1 ]; then
        recordings_size=$((end - align))
        [ $((recordings_size * sector)) -ge "$smallest" ] ||
            fail "$device is too small: $use % of it leaves" \
                "$((recordings_size > 0 ? recordings_size * sector >> 20 : 0)) MiB, FAT32 here needs" \
                "$((smallest >> 20)) MiB"
        videos_start=$end
        videos_size=0
    else
        recordings_size=$(((end > align ? end - align : 0) / 2 / align * align))
        videos_start=$((align + recordings_size))
        videos_size=$((end > videos_start ? end - videos_start : 0))
        [ $((recordings_size * sector)) -ge "$smallest" ] && [ $((videos_size * sector)) -ge "$smallest" ] ||
            fail "$device is too small: $use % of it leaves $((recordings_size * sector >> 20)) MiB for the" \
                "recordings and $((videos_size * sector >> 20)) MiB for the videos, FAT32 here needs" \
                "$((smallest >> 20)) MiB each"
    fi
    recordings_cluster=$(clusters_for "$recordings_size")
    videos_cluster=$(clusters_for "$((videos_size > 0 ? videos_size : recordings_size))")
    recordings_serial=$(random_serial)
    videos_serial=$(random_serial)
    while [ "$videos_serial" = "$recordings_serial" ]; do
        videos_serial=$(random_serial)
    done
    shown="${recordings_serial:0:4}-${recordings_serial:4:4}"
    videos_shown="${videos_serial:0:4}-${videos_serial:4:4}"
    mebibytes=$((bytes >> 20))

    echo "card:      $device ($description), $mebibytes MiB"
    if [ "$partitions" = 1 ]; then
        echo "plan:      discard all $mebibytes MiB, then an MBR with one FAT32 partition (type 0x0c)," \
            "$use % of the card:"
        echo "           1, the recordings and the videos (videos/ a directory): sectors $align..$((end - 1))," \
            "$((recordings_size * sector >> 20)) MiB, label $recordings_label, volume serial $shown," \
            "clusters of $((recordings_cluster * sector / 1024)) KiB"
    else
        echo "plan:      discard all $mebibytes MiB, then an MBR with two FAT32 partitions (type 0x0c)," \
            "$use % of the card:"
        echo "           1, the recordings: sectors $align..$((videos_start - 1))," \
            "$((recordings_size * sector >> 20)) MiB, label $recordings_label, volume serial $shown," \
            "clusters of $((recordings_cluster * sector / 1024)) KiB"
        echo "           2, the videos: sectors $videos_start..$((end - 1)), $((videos_size * sector >> 20)) MiB," \
            "label $videos_label, volume serial $videos_shown, clusters of $((videos_cluster * sector / 1024)) KiB"
    fi
    echo "untouched: sectors $end..$((total - 1)), $(((total - end) * sector >> 20)) MiB," \
        "spare area for the card's controller"
    echo "EVERYTHING ON $device IS LOST."
    if [ "$dry_run" = 1 ]; then
        echo 'dry run: nothing written'
        exit 0
    fi
    printf "check the card: line, then type the card's size in MiB from it to go on: "
    IFS= read -r answer || answer='' # exactly as typed: no space trimmed
    [ "$answer" = "$mebibytes" ] ||
        fail "the confirmation was '$answer', not the card's size in MiB: nothing written"
    # the card again: the prompt waited as long as it took, and the name may hold another device by now
    planned=$identity
    judge "$device"
    [ "$identity" = "$planned" ] ||
        fail "$device is not the card planned any more (it was $planned, it is $identity): nothing written"

    # 1. discard everything: the controller learns every block is free, and no old partition table or filesystem
    #    signature survives anywhere on the card. The card's own signatures (its partition table) go first: run from
    #    a terminal, blkdiscard refuses a device that still carries one unless forced, and forcing it would also drop
    #    its exclusive open, the last guard against a device something holds
    if [ "$kind" = device ]; then
        wipefs --all --quiet "$device" || fail "wipefs could not clear $device's old partition table"
        if ! blkdiscard "$device"; then
            echo "sd_prepare.sh: $device took no discard (a USB reader may not pass one on): going on without it" >&2
        fi
    else
        truncate -s 0 "$device"
        truncate -s "$bytes" "$device"
    fi

    # 2. the partition table; the signatures sfdisk finds are wiped as well (a discard may leave the old bytes
    #    readable)
    {
        printf 'label: dos\nunit: sectors\nstart=%d, size=%d, type=c\n' "$align" "$recordings_size"
        [ "$partitions" = 1 ] || printf 'start=%d, size=%d, type=c\n' "$videos_start" "$videos_size"
    } | sfdisk --quiet --wipe always --wipe-partitions always "$device" >/dev/null ||
        fail "sfdisk could not write the partition table"

    # 3. each partition FAT32, in place at its offset in the card itself
    format_partition "$device" "$align" "$recordings_size" "$recordings_cluster" "$recordings_label" \
        "$recordings_serial"
    [ "$partitions" = 1 ] ||
        format_partition "$device" "$videos_start" "$videos_size" "$videos_cluster" "$videos_label" "$videos_serial"
    sync
    if [ "$kind" = device ]; then
        blockdev --flushbufs "$device"
        # the kernel's view of the new table; the card is written either way
        blockdev --rereadpt "$device" 2>/dev/null || true
    fi

    # 4. read back: the partitions where planned, in that order, and a FAT32 boot sector at each naming its serial
    dump=$(sfdisk --dump "$device")
    grep -Eq "1 : start= *$align, size= *$recordings_size, type=c$" <<<"$dump" &&
        { [ "$partitions" = 1 ] || grep -Eq "2 : start= *$videos_start, size= *$videos_size, type=c$" <<<"$dump"; } ||
        fail "the partition table did not read back as written:
$dump"
    [ "$(grep -c 'start=' <<<"$dump")" = "$partitions" ] || fail "the partition table holds other partitions:
$dump"
    check_boot_sector "$device" "$align" "$recordings_serial"
    if [ "$partitions" = 1 ]; then
        echo "done:      $device holds FAT32 $recordings_label, volume serial $shown" \
            "(the board names the card by it)," \
            "the recordings and the videos on one volume"
    else
        check_boot_sector "$device" "$videos_start" "$videos_serial"
        echo "done:      $device holds FAT32 $recordings_label, volume serial $shown" \
            "(the board names the card by it)," \
            "and FAT32 $videos_label, volume serial $videos_shown"
    fi
}

# run as a command; sourced, only the functions above are defined
if [ "${BASH_SOURCE[0]}" = "$0" ]; then
    main "$@"
fi
