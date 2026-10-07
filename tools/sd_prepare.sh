#!/bin/bash
# Coludo project, copyright under MIT license, Alexander Moiseichuk
#
# Prepare a microSD card for a Rust (src/avionics) board, the operator's pre-cook (doc/specs/avionics-port.md "SD and
# flash file layout", Card preparation). DESTRUCTIVE: everything on the card is gone afterwards.
#
#   tools/sd_prepare.sh [--dry-run] [--use PERCENT] <device>
#
# What it does, in this order:
#   1. wipes the card's old signatures and discards the whole card (wipefs, blkdiscard), so its controller knows every
#      block is free; a reader that cannot pass a discard on (many USB readers) only costs that knowledge, and the
#      script says so and goes on;
#   2. writes an MBR with ONE primary partition, type 0x0c (FAT32, LBA), from 4 MiB (the card's usual allocation-unit
#      alignment) up to PERCENT of the card, rounded down to 4 MiB (--use, 25..50, default 50);
#   3. formats it FAT32 (label COLUDO, a fresh random volume serial, 32 KiB clusters, fewer when a small partition
#      needs them to stay FAT32), written in place at the partition's offset in the device itself;
#   4. reads the table and the boot sector back and checks them.
# The rest of the card stays unpartitioned and untouched: spare area for the card's controller.
#
# One partition, not yet the spec's two (the recordings, then the videos): the firmware mounts the first partition
# only and keeps its video/ directory there, so a recordings partition of a few GB would fill with video (the
# default 8 Mbit/s is 3.6 GB an hour) and stop the ring_saver. A card prepared so runs on the spec's one-partition
# fallback; prepare it again, once its sessions are pulled, when the firmware mounts the second partition.
#
# The firmware mounts the first partition and never formats a card (components/sd_card). The volume serial printed
# at the end is how the board names the card (the session-start record's `card`, the boot log's `sd :: mounted card
# XXXX-XXXX`): the ring_saver resumes at its saved position only on the card it was saved to, so a card prepared
# again counts as a new card, and gets everything still in the board's flash.
#
# Refused, before anything is written:
#   * anything but an explicit device: there is no default;
#   * a block device that is not a whole disk (a partition, a loop, dm, md or zram device);
#   * a block device that is not a card: in an MMC slot, anything the kernel does not call an SD card (soldered eMMC,
#     a board's or a laptop's own storage, reads MMC); anywhere else, anything that is not removable media (a card
#     in a reader is; a USB, SATA or NVMe disk is not, a USB backup SSD included);
#   * a card larger than 256 GiB (no card this board uses), or one that holds a mounted filesystem, active swap, or a
#     mapper/RAID holder on it or on any of its partitions;
#   * a card too small for a FAT32 partition of 512 MiB at the share asked;
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
#   --dry-run    every check and the plan, then exit without writing or asking
#   --use N      the share of the card to partition, 25..50 percent (default 50)
#
# Exit codes: 0 done (or a dry run), 1 refused or failed, 2 a usage error. Sourced rather than run (as the test does),
# it only defines its functions.
set -euo pipefail
usage='usage: sd_prepare.sh [--dry-run] [--use PERCENT] <device>'
sector=512
align=8192                  # 4 MiB in sectors: the partition's start and its end are multiples of it
smallest=$((512 << 20))     # the smallest partition, bytes
largest=$((256 << 30))      # the largest card, bytes
label=COLUDO
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
    local dry_run=0 use=50 device='' tool total end size cluster serial shown mebibytes answer planned formatted \
        dump boot fat_type signature read_serial hidden
    while [ $# -gt 0 ]; do
        case $1 in
            --dry-run) dry_run=1; shift ;;
            --use)
                [ $# -ge 2 ] || misuse '--use needs a value'
                use=$2
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
    [[ $use =~ ^[0-9]+$ ]] && [ "$use" -ge 25 ] && [ "$use" -le 50 ] || misuse "--use takes 25..50 percent, not $use"
    for tool in sfdisk mkfs.fat od; do
        command -v "$tool" >/dev/null || fail "$tool is not installed (util-linux, dosfstools, coreutils)"
    done
    judge "$device"

    # the plan: one partition from 4 MiB up to the share asked, both ends on the 4 MiB grid
    total=$((bytes / sector))
    end=$((total * use / 100 / align * align))
    size=$((end - align))
    [ $((size * sector)) -ge "$smallest" ] ||
        fail "$device is too small: $use % of it leaves $((size * sector >> 20)) MiB, FAT32 here needs" \
            "$((smallest >> 20)) MiB"
    # 32 KiB clusters (what the SD Association formats 8-32 GB cards with), halved while FAT32's 65525 clusters
    # would not fit (the margin covers the reserved sectors and the FATs)
    cluster=64
    while [ $((size / cluster)) -lt 66000 ] && [ "$cluster" -gt 8 ]; do
        cluster=$((cluster / 2))
    done
    if [ -n "${SRANDOM:-}" ]; then
        serial=$(printf '%08X' "$SRANDOM")
    else
        serial=$(printf '%04X%04X' "$((RANDOM & 0xFFFF))" "$(((RANDOM << 1 ^ RANDOM) & 0xFFFF))")
    fi
    shown="${serial:0:4}-${serial:4:4}"
    mebibytes=$((bytes >> 20))

    echo "card:      $device ($description), $mebibytes MiB"
    echo "plan:      discard all $mebibytes MiB, then an MBR with one FAT32 partition (type 0x0c):"
    echo "           sectors $align..$((end - 1)), $((size * sector >> 20)) MiB, $use % of the card"
    echo "           label $label, volume serial $shown, clusters of $((cluster * sector / 1024)) KiB"
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
    printf 'label: dos\nunit: sectors\nstart=%d, size=%d, type=c\n' "$align" "$size" |
        sfdisk --quiet --wipe always --wipe-partitions always "$device" >/dev/null ||
        fail "sfdisk could not write the partition table"

    # 3. FAT32 in place at the partition's offset in the card itself: no partition node and no udev wait, the same on
    #    a card and on an image (-I: mkfs.fat would refuse a whole card; --mbr=n: sector 0 is sfdisk's; -h: the boot
    #    sector counts the sectors before the partition). Its output is shown only when it fails: it notes, every
    #    time, that the card holds more blocks than the partition takes
    if ! formatted=$(mkfs.fat -F 32 -I --mbr=n --offset "$align" -h "$align" -s "$cluster" -n "$label" -i "$serial" \
        "$device" $((size / 2)) 2>&1); then
        fail "mkfs.fat could not format the partition:
$formatted"
    fi
    sync
    if [ "$kind" = device ]; then
        blockdev --flushbufs "$device"
        # the kernel's view of the new table; the card is written either way
        blockdev --rereadpt "$device" 2>/dev/null || true
    fi

    # 4. read back: one partition where planned, and a FAT32 boot sector there that names the serial
    dump=$(sfdisk --dump "$device")
    grep -Eq "start= *$align, size= *$size, type=c$" <<<"$dump" || fail "the partition table did not read back as written:
$dump"
    [ "$(grep -c 'start=' <<<"$dump")" = 1 ] || fail "the partition table holds more than one partition:
$dump"
    boot=$((align * sector))
    fat_type=$(od -An -c -j $((boot + 82)) -N 8 "$device" | tr -d ' ')
    signature=$(od -An -tx1 -j $((boot + 510)) -N 2 "$device" | tr -d ' ')
    read_serial=$(od -An -tx4 -j $((boot + 67)) -N 4 "$device" | tr -d ' ' | tr 'a-f' 'A-F')
    hidden=$(od -An -tu4 -j $((boot + 28)) -N 4 "$device" | tr -d ' ')
    [ "$fat_type" = FAT32 ] && [ "$signature" = 55aa ] || fail "no FAT32 boot sector at sector $align"
    [ "$read_serial" = "$serial" ] || fail "the volume serial read back as $read_serial, not $serial"
    [ "$hidden" = "$align" ] || fail "the boot sector counts $hidden hidden sectors, not $align"
    echo "done:      $device holds FAT32 $label, volume serial $shown (the board names the card by it)"
}

# run as a command; sourced, only the functions above are defined
if [ "${BASH_SOURCE[0]}" = "$0" ]; then
    main "$@"
fi
