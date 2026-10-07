/* Coludo project, copyright under MIT license, Alexander Moiseichuk */
/*
 * The card's partition table as the board judges it before it mounts anything (doc/specs/avionics-port.md "SD and
 * flash file layout"): one FAT volume or two, from sector 0 and the two partitions' boot sectors. Plain C with no
 * IDF in it, so the firmware (sd_card.c) and a host test (src/control/test/test_sd_partition_table.py, against card
 * images tools/sd_prepare.sh and mkfs.fat write) build the same file.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

/* The only sector size judged: a card with other sectors is mounted as one volume, unjudged. */
#define PARTITION_TABLE_SECTOR_BYTES 512

/* One primary MBR partition: its first sector and its length, in sectors. */
typedef struct {
    uint32_t start;
    uint32_t sectors;
} partition_t;

/* What the card holds, as partition_table_read judges it. */
typedef enum {
    /* Sector 0 would not read, after the reader's retries: there is no card to mount. */
    PARTITION_TABLE_UNREADABLE = 0,
    /* One volume, wherever FatFs's own search finds it: a superfloppy (a FAT boot sector at sector 0), no MBR at all,
     * one FAT partition, or partitions 1 and 2 not both of a FAT type (an M4-era card, one formatted elsewhere). */
    PARTITION_TABLE_ONE = 1,
    /* Partitions 1 and 2, both of a FAT type, apart from each other and within the card, and each FAT file system
     * found at their starts within its own partition. A partition whose start holds no FAT boot sector is left to the
     * mount, which fails on it. */
    PARTITION_TABLE_TWO = 2,
    /* Partitions 1 and 2 are both of a FAT type, but they overlap, one passes the card's end, the file system at one's
     * start runs past that partition, or a partition's boot sector would not read. Two volumes would share sectors, or
     * write past the card: partition 1 is mounted alone (so the recordings go on), and there is no video. */
    PARTITION_TABLE_BROKEN = 3,
} partition_table_t;

/* Reads the card's sector `sector` (PARTITION_TABLE_SECTOR_BYTES bytes) into `buffer`; false when it would not read.
 * `context` is the caller's, passed through. */
typedef bool (*partition_table_reader_t)(void *context, uint32_t sector, uint8_t *buffer);

/* Judges the card: sector 0, then, for two FAT partitions, where they lie and the boot sector at each one's start.
 *
 * Args:
 *   read - reads one sector of the card.
 *   context - passed to `read`.
 *   card_sectors - the card's size, in sectors.
 *   partitions - set to MBR entries 1 and 2 for PARTITION_TABLE_TWO and PARTITION_TABLE_BROKEN, untouched otherwise.
 *   reason - set to why the card is judged so, a static text for the boot log.
 *
 * Returns what the card holds. */
partition_table_t partition_table_read(partition_table_reader_t read, void *context, uint32_t card_sectors,
                                       partition_t partitions[2], const char **reason);
