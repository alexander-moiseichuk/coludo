/* Coludo project, copyright under MIT license, Alexander Moiseichuk */
/*
 * The card's partition table as the board judges it (partition_table.h). FatFs's own reading of a card is the model
 * (IDF v5.5.5 components/fatfs/src/ff.c: check_fs, find_volume): what it calls a FAT boot sector, where it finds the
 * table. What FatFs never checks is checked here, before two volumes are mounted on one card: whether the two
 * partitions lie apart and within the card, and whether the file system at each one's start fits in it. FatFs takes a
 * volume's size from its boot sector alone, so two FATFS objects over shared sectors would each allocate clusters the
 * other owns: the video writer would overwrite recordings.
 */

#include "partition_table.h"

#include <string.h>

/* The MBR as FatFs reads it (ff.c): the table of four 16-byte entries at 446, the signature 0x55AA at 510. */
#define MBR_TABLE 446
#define MBR_ENTRY_BYTES 16
#define MBR_ENTRY_TYPE 4
#define MBR_ENTRY_START 8
#define MBR_ENTRY_SECTORS 12
#define BOOT_SIGNATURE 510
/* A FAT boot sector's size of the volume in sectors (ff.c BPB_TotSec16, BPB_TotSec32): the 16-bit field, or the
 * 32-bit one when that is 0, as FatFs's mount reads it. */
#define BOOT_TOTAL_SECTORS_16 19
#define BOOT_TOTAL_SECTORS_32 32

static uint16_t read_word(const uint8_t *bytes)
{
    return (uint16_t)(bytes[0] | bytes[1] << 8);
}

static uint32_t read_double_word(const uint8_t *bytes)
{
    return (uint32_t)bytes[0] | (uint32_t)bytes[1] << 8 | (uint32_t)bytes[2] << 16 | (uint32_t)bytes[3] << 24;
}

/* Whether `sector` is a FAT boot sector, as FatFs's check_fs() judges one (exFAT is off in this build): at sector 0
 * that makes the card a superfloppy (no partition table), and what looks like a table at 446 is boot code. */
static bool is_fat_boot_sector(const uint8_t *sector)
{
    uint8_t jump = sector[0];
    if (jump != 0xEB && jump != 0xE9 && jump != 0xE8) {
        return false;
    }
    if (read_word(sector + BOOT_SIGNATURE) == 0xAA55 && memcmp(sector + 82, "FAT32   ", 8) == 0) {
        return true;
    }
    /* a FAT12/16 volume from an early DOS may lack the signature and the type string: its parameter block tells */
    uint16_t bytes_per_sector = read_word(sector + 11);
    uint8_t sectors_per_cluster = sector[13];
    return (bytes_per_sector & (bytes_per_sector - 1)) == 0 && bytes_per_sector >= 512 && bytes_per_sector <= 4096 &&
           sectors_per_cluster != 0 && (sectors_per_cluster & (sectors_per_cluster - 1)) == 0 &&
           read_word(sector + 14) != 0 && (unsigned)sector[16] - 1 <= 1 && read_word(sector + 17) != 0 &&
           (read_word(sector + BOOT_TOTAL_SECTORS_16) >= 4 ||
            read_double_word(sector + BOOT_TOTAL_SECTORS_32) >= 0x10000) &&
           read_word(sector + 22) != 0;
}

/* The size in sectors of the FAT volume whose boot sector is `boot`, as FatFs's mount takes it. */
static uint32_t read_volume_sectors(const uint8_t *boot)
{
    uint16_t small = read_word(boot + BOOT_TOTAL_SECTORS_16);
    return small != 0 ? small : read_double_word(boot + BOOT_TOTAL_SECTORS_32);
}

/* MBR entry `index` (0..3) of `sector`. */
static partition_t entry_of(const uint8_t *sector, int index)
{
    const uint8_t *entry = sector + MBR_TABLE + index * MBR_ENTRY_BYTES;
    return (partition_t){
        .start = read_double_word(entry + MBR_ENTRY_START),
        .sectors = read_double_word(entry + MBR_ENTRY_SECTORS),
    };
}

/* Whether MBR entry `index` (0..3) of `sector` is a FAT partition that holds sectors. */
static bool is_fat_partition(const uint8_t *sector, int index)
{
    partition_t partition = entry_of(sector, index);
    switch (sector[MBR_TABLE + index * MBR_ENTRY_BYTES + MBR_ENTRY_TYPE]) {
    case 0x01: /* FAT12 */
    case 0x04: /* FAT16 under 32 MiB */
    case 0x06: /* FAT16 */
    case 0x0b: /* FAT32, CHS */
    case 0x0c: /* FAT32, LBA: what tools/sd_prepare.sh writes */
    case 0x0e: /* FAT16, LBA */
        return partition.start != 0 && partition.sectors != 0;
    default:
        return false;
    }
}

partition_table_t partition_table_read(partition_table_reader_t read, void *context, uint32_t card_sectors,
                                       partition_t partitions[2], const char **reason)
{
    static const char *const unread[2] = {
        "partition 1's boot sector would not read",
        "partition 2's boot sector would not read",
    };
    static const char *const overrun[2] = {
        "the file system on partition 1 runs past it, into what follows",
        "the file system on partition 2 runs past it, into what follows",
    };
    uint8_t sector[PARTITION_TABLE_SECTOR_BYTES];
    if (!read(context, 0, sector)) {
        *reason = "sector 0 would not read";
        return PARTITION_TABLE_UNREADABLE;
    }
    if (is_fat_boot_sector(sector)) {
        *reason = "a FAT file system at sector 0, no partition table (a superfloppy)";
        return PARTITION_TABLE_ONE;
    }
    if (read_word(sector + BOOT_SIGNATURE) != 0xAA55) {
        *reason = "no partition table at sector 0";
        return PARTITION_TABLE_ONE;
    }
    if (!is_fat_partition(sector, 0) || !is_fat_partition(sector, 1)) {
        *reason = "partitions 1 and 2 are not both FAT";
        return PARTITION_TABLE_ONE;
    }
    partitions[0] = entry_of(sector, 0);
    partitions[1] = entry_of(sector, 1);
    /* where they lie: the ends in 64 bits, as a start plus a size may pass 2^32 */
    uint64_t ends[2];
    for (int index = 0; index < 2; index++) {
        ends[index] = (uint64_t)partitions[index].start + partitions[index].sectors;
    }
    if (ends[0] > partitions[1].start && ends[1] > partitions[0].start) {
        *reason = "partitions 1 and 2 overlap";
        return PARTITION_TABLE_BROKEN;
    }
    if (ends[0] > card_sectors || ends[1] > card_sectors) {
        *reason = "a partition passes the card's end";
        return PARTITION_TABLE_BROKEN;
    }
    /* what each holds: a FAT file system there must end within it (one that is not FAT is the mount's to refuse) */
    for (int index = 0; index < 2; index++) {
        if (!read(context, partitions[index].start, sector)) {
            *reason = unread[index];
            return PARTITION_TABLE_BROKEN;
        }
        if (is_fat_boot_sector(sector) && read_volume_sectors(sector) > partitions[index].sectors) {
            *reason = overrun[index];
            return PARTITION_TABLE_BROKEN;
        }
    }
    *reason = "partitions 1 and 2, both FAT";
    return PARTITION_TABLE_TWO;
}
