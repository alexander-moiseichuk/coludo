/* Coludo project, copyright under MIT license, Alexander Moiseichuk */
/* The microSD card on the WaveShare ESP32-P4-WIFI6 (SDMMC slot 0), shared host with the C6 (slot 1). */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

/* How the card's FAT volumes are mounted (doc/specs/avionics-port.md "SD and flash file layout"). */
typedef enum {
    /* No card mounted. */
    SD_CARD_LAYOUT_NONE = 0,
    /* One volume at the base path: a superfloppy, an MBR with one FAT partition, or a second partition that is not
     * FAT. The first FAT volume FatFs finds is mounted; the videos directory is a plain directory on it. */
    SD_CARD_LAYOUT_ONE_PARTITION = 1,
    /* Partition 1 at the base path (the recordings), partition 2 at the videos path (the videos). */
    SD_CARD_LAYOUT_TWO_PARTITIONS = 2,
    /* Partition 1 at the base path, and nothing at the videos path: partition 2 is FAT by its MBR type but would not
     * mount, or the two partitions overlap, pass the card's end or hold a file system larger than themselves (the
     * boot log says which). Video stays off rather than filling the recordings partition (the ring_saver's). */
    SD_CARD_LAYOUT_VIDEOS_FAILED = 3,
} sd_card_layout_t;

/* A mounted volume of the card, for sd_card_space. */
typedef enum {
    /* The volume at the base path: partition 1, or a one-partition card's only volume. */
    SD_CARD_VOLUME_RECORDINGS = 0,
    /* The videos partition, at the videos path (SD_CARD_LAYOUT_TWO_PARTITIONS only). */
    SD_CARD_VOLUME_VIDEOS = 1,
} sd_card_volume_t;

/* Power the card (GPIO45 low: Q1, a P-channel MOSFET, switches ESP_3V3 to the card; a 10k pull-down keeps it on
 * by default), initialise it once, and mount its FAT volumes: partition 1 at `base_path` and, when the MBR's
 * partitions 1 and 2 are both FAT and sound (apart, on the card, each file system within its partition:
 * partition_table.h), partition 2 at `videos_path` (a path under `base_path`: IDF's VFS takes the longest matching
 * prefix, so it shadows that directory of partition 1). Never formats. Mount once and never unmount: an
 * unmount deinits the slot, and while the C6 holds slot 1 that is all it does -- but there is no reason to risk it.
 * On failure only slot 0 is torn down (IDF's default host deinits per slot), the C6 link is untouched; a sector 0
 * that will not read in three tries is a failure (ESP_FAIL), not a card without a table. A second partition that
 * will not mount, or a table whose two FAT partitions are unsound, leaves the first mounted alone
 * (SD_CARD_LAYOUT_VIDEOS_FAILED). */
esp_err_t sd_card_mount(const char *base_path, const char *videos_path);

/* How the card is mounted (sd_card_layout_t); SD_CARD_LAYOUT_NONE before a mount or after a failed one. */
sd_card_layout_t sd_card_layout(void);

/* The FAT volume serial number of the volume at the base path, partition 1 (what `blkid` shows as
 * UUID="XXXX-XXXX", mkfs.fat's -i): it tells one card from another, so a boot knows whether the card is the one the
 * previous boot saved to (the ring_saver's hint is trusted only for that card). Read from the boot sector once, at
 * boot (CONFIG_FATFS_USE_LABEL brings f_getlabel). ESP_ERR_INVALID_STATE when no card is mounted, ESP_FAIL when the
 * boot sector would not read. */
esp_err_t sd_card_volume_serial(uint32_t *serial);

/* The FAT volume serial number of the videos partition, as sd_card_volume_serial reads partition 1's: for the boot
 * log. ESP_ERR_INVALID_STATE unless the layout is SD_CARD_LAYOUT_TWO_PARTITIONS, ESP_FAIL when it would not read. */
esp_err_t sd_card_videos_serial(uint32_t *serial);

/* The size of mounted volume `volume` and what is free on it, in bytes, as esp_vfs_fat_info sums them (the FAT's
 * clusters), read by its FatFs drive: vfs_fat cannot look the videos volume up by its 14-character base path (it
 * keeps it cut to 13). Slow: it takes the volume's lock, and its first call scans the FAT. ESP_ERR_INVALID_STATE when
 * that volume is not mounted, ESP_FAIL when FatFs would not tell. */
esp_err_t sd_card_space(sd_card_volume_t volume, uint64_t *total_bytes, uint64_t *free_bytes);

/* Print the mounted card's identity, speed and capacity to the console. */
void sd_card_print_info(void);

/* Card power: false cuts it (GPIO45 high) -- the recovery for a hung card, followed by a remount. */
void sd_card_power(bool on);
