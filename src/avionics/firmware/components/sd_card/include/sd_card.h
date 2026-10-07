/* Coludo project, copyright under MIT license, Alexander Moiseichuk */
/* The microSD card on the WaveShare ESP32-P4-WIFI6 (SDMMC slot 0), shared host with the C6 (slot 1). */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

/* Power the card (GPIO45 low: Q1, a P-channel MOSFET, switches ESP_3V3 to the card; a 10k pull-down keeps it on
 * by default) and mount its FAT filesystem at `base_path`. Never formats. Mount once and never unmount: an
 * unmount deinits the slot, and while the C6 holds slot 1 that is all it does -- but there is no reason to risk
 * it. On failure only slot 0 is torn down (IDF's default host deinits per slot), the C6 link is untouched. */
esp_err_t sd_card_mount(const char *base_path);

/* The mounted card's FAT volume serial number (what `blkid` shows as UUID="XXXX-XXXX", mkfs.fat's -i): it tells
 * one card from another, so a boot knows whether the card is the one the previous boot saved to (the ring_saver's
 * hint is trusted only for that card). Read from the boot sector once, at boot (CONFIG_FATFS_USE_LABEL brings
 * f_getlabel). ESP_ERR_INVALID_STATE when no card is mounted, ESP_FAIL when the boot sector would not read. */
esp_err_t sd_card_volume_serial(uint32_t *serial);

/* Print the mounted card's identity, speed and capacity to the console. */
void sd_card_print_info(void);

/* Card power: false cuts it (GPIO45 high) -- the recovery for a hung card, followed by a remount. */
void sd_card_power(bool on);
