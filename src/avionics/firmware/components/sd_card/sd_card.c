/* Coludo project, copyright under MIT license, Alexander Moiseichuk */

#include "sd_card.h"

#include <stdio.h>

#include "driver/gpio.h"
#include "driver/sdmmc_host.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "ff.h"
#include "diskio_sdmmc.h" /* after ff.h: it names FatFs's BYTE without including it */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sd_pwr_ctrl_by_on_chip_ldo.h"
#include "sdmmc_cmd.h"

static const char *TAG = "sd_card";

/* Q1's gate: low = card powered (schematic: SI2301CDS, R22 10k to GND) */
#define POWER_GPIO 45
/* ESP_LDO_VO4 feeds the bus pull-ups (and VDD_IO_5, which also serves the IMU's SPI pins): kept at 3.3 V */
#define IO_LDO_CHANNEL 4

static sdmmc_card_t *card;

/* The SD driver's bounce buffer: one per card, for good (see sd_card_mount). IDF's default chunk, 16 sectors. */
#define BOUNCE_BYTES (16 * 512)
static void *bounce;

void sd_card_power(bool on)
{
    gpio_set_level(POWER_GPIO, on ? 0 : 1);
}

esp_err_t sd_card_mount(const char *base_path)
{
    if (card != NULL) {
        return ESP_OK;
    }
    gpio_config_t power = {.pin_bit_mask = 1ULL << POWER_GPIO, .mode = GPIO_MODE_OUTPUT};
    ESP_ERROR_CHECK(gpio_config(&power));
    sd_card_power(true);
    vTaskDelay(pdMS_TO_TICKS(20)); /* the card's supply settles */

    sdmmc_host_t host = SDMMC_HOST_DEFAULT(); /* deinit_p = per-slot deinit: a failure leaves the C6's slot alone */
    host.slot = SDMMC_HOST_SLOT_0;
    host.max_freq_khz = SDMMC_FREQ_HIGHSPEED; /* 40 MHz, 3.3 V signalling; never UHS (it drops VO4 to 1.8 V) */
    sd_pwr_ctrl_ldo_config_t ldo = {.ldo_chan_id = IO_LDO_CHANNEL};
    sd_pwr_ctrl_handle_t ldo_handle = NULL;
    esp_err_t err = sd_pwr_ctrl_new_on_chip_ldo(&ldo, &ldo_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "LDO VO4: %s", esp_err_to_name(err));
        return err;
    }
    host.pwr_ctrl_handle = ldo_handle;

    /* A transfer whose buffer is not DMA-ready goes through a DMA-capable bounce buffer: FatFs's own sector buffers
     * (every append the ring_saver makes, every FAT and directory update) are not cache-aligned. IDF's default
     * allocates that buffer per transfer -- 8 KB of internal DMA memory (16 x 512 B), freed after -- churning the very
     * memory esp_hosted asserts on when it cannot get a frame buffer (taster, 2026-10-06: allocate_dma_buf failures,
     * then the sdio_drv.c pkt_rxbuff assert, under video + ring_saver + a Wi-Fi download). One buffer for good
     * instead, taken here at boot while the heap is whole. Safe with one FAT volume: FatFs's volume lock serialises
     * every call into the driver. A second volume on this card (the two-partition layout) shares this buffer and
     * needs a card-level lock around the driver calls first. */
    if (bounce == NULL) {
        bounce = heap_caps_aligned_alloc(64, BOUNCE_BYTES, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
        if (bounce == NULL) {
            ESP_LOGE(TAG, "no %d B of internal DMA memory for the bounce buffer", BOUNCE_BYTES);
            return ESP_ERR_NO_MEM;
        }
    }
    host.dma_aligned_buffer = bounce;

    sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT(); /* slot 0 is IOMUX: CLK 43, CMD 44, D0..D3 39..42 */
    slot.width = 4;

    /* max_files: the files open at once on the card, each a FatFs FIL with its 4 KiB sector cache, in PSRAM
     * (CONFIG_FATFS_ALLOC_PREFER_EXTRAM): about 170 KB at 40. The ring_saver appends to up to 32 per batch, one at
     * a time; the video writer holds one; HTTP pulls read some more (doc/specs/avionics-port.md "The ring_saver",
     * Handles). CONFIG_FATFS_FS_LOCK in sdkconfig.defaults is at least as large. */
    esp_vfs_fat_sdmmc_mount_config_t mount = {
        .format_if_mount_failed = false, /* never format the operator's card */
        .max_files = 40,
        .allocation_unit_size = 64 * 1024,
    };
    err = esp_vfs_fat_sdmmc_mount(base_path, &host, &slot, &mount, &card);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "mount %s: %s", base_path, esp_err_to_name(err));
        card = NULL;
        return err;
    }
    return ESP_OK;
}

esp_err_t sd_card_volume_serial(uint32_t *serial)
{
    if (card == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    /* the card's FatFs drive is the physical drive the VFS registered it as: "<pdrv>:" */
    BYTE pdrv = ff_diskio_get_pdrv_card(card);
    if (pdrv == 0xff) {
        return ESP_ERR_NOT_FOUND;
    }
    char drive[3] = {(char)('0' + pdrv), ':', '\0'};
    DWORD vsn = 0;
    FRESULT result = f_getlabel(drive, NULL, &vsn); /* no label: only the boot sector is read */
    if (result != FR_OK) {
        ESP_LOGE(TAG, "volume serial: FatFs error %d", (int)result);
        return ESP_FAIL;
    }
    *serial = (uint32_t)vsn;
    return ESP_OK;
}

void sd_card_print_info(void)
{
    if (card != NULL) {
        sdmmc_card_print_info(stdout, card);
    }
}
