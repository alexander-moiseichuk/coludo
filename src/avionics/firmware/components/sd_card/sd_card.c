/* Coludo project, copyright under MIT license, Alexander Moiseichuk */

#include "sd_card.h"

#include <stdio.h>

#include "driver/gpio.h"
#include "driver/sdmmc_host.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
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

    sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT(); /* slot 0 is IOMUX: CLK 43, CMD 44, D0..D3 39..42 */
    slot.width = 4;

    esp_vfs_fat_sdmmc_mount_config_t mount = {
        .format_if_mount_failed = false, /* never format the operator's card */
        .max_files = 8,
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

void sd_card_print_info(void)
{
    if (card != NULL) {
        sdmmc_card_print_info(stdout, card);
    }
}
