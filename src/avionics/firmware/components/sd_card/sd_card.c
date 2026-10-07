/* Coludo project, copyright under MIT license, Alexander Moiseichuk */
/*
 * The microSD card: initialised once, one FatFs physical drive, and up to two FAT volumes on it
 * (doc/specs/avionics-port.md "SD and flash file layout"): partition 1, the recordings, at the base path
 * (/sdcard), and partition 2, the videos, at the videos path (/sdcard/videos). IDF's esp_vfs_fat_sdmmc_mount mounts
 * one volume per card, so this does what it does inside (components/fatfs/vfs/vfs_fat_sdmmc.c: host, slot and card
 * init, then the VFS and f_mount) and registers a disk I/O layer of its own in place of diskio_sdmmc.c's.
 *
 * Why a disk I/O layer of its own: FatFs serialises the calls into one volume with that volume's lock, and nothing
 * more. Two volumes on one card call the driver at once -- the video writer on one, the ring_saver on the other --
 * and every unaligned transfer of both goes through the one persistent bounce buffer (sd_card_mount), while the
 * card itself answers one command sequence at a time. So every call that reaches the card holds one card lock, a
 * FreeRTOS mutex (priority inheritance: the saver at 4 holding it lifts to the video writer's 6 while that waits).
 * Each lock is taken around one driver call and nothing is taken under it, so it cannot deadlock with the volume
 * locks: a volume lock is always taken first, the card lock only inside it. The SDMMC host's own lock still
 * serialises single transactions with the C6 on slot 1.
 *
 * Which volumes: the board reads the card's table itself before anything mounts (partition_table.c, host-tested
 * against card images). Two volumes mount only when partitions 1 and 2 are both FAT, lie apart within the card and
 * each holds a file system that fits it: FatFs checks none of that, and two volumes over shared sectors would each
 * allocate clusters the other owns. A table that names two FAT partitions but fails those checks mounts partition 1
 * alone, with no video; a sector 0 that will not read mounts nothing.
 */

#include "sd_card.h"

#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/sdmmc_host.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_vfs.h"
#include "esp_vfs_fat.h"
#include "ff.h"
#include "diskio_impl.h" /* after ff.h: it names FatFs's disk I/O types */
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "partition_table.h"
#include "sd_pwr_ctrl_by_on_chip_ldo.h"
#include "sdmmc_cmd.h"

static const char *TAG = "sd_card";

/* Q1's gate: low = card powered (schematic: SI2301CDS, R22 10k to GND) */
#define POWER_GPIO 45
/* ESP_LDO_VO4 feeds the bus pull-ups (and VDD_IO_5, which also serves the IMU's SPI pins): kept at 3.3 V */
#define IO_LDO_CHANNEL 4

/* The SD driver's bounce buffer: one per card, for good (see sd_card_mount). IDF's default chunk, 16 sectors. */
#define BOUNCE_BYTES (16 * 512)

/*
 * max_files per volume: the files open at once on it, each a FatFs FIL with its 4 KiB sector cache, in PSRAM
 * (CONFIG_FATFS_ALLOC_PREFER_EXTRAM), about 4.3 KB each. The recordings volume: the ring_saver appends to up to 32 a
 * batch, one at a time, and HTTP pulls read some more. The videos volume: the video writer holds one, HTTP pulls read
 * the others. One volume holds both on a one-partition card. FatFs's lock table (CONFIG_FATFS_FS_LOCK) is one for all
 * volumes, so it must hold the sum, or a full table answers FR_TOO_MANY_OPEN_FILES before the VFS runs out of handles.
 */
#define RECORDINGS_FILES 32
#define VIDEOS_FILES 8
#if FF_FS_LOCK < RECORDINGS_FILES + VIDEOS_FILES
#error "CONFIG_FATFS_FS_LOCK must be at least RECORDINGS_FILES + VIDEOS_FILES: one lock table serves both volumes"
#endif
#if FF_VOLUMES < 2 || !FF_MULTI_PARTITION
#error "two volumes on one card need CONFIG_FATFS_VOLUME_COUNT >= 2 and FatFs's FF_MULTI_PARTITION"
#endif

/* The logical drives (FatFs volumes): partition 1, or the one volume of a one-partition card; partition 2. */
#define RECORDINGS_DRIVE "0:"
#define RECORDINGS_VOLUME 0
#define VIDEOS_DRIVE "1:"
#define VIDEOS_VOLUME 1

static sdmmc_card_t *card;
static void *bounce;
static sd_card_layout_t layout = SD_CARD_LAYOUT_NONE;

/* The card lock (see the file notes), static: no heap at all, and it lives as long as the mount. */
static StaticSemaphore_t card_lock_storage;
static SemaphoreHandle_t card_lock;

void sd_card_power(bool on)
{
    gpio_set_level(POWER_GPIO, on ? 0 : 1);
}

/* Disk I/O for the card (diskio_sdmmc.c's calls, each under the card lock). Every volume on the card names the same
 * physical drive, so `drive` is the card's whichever volume asks. */

static DSTATUS card_initialize(BYTE drive)
{
    (void)drive;
    xSemaphoreTake(card_lock, portMAX_DELAY);
    esp_err_t err = sdmmc_get_status(card);
    xSemaphoreGive(card_lock);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "card status: %s", esp_err_to_name(err));
        return STA_NOINIT;
    }
    return 0;
}

static DSTATUS card_status(BYTE drive)
{
    (void)drive;
    return 0; /* IDF's default (disk_status_check_enable off): no card command per FatFs call */
}

static DRESULT card_read(BYTE drive, BYTE *buffer, DWORD sector, UINT count)
{
    (void)drive;
    xSemaphoreTake(card_lock, portMAX_DELAY);
    esp_err_t err = sdmmc_read_sectors(card, buffer, sector, count);
    xSemaphoreGive(card_lock);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "read of %u sectors at %lu: %s", count, (unsigned long)sector, esp_err_to_name(err));
        return RES_ERROR;
    }
    return RES_OK;
}

static DRESULT card_write(BYTE drive, const BYTE *buffer, DWORD sector, UINT count)
{
    (void)drive;
    xSemaphoreTake(card_lock, portMAX_DELAY);
    esp_err_t err = sdmmc_write_sectors(card, buffer, sector, count);
    xSemaphoreGive(card_lock);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "write of %u sectors at %lu: %s", count, (unsigned long)sector, esp_err_to_name(err));
        return RES_ERROR;
    }
    return RES_OK;
}

static DRESULT card_ioctl(BYTE drive, BYTE command, void *buffer)
{
    (void)drive;
    switch (command) {
    case CTRL_SYNC:
        return RES_OK; /* a write above returns only once the card took it */
    case GET_SECTOR_COUNT:
        *(DWORD *)buffer = card->csd.capacity;
        return RES_OK;
    case GET_SECTOR_SIZE:
        *(WORD *)buffer = card->csd.sector_size;
        return RES_OK;
    case GET_BLOCK_SIZE:
        return RES_ERROR; /* unknown, as diskio_sdmmc.c answers: only f_mkfs asks, which never runs here */
#if FF_USE_TRIM
    case CTRL_TRIM: {
        /* the freed sectors [first, last] of a removed or truncated file; SD cards take no TRIM, eMMC does */
        DWORD first = ((DWORD *)buffer)[0];
        DWORD last = ((DWORD *)buffer)[1];
        DRESULT result = RES_OK;
        xSemaphoreTake(card_lock, portMAX_DELAY);
        if (sdmmc_can_trim(card) != ESP_OK) {
            result = RES_PARERR;
        } else {
            sdmmc_erase_arg_t argument = sdmmc_can_discard(card) == ESP_OK ? SDMMC_DISCARD_ARG : SDMMC_ERASE_ARG;
            esp_err_t err = sdmmc_erase_sectors(card, first, last - first + 1, argument);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "erase of sectors %lu..%lu: %s", (unsigned long)first, (unsigned long)last,
                         esp_err_to_name(err));
                result = RES_ERROR;
            }
        }
        xSemaphoreGive(card_lock);
        return result;
    }
#endif
    default:
        return RES_ERROR;
    }
}

static const ff_diskio_impl_t card_disk = {
    .init = &card_initialize,
    .status = &card_status,
    .read = &card_read,
    .write = &card_write,
    .ioctl = &card_ioctl,
};

/* How often a sector the partition table needs is read before it counts as unreadable: a single CRC error or
 * timeout on the first reads after the card's init must not pass for a card without a table. */
#define TABLE_READ_ATTEMPTS 3

/* partition_table_read's reader: one sector of the card (`context`, its FatFs physical drive), under the card lock,
 * tried TABLE_READ_ATTEMPTS times (card_read logs each failure). */
static bool read_table_sector(void *context, uint32_t sector, uint8_t *buffer)
{
    BYTE drive = *(const BYTE *)context;
    for (int attempt = 0; attempt < TABLE_READ_ATTEMPTS; attempt++) {
        if (card_read(drive, buffer, sector, 1) == RES_OK) {
            return true;
        }
    }
    return false;
}

/*
 * Unregisters the volume mount_volume registered at `path`, the VFS entry and vfs_fat's context. vfs_fat looks a
 * volume up by its base path as it keeps it, and it keeps it cut to ESP_VFS_PATH_MAX - 2 characters (IDF v5.5.5
 * vfs_fat.c esp_vfs_fat_register_cfg: strlcpy into base_path[ESP_VFS_PATH_MAX] with sizeof - 1), while the VFS holds
 * the whole path: to vfs_fat, "/sdcard/videos" (14) is "/sdcard/video". esp_vfs_fat_unregister_path takes a path that
 * long by neither name: by the whole one it finds no context, by the cut one its own esp_vfs_unregister of the cut
 * path finds no entry (it warns, then frees the context anyway, leaving the entry over freed memory). So for such a
 * path the entry goes first, by the whole path -- from then on nothing reaches the volume -- and the context after,
 * by the cut one (its warning about the entry is expected). An IDF that keeps the whole path finds no context by the
 * cut one and leaks it (its file table, in PSRAM) on a path only a failed boot takes. (sd_card_space reads the free
 * space by the FatFs drive for the same reason: esp_vfs_fat_info looks the volume up by its base path too.)
 */
static esp_err_t unregister_volume(const char *path)
{
    if (strlen(path) <= ESP_VFS_PATH_MAX - 2) {
        return esp_vfs_fat_unregister_path(path); /* vfs_fat keeps it whole */
    }
    esp_err_t err = esp_vfs_unregister(path);
    char kept[ESP_VFS_PATH_MAX];
    strlcpy(kept, path, sizeof(kept) - 1);
    esp_vfs_fat_unregister_path(kept);
    return err;
}

/* Registers FatFs's volume `drive` with the VFS at `path` and mounts it (IDF's mount_to_vfs_fat, past the disk I/O
 * registration). On failure nothing of it stays registered, unless the unregister itself fails, which is logged. */
static esp_err_t mount_volume(const char *path, const char *drive, size_t max_files)
{
    esp_vfs_fat_conf_t vfs = {
        .base_path = path,
        .fat_drive = drive,
        .max_files = max_files,
    };
    FATFS *volume = NULL;
    esp_err_t err = esp_vfs_fat_register_cfg(&vfs, &volume);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "VFS at %s: %s", path, esp_err_to_name(err));
        return err;
    }
    FRESULT result = f_mount(volume, drive, 1); /* now, not at the first file call: a bad volume shows at boot */
    if (result != FR_OK) {
        ESP_LOGE(TAG, "mount of volume %s at %s: FatFs error %d", drive, path, (int)result);
        f_mount(NULL, drive, 0);
        err = unregister_volume(path);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "%s stays registered over an unmounted volume: %s", path, esp_err_to_name(err));
        }
        return ESP_FAIL;
    }
    return ESP_OK;
}

/* Brings the host, slot 0 and the card up (esp_vfs_fat_sdmmc_mount's first half) into `card`. On failure only slot
 * 0 is deinitialised, and `card` is freed. */
static esp_err_t start_card(const sdmmc_host_t *host, const sdmmc_slot_config_t *slot)
{
    card = malloc(sizeof(sdmmc_card_t)); /* internal RAM, as IDF allocates it */
    if (card == NULL) {
        return ESP_ERR_NO_MEM;
    }
    esp_err_t err = host->init();
    if (err == ESP_OK) {
        err = sdmmc_host_init_slot(host->slot, slot);
        if (err == ESP_OK) {
            err = sdmmc_card_init(host, card);
        }
        if (err != ESP_OK) {
            host->deinit_p(host->slot); /* SDMMC_HOST_FLAG_DEINIT_ARG: this slot only, the C6's stays */
        }
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "card init: %s", esp_err_to_name(err));
        free(card);
        card = NULL;
    }
    return err;
}

/* Takes back what sd_card_mount set up past start_card, when no volume mounted: the card's FatFs drive, slot 0 (the
 * C6's slot 1 stays) and `card`. */
static void stop_card(const sdmmc_host_t *host, BYTE drive)
{
    ff_diskio_unregister(drive);
    host->deinit_p(host->slot);
    free(card);
    card = NULL;
}

esp_err_t sd_card_mount(const char *base_path, const char *videos_path)
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
     * instead, taken here at boot while the heap is whole. Both volumes share it: the card lock (the file notes)
     * keeps their transfers apart. */
    if (bounce == NULL) {
        bounce = heap_caps_aligned_alloc(64, BOUNCE_BYTES, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
        if (bounce == NULL) {
            ESP_LOGE(TAG, "no %d B of internal DMA memory for the bounce buffer", BOUNCE_BYTES);
            return ESP_ERR_NO_MEM;
        }
    }
    host.dma_aligned_buffer = bounce;
    if (card_lock == NULL) {
        card_lock = xSemaphoreCreateMutexStatic(&card_lock_storage);
    }

    sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT(); /* slot 0 is IOMUX: CLK 43, CMD 44, D0..D3 39..42 */
    slot.width = 4;

    BYTE drive = FF_DRV_NOT_USED;
    if (ff_diskio_get_drive(&drive) != ESP_OK || drive == FF_DRV_NOT_USED) {
        ESP_LOGE(TAG, "no FatFs drive left for the card");
        return ESP_ERR_NO_MEM;
    }
    err = start_card(&host, &slot);
    if (err != ESP_OK) {
        return err;
    }
    ff_diskio_register(drive, &card_disk);

    /* the table, judged before anything mounts (partition_table.h); a card with sectors other than 512 B is mounted
     * as one volume, unjudged */
    partition_t partitions[2];
    const char *reason = "sectors of other than 512 B";
    partition_table_t table = PARTITION_TABLE_ONE;
    if (card->csd.sector_size == PARTITION_TABLE_SECTOR_BYTES) {
        table = partition_table_read(read_table_sector, &drive, (uint32_t)card->csd.capacity, partitions, &reason);
    }
    switch (table) {
    case PARTITION_TABLE_UNREADABLE:
        /* no guess at the layout: on a two-partition card FatFs's own search would mount partition 1 as the card's
         * one volume, and video would fill the recordings partition */
        ESP_LOGE(TAG, "%s, %d times: no card this boot", reason, TABLE_READ_ATTEMPTS);
        stop_card(&host, drive);
        return ESP_FAIL;
    case PARTITION_TABLE_BROKEN:
        ESP_LOGE(TAG, "partition 1 at %lu, %lu sectors; partition 2 at %lu, %lu sectors; the card %d sectors: %s, so "
                 "partition 1 alone and no video this boot", (unsigned long)partitions[0].start,
                 (unsigned long)partitions[0].sectors, (unsigned long)partitions[1].start,
                 (unsigned long)partitions[1].sectors, card->csd.capacity, reason);
        break;
    default:
        ESP_LOGI(TAG, "the card's table: %s", reason);
        break;
    }

    /* which partition each volume is (FatFs's VolToPart, FF_MULTI_PARTITION): partition 1 forced whenever the table
     * names two FAT partitions, and partition 2 with it when they are sound; else 0, FatFs's own search (a
     * superfloppy, or the first FAT partition of the table) */
    bool two = table == PARTITION_TABLE_TWO;
    VolToPart[RECORDINGS_VOLUME] = (PARTITION){.pd = drive, .pt = table == PARTITION_TABLE_ONE ? 0 : 1};
    if (two) {
        VolToPart[VIDEOS_VOLUME] = (PARTITION){.pd = drive, .pt = 2};
    }
    err = mount_volume(base_path, RECORDINGS_DRIVE, two ? RECORDINGS_FILES : RECORDINGS_FILES + VIDEOS_FILES);
    if (err != ESP_OK) {
        stop_card(&host, drive);
        return err;
    }
    if (table == PARTITION_TABLE_BROKEN) {
        layout = SD_CARD_LAYOUT_VIDEOS_FAILED;
    } else if (!two) {
        layout = SD_CARD_LAYOUT_ONE_PARTITION;
    } else if (mount_volume(videos_path, VIDEOS_DRIVE, VIDEOS_FILES) == ESP_OK) {
        layout = SD_CARD_LAYOUT_TWO_PARTITIONS;
    } else {
        ESP_LOGE(TAG, "partition 2 is FAT by its type but did not mount at %s: no video this boot", videos_path);
        layout = SD_CARD_LAYOUT_VIDEOS_FAILED;
    }
    return ESP_OK;
}

sd_card_layout_t sd_card_layout(void)
{
    return layout;
}

/* The volume serial of FatFs's volume `drive`, from its boot sector. */
static esp_err_t serial_of(const char *drive, uint32_t *serial)
{
    DWORD vsn = 0;
    FRESULT result = f_getlabel(drive, NULL, &vsn); /* no label: only the boot sector is read */
    if (result != FR_OK) {
        ESP_LOGE(TAG, "volume serial of %s: FatFs error %d", drive, (int)result);
        return ESP_FAIL;
    }
    *serial = (uint32_t)vsn;
    return ESP_OK;
}

esp_err_t sd_card_volume_serial(uint32_t *serial)
{
    if (layout == SD_CARD_LAYOUT_NONE) {
        return ESP_ERR_INVALID_STATE;
    }
    return serial_of(RECORDINGS_DRIVE, serial);
}

esp_err_t sd_card_videos_serial(uint32_t *serial)
{
    if (layout != SD_CARD_LAYOUT_TWO_PARTITIONS) {
        return ESP_ERR_INVALID_STATE;
    }
    return serial_of(VIDEOS_DRIVE, serial);
}

esp_err_t sd_card_space(sd_card_volume_t volume, uint64_t *total_bytes, uint64_t *free_bytes)
{
    bool mounted = volume == SD_CARD_VOLUME_VIDEOS ? layout == SD_CARD_LAYOUT_TWO_PARTITIONS
                                                   : layout != SD_CARD_LAYOUT_NONE;
    if (!mounted) {
        return ESP_ERR_INVALID_STATE;
    }
    /* esp_vfs_fat_info's sums, by the FatFs drive rather than the base path vfs_fat cannot find (unregister_volume) */
    const char *drive = volume == SD_CARD_VOLUME_VIDEOS ? VIDEOS_DRIVE : RECORDINGS_DRIVE;
    FATFS *fs = NULL;
    DWORD free_clusters = 0;
    FRESULT result = f_getfree(drive, &free_clusters, &fs);
    if (result != FR_OK) {
        ESP_LOGE(TAG, "free space of %s: FatFs error %d", drive, (int)result);
        return ESP_FAIL;
    }
#if FF_MAX_SS != FF_MIN_SS
    uint64_t cluster_bytes = (uint64_t)fs->csize * fs->ssize;
#else
    uint64_t cluster_bytes = (uint64_t)fs->csize * FF_MIN_SS;
#endif
    *total_bytes = (uint64_t)(fs->n_fatent - 2) * cluster_bytes;
    *free_bytes = (uint64_t)free_clusters * cluster_bytes;
    return ESP_OK;
}

void sd_card_print_info(void)
{
    if (card != NULL) {
        sdmmc_card_print_info(stdout, card);
    }
}
