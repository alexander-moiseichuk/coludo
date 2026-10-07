/* Coludo project, copyright under MIT license, Alexander Moiseichuk */
/*
 * The board's judgment of a card's partition table (../partition_table.c), run on the host over a card image: what
 * src/control/test/test_sd_partition_table.py builds and runs against images tools/sd_prepare.sh and mkfs.fat write.
 *
 *   partition_table_host <image> [unreadable sector ...]
 *
 * The image's size is the card's; every sector named after it fails to read, as a card's read can. Prints one line,
 * `<verdict>;<partition 1 start>;<its sectors>;<partition 2 start>;<its sectors>;<reason>`, the verdict `unreadable`,
 * `one`, `two` or `broken` and the partitions 0 when the verdict names none. Exit 0 once judged, 2 on a usage error or
 * an image that would not open.
 */

#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>

#include "../partition_table.h"

/* The image, and the sectors that fail to read. */
typedef struct {
    FILE *image;
    char **unreadable;
    int unreadable_count;
} card_t;

/* partition_table_read's reader over the image: a named sector, or one past the image's end, would not read. */
static bool read_sector(void *context, uint32_t sector, uint8_t *buffer)
{
    card_t *card = context;
    for (int index = 0; index < card->unreadable_count; index++) {
        if (strtoul(card->unreadable[index], NULL, 10) == sector) {
            return false;
        }
    }
    return fseeko(card->image, (off_t)sector * PARTITION_TABLE_SECTOR_BYTES, SEEK_SET) == 0 &&
           fread(buffer, PARTITION_TABLE_SECTOR_BYTES, 1, card->image) == 1;
}

int main(int count, char **arguments)
{
    static const char *const verdicts[] = {"unreadable", "one", "two", "broken"};
    if (count < 2) {
        fprintf(stderr, "usage: partition_table_host <image> [unreadable sector ...]\n");
        return 2;
    }
    struct stat status;
    card_t card = {.image = fopen(arguments[1], "rb"), .unreadable = arguments + 2, .unreadable_count = count - 2};
    if (card.image == NULL || stat(arguments[1], &status) != 0) {
        perror(arguments[1]);
        return 2;
    }
    partition_t partitions[2] = {{0, 0}, {0, 0}};
    const char *reason = "";
    uint32_t card_sectors = (uint32_t)(status.st_size / PARTITION_TABLE_SECTOR_BYTES);
    partition_table_t verdict = partition_table_read(read_sector, &card, card_sectors, partitions, &reason);
    printf("%s;%lu;%lu;%lu;%lu;%s\n", verdicts[verdict], (unsigned long)partitions[0].start,
           (unsigned long)partitions[0].sectors, (unsigned long)partitions[1].start,
           (unsigned long)partitions[1].sectors, reason);
    fclose(card.image);
    return 0;
}
