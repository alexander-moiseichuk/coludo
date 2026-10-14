# The microSD on SDMMC slot 0 -- mounted in C because the host/slot defaults are C macros (SDMMC_HOST_DEFAULT,
# SDMMC_SLOT_CONFIG_DEFAULT) that bindgen cannot carry into Rust. partition_table.c is plain C, also built on the host
# by src/control/test/test_sd_partition_table.py.
idf_component_register(SRCS "sd_card.c" "partition_table.c"
                       INCLUDE_DIRS "include"
                       REQUIRES fatfs vfs sdmmc esp_driver_sdmmc esp_driver_gpio log)
