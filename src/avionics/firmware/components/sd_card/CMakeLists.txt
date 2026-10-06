# The microSD on SDMMC slot 0 -- mounted in C because the host/slot defaults are C macros (SDMMC_HOST_DEFAULT,
# SDMMC_SLOT_CONFIG_DEFAULT) that bindgen cannot carry into Rust.
idf_component_register(SRCS "sd_card.c"
                       INCLUDE_DIRS "include"
                       REQUIRES fatfs sdmmc esp_driver_sdmmc esp_driver_gpio log)
