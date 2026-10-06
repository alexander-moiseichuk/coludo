# esp_hosted 2.7.0 (the host side of the C6 Wi-Fi link) restarts the whole P4 from its SDIO error paths
# (sdio_drv.c: an unresponsive slave, a TX write failing twice, a register/interrupt read failing). In flight a
# C6 glitch must not reboot the flight computer, so this replaces that restart hook -- without patching the
# downloaded component (2.9.4+ has ESP_HOSTED_TRANSPORT_RESTART_ON_FAILURE; 2.7.0 is pinned for the C6).
idf_component_register(SRCS "hosted_no_reboot.c"
                       INCLUDE_DIRS "include"
                       REQUIRES espressif__esp_hosted freertos log)
# The OS-function table's layout depends on esp_hosted's transport macros (H_TRANSPORT_IN_USE, H_USE_MEMPOOL),
# defined in its PRIVATE port config header: without it every transport block compiles in and the field offsets
# are wrong. Compile against the same header the table's own definition sees.
idf_component_get_property(hosted_directory espressif__esp_hosted COMPONENT_DIR)
target_include_directories(${COMPONENT_LIB} PRIVATE "${hosted_directory}/host/port/esp/freertos/include")
