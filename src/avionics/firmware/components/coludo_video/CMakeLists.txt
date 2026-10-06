# coludo_video: OV5647 -> ISP -> H.264 HW encoder -> Annex-B file (see include/coludo_video.h)
idf_component_register(
    SRCS "coludo_video.c" "coludo_ov5647.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_i2c
    PRIV_REQUIRES espressif__esp_video espressif__esp_cam_sensor espressif__esp_sccb_intf espressif__esp_h264
                  esp_timer vfs)
