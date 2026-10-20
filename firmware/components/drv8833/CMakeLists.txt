idf_component_register(SRCS "drv8833.c" INCLUDE_DIRS "include"
                       REQUIRES esp_common
                       PRIV_REQUIRES board config esp_driver_gpio esp_driver_ledc esp_timer freertos log)
