idf_component_register(SRCS "encoder.c" INCLUDE_DIRS "include"
                       REQUIRES esp_common
                       PRIV_REQUIRES board config esp_driver_gpio esp_driver_pcnt freertos log)
