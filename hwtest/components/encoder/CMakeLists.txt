idf_component_register(SRCS "encoder.c" INCLUDE_DIRS "include"
                       REQUIRES board motor esp_driver_pcnt esp_driver_gpio
                       PRIV_REQUIRES freertos esp_timer)
