idf_component_register(SRCS "adc_meas.c" INCLUDE_DIRS "include"
                       REQUIRES board esp_adc PRIV_REQUIRES esp_driver_gpio freertos)
