idf_component_register(SRCS "led.c" INCLUDE_DIRS "include"
                       REQUIRES board esp_driver_gpio PRIV_REQUIRES freertos)
