idf_component_register(SRCS "accel.c" INCLUDE_DIRS "include"
                       REQUIRES board esp_driver_i2c PRIV_REQUIRES freertos)
