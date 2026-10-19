idf_component_register(SRCS "main.c" INCLUDE_DIRS "."
                       REQUIRES board led button adc charger accel drv motor encoder console
                       PRIV_REQUIRES freertos esp_app_format)
