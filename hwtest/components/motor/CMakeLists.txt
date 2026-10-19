idf_component_register(SRCS "motor.c" INCLUDE_DIRS "include"
                       REQUIRES board drv esp_driver_ledc PRIV_REQUIRES freertos esp_timer)
