# battery_calc.c ist reine Logik (Host-getestet)
idf_component_register(SRCS "battery.c" "battery_calc.c" INCLUDE_DIRS "include"
                       REQUIRES esp_common
                       PRIV_REQUIRES board config esp_adc esp_driver_gpio freertos log)
