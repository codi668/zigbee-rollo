# btn_logic.c und led_pattern.c sind reine Logik (Host-getestet)
idf_component_register(SRCS "ui.c" "btn_logic.c" "led_pattern.c" INCLUDE_DIRS "include"
                       REQUIRES esp_common
                       PRIV_REQUIRES board config esp_driver_gpio esp_timer freertos log)
