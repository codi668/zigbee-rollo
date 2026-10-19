# SENSOR: nicht bestückt, deaktiviert – zum Aktivieren einkommentieren:
# ROLLO_WINDOW_SENSOR in config.h. Ohne den Schalter zieht main diese Komponente
# nicht an, sie wird dann (MINIMAL_BUILD) nicht gebaut. Code bleibt erhalten.
# tilt_logic.c ist reine Logik (Host-getestet)
idf_component_register(SRCS "window_sensor.c" "tilt_logic.c" INCLUDE_DIRS "include"
                       REQUIRES esp_common lis2dh12
                       PRIV_REQUIRES config esp_timer freertos log)
