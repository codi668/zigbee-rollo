# SENSOR: nicht bestückt, deaktiviert – zum Aktivieren einkommentieren:
# ROLLO_WINDOW_SENSOR in config.h. Ohne den Schalter zieht main diese Komponente
# nicht an, sie wird dann (MINIMAL_BUILD) nicht gebaut. Code bleibt erhalten.
# lis2dh12_conv.c ist reine Logik (Host-getestet)
idf_component_register(SRCS "lis2dh12.c" "lis2dh12_conv.c" INCLUDE_DIRS "include"
                       REQUIRES esp_common
                       PRIV_REQUIRES board config esp_driver_i2c log)
