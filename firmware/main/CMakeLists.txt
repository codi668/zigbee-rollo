# Platzhalter-main (Teil 1): initialisiert alle Module und gibt einen Selbsttest aus.
# Die eigentliche Anwendung (app_state, zb_device, Zigbee) kommt in Teil 2.

set(MAIN_REQUIRES board config drv8833 encoder motor_ctrl battery storage ui)

# SENSOR: nicht bestückt, deaktiviert – zum Aktivieren in config.h die Zeile
# "#define ROLLO_WINDOW_SENSOR 1" einkommentieren. Diese Datei liest denselben
# Schalter, damit window_sensor + lis2dh12 (I2C) nur dann gebaut werden
# (MINIMAL_BUILD baut nur, was main anzieht).
file(READ "${CMAKE_CURRENT_LIST_DIR}/../components/config/include/config.h" ROLLO_CONFIG_H)
# Gleiche Logik wie "#ifdef ROLLO_WINDOW_SENSOR" in C: jede aktive #define-Zeile
# zählt (mit oder ohne Wert), auskommentierte ("// #define") nicht.
if(ROLLO_CONFIG_H MATCHES "\n[ \t]*#[ \t]*define[ \t]+ROLLO_WINDOW_SENSOR([ \t]|\r|\n)")
    message(STATUS "Rollo: Fenstersensor AKTIV (ROLLO_WINDOW_SENSOR)")
    list(APPEND MAIN_REQUIRES window_sensor lis2dh12)
else()
    message(STATUS "Rollo: Fenstersensor deaktiviert (ROLLO_WINDOW_SENSOR in config.h auskommentiert)")
endif()

idf_component_register(SRCS "main.c" INCLUDE_DIRS "."
                       PRIV_REQUIRES ${MAIN_REQUIRES} esp_app_format esp_system freertos log)

# Änderung des Schalters in config.h -> CMake neu konfigurieren
if(NOT CMAKE_BUILD_EARLY_EXPANSION)
    set_property(DIRECTORY "${COMPONENT_DIR}" APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
                 "${CMAKE_CURRENT_LIST_DIR}/../components/config/include/config.h")
endif()
