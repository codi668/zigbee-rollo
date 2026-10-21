# Teil 2: Applikation (app_state, app_logic), Zigbee (zb_device, zb_logic),
# sensor_task, Debug-Konsole. app_logic.c und zb_logic.c sind reine Logik
# (ohne ESP-IDF) und werden zusätzlich auf dem Host getestet (test_host/).
# esp-zigbee-lib / esp-zboss-lib kommen über main/idf_component.yml.

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

idf_component_register(SRCS "main.c" "powertest.c" "app_state.c" "app_logic.c" "zb_device.c" "zb_logic.c"
                            "sensor_task.c" "console_cmds.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES ${MAIN_REQUIRES} esp_app_format esp_system esp_timer esp_pm
                                     esp_partition esp_driver_gpio esp_driver_usb_serial_jtag console freertos log)

# Änderung des Schalters in config.h -> CMake neu konfigurieren
if(NOT CMAKE_BUILD_EARLY_EXPANSION)
    set_property(DIRECTORY "${COMPONENT_DIR}" APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
                 "${CMAKE_CURRENT_LIST_DIR}/../components/config/include/config.h")
endif()
