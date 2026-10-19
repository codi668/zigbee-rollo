# Zigbee-Rollosteuerung – Firmware (ESP32-H2FH4S, ESP-IDF 5.5)
# Version kommt automatisch aus version.txt (PROJECT_VER, App-Descriptor).
cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# Nur main und seine Abhängigkeiten bauen (kürzere Buildzeit, kleineres Image)
idf_build_set_property(MINIMAL_BUILD ON)
project(zigbee-rollo)
