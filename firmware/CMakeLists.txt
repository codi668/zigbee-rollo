# Zigbee-Rollosteuerung – Firmware (ESP32-H2FH4S, ESP-IDF 5.5)
# Version kommt automatisch aus version.txt (PROJECT_VER, App-Descriptor).
cmake_minimum_required(VERSION 3.16)
# Light-Sleep-Variante (Strommessung), eigener Build-Ordner:
#   idf.py -B build-ls -DSDKCONFIG=build-ls/sdkconfig -DROLLO_LS=1 build
# Setzt ROLLO_LIGHT_SLEEP für alle Komponenten und lädt sdkconfig.lightsleep zusätzlich.
# Leistungstest-Variante (Akku-Strommessung, Zustände nacheinander, siehe main/powertest.c):
#   idf.py -B build-pt -DSDKCONFIG=build-pt/sdkconfig -DROLLO_PT=1 build
if(ROLLO_PT)
    set(SDKCONFIG_DEFAULTS "sdkconfig.defaults;sdkconfig.lightsleep")
    add_compile_options(-DROLLO_POWERTEST=1)
endif()
# Nur-Deep-Sleep-Variante (Boden messen): idf.py -B build-ptd -DSDKCONFIG=build-ptd/sdkconfig -DROLLO_PTD=1 build
if(ROLLO_PTD)
    set(SDKCONFIG_DEFAULTS "sdkconfig.defaults;sdkconfig.lightsleep")
    add_compile_options(-DROLLO_POWERTEST=1 -DPT_DEEP_ONLY=1)
endif()
if(ROLLO_LS)
    set(SDKCONFIG_DEFAULTS "sdkconfig.defaults;sdkconfig.lightsleep")
    add_compile_options(-DROLLO_LIGHT_SLEEP=1)
endif()
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# Nur main und seine Abhängigkeiten bauen (kürzere Buildzeit, kleineres Image)
idf_build_set_property(MINIMAL_BUILD ON)
project(zigbee-rollo)
