# NVS-Wrapper: Kalibrierung, Positionen, moving-Flag, Fenster-Referenzlage
idf_component_register(SRCS "storage.c" INCLUDE_DIRS "include"
                       REQUIRES esp_common
                       PRIV_REQUIRES config nvs_flash freertos log)
