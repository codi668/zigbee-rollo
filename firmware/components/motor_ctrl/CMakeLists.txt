# axis.c, block_detect.c, motion.c sind reine Logik (ohne IDF) und werden
# zusätzlich auf dem Host getestet (firmware/test_host).
idf_component_register(SRCS "motor_ctrl.c" "motion.c" "block_detect.c" "axis.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_common
                       PRIV_REQUIRES config drv8833 encoder esp_timer esp_pm freertos log)
