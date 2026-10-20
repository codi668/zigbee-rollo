# Host-Tests (PC, gcc) für die reine Logik der Firmware – ohne ESP-IDF-Abhängigkeit.
# Aufruf:  make -C firmware/test_host test
# Unity kommt aus dem ESP-IDF (nur die Quellen, kein IDF-Build nötig).

IDF_PATH  ?= $(HOME)/.platformio/packages/framework-espidf
UNITY_DIR := $(IDF_PATH)/components/unity/unity/src
COMP      := ../components
BUILD     := build

CC      ?= gcc
CFLAGS  := -std=c11 -O1 -g -Wall -Wextra -Werror -Wno-unused-parameter \
           -I$(UNITY_DIR) \
           -I$(COMP)/config/include \
           -I$(COMP)/battery/include \
           -I$(COMP)/motor_ctrl/include \
           -I$(COMP)/lis2dh12/include \
           -I$(COMP)/window_sensor/include \
           -I$(COMP)/ui/include \
           -I../main
LDLIBS  := -lm

# Reine Logik aus den Komponenten und aus main/ (gleiche Dateien wie im Firmware-Build).
# lis2dh12_conv/tilt_logic: Sensor ist in der Firmware deaktiviert (ROLLO_WINDOW_SENSOR),
# die Logik wird trotzdem getestet.
LOGIC_SRC := $(COMP)/battery/battery_calc.c \
             $(COMP)/motor_ctrl/axis.c \
             $(COMP)/motor_ctrl/block_detect.c \
             $(COMP)/motor_ctrl/motion.c \
             $(COMP)/lis2dh12/lis2dh12_conv.c \
             $(COMP)/window_sensor/tilt_logic.c \
             $(COMP)/ui/btn_logic.c \
             $(COMP)/ui/led_pattern.c \
             ../main/app_logic.c \
             ../main/zb_logic.c

TEST_SRC  := test_main.c test_battery_calc.c test_axis.c test_block_detect.c test_motion.c \
             test_lis2dh12_conv.c test_tilt_logic.c test_btn_logic.c test_led_pattern.c \
             test_app_logic.c test_zb_logic.c

SRC  := $(TEST_SRC) $(LOGIC_SRC) $(UNITY_DIR)/unity.c
BIN  := $(BUILD)/test_host

# Zweites Programm: ROLLO_TILT_ONLY = 1 (nur Motor 2) und manuelle Endpunkte
TILT_SRC := test_tilt_only.c ../main/app_logic.c $(UNITY_DIR)/unity.c
TILT_BIN := $(BUILD)/test_tilt_only

.PHONY: all test clean

all: $(BIN) $(TILT_BIN)

$(BIN): $(SRC) $(wildcard *.h) $(wildcard $(COMP)/*/include/*.h) $(wildcard ../main/*_logic.h)
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) -DROLLO_TILT_ONLY=0 -o $@ $(SRC) $(LDLIBS)

$(TILT_BIN): $(TILT_SRC) $(wildcard *.h) $(wildcard ../main/*_logic.h) $(wildcard $(COMP)/*/include/*.h)
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) -DROLLO_TILT_ONLY=1 -o $@ $(TILT_SRC) $(LDLIBS)

test: $(BIN) $(TILT_BIN)
	./$(BIN)
	./$(TILT_BIN)

clean:
	rm -rf $(BUILD)
