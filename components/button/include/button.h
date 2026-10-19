#pragma once
#include <stdbool.h>

void button_init(void);
/* Test 2: zaehlt entprellte Tastendruecke der BOOT-Taste fuer duration_s Sekunden.
 * PASS, wenn mindestens ein Druck erkannt wurde. */
bool button_test(int duration_s);
