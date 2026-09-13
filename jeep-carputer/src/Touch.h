#pragma once
// Capacitive touch for the ESP32-S3-Touch-LCD-2.8 (V1 board: CST328 at 0x1A).
//
// Brings up its own I2C master bus, creates the CST328 driver on top of it and
// hands the result to esp_lvgl_port as an LVGL input device, so that taps,
// drags and swipe gestures reach the widgets Display/Ui already put on the
// panel.
//
// Deliberately no touch API of its own: nothing in this firmware should read
// coordinates directly. LVGL owns the pointer, and anything that wants a swipe
// asks LVGL for one (LV_EVENT_GESTURE / lv_indev_get_gesture_dir).
#include <stdbool.h>

#include "esp_err.h"

// Must be called AFTER uiInit(), because it attaches to the lv_display_t
// that the LVGL port created there and takes the LVGL lock to register the
// input device. Safe to skip entirely: a failure here leaves the screen
// readable, just not touchable.
esp_err_t touchInit(void);

// True once the controller answered and LVGL accepted the input device.
bool touchReady(void);
