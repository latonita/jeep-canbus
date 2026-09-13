#pragma once
// Minimal on-screen UI: the firmware's own log, on the panel.
//
// Deliberately not a dashboard. It is the smallest useful thing to put on
// screen -- and genuinely useful in the car, where the USB console is not
// reachable while the unit is behind the dash. The real interface gets
// designed separately; this must not grow into it.
#include "esp_err.h"

// Takes over the panel from Display's fill test: registers it with LVGL and
// starts capturing esp_log output into an on-screen ring buffer.
esp_err_t uiLogInit(void);

// Put a line on the screen. Safe to call from any task; never blocks the
// caller for more than 50 ms, and drops the line rather than waiting.
//
// NOTE: this module does NOT intercept esp_log. Screen output is explicit, so
// nothing in the logging path can re-enter it -- see the comment in UiLog.cpp
// for the deadlock that taught us that.
void uiLogLine(const char* text);
void uiLogPrintf(const char* fmt, ...) __attribute__((format(printf, 1, 2)));
