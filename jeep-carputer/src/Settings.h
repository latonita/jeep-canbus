#pragma once
// Driver-changeable settings, persisted in NVS.
//
// Deliberately a flat struct of small values rather than a key per setting.
// One blob is one write, and the write budget matters: this shares a flash
// partition with the trip meter, which already commits once a minute for the
// life of the car.
//
// Everything here must have a sane value when NVS is empty or unreadable --
// settingsLoad() leaves the defaults in place rather than failing, because a
// corrupt settings record must never be the reason a dashboard will not boot.
//
// No dependency on the display or the UI, on purpose. This module stores and
// recalls; deciding what a setting MEANS belongs to whatever owns the thing
// being set. main.cpp hands the stored theme to the UI at bring-up.
#include <stdint.h>

#include "esp_err.h"

struct Settings {
  // Which palette the instrument wears. A UiTheme value (see UiTheme.h), kept
  // as a plain uint8_t so this header stays free of LVGL.
  //
  // Defaults to 0 = NIGHT. A dashboard that comes up blindingly bright in the
  // dark is worse than one that comes up dim in daylight: the first ruins
  // night vision, the second is merely hard to read for a second.
  uint8_t theme;

  Settings() : theme(0u) {}
};

// The live settings. Read freely; change only through settingsSet*.
const Settings& settingsGet(void);

// Reads NVS into the live settings. Safe to call before nvs_flash_init has
// succeeded: it simply leaves the defaults alone.
esp_err_t settingsLoad(void);

// Stores the theme and commits it. Does NOT apply it -- the caller does that,
// because applying means rebuilding the screen and only the UI task may.
esp_err_t settingsSetTheme(uint8_t theme);
