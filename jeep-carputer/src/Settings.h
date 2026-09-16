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

  // Which way round the instrument is drawn. A UiOrient value (see Ui.cpp),
  // kept as a plain uint8_t so this header stays free of LVGL: 0 = PORTRAIT,
  // 1 = LANDSCAPE, 2 = LANDSCAPE turned end for end.
  //
  // Two landscapes because the bracket can take the panel on either edge, and
  // which one is upright is a property of the mount. Defaults to PORTRAIT,
  // which is the panel's own geometry -- a default that needs no guess about
  // how the bracket ended up. Unlike the palette this is set once, when the
  // mount is chosen, and then never again.
  //
  // An out-of-range value must not brick the boot: Ui.cpp clamps anything it
  // does not recognise back to PORTRAIT.
  uint8_t orientation;

  // What a litre of diesel costs, in whole cents, so the stored value is exact
  // -- a price is a decimal quantity and floats do not hold 2.01 exactly.
  //
  // Defaults to 201, i.e. 2.01 EUR. A default that is roughly right is worth
  // far more than a zero here: a zero makes every cost readout show dashes
  // until someone finds this row, whereas a plausible default is useful
  // immediately and obviously wrong enough to correct when it drifts.
  uint16_t fuelPriceCents;

  Settings() : theme(0u), orientation(0u), fuelPriceCents(201u) {}
};

// The live settings. Read freely; change only through settingsSet*.
const Settings& settingsGet(void);

// Reads NVS into the live settings. Safe to call before nvs_flash_init has
// succeeded: it simply leaves the defaults alone.
esp_err_t settingsLoad(void);

// Stores the theme and commits it. Does NOT apply it -- the caller does that,
// because applying means rebuilding the screen and only the UI task may.
esp_err_t settingsSetTheme(uint8_t theme);

// Stores the orientation and commits it. Does NOT apply it, for the same
// reason as the theme and rather more so: flipping the panel tears down the
// whole widget tree, which only the UI task may do.
esp_err_t settingsSetOrientation(uint8_t orientation);

// Stores the fuel price and commits it. Clamped to a sane band by the caller;
// this only records what it is given.
esp_err_t settingsSetFuelPrice(uint16_t cents);
