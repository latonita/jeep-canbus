#include "UiTheme.h"

// The palettes live in a GENERATED file, because they are not hand-picked
// values: design/daypalette.py builds every ramp in OKLCH at hue 151 deg,
// mirroring the night ramp's lightness spacing and taking chroma as a fraction
// of what sRGB holds at each lightness. The same script renders them onto the
// real screens in design/daylight-local.html, so the firmware and the design
// sheet cannot disagree about what a theme looks like.
//
// Do not hand-edit a single step. The ramps only hold together because every
// step was placed against the others; change one and the rest stop being a
// ramp. Edit daypalette.py and regenerate:
//
//     cd design && python daypalette.py && cp palettes.inc ../src/UiPalettes.inc
//
// Defines kNight/kDayTeal/kDayMint/kBlackTeal/kBlackWhite, kTables[] and
// kUiThemeName[].
#include "UiPalettes.inc"

// Night by default. A dashboard that comes up blindingly bright in the dark is
// worse than one that comes up dim in daylight: the first ruins night vision,
// the second is merely hard to read for the second it takes to change it.
static uint8_t s_theme = UI_THEME_NIGHT;
const uint32_t* uiPal = kNight;

void uiThemeSet(uint8_t theme) {
  if (theme >= UI_THEME_COUNT) {
    theme = UI_THEME_NIGHT;
  }
  s_theme = theme;
  uiPal = kTables[theme];
}

uint8_t uiThemeGet(void) { return s_theme; }
