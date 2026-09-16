#pragma once
// One place for every colour and font the dashboard uses.
//
// Kept in a single header on purpose: the car's own telltales and instruments
// are green-teal, so the palette is re-pitched wholesale rather than per
// widget. Adding a theme means adding a table in UiTheme.cpp and nothing else.
//
// The colours are macros over a runtime table, so every call site keeps
// reading UI_PRIMARY and none of them know a theme exists. What they cannot do
// is cache: a widget's colour is baked in when it is built, so changing the
// table only affects what is built AFTERWARDS. See uiApplyTheme() in Ui.cpp,
// which rebuilds the screen for exactly that reason.
#include "lvgl.h"


// ---- Palette -------------------------------------------------------------
// RUNTIME, not compile-time. The driver can swap the whole instrument between
// a night palette and two daylight ones from the SETTINGS page, so every token
// is an index into the active table rather than a literal.
//
// Every ramp is built in OKLCH at hue 151 deg -- which is where the shipping
// teal actually sits; the "178 deg" in the old comment here was wrong -- and
// mirrors the night ramp's LIGHTNESS SPACING so the steps keep their
// perceptual distance from each other whichever way round the theme runs.
//
// Chroma is a fraction of what sRGB can hold AT EACH LIGHTNESS, not a fixed
// number. That distinction is the whole trick: copying the night ramp's chroma
// values onto a light background produces grey mid-tones (#859087, #718073),
// because low chroma at high lightness is simply grey.
enum UiColour : uint8_t {
  UI_C_BG = 0,
  UI_C_PANEL,
  UI_C_RULE,
  UI_C_DOT_OFF,
  UI_C_FAINT,
  UI_C_LABEL,
  UI_C_DIM,
  UI_C_PRIMARY,
  UI_C_CRIT,
  UI_C_CRIT_DIM,
  UI_C_CRIT_TEXT,
  UI_C_CRIT_PANEL,
  UI_C_COUNT,
};

// Order here must match kTables[] in the generated src/UiPalettes.inc.
enum UiTheme : uint8_t {
  UI_THEME_NIGHT = 0,   // teal on black -- the default, and right after dark
  UI_THEME_DAY_TEAL,    // near-black on teal, teal-family supporting tones
  UI_THEME_DAY_MINT,    // dark teal on pale mint -- brightest field of the five
  UI_THEME_BLACK_TEAL,  // pure black on the brand teal, one hue all the way down
  UI_THEME_BLACK_WHITE, // pure black on white, no hue at all
  UI_THEME_COUNT,
};

// Short names for the SETTINGS row. Index with a UiTheme.
extern const char* const kUiThemeName[UI_THEME_COUNT];

// The active table. Plain uint32_t so the tables are real compile-time
// constants; lv_color_hex() is inline, so the indirection costs nothing.
extern const uint32_t* uiPal;

void uiThemeSet(uint8_t theme);   // swaps the table only; see uiApplyTheme()
uint8_t uiThemeGet(void);

#define UI_BG          lv_color_hex(uiPal[UI_C_BG])
#define UI_PANEL       lv_color_hex(uiPal[UI_C_PANEL])
#define UI_RULE        lv_color_hex(uiPal[UI_C_RULE])
#define UI_DOT_OFF     lv_color_hex(uiPal[UI_C_DOT_OFF])
#define UI_FAINT       lv_color_hex(uiPal[UI_C_FAINT])
#define UI_LABEL       lv_color_hex(uiPal[UI_C_LABEL])
#define UI_DIM         lv_color_hex(uiPal[UI_C_DIM])
#define UI_PRIMARY     lv_color_hex(uiPal[UI_C_PRIMARY])

// ---- Critical / alert ----------------------------------------------------
// Red does not follow the palette swap in hue, but it DOES have to follow it
// in lightness: #FF4426 on pale mint is a weak 3.4:1, so each theme carries
// its own critical that keeps the warning louder than everything around it.
#define UI_CRIT        lv_color_hex(uiPal[UI_C_CRIT])
#define UI_CRIT_DIM    lv_color_hex(uiPal[UI_C_CRIT_DIM])
#define UI_CRIT_TEXT   lv_color_hex(uiPal[UI_C_CRIT_TEXT])
#define UI_CRIT_PANEL  lv_color_hex(uiPal[UI_C_CRIT_PANEL])

// ---- Type ----------------------------------------------------------------
// Hero and mid are subset faces (digits and a handful of symbols only), which
// is what makes a 64 px face affordable. Labels use stock Montserrat.
//
// The guard keeps the firmware buildable before the generated fonts land: it
// falls back to stock Montserrat, which is ugly at hero size but compiles and
// runs, so a font problem never blocks bring-up of anything else.
#if defined(__has_include)
#if __has_include("fonts/fonts.h")
#include "fonts/fonts.h"
#define UI_HAVE_DIGIT_FONTS 1
#endif
#endif

#ifdef UI_HAVE_DIGIT_FONTS
#define UI_FONT_HERO   (&font_digits_64)
#define UI_FONT_MID    (&font_digits_30)
#else
#define UI_FONT_HERO   (&lv_font_montserrat_14)
#define UI_FONT_MID    (&lv_font_montserrat_14)
#endif
#ifdef UI_HAVE_DIGIT_FONTS
#define UI_FONT_LABEL  (&font_label_14)
// The HERO UNIT face, one step up from the caption face. The artboards draw the
// unit larger than the caption (13 px against 9); drawing both in
// font_label_14 flattened that and made the unit read as a second caption.
// Used only for the page unit keys -- every stack-row unit stays at 14, and
// that difference IS the hierarchy.
#define UI_FONT_UNIT   (&font_label_20)
#else
#define UI_FONT_LABEL  (&lv_font_montserrat_14)
#define UI_FONT_UNIT   (&lv_font_montserrat_14)
#endif

// ---- Layout --------------------------------------------------------------
#define UI_STATUSBAR_H 26
#define UI_DOTS_H      26
#define UI_PAD         12
