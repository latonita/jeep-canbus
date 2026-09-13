#pragma once
// One place for every colour and font the dashboard uses.
//
// Kept as macros in a single header on purpose: the car's own telltales and
// instruments are green-teal, not amber, so the palette is expected to be
// re-pitched wholesale. When that happens this file changes and nothing else
// does. The critical colours are deliberately NOT part of that swap -- red is
// the one hue that must not move when the rest of the instrument does.
#include "lvgl.h"


// ---- Instrument palette: green-teal, matching the car's own telltales -----
// Derived from the original amber ramp by holding OKLCH lightness and chroma
// and moving hue to 178 deg, so each step keeps its perceptual weight instead
// of teal reading as neon (a naive HSL rotation gives #1AFFE0, which does).
// Chroma is reduced where sRGB cannot hold amber's saturation at this hue.
#define UI_BG          lv_color_hex(0x000000)
#define UI_PANEL       lv_color_hex(0x071109)
#define UI_RULE        lv_color_hex(0x132216)
#define UI_DOT_OFF     lv_color_hex(0x173720)
#define UI_FAINT       lv_color_hex(0x21462B)
#define UI_LABEL       lv_color_hex(0x196433)
#define UI_DIM         lv_color_hex(0x008D43)
#define UI_PRIMARY     lv_color_hex(0x00DA6B)

// ---- Critical / alert (does not follow the palette swap) -----------------
#define UI_CRIT        lv_color_hex(0xFF4426)
#define UI_CRIT_DIM    lv_color_hex(0x8A2E1C)
#define UI_CRIT_TEXT   lv_color_hex(0xC08070)
#define UI_CRIT_PANEL  lv_color_hex(0x160D09)

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
#define UI_FONT_LABEL  (&lv_font_montserrat_14)

// ---- Layout --------------------------------------------------------------
#define UI_STATUSBAR_H 26
#define UI_DOTS_H      26
#define UI_PAD         12
