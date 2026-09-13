// Large-digit LVGL fonts for the 240x320 instrument dashboard.
//
// Typeface: Barlow Condensed Medium (SIL Open Font License 1.1, see OFL.txt).
// Generated with lv_font_conv 1.5.3, LVGL 9 "lvgl" format, 4 bpp, uncompressed,
// kerning dropped. Glyph subset (15 glyphs): 0-9 . - : space and U+00B0 degree.
//
// Source TTF (fetched in WSL, not committed):
//   curl -fsSLO https://raw.githubusercontent.com/google/fonts/main/ofl/barlowcondensed/BarlowCondensed-Medium.ttf
// Regeneration (the full option string is also in the header of each .c file):
//   npx --yes lv_font_conv@1.5.3 --font BarlowCondensed-Medium.ttf
//       -r 0x20 -r 0x2D-0x2E -r 0x30-0x3A -r 0xB0 --size 88 --bpp 4
//       --no-compress --no-prefilter --no-kerning --format lvgl
//       --lv-include lvgl.h --lv-font-name font_digits_64 -o font_digits_64.c
//   (same with --size 42 / --lv-font-name font_digits_30 / -o font_digits_30.c)
//
//   font_digits_64  em size 88 px -> digit height 64 px, line_height 73, base_line 1
//   font_digits_30  em size 42 px -> digit height 30 px, line_height 35, base_line 0
//
// The em size is larger than the digit height because Barlow Condensed figures
// are ~0.73 em tall; the names refer to the rendered digit height, which is what
// the dashboard layout is specified in.
//
// NOTE: these faces are proportional, not tabular -- '1' is much narrower than
// '0'. Centre-aligned readouts will shift horizontally as values change; pin the
// label width / use LV_TEXT_ALIGN_RIGHT if that matters.

#ifndef SRC_FONTS_FONTS_H
#define SRC_FONTS_FONTS_H

#include "lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

extern const lv_font_t font_digits_64;
extern const lv_font_t font_digits_30;

#ifdef __cplusplus
}
#endif

#endif  // SRC_FONTS_FONTS_H
