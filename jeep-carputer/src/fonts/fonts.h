// LVGL fonts for the 240x320 instrument dashboard.
//
// Typefaces: Barlow Condensed Medium for the numerals, Barlow Semi Condensed
// Medium for the labels -- the same pair the design artboards are drawn in
// (SIL Open Font License 1.1, see OFL.txt).
// All generated with lv_font_conv 1.5.3, LVGL 9 "lvgl" format, 4 bpp,
// uncompressed, kerning dropped.
//
// The DIGIT faces carry a 15-glyph subset (0-9 . - : space and U+00B0), which
// is what makes a 64 px face affordable at all. The LABEL face carries full
// printable ASCII, because it has to render whatever a caption says.
//
// Source TTFs (fetched in WSL, not committed):
//   curl -fsSLO https://raw.githubusercontent.com/google/fonts/main/ofl/barlowcondensed/BarlowCondensed-Medium.ttf
// Regeneration (the full option string is also in the header of each .c file):
//   npx --yes lv_font_conv@1.5.3 --font BarlowCondensed-Medium.ttf
//       -r 0x20 -r 0x2D-0x2E -r 0x30-0x3A -r 0xB0 --size 88 --bpp 4
//       --no-compress --no-prefilter --no-kerning --format lvgl
//       --lv-include lvgl.h --lv-font-name font_digits_64 -o font_digits_64.c
//   (same with --size 42 / --lv-font-name font_digits_30 / -o font_digits_30.c)
//
// The label face, which replaced stock lv_font_montserrat_14. Montserrat is a
// normal-width grotesque and the layout is drawn for a condensed one, so every
// caption ran wide -- visibly wider than the artboards, and wide enough to push
// unit captions off the right edge of the panel.
//   curl -fsSLO https://raw.githubusercontent.com/google/fonts/main/ofl/barlowsemicondensed/BarlowSemiCondensed-Medium.ttf
//   npx --yes lv_font_conv@1.5.3 --font BarlowSemiCondensed-Medium.ttf
//       -r 0x20-0x7E -r 0xB0 -r 0x20AC --size 16 --bpp 4 --no-compress
//       --no-prefilter --no-kerning --format lvgl --lv-include lvgl.h
//       --lv-font-name font_label_14 -o font_label_14.c
//   (same with --size 20 / --lv-font-name font_label_20 / -o font_label_20.c)
//
// U+20AC EURO SIGN was added to both label faces for the fuel price. It is the
// only glyph outside ASCII+degree, and it is in BOTH so a price can be printed
// at either size. The digit faces still have no letters and no currency: a cost
// is numerals in font_digits_* with the sign beside it in a label face.
//
// font_label_20 WAS the hero-unit-key face -- the KM / L/100 / V / BAR that sat
// top-right of a page. THERE ARE NO UNIT KEYS ANY MORE: both bands now put the
// hero's unit on the hero's BASELINE in font_label_14, mkUnitKey() is deleted,
// and the argument this paragraph used to make (the artboards give the unit
// 13px against a 9px caption, so the unit should be LARGER than the caption)
// applies to a layout that no longer exists.
//
// KEEP THE FACE. It is still drawn as UI_FONT_UNIT in two places that are not
// units at all: the - / + glyphs on the SETTINGS fuel-price buttons, and the
// fault code in the fault dialog. Deleting it would take both with it.
// Regenerating label_14 with the euro did not move a single existing advance
// width -- same --size 16, same metrics (line_height 16, base_line 3) -- so
// nothing in either band shifted.
//
//   font_label_14  em size 16 px, line_height 16, base_line 3 -- EXACTLY the
//   metrics of lv_font_montserrat_14 it replaces, chosen so the swap changes
//   width and nothing else. Full printable ASCII plus U+00B0, 96 glyphs.
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
extern const lv_font_t font_label_14;

//   font_label_20  em size 20 px, line_height 21, base_line 4. Same glyph set
//   as font_label_14. Note the DIFFERENT base_line: to sit a key on the same
//   baseline as a 14px caption its y must be 4 px HIGHER, not equal.
extern const lv_font_t font_label_20;

#ifdef __cplusplus
}
#endif

#endif  // SRC_FONTS_FONTS_H
