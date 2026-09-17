---
paths:
  - "design/**/*"
---

# The `design/` artboards

`.dc.html` artboards at true size — 240×320 portrait plus a `*.landscape.dc.html`
band at 320×240 — with `canvas.json` and `build.py`. Reasoning for the landscape
layout lives in the `landscape-*` notes in `canvas.json`.

**The artboards are ~30 % narrower than the firmware.** They are CSS Barlow
Condensed and Barlow figures are ~0.727 em, so `font-size:66px` renders 48px
digits while `font_digits_64` renders 64. **Never take a coordinate off an
artboard without re-measuring the string** against `adv_w` in `src/fonts/*.c`
(1/16 px units, plus `letter_space` between glyphs but not after the last).
Three places on the landscape band only fit once measured and would otherwise
have shipped overlapping.

**Palettes are GENERATED, not hand-picked:**

```bash
cd design && python daypalette.py && cp palettes.inc ../src/UiPalettes.inc
```

The same script renders `daylight-local.html`, so firmware and design cannot
disagree. Teal sits at **OKLCH hue 151°** (an old comment said 178 — wrong).

**`Tyres*.dc.html` is the ASPIRATIONAL board** (LEFT FRONT / RIGHT FRONT …);
`TyresUnmapped*.dc.html` is what ships (SENSOR 1-4). **They are meant to disagree
— do not "fix" the mapped one against the firmware.** Both landscape boards also
draw a per-cell `LOW` chip the firmware does not have; the firmware colours the
value instead. The two boards now differ only in the NAMES — both are 2×2 in
both bands.

**FAULTS and ALERT are artboards only, in both bands** — they need a vertical
axis the instrument did not have when they were drawn. The FAULTS drill-down is
built; ALERT is not.
