#!/usr/bin/env python
"""Re-pitch the instrument palette to a new hue and write it into src/UiTheme.h.

    python design/tint.py              # current default: warm green
    python design/tint.py --hue 178    # teal
    python design/tint.py --hue 60 --boost 1.0    # back to something amber-ish

Hue rotation is done in OKLCH, holding each step's perceptual LIGHTNESS, so the
ramp keeps its relative weight instead of the brighter hues washing out the
darker ones. A naive HSL rotation does not: it turns amber's #FFA31A into the
neon #1AFFE0, because green and cyan are perceptually much lighter than amber at
the same nominal HSL lightness.

Chroma is scaled by --boost and then clamped to the sRGB gamut, because amber
carries less chroma than a saturated green can, and a straight copy looks washed
out on black.

The critical reds are deliberately never touched: red is the one hue that must
not move when the rest of the instrument does.
"""
import argparse
import io
import math
import os
import re

# The amber values the theme was originally authored in. These are the keys the
# rewrite matches on, so UiTheme.h can be re-tinted repeatedly without drift:
# every run starts from this table, not from whatever is currently in the file.
BASE = [
    ("UI_BG", "0x0B0907"),
    ("UI_PANEL", "0x140C08"),
    ("UI_RULE", "0x241C13"),
    ("UI_DOT_OFF", "0x3A2C18"),
    ("UI_FAINT", "0x4A3A22"),
    ("UI_LABEL", "0x6B4E20"),
    ("UI_DIM", "0x9A6A22"),
    ("UI_PRIMARY", "0xFFA31A"),
]


def s2l(c):
    return c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4


def l2s(c):
    return 12.92 * c if c <= 0.0031308 else 1.055 * (c ** (1 / 2.4)) - 0.055


def to_oklch(hexstr):
    h = hexstr.lstrip("#").replace("0x", "")
    r, g, b = (s2l(int(h[i:i + 2], 16) / 255) for i in (0, 2, 4))
    l = 0.4122214708 * r + 0.5363325363 * g + 0.0514459929 * b
    m = 0.2119034982 * r + 0.6806995451 * g + 0.1073969566 * b
    s = 0.0883024619 * r + 0.2817188376 * g + 0.6299787005 * b
    l_, m_, s_ = (v ** (1 / 3) if v > 0 else -((-v) ** (1 / 3)) for v in (l, m, s))
    L = 0.2104542553 * l_ + 0.7936177850 * m_ - 0.0040720468 * s_
    A = 1.9779984951 * l_ - 2.4285922050 * m_ + 0.4505937099 * s_
    B = 0.0259040371 * l_ + 0.7827717662 * m_ - 0.8086757660 * s_
    return L, math.hypot(A, B), math.degrees(math.atan2(B, A)) % 360


def to_rgb(L, C, H):
    A = C * math.cos(math.radians(H))
    B = C * math.sin(math.radians(H))
    l_ = L + 0.3963377774 * A + 0.2158037573 * B
    m_ = L - 0.1055613458 * A - 0.0638541728 * B
    s_ = L - 0.0894841775 * A - 1.2914855480 * B
    l, m, s = l_ ** 3, m_ ** 3, s_ ** 3
    return (4.0767416621 * l - 3.3077115913 * m + 0.2309699292 * s,
            -1.2684380046 * l + 2.6097574011 * m - 0.3413193965 * s,
            -0.0041960863 * l - 0.7034186147 * m + 1.7076147010 * s)


def in_gamut(rgb, eps=1e-4):
    return all(-eps <= c <= 1 + eps for c in rgb)


def to_hex(L, C, H):
    if not in_gamut(to_rgb(L, C, H)):
        lo, hi = 0.0, C
        for _ in range(40):
            mid = (lo + hi) / 2
            if in_gamut(to_rgb(L, mid, H)):
                lo = mid
            else:
                hi = mid
        C = lo
    rgb = to_rgb(L, C, H)
    return "0x%02X%02X%02X" % tuple(
        max(0, min(255, round(l2s(max(0.0, min(1.0, c))) * 255))) for c in rgb)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--hue", type=float, default=136.0,
                    help="OKLCH hue in degrees. 136=warm green, 178=teal, 60=amber")
    ap.add_argument("--primary", default=None,
                    help="pin UI_PRIMARY to exactly this hex (e.g. 0x1AFFE0) and take "
                         "the ramp hue from it, instead of deriving primary from --hue")
    ap.add_argument("--boost", type=float, default=1.45,
                    help="chroma multiplier before gamut clamping")
    ap.add_argument("--black", action="store_true", default=True,
                    help="force the background to pure black rather than a tinted near-black")
    ap.add_argument("--theme", default=os.path.join(os.path.dirname(__file__), "..", "src", "UiTheme.h"))
    args = ap.parse_args()

    # A pinned primary is the authority: the ramp takes its hue, and the primary
    # itself is written through untouched so the requested hex is what ships.
    hue = args.hue
    if args.primary is not None:
        pinned = "0x" + args.primary.lstrip("#").replace("0x", "").upper()
        pl, pc, ph = to_oklch(pinned)
        hue = ph
        print("pinned UI_PRIMARY %s  ->  OKLCH L=%.3f C=%.3f H=%.1f" % (pinned, pl, pc, ph))

    mapping = {}
    print("%-11s %-9s -> %-9s" % ("token", "base", "tinted"))
    for name, base in BASE:
        L, C, _ = to_oklch(base)
        value = to_hex(L, C * args.boost, hue)
        if name == "UI_BG" and args.black:
            value = "0x000000"
        if name == "UI_PRIMARY" and args.primary is not None:
            value = pinned
        mapping[name] = value
        print("%-11s %-9s -> %-9s" % (name, base, value))

    path = os.path.normpath(args.theme)
    src = io.open(path, encoding="utf-8").read()
    for name, value in mapping.items():
        # Rewrite by TOKEN NAME, not by old value, so repeated runs never drift.
        src = re.sub(r"(#define\s+%s\s+lv_color_hex\()0x[0-9A-Fa-f]{6}(\))" % name,
                     r"\g<1>%s\g<2>" % value, src)
    io.open(path, "w", encoding="utf-8").write(src)
    print("\nwrote %s (hue %.1f, boost %.2f)" % (path, hue, args.boost))


main()
