# Day-palette preview.
#
# The .dc.html artboards are authored in amber; build.py maps that authoring
# palette onto whatever the device actually ships. This does the same trick for
# the candidate DAYLIGHT palettes, so they can be judged on the real screens at
# true size rather than as a row of swatches -- a palette only fails once it is
# carrying actual type.
#
# Generates daylight-local.html. Not part of the build; run it when the
# palette is in question.
import re, pathlib

import math

# ---------------------------------------------------------------------------
# The ramps, generated rather than picked.
#
# Hue 151 deg is where the shipping teal actually sits (UiTheme.h used to claim
# 178; it was wrong). Each ramp mirrors the night ramp's LIGHTNESS SPACING so
# the steps keep their perceptual distance whichever way round the theme runs,
# and chroma is a fraction of what sRGB can hold AT EACH LIGHTNESS -- copying
# fixed chroma values onto a light background yields grey mid-tones, because
# low chroma at high lightness is simply grey.
# ---------------------------------------------------------------------------
HUE = 150.9
LN  = [0.000, 0.164, 0.234, 0.305, 0.358, 0.446, 0.563, 0.777]   # night spacing

def _srgb_to_lin(c):
    c /= 255.0
    return c/12.92 if c <= 0.04045 else ((c+0.055)/1.055)**2.4
def _lin_to_srgb(c):
    v = 12.92*c if c <= 0.0031308 else 1.055*(c**(1/2.4))-0.055
    return max(0, min(255, round(v*255)))
def _rgb(L, C, H):
    A = C*math.cos(math.radians(H)); B = C*math.sin(math.radians(H))
    l_ = L+0.3963377774*A+0.2158037573*B
    m_ = L-0.1055613458*A-0.0638541728*B
    s_ = L-0.0894841775*A-1.2914855480*B
    l, m, s = [x**3 for x in (l_, m_, s_)]
    return ( 4.0767416621*l-3.3077115913*m+0.2309699292*s,
            -1.2684380046*l+2.6097574011*m-0.3413193965*s,
            -0.0041960863*l-0.7034186147*m+1.7076147010*s)
def _hex(L, C, H):
    r, g, b = _rgb(L, C, H)
    return "#%02X%02X%02X" % (_lin_to_srgb(r), _lin_to_srgb(g), _lin_to_srgb(b))
def _fits(L, C, H):
    return all(-0.001 <= v <= 1.001 for v in _rgb(L, C, H))
def _maxC(L, H):
    lo, hi = 0.0, 0.45
    for _ in range(40):
        mid = (lo+hi)/2
        if _fits(L, mid, H): lo = mid
        else: hi = mid
    return lo

def ramp(Ltop, Lbot, cfrac, hue=HUE):
    """Eight steps, BG first. cfrac is the per-step fraction of gamut chroma."""
    span = LN[-1]-LN[0]
    return [_hex(Ltop-(L-LN[0])/span*(Ltop-Lbot), _maxC(Ltop-(L-LN[0])/span*(Ltop-Lbot), hue)*cfrac[i], hue)
            for i, L in enumerate(LN)]

def rel_lum(h):
    h = h.lstrip('#'); r, g, b = [_srgb_to_lin(int(h[i:i+2], 16)) for i in (0, 2, 4)]
    return 0.2126*r+0.7152*g+0.0722*b
def contrast(f, b):
    a, c = rel_lum(f), rel_lum(b); hi, lo = max(a, c), min(a, c)
    return (hi+0.05)/(lo+0.05)

# The amber authoring tokens the .dc.html artboards are drawn in, in ramp order.
AMBER = ["#0B0907", "#140C08", "#241C13", "#3A2C18", "#4A3A22",
         "#6B4E20", "#9A6A22", "#FFA31A"]
TOKENS = ["UI_BG", "UI_PANEL", "UI_RULE", "UI_DOT_OFF", "UI_FAINT",
          "UI_LABEL", "UI_DIM", "UI_PRIMARY"]
# Sheet-only extras that have no firmware token of their own.
EXTRA = {"#FFC46B": 6, "#2A1F12": 3, "#1C150E": 1}   # -> index into the ramp

def palette(steps, crit):
    m = dict(zip(AMBER, steps))
    for k, idx in EXTRA.items():
        m[k] = steps[idx]
    m.update(crit)
    return m

CRIT_DARKBG  = {"#FF4426": "#FF4426", "#8A2E1C": "#8A2E1C",
                "#C08070": "#C08070", "#160D09": "#160D09"}
CRIT_TEAL    = {"#FF4426": "#7A0C00", "#8A2E1C": "#9E4634",
                "#C08070": "#5E2418", "#160D09": "#74927B"}
CRIT_LIGHT   = {"#FF4426": "#A81400", "#8A2E1C": "#C4705E",
                "#C08070": "#8A3524", "#160D09": "#D8C9C4"}

R_NIGHT  = ["#000000", "#071109", "#132216", "#173720",
            "#21462B", "#196433", "#008D43", "#00DA6B"]
R_TEAL   = ramp(0.780, 0.130, [1.00, 0.30, 0.38, 0.52, 0.60, 0.78, 0.85, 0.55])
R_MINT   = ramp(0.960, 0.292, [0.22, 0.20, 0.26, 0.42, 0.50, 0.72, 0.95, 0.70])
# Black on teal: the field is the brand teal and everything under it walks the
# SAME hue straight down to black. No greys, no second colour, maximum simple.
R_BKTEAL = ramp(0.777, 0.000, [1.00, 0.95, 0.92, 0.88, 0.84, 0.78, 0.70, 0.00])
R_BKTEAL[0] = "#00DA6B"          # exactly the brand teal, not a rounded cousin
R_BKTEAL[7] = "#000000"
# Black on white: no hue at all. The brightest possible field and the darkest
# possible ink -- nothing here can be washed out by sun except by the sun
# winning outright.
R_BKWHITE = ["#FFFFFF", "#E2E2E2", "#C6C6C6", "#ABABAB",
             "#909090", "#6E6E6E", "#3F3F3F", "#000000"]

PALETTES = [
    ("NIGHT",      "teal on black",        R_NIGHT,   CRIT_DARKBG),
    ("DAY TEAL",   "near-black on teal",   R_TEAL,    CRIT_TEAL),
    ("DAY MINT",   "dark teal on mint",    R_MINT,    CRIT_LIGHT),
    ("BLACK/TEAL", "black on brand teal",  R_BKTEAL,  CRIT_TEAL),
    ("BLACK/WHITE","black on white",       R_BKWHITE, CRIT_LIGHT),
]

CANDS = [("%s -- %s" % (n, d), palette(r, c)) for n, d, r, c in PALETTES]

# ---- the C tables, so firmware and design cannot drift ---------------------
NL = chr(10)
_out = []
_out.append("// GENERATED by design/daypalette.py -- do not hand-edit.")
_out.append("// Regenerate with:  cd design && python daypalette.py")
_out.append("")
for name, desc, r, c in PALETTES:
    ident = "k" + name.title().replace("/", "").replace(" ", "")
    _out.append("// %s -- %s" % (name, desc))
    _out.append("static const uint32_t %s[UI_C_COUNT] = {" % ident)
    for tok, hx in zip(TOKENS, r):
        _out.append("    0x%s,  // %s" % (hx.lstrip("#"), tok[3:]))
    for tok, key in (("CRIT", "#FF4426"), ("CRIT_DIM", "#8A2E1C"),
                     ("CRIT_TEXT", "#C08070"), ("CRIT_PANEL", "#160D09")):
        _out.append("    0x%s,  // %s" % (c[key].lstrip("#"), tok))
    _out.append("};")
    _out.append("")
_out.append("static const uint32_t* const kTables[UI_THEME_COUNT] = {")
_out.append("    " + ", ".join("k"+n.title().replace("/","").replace(" ","")
                               for n,_,_,_ in PALETTES) + ",")
_out.append("};")
_out.append("")
_out.append("const char* const kUiThemeName[UI_THEME_COUNT] = {")
_out.append("    " + ", ".join('"%s"' % n for n,_,_,_ in PALETTES) + ",")
_out.append("};")
open("palettes.inc", "w").write(NL.join(_out) + NL)

print("palettes.inc  <-  %d palettes" % len(PALETTES))
for name, desc, r, c in PALETTES:
    print("  %-12s bg %s  ink %s   ink %5.1f:1  dim %4.1f:1  label %4.1f:1  faint %4.1f:1"
          % (name, r[0], r[7], contrast(r[7], r[0]), contrast(r[6], r[0]),
             contrast(r[5], r[0]), contrast(r[4], r[0])))

import re, pathlib

def recolour(src, subs):
    def rep(m):
        return subs.get(m.group(0).upper(), m.group(0))
    return re.sub(r"#[0-9A-Fa-f]{6}", rep, src)

def body(path):
    txt = pathlib.Path(path).read_text(encoding="utf-8")
    inner = re.search(r"<x-dc>(.*?)</x-dc>", txt, re.S).group(1)
    return re.sub(r"<helmet>.*?</helmet>", "", inner, flags=re.S).strip()

SCREENS = ["Main.dc.html", "Tyres.dc.html", "Trip.dc.html", "Settings.dc.html"]

cols = []
for title, subs in CANDS:
    figs = "".join('<div class="screen">%s</div>' % recolour(body(s), subs)
                   for s in SCREENS)
    cols.append('<section><h2>%s</h2><div class="row">%s</div></section>' % (title, figs))

html = """<!doctype html><html><head><meta charset="utf-8">
<title>daylight palette candidates</title>
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=Barlow+Condensed:wght@400;500;600&family=Barlow+Semi+Condensed:wght@500;600&display=swap">
<style>
 body{margin:0;padding:28px;background:#101214;color:#C8D2CC;
      font-family:'Barlow Semi Condensed',system-ui,sans-serif;}
 h1{font-size:20px;letter-spacing:.14em;margin:0 0 4px;color:#00DA6B;}
 p.sub{font-size:14px;color:#7C8C82;margin:0 0 26px;max-width:70ch;line-height:1.55;}
 h2{font-size:13px;letter-spacing:.18em;color:#9FB0A5;margin:30px 0 10px;font-weight:600;}
 .row{display:flex;gap:16px;flex-wrap:wrap;}
 .screen{width:240px;height:320px;overflow:hidden;
         box-shadow:0 0 0 1px #2A3330,0 8px 22px rgba(0,0,0,.55);}
 @media (prefers-color-scheme: light){body{background:#E6EAE7;color:#2A322D;}
   p.sub{color:#55605A;} h2{color:#3C463F;}}
</style></head><body>
<h1>DAYLIGHT PALETTE CANDIDATES</h1>
<p class="sub">True 240&times;320, real screens, not swatches. Every ramp is built in OKLCH at hue 151&deg; &mdash;
the same hue the shipping palette actually sits at &mdash; mirroring the night ramp's lightness spacing so the
steps keep their perceptual distance. Chroma is a fraction of what the gamut allows at each lightness, which is
what stops the mid-tones going grey. Judge these in sun, not at a desk.</p>
%s
</body></html>""" % "\n".join(cols)

pathlib.Path("daylight-local.html").write_text(html, encoding="utf-8")
print("daylight-local.html  <-  %d screens x %d palettes" % (len(SCREENS), len(CANDS)))
