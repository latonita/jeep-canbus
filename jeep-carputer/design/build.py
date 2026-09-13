#!/usr/bin/env python
"""Build the local contact sheet from the .dc.html sources.

    python build.py            -> screens-local.html        (Factory amber)
    python build.py teal       -> screens-local-teal.html   (green-teal, matches the WH telltales)

The palette swap is a hue rotation applied to the amber family only; the
critical reds are deliberately left alone, because red is the one colour that
must not shift when the rest of the instrument does.
"""
import colorsys, io, json, re, sys

SCREENS = ["Splash.dc.html", "Main.dc.html", "Tyres.dc.html", "Health.dc.html", "Trip.dc.html",
           "Tilt.dc.html", "Faults.dc.html", "TyresUnmapped.dc.html", "Alert.dc.html"]

# Amber-family tokens. Everything else in the sources (the criticals #FF4426,
# #8A2E1C, #C08070, #8A5E4A, #3A1610, #1A0A07, #160D09) is left untouched.
AMBER_FAMILY = ["FFA31A", "FFC46B", "9A6A22", "6B4E20", "4A3A22",
                "3A2C18", "2A1F12", "241C13", "1C150E", "140C08", "0B0907"]

HUE_FROM, HUE_TO = 36.0, 172.0          # amber -> green-teal


def rotate(hex6, degrees):
    r, g, b = (int(hex6[i:i + 2], 16) / 255.0 for i in (0, 2, 4))
    h, l, s = colorsys.rgb_to_hls(r, g, b)
    r, g, b = colorsys.hls_to_rgb((h + degrees / 360.0) % 1.0, l, s)
    return "%02X%02X%02X" % tuple(round(c * 255) for c in (r, g, b))


def palette(name):
    if name == "teal":
        shift = HUE_TO - HUE_FROM
        return {("#" + h): ("#" + rotate(h, shift)) for h in AMBER_FAMILY}
    return {}


def body_of(path, subs):
    s = io.open(path, encoding="utf-8").read()
    inner = s[s.index("<x-dc>") + 6:s.index("</x-dc>")]
    inner = re.sub(r"<helmet>.*?</helmet>", "", inner, flags=re.S).strip()
    for old, new in subs.items():
        inner = re.sub(re.escape(old), new, inner, flags=re.I)
    return inner


def main():
    name = sys.argv[1] if len(sys.argv) > 1 else "amber"
    subs = palette(name)
    out = "screens-local.html" if name == "amber" else "screens-local-%s.html" % name

    canvas = json.load(io.open("canvas.json", encoding="utf-8"))
    titles = {a["file"]: a.get("title", a["file"]) for a in canvas["artboards"]}
    notes = {n["id"]: n["text"] for n in canvas["annotations"]}
    note_for = {"Splash.dc.html": "splash-note", "Main.dc.html": "drive-note", "Tyres.dc.html": "tyres-note",
                "Health.dc.html": "health-note", "Trip.dc.html": "trip-note",
                "Tilt.dc.html": "tilt-note", "Faults.dc.html": "faults-note",
                "Alert.dc.html": "alert-note"}

    cards = []
    for f in SCREENS:
        note = notes.get(note_for.get(f) or "", "")
        note_html = ""
        if note:
            note_html = '<div class="note">%s</div>' % "".join(
                "<p>%s</p>" % p.replace("\n", "<br>") for p in note.split("\n\n")[1:])
        cards.append('<figure><figcaption>%s</figcaption><div class="screen">%s</div>%s</figure>'
                     % (titles.get(f, f.replace(".dc.html", "")), body_of(f, subs), note_html))

    accent = subs.get("#FFA31A", "#FFA31A")
    flat = lambda t: t.replace("\n\n", "<br><br>").replace("\n", " ")

    html = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Jeep Carputer Screens</title>
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=Barlow+Condensed:wght@400;500;600&family=Barlow+Semi+Condensed:wght@400;500;600&display=swap">
<style>
  :root { color-scheme: dark; }
  body { margin:0; background:#17130F; color:#C4A882;
         font-family:'Barlow Semi Condensed','Arial Narrow',system-ui,sans-serif;
         padding:28px 24px 60px; }
  h1 { font-size:22px; font-weight:600; letter-spacing:.14em; color:%(accent)s;
       margin:0 0 4px; text-transform:uppercase; }
  .sub { font-size:14px; color:#7A6449; margin:0 0 6px; max-width:66ch; line-height:1.5; }
  .axis { font-size:13px; color:#8C7452; margin:0 0 28px; max-width:78ch; line-height:1.6;
          border-left:2px solid #3A2C18; padding-left:12px; }
  .sheet { display:flex; flex-wrap:wrap; gap:34px 30px; align-items:flex-start; }
  figure { margin:0; width:240px; }
  figcaption { font-size:11px; font-weight:600; letter-spacing:.16em; color:%(dim)s;
               text-transform:uppercase; margin-bottom:7px; }
  .screen { width:240px; height:320px; box-shadow:0 0 0 1px #3A2C18, 0 10px 26px rgba(0,0,0,.6);
            overflow:hidden; }
  .note { font-size:12px; line-height:1.55; color:#7A6449; margin-top:10px; }
  .note p { margin:0 0 7px; }
  @media (prefers-color-scheme: light) {
    body { background:#EDE7DE; color:#4A3C28; }
    .sub, .axis, .note { color:#6B5A42; }
  }
</style>
</head>
<body>
<h1>Jeep Carputer Screens</h1>
<p class="sub">Nine cards at true 240&times;320 for the Waveshare ESP32-S3 panel &mdash; a welcome screen, five swipeable peers, one drill-down, one pre-mapping state, one overlay. Zoom to judge legibility; these are life size.</p>
<p class="axis">%(axis)s</p>
<div class="sheet">%(cards)s</div>
<p class="axis" style="margin-top:34px;">%(stale)s</p>
</body>
</html>""" % {"accent": accent, "dim": subs.get("#9A6A22", "#9A6A22"), "axis": flat(notes["swipe-axis"]),
              "cards": "".join(cards), "stale": flat(notes["stale-note"])}

    io.open(out, "w", encoding="utf-8").write(html)
    print("%s  <-  %d screens, palette '%s'" % (out, len(cards), name))
    if subs:
        for k, v in list(subs.items())[:4]:
            print("   %s -> %s" % (k, v))


main()
