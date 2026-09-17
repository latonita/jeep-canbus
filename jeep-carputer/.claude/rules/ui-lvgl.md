---
paths:
  - "src/Ui.*"
  - "src/Touch.*"
  - "src/Display.*"
  - "src/UiTheme.*"
---

# LVGL, touch and the render loop

## Allocation is a silent freeze

The pool is FIXED (`CONFIG_LV_MEM_SIZE_KILOBYTES`, expand 0, currently 96 KB —
see the build rules for why both 64 and 128 fail) and `LV_USE_ASSERT_MALLOC=y`,
whose handler is a bare `while(1)`. **Allocation failure is a freeze, not an
error** — decode the backtrace and it lands in `lv_draw.c`. When rebuilding the
UI: free the old tree *before* building the new
one, and watch the `lvgl heap` log line. Measured on the board: portrait
`39360/59984 used, frag 8%, largest free 18984`, landscape `39308/60024` — those
are 64 KB-pool figures and are why it is now 96, because draw tasks come out of
the same pool at RENDER time and never appear in a figure logged at BUILD time.
Six rebuilds back to back oscillate inside a 76-byte band with no monotonic
growth, so the teardown leaks nothing. If `largest free` ever drops toward a
page's worth of widgets, the freeze is coming.

**Never `ESP_LOGx` while holding `lvgl_port_lock`.** USB-Serial-JTAG blocks
until the host drains; it stalls the renderer and the instrument lowers the
measurement it is taking. Sample under the lock, print outside it — that is what
the frame-rate line and `faultPoolSample()` both do.

**Pages that are only opened deliberately are built ON DEMAND and deleted on the
way out.** A hidden page does not draw but it still OCCUPIES, and what ran out
is what RENDER needs, not what build needs. Whatever owns the build must null
its widget pointers on the way out — a dangling pointer that is only safe
because of a guard somewhere else is the next person's crash.

**A cache that lives OUTSIDE a widget must be cleared in `buildRoot()`.**
`s_flowLit` survives a tree teardown and `setFlow()` only repaints on a change,
so a palette swap or a flip left a freshly-dark bar agreeing with a stale count
and never lighting again. The compass tape is the second instance —
`s_compDrawn` caches tick POSITIONS, `s_compShown`/`s_compBlank` go with it — so
this is a pattern, not an anecdote.

## Touch

**THERE IS ONE TOUCH RELEASE HOLD, `TOUCH_RELEASE_HOLD_MS` = 60 ms.** A
gesture-dependent hold (20 ms moving, 60 still) produced every input complaint
on the panel at once — taps firing twice, swipes arriving as taps, long-press
unreachable. Two faults, either sufficient: the "moved recently" test was armed
by the contact STARTING (`!s_pressed` is true on the first frame of every
contact, so a still tap counted as moving for 200 ms), and **20 ms is SHORTER
THAN THIS CONTROLLER'S REPORTING INTERVAL**, so a real swipe was chopped.

**A fragment is not a short press, it is a NEW press.** LVGL clears `act_obj` on
release and the next press takes its new-object branch, zeroing `scroll_sum`,
`scroll_dir`, `scroll_obj`, `gesture_sum`, `gesture_sent`, `press_moved` and
`pr_timestamp` (`lv_indev.c:1317-1338`) — and **LVGL has no debounce anywhere**.
Every fragment restarts from zero travel: never 10 px inside one press, so no
scroll and no gesture, so CLICKED on release, twice over, and never the 400 ms
`LONG_PRESSED` needs.

Once INT has proven itself a level the hold drops to one poll
(`TOUCH_RELEASE_HOLD_INT_MS`). `lv_indev_set_scroll_throw(indev, 4)` covers what
the controller's sub-poll frame rate still eats: LVGL throws velocity from the
last eight per-poll deltas weighted `1 - t/99ms`, so a hold that repeats
coordinates after the lift injects zero-motion samples and ages the real ones,
collapsing a flick's measured velocity 5-10× — the swipe snap-back.

## Clicks, scrolls and gestures

**LVGL suppresses `LV_EVENT_CLICKED` when a press becomes a scroll**
(`lv_indev.c`: `if(scroll_obj == NULL)`), and every page lives inside a
horizontal scroller. So any latch cleared only on CLICKED gets stuck: a
fingertip that drifts past the scroll limit produces the long press and then no
click at all. **Clear those on `LV_EVENT_PRESSED`, which always arrives** —
`s_priceHeld` and `s_tripLongFired` both do. For the same reason the fuel price
commits from `uiTask` on a dirty flag rather than from the click, or a held
adjustment ending in a swipe would never reach NVS.

**That suppression only works if the list CAN scroll.** `lv_indev_scroll.c` will
not consider an object a scroll candidate unless `scroll_top > 0 ||
scroll_bottom > 0`. Five faults at 40px is 200px of portrait's 206, so there was
nothing to scroll, so every drag arrived as a click. `SCROLL_ELASTIC` /
`SCROLL_MOMENTUM` are not the fix — they are LVGL defaults, `stripChrome()` only
strips `SCROLLABLE` and `CLICKABLE`, and with nothing off-screen there is no
scroll to be elastic about. A short list answers the drag by doing nothing; the
header count and the auto scrollbar are what say whether there is more below.

**A DRAG IS TOLD FROM A TAP BY LVGL'S OWN GESTURE, once, for the whole
instrument**: `touchWasDrag()` in `Touch.cpp`, consulted by `tapIsDrag()` at the
top of every `LV_EVENT_CLICKED` handler in `Ui.cpp`. It is the COMPLEMENT of
click suppression, not a replacement — `indev_gesture()` opens
`if(indev->pointer.scroll_obj) return;` (`lv_indev.c:1775`), so it fires exactly
when no scroll was claimed, which is the case LVGL cannot suppress. That hole
was real on five widgets: the page scroller is `LV_DIR_HOR`, so a VERTICAL drag
finds no candidate and LVGL delivers a plain click — dragging on the HEALTH
faults strip opened the drill-down, on a SETTINGS row toggled the setting, on a
price button spent a cent.

Two settings, both FORCED rather than chosen:

- **`gesture_min_distance` = 10**, matching `LV_INDEV_DEF_SCROLL_LIMIT`
  (`lv_indev.c:32`). At LVGL's default 50 there is a 10..50 px dead band where a
  drag claims no scroll AND raises no gesture, and a release there is a plain
  click.
- **`gesture_min_velocity` = 0.** `indev_gesture()` zeroes its accumulator on any
  poll where both axes moved less than this (`lv_indev.c:1787-1791`), and the
  read callback deliberately repeats the last coordinate while bridging a
  reporting gap — so mid-drag polls legitimately carry a zero delta. At the
  default 3 those polls wipe the travel.

**Subscribed on the INDEV, not a screen or widget.** `lv_indev_send_event`
forwards PRESSED and GESTURE to indev subscribers (`lv_indev.c:1888-1900`,
`:1818`), and an indev outlives every screen — which matters because
`rootTeardown()`/`rootRebuild()` delete the whole tree on a palette swap or a
flip. That is exactly where the latches it replaced leaked.

## Page scrolling

- **`LV_OBJ_FLAG_SCROLL_ONE`**, not a hand-rolled clamp on the throw animation.
  A clamp limits only the throw, so a long drag still slides past two pages
  under the finger; and it anchors on a *rounded* `scroll_x` sampled at
  SCROLL_BEGIN, an intermediate value when a new drag starts during an in-flight
  settle — so a quick second flick got clamped back to the page it was leaving.
- **`UI_PAGE_ANIM_MS` is written straight onto the animation in
  `onScrollBegin`.** LVGL derives page settle from a speed encoding that always
  saturates `SCROLL_ANIM_TIME_MAX` for a full-page move and there is no Kconfig
  for it; `resolve_time()` runs after that callback, so writing it there wins.
- **`LV_EVENT_SCROLL_END` fires TWICE**, the first from the same call that only
  *starts* the animation, before `scroll_x` has moved — so the page dot showed
  the page being left for the whole settle and then corrected, which reads as the
  swipe not having taken. The dot is set to the DESTINATION in `onScrollBegin`.

## Rotation

**The rotation is one call**: `lv_display_set_rotation(s_disp, ROTATION_90)` in
`uiApplyOrientation()`.

- LVGL swaps the resolution it reports, which is what `UI_W` / `UI_H` read.
  Below the geometry block in `Ui.cpp`, `LCD_H_RES` / `LCD_V_RES` mean the
  *panel* and appear only in buffer sizing and the port config.
- `esp_lvgl_port` hears `LV_EVENT_RESOLUTION_CHANGED` and writes
  `swap_xy`/`mirror` into the ST7789, composed against `disp_cfg.rotation`.
- `lv_indev` runs every touch point through `lv_display_rotate_point()` before a
  widget sees it.

**So `Touch.cpp` is not part of the rotation and must not be made part of it.**
Only one of the two is ever told, because the digitiser is glued to the glass
and does not turn. Adding a `swap_xy` there to "match" would rotate the input
twice. The same goes for which 90° is correct (see `docs/open-questions.md`) —
if landscape comes up upside down, the fix is the one token in
`uiApplyOrientation()`, not `Touch.cpp` or `Display.cpp`.

**`disp_cfg.rotation` all-false is NOT a no-op.** `lvgl_port_add_disp()` calls
its rotation update unconditionally, so those three zeroes are written into
MADCTL and *replace* what `Display.cpp` set. They are the panel's orientation.

**Free before build, still.** `rootTeardown()` / `rootRebuild()` are the one
path for both the palette swap and the flip; the rotation happens in the gap
between them with one bare screen alive. That gap is the only safe moment —
`lv_display_set_rotation()` resizes every screen and dirties every layout.

## Benchmarking

Full-screen frame 65 → 29 ms, idle 4.5 → 2.8 ms, page scrolls 73-78 → ~43 ms.
Measure without touching the glass: the splash-to-root fade forces full-screen
frames, and `avg render` for them is the first two `ui:` lines after boot. The
panel was never the problem — 240×320×16bpp at 80 MHz SPI is 15.4 ms.
