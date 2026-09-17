---
paths:
  - "core/Faults.*"
  - "core/FaultScan.*"
  - "core/Kwp.*"
  - "src/FaultStore.*"
---

# DTC reading, muting and clearing

`core/Faults.h` plus a scan driver in `main.cpp`. Everything underneath already
existed: `kwpBuildReadDtc`, `kwpParseDtcList`, `kwpDtcToSae` (verified against
all 1152 pairs) and 1152 descriptions in `core/SignalDb.cpp` (~61 KB of image,
binary-searched by (ecu, code)).

## The scan

- **One module per pass of carTask** — twelve modules, half on CAN-C where they
  may never answer, so doing it in one go would cost five seconds of not
  listening.
- **A module that did not answer is not a module with no faults.** `answered` and
  `unreachable` are separate bitmasks, and HEALTH says `none (7 asked)` rather
  than a bare `none`.
- **AUTOMATIC: once when the link comes up, then every 5 minutes**
  (`FAULT_SCAN_PERIOD_MS`, with the wrap-safe comparison, in `core/FaultScan.h`).
  The automatic start is gated on `gLinkUp`, not on boot — waiting for the link
  is what makes "once on link" and "every five minutes" one code path. A walk cut
  short by the link dropping sets the due stamp to *now*, so it is re-owed the
  moment the link returns.
- **RE-READ only brings the next scan forward.** The control sits on its own
  result, which is the point: you read `3 stored`, you want to know if it still
  holds. A single tap is enough because the read is harmless — unlike the TRIP
  reset two pages over, which destroys something and earns a long press and a
  confirmation.
- **There is ALSO a pre-existing `readDtcs()`** at link bring-up for RADIO, CCN
  and WCM. It only logs — it does not feed the fault store — and it is kept
  because a module answering (even with zero faults) proves ISO-TP, KWP and the
  adapter end to end. Two paths build the same `18 00 FF 00`; change one and look
  at the other.
- **The list is SORTED by the code AS DRAWN, in `faultsEndScan()`.** Not by the
  raw `uint16_t`: `kwpDtcToSae` takes its letter from `"PCBU"` (bits 15..14), so
  raw ascending puts every P before every C and reads as no order at all. Sorting
  the rendered five characters gives B, C, P, U. Stable, so two modules reporting
  the same code keep scan order. Sorted at END-of-scan only — per record would
  shuffle rows under a finger mid-walk, and the open dialog holds an index.

## Muting

**A HARD mute, and it is the driver's decision, not a finding.** Matched on
(ecu, code) so it follows the fault rather than its list position, preserved
across `faultsBeginScan()`, persisted to NVS by `src/FaultStore`. This car
carries faults that are permanent and known; without muting the count never
changes, so nobody reads it and the one genuinely new fault is lost.
`faultsActiveCount()` is what HEALTH shows.

**A muted fault stays in the list**, reading `not important`, dimmed, with
UNMUTE in its dialog. Removing it would make the list a lie; the only thing
muting changes is the COUNT.

## Clearing — BROKEN ON THE CAR, and we know why

Reported by the owner: *"we have implemented clearing one particular DTC in our
jeepdiag app and it was working fine. however our carputer fails to clear dtc."*
`D:\Repos\car\jeepdiag` is a KNOWN-WORKING reference against this vehicle over
this same gateway — when something diagnostic fails here, diff against it before
theorising. Three differences, and the first is sufficient alone:

| | jeepdiag (works) | carputer (fails) |
|---|---|---|
| preamble | **`10 92`** startDiagnosticSession, expects `50 92` | none |
| clear PDU | **`14 FF 00`**, all groups | `14 <hi> <lo>`, one code |
| `7F 14 78` | loops, up to 10 re-reads | returns it as the answer |

Addressing is IDENTICAL (`6A0/514` etc.), so that is not it.

1. **The response-pending hole is the killer, and it is ours.**
   `IsoTpClient::transact()` completes on the first whole PDU, so `7F 14 78` IS
   the answer as far as it is concerned; `faultClearIfAsked()` then takes the
   `ks != KWP_OK` branch and abandons the exchange. jeepdiag's capture shows
   `7F 14 78` **always** precedes the positive `54 FF 00`, ~80 ms later. So a
   clear that actually SUCCEEDED is reported as `FAILED`. This also silently
   costs every polled signal on a module that defers.
   `emulate-car.py`'s `pending once` reproduces it with no car.
2. **No `10 92` session.** Every read path here needs none and `core/Kwp.h` says
   so — but that says nothing about a WRITE, and the known-good path never omits
   it.
3. **The single-code form has never been observed working ANYWHERE.** The vendor
   app only ever emits `FF 00`. See `docs/open-questions.md` — `14 FF 00` wipes a
   module's whole fault memory, which is what the single-code form was chosen to
   avoid, so that one is a product question, not a code change.

**As shipped, CLEAR erases ONE code** — `kwpBuildClearDtc()` sends
`14 <hi> <lo>`, the group field carrying the specific code. A refusal is logged
and the row stays, which is what `FAILED` means. **The row leaves our list only
on a POSITIVE reply** (`faultsRemove()`), never optimistically: showing a cleared
fault the car still holds, which reappears at the next read, is the
frozen-but-plausible failure applied to the one screen whose job is saying what
is wrong.

**The clear request travels as (ecu, code), not a list index** — unlike the
mute. A scan finishing between the tap and carTask consuming it re-sorts the
list, and an index would name a different fault; for a reversible mute that is a
wrong row, for an erase it is the one mistake that cannot be undone.
`requestFaultClear()` writes the code FIRST and the ecu LAST, so the pair is
complete whenever the reader sees a valid ecu.

## The FAULTS drill-down (UI)

**The first DRILL-DOWN** — the vertical axis the canvas notes describe and
nothing had used. NOT a peer on the swipe axis: a fault list is read
deliberately, from a standstill, so it should cost a flick from nobody who is not
going there. It covers the page area only, leaving the status bar and dots
showing, because the dots still say HEALTH — you are *inside* health, not
somewhere else. Tap FAULTS to enter, BACK to leave, RE-READ to scan.

- **Built ON DEMAND and deleted on the way out.** ~42 objects resident for a page
  opened maybe once a month. `faultPageShow()` owns both halves and nulls
  `W.faultTitle` / `W.faultRow[]` / `W.faultState[]` on the way out.
- **The BACK / RE-READ bar is 1.5× its original height** (60px portrait, 39
  landscape, owner's instruction): two words on a 16px line had hit targets only
  as tall as the bar, and nothing else competes for the height — the list just
  scrolls one row less.
- **A row is 40px and carries the DESCRIPTION.** `UI_FAULT_ROW_H` was 20 — one
  line, and half the 44px a fingertip needs, on the page read with a thumb. The
  second line is the description from `dtcLookup()`, dotted at one line (220px ≈
  34 chars against a median 40), with a rule under each row so the pair reads as
  one target.
- **One tap handler for sixteen rows**: the row is worked out from touch y plus
  scroll offset (`y / UI_FAULT_ROW_H`) rather than sixteen invisible tap targets.
- **Tapping a row OPENS it; it does not mute.** Muting from the list put the
  decision to ignore a fault behind the same gesture used to read one, on a
  target showing a code and a state and no reason. `buildFaultDlg` shows code,
  module, status and the full description over three wrapped lines, with CLOSE
  and MUTE/UNMUTE — **the button's word is what it will DO**. MUTE posts the same
  request the row used to, so the dialog stays open and flips to UNMUTE when
  carTask's change lands: the undo is the same button.
- **The dialog is a CHILD of the drill-down, scrim included.** BACK deletes the
  page and takes the dialog with it, so `faultDlgForget()` only drops pointers;
  `faultDlgShow(false)` is the path that deletes. The scrim is translucent
  (`LV_OPA_70`) so the list stays visible behind the card, and it is CLICKABLE so
  it absorbs the taps the card does not — without that flag
  `lv_indev_search_obj()` carries on to the siblings underneath, which is the
  exact bug that made the TRIP confirm dialog dismiss itself wherever it was
  touched.
- **The dialog tracks its fault by (ecu, code) AND index**, and CLOSES if a
  RE-READ renumbers the list — an index alone would quietly start describing a
  different fault.
- **The three controls are arranged PER BAND**, like SETTINGS. Landscape has the
  width and not the height: 280−24 of card divides into three 78px buttons on an
  11px gap (`UNMUTE` is 55px at `font_label_14`), where 218px of page cannot take
  two 46px rows plus the description. Portrait has the height and not the width
  (three 88s want 288px of 192), so CLOSE and MUTE keep their row and CLEAR goes
  underneath, full width, in `UI_CRIT` — which is the better arrangement anyway,
  since the destructive control is not adjacent to the one meaning "never mind".
- **CLEAR arms on the first tap and acts on the second** (`UI_CLEAR_ARM_MS`, 4 s),
  and the button says which of its four states it is in: `CLEAR`, `CLEAR?`,
  `CLEARING`, `FAILED`. No second dialog — a dialog on top of a dialog is worse
  than the problem it solves — and the arming lapses on its own, so a dialog left
  open cannot be finished by a bump on a rough road. `FAILED` after
  `UI_CLEAR_WAIT_MS` is not a guess: the request went to carTask, carTask
  transacted, and the row is still here. A successful clear takes the row with it
  and the dialog closes itself.
- **`faultPoolSample()` logs the pool when the drill-down opens** (`faults page:
  lvgl heap …`) — sampled under the lock, printed by `uiTask` outside it. This is
  the page that grew and the one whose allocation failure is a bare `while(1)`.
  **Look at that line after touching this page.**
