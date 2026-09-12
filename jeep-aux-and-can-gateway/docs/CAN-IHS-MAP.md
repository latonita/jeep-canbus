# CAN-IHS message map — 2005 Jeep Grand Cherokee WK

Interior CAN-B ("IHS"), **83.3 kbps**. This is the synthesis document: every id our
in-radio ATmega328 saw on the bus, cross-referenced against our own KWP2000 reads
(the source of truth) and our dash readings. Our car is **FCM/CCN-based** (Front
Control Module + cluster).

## Confidence tags

| Tag | Meaning |
|---|---|
| **CONFIRMED** | Byte position *and* value validated against our own KWP read, the dash, or firmware the board already acts on. |
| **PLAUSIBLE** | Byte position fits and our observed value is consistent, but scaling / enum is unverified on the WK. |
| **UNKNOWN** | Seen on the bus, undecoded. Sample bytes recorded, no meaning invented. |

## Sources

- **Broadcast captures:** `logs/drive-2026-09-11.txt`, `ATMA` snapshots `[start]`
  (17:00:24, 45 ids) and `[mid]` (17:05:14, 47 ids). No `[end]` snapshot exists.
  **47 distinct broadcast ids** were captured across the two snapshots (`[mid]`
  adds `1B0` and `1B5`, which were absent from `[start]`).
- **★ Long real bus logs from THIS car (2015 RPi CAN logger), `jeep-can-b-logs/`.**
  Five distinct recordings, **~365 000 frames total** — far stronger than the ~5 s
  snapshots above. Referenced below by their `CANB-analysis.md` labels:
  **A** (`playback1.log`/`jeep.log`/`ride.txt`, 107 978 frames, a 2.4 km drive,
  engine already running); **B** (`playback.log`, 126 183 frames, a 4.5 km drive with a
  *full ignition cycle* OFF→ACC→RUN→CRANK→RUN→OFF); **C/D** (`radio-wait-ignition` /
  `radio-ignition-on`, the **only captures with real millisecond timestamps** — update
  rates below come from **D**); **E** (`playback-radio-on-without-radio.log`, the
  **radio-unplugged differential**). All are VIN `1J8HCE8M45Y588369`, odometer
  ~103 168 km (2015) — i.e. this same truck, ~143 000 km before the 2026 snapshots.
  Prior analysis lives in `jeep-can-b-logs/CANB-analysis.md` and `names.dic`.
- **★ Vendor KWP PID database (AppcarDiagFCA extraction), `extracted/`.** Authoritative
  signal names / scalings / enums for request-response PIDs — used here to cross-check
  the *meaning and units* of decoded broadcast bytes (`pids.csv`, `pid_scaling.csv`,
  `pid_enum_values.csv`, `ecus.csv`).
- **KWP truth / validation:** `docs/CAN-B-CAPABILITIES.md`, `docs/PROBE-LOG.md`.

> **What the new sources changed (2026 refinement pass).** Confidence tally:
> **CONFIRMED 22, PLAUSIBLE 8, UNKNOWN 17.** Seven ids were promoted to CONFIRMED on
> real-car log evidence (`000 006 1B4 1B6 1C8 210 2C0`), three UNKNOWNs were resolved to a
> subsystem (`1A7`→engine, `1AB`/`1BB`→head unit). The real logs **pinned the `0x000` key
> enum** and **reassigned `0x11D` to the stalk switch** (the flasher state is `0x006`), and
> **correct one of our own guesses** (`0x159` B0–B3 is *not* a runtime counter — it is
> constant). Every change is cited to its capture (A/B/D/E) or to the vendor DB inline.

---

# Part A — Automatically broadcast (unsolicited)

Every id our board saw appear on the bus on its own. Sample bytes are quoted
verbatim from the two snapshots; where a byte moved between them it is called out,
because a byte that changes with the world is worth more than one that sits still.

## A.1 CONFIRMED (22 ids)

### `0x002` — Engine RPM + vehicle speed + distance counters — CONFIRMED
- **Source:** cluster (CCN). **Rate:** 50 ms (fastest telemetry id on the bus; real
  timestamps in capture D confirm ~50 ms exactly).
- **Samples:** `[start] 04 C3 08 0C 17 FA 17 58` · `[mid] 02 6B 00 00 C4 C6 C4 B3`
- **Decode (now fully pinned — every byte matches the vendor `CCN 213E` layout):**
  - RPM = `(B0<<8)|B1`. `[mid]` `02 6B` = **619 rpm** — an exact match for our KWP
    `CCN 213E` warm-idle read (619/621 rpm). `[start]` `04 C3` = 1219 rpm (cold fast-idle).
    Vendor DB: `213E` "Engine Revolutions Per Minute", bits 16-31, **×1.0 rpm**.
  - Speed = `((B2<<8)|B3) >> 7` km/h = **×0.0078125 km/h**. Vendor DB: `213E`
    "Vehicle Speed", bits 32-47, factor 0.0078125 — i.e. `/128`, exactly our shift.
    **Real-log range (capture B, full drive): 0–82.9 km/h**, RPM 0–4163.
  - **B4:B5 and B6:B7 = the two 16-bit wheel-distance counters.** Vendor DB names them
    `213E` "Distance Traveled by **Left** Wheel" (bits 48-63) and "**Right** Wheel"
    (bits 64-79), each **×4.0 cm/count** → ≈25 000 counts/km, matching the ~25 100
    counts/km measured over capture A's odometer climb (CANB-analysis §2.1). `[mid]`
    `C4 C6 C4 B3` is byte-identical to the KWP `CCN 213E` consecutive frame.
- **Validation:** RPM matched KWP idle to the rpm; both distance words match the KWP
  frame *and* the vendor `213E` per-wheel-distance scaling; speed range sane over a real
  drive. This is the single most thoroughly cross-validated id on the bus.

### `0x014` — Odometer — CONFIRMED
- **Source:** cluster. **Rate:** ~100 ms (real timestamps, capture D: 99 ms).
- **Samples:** `[start] 25 94 30 FF 1C 8A 42` · `[mid] 25 94 41 FF 1B CB 3F`
- **Decode:** odometer = `uint24(B0..B2) / 10` km. `[mid]` `0x259441` = 2 462 785 →
  **246 278.5 km**, matched the dash exactly. **Vendor DB authoritatively confirms the
  formula:** `FCMCGW 21A3` "Odometer", 24-bit, **×0.1 km** (offset 0) — identical.
- **★ Confirmed live and monotonic over two real drives (2015 logs).** In capture A the
  uint24 climbs `1 031 679 → 1 031 703` = **103 167.9 → 103 170.3 km, +2.40 km**, strictly
  monotonic across 8 003 frames; in capture B `1 032 015 → 1 032 060` = **+4.50 km**,
  again strictly monotonic. Both drives independently prove it is a live climbing counter,
  so **trip distance is free** (diff two reads). (The 2015 logs read ~103 168 km vs the
  2026 dash 246 279 km — the same ÷10 formula, ~143 000 km of driving apart. Sanity check
  passes at both ends of a decade.)
- **B4–B6 vary** (`b3 = FF` always). In capture A `b6` sweeps **4…49** and looks like an
  instantaneous fuel-economy / trip channel (CANB-analysis §2.1); `b4` is slow (0x11–0x16),
  `b5` noisy. Secondary trip/segment counters, exact scaling unconfirmed — leave undecoded.
- **Also request-readable** as `WCM 21 14` (bytes 2-4 = `25 93 C5`), see Part B — same value.

### `0x015` — Ambient temperature + battery voltage — CONFIRMED
- **Source:** cluster. **Rate:** ~100 ms.
- **Samples:** `[start] 92 8C 26 FF 00 00` · `[mid] 91 8B 29 FF 00 00`
- **Decode (WK scaling):**
  - **B1 = battery voltage / 10 V.** `[start]` `8C` = **14.0 V**, matches our KWP
    `CCN 2134` running voltage (13.9 V). `[mid]` `8B` = 13.9 V.
  - **B0 = ambient temp `raw × 0.5 − 40` °C.** `[start]` `92` = 146 → **33 °C**;
    `[mid]` `91` = 145 → 32.5 °C. Both match `CCN 2134` ambient (32.5 / 33.5 °C).
    B0 is byte-identical to the `CCN 2134` ambient byte, so **`0x015` mirrors `CCN 2134`.**
  Our WK uses the CCN `raw×0.5−40` convention.

### `0x01B` — VIN (multi-frame ASCII) — CONFIRMED
- **Source:** ECU→cluster relay. **Rate:** ~100 ms.
- **Samples:** `[start] 00 31 4A 38 48 43 45 38` · `[mid] 01 4D 34 35 59 35 38 38`
- **Decode:** B0 = frame index; B1–B7 = 7 ASCII chars.
  - index 0 → `31 4A 38 48 43 45 38` = `"1J8HCE8"`
  - index 1 → `4D 34 35 59 35 38 38` = `"M45Y588"`
  - Concatenated: **`1J8HCE8M45Y588…`** = owner-confirmed VIN `1J8HCE8M45Y588369`.
- **Note:** on our car both index 0 and 1 are populated, so the identity relay is healthy.

### `0x02C` — Coolant temperature — CONFIRMED
- **Source:** cluster/powertrain relay. **Rate:** ~500 ms.
- **Samples:** `[start] 3A 72 24 64 00 00` · `[mid] 22 81 00 64 00 00`
- **Decode:** **B1 = coolant `raw − 40` °C.** `[start]` `72` = 114 → **74 °C**, matches
  KWP `CCN 2140` (74 °C). `[mid]` `81` = 129 → 89 °C, matches `CCN 2140` at that time
  (`61 40 81` = 89 °C). B0 and B3 (`64`) not yet decoded.

### `0x09F` — Radio mode / source — CONFIRMED (firmware-known)
- **Source:** radio. **Samples:** `[start] 11 13 E2 00 FF FF FF 1F` · `[mid] 01 14 0A 00 …`
- The firmware already acts on this id; B0 low nibble encodes the source (per
  `lib/Core/JeepCan.h`, nibble `6` = AUX). Confirmed in prior probing by watching AUX select.

### `0x394` — Radio panel buttons — CONFIRMED (firmware-known)
- **Source:** radio/panel. **Samples:** `[start] 17 03 E2 00 00 00` · `[mid] 07 04 0A 00 00 00`
- B3 is a button bitmask the firmware decodes; confirmed by pressing panel buttons.

### `0x3D0` — Radio volume / balance / fader / tone — CONFIRMED
- **Source:** radio. **Rate:** ~1 s. **Samples:** `[start] 0A 0A 0A 0A 0A 0A FF` ·
  `[mid] 0E 0A 0A 0A 0A 0A FF`
- **Decode (validated by our own volume change):** B0 volume, B1
  balance, B2 fader, B3 bass, B4 mid, B5 treble, B6 = state `FF`. **B0 moved `0A → 0E`
  when the volume was raised** — confirms B0 = volume.

### `0x400` `0x402` `0x414` `0x415` `0x416` `0x41C` `0x41E` — Module "Device Status" heartbeats — CONFIRMED (as a family)
- **One family, one structure:** `B0 = FD` (alive), **`B1 = module id`** (differs per
  id), `B2 = 3F` or `BF`, `B3..B7 = FF`. Rock-steady across both snapshots.
- **Samples:** `400 FD 02 …` · `402 FD 14 …` · `414 FD 15 …` · `415 FD 16 …` ·
  `416 FD 1C …` · `41C FD 1E BF …` · `41E FD 00 BF …`
- **What's CONFIRMED:** the heartbeat structure and B1=module-id convention (our own
  observation). Treat these as presence/liveness beacons, useful for a "which modules are
  on the bus" panel.
- `41C`/`41E` carry `B2 = BF` (vs `3F` on the others) — a distinct sub-family.
- **NM ring refined by the 2015 logs (capture E):** IDs are `0x400 + node_addr`; `B0=FE`
  = "alive/ring-join" (B1 = own address), `B0=FD` = "ring data/token" (B1 = **successor**).
  Ring order `00→02→14→15→16→1C→1E→00`, ~576 ms/frame (real timestamps, capture D). The
  **radio unplug (capture E) proved node `0x16` = the head unit** (`416` vanishes with it),
  and **node `0x1C` (`41C`) joins the ring only while ignition is ON** — so `41C`'s owner
  is an ignition-switched module. Exactly **7 modules** on this CAN-B segment.

### `0x000` — Key / ignition position — CONFIRMED (real enum)
- **Source:** FCM/cluster. **Rate:** 100 ms (capture D: 99 ms). **Sample:** `[start]/[mid]
  85 00 00 00 00 00` (steady, engine running). Only B0 carries data; B1–B5 = 00.
- **★ Real enum observed directly** by watching B0 walk a full ignition cycle in capture B
  and the wake-up in capture E:
  - `0x00` bus wake-up (pre-OFF, capture E only) · **`0x01` OFF** · **`0x63` ACC / key-on** ·
    **`0x85` RUN** · **`0xA9` CRANK**.
  - Capture B B0 sequence: `63 → 85 → A9 → 85 → 01` (ACC→RUN→crank→run→off) — a textbook
    key cycle. Our steady `0x85` = **RUN**, consistent with engine-running captures.
- Our FCM car uses the encoding `01/63/85/A9`. This is a CONFIRMED discrete ignition-state
  byte, validated by the physical key cycle.

### `0x006` — Turn-signal / hazard state — CONFIRMED
- **Source:** cluster/FCM. **Rate:** 100 ms (capture D: 99 ms). **Samples:** 2026 snapshot
  `18 22 04 3C 3C 02 58`; 2015 running idle `18 22 04 00 00 02 58` (B0 = `0x18` in both).
- **★ Real enum observed directly (captures A/B):** **B0** is a turn/hazard bitfield —
  `0x18` idle (running), `0x19` **left**, `0x1A` **right**, `0x1B` **hazard**, `0x38` an
  extra flash bit, `0x00` key-off. All values seen live during the two drives; matches the
  2015 `names.dic` ("18 idle, 19 left, 1A right, 1B hazard"). B1–B6 ≈ constant
  (`22/20 04 00 00 02 58`). Real behavior is specifically the **turn-signal/flasher state**.
- **NB:** the *stalk switch inputs* live on `0x11D` (A.2); `0x006` is the resulting
  flasher/lamp state. Both are lighting-related but distinct.

### `0x1B4` — Compass heading — CONFIRMED (our finding, now firmed up)
- **Source:** cluster (mirrors `CCN 2152`). **Rate:** 500 ms (capture D: 496 ms).
- **Samples:** `[start] 06 8A 07 00 00 00` · `[mid] 01 12 07 00 00 00`
- **★ B0 = 8-point compass heading.** Across capture B, **B0 takes exactly the 8 values
  `0x00–0x07` and nothing else** — an 8-point compass, matching the vendor `CCN 2152`
  compass PID (B0 = 8-point enum; our earlier probe read `02` = East). `B2 = 07` is fixed;
  `B1` (0x04–0xAE) is a finer field (candidate fine-heading, unmapped). B0 does **not**
  track speed or RPM, ruling out a motion counter. Promoted on the clean 0–7 range + the
  `2152` cross-reference.

### `0x1B6` — TPMS lamp state only (NOT per-wheel pressure) — CONFIRMED
- **Rate:** 1 s. **Sample:** `[start]/[mid] 00 18 FF 18 FF 18 FF`
- **★ Proven a lamp/status flag, not pressure, across 1 661 real samples** (813 in capture
  A + 848 in B): **only B0 ever changes** (`0x00`/`0x01`); B1–B6 are frozen at
  `18 FF 18 FF 18 FF` in every single frame. Our B0 = `0x00` = lamp off, agreeing with
  `CCN 2130` lamp = Off and the reality (TPMS light off). The `18 FF` triples are **not**
  tyre pressures (see the tyre-pressure verdict, Part B). B0 = warning bit; that is all
  this id carries.

### `0x1C8` — Exterior lights on/off — CONFIRMED (radio/lights differential)
- **Source:** light-switch cluster. **Rate:** 500 ms (capture D: 497 ms). **Sample:**
  `[start]/[mid] 02`.
- **★ Confirmed by the capture-E/B differential:** B0 = `0x00` **lights off** (captures
  C/D/E, parked/no-lights) vs `0x02` **lights on** (capture B, a lit drive) — and it moves
  in lock-step with the `0x210` dimmer (below). Only two values ever seen (`00`/`02`).
  Our `0x02` = headlamps on.

### `0x210` — Light-switch / panel-dimmer level — CONFIRMED (radio/lights differential)
- **Source:** light-switch cluster. **Rate:** 500 ms (capture D: 492 ms). **Sample:**
  `[start]/[mid] 03 C8`.
- **★ B1 = dash-illumination / dimmer level**, confirmed by the same differential:
  **`0x00` whenever lights are off** (captures C/D/E) and **`0xA4` (164) or `0xC8` (200)
  when lights are on** (capture B) — two detents of the dimmer thumbwheel. B0 = 00…03
  (state). Our snapshot `03 C8` = lights on, near-full brightness.

### `0x2C0` — Accelerator-pedal / throttle position — CONFIRMED (correlated with RPM)
- **Source:** ECU-relayed (gatewayed onto CAN-B). **Rate:** 100 ms (capture D: 100 ms).
  **Present only with the engine running** (absent from capture A's tail and from E).
- **Samples:** `[start] FF FF 33 18 8C FF 33 82` · `[mid] FF FF 17 0B 82 FF 17 0E`
- **★ Confirmed a pedal/throttle-demand signal on our car:** B0 = B1 = `FF` (unused);
  **B2 and B6 are always byte-identical** (safety-style redundant channel), and **B2
  correlates with engine RPM at r = 0.75** over capture B (range 0x17–0xEA). B4 idles at
  `0x7E` and swings 0x7B–0xFF (a second pedal/position channel); B7 is a flag byte
  (`02/0E/82/8E`). **Engine-demand data for free**, since our PCM is otherwise CAN-C-only.
  Exact %-scaling still unpinned (needs a pedal-sweep), but the *identity* is now confirmed.

## A.2 PLAUSIBLE (8 ids)

### `0x003` — Doors / brake / shifter state — PLAUSIBLE (brake bit firmed up)
- **Source:** cluster/FCM. **Rate:** ~450 ms (capture D: 262 ms, capture C: 496 ms — varies).
- **Samples:** `[start] 00 21 84 00 44 FF 00` · `[mid] 00 21 80 01 50 FF 00`
- **★ B3 bit0 = brake-pedal switch, confirmed by correlation:** in the 2015 logs B3 bit0
  correlates with RPM at **r = −0.59** (brake pressed → revs drop) and matches our snapshot
  `B3 00 → 01`. **B4 (0x20–0x52) is a gear/state code** (our `44 → 50`; unmapped scaling;
  vendor DB carries `CCN 213F` "PRNDL Display Request" / `FCMCGW 21B1` "PRND Status" but
  no confirmed byte match yet). Capture E (parked) shows B0/B2/B3/B4 all moving richly →
  this id is really **door / latch / switch state with the brake bit riding along**, not a
  simple "cluster display" line. Kept PLAUSIBLE: brake bit solid, the rest structural.

### `0x0D0` — Parking brake? — PLAUSIBLE (meaning unconfirmed)
- **Samples:** `[start]/[mid] 04 00`. **Rate:** ~1.15 s. Never asserted in any 2015 log
  (always `00 00` or `04 00`), so no "engaged" code was ever observed. Our KWP `CCN 2110`
  P-brake = False. B0 = `04`/`00`; the meaning is unconfirmed on our car.

### `0x11D` — Steering-column stalk (turn / flash / wiper switch) — PLAUSIBLE
- **Source:** SCM / cluster. **Rate:** 100 ms (capture D: 100 ms). **Samples:**
  `[start]/[mid] 00 00 00` (nothing active); 2015 logs show B0 ∈ {`00,08,10,40`}, B1 ∈ {0–3}.
- **★ Observed B0 bits:** `08`, `10`, `40` appear as discrete switch bits and B1 (0–3)
  steps like a wiper intermittent detent. This is the **stalk switch INPUT**, whereas the
  resulting flasher state is `0x006` (now CONFIRMED, A.1). The specific bit→action mapping
  is not yet confirmed from a labelled stalk-press capture — hence PLAUSIBLE, not CONFIRMED.
  **Reassigned:** the old "turn signals" label really belongs to `0x006`.

### `0x159` — Fuel level (B5) — PLAUSIBLE (the fuel-consumption lead; runtime-field guess REFUTED)
- **Source:** cluster. **Rate:** 1 s (capture D: 991 ms). **Samples:** 2026 `B1 FF FF FF
  00 47` · `[mid] B1 FF FF FF 00 46`. 2015 pattern `FE FF FF FF 00 xx`.
- **★ B5 behaves like a fuel level/quantity — real-drive evidence (see fuel verdict below).**
  In **both** 2015 drives B5 drifts *down* as distance climbs: capture A `30 → 28`
  (range 25–30) over 2.40 km; capture B `154 → 153` over 4.50 km — a slow monotone-ish
  decline in the burn direction, in two independent drives. The large *between-session*
  jump (≈28 in A vs ≈153 in B, sessions ~33 km apart) is consistent with a refuel between
  them; it rules out a small fixed-offset scaling and means **the absolute scale is still
  unpinned** (`0x47`=71 in 2026 still doesn't equal the KWP `CCN 2103` 50 % = raw×0.5).
- **CORRECTION to the earlier map:** B0–B3 are **not** a "runtime counter." Across 107 978
  frames B0 is constant (`0xFE`, or `0x00`=unavailable at key-off) and B1–B3 are frozen at
  `FF FF FF`. There is **no monotonic runtime field here.** The only free monotonic
  "how far/how long" counters on the bus are the odometer `0x014` and the wheel-pulse
  counters in `0x002`. For live fuel *consumption*, pair `0x014` distance with `0x159` B5
  level, or use the injection-rate id `0x1A7` (below) for instantaneous burn.

### `0x1A7` — Engine injection quantity / fuel rate — PLAUSIBLE (resolved from UNKNOWN)
- **Source:** ECU-relayed onto CAN-B. **Rate:** 100 ms running / ~400 ms key-off (capture
  D: 100 ms, capture C key-off: 397 ms). **Samples:** `[start] 02 41 00 00 00 00` ·
  `[mid] 00 84 00 00 00 00`. B2–B5 = 00 always.
- **★ B0:B1 (16-bit) correlates with engine RPM at r = 0.84 (capture A) / 0.79 (capture B)**,
  range 6–2498 over a drive — i.e. an **engine load / injection-quantity / fuel-rate**
  signal, *not* RPM itself (real RPM is `0x002`). Cross-reference: the vendor DB lists PCM
  `2103`/`2106` "Current
  Injection Mass" ×0.01 mm³/stroke — `0x1A7` B0:B1 ×0.01 → 0.06–24.98 mm³/stroke, a
  plausible diesel injection-mass span. Present in capture E (radio off), so **not** a
  radio id. Good instantaneous-consumption candidate; exact scaling unconfirmed.

### `0x1AB` — Head-unit (radio) status — PLAUSIBLE (resolved by radio-unplug differential)
- **Source:** head unit. **Rate:** ~3 s. **Samples:** `[start]/[mid] 03 00 00 00 00 00`.
- **★ Attributed to the radio by capture E:** `0x1AB` is one of exactly six ids that
  **vanish when the radio is unplugged** (`09F 394 1AB 1BB 3D0 416`) and are present in all
  radio-powered captures. Subsystem = head unit (radio); payload (`03 00 …`) undecoded.

### `0x1BB` — Head-unit (radio) status — PLAUSIBLE (resolved by radio-unplug differential)
- **Source:** head unit. **Rate:** 1 s. **Samples:** all zeros in every log.
- **★ Attributed to the radio by capture E** (same six-id vanish set as `0x1AB`). Subsystem
  = head unit (radio); payload is all-zero in these captures, meaning undecoded.

### `0x1F8` — Rear wiper control? — PLAUSIBLE (meaning unconfirmed)
- **Samples:** `[start]/[mid] 00` (off). **Rate:** ~2 s. Always `00` in every 2015 log.
- Our value is `00` in every capture; never exercised, so the meaning is unconfirmed on our car.

## A.4 UNKNOWN (17 ids) — seen, undecoded; subsystem hints added from the 2015 logs

> **Resolved out of this list:** `0x1A7` → engine injection quantity (A.2), `0x1AB` and
> `0x1BB` → head unit / radio (A.2). **Subsystem attribution from the scenario logs:** the
> ids that appear in the 2015 logs are **present in capture E (radio unplugged), so are not
> radio ids** — they are body / powertrain / safety. The bit-1-of-B0 toggle group
> (`010 011 1AD 1AF`, plus the out-of-list `110`) all flip that bit together in the same
> ~2 s window → **one shared transmitting module.** Bytes still undecoded; no meaning invented.

| Id | sample (2026 snapshot) | 2015-log behaviour + subsystem hint |
|---|---|---|
| `0x008` | `00 00 80 80 00 00` | **Not static:** B0 toggles `00`/`08`, B2/B3 toggle `40`/`80`. 1 s. |
| `0x010` | `41 01 15 71 63 80 2A 03` | Near-static; **B0 bit0 toggles `41`↔`43`** (shared-module alive bit). 2 s. |
| `0x011` | `3D 13 00 03 8B 40 00 00` | Near-static; **B0 toggles `3D`↔`3F`** (same toggle group). 2 s. |
| `0x012` | `00 30 02 40 00 00` | Seen, meaning unknown. 2015 logs: B0 stayed `00`; B1 read `0x10`; B2 walked `02`/`06`; **B4:B5 toggled `0000`↔`4999`** — `0x4999` is the same constant in `0x013` B4:B5 → shared SKREEM/security handshake source. |
| `0x013` | `80 00 00 49 99` | B0 `80`/`88`, B2 `00`/`02`; `49 99` constant is shared with `0x012` B4:B5 → SKREEM/security handshake. 500 ms. |
| `0x02E` | `1F 00` | Constant `1F 00`, 1 s. |
| `0x0EC` | `01` | Seen, meaning unknown. Static `01` in the snapshot. |
| `0x1AC` | `02` | **B0 toggles `03`↔`04`** every ~950 ms — a slow alternating status/alive flag. |
| `0x1AD` | `05 1E 1E 04 01 00` | Near-static `01 42 42 04 01 00`; **B0 bit0 toggles** (toggle group). The `1E 1E`/`42 42` pair is **not** tyre pressure. 2.7 s. |
| `0x1AE` | `00 00 00 00 00 00` | All zeros in every log. 1.4 s. |
| `0x1AF` | `01 E7 00 40 01 A4 08 FF` | Static except **B0 bit0 toggles `01`↔`03`** (toggle group). 2.5 s. |
| `0x1B0` | `0D DE 03 E8 0A A0 08 DE` | Static; four 16-bit words `0DDE 03E8 0AA0 08DE` = 3550/1000/2720/2270 (`03E8`=1000 looks like a scaled constant). Not tyre pressure. 5.4 s. |
| `0x1B2` | `00 02 00 00 00 00` | Only **B1 moves (0–7)**; rest zero. 1.45 s. |
| `0x1B3` | `00 00 00 00 00 00` | All zeros in every log. 2 s. |
| `0x1B5` | `FD 0D 17 D1 00 00` | Static `FD 0D 17 D1 00 00` (B0 also seen `FF`). 6 s. |
| `0x3A0` | `00 00` | Seen, meaning unknown. Stayed `00 00` (500 ms) in every capture — never exercised. |
| `0x3F8` | `04 00 00 00 00 00` | Seen, meaning unknown. Rock-steady `04 00 00 00 00 00` at 100 ms (capture D: 99 ms) across all captures — a pure heartbeat. |

---

# Part B — Request-only (KWP2000 service 21 / 1A / 18)

PIDs we can poll but that are **not** broadcast on the bus. All read with `ATCAF0`, so
request bytes carry the ISO-TP PCI byte (`21 3B` is typed `02 21 3B`). Module ids:
CCN `6A0/514`, RADIO `6B0/516`, ORC `6E0/51C`, WCM `600/500`, SCM `6A8/515`, EOM `6F0/51E`.

## B.1 Tyre pressure — the flagship, request-only

The vendor database has **no** per-wheel pressure PID; the cluster only exposes a lamp
state (`CCN 2130` = TPM localization "Non-Localized" = base TPMS). The **WCM knows every
pressure** and answers an undocumented service-21 range.

### `WCM 21 3B` — four road-wheel pressures — CONFIRMED
- **Request:** `02 21 3B` to `600`. **Response:** `61 3B <p1> <p2> <p3> <p4>`.
- **Decode:** each byte = **1 psi per bit, no offset, no scaling.**
- **Observed, live over the drive** (`W3B` lines): started `61 3B 22 1F 1F 1E` = **34, 31,
  31, 30 psi**, settled to `61 3B 21 20 20 1E` = **33, 32, 32, 30 psi** as tyres warmed.
  A separate engine-off sweep read `61 3B 21 20 21 1E` = **33, 32, 33, 30 psi**
  (2.28 / 2.21 / 2.28 / 2.07 bar).
- **Validation (three independent ways):** owner's expected 2.1–2.3 bar range; one
  genuinely soft 30-psi corner; a stored **`C0077` Low Tire Pressure** DTC on the WCM.
- **Corner order is unmapped** — bytes → corners is unknown. Deflate one wheel a few psi
  and watch which byte moves; that is the only way to map it.

### `WCM 21 20`…`21 24` — per-sensor pressure + temperature + status — CONFIRMED (pressure)
- **Response:** `61 2x <psi> <temp?> <status16>`.
- **Observed (`W20`–`W24`):** e.g. `61 20 22 59 01 A3` = sensor 1: **34 psi**, byte `59`
  (≈ 49 °C if `raw−40`, **hypothesis**), status `01A3`. Sensor 5 (`21 24`) reads **~8 psi**
  — the spare, and it is excluded from the `21 3B` road-wheel set.
- Five sensors match `C1501..C1505` "Tire Pressure Sensor 1..5" in the WCM DTC table.
- **Temperature column is a hypothesis**, not confirmed. The trailing 16-bit field
  (`01A3`) repeats across sensors → status/counter, not an id.

## B.2 Other request-only signals (all CONFIRMED unless noted)

| Signal | Request | Decode | Observed |
|---|---|---|---|
| Fuel level | `CCN 2103` | B3 `× 0.5` % | `61 03 …64` → **50 %**. Vendor DB confirms: `2103` has "Fuel Level Sense Voltage" (byte 2, ×0.0196 V) **and** "Fuel Level **Percentage**" (byte 3, **×0.5 %**) — our B3 read. |
| Coolant + throttle | `CCN 2140` | B0 `−40` °C; B1 `× 0.392` % | 74 °C / 0 % |
| Ambient + voltage | `CCN 2134` | B0 `×0.5−40` °C; B1 `×0.1` V | 32.5 °C / 12.6→13.9 V |
| RPM + speed | `CCN 213E` | RPM = bits16-31; speed = bits32-47 `/128` | 619 rpm / 0 km/h |
| Warning lamps | `CCN 213F` | bit24 MIL, bit26 oil, bit16 ABS, bit23 charge | MIL+oil true (engine stopped) |
| TPM lamp + localization | `CCN 2130` | bits56-63 lamp; bits64-71 enum | Off / Non-Localized |
| Fluid-low flags | `CCN 2135` | bit16 washer, bit18 coolant, bit19 brake | all No |
| Parking brake + door | `CCN 2110` | bit21 P-brake; bit27 LF door ajar | False / (door polarity unverified) |
| Compass | `CCN 2152` | B0 = 8-point enum | `02` = East |
| Odometer (mirror) | `WCM 21 14` | bytes 2-4 uint24 `/10` km | `25 93 C5` = same as broadcast `0x014`. Vendor DB `FCMCGW 21A3` "Odometer" 24-bit **×0.1 km** confirms the formula authoritatively. |
| Injection mass | PCM `2103`/`2106` | 16-bit `×0.01` mm³/stroke | Cross-ref for broadcast `0x1A7` (engine injection qty, r=0.84 vs RPM) |
| ECU part numbers | `1A 87` (all six modules) | ASCII P/N | CCN `5172008AD`, RADIO `5064065AE`, etc. |
| DTC lists | `04 18 00 FF 00` (all modules) | ISO-TP multi-frame | 315 codes readable; `C0077` on WCM |

> **`21 3B` scaling note carried from PROBE-LOG:** treat every non-zero *offset* in the
> extracted scaling table as unverified — the voltage `+5.0` offset was an extraction bug
> (gave 17.6 V; plain `×0.1` gives the correct 12.6 V). **Pinpointed in this pass:** the bad
> offset lives in `pid_scaling.csv` **scaling_id 15** (`×0.1, offset +5.0`); the *correct*
> battery-voltage scaling the DB uses elsewhere is **scaling_id 24** (`CCN 2105` "CCN Battery
> Voltage", `×0.1, offset 0`) — so prefer the DB's own `2105` row over `scaling_id 15`.

---

# Is anything from Part B also broadcast in Part A?

**Yes for several signals — but NOT tyre pressure.** Verified by scanning every broadcast
payload in both snapshots **and now across ~365 000 frames of real 2015 bus logs.**

| Part B signal | Broadcast on CAN-B? | Evidence |
|---|---|---|
| **Tyre pressure** (`WCM 21 3B`) | **NO — request-only** | see below (240k-frame scan) |
| Odometer (`WCM 21 14`) | **Yes** — `0x014` | `25 93 C5` mirror; live monotonic increment (2.4/4.5 km) |
| RPM + speed + distance (`CCN 213E`) | **Yes** — `0x002` | 619 rpm + `C4 C6 C4 B3` byte-match; wheel-dist ×4 cm |
| Coolant (`CCN 2140`) | **Yes** — `0x02C` B1 | 74 / 89 °C both match; B0 tracks RPM r=0.95 (load) |
| Ambient + voltage (`CCN 2134`) | **Yes** — `0x015` | byte-identical ambient/voltage |
| Compass (`CCN 2152`) | **Yes** — `0x1B4` B0 | B0 = 8-point 0–7 (all 8 seen); now **CONFIRMED** |
| Fuel (`CCN 2103`) | **Partial** — `0x159` B5 | B5 falls with distance in both real drives; scale differs |
| Warning lamps / fluid flags / TPM lamp / door / P-brake | **No** (not found) | `213F`/`2135`/`2130`/`2110` composites absent from broadcast |

### Tyre pressure is request-only — proof (now with real-log sample counts)

Our real corner pressures are the byte sequence **`21 20 21 1E`** (= 33, 32, 33, 30 psi),
or the warmed drive variants `22 1F 1F 1E` / `21 20 20 1E`. Beyond the original 47-id
snapshot scan, an **exhaustive programmatic scan of all five 2015 recordings (~365 000
frames across all ~52 ids)** — including every one of the **1 661 `0x1B6` frames** (813 in
capture A + 848 in B) and all **813 `0x1B6` samples in `ride.txt`** the owner flagged —
found:

- **Zero** occurrences of any 4-byte pressure sequence (`21 20 21 1E`, `22 1F 1F 1E`,
  `21 20 20 1E`, `21 1F 1F 1E`) in any payload of any id, in any capture.
- **Zero** ids carrying even **three consecutive bytes** that jointly sit in the 30-34 psi
  window (`0x1C`–`0x24`) *and* vary — the single machine hit was a coincidental slice of
  `0x002` (RPM/speed data), not four static corners. **No id has three or more bytes whose
  full observed range stays inside the pressure window.**
- `0x1B6` is frozen at `00 18 FF 18 FF 18 FF` in all 1 661 frames except **B0** (`00`/`01`
  lamp bit) — definitively a TPMS *lamp* flag, never per-wheel pressure.
- `0x1AD`'s `1E 1E` (2015: `42 42`) is a 2-byte artefact, not four corners.

**Conclusion (definitive, ~365 000-frame basis):** per-wheel tyre pressure is **NOT
broadcast** on CAN-B. It exists only behind the WCM `21 3B` poll.

**Practical implication for the ESP32:** tyre pressure — the feature that motivated the
whole project — cannot be obtained by passively sniffing the bus. The ESP32 must
**actively poll** `600 → 02 21 3B` on a timer. This is cheap (one request/response,
~2.6 ms on the bus, well inside a 200 ms `ATST`), and it does not need security access
(the pressure PIDs are ungated, unlike `21 03`–`0B`). Poll it at ~0.5–1 Hz alongside the
other dashboard PIDs.
