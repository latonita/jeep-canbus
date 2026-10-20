# jeep-canbus [![Build Status](https://api.travis-ci.org/latonita/jeep-canbus.svg?branch=master)](https://travis-ci.org/latonita/jeep-canbus) [![Analytics](https://ga-beacon.appspot.com/UA-99380399-3/welcome-page)](https://github.com/igrigorik/ga-beacon) 

Projects around CAN BUS in Chrysler Jeep - VES enabler, other small features

PCB for Aux + bluetooth based on xs3868 v3.0 added.

![PCBv2.0](https://raw.githubusercontent.com/latonita/jeep-canbus/master/jeep-in-radio-board/jeep-in-radio-pcb-2.0.png)

Here is CAN ID map I collected on CAN-B:

# Jeep Grand Cherokee WH 3.0 CRD - CAN-B — complete observed ID/byte map

This is the consolidated byte-level reference for every standard CAN ID that
actually occurs in the captures in this repository. Byte numbering is zero
based. Multi-byte integers are big-endian unless stated otherwise.

Confidence labels:

- **Confirmed** — controlled switch/action test or an unambiguous decode.
- **Probable** — strong behaviour and cross-log evidence; physical identity or
  scale is not completely calibrated.
- **Candidate** — useful working hypothesis that needs an external reference.
- **Unknown** — observed bytes/pattern only; no semantic claim.
- **Static/NM** — constant heartbeat or network-management traffic, not a
  vehicle measurement.

`FF`, a field of all `FF`, or `FFFFFF` frequently means unavailable during
wake-up/key-off. It is not a maximum physical reading.

## Vehicle, controls and measurements

| CAN ID | DLC / period | Byte and bit meaning | Confidence |
|---|---|---|---|
| `000` | 6 / 100 ms | `b0` ignition state: `00` wake/uninitialised, `01` off, `63` ACC, `85` ignition ON/running, `A9` crank. `b1..b5=00`. | **Confirmed** |
| `002` | 8 / 50 ms | `b0:b1` engine RPM, 1 rpm/count. `b2:b3` speed, raw/128 km/h. `b4:b5` wheel/distance pulse counter 1. `b6:b7` counter 2. Counters are rolling unsigned 16-bit values, about 25 counts/m. All `FF` = unavailable/engine off. They are distance pulses, **not wheel angles**. | **Confirmed** |
| `003` | 7 / ~450 ms | `b0` normally `00`; **`bit2`/`+04` = ESP/stability control switched off** (`04 21 80 01 50 FF 00` = Park with ESP off, 2026-10-13 stationary switch test). `b1` unknown (`00/01/05/0D/20/21` observed; constant `21` throughout the 2026-10-13 test). `b2` lever bitfield over a fixed `0x80`: `80=P`, `81=R`, `82=N`, `84=D` — **every AutoStick manual range also reads `84`**, so `b2` alone cannot separate D from a limited range. `b3 bit0` brake switch (`01` for the whole stationary shift test, foot on the brake). `b4` redundant PRNDL and the one byte that resolves the manual ranges — it is the **ASCII code of the character the cluster displays**: `50='P'`, `52='R'`, `4E='N'`, `44='D'`, `34='4'`, `33='3'`, `32='2'`, `31='1'` (AutoStick upper-limit ranges). `b5` normally `FF`. `b6 bit3` 4WD lever pulled; other bits unknown. | Gear/brake/4WD and ESP-off **confirmed**; `b4` ASCII encoding **confirmed**; `b1` unknown |
| `006` | 7 / 100 ms | `b0`: `00` key-off, `18` no indicator, `19` left, `1A` right, `1B` hazard; `38` also observed with an extra unknown bit. `b1..b6` are not decoded; common payloads include `22 04 00 00 02 58`, while the 2026 capture also has different `b1/b3/b4/b5` values. | Indicator byte **confirmed** |
| `013` | 5 / 500 ms | `b0 bit7` data-valid/module-awake; `b0 bit3=1` driver belt unbuckled. `b1=00`. `b2` door-ajar bits: bit0/`01` FL, bit1/`02` FR, bit2/`04` RR, bit3/`08` RL, bit4/`10` hatch; bits add. `b3:b4=49 99`. | **Confirmed** |
| `014` | 7 / 100 ms | `b0:b1:b2` 24-bit odometer, 0.1 km/count; `FFFFFF` unavailable. `b3=FF`. `b4` ambient/outside temperature in °C for the observed positive range: 2026 warm-weather drive `24→25`, older captures mostly `18` (`17–22` total). `b5` and `b6` remain unknown and are **not** validated as fuel level or economy. | Odometer **confirmed**; ambient **probable/high confidence**; `b5..b6` unknown |
| `015` | 6 / 100 ms | `b0` unknown temperature-like channel. `b1` battery/charging voltage in 0.1 V/count: `125` = 12.5 V key-on, `103` = 10.3 V cranking, `143–146` = 14.3–14.6 V running, and `126` = 12.6 V immediately after key-off. `b2` fast air-temperature channel, probably intake/charge air; its observed positive values read plausibly as raw °C, but identity and negative encoding are unconfirmed. `b3=FF`, `b4:b5=00`. | Voltage **confirmed/high confidence**; `b2` candidate; `b0` unknown |
| `01B` | 8 / ~100 ms | `b0` multiplex index `00/01/02`; `b1..b7` are seven ASCII VIN characters for that index. Concatenate the three indexed chunks and discard padding to obtain `1J8HCE8M45Y588369`. Index `03` with zeros appears during boot. | **Confirmed** |
| `02C` | 6 / ~500 ms in 2026 (~600 ms older captures) | `b0` engine-related analog strongly correlated with RPM (`r≈0.95`), possibly oil pressure or load. `b1` coolant temperature, `raw - 40` °C: the 2026 cold drive rises smoothly `68→132` (28→92 °C) in about 10 minutes, then holds `127–131` (87–91 °C); an independent 2015 cold start rises `65–70` to `126–128`. `b2` changing/load-related unknown analog. `b3` normally `64` (occasionally `63`); `b4:b5=00`. All `FF` means unavailable. | Coolant **high confidence**; scale **probable/high confidence**; `b0` candidate; `b2` unknown |
| `0D0` | 2 / ~1.15 s | `b0 bit7`/`+80` **parking brake engaged** — asserted for the first time in the 2026-10-13 stationary test (`04 00` released → `84 00` lever pulled). `b0 bit2`/`+04` was set throughout that session as the released-state baseline, while every 2015 capture reads a flat `00 00`; its meaning is unknown and it is not ESP-off (ESP was on for the `04 00` baseline). `b1=00` always. | Parking brake **confirmed**; `b0 bit2` unknown |
| `11D` | 3 / 100 ms | `b0` switch/button bitfield (`08/10/40` observed); exact button assignments unknown. `b1` small state `0..3`. `b2=00`. | **Unknown switch group** |
| `159` | 6 in every log / ~1 s (2026 drive n=1450: mean 1.0039 s, median 1.000 s, p5 0.998 s, p95 1.001 s; the sole 3.0 s gap is a pre-settling artefact between the first two frames) | `b0=FE` during normal running (100% of the 2026 drive log); power-up/power-down states also carry `00`, `04`, `05`, `0B` or `FF`. `b1:b3=FF FF FF` and DLC 6 are constant in every log in the repo. `b4` is a **validity flag**: `b4=00` ↔ `b5` is a real reading, `b4=01` ↔ `b5=0xFF` (unavailable), true with zero exceptions across every log — confirmed by two live transitions, `playback.log`'s ignition-off (`b4` flips to `01` and the frame settles permanently to `00 FF FF FF 01 FF`) and `radio-ignition-on`'s key-on (`05 FF FF FF 01 FF` → `05 FF FF FF 00 4D` the instant the key reaches ON). `b5` fuel level, probably **litres at 0.5 L/count**: the largest valid raw ever recorded is 154 (77.0 L) against this truck's 77.6 L/20.5 US gal tank — a percentage scale has no reason to stop at 154/255, a litre scale stops exactly where the tank does (`b5×0.5` and `b5/2.0` are the same transform; range alone cannot separate litres from percent, only this ceiling does). 2026 drive raw 138–142 = 69–71 L, logged the morning after a fill and matching the owner's "almost full" (the old `/2.55` percent conversion read the same fill as only 54%); hill trip 140→142 on the slope→138 at home = 70→71→69 L. Cross-check on older captures: raw 25–30 = 12.5–15.0 L (low fuel), raw 77 = 38.5 L (about half), raw 152–154 = 76–77 L (essentially full). Slosh is already heavily damped at the sender: every `b5` change in the 2026 drive is exactly ±1 count (30 changes, largest single step 0.5 L, full-trip peak-to-trough only 4 counts/2.0 L); a consumer could debounce changes shorter than ~10 s to remove the residual flip. Absolute calibration remains open — no brim-to-brim fill has been run against it. | Byte layout (`b4` validity flag, `b0` power-state variability) **confirmed** across every log in the repo; litre scale **probable** (capacity ceiling + owner cross-check), uncalibrated |
| `1A7` | 6 / 100 ms running, ~400 ms key-off | `Q=b0:b1` instantaneous fuel/injection flow; `FFFF` unavailable and zero is valid overrun cut. `b2..b5=00`. Integrate `Q×dt`; do not sum frames or multiply by RPM. Provisional scale: `flow_L/h=Q/160`, so `litres += Q×dt/(160×3600)`. | Rate identity **probable**; divisor 160 candidate |
| `1AC` | 1 / ~950 ms | `b0` dash/dome dimmer-wheel position: `01` top lock (dome on, instruments maximum), `02..08` descending brightness with dome off, `09` bottom lock (dome forced off, minimum). | **Confirmed** |
| `1B2` | 6 / ~1.45 s | `b0 bit7` 4WD lever pulled, causing blinking 4WD telltale/chime. `b1` observed `00/02/07`, probably transfer-case range/status but undecoded. `b2..b5=00`. | Lever bit **confirmed**; `b1` candidate |
| `1B4` | 6 (2953/2953) / 500 ms (mean 502.8 ms, median 500 ms, σ78 ms; 99.83% of gaps 480–520 ms) | `b0` 8-way direction enum: `0=N`, `1=NE`, `2=E`, `3=SE`, `4=S`, `5=SW`, `6=W`, `7=NW`, computed as `round(b1×2/45) mod 8` — sector centres sit at 0/45/90/.../315°, boundaries at 22.5/67.5/.../337.5°. A full 8-phase offset sweep confirms offset 0 with `round` (99.46% agreement, 2937/2953 frames) and rejects `floor` outright (best `floor` phase only 58.35%); independently cross-checked at 99.75% on a 1584-frame 2019 session. The 0.54% of disagreements are all boundary-quantization artifacts (e.g. `b1=146` → 292°, 0.5° shy of the 292.5° boundary, within the ±1° the 2°/count scale allows). `b1` magnetic heading in 2°/count (`0..179` → `0..358°`; observed range this session `0..178`, never reaching the documented 179 ceiling). `b2=07` constant in 2953/2953 frames, likely compass variance-zone/status — only its constancy is established, its meaning is not. `b3..b5=00` constant in 2953/2953 frames. A circular derivative of `b1×2` is only a coarse yaw-rate proxy. | Direction/heading **confirmed** against two independent sessions (2026 n=2953, 2019 n=1584); `b2` constancy **confirmed**, meaning still candidate |
| `1C8` | 1 / 500 ms | `b0` low bits: `00` exterior lights off, `01` position lamps, `02` low beam. Add bit4/`10` for front fog and bit5/`20` for rear fog: e.g. `12` low+front, `32` low+front+rear. | **Confirmed** |
| `210` | 2 / 500 ms | `b0`: `03` lights off, `02` any lights on; `00/01` occur as wake/key-off transients. `b1` instrument-panel dimmer: `00` with lights off; observed/hand-tested ladder `22/44/66/A4/C8`. | **Confirmed**, dimmer ladder one-note caveat |
| `2C0` | 8 / 100 ms | `b0:b1=FF FF`. `b2` accelerator pedal (23 rest, up to 234 floored in old capture). `b3≈b2/2`. `b4` unknown analog, about `7E` at rest. `b5=FF`. `b6` exact mirror of `b2`. `b7` status/validity flags (`02/0E/82/8E/...`). | Pedal **probable/high confidence**; flags unknown |
| `370` | 6 / event-driven | `b0` pulses `01` then `00`, one event per windscreen-wiper sweep. `b1..b5=00`. | **Confirmed** |

## RAR radio/head-unit IDs

The installed head unit is the **RAR radio**. The radio-unplug experiment
confirms that `09F`, `1AB`, `1BB`, `394`, `3D0` and NM node `416` belong to it.

| CAN ID | DLC / period | Byte and bit meaning | Confidence |
|---|---|---|---|
| `09F` | 8 / ~1 s | RAR radio source/current-mode state in `b0..b2`; exact value map incomplete. `b3=00`, `b4:b6=FF FF FF`, `b7=1F`. Closely mirrored by `394`. | Source **confirmed**, value map incomplete |
| `1AB` | 6 / ~3 s | Payload `03 00 00 00 00 00` in observed sessions. Head-unit heartbeat/status; individual semantics unknown. | Head unit **confirmed**; bytes unknown/static |
| `1BB` | 6 / ~1 s | `b0..b5=00` in all captures. Head-unit heartbeat/status. | Head unit **confirmed**; static |
| `394` | 6 / ~1 s | `b0..b2` mirror radio mode/status from `09F`. `b3` button bits: bit0 seek right, bit1 seek left, bit2 fast-forward, bit3 rewind, bit5 random/PTY. b4? `b5=00`. The head unit sends this frame **on press and again on release**, carrying the new state each time (owner, checked against the real unit) — so the ~1 s cadence is the idle heartbeat, not the report rate, and a short tap cannot fall between two frames. **No capture in this repository can confirm any of the bit or byte assignments: `b3`, `b4` and `b5` are all `0x00` in every one of the 5510 `394` frames logged, because nobody has ever pressed one of these buttons while recording.** The row said `b4` until the owner corrected it, and the logs were silent either way. | Byte/bit assignment **owner-stated, unconfirmed by any capture**; press/release behaviour **confirmed** |
| `3D0` | 7 / ~1 s | `b0` volume. `b1` bass, `b2` mid, `b3` treble, `b4` balance, `b5` fade; centre values are commonly around `0A` (fade observed `0C`). `b6=FF`. | **Confirmed/high confidence** |
| `3DD` | 8 / injected command | VES-present announcement to the RAR radio: `b0=03`, `b1..b7=00`. Transmitting `03 00 00 00 00 00 00 00` makes the RAR recognise VES as present and enables its AUX input. This was actively tested, not passively recorded. | **Confirmed by injection test** |

## Static, heartbeat and still-unknown cyclic IDs

These frames are retained because a complete map must show them, but a constant
payload is not proof of a physical meaning.

| CAN ID | DLC / period | Observed bytes | Confidence |
|---|---|---|---|
| `008` | 6 / ~1 s | `00 00 80 80 00 00`. | **Static/unknown** |
| `010` | 8 / ~2 s | Usually `41 01 15 71 63 80 2A 03`; `b0 bit1` participates in a shared alive/toggle burst (`41↔43`). | **Static/unknown** |
| `011` | 8 / ~2 s | Usually `3D 13 00 03 8B 40 00 00`; `b0 bit1` toggles (`3D↔3F`) with several related IDs. | **Static/unknown** |
| `012` | 6 / ~1 s | `b0=00`; `b1=00/10`; `b2=00/02/06`; `b3=40`; `b4:b5=0000/4999`. Possibly security/SKREEM-related, not established. | **Unknown** |
| `02E` | 2 / ~1 s | `1F 00`. | **Static/unknown** |
| `0EC` | 1 / ~1.5 s | `b0=00/01` toggle. | **Unknown** |
| `110` | 8 / ~3 s | Usually `95 7E 04 83 83 00 55 00`; `b0 bit1` toggles (`95↔97`) in the shared burst. | **Static/unknown** |
| `1AD` | 6 / ~2.7 s | Usually `01 42 42 04 01 00`; `b0 bit1` toggles (`01↔03`) in the shared burst. | **Static/unknown** |
| `1AE` | 6 / ~1.4 s | `00 00 00 00 00 00`. | **Static/unknown** |
| `1AF` | 8 / ~2.5 s | Usually `01 E7 00 40 01 A4 08 FF`; `b0 bit1` can toggle (`01↔03`) in the shared burst. | **Static/unknown** |
| `1B0` | 8 / ~5.4 s | `0D DE 03 E8 0A A0 08 DE`. | **Static/unknown** |
| `1B3` | 6 / ~2 s | `00 00 00 00 00 00`. | **Static/unknown** |
| `1B5` | 6 / ~6 s | `FD 0D 17 D1 00 00`. | **Static/unknown** |
| `1B6` | 7 / ~1 s | `00 18 FF 18 FF 18 FF`. | **Static/unknown** |
| `1F8` | 1 / ~2 s | `00`. | **Static/unknown** |
| `3A0` | 2 / 500 ms | `00 00`; heartbeat. | **Static** |
| `3F8` | 6 / 100 ms | `04 00 00 00 00 00`; fast heartbeat/reference frame. | **Static** |

The simultaneous `b0 bit1` changes in `010`, `011`, `110`, `1AD` and `1AF`
strongly suggest a shared transmitter/alive state. They do not establish what
the remaining payload fields mean.

## Rare/event-driven IDs with unknown function

| CAN ID | DLC | Observed bytes/behaviour | Confidence |
|---|---|---|---|
| `1E0` | 6 | Short burst with `b0=01`, ending in `b0=00`; `b1..b5=00`. | **Unknown event** |
| `1E2` | 6 | Rare event; `b0..b2` vary by session (`E4 0D D9`, `05 B7 6B`, `3D B4 65` observed); `b3..b5=00`. | **Unknown event** |
| `3F4` | 6 | Rare burst; `b0..b3` vary by session (`A5 1D E9 37`, `12 60 8F 55`, `28 8A A0 63` observed); `b4:b5=00`. | **Unknown event** |

## OSEK network-management ring

All seven frames have DLC 8 and recur about every 690 ms per node. They are
network management, not vehicle sensor data.

Common layout:

- `b0=FE`: node alive/announcing; `b1` is its own node address.
- `b0=FD`: ring/token data; `b1` is the next node address.
- `b2=3F` normally; bit7 gives `BF`, a sleep indication/request.
- `b3..b7=FF`.

| CAN ID | Node | Normal `FD` successor | Additional observation |
|---|---:|---:|---|
| `400` | `00` | `02` | NM node; module identity unknown. |
| `402` | `02` | `14` | NM node; module identity unknown. |
| `414` | `14` | `15` | NM node; module identity unknown. |
| `415` | `15` | `16` | NM node; module identity unknown. |
| `416` | `16` | `1C` | **RAR head unit/radio**, proven by unplug test. |
| `41C` | `1C` | `1E` | Joins the ring only while ignition is ON. |
| `41E` | `1E` | `00` | Often carries the sleep-indication bit in `b2`. |

The logical ring is `400 → 402 → 414 → 415 → 416 → 41C → 41E → 400`.

## Injected but not passively observed

`3DD` is a **confirmed VES-presence command** with payload
`03 00 00 00 00 00 00 00`: the RAR radio enables AUX when this frame is
transmitted. It never occurs naturally in the repository captures, so it
remains excluded from the 52 passively observed IDs.

## Important absences

- No direct steering-wheel or road-wheel angle has been identified.
- No accelerometer, pitch, roll or high-rate gyro/yaw signal has been identified.
- `1B4 b1` is compass heading and permits only a coarse derived yaw rate.
- `002 b4:b7` are wheel-pulse counters, not wheel angles.
- `1B4` carries no unavailable sentinel and does not stop at key-off: `b1=0xFF`
  is never observed across ~10 log files, and in `playback.log` the frame
  keeps transmitting at its normal cadence with an unchanged, valid heading
  (`06 85 07 00 00 00`, W/266°) straight through the `000` ignition-off
  transition and for ~1000 further lines until the capture ends. Anything
  consuming this must gate key-off on `000` or on bus silence, never on the
  compass frame itself (whether `1B4` eventually goes quiet on true bus sleep
  is unknown — the capture ends mid-stream).
- `159` is the mirror-image case: it does carry a real unavailable sentinel, in `b4`, and consumers must gate on it — `b4=01` means `b5=0xFF` is not a reading (let alone 100% full), true with no exceptions across every log in the repo.
- `1B4` heading is bit-for-bit static when stopped: across all 196
  consecutive stationary (≤2 km/h) sample pairs the 500 ms heading delta was
  exactly 0.0° (mean, median, max and stdev all zero) — no jitter to filter.
  Moving, the median 500 ms delta is 0° and mean |delta| is 1.9°; sustained
  turns run ~18–26°/s, with a genuine peak of 96° in one 500 ms sample
  (192°/s) at a hairpin apex on the hill switchbacks.


