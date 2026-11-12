# Jeep Grand Cherokee WH 3.0 CRD - CAN-B — complete observed ID/byte map

Byte-level reference for every standard CAN ID observed on CAN-B (83.3 kbit/s
interior bus). Byte numbering is zero based. Multi-byte integers are
big-endian unless stated otherwise.

Confidence labels:

- **Confirmed** — controlled switch/action test or an unambiguous decode.
- **Probable** — strong behaviour and cross-log evidence; physical identity or
  scale is not completely calibrated.
- **Candidate** — working hypothesis that needs an external reference.
- **Unknown** — observed bytes/pattern only; no semantic claim.
- **Static/NM** — constant heartbeat or network-management traffic, not a
  vehicle measurement.

`FF`, a field of all `FF`, or `FFFFFF` usually means unavailable during
wake-up/key-off. It is not a maximum physical reading.

The same map is available as a DBC file: [jeep-wk-wh-crd.dbc](jeep-wk-wh-crd.dbc).

## Vehicle, controls and measurements

One table per ID: one row per byte (or per multi-byte field), one row per bit
for bitfields. `b0` is the first data byte; multi-byte fields are big-endian
unless stated. Evidence is in the notes under each table.

### `000` — ignition / key state
DLC 6 · 100 ms

| Byte | Bit | Meaning | Confidence |
|---|---|---|---|
| `b0` | — | ignition state: `00` no key, `01` key in at OFF, `63` ACC, `85` ON/running, `A9` crank | **Confirmed** |
| `b0` | 0 | key present (`00` no key vs `01` key in) | **Probable** |
| `b1..b5` | — | always `00` | Static |

- Sequence: `00` locked/no key → `01` key inserted → `63` ACC → `85` ON → `A9` crank; at the end `63 → 01` at OFF and `00` when the key is pulled. Matches the BiggRanger radio-interface encoding.
- Key-bit caveat: `01 → 00` can follow `63 → 01` within ~0.2–0.3 s, and `85 → 00` can occur directly.

### `002` — RPM, speed, distance pulses
DLC 8 · 50 ms

| Byte | Bit | Meaning | Confidence |
|---|---|---|---|
| `b0:b1` | — | engine RPM, 1 rpm/count | **Confirmed** |
| `b2:b3` | — | vehicle speed, raw/128 km/h | **Confirmed** |
| `b4:b5` | — | **left-wheel distance counter**, rolling u16, ~24.9 counts/m | **Confirmed** (= ABS `2151` / CCN `213E` left) |
| `b6:b7` | — | **right-wheel distance counter**, rolling u16, ~24.9 counts/m | **Confirmed** (= ABS `2151` / CCN `213E` right) |

- All `FF` = unavailable / engine off. The counters are distance pulses, **not wheel angles**.
- The raw words in ABS KWP `21 51` bytes 4–5 / 6–7 (*Distance Traveled by Left / Right Wheel*; the tool shows them ×4) equal `b4:b5` / `b6:b7` value for value, and CCN `21 3E` carries the same pair. Over 8.7 km: left 216,332 and right 216,820 counts = 24.9 counts/m against both the odometer and integrated speed.
- Left/right differ in turns: the outer wheel's counter runs ahead (~10 % in a tight circle). Rate of each counter = that side's wheel speed: `km/h = Δcounts/Δt / 24.9 × 3.6`. Which axle the ABS uses for these two is not stated; the four separate wheel speeds of ABS `2101` are not broadcast.

### `003` — cluster telltales + gear selector
DLC 7 · ~450 ms

| Byte | Bit | Meaning | Confidence |
|---|---|---|---|
| `b0` | 0 | ABS lamp request | **Confirmed** (CCN `213F` mirror) |
| `b0` | 1 | ESP lamp request | **Confirmed** (CCN `213F` mirror) |
| `b0` | 2 | **ESP lamp steady = ESP disabled** (ESP-off button, or automatically in 4WD Low) | **Confirmed** (button test + 4LO) |
| `b0` | 3 | ESP lamp flash | **Probable** |
| `b0` | 4 | parking-brake lamp request | **Probable** (CCN bit map) |
| `b0` | 7 | charging-system failure lamp | **Probable** (CCN bit map; set at engine stop) |
| `b1` | 0 | MIL (check-engine) lamp | **Confirmed** (PCM agrees MIL on) |
| `b1` | 1 | MIL blink | **Probable** (CCN bit map) |
| `b1` | 2 | oil-pressure lamp (`05` before start / after stop, `01` running) | **Confirmed** |
| `b1` | 3 | wait-to-start (glow-plug) lamp | **Probable** (CCN bit map) |
| `b1` | 4 | water-in-fuel lamp | **Probable** (CCN bit map) |
| `b1` | 5 | A/C compressor running (same as `0EC b0`) | **Confirmed** (vs PCM `210D`) |
| `b1` | 6 | cruise engaged (speed set) | **Confirmed** (cruise test) |
| `b1` | 7 | cruise on (armed) | **Confirmed** (cruise test) |
| `b2` | — | selector lever over a fixed `0x80`: `80` P, `81` R, `82` N, `84` D (all AutoStick ranges also `84`); `C0` unavailable | **Confirmed** |
| `b3` | 0 | brake switch | **Confirmed** (17/17 vs PCM brake switches) |
| `b4` | — | PRNDL ASCII shown on the cluster: `50` P, `52` R, `4E` N, `44` D, `34/33/32/31` AutoStick 4/3/2/1 | **Confirmed** |
| `b5` | — | normally `FF` | Static |
| `b6` | 2 | **4WD Low engaged** (4LO telltale; CCN transfer-case lamp `01`) | **Confirmed** (4LO test) |
| `b6` | 3 | transfer-case shift in progress / 4WD lever pulled (telltale flashing) | **Confirmed** |

- `b0`, `b1`, `b4` are byte-for-byte the CCN KWP `21 3F` response bytes 2, 3, 7; `b6` bits 2–3 carry its byte 8 (transfer-case lamp, shifted).
- The ESP-off button changes **only** `b0` bit2 — no other CAN-B frame. 4LO sets the same bit; use `1B2 b1=01` or `b6` bit2 to tell 4LO apart. The button itself is not broadcast.
- Cruise: on → `b1` `81`, set → `C1`, off → `41` → `01`; CCN `213F` *Cruise On* / *Cruise Engaged* agree. The set speed is not broadcast.
- Key-off frame `93 05 C0 C0 50 FF 0C` = engine stopped (ABS + ESP + park + charge lamps, oil + MIL, gear/brake unavailable, transfer case unavailable).

### `006` — lights, indicators, RAP, CCN settings
DLC 7 · 100 ms

| Byte | Bit | Meaning | Confidence |
|---|---|---|---|
| `b0` | 0 | left turn indicator (`19`) | **Confirmed** |
| `b0` | 1 | right turn indicator (`1A`); both bits = hazard (`1B`) | **Confirmed** |
| `b0` | 3 | parking/position lamps on (follows `1C8` `01`) | **Confirmed** |
| `b0` | 4 | low beam on (follows `1C8` `02`) | **Confirmed** |
| `b0` | 5 | high-beam indicator (`38` = low + high beam) | **Confirmed** (high-beam test) |
| `b0` | 6 | front fog lamps on (`58`) | **Confirmed** (fog test, CCN `2133`) |
| `b0` | 7 | rear fog lamp on (`D8`) | **Confirmed** (fog test, CCN `2133`) |
| `b0` | — | `03` for ~0.2 s = lock-confirm flash; `00` = lights off | **Confirmed** |
| `b1` | 1 | **RAP / accessory power** | **Confirmed** |
| `b1` | 5 | set at ACC before bit1, survives crank, clears first at the timeout | **Unknown** |
| `b2` | — | `04`; probably a CCN settings-flag byte | Candidate |
| `b3` | — | illuminated-approach time, s (`3C` = 60) | **Confirmed** (CCN `2133`) |
| `b4` | — | headlamp off-delay time, s (`3C` = 60) | **Confirmed** (CCN `2133`) |
| `b5:b6` | — | power-accessory (RAP) delay, s (`02 58` = 600) | **Confirmed** (CCN `2133` + measured timeout) |

- `b0` common values: `00` lights off (hence key-off), `08` parking lamps, `18` low beam (normal driving).
- `b1` RAP: set at ACC (`00 → 20 → 22`), cleared during crank, held after key-off, cleared by a door opening with the key out or exactly 600.0 s after OFF. A door opened with the key **in** at ACC leaves it at `22`. Details in *Power modes and RAP*.
- `b2..b6` = CCN KWP `21 33` bytes 3–7. `04 00 00 02 58` = approach/headlamp delay set to 0. CCN `2133` byte 3 reads `02` (flash lamps with lock) while `b2` is `04`, so the `b2` bit mapping is not established.

### `012` — WCM key / RKE / chime status ("Bussed Output 1")
DLC 6 · ~1 s

| Byte | Bit | Meaning | Confidence |
|---|---|---|---|
| `b0` | — | RKE basic function request (`01` around a fob unlock) | **Probable** |
| `b1` | 4–7 | SKIM key number in ignition (`0` key out) | **Confirmed** (live WCM read) |
| `b1` | 0–3 | probably RKE complex request / fob number (`3` in the unlock frame) | Candidate |
| `b2` | 1 | valid SKIM key (`02` key in, `00` key out) | **Confirmed** |
| `b2` | 2 | SKREEM chime request (`06` for 1–3 frames after key insertion) | **Probable** (WCM bit map) |
| `b2` | 3 | SKREEM seed request to engine | Candidate (WCM bit map) |
| `b2` | 4 | TPM horn/chirp request | Candidate (WCM bit map) |
| `b2` | 5 | TPM chime request — injecting it does **not** chime | Candidate (WCM bit map) |
| `b3` | — | `40` | Static |
| `b4:b5` | — | `49 99` only during the key-authorisation burst, else `00 00` | Unknown |

- Matches WCM KWP `21 18` (bytes 2–6) and CCN `21 30` read live with ignition ON; becomes `00 00 00 40 00 00` the moment the key is pulled. Fob-unlock frame: `01 03 00 40 49 99`.
- Injecting `00 30 22 40 00 00` (ignition ON, engine off, 100 ms and 50 ms rates) produces no chime, no telltale and no other bus change. The cluster probably ignores spoofed `012` while the real WCM keeps sending, or TPM is not fitted. The SKREEM bit is untested.

### `013` — doors, seatbelt
DLC 5 · 500 ms

| Byte | Bit | Meaning | Confidence |
|---|---|---|---|
| `b0` | 3 | driver seatbelt unbuckled | **Confirmed** |
| `b0` | 7 | data valid / module awake | **Confirmed** |
| `b1` | — | always `00` | Static |
| `b2` | 0 | front-left door ajar | **Confirmed** |
| `b2` | 1 | front-right door ajar | **Confirmed** |
| `b2` | 2 | rear-right door ajar | **Confirmed** |
| `b2` | 3 | rear-left door ajar | **Confirmed** |
| `b2` | 4 | hatch ajar | **Confirmed** |
| `b3:b4` | — | `49 99` | Static |

- Seatbelt unfastened while driving: `b0` `80 → 88 → 80`; the cluster chimes and lights the telltale and **no other CAN-B frame changes**. Door-open-with-key-in and lights-on chimes likewise produce no frame. These chimes are generated inside the cluster; there is no chime request to copy.

### `014` — odometer, outside temperature, fuel sender
DLC 7 · 100 ms

| Byte | Bit | Meaning | Confidence |
|---|---|---|---|
| `b0:b1:b2` | — | odometer, 24-bit, 0.1 km/count; `FFFFFF` unavailable | **Confirmed** |
| `b3` | — | `FF` | Static |
| `b4:b5` | — | **filtered outside temperature (dash value)**, `raw×0.01 − 40` °C; `FFFF` unavailable | **Probable / high confidence** |
| `b6` | — | **raw fuel-sender voltage**, 5/255 V/count; `FF` unless RUN | **Confirmed** (= CCN `2103`) |

- `b4:b5`: `17 3E` = 19.50 °C with 19 °C on the dash; clean carry `18 FE → 19 01`. Tracks `015 b0` within ~0.1 °C while moving; rise is blocked when slow or stopped.
- `b6`: byte-identical to CCN KWP `2103` *Fuel Level Sense Voltage* (`57` = 1.71 V at ~70 L). Frozen with the engine off; changes every ~0.2 s while running (slosh, ±1) and rises on climbs (tilt). Monotonic but nonlinear against `159 b5` (14 L→35, 38.5 L→67, 66.5 L→87, 70 L→89, 72 L→90, 76.5 L→92) — the sender curve. See *Temperatures and gauge fuel level*.

### `015` — raw outside temperature, battery voltage
DLC 6 · 100 ms

| Byte | Bit | Meaning | Confidence |
|---|---|---|---|
| `b0` | — | **raw outside-temperature sensor**, `raw×0.5 − 40` °C | **Probable** (CCN `2134` agrees) |
| `b1` | — | **battery/charging voltage**, 0.1 V/count | **Confirmed** (CCN `2134`, r=0.999) |
| `b2` | — | temperature-like value | **Unknown** |
| `b3` | — | `FF` | Static |
| `b4:b5` | — | `00 00` | Static |

- `b0`: equals cold-soaked coolant (113 = 16.5 °C vs 17 °C) and heat-soaks to 25–27.5 °C when stopped with a hot engine. `014 b4:b5` is its filtered copy.
- `b1`: `125` = 12.5 V key-on, `103` cranking, `143–146` running, `126` right after key-off. Not the low byte of `b0`.
- `b2`: not intake or charge air (no OBD match among intake, charge-air, MAF, fuel, oil, transmission or coolant temps; ≈0 correlation with `1A7`). Settles 5–11 below outside temp, reaches 37–45 on a long slow climb, `0B/0C` in cool weather.

### `01B` — VIN
DLC 8 · ~100 ms

| Byte | Bit | Meaning | Confidence |
|---|---|---|---|
| `b0` | — | multiplex index `00/01/02` (`03` with zeros during boot) | **Confirmed** |
| `b1..b7` | — | seven ASCII VIN characters for that index | **Confirmed** |

- Concatenate the three chunks: `1J8HCE8M45Y588369`.

### `02C` — engine: oil pressure, coolant, pedal
DLC 6 · ~500 ms

| Byte | Bit | Meaning | Confidence |
|---|---|---|---|
| `b0` | — | **engine oil pressure**, gauge, ≈4 kPa (0.04 bar)/count; `00` engine off | **Confirmed**; scale probable |
| `b1` | — | **coolant temperature**, `raw − 40` °C | **Confirmed** (exact vs CCN `2140`) |
| `b2` | — | **accelerator pedal**, 0.4 %/count (`250` = 100 %) | **Confirmed** (r=0.9995 vs TCM) |
| `b3` | — | `64` (`65` also common) | Unknown/static |
| `b4` | — | `00` (`03` when unavailable) | Unknown/static |
| `b5` | — | `00` | Static |

- `b0`: PCM `21AA` oil pressure (hPa abs) = `39.9×b0 + 1031`, r=0.996 over `0..148` (0–5.9 bar).
- All `FF` = unavailable.

### `0D0` — parking brake
DLC 2 · ~1.15 s

| Byte | Bit | Meaning | Confidence |
|---|---|---|---|
| `b0` | 2 | ignition RUN (set key-on→RUN, cleared at `85 → 63`) | **Probable** |
| `b0` | 7 | parking brake engaged | **Confirmed** |
| `b1` | — | always `00` | Static |

- `84 00` = RUN with the brake set, `04 00` = RUN with the brake released, `80 00` = key-off with the brake set, `00 00` = not RUN, brake released.

### `0EC` — A/C compressor
DLC 1 · ~1.5 s (and on change)

| Byte | Bit | Meaning | Confidence |
|---|---|---|---|
| `b0` | 0 | A/C compressor clutch engaged | **Confirmed** (4/4 cycles vs PCM `210D`) |

- Each `01` period matches PCM `210D` *A/C Compressor Output* On/Off, with A/C pressure (PCM `2146`) rising 6 → 13 bar. `003 b1` bit5 mirrors it. CCN `2148` *A/C Clutch Request* stays False (the PCM drives the clutch).

### `11D` — column stalks
DLC 3 · 100 ms

| Byte | Bit | Meaning | Confidence |
|---|---|---|---|
| `b0` | 3 | left turn stalk (`08`) | **Confirmed** |
| `b0` | 4 | right turn stalk (`10`) | **Confirmed** |
| `b0` | 5 | high beams on, latched (`20`) | **Confirmed** (high-beam test) |
| `b0` | 6 | high-beam stalk push, momentary (`40`) | **Confirmed** (high-beam test) |
| `b0` | 7 | `80` once for 0.2 s at idle | **Unknown** |
| `b1` | — | small state `0..3` | Unknown |
| `b2` | — | `00` | Static |

- High beam on = `00 → 40 → 60 → 20`, off = `20 → 60 → 40 → 00`, matching `006 b0` bit5. Turn bits are held while the stalk is out, with `006 b0` `19`/`1A`.

### `159` — fuel level, minutes since RUN
DLC 6 · ~1 s

| Byte | Bit | Meaning | Confidence |
|---|---|---|---|
| `b0` | — | **minutes since ignition left RUN** (`00..FE`; `FF` transient on re-init) | **Confirmed** |
| `b1..b3` | — | `FF FF FF` | Static |
| `b4` | — | validity flag: `00` = `b5` valid, `01` = `b5` unavailable | **Confirmed** |
| `b5` | — | **fuel level**, filtered, ≈0.5 L/count | **Probable**, uncalibrated |

- `b0`: resets ~0.7 s after `000` leaves `85` (not at OFF), ticks `+1` every ~60 s (first tick can come early), counts across bus sleep, frozen while ON/crank, sits at `FE` after long parks. `FF` ~0.8 s on bus wake, key insert, door-open wake.
- `b4`: holds without exception; key-off → `00 FF FF FF 01 FF`, key-on → `05 FF FF FF 01 FF` → `05 FF FF FF 00 4D` at ON.
- `b5` scale: largest raw value 154 (77.0 L) vs the 77.6 L tank; 25–30 = low fuel, 77 = half, 152–154 = full. Every change is ±1 count (sender-damped). **PCM `2111` *Fuel Level* (L) tracks `b5` step for step but reads ~4 L above `b5×0.5`** (130→68 L, 131→69, 132/133→70, 135→72; ratio 0.528), verified only over 129–135. CCN `2103` *Fuel Level Percentage* is the unfiltered sender value, not this.
- Period: median 1.000 s (n=1450).

### `1A7` — fuel flow
DLC 6 · 100 ms running, ~400 ms key-off

| Byte | Bit | Meaning | Confidence |
|---|---|---|---|
| `b0:b1` | — | **instantaneous fuel flow Q, `Q/128` L/h**; `FFFF` unavailable, `0` = overrun cut | **Confirmed** (vs ECU fuel rate) |
| `b2..b5` | — | `00` | Static |

- Integrate: `litres += Q×dt/(128×3600)`. Do not sum frames or multiply by RPM.
- PCM `212B` *Fuel Consumption* gives Q/(L/h) = 127–128.5 from idle (1.0 L/h, Q≈129) to 30 L/h, r=0.999; injection volume × RPM gives 127–128.7 in every RPM band.

### `1AC` — dash/dome dimmer wheel
DLC 1 · ~950 ms

| Byte | Bit | Meaning | Confidence |
|---|---|---|---|
| `b0` | — | `01` top lock (dome on, max), `02..08` descending brightness, `09` bottom lock (dome off, min) | **Confirmed** |

### `1B2` — 4WD / transfer case
DLC 6 · ~1.45 s

| Byte | Bit | Meaning | Confidence |
|---|---|---|---|
| `b0` | 7 | 4WD lever pulled (telltale blinks + chime) | **Confirmed** |
| `b1` | — | transfer-case range: `02` 4WD High, `01` **4WD Low**, `00` in transit (~1.6 s per shift), `07` unavailable (key-off) | **Confirmed** (4LO test) |
| `b2..b5` | — | `00` | Static |

### `1B4` — compass
DLC 6 · 500 ms

| Byte | Bit | Meaning | Confidence |
|---|---|---|---|
| `b0` | — | 8-way direction: `0` N, `1` NE, `2` E, `3` SE, `4` S, `5` SW, `6` W, `7` NW | **Confirmed** |
| `b1` | — | magnetic heading, 2°/count (`0..179` → `0..358°`) | **Confirmed** (= CCN `2152` raw ÷ 2) |
| `b2` | — | `07` constant; probably compass variance zone | Candidate |
| `b3..b5` | — | `00` | Static |

- `b0 = round(b1×2/45) mod 8` (99.46 % of 2953 frames, 99.75 % of 1584 frames in a second session; misses are boundary quantisation). `b1` matches CCN `2152` *Compass Direction (Raw)* 42/42 with zero residual. A derivative of `b1` is only a coarse yaw proxy.

### `1C8` — exterior lights
DLC 1 · 500 ms

| Byte | Bit | Meaning | Confidence |
|---|---|---|---|
| `b0` | 0–1 | `00` off, `01` parking/position lamps, `02` low beam | **Confirmed** |
| `b0` | 4 | front fog lamps | **Confirmed** |
| `b0` | 5 | rear fog lamp | **Confirmed** |

- e.g. `12` = low + front fog, `32` = low + front + rear fog; `11` = parking lamps + front fog (transient while turning the switch). Fog bits are mirrored in `006 b0` bits 6/7. High beam is **not** here (see `006 b0` bit5, `11D b0` bit5).

### `210` — light switch, panel dimmer
DLC 2 · 500 ms

| Byte | Bit | Meaning | Confidence |
|---|---|---|---|
| `b0` | — | `03` lights off, `02` any lights on (`00/01` wake/key-off transients) | **Confirmed** |
| `b1` | — | instrument-panel illumination % × 2 (`00` with lights off) | **Confirmed** (exact vs CCN `2106`) |

- Dimmer ladder `22/44/66/A4/C8` = 17/34/51/82/100 %.

### `2C0` — pedal sensors, boost, brake flags
DLC 8 · 100 ms

| Byte | Bit | Meaning | Confidence |
|---|---|---|---|
| `b0:b1` | — | `FF FF` | Static |
| `b2` | — | **accelerator pedal sensor 1 voltage**, 5/256 V/count (rest `17` = 0.45 V) | **Confirmed** |
| `b3` | — | **pedal sensor 2 voltage**, 5/256 V/count (half-voltage track, so ≈ `b2`/2) | **Confirmed** |
| `b4` | — | **boost / manifold absolute pressure**, 8 hPa/count (`7E` ≈ 1010 hPa at rest); saturates at `FE` (~2030 hPa) | **Confirmed** (r=0.999) |
| `b5` | — | `FF` | Static |
| `b6` | — | exact copy of `b2` | **Confirmed** |
| `b7` | 1 | always set | Unknown |
| `b7` | 2 | brake switch | **Confirmed** (17/17 vs PCM) |
| `b7` | 3 | brake switch (second contact) | **Confirmed** (17/17 vs PCM) |
| `b7` | 7 | selector not in Park | **Probable** |

- PCM `210C` APP1 volts = `0.0194×b2`, APP2 = `0.0195×b3`; PCM `2103` boost (hPa) = `8.00×b4 + 4`. The PCM reads up to 2565 hPa while `b4` sits at `FE`.

### `370` — wiper
DLC 6 · event-driven

| Byte | Bit | Meaning | Confidence |
|---|---|---|---|
| `b0` | — | pulses `01` then `00`, one event per wiper sweep | **Confirmed** |
| `b1..b5` | — | `00` | Static |

## RAR radio/head-unit IDs

These are sent by the factory **RAR radio** (they disappear with the radio
unplugged). With an aftermarket head unit and CAN box fitted instead, see
*Aftermarket CAN-box frames* below.

### `09F` — radio source/mode
DLC 8 · ~1 s

| Byte | Bit | Meaning | Confidence |
|---|---|---|---|
| `b0..b2` | — | radio source / current mode (value map incomplete) | **Confirmed** source |
| `b3` | — | `00` | Static |
| `b4..b6` | — | `FF FF FF` | Static |
| `b7` | — | `1F` | Static |

### `1AB` — head-unit heartbeat
DLC 6 · ~3 s

| Byte | Bit | Meaning | Confidence |
|---|---|---|---|
| `b0` | — | `03` | Unknown/static |
| `b1..b5` | — | `00` | Static |

### `1BB` — head-unit heartbeat
DLC 6 · ~1 s

| Byte | Bit | Meaning | Confidence |
|---|---|---|---|
| `b0..b5` | — | `00` in all captures | Static |

### `394` — radio buttons
DLC 6 · ~1 s heartbeat, plus on press and on release

| Byte | Bit | Meaning | Confidence |
|---|---|---|---|
| `b0..b2` | — | mirror of radio mode/status from `09F` | **Confirmed** |
| `b3` | 0 | seek right | Unverified (not in captures) |
| `b3` | 1 | seek left | Unverified (not in captures) |
| `b3` | 2 | fast-forward | Unverified (not in captures) |
| `b3` | 3 | rewind | Unverified (not in captures) |
| `b3` | 5 | random / PTY | Unverified (not in captures) |
| `b4` | — | unknown | Unknown |
| `b5` | — | `00` | Static |

- `b3..b5` are `00` in all 5510 captured frames (no button presses captured).

### `3D0` — amplifier volume/EQ
DLC 7 · ~1 s

| Byte | Bit | Meaning | Confidence |
|---|---|---|---|
| `b0` | — | volume | **Confirmed** |
| `b1` | — | bass (centre ~`0A`) | **Confirmed** |
| `b2` | — | mid | **Confirmed** |
| `b3` | — | treble | **Confirmed** |
| `b4` | — | balance | **Confirmed** |
| `b5` | — | fade (e.g. `0C`) | **Confirmed** |
| `b6` | — | `FF` | Static |

### `3DD` — VES-present command (injected)
DLC 8 · sent on demand

| Byte | Bit | Meaning | Confidence |
|---|---|---|---|
| `b0` | — | `03` = VES present → RAR enables its AUX input | **Confirmed by injection** |
| `b1..b7` | — | `00` | Static |

## Aftermarket CAN-box frames (not factory)

With an Android head unit and CAN box (N32L406 firmware, profile 0, 83.3k)
in place of the RAR radio, the box transmits these frames itself.

| CAN ID | DLC | Observed | Meaning (from the firmware) |
|---|---|---|---|
| `09F` | 8 | `01 23 E2 00 00 00 00 1F`; `07 …` after RAP ends | radio source/mode frame, emulating the RAR |
| `3D0` | 7 | e.g. `11 0A 0A 0A 0A 0A FF` | amplifier volume/EQ frame (factory amp order) |
| `18C` | 8 | `40 03 E2 20 00 00 00 00`; `50 …` | `b0` bit4 = "ignition off" marker (set when ACC/RAP ends) |
| `190` | 7 | `01 0D 65 00 10 00 07` | box periodic frame, semantics not established |
| `326` | 8 | `00 07 0F 07 00 00 00 00` | box periodic frame, semantics not established |
| `0F0` | 4 | `00 00 00 00` | box periodic frame, semantics not established |

In this configuration `394`, `1AB` and `1BB` are absent. NM node `416` is
present, so the box takes over the radio's network-management slot. `09F` and
`3D0` then come from the box, not a factory module.

## Static, heartbeat and still-unknown cyclic IDs

Constant payloads are listed for completeness; a constant payload is not
proof of a physical meaning.

| CAN ID | DLC / period | Observed bytes | Confidence |
|---|---|---|---|
| `008` | 6 / ~1 s | `00 00 80 80 00 00`. | **Static/unknown** |
| `010` | 8 / ~2 s | Usually `41 01 15 71 63 80 2A 03`; `b0 bit1` participates in a shared alive/toggle burst (`41↔43`). | **Static/unknown** |
| `011` | 8 / ~2 s | Usually `3D 13 00 03 8B 40 00 00`; `b0 bit1` toggles (`3D↔3F`) with several related IDs. | **Static/unknown** |
| `02E` | 2 / ~1 s | `1F 00`. | **Static/unknown** |
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
indicate a shared transmitter/alive state. They do not establish what the
remaining payload fields mean. `011 b1 = 0x13` is constant, not a temperature.

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
| `416` | `16` | `1C` | **RAR head unit/radio** (absent with the radio unplugged). |
| `41C` | `1C` | `1E` | Joins the ring only while ignition is ON. |
| `41E` | `1E` | `00` | Often carries the sleep-indication bit in `b2`. |

The logical ring is `400 → 402 → 414 → 415 → 416 → 41C → 41E → 400`.

## Power modes and RAP

Retained Accessory Power (RAP) is decided by the body module and broadcast
in `006 b1`; it is not just a consequence of `000` plus the doors.

| Event | `000 b0` | `006 b1` | Bus |
|---|---|---|---|
| Wake (fob unlock), no key | `00` | `00` | awake |
| Key inserted at OFF | `01` | `00` | awake |
| ACC | `63` | `20` → `22` | awake |
| ON / RUN | `85` | `22` | awake |
| Crank | `A9` | `20` (bit1 shed) | awake |
| Back to ACC, then OFF | `63` → `01` | `22` (RAP held) | awake |
| Key pulled | `00` | `22` | awake |
| Door opened at ACC, key **in** | `63` | stays `22` | awake |
| Door opened ~30 s after OFF, key out | `00` | `22` → `00` in one frame | awake |
| Door opened during RAP, key out, 1.65 s after OFF | `00` | `22` → `20` (bit5 stays) | awake |
| Door opened during RAP, key out, 54 s after OFF | `00` | `22` → `02` → `00` | awake |
| 600.0 s after OFF, doors closed | `00` | `22` → `02` → `00` | **silent 2.5 s later** (accessory relay clicks, bus sleeps) |
| Door opened after timeout | `00` | stays `00` | wakes again |

Rule for an accessory device that should behave like the radio:

- **On** while `006 b1 & 0x02` is set.
- **Off** when it clears; as a backup also off after ~2 s of bus silence.
- Never switch on merely because the bus is awake: a door opening after the
  timeout wakes the bus with RAP still off.

`bit5` (`006 b1 & 0x20`) meaning is **unknown**: it is set at ACC, survives
crank and clears first at the timeout, but the two door cases disagree
(`22 → 20` vs `22 → 02 → 00`; one sample each, both key out). Candidates:
a short post-OFF latch, or ACC→OFF path vs RUN→OFF. With the key in, a door
opened at ACC leaves `006 b1` at `22`.

`159 b0` counts minutes since ignition left RUN (resets ~0.7 s after `000`
leaves `85`, ~60 s ticks), so it reaches `0A` about 5 s before RAP clears.
The RAP timer itself counts from OFF (`01`); do not use `159` for exact
timing.

## Temperatures and gauge fuel level

Vehicle: **3.0 CRD diesel** (OM642).

**Outside temperature has two signals.** `015 b0` is the raw sensor. `014 b4:b5`
is the filtered value the cluster displays: it follows the sensor while
moving, and rises are blocked when slow or stopped (to hide engine-bay heat
soak). Example drive, per minute:

| Minute | `014 b4:b5` | `015 b0` | km/h | Behaviour |
|---|---|---|---|---|
| 0–10 | 23.2 → 25.0 | 23.2 → 24.9 | 20–58 | tracks |
| 11–18 | 25.00 held | 25.1 → 26.5 | 6–18 | rise blocked |
| 19–23 | 25.0–25.5 | 25.0–25.4 | 31–49 | tracks |
| 24–27 | 24.67 → 24.40 held | 24.4 → 25.1 | 0–22 | rise blocked |

`014 b4:b5` = 19.50 °C reads as 19 °C on the dash. Open: dash rounding/offset
(one dash reading); no sub-zero value observed.

`014 b4:b5` is **unsigned** 16-bit big-endian: °C = raw × 0.01 − 40, where
raw 0 = −40 °C and raw 4000 = 0 °C (−5 °C should read about `0DAC`). `FFFF`
means unavailable; check for it before decoding.

**Fuel level: `159 b5` and `014 b6`.** `159 b5` is the damped level,
≈0.5 L/count (maximum 154 ≈ the 77.6 L tank; PCM `2111` reads ~4 L above
`b5×0.5` at 68–72 L). `014 b6` is the raw sender voltage, 5/255 V/count,
nonlinear in litres (float sender curve). Medians per tank level, range in
brackets:

| Tank | `159 b5` raw | `159 b5` × 0.5 | `014 b6` raw | sender V |
|---|---|---|---|---|
| low | 28 (25–30) | 14.0 L | 35 (4–49) | 0.69 |
| half | 77 | 38.5 L | 67 | 1.31 |
| ~7/8 | 133–135 | 66.5–67.5 L | 87–88 (83–91) | 1.71–1.73 |
| ~9/10 | 140 (138–142) | 70.0 L | 89 (83–92) | 1.75 |
| ~9/10 | 143–145 | 71.5–72.5 L | 90 (84–92) | 1.76 |
| full | 152–154 | 76.2–76.5 L | 92 (78–92) | 1.80 |

Open: absolute `159 b5` scale. To calibrate, log ~1 min with the engine
running at a low tank, then fill to the brim and record the pump litres.

## Derived values: fuel consumption, injection, torque, load

Calibrated against PCM `2103`/`2106` injection quantity, PCM `212B` fuel rate
and TCM `2154` static engine torque. All inputs are CAN-B broadcasts:

```
Q    = 1A7 b0:b1                 (FFFF = n/a)
rpm  = 002 b0:b1                 (FFFF = n/a)
kmh  = 002 b2:b3 / 128

fuel_L_per_h   = Q / 128
fuel_L_per_100 = fuel_L_per_h / kmh * 100          (only while kmh > ~3)
trip_litres   += Q * dt_s / (128 * 3600)           (integrate over time)

inj_mm3_stroke = 43.4 * Q / rpm                    (rpm > 400; 43.4 = 1e6 / (128 * 180))
torque_Nm      ≈ 6.43 * inj_mm3_stroke - 48        (brake torque, ±15 Nm)
load_pct       ≈ 100 * inj_mm3_stroke / 90         (clamp 0..100)
power_kW       ≈ torque_Nm * rpm / 9549
```

- **Injection**: `43.4×Q/rpm` equals the PCM's *Current Injection Mass*
  (mm³/stroke) with slope 1.00, r = 0.989–0.994. The constant comes from
  `L/h → mm³/h` (×10⁶) over strokes per hour (`rpm/2 × 6 cyl × 60`).
- **Torque**: fit against TCM `2154` *Static Engine Torque*, r = 0.982, n = 67.
  6.43 Nm per mm³ ≈ 38 % thermal efficiency; −48 Nm is friction/pumping at zero
  fuel (overrun). TCM `2130` *Engine Torque* is mis-scaled in the tool tables
  (wraps at ±32768) — not used.
- **Load %**: a diesel has no throttle, so "load" is injected fuel per stroke
  against full load. Peak injection is 89–91 mm³ (≈530 Nm, the OM642's
  rated 510 Nm), so 90 mm³ = 100 %. Idle ≈ 9 mm³ ≈ 10 %; overrun = 0.
- Secondary load indicators: boost `2C0 b4` (8 hPa/count abs; boost above
  ambient = `b4×8 − 1013` hPa) and pedal `02C b2` (0.4 %/count). Pedal is the
  driver's request, not the engine's load (cruise, limiters and gear changes
  separate them).

## OBD cross-reference

CAN-B logged passively alongside KWP2000 local-identifier reads from PCM,
TCM, ABS and CCN on the OBD port, on one shared clock. Each OBD response is
matched against the last CAN-B frame at the request midpoint, with a lag scan
of ±1 s.

| CAN-B field | OBD reference | Result |
|---|---|---|
| `02C b0` | PCM `21AA` Oil Pressure | oil pressure, ≈4 kPa/count gauge, r=0.996 |
| `02C b1` | CCN `2140` Coolant | `raw − 40` °C, exact |
| `02C b2` | TCM `2155` pedal % | 0.4 %/count, r=0.9995 |
| `2C0 b2` / `b3` | PCM `210C` APP1/APP2 volts | 5/256 V/count each, r=0.999 |
| `2C0 b4` | PCM `2103` Boost Pressure | 8 hPa/count absolute, r=0.999, saturates at `FE` |
| `2C0 b7` bit2/bit3 | PCM `2127` brake switches | 17/17 |
| `003 b3` bit0 | PCM `2127` brake switches | 17/17 |
| `003 b0`, `b1`, `b4`, `b6` | CCN `213F` lamp requests | same bytes (b6 shifted) |
| `014 b6` | CCN `2103` fuel sense voltage | identical raw byte |
| `159 b5` | PCM `2111` Fuel Level (L) | follows; ≈4 L offset vs `×0.5` at 70 L |
| `1A7 b0:b1` | PCM `212B` Fuel Consumption L/h | **Q/128 L/h** |
| `1B4 b1` | CCN `2152` Compass Raw | ×2 = degrees, exact |
| `002 b4:b5`, `b6:b7` | ABS `2151` / CCN `213E` wheel distance | left / right counters, identical raw words, 24.9 counts/m |
| `1B2 b1` | 4LO test | `02` 4H, `01` 4L, `00` in shift, `07` n/a |
| `006 b0` bit5, `11D b0` | high-beam test | indicator; stalk push `40`, on `20` |
| `210 b1` | CCN `2106` IP Illumination | % × 2, exact |
| `0EC b0`, `003 b1` bit5 | PCM `210D` A/C Compressor Output | compressor on/off, 4/4 cycles |
| `003 b1` bits 6/7 | CCN `213F` Cruise Engaged / On | cruise test |
| `006 b0` bits 6/7 | CCN `2133` Front / Rear Fog | fog test |
| `015 b0`, `b1` | CCN `2134` ambient / voltage | `×0.5−40` °C, `×0.1` V |

No CAN-B match exists for steering angle, yaw, lateral/longitudinal
acceleration (ABS `2102`), the four separate wheel speeds (ABS `2101`; only the left/right distance counters in `002 b4..b7` are on CAN-B), rail pressure, MAF,
charge-air or oil/fuel/transmission temperatures; those are CAN-C only.
Open: `015 b2`, `11D b0` `80`, `159 b5` absolute scale (needs a low-tank OBD
log).

## Injected but not passively observed

`3DD` is a **confirmed VES-presence command** with payload
`03 00 00 00 00 00 00 00`: the RAR radio enables AUX when this frame is
transmitted. It does not occur in passive captures, so it is excluded from
the 52 passively observed IDs.

## Important absences

- No direct steering-wheel or road-wheel angle has been identified.
- No accelerometer, pitch, roll or high-rate gyro/yaw signal has been
  identified (none of ABS `2102` steering angle, yaw or acceleration
  correlates with any CAN-B field).
- `1B4 b1` is compass heading and permits only a coarse derived yaw rate. A
  better derived yaw rate comes from the left/right wheel counters in `002`:
  `yaw ≈ (v_right − v_left) / track width` (WK track ≈ 1.6 m), positive = left turn.
- `002 b4:b7` are wheel-pulse counters, not wheel angles.
- `1B4` carries no unavailable sentinel and does not stop at key-off: `b1=0xFF`
  does not occur, and the frame keeps transmitting at its normal cadence with
  an unchanged, valid heading through the `000` ignition-off transition.
  Consumers must gate key-off on `000` or on bus silence, never on the compass
  frame itself. On bus sleep (RAP timeout) `1B4` stops together with every
  other ID.
- `159` does carry an unavailable sentinel, in `b4`, and consumers must gate on
  it: `b4=01` means `b5=0xFF` is not a reading.
- `1B4` heading is static when stopped: across 196 consecutive stationary
  (≤2 km/h) sample pairs the 500 ms heading delta is exactly 0.0° — no jitter
  to filter. Moving, the median 500 ms delta is 0° and mean |delta| is 1.9°;
  sustained turns run ~18–26°/s, with peaks up to 96° per 500 ms sample
  (192°/s) at hairpins.
