# What CAN-B actually gives us

Everything below was read from the vehicle on **2026-09-11** through the in-radio
ATmega328 on the 83.3 kbps interior bus. This is the planning document: what is
available, what it is worth, and what the missing things cost to get. Raw
evidence and the reasoning behind each conclusion are in
[PROBE-LOG.md](PROBE-LOG.md).

**The headline number:** the extracted vendor database contains 29 decodable
signals in total — 19 on CCN and 10 on FCMCGW. The gateway has no CAN-B
diagnostic interface, so **the documented dashboard is 19 signals**. The tyre
pressures are a twentieth group that we found ourselves and the database does not
know about.

---

## 1. Confirmed working

Read live, decoded, and where possible cross-checked against the dash. ✓ means a
real-world value confirmed it.

### Engine and driving

| Signal | Source | Formula | Observed |
|---|---|---|---|
| Engine RPM | `CCN 213E` bits 16-31 | `raw` rpm | ✓ **621 rpm** at warm idle |
| Vehicle speed | `CCN 213E` bits 32-47 | `raw / 128` km/h | 0 stationary — **untested moving** |
| Coolant temperature | `CCN 2140` bits 16-23 | `raw − 40` °C | ✓ **74 °C** hot, **71 °C** running |
| Throttle position | `CCN 2140` bits 24-31 | `raw × 0.392157` % | ✓ **0 %** foot off |

### Electrical and environment

| Signal | Source | Formula | Observed |
|---|---|---|---|
| System voltage | `CCN 2134` bits 24-31 | `raw × 0.1` V | ✓ **12.6 V** rest → **13.9 V** running |
| Ambient temperature | `CCN 2134` bits 16-23 | `raw × 0.5 − 40` °C | ✓ **32.5 / 33.5 °C** |

The voltage jump on starting the engine is the single best validation we have —
it also caught a bug in the extracted scaling table.

### Tyres — the find of the session

The database says per-wheel pressure does not exist on this vehicle. It does; it
just is not published to the cluster, because this is base TPMS rather than
Premium. `CCN 2130` confirms `TPM System Localization Status = Non-Localized`.

| Signal | Source | Formula | Observed |
|---|---|---|---|
| **Four road-wheel pressures** | `WCM 21 3B` bytes 0-3 | `raw` psi | ✓ **33, 32, 33, 30 psi** = 2.28 / 2.21 / 2.28 / **2.07** bar |
| Per-sensor pressure | `WCM 21 20`…`21 24` byte 0 | `raw` psi | ✓ five sensors, spare at 7 psi |
| Per-sensor temperature | same, byte 1 | `raw − 40` °C? | 45 / 49 / 51 / 43 / 36 °C — **hypothesis** |
| TPM lamp request | `CCN 2130` bits 56-63 | Off / On / Flashing | ✓ **Off** |
| TPM localization | `CCN 2130` bits 64-71 | enum | ✓ **Non-Localized** |

Confirmed three independent ways: the owner's expected 2.1–2.3 bar range, the
genuinely soft corner and underinflated spare, and a stored `C0077 Low Tire
Pressure` DTC on the WCM.

**Corner order — a hypothesis from a third party, service/formula refuted on our car.**
Someone supplied UDS-style entries: `Pressure Tire FL 0x22A020`, `FR 0x22A021`,
`RL 0x22A022`, `RR 0x22A023`, formula `A*92.46/255`. Tested on our WCM 2026-09-11:

* **Service `22` is rejected** — `03 22 A0 20` → `7F 22 80` (identical for all four
  DIDs). Our 2005 WK WCM is KWP2000 (service `21`), not UDS, so that addressing is
  for a different/newer platform. It is NOT how we read pressure here.
* **The formula does not fit our data.** Direct byte `0x21`=33 → the tip's formula
  gives 12 psi; applied to the second byte it gives a flat ~31 psi and makes the
  spare read normal, but the spare is provably soft. Our encoding stays **1 psi per
  byte, direct** — confirmed by the spare and the owner's 2.1–2.3 bar range.
* **Corner order: two hypotheses, and they conflict on the rears.** The DID
  low-bytes `20/21/22/23` map to sensor indices `21 20`–`21 23`. The third party
  labels them **FL, FR, RL, RR**. But `jeepdiag`'s `tires` command independently
  labels the same four **LF, RF, RR, LR** (its `_TIRE_CORNERS` table). They agree
  on the front pair and **swap the rears**. Latest read `21 3B` = `21 20 20 1E`
  = 33, 32, 32, 30 psi; the 30-psi (soft) corner is therefore either RR (tip) or
  LR (jeepdiag) — unresolved. Both are guesses; neither is verified on the car.

**Still verify by deflation** — the third-party order is a strong hint, not proof.
Let one known tyre down a few psi and confirm which byte moves before trusting the
corner labels.

### Warnings and fluids

| Signal | Source | Values | Observed |
|---|---|---|---|
| MIL lamp | `CCN 213F` bit 24 | True / False | ✓ **True** with engine stopped — correct |
| Oil lamp | `CCN 213F` bit 26 | True / False | ✓ **True** with engine stopped — correct |
| ABS lamp | `CCN 213F` bit 16 | True / False | ✓ False |
| Charging system failure | `CCN 213F` bit 23 | True / False | ✓ False |
| Washer fluid low | `CCN 2135` bit 16 | Yes / No | ✓ No |
| Engine coolant low | `CCN 2135` bit 18 | Yes / No | ✓ No |
| Brake fluid low | `CCN 2135` bit 19 | Yes / No | ✓ No |
| Fuel level | `CCN 2103` bits 24-31 (also broadcast `0x159` B5) | `raw × 0.5` % | **50 %** — needs a gauge check. See fuel-consumption section below |

### Body and navigation

| Signal | Source | Values | Observed |
|---|---|---|---|
| Compass direction | `CCN 2152` bits 16-23 | 8 points | ✓ `02` = **East** |
| Parking brake | `CCN 2110` bit 21 | True / False | False |
| Left front door ajar | `CCN 2110` bit 27 | True / False | **True** — polarity unverified, check against the actual door |

### Identity

`1A 87` returns an ECU part number from every reachable module:

| Module | Request / response | Part number |
|---|---|---|
| CCN cluster | `6A0` / `514` | `5172008AD` |
| RADIO | `6B0` / `516` | `5064065AE` |
| ORC airbag | `6E0` / `51C` | `4606938AD` |
| WCM wireless | `600` / `500` | `56053017AU` |
| SCM steering | `6A8` / `515` | `6050097AM` |
| EOM overhead | `6F0` / `51E` | `6050242AG` |

### Odometer and VIN — SOLVED from broadcast traffic

Neither is reachable by KWP (both live only on the unreachable gateway), but both
are broadcast continuously on CAN-B and were decoded live on 2026-09-11:

| Signal | Source | Formula | Observed |
|---|---|---|---|
| **Odometer** | broadcast **`014`** bytes 0-2 | 24-bit big-endian × 0.1 km | ✓ **246278.5 km**, matched the dash exactly. Also readable as KWP via `WCM 21 14` bits 32-55 |
| **VIN** | broadcast **`01B`** | ASCII, byte 0 = frame index, bytes 1-7 = chars | ✓ **`1J8HCE8M45Y588369`**, owner-confirmed |

The odometer means **trip distance is free** — diff two `014` reads. `WCM 21 14`
gives the same value as a normal poll, so it fits the dashboard like any other
signal, no broadcast sniffing needed. The two 16-bit wheel-distance counters in
`CCN 213E` bits 48-79 also climb with distance and give left/right independently.

### Fuel consumption — derivable from broadcast, CONFIRMED live

The owner's fuel-economy goal is reachable on CAN-B without a CAN-C tap. Three
broadcast signals, all confirmed on the car:

| Piece | Source | Notes |
|---|---|---|
| **Instantaneous burn** | broadcast **`0x1A7`** B0-B1 (uint16) | Injection quantity / engine load. Verified live 2026-09-11: r=**0.903** vs RPM over a rev sweep, monotonic (600 rpm→132, 1500 rpm→689). Vendor cross-ref: PCM "Current Injection Mass" ×0.01 mm³/stroke |
| **RPM** | broadcast **`0x002`** B0-B1 | needed to turn per-stroke mass into a rate: fuel/time ∝ `1A7 × RPM` |
| **Distance** | broadcast **`0x014`** (odometer) | for L/100 km |
| **Tank level** | broadcast **`0x159`** B5 | falls in the burn direction over both real drives; absolute scale unpinned |

**Vehicle is a CRD diesel** (engine ECU is a Bosch EDC16CP31, per the owner's bench
flash backup in `my-ecu/`). That matters here: diesel ECUs meter fuel natively in
**mm³/stroke injection quantity**, which is exactly what `0x1A7` is — so this is the
ECU's own fuel-metering value on the bus, not a proxy. The scaling constant **k**
below can be read out of the EDC16 flash (injector maps) rather than only estimated
from a fill-to-fill.

**Live rate** = k · (`0x1A7` value) · RPM · (cylinders/2). Integrate over time for
fuel used; divide by odometer distance for L/100 km. Only the constant **k** needs
one calibration (integrate over a known fill-to-fill and solve). A live economy
readout and a trip average are otherwise fully derivable from unsolicited traffic —
no polling, no CAN-C.

Corrected earlier guess: `0x159` bytes 0-3 are NOT a runtime counter (B0 constant
`0xFE`, B1-B3 frozen `FF`); only B5 carries the level.

### Fault codes

**315 DTC descriptions readable right now**, via `04 18 00 FF 00`, with
multi-frame reassembly and auto flow control both proven on the real bus:

| Module | Codes in table | Status |
|---|---|---|
| WCM | 93 | ✓ reads |
| CCN | 88 | ✓ reads |
| SCM | 53 | ✓ reads |
| RADIO | 51 | ✓ reads |
| EOM | 30 | ✓ reads |
| **Total readable** | **315** | |
| ORC | 185 | ✗ answers `responsePending` and never delivers — needs `0x10 startDiagnosticSession` |

---

## 2. Sitting there, meaning unknown

Real data we can read but cannot yet interpret. All of it is raw-logged in
PROBE-LOG.md.

| What | Amount | Notes |
|---|---|---|
| WCM PIDs not in the vendor DB | a few (of ~45 seen) | The AppcarFCA PID database names **34** of the WCM `21` requests (162 fields, incl. the tyre set), so most are documented; only a handful the sweep saw aren't in it — e.g. `21 2A` = `01 02 03 04 05` (a sensor index table) and `21 10` = 173/173/171, unidentified |
| WCM security-gated PIDs | 11 ids | `21 03`–`0B`, `12`, `13` answer `securityAccessDenied`. Unlock + seed→key now known — see below |
| CCN PIDs not in the vendor DB | the residual of a full `21` sweep | The AppcarFCA PID database names **39** of the CCN `21` requests (165 fields), so most of the ~54 ids the sweep saw are documented there — not 9. Genuinely unnamed: `21 D4`/`21 D8` (a 58-byte composite that is **static config**, byte-identical across throttle 0→66 %, so NOT live telemetry) and some `21 4x`/`21 Ex` |
| SCM undocumented PIDs | 28 ids | mostly identity: date code `2005-06-21`, serial `252898F` |
| **SCM deferred PIDs** | **11 ids** | `21 18`, `21`, `30`–`38` answer `responsePending`. **The SCM straddles CAN-B and CAN-C** — these are the best free lead we have |
| Broadcast ids | 36 seen, 34 unidentified | `09F` radio mode and `394` panel buttons are known and used. `01B` carries ASCII `M45Y588`; `1B4` mirrors the compass PID |

### WCM security access — unlock path and seed→key both known

The 11 `securityAccessDenied` PIDs sit behind a standard `27` seed/key unlock on
the WCM at `600/500` (see `../../../capture/WCM-SECURITY-ANALYSIS.md`):

* Send `27 01`, WCM replies `67 01 <8-byte seed>`; send `27 02 <4-byte key>`,
  WCM replies `67 02` (accepted). A second level `27 03`/`27 04` exists.
* **The seed→key transform is solved** — a Hitag2-style bit-serial cipher,
  implemented in jeepdiag (`jeepdiag/jeepdiag/seedkey.py`, `wcm_key`). Level 1
  gates the PIN read, level 3 the write/verify routine. Confirmed live on the
  car: `jeepdiag pin WCM` read PIN **9378**.
* The SKIM **PIN 9378** is read *after* unlock via `23 00 2E B9 02`
  (ReadMemoryByAddress), returned as BCD `93 78` — not derived from the seed.

**Consequence for us:** the gated WCM PIDs are now reachable — send the `27 01`/
`27 02` unlock (jeepdiag computes the key) before reading them. The tyre pressures
are ungated and need no unlock. It is the same `27` mechanism the SCM/ORC deferred
PIDs may use — worth comparing when a security path is added to the gateway.

---

## 3. What this supports as a dashboard

A realistic, honest screen from what is confirmed today:

* **Tyres** — four pressures in bar, per-corner, with the low one highlighted.
  This is the feature that motivated the project and no factory display on this
  vehicle offers it.
* **Engine** — RPM, coolant temperature, throttle.
* **Electrical** — battery voltage, with charging vs rest obvious at a glance.
* **Environment** — ambient temperature, compass.
* **Fuel** — level percentage.
* **Warnings** — MIL, oil, ABS, charging, and the three fluid-low flags, which
  together are a better warning panel than the cluster shows.
* **Faults** — a fault list across five modules with real text, 315 codes deep.

Missing from that list versus a modern car: **odometer, trip distance, gear,
outside-air-temp history, and anything engine-internal.** Speed is available but
has never been seen non-zero.

### Timing budget

Polling is cheap. A single-frame request/response is two CAN frames, ~2.6 ms on
an 83.3 kbps bus, plus the UART round trip to the ATmega328. Measured responses
came back well inside a 200 ms `ATST`. Six PIDs at 2 Hz is ~12 requests/second
against a bus already carrying ~168 frames/second, so a full dashboard refresh
is not remotely near the limit. Multi-frame DTC reads are slower and belong
off the fast path.

The one caveat: everything is a **poll**, not a subscription. Values update at
whatever rate we ask, so a smooth analogue-style tacho needle is not realistic;
digital readouts are.
