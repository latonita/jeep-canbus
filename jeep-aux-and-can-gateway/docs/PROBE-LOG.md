# Probe log -- first bring-up in the car

A worksheet for the first real session on the interior CAN-B bus. Copy this file, run
the script from top to bottom, and fill in the tables by hand as you go. Everything here
is a question, not a claim: nothing in the tables has been confirmed on this vehicle yet.

Command reference: [AT-COMMANDS.md](AT-COMMANDS.md). All commands below assume
`ATCAF0`, so **request bytes include the ISO-TP PCI byte** -- `21 30` is typed as
`02 21 30`.

---

## How to run this session

* **Engine off, ignition on** for the first pass. Do the whole script once that way
  before you try anything with the engine running -- RPM and speed reads are the only
  ones that need it, and they come later.
* **Watch the dash.** If a warning lamp appears, the cluster blanks, the radio resets, or
  anything at all behaves oddly: stop, note what you had just sent, power-cycle the
  ignition, and do not repeat that command until you understand it.
* **Do not flood the bus.** This is a shared 83.3 kbps interior bus and the radio and
  cluster are using it for real work. One request, read the answer, then the next. No
  loops, no polling scripts, no "let me just retry that twenty times".
* **Read-only path only.** Every service used here is a read: `1A` ident, `21` read local
  id, `18` read DTCs. No security access, no session setup, no writes, no clearing of
  DTCs. If a module answers `7F xx 33` (or any other negative code) note it and move on
  rather than trying to get past it.
* Keep the terminal log. Paste raw hex into the tables even when you cannot decode it --
  an unexplained byte is still evidence.

Session metadata to fill in:

| | |
|---|---|
| Date | 2026-09-11 |
| Vehicle | 2005 Jeep Grand Cherokee WH |
| VIN | |
| Odometer (dash) | |
| Ignition state | run, then engine running |
| Ambient temp (dash) | |
| Fuel gauge (dash, eyeballed) | |
| Firmware (`AT@I`) | |

---

## Module map (from the vendor diagnostic database)

Reference material only -- this project has no dependency on that database, and none of
it has been verified on this car. Modules answer KWP2000 request/response over ISO-TP
with 11-bit ids, and **there is no gateway hop on this vehicle**: every module is
addressed directly as a peer.

Expected on the interior bus:

| Module | What it is | Request id | Response id |
|---|---|---|---|
| CCN | instrument cluster | `6A0` | `514` |
| FCMCGW | front control module / central gateway | `620` | `504` |
| RADIO | head unit | `6B0` | `516` |
| ORC | airbag / occupant restraint | `6E0` | `51C` |
| WCM | wireless control module | `600` | `500` |
| SCM | steering column module | `6A8` | `515` |
| EOM | | `6F0` | `51E` |

On the 500 kbps CAN-C bus, so probably **not** reachable from here:

| Module | What it is | Request id | Response id |
|---|---|---|---|
| PCM | engine | `7E0` | `7E8` |
| TCM | transmission | `7E1` | `7E9` |
| ABS | brakes | `784` | `785` |
| FDCM | | `7B6` | `7B7` |
| ESM | | `788` | `789` |

---

## Probe script

Run in this order. Each step is a confidence-builder for the next one.

### Step 0 -- does it send and receive at all?

This is the whole question on the first trip, and most of it is answered without
typing anything. Do it before the KWP steps below.

**Receiving.** `AT@ST` rx counter rising, and `ATMA` streaming frames (Steps 1-2).

**Transmitting.** CAN needs at least one other node to acknowledge a frame, so an
unacknowledged transmit is not silent -- the MCP2515 retries and its transmit error
counter climbs. That gives two independent proofs:

* `AT@ST` **tx rising while err stays flat** -- frames are being ACKed by other
  modules, so transmit works electrically and at the right bit rate.
* **The radio offers AUX / VES in its source list.** That is the announce on `0x3DD`
  being received and understood by the radio, which is transmit working end to end,
  and it needs no terminal at all.

**Both at once.** Any request/response pair in Steps 3-7 proves transmit and receive
together at the protocol level. `02 1A 87` to the cluster is the cheapest one.

#### If rx stays at zero

Do not assume wiring. The **MCP2515 crystal is the open question**: `CAN_CRYSTAL_ID`
is built as `MCP_16MHz`, and if the part is really 8 MHz the controller sits on the bus
at 41.7 kbps and receives nothing.

Distinguish them by the error counter:

| `AT@ST` shows | Meaning |
|---|---|
| rx 0, err **0** and flat | Not connected to the bus at all -- check the splice |
| rx 0, err **climbing** | Wrong bit rate: the controller is on a live bus it cannot decode |

**If err is climbing, power the board down promptly.** A node running at the wrong bit
rate does not just fail to read -- it transmits error frames into a bus the radio and
cluster are using for real work. Do not leave it sitting there while you think about it.

The fix is a reflash, which is why the Mega and a prebuilt hex are worth having in the
car: see `tools/flash-garage.bat`.

### Step 0b -- the autonomy regression

The invariant is that the car never depends on the host. Confirm it here, because the
garage is the only place with a real radio:

| Check | Expected | Result |
|---|---|---|
| Radio source list offers VES / AUX | yes | |
| Select AUX -> `AT@EV BTPWR 1`, BT module powers up | yes | |
| Phone pairs and plays through the radio | yes | |
| Press seek / RND -> `AT@EV BUTTON` then `AT@EV BTCMD` | yes | |
| **Unplug the UART adapter while audio is playing** | nothing skips, audio continues | |
| Plug it back in, `ATI` | answers immediately, audio undisturbed | |

The last two are the point of the whole design: the AT link is observe-only and the
board has no idea whether anyone is listening.


### Step 1 -- link alive

```
ATZ                     # reset; expect the identity string back
AT@I                    # true firmware name + version
AT@ST                   # CAN health, rx/tx/err counters, header, filter, bench state
AT@EV1                  # turn on event lines - this is the only debug output there is
ATE0                    # echo off
ATH1                    # print the id of every received frame
ATS1                    # spaces between hex bytes
ATCAF0                  # raw frames; we supply the PCI byte
```

Pass condition: `AT@ST` reports CAN healthy and a **non-zero, rising rx counter**. If rx
is stuck at zero, do not jump to the wiring -- see [Step 0](#step-0--does-it-send-and-receive-at-all)
for how to tell a bad splice from the wrong MCP2515 crystal.

| Check | Result |
|---|---|
| `ATZ` reply | |
| `AT@I` reply | |
| `AT@ST` rx / tx / err | |
| CAN healthy? | |
| `AT@EV` lines seen while idle | |

### Step 2 -- is the bus actually carrying traffic

```
ATCRA                   # clear any leftover receive filter
ATCM 000                # accept everything
ATMA                    # monitor all; send any character to stop
```

Let it run a few seconds, then stop it. Wiggle the radio source/volume and press panel
buttons while it runs -- `0x09F` and `0x394` should appear. Record what you see in
[Observed broadcast ids](#observed-broadcast-ids-atma).

Pass condition: a steady stream of frames. If `ATMA` is silent but `AT@ST` showed rx
counting, suspect the filter; if both are silent, suspect the wiring.

### Step 3 -- CCN ident

The first request/response round trip. Short reply, so no flow control involved yet.

```
ATSH 6A0                # CCN request id
ATCRA 514               # CCN response id
ATFCSH 6A0              # flow control back out on the request id
02 1A 87                # readEcuIdentification, ident block 87
```

Expect a positive response starting `5A 87` (`0x1A + 0x40`). This may well be multi-frame
(`1x ...` First Frame) -- if so, flow control is already being exercised and step 6 will
be easy.

| | |
|---|---|
| Raw response | |
| Multi-frame? | |
| Decoded ident string | |

Optional while you are here: `02 1A 88` and `02 1A 90` are VIN blocks. Cross-check
against the VIN on the windscreen.

### Step 4 -- fuel level, cross-checked against the gauge

The first reading with a physical thing to compare it to. **Note the dash gauge before
you send this.**

```
02 21 03                # CCN local id 2103 - fuel level %
```

Expect `61 03 ...`. Find the byte that plausibly matches the gauge. Fuel level as a
percentage suggests one byte, 0..100 or 0..255 scaled -- work it out from the reading,
do not assume a scale.

| | |
|---|---|
| Dash gauge reading | |
| Raw response | |
| Candidate byte / offset | |
| Decoded value | |
| Matches gauge? | |

### Step 5 -- RPM and vehicle speed

```
02 21 3E                # CCN local id 213E - engine RPM + vehicle speed
```

Engine off, both should read zero or near it -- which is itself useful, because a byte
that is zero when it should be zero is a byte you can trust. Repeat with the engine
idling and compare against the tachometer.

| Condition | Raw response | RPM decoded | Speed decoded | Matches dash? |
|---|---|---|---|---|
| Engine off | | | | |
| Engine idling | | | | |

Then the other cluster reads, same pattern:

```
02 21 40                # coolant temp + throttle position
02 21 34                # ambient temp + system voltage
02 21 3F                # warning-lamp requests (MIL, oil, ABS, charging)
02 21 30                # tire-pressure indication + TPM localisation
02 21 52                # compass
```

`2134` is the best sanity check of the lot: system voltage should be around 12.5 V with
the engine off and around 14 V with it running, and ambient temp should match the dash
readout. If those two decode, the byte order convention is established for everything
else.

### Step 6 -- DTC list: the flow-control test

This is the first request whose reply is certainly too long for one frame, so it is the
real test of the auto-`30 FF 00`. No security access and no session setup are needed on
the read path.

```
ATST 64                 # 400 ms - the module may answer 7F 18 78 (pending) first
04 18 00 FF 00          # readDiagnosticTroubleCodesByStatus, all groups
```

Expect a First Frame (`10 xx 58 ...`), then Consecutive Frames (`21 ...`, `22 ...`).
Reassemble on the host and count payload bytes against the length declared in the First
Frame.

If you get the First Frame and then **silence**, flow control failed. Pad the FC frame to
DLC 8 and retry -- see the flow-control section of
[AT-COMMANDS.md](AT-COMMANDS.md#6-flow-control):

```
ATFCSM 1                            # use my FC header and my FC data
ATFCSD 30 FF 00 00 00 00 00 00      # same FC, padded to 8 bytes
04 18 00 FF 00                      # retry
```

| | |
|---|---|
| First Frame | |
| Declared total length | |
| Consecutive Frames received | |
| Bytes reassembled | |
| Needed padded FC? | |
| Response pending (`7F 18 78`) seen? | |
| Decoded DTCs | |

Repeat the same `04 18 00 FF 00` against every module in
[Module reachability](#module-reachability) once it answers ident.

### Step 7 -- odometer from the gateway, cross-checked

Switches modules, which also proves the addressing works for more than the cluster.

```
ATSH 620                # FCMCGW request id
ATCRA 504               # FCMCGW response id
ATFCSH 620              # flow control follows the request id
02 1A 87                # ident first - confirm the module is there at all
02 21 A3                # local id 21A3 - odometer
```

The odometer is the single best cross-check available: it is a large, unambiguous number
that you can read off the dash to the kilometre. Getting it to match nails down byte
order and scaling in one shot.

| | |
|---|---|
| Dash odometer | |
| Raw response | |
| Decoded value | |
| Byte order / scale deduced | |

Then:

```
02 21 B0                # vehicle speed
02 21 B1                # RPM + PRND
02 21 10                # ambient temp + battery voltage
```

`FCMCGW 2110` and `CCN 2134` both claim ambient temp and voltage. If they agree, both
decodes are probably right; if they disagree, at least one is wrong.

---

## The strategic test: is CAN-C reachable from here?

**Run this deliberately, and write the answer down.** It decides the shape of the whole
project.

```
ATSH 7E0                # PCM (engine) request id - lives on the 500 kbps CAN-C bus
ATCRA 7E8               # PCM response id
ATFCSH 7E0              # flow control follows the request id
ATST 64                 # give it time
02 1A 87                # readEcuIdentification - the cheapest possible "are you there"
```

| Outcome | What it means |
|---|---|
| **A reply** (`5A 87 ...`, or even a negative `7F 1A xx`) | CAN-C modules are reachable from the interior bus. That puts all 1152 DTCs from the vendor database in play, plus live engine and transmission data straight from the PCM/TCM. The dashboard can be built on direct reads. |
| **`NO DATA`** | The interior bus does not carry CAN-C diagnostic traffic and there is no gateway hop doing it for us. The dashboard must be built from the values the **cluster (CCN) and gateway (FCMCGW) already mirror** onto the interior bus -- which is why steps 4 to 7 above matter so much: they are the fallback data source. |
| **`CAN ERROR`** | Not an answer to this question. The frame never made it onto the wire; fix that first and re-run. |

Either way, this one result is what decides **whether a second CAN-C transceiver is ever
needed**. If CAN-C answers, the board stays as it is. If it does not, then any future
access to engine/ABS/transmission data means a second MCP2515 on the 500 kbps bus (or an
ESP32-S3 with two controllers) -- a hardware change, not a firmware one. Do not start
that work until this line has been run in the car.

| | |
|---|---|
| Result | |
| Raw response, if any | |
| Repeated with `7E1` / `7E9` (TCM)? | |
| Repeated with `784` / `785` (ABS)? | |
| Conclusion | |

---

## Results tables

### Module reachability

Probe each with `02 1A 87` after setting `ATSH` / `ATCRA` / `ATFCSH`.

**Run 2026-09-11, engine running.** Six of seven CAN-B modules answered, all with
part numbers matching the extracted database's module map -- strong end-to-end
validation of both the DB and the adapter.

| Module | Req id | Resp id | Answered | Part number from `1A 87` |
|---|---|---|---|---|
| CCN | `6A0` | `514` | **yes** | `5172008AD` |
| FCMCGW | `620` | `504` | **no** | -- silent, see below |
| RADIO | `6B0` | `516` | **yes** | `5064065AE` |
| ORC | `6E0` | `51C` | **yes** | `4606938AD` |
| WCM | `600` | `500` | **yes** | `56053017AU` |
| SCM | `6A8` | `515` | **yes** | `6050097AM` |
| EOM | `6F0` | `51E` | **yes** | `6050242AG` |
| PCM (CAN-C) | `7E0` | `7E8` | **no** | -- |
| TCM (CAN-C) | `7E1` | `7E9` | **no** | -- |
| ABS (CAN-C) | `784` | `785` | **no** | -- |
| FDCM (CAN-C) | `7B6` | `7B7` | **no** | -- |
| ESM (CAN-C) | `788` | `789` | **no** | -- |

#### Exhaustive scan: there are exactly six modules, and no more

Every request id `600`..`6FF` was tried with `02 1A 87`, with the receive filter
opened to the whole `500`..`5FF` response range so an unknown module could not be
missed by guessing its reply id (`ATCF 500` / `ATCM 700`). **Exactly six
responders, all already known.** No undocumented modules exist on this bus.

That settles two things that were previously inference:

* **FCMCGW is genuinely absent**, not sitting at some other address. `620` was
  silent and so was every other id in the space.
* **The module map is complete.** The dashboard has six modules to work with:
  CCN, RADIO, ORC, WCM, SCM, EOM.

#### The strategic question is answered: CAN-C is NOT reachable

All five CAN-C modules -- PCM `7E0`, TCM `7E1`, ABS `784`, FDCM `7B6`, ESM `788`
-- were tried and none answered. `7E0` and `7E1` were tried with the ignition on
**and** with the engine running, so "the module was asleep" does not explain it. Nothing on CAN-C answers a
request originating on the interior bus. **The dashboard is therefore built on
CCN mirrors**, and a CAN-C transceiver stays a genuine option for later rather
than a solved problem. The ESP32-S3's TWAI controller is still deliberately
unused, so adding one changes no architecture.

#### How the gateway actually works -- from the factory schematics

Read off `temp/Jeep WK Schematics.pdf`, Fig. 8 and Fig. 9 ("Computer Data Lines
Circuit"), 2005 WK. This corrects the earlier guess that FCMCGW is absent: **it
exists, it is the Front Control Module, and its diagnostic interface simply is
not on CAN-B.**

```
        DLC pin 6 (+) / pin 14 (-)        <- standard OBD-II CAN pins
                  |   D72 / D71   dedicated diagnostic stub
                  v
        +-------------------------+
CAN-B --|  FCM / CGW              |-- CAN-C main (D64/D65)
D54/D55 |  Front Control Module   |
        |  / Central GateWay      |
        +-------------------------+
```

**Nets.** CAN-B is `D54` (-, WHT) / `D55` (+, WHT/ORG). CAN-C main is `D64`
(-, WHT/LT BLU) / `D65` (+, WHT/LT GRN). The stub is `D71` (-, WHT/DK BLU) /
`D72` (+, WHT/LT BLU).

**`D71`/`D72` appear nowhere else in the vehicle.** Across both figures they
touch exactly two things: the Data Link Connector (pins 14 and 6) and the FCM
(pins 6 and 5). That is what makes it a diagnostic stub rather than a bus.

**Dual-bus modules:** the **FCM** (CAN-B + CAN-C main + the stub) and the **SCM**
(CAN-B pins 6/7, CAN-C pins 2/3) -- spotted by the owner from the diagram, and it
is what makes the SCM interesting below.

**CAN-C members:** PCM, TCM, ABS, FDCM (final drive), shift lever assembly, SCM, FCM.

**CAN-B members** (Fig. 8 + Fig. 9): CCN cluster, RADIO, ORC, WCM/SKREEM, SCM,
EOM, A/C heater control, hands-free module, park assist, monitor/DVD media,
rain sensor, inside rearview mirror, sunroof motor, satellite receiver, radio
amplifier, memory seat, heated seat, driver + passenger memory mirrors, occupant
classification. Note how many of these never appear in the diagnostic database --
only six answer KWP.

#### Why our `7E0` probe could never have worked

The gateway routes between the **diagnostic stub** and the two buses. A scan tool
plugs into DLC 6/14, addresses the CGW, and the CGW forwards onto CAN-C and
CAN-B. Our board is spliced onto CAN-B itself -- *behind* the gateway -- so a
`7E0` request goes out on a bus where no PCM exists, and nothing forwards it.
The gateway translates selected *signals* B<->C (which is how the cluster gets
RPM to display) but does not route diagnostic requests inbound from CAN-B.

Likewise `620/504` is silent on CAN-B because the CGW's own diagnostic interface
is on the stub, not on B. The exhaustive `600`..`6FF` scan was still worth doing:
it proves no module on CAN-B answers for the gateway.

#### This makes CAN-C cheap to reach after all

**Tap DLC pins 6 and 14.** That is a connector under the dash on the standard
OBD-II CAN pinout, so an OBD-II pigtail plugs straight in -- no splicing into a
harness anywhere. Behind it sits the gateway, which is exactly what a scan tool
talks to, and through it the whole CAN-C side: PCM 226 PIDs, TCM 84, ABS 82,
FDCM 76, ESM 8, plus 476 DTCs.

Cost is one ~$2 transceiver at 500 kbps. The ESP32-S3's TWAI controller was
deliberately left unused for precisely this, so no architectural change is
needed -- see ARCHITECTURE.md.

**Try the free thing first.** The SCM straddles both buses and eleven of its
PIDs answer `7F 21 78` (responsePending) rather than returning data. A module
fetching something across a bridge is a plausible reason to stall. Implementing
a wait-for-deferred-reply path costs no hardware and may yield CAN-C data through
the SCM. Worth an hour before buying anything.

#### FCMCGW does not exist on this vehicle

**Superseded by the schematic reading above** -- the module exists and is the
Front Control Module; it just does not expose diagnostics on CAN-B. Kept here
because the measurement stands: nothing answers `620/504`, or any other id in
`600`..`6FF`, from the interior bus.

**This costs us the odometer.** `FCMCGW 0x21A3` is the *only* odometer signal in
all 1152 PIDs of the extracted database, and CCN has no equivalent. Options, none
yet tried:

* look for it in the broadcast traffic below and correlate against the dash
* try service `0x22` (readDataByIdentifier) on CCN -- the cluster displays the
  odometer, so it holds the value somewhere
* accept that trip distance has to be integrated from vehicle speed

### PID readings

Reassembled PDUs, i.e. after the ISO-TP PCI byte. Note the modules pad short
frames with `13 00 30` rather than zeros -- ignore anything past the declared
length.

| Module | Local id | Raw PDU | Decoded | Cross-check |
|---|---|---|---|---|
| CCN | `1A87` | `5A 87 02 87 03 05 80 05 10 05 ... 35 31 37 32 30 30 38 41 44` | P/N **5172008AD** | |
| CCN | `2103` | `61 03 43 64 FF 00` | byte3 `64` x0.5 = **50 %** fuel | fuel gauge -- **confirm half a tank**; if it reads a quarter the signal is byte2 `43` = 33.5 % |
| CCN | `213E` engine off | `61 3E 00 00 00 00 00 00 00 00` | RPM **0**, speed **0** | ignition on, not running |
| CCN | `213E` engine running | `61 3E 02 6D ...` / `02 6E` | **621 / 622 rpm** | warm idle |
| CCN | `2140` engine off | `61 40 72 00` | coolant **74 C**, throttle **0 %** | |
| CCN | `2140` engine running | `61 40 6F 00` | coolant **71 C**, throttle **0 %** | temp gauge |
| CCN | `2134` engine off | `61 34 91 7E` | ambient **32.5 C**, voltage **12.6 V** | battery at rest |
| CCN | `2134` engine running | `61 34 93 8B` | ambient **33.5 C**, voltage **13.9 V** | alternator charging |
| CCN | `213F` | `61 3F 00 05 00 00 00 50 00` | byte3 `05`: MIL (bit0) and oil (bit2) requested | correct with ignition on, engine stopped |
| CCN | `2130` | `61 30 00 00 02 00 00 00 00` | TPM indication (bit56) `00`, localisation (bit64) `00` | no TPM warning |
| CCN | `2110` | `61 10 00 68` | parking brake (bit21) `0`; LF door ajar (bit27) reads `1` | **needs checking against the actual door** -- enum polarity unverified |
| CCN | `2135` | `61 35 00` | washer / coolant / brake fluid low: **all 0** | no fluid warnings |
| CCN | `2152` | `61 52 02 00 26 07` | compass direction (bit16) = `02` | compass readout |
| RADIO | `1A87` | `... 35 30 36 34 30 36 35 41 45` | P/N **5064065AE** | |
| ORC | `1A87` | `... 34 36 30 36 39 33 38 41 44` | P/N **4606938AD** | |
| WCM | `1A87` | `... 36 30 35 33 30 31 37 41 55` | P/N **56053017AU** | |
| SCM | `1A87` | `... 36 30 35 30 30 39 37 41 4D` | P/N **6050097AM** | |
| EOM | `1A87` | `... 36 30 35 30 32 34 32 41 47` | P/N **6050242AG** | |
| FCMCGW | any | `NO DATA` | module does not answer | |

#### Extraction bug found: System Voltage scaling

`CCN 2134` System Voltage is generated as `raw x 0.1 + 5.0` (pid_scaling.csv
scaling_id 15). That yields **17.6 V** at rest and **18.9 V** running, both
impossible. Plain `raw x 0.1` gives **12.6 V** at rest and **13.9 V** with the
engine running -- textbook values, and the jump on starting confirms it.

**The `+5.0` offset is wrong.** Fix `kScalings[1]` in `jeep-carputer/core/SignalDb.cpp`
before the ESP32 displays this signal, and treat every other non-zero offset in
that table as unverified until something real cross-checks it.

### Undocumented WCM PIDs -- TYRE PRESSURE IS READABLE

**Found 2026-09-11.** The extracted database contains no per-wheel pressure
signal anywhere in its 1152 PIDs, and `CCN 2130` offers only a lamp state plus
`TPM System Localization Status` = `Non-Localized`, i.e. base TPMS rather than
Premium. That is a *display* limitation, not a measurement one: the WCM knows
every pressure and simply never publishes it to the cluster.

The WCM answers service `21` on a wide undocumented range. Swept `00`..`3F`,
engine just stopped, ambient 33.5 C:

| PID | PDU | Reading |
|---|---|---|
| **`21 3B`** | `61 3B 21 20 21 1E` | **the four road-wheel pressures: 33, 32, 33, 30 psi** |
| `21 20` | `61 20 21 55 01 A3` | sensor 1: 33 psi, 2nd byte 85 (45 C if -40), status `01A3` |
| `21 21` | `61 21 20 59 01 A3` | sensor 2: 32 psi, 89 (49 C) |
| `21 22` | `61 22 21 5B 03 83` | sensor 3: 33 psi, 91 (51 C), status `0383` |
| `21 23` | `61 23 1E 53 01 A3` | sensor 4: 30 psi, 83 (43 C) |
| `21 24` | `61 24 07 4C 01 83` | sensor 5: 7 psi (0.48 bar), 76 (36 C) -- excluded from `21 3B`. Either a genuinely flat spare, or an absent/unpaired 5th sensor |
| `21 10` | `61 10 00 AD 01 AD 00 AB` | three values 173/173/171, unidentified |
| `21 02` | `61 02 81 7E` | `7E` = 12.6 V, matches `CCN 2134` |
| `21 03`-`0B`, `12`, `13` | `7F 21 33` | securityAccessDenied -- data exists, gated |
| `21 30`-`21 34` | `61 3x E0 01 xx xx` | five entries, `E0`-prefixed -- RKE fobs? |

Five sensors matches `C1501`..`C1505` ("Tire Pressure Sensor 1..5") in the WCM
DTC list, and four road wheels plus a spare is the right shape for this vehicle.

**CONFIRMED against the owner's expected pressures.** The vehicle should sit at
**2.1-2.3 bar**. The four bytes convert to **2.28, 2.21, 2.28, 2.07 bar** -- in
range, and the 30 psi corner lands just below it, so that wheel is genuinely a
little soft. Four identical numbers would have proved much less than one
slightly-low outlier does. **Unit: 1 psi per bit, no offset, no scaling.**

#### Full WCM service-21 sweep, raw

Swept `00`..`3F`, engine just stopped, ambient 33.5 C. `7F 21 12` =
requestOutOfRange (id not supported), `7F 21 33` = securityAccessDenied.
Trailing bytes past the declared PCI length are stale-buffer padding -- ignore
them. Unlisted ids in the range returned `7F 21 12`.

```
21 00 -> 7F 21 12                      21 1B -> 61 1B 07 01 01
21 01 -> 61 01 00                      21 1C -> 61 1C 03 01 00
21 02 -> 61 02 81 7E                   21 1D -> 61 1D 6D FF
21 03 -> 7F 21 33   (security)         21 20 -> 61 20 21 55 01 A3
21 04 -> 7F 21 33   (security)         21 21 -> 61 21 20 59 01 A3
21 05 -> 7F 21 33   (security)         21 22 -> 61 22 21 5B 03 83
21 06 -> 7F 21 33   (security)         21 23 -> 61 23 1E 53 01 A3
21 07 -> 7F 21 33   (security)         21 24 -> 61 24 07 4C 01 83
21 08 -> 7F 21 33   (security)         21 25 -> 61 25 00 00
21 09 -> 7F 21 33   (security)         21 26 -> 61 26 00 00
21 0A -> 7F 21 33   (security)         21 27 -> 61 27 00 00
21 0B -> 7F 21 33   (security)         21 28 -> 61 28 00 00
21 0C -> 61 0C 64 00 00                21 29 -> 61 29 00 01
21 0D -> 61 0D 00 00 03 03 00          21 2A -> 61 2A 01 02 03 04 05
21 0E -> 61 0E 00 00 00 00 00 00 00 00 21 2B -> 61 2B 00
21 0F -> 61 0F 03 00                   21 30 -> 61 30 E0 01 49 35
21 10 -> 61 10 00 AD 01 AD 00 AB       21 31 -> 61 31 E0 01 47 1F
21 11 -> 61 11 00 00 00                21 32 -> 61 32 E0 06 8D 48
21 12 -> 7F 21 33   (security)         21 33 -> 61 33 E0 01 46 79
21 13 -> 7F 21 33   (security)         21 34 -> 61 34 E0 01 49 26
21 14 -> 61 14 00 00 25 93 C5          21 35 -> 61 35 1F
21 15 -> 61 15 81 FF 64                21 37 -> 61 37 00
21 16 -> 61 16 02                      21 38 -> 61 38 00
21 17 -> 61 17 03 01 00 00             21 39 -> 61 39 00
21 18 -> 61 18 00 00 00 03 02          21 3A -> 61 3A 00
21 19 -> 61 19 00 01 00 00             21 3B -> 61 3B 21 20 21 1E   <-- PRESSURES
21 1A -> 61 1A 9F                      21 3C -> 61 3C 03 00
```

Notes on the unexplained rows:

* `21 2A` -> `01 02 03 04 05` is a literal 1..5 sequence -- almost certainly the
  sensor *index* table, reinforcing five TPM sensors.
* `21 30`..`21 34` are five `E0`-prefixed 4-byte values. Five again, and a
  different family from `21 20`..`21 24`. Candidate: the actual sensor IDs, with
  `21 20`..`21 24` being their live readings.
* `21 14` -> `00 00 25 93 C5`: bytes `25 93 C5` match broadcast id `014`
  (`25 93 C5 FF 1C 22 43`), so `21 14` mirrors a broadcast frame.
* `21 0E` returns eight zero bytes; `21 10`'s `00 AD 01 AD 00 AB` (173/173/171)
  is still unidentified -- three values, not four or five.
* The whole `21 03`-`0B` / `12` / `13` block is security-gated, so there is more
  behind `0x27 securityAccess` than this firmware will ever ask for.

**Interpretation, and how confident to be.** The pressures are solid: four
plausible psi values, in a plausible spread, in a dedicated PID, with the spare
excluded, matching the owner's expected range. The temperature column is a good hypothesis and not yet proven --
tyres warm from driving with the spare cooler in the boot fits, and the
prediction is that all five converge on ambient once the car sits. The trailing
16-bit field repeats (`01A3` on three sensors) so it is status or a counter, not
a sensor id.

**Still to verify:**

* gauge one wheel, ideally the one reading 30 psi -- is a corner really ~3 psi down?
* re-read after the car has sat several hours: pressures fall slightly, the
  putative temperatures should fall to ambient
* corner order in `21 3B` is unknown. Let one tyre down by a few psi and see
  which byte moves; that is the only way to map bytes to corners
* `21 3C` -> `61 3C 03 00` and `21 0C`..`21 1D` are unexplored

This is the single most valuable find of the session: tyre pressure was the
original motivating feature of the whole project, the vendor database said it
was impossible, and it turns out to be one undocumented PID away.

#### Still to read

| Module | Local id | Expected content |
|---|---|---|
| CCN | `1A88` / `1A90` | VIN -- cross-check against the windscreen |
| all | `04 18 00 FF 00` | DTC lists (below) |

DTC reads, one row per module, all via `04 18 00 FF 00`:

**Run 2026-09-11.** Every reachable module answered and every code resolved
against the generated table. Multi-frame worked unaided -- the adapter's auto
flow control is doing its job on a real bus. Status bits: `0x20` stored,
`0x40` active, `0x80` lamp.

| Module | Raw PDU | Code | Status | Meaning |
|---|---|---|---|---|
| CCN | `58 02 C1 84 20 98 51 20` | `U0184` | stored | Lost Communication with Radio |
| CCN | " | `B1851` | stored | Flipper Glass Switch Stuck |
| WCM | `58 02 A1 02 27 40 77 27` | `B2102` | stored `27` | Ignition Run/Start Input - Circuit High |
| WCM | " | **`C0077`** | stored `27` | **Low Tire Pressure** |
| RADIO | `58 01 C0 19 60` | `U0019` | **active** `60` | CAN B Bus |
| SCM | `58 01 C1 84 20` | `U0184` | stored | Lost Communication with Radio |
| EOM | `58 02 C0 19 20 C1 84 20` | `U0019` | stored | CAN B Bus |
| EOM | " | `U0184` | stored | Lost Communication with Radio |
| ORC | `7F 18 78` only | -- | -- | response pending, deferred reply never arrives |
| FCMCGW | -- | -- | -- | module absent |

#### `C0077` corroborates the tyre-pressure find

The WCM independently stores **Low Tire Pressure**, which is exactly what a spare
at 0.5 bar produces. Two independent paths -- an undocumented data PID and a
documented DTC -- agree, which is worth more than either alone.

#### Most of the rest are ours

`U0184 Lost Communication with Radio` on CCN, SCM and EOM, and `U0019 CAN B Bus`
on RADIO and EOM, are very likely artefacts of this session rather than vehicle
faults: the safe-state thrash (since fixed) was re-initialising the MCP2515
repeatedly on a live bus, the radio locked up, and the power cord was pulled to
recover it. The RADIO entry still reads **active**.

There is no *dedicated* clear command on the AT surface -- deliberately, so no
host-side bug can reach the car's autonomous features. But clearing is just
service `0x14` (a write), and the adapter transmits arbitrary frames from a bare
hex line, so a host can send the clear as raw `03 14 FF 00`. That has since been
done over this gateway: the RADIO codes were cleared on 2026-09-12 (`10 92` to
open a diagnostic session, then `14 FF 00`, no security access needed). After a
clear, re-read to see which, if any, come back. Anything that returns is a real
fault; anything that does not was us.

#### ORC needs a diagnostic session, and that matters for the ESP32

The airbag module answers `7F 18 78` (responsePending) and then never sends the
deferred reply, so it probably wants `0x10 startDiagnosticSession` first. Its
padding bytes were `C1 84 20` -- `U0184` stored, left in its transmit buffer --
so it likely carries the same code as its siblings.

**Design consequence:** `jeep-carputer`'s `pollOne()` treats `KWP_PENDING` as a
failed poll and moves on. That is right for a fast cyclic poll, but any module
that defers will never be read. A DTC sweep needs a wait-for-deferred-reply path
before ORC can be included.

### Undocumented SCM PIDs -- the bridge module

The SCM has **53 DTCs and zero data signals** in the extracted database, the same
situation the WCM was in before it turned out to carry ~45 PIDs. Swept `00`..`3F`
on `6A8/515`, engine off. It is the interesting module because Fig. 8 shows it on
**both** CAN-B (C1 pins 6/7) and CAN-C (C1 pins 2/3).

`7F 21 12` = requestOutOfRange (skipped below). `7F 21 78` = responsePending.
Trailing bytes past the declared PCI length are stale-buffer padding.

```
21 01 -> 61 01 80 24 80 40 00 01 04 02   21 1B -> 61 1B FF FF FF FF
21 02 -> 61 02 00 00 00 00 00 00 00 00   21 20 -> 61 20 00 00 00
21 03 -> 61 03 8F                        21 21 -> 7F 21 78   (pending)
21 04 -> 61 04 10 00 15                  21 24 -> 61 24 03 5C
21 0A -> 61 0A 07                        21 26 -> 61 26 32 35 32 38 39 38 46 8A 16
21 10 -> 61 10 BD AC D6 FB D3            21 2D -> 61 2D 00 00 00 00 00
21 11 -> 61 11 89 D0 D0 D6 D6            21 30 -> 7F 21 78   (pending)
21 12 -> 61 12 AC 4D D4                  21 31 -> 7F 21 78   (pending)
21 13 -> 61 13 35 FF                     21 32 -> 7F 21 78   (pending)
21 15 -> 61 15 32 39 34 35 19 16 23 02 00  21 33 -> 7F 21 78 (pending)
21 16 -> 61 16 20 05 06 21 14 53 42 00   21 34 -> 7F 21 78   (pending)
          05 48 00 05 20 20 03 11 27     21 35 -> 7F 21 78   (pending)
          17 23 09 00 04 41              21 36 -> 7F 21 78   (pending)
21 18 -> 7F 21 78   (pending)            21 37 -> 7F 21 78   (pending)
                                         21 38 -> 7F 21 78   (pending)
                                         21 39 -> 61 39 00 00 00 00 00
```

28 supported ids. Two groups, and the split is the point:

* **Answers immediately** -- static identity data. `21 16` starts `20 05 06 21`,
  which reads as a 2005-06-21 date code, and contains ASCII `SB`. `21 26` is
  ASCII `252898F`, a serial. `21 15` is ASCII `2945` plus binary.
* **Answers `7F 21 78`** -- eleven ids (`18`, `21`, `30`..`38`) that defer and,
  within the adapter's `ATST` window, never deliver. **These are the ones to
  chase.** A module that stalls is a module fetching something it does not have
  to hand, and the SCM's distinguishing feature is that it straddles both buses.

**The cheap experiment, before buying a CAN-C transceiver:** implement a
wait-for-deferred-reply path (send, then keep listening well past `ATST` for the
real answer, as `ATMA` filtered to `515` can already do by hand) and see what
those eleven PIDs return. If any of them carry CAN-C-sourced values, the SCM is
a software-only bridge to the other bus. Note that ORC behaves the same way, so
the mechanism is worth understanding regardless -- see the DTC section.

### Observed broadcast ids (`ATMA`)

Known ids first -- these are the ones the firmware itself acts on
(`lib/Core/JeepCan.h`). Everything below the divider is for whatever else shows up.

**Captured 2026-09-11, ignition on, engine off: 36 distinct ids, ~61 frames/s
through `ATMA`** (the driver's own RX counter saw ~168/s -- `ATMA` at 115200 is
the bottleneck, so this list is complete but the rates are undercounted).

| Id | Sample data | Seen in 4 s | Known / guess |
|---|---|---|---|
| `000` | `85 00 00 00 00 00` | 16 | ignition / power state |
| `006` | `00 22 04 3C 3C 02 58` | 18 | turn signals |
| `09F` | `06 10 00 00 FF FF FF 1F` | 3 | **radio mode** -- byte 0 low nibble `6` = AUX, confirmed |
| `394` | `17 03 E2 00 00 00` | 3 | **radio panel buttons** -- byte 3 bitmask, confirmed |
| `3DD` | -- | -- | VES presence: **ours**, and the radio accepts it |
| `002` | `00 00 00 00 00 00 00 00` | 33 | fastest id on the bus |
| `01B` | `01 4D 34 35 59 35 38 38` | 18 | ASCII `M45Y588` -- VIN fragment? |
| `015` | `94 7E 10 FF 00 00` | 18 | |
| `014` | `25 93 C5 FF 1C 22 43` | 16 | |
| `1A7` | `00 00 00 00 00 00` | 16 | |
| `2C0` | `FF FF 17 0B 7E FF 17 02` | 17 | |
| `3F8` | `04 00 00 00 00 00` | 17 | |
| `11D` | `00 00 00` | 14 | |
| `003` | `00 05 80 00 50 FF 00` | 4 | |
| `02C` | `00 75 00 64 00 00` | 4 | |
| `1C8` | `00` | 4 | |
| `3A0` | `00 00` | 4 | |
| `1B4` | `02 26 07 00 00 00` | 3 | bytes 1-2 match `CCN 2152` compass `26 07` |
| `1BB` | `00 00 00 00 00 00` | 3 | |
| `3D0` | `0C 0A 0A 0A 0A 0A FF` | 3 | five equal values -- EQ / fader settings? |
| `400` `402` `414` `415` `416` `41C` `41E` | `FD xx 3F FF FF FF FF FF` | 2-3 each | one family, byte 1 differs per id |
| `013` | `88 00 01 49 99` | 2 | |
| `210` | `03 00` | 2 | |
| `1AB` | `03 00 00 00 00 00` | 1 | |
| `1AC` | `02` | 1 | |
| `1AE` | `00 00 00 00 00 00` | 1 | |
| `008` | `00 00 80 80 00 00` | 1 | |
| `012` | `00 30 02 40 00 00` | 1 | |
| `02E` | `1F 00` | 1 | |
| `0D0` | `04 00` | 1 | |
| `0EC` | `01` | 1 | |
| `1B6` | `00 18 FF 18 FF 18 FF` | 1 | repeating `18 FF` triple |

Two leads worth chasing, both aimed at the missing odometer:

* `01B` carries printable ASCII (`M45Y588`) -- if that is a VIN fragment, the
  broadcast traffic carries identity data and may well carry distance too.
* `1B4` bytes 1-2 are `26 07`, exactly the last two bytes of the `CCN 2152`
  compass response. So at least one broadcast id mirrors a KWP PID, which makes
  hunting the odometer in broadcast traffic a reasonable bet: note the dash
  reading, then look for a slowly-incrementing multi-byte counter.

To correlate an unknown id with a real-world action: start `ATMA`, do the thing (indicator
on, door open, wiper, window, button press), stop, and see which id changed. Note the
action next to the guess -- an id with a known trigger is worth more than an id with a
plausible name.
