# jeep-carputer

ESP32-S3 in-car computer for a 2005 Jeep Grand Cherokee WH. Reads vehicle data over KWP2000
and shows it on a touch screen.

It has **no CAN transceiver**. The only path to the vehicle is the ten-year-old ATmega328
board inside the radio ([`../jeep-aux-and-can-gateway`](../jeep-aux-and-can-gateway)), reached
over UART with an ELM327-style AT command set. That board is a dumb frame pipe and its own
VES/AUX and Bluetooth behaviour is autonomous — nothing here can disturb the car's audio.

See [`../ARCHITECTURE.md`](../ARCHITECTURE.md) for the whole-system picture.

## State

| | |
|---|---|
| `core/` protocol + decode + state + scheduling | **written, 86,300 host assertions passing across 17 suites** |
| `src/At328Link.cpp` (the UART link) | written, **never run against hardware** |
| Display / LVGL UI | not started, and to be designed before it is built |
| LoRa, GPS | not started — and LoRa does not fit this board, see ARCHITECTURE.md |

Nothing here has talked to a car. The AT session settings, the flow-control timing and every
decoded value are unverified against the real bus.

## Build

Under WSL, where PlatformIO and the ESP-IDF toolchain live:

```sh
wsl -- bash -lc 'cd /mnt/d/Repos/car/jeep-canbus/jeep-carputer && pio run -e s3'
```

ESP-IDF 5.3.2, no Arduino libraries. First build takes several minutes on `/mnt/d`;
incremental builds are quick.

## Host tests

`core/` is deliberately plain C++ with **zero ESP-IDF dependencies**, so all the fiddly logic
runs on a laptop instead of requiring a flash-and-drive cycle:

```sh
sh scripts/run-host-tests.sh            # all suites
sh scripts/run-host-tests.sh isotp      # one, by substring
```

Needs clang or gcc; honours `CC`/`CXX`. No framework to install — `test/doctest_min.h` is a
30-line assertion header, and each `test/test_*/` directory is one `main()`.

Keep the zero-dependency rule: no `esp_log`, no `esp_err_t`, no IDF headers anywhere in
`core/`. IDF-dependent code belongs in `src/`.

## Regenerating the signal tables

`core/SignalDb.cpp` is generated from a decrypted vendor diagnostic database and **committed as
source** — deliberately not a build step, so the firmware builds without that data present and
changes show up in a diff.

```sh
python tools/gen_signals.py D:/Repos/car/extracted
```

Output is byte-identical across runs. 29 curated signals, 1152 DTC descriptions, 12 modules.

## Things that will bite you

**Bit conventions.** Bytes big-endian, bits **LSB-first within a byte**, and `bitOffset` counts
from byte 0 of the *full* response including the service byte and the identifier echo. Confirmed
against real captures: the two `1A87` part-number fields decode to ASCII matching `ecus.csv`
exactly. Get this wrong and every gauge is subtly wrong rather than obviously broken.

**Trust the declared length, never the frame content.** A recorded WCM DTC reply declares 8 bytes
but its second frame carries `02 27 50 20 00 07 35` — only `02 27` is real and the rest is *stale
live data from the previous exchange*, not padding. A reassembler that trusts frame content
invents two extra DTC records out of it. There is a dedicated test for this vector.

**After a `7F xx 78` "response pending", re-READ, do not re-SEND.** The module's real answer is
already on its way; sending the request again leaves a stale reply sitting in the adapter's
buffer that the next poll will mis-attribute.

**`IsoTpClient::transact` has no clock**, because `ICarLink` exposes none. It spends the caller's
whole timeout on the first read — where the latency actually is, up to ~140 ms when a pending
intervenes — then allows 150 ms (ISO 15765-2 N_Cr) per follow-on frame under a hard call budget.
If a true deadline is ever needed, `ICarLink` needs a `nowMs()`.

**The link session is `AT@SML0 AT@SM0` / `ATZ ATE0 ATL0 ATS0 ATH1 ATCAF0` / `AT@SML2`.** The two
silencing commands come FIRST, before `ATZ`, because the 328 is not reset by us opening the port:
it may still be streaming from a previous session in a format this one has not agreed to, and
every such line would be misclassified into the eight-slot reply queue that `ATZ`'s own answer
has to arrive in.

**`ATS0`'s role has SHRUNK to the request/response path.** It is what makes a RESPONSE line
exactly three id nibbles plus 2n data nibbles, and that is now all it governs: the background
stream is LAWICEL under `AT@SML2`, which ignores `ATS` and `ATL` entirely and carries its own
`t` marker, DLC nibble and timestamp. `ATCAF0` means we build the ISO-TP PCI byte ourselves.
`ATZ` resets the AT layer only and never re-initialises the adapter's CAN controller, so it cannot
stall the car's VES announce.

**The ROM bootloader prints on IO43 at every reset.** The 328 sees noise whenever this board
restarts; it answers `?` and moves on. `At328Link::begin()` drains before handshaking.

## Folders

```
core/       pure C++, no ESP-IDF: link interface, the two frame-line parsers,
            StreamClock, ISO-TP, KWP, signal tables, decoder, vehicle-state hub,
            poll scheduler, the derived values and the snapshot
src/        ESP-IDF side: Board.h (pin map), At328Link (UART), Display, Touch,
            Ui, the NVS stores, main.cpp
design/     .dc.html artboards at true size, canvas.json, build.py
test/       host test suites, one directory each, auto-discovered by the runner
fixtures/   real recorded frames and DTC pairs, extracted from captures
tools/      gen_signals.py — the table generator
scripts/    run-host-tests.sh, emulate-car.py (the bench emulator: the 328 AND
            the modules behind it), emu-verify.cpp (checks that emulator against
            the real core/), replay-accel.cpp (a whole drive through core/ on the
            laptop), playback.log, flash.ps1
```

## Desk testing without a car

```sh
python scripts/emulate-car.py --port COM5                  # polled pages only
python scripts/emulate-car.py --port COM5 --log playback --loop
python scripts/emulate-car.py --port COM5 --log drive      # the owner's real drive
```

`--log` takes a **name** out of the `LOGS` table in `emulate-car.py` or any path.
`playback` is the 2015 capture in `scripts/`; `drive` is a real 25-minute timestamped
drive (224,709 frames, 1484.9 s, 48 CAN ids) kept outside the repo in
`../../jeep-can-b-logs/` — too big to vendor, and it is the one that exercises the
rollover-vs-restart guard, since the driver cranked the engine mid-capture and the
gateway's stamp counter restarted.

## Licence

MIT, as the rest of the repo — see [`../LICENSE`](../LICENSE).
