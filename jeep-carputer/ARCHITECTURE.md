# App architecture

A compact view of the current implementation, based on the source as of 2026-09-17.
The app is an ESP32-S3 dashboard: it receives vehicle data through the radio's
ATmega328 gateway, calculates driving information, and displays it with LVGL.

The four diagrams below cover **where it runs**, **what it contains**, **how data
moves**, and **how it runs over time**. Mermaid diagrams render directly on GitHub
and in Markdown viewers with Mermaid support.

## 1. System context

The ESP32 has no direct CAN connection. The external gateway carries both vehicle
broadcasts and diagnostic requests over one UART link.

```mermaid
flowchart LR
    car["Jeep modules<br/>Interior CAN-B"]
    gateway["ATmega328 gateway<br/>Inside the radio<br/>Separate firmware"]
    app["ESP32-S3 carputer<br/>This application"]
    screen["Touch display<br/>LVGL dashboard"]
    driver["Driver"]
    bench["PC car emulator<br/>Optional bench substitute"]

    car <-->|"CAN frames"| gateway
    gateway <-->|"UART 115200<br/>AT commands and frames"| app
    app -->|"SPI: pixels"| screen
    screen -->|"I2C: touch"| app
    driver <-->|"Read and interact"| screen
    bench <-.->|"Replaces gateway and car over UART"| app
```

The gateway's audio/Bluetooth responsibilities remain in its own firmware. Vehicle
protocol handling and interpretation happen in this app.

## 2. Component diagram

`src/main.cpp` wires the system together. `core/` holds portable C++ logic;
`src/` supplies ESP-IDF hardware access, tasks, storage, and presentation.
Arrows show calls or supplied data, rather than every include dependency.

```mermaid
flowchart TB
    subgraph platform["src/ — ESP-IDF integration"]
        main["main.cpp<br/>Boot, car loop, action handling"]
        link["At328Link<br/>UART session and receive routing"]
        ui["Ui + UiTheme + fonts<br/>Pages, gestures, rendering"]
        hardware["Display + Touch + Board<br/>Panel, input, pin mapping"]
        stores["Settings / TripStore / FaultStore<br/>NVS persistence"]
    end

    subgraph portable["core/ — portable, host-tested C++"]
        policy["Scheduling and health<br/>PollScheduler, BusCycle, FaultScan, LinkHealth"]
        protocol["Diagnostics<br/>ICarLink, IsoTp, Kwp, PollDecode<br/>Decoder + SignalDb + TpmsSignals"]
        live["Broadcast processing<br/>LinkLine, StreamClock, Broadcast<br/>Buttons, Compass, Accel"]
        state["Vehicle and driving state<br/>VehicleState, Trip, Faults"]
        snap["Snapshot<br/>Resolved readouts and freshness"]
    end

    main --> policy
    main --> protocol
    protocol -->|"ICarLink implementation"| link
    link -->|"Received broadcasts"| live
    protocol --> state
    live --> state
    main -->|"Build and publish"| snap
    state --> snap
    live --> snap
    snap -->|"Private copy for each refresh"| ui
    ui -->|"Vehicle action requests"| main
    ui --> hardware
    main --> stores
    ui -->|"Appearance and preferences"| stores
```

The snapshot is the main vehicle-data boundary: it resolves values, units, warnings,
and whether readings are fresh, held, or absent. The UI owns their visual appearance.
Link status, logs, and page-change notifications also have small direct UI interfaces.

## 3. Data flow: two ways to learn about the car

Broadcasts provide frequent driving data. Scheduled diagnostics fill in information
that broadcasts do not supply, such as tyre pressures and warning flags.

```mermaid
flowchart LR
    wire["UART receive"] --> route["At328Link + LinkLine<br/>Classify incoming lines"]

    route -->|"Broadcast frames"| time["StreamClock<br/>Map gateway timestamps"]
    time --> bc["Broadcast state<br/>Speed, RPM, fuel, heading, etc."]
    bc --> derived["Derived information<br/>Trips and acceleration"]

    request["Car task<br/>Scheduled polls / fault scans"] -->|"KWP over ISO-TP"| tx["At328Link transmit<br/>To gateway and ECU"]
    route -->|"Response frames"| queue["Response queue"]
    queue --> decode["ISO-TP reassembly<br/>KWP parsing and decoding"]
    decode --> values["VehicleState / Faults<br/>Polled values and trouble codes"]

    bc --> snapshot["Snapshot<br/>Resolve values and freshness"]
    derived --> snapshot
    values --> snapshot
    snapshot --> copy["Publish under short lock<br/>UI reads its own copy"]
    copy --> pages["Dashboard pages"]
```

Fresh readings display normally; old readings remain visible but dimmed; unknown or
explicitly unavailable readings become dashes. Trips A/B and fault mute choices are
saved to flash; the “from start” trip begins again at boot.

Broadcast button events also request page changes, and dimmer/light frames control
the backlight. These short paths do not wait for diagnostic polling.

## 4. Runtime sequence and task boundaries

The UART reader and car task run on core 0; UI refresh and LVGL run on core 1.
Only the car task issues vehicle requests. The reader keeps receiving while a
diagnostic transaction waits for a reply.

```mermaid
sequenceDiagram
    participant Boot as app_main
    participant Rx as UART reader (core 0)
    participant Car as Car task (core 0)
    participant Pub as Published snapshot
    participant UI as UI / LVGL (core 1)

    Boot->>Boot: Restore settings, trips, fault mutes
    Boot->>Pub: Publish initial values
    Boot->>UI: Start display and touch
    Boot->>Car: Start car task
    Car->>Rx: Start reader and negotiate gateway session
    Note over Car,Rx: Retry initial connection if unavailable
    Car->>Car: Probe modules and register poll plan

    loop While running (tasks execute independently)
        Rx->>Rx: Receive broadcasts and update broadcast state and acceleration
        Car->>Car: Listen/pace, check link, apply backlight
        Car->>Pub: Build snapshot and copy under lock
        UI->>Pub: Read private copy under lock
        UI->>UI: Refresh dashboard
        opt Driver requests trip reset or fault action
            UI->>Car: Set action request
        end
        Car->>Car: Trip housekeeping and requested actions
        Car->>Car: Advance fault scan and run up to 3 due polls
        Rx-->>Car: Queued diagnostic responses
        Car->>Car: Decode results and update state for next snapshot
    end
```

Normal operation uses continuous background streaming. Older gateways fall back
to alternating monitor windows and requests; if monitoring is unavailable, the
loop can run polls alone. The normal streaming delay is 20 ms, but diagnostic waits
can extend a loop pass. Snapshot publication is protected; building it reads live
inputs outside that lock, so it is not an atomic capture of every producer.

## 5. Data sources and read intervals

All vehicle reads pass through the ATmega328 gateway. Broadcast IDs below identify
the received CAN frames; they do not imply a direct ESP32 connection to the sending
module. Broadcast timing is vehicle-controlled, not an app polling schedule.

### Passive broadcasts

| Data read | CAN frame | Arrival timing documented in code | Used for |
|---|---|---|---|
| Engine RPM and vehicle speed | `0x002` | About 50 ms | Gauges, acceleration timing, consumption |
| Odometer | `0x014` | On receipt; rate not specified | Trip distance |
| Ambient temperature and battery voltage | `0x015` | On receipt; rate not specified | Dashboard / health |
| Engine coolant temperature | `0x02C` | On receipt; rate not specified | Dashboard / health |
| Ignition / key position | `0x000` | On receipt; rate not specified | Trip lifecycle |
| Instantaneous fuel flow | `0x1A7` | About 100 ms | Consumption and accumulated trip fuel |
| Accelerator pedal | `0x2C0` | About 100 ms | Pedal display and acceleration launch detection |
| Fuel tank level | `0x159` | About 1 s | Fuel gauge |
| Compass direction, heading and status | `0x1B4` | About 500 ms | Compass |
| Panel dimmer level | `0x210` | On receipt; rate not specified | Screen brightness |
| Exterior lights state | `0x1C8` | About 500 ms | Day/night brightness behavior |
| Head-unit button bits | `0x394` | Press/release events | Page navigation |

Source: [Broadcast.cpp](../core/Broadcast.cpp) and the broadcast handlers in
[main.cpp](../src/main.cpp). These values are not periodically requested by the app.
In particular, fuel level comes from `0x159`; the older `21 03` fuel poll is no longer
in the active plan.

### Diagnostic requests

CCN is the instrument cluster; WCM is the wireless control module that supplies
tyre pressures. Request bytes below are KWP payloads, before ISO-TP framing.

| Data read | Module / request | When requested |
|---|---|---|
| ABS lamp, charging failure, check-engine (MIL) lamp, oil lamp flags | CCN / `21 3F` | Every **5 s** |
| Washer fluid low, engine coolant low, brake fluid low flags | CCN / `21 35` | Every **10 s** |
| Four road-wheel tyre pressures | WCM / `21 3B` | Every **60 s**, one request for all four |
| Spare tyre pressure | WCM / `21 24` | Every **60 s**, separate request |
| Stored trouble codes and their status | Each configured module / `18 00 FF 00` | First scan after connection, then **5 min after the previous scan finishes**, or driver RE-READ |
| ECU identification and initial trouble-code reads | RADIO, CCN, WCM / `1A 87` and `18 00 FF 00` | Once during startup after the gateway connects |

The four periodic signal reads are the complete active `kPlan` in
[main.cpp](../src/main.cpp). The intervals are measured from request completion;
blocking work can delay the next attempt. [PollScheduler](../core/PollScheduler.cpp)
doubles an item's interval on each consecutive failure, up to **32 times** its
normal interval, and restores the normal interval after success. These are request
intervals, not guarantees that the ECU or tyre sensor has produced a new sample.

The fault scan asks one module per car-loop pass, walking the 12 entries in
[SignalDb.cpp](../core/SignalDb.cpp): CCN, FCMCGW, RADIO, ORC, WCM, SCM, EOM, PCM,
TCM, ABS, FDCM, and ESM. A module that does not answer is marked unreachable rather
than fault-free. Timing policy lives in [FaultScan.cpp](../core/FaultScan.cpp).
Fault clearing is a separate driver-triggered write, not a periodic read.

Gateway health checks are separate from vehicle polling: when the bus becomes quiet,
the app can query adapter status (`AT@ST`), with probes spaced at least **3 s** apart.
It also reads adapter identity/status at connection and recovery.

### Calculated and local data

Trip distance comes from odometer changes; trip fuel comes from integrating fuel
flow. Consumption combines fuel flow and speed, and acceleration times use speed
samples with pedal-based launch detection. These results are calculated locally,
not fetched from another ECU. Fuel price, appearance settings, saved trips A/B, and
fault mute choices come from local settings / NVS storage.

## Where to look next

| Concern | Source |
|---|---|
| Startup, poll plan, task wiring, UI actions | [src/main.cpp](../src/main.cpp) |
| UART reader, routing, gateway compatibility | [src/At328Link.cpp](../src/At328Link.cpp) |
| Display-ready vehicle data | [core/Snapshot.h](../core/Snapshot.h) |
| Pages, refresh, appearance, interaction | [src/Ui.cpp](../src/Ui.cpp) |
| Portable logic checks | [test/](../test/), [host test runner](../scripts/run-host-tests.sh) |
| Bench operation without the car | [scripts/emulate-car.py](../scripts/emulate-car.py) |

This describes the implemented structure, not a claim that every vehicle signal has
been validated on hardware. Older README status text and introductory comments in
`main.cpp` predate the implemented LVGL UI; these diagrams follow the actual code.
