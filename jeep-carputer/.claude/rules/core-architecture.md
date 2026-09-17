---
paths:
  - "core/**/*"
  - "test/**/*"
  - "src/main.cpp"
---

# The `core/` layer: boundaries, ownership, publication

`core/` is pure C++ with no ESP-IDF, host-tested by `scripts/run-host-tests.sh`.
Both the arithmetic AND the **policy** (the decision about when to call it) live
here — the sequencing is where every recent defect actually lived, so leaving it
in `main.cpp` left the wrong half untested.

## The publication boundary — `core/Snapshot.h`

**The renderer reads ONE structure, and its own private copy of it.** This
replaced five live pointers, each handing the UI task a structure another task
kept mutating. What was actually wrong with that:

- **Torn VALUES were never the risk** — every field is ≤4 bytes and naturally
  aligned, so on Xtensa a half-written number cannot be read.
- **A torn STRUCTURE was.** `faultsEndScan()` insertion-sorts the fault list
  while the drill-down walks it: a row drawn twice or skipped. Not a wrong
  number, a wrong LIST, on the page whose job is saying what is wrong.
- **Nothing agreed about "now"** — each readout sampled the clock in its own
  refresh function, so two numbers in one frame could be judged against
  different instants.
- **Staleness was reimplemented five times**, in a translation unit bound to
  LVGL and therefore impossible to host-test.

`snapshotBuild()` resolves one frame's truth in `core/`, `main.cpp` publishes it
under a spinlock (`publishSnapshot`, from carTask), the UI pulls a copy once per
refresh (`readSnapshot`, wired by `uiSetSnapshotReader`). 504 bytes a copy,
three copies, 1540 bytes of `.bss` — measured with `nm -S` on the elf.

- **`SnapAge` has THREE states and the third is the point.** `SNAP_ABSENT` →
  dashes (the module said it has no value). `SNAP_HELD` → the number, dimmed (a
  gap in a poll cycle is normal; a held reading must never be the most confident
  thing on the panel). `SNAP_FRESH` → the number plainly. The renderer no longer
  decides which — it arrives decided, so two pages cannot disagree.
- **A spinlock, not a mutex**, because neither side may sleep: the writer has to
  get back to the UART, the reader is the LVGL task at the top of a frame.
  `publishSnapshot` builds into a staging copy OUTSIDE the lock and copies in,
  so the lock covers a memcpy rather than the whole resolve.
- **`gSnapLock` protects `gSnapLive` and nothing else.** Do not reach for it to
  guard `gBcast` or `gFaults`: those keep their single-writer discipline, and a
  lock around a structure written unlocked buys nothing while looking like it
  did.
- **`TripSet` / `AccelSet` / `FaultSet` are copied WHOLESALE**, not re-modelled
  field by field — the copy is what buys the consistency. `Ui.cpp`'s `s_trips` /
  `s_accel` / `s_faults` are `const` aliases INTO the private copy.
- **The renderer no longer includes `VehicleState.h`** — the boundary enforced
  by the compiler rather than a comment. Slot resolution (`indexOfName`) lives
  in `main.cpp` where the signal names already are; `uiInit()` takes no args.
- **Presentation arithmetic lives here too**: psi → bar, the L/h ↔ L/100km
  hysteresis and its latch, the low-corner test, the flow bar's segment count,
  broadcast-beats-polled. Constants `UI_PSI_TO_BAR`, `UI_TYRE_LOW_BAR`,
  `UI_TYRE_TTL_MS`, `UI_SPARE_TTL_MS`, the five `UI_FLOW_*` are in
  `core/Snapshot.h`, not `Ui.cpp` — there is exactly ONE definition of each.
  (`UI_FLOW_SEGS` is still read in `Ui.cpp` for the widget array and bar
  geometry, which is a layout fact.)
- **A HELD broadcast beats a FRESH poll**, deliberately: two readings of the
  same quantity, so switching source mid-glance would change the number for a
  reason that has nothing to do with the car. Only an ABSENT broadcast hands
  over.
- **`UI_TYRE_TTL_MS` / `UI_SPARE_TTL_MS` are twice the tyre poll interval and
  must move with it.** `test_snapshot` asserts a corner 90 s old still reads
  FRESH — the coupling that used to be a comment.

## Policy modules

- **`core/FaultScan.h`** owns the DTC walk's timing and gating: the wrap-safe
  deadline, the self-start gate, whether a RE-READ mid-walk restarts or waits,
  and whether a walk cut short by the link dropping is owed immediately or in
  five minutes. `faultScanStep()` in `main.cpp` is I/O around
  `faultScanDecide()`. **`FSCAN_BEGIN` deliberately does not consume the pass.**
- **`core/LinkHealth.h`** owns link liveness: quiet window, probe spacing, and
  the bench case (a successful probe on a silent bus means UP — a bench with no
  car produces no frames and must not read as a dead adapter). `linkWatchdog()`
  keeps only what cannot be pure: sending `AT@ST`, and calling `begin()`.
- **`core/PollDecode.h` owns the signal FAN-OUT.** A `21 xx` reply is not one
  value — the WCM's `21 3B` carries four tyre pressures — so a poll is "ask for
  a PID, then publish every signal defined against it". **The rule worth testing
  is the TWO TABLES**: `kSignals` is generated, `kTpmsSignals` is hand-written
  (the per-wheel PID is undocumented and a generator can only emit what its
  source holds). A fan-out walking one and not the other takes out the hero
  feature SILENTLY — request succeeds, response parses, poll reports success,
  tyre page stays dashed.
- **`core/BusCycle.h` owns the listen decision.** `BUS_STREAM_PACE_MS` is a
  SAMPLING RATE, not a throttle: `tripUpdate()` integrates rate × interval and
  can only see the broadcast stamps present when it looks, so the loop must come
  round faster than the quickest signal it must not miss — 20 ms against 0x002's
  50 ms. Slow it and the trips lose fuel silently, with the number still
  plausible. Better (gateway) stamps do not excuse a slower sampler.
  `test_buscycle` also asserts `BUS_MONITOR_WINDOW_MS` + `BUS_POLLS_PER_PASS` ×
  400 ms against `LINK_QUIET_MS`, so moving any of the three notices the others.
- **`faultsFind(set, ecu, code)` is in `core/Faults`** because the mute path and
  the clear path had each grown their own copy of the same lookup. Requests
  travel as (ecu, code) and never as a list index: the list is re-sorted at the
  end of every scan and the renderer reads a snapshot that can be a pass behind.

## `main.cpp` obligations

- **The backlight latch is PER TASK.** `applyBacklight()` is called from carTask
  AND from the link reader (deliberately — deferring it cost up to two seconds
  of lag on a control the driver is actively turning). One shared function-local
  `static uint16_t applied` is a lost update: carTask writes `old`, the
  higher-priority reader writes `new`, carTask resumes and writes `old` back. It
  self-heals because carTask recomputes from `gBcast` every pass, but a
  correctness argument resting on "something else fixes it shortly" stops being
  true when the loop changes. One latch each — no lock, and in particular no
  LEDC call inside a critical section. The log moved to carTask only:
  USB-JTAG blocks until the host drains, and there is ~1.2 s of `At328Link`
  buffer before the 328 back-pressures and the MCP2515 drops frames uncounted.
- **Probe spacing must not be a zero-initialised static.** `now - lastProbe <
  LINK_PROBE_MS` was true for the first three seconds of uptime, so a missing
  adapter could not be reported in that window — a grace period that was an
  accident, and one that comes back round at the far end of the 49-day clock.
  `everProbed` is what replaced it.
- **`xTaskCreatePinnedToCore(carTask, ...)` return is CHECKED.** A failed
  creation is the only silent failure in `main.cpp`: no task, so no `begin()`,
  so neither of the two logs anybody debugging a dead link goes looking for.
  `car task up` at the top of `carTask` is what tells "never started" apart from
  "stuck in begin()" — they look identical otherwise, and both look like a
  missing gateway.
- **Accel is fed from `onBroadcastFrame`, not carTask** — the one derived value
  that is. carTask samples `gBcast` around a loop containing a 700 ms monitor
  window, and a launch timed off that would start its clock up to 700 ms early:
  a fifth of a brisk 0-60, all of it flattering. Safe because it keeps ONE
  writer; the trips cannot move there because carTask also resets and commits
  them.
- **`faultScanStep()` and `tripHousekeeping()` run on BOTH sides of the
  link-retry loop.** Called only past the loop, a tap on FAULTS with the link
  down sits unconsumed until the link happens to return — a dead control. With
  no link `faultScanStep()` marks each module unreachable *without transacting*,
  so the walk costs nothing instead of twelve 800 ms timeouts inside the loop
  that is trying to reconnect.
- **`faultStoreLoadMutes()` must run AFTER `tripStoreInit()`**, which is what
  brings NVS up. Called earlier it returns an empty list, which looks exactly
  like a driver who never muted anything.
