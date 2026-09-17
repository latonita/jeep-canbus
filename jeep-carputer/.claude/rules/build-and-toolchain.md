---
paths:
  - "platformio.ini"
  - "CMakeLists.txt"
  - "src/CMakeLists.txt"
  - "src/idf_component.yml"
  - "dependencies.lock"
  - "sdkconfig*"
  - "scripts/*.ps1"
  - "scripts/run-host-tests.sh"
---

# Build system, ESP-IDF and sdkconfig

**`lib_ldf_mode = off` is load-bearing.** Every dependency comes from
`src/idf_component.yml` into `managed_components/` — LVGL **9.5**,
esp_lvgl_port, esp_lcd_touch_cst328. PlatformIO's Library Dependency Finder
knows nothing about that and also scans `~/.platformio/lib`; this machine has an
LVGL **8** there, so with the LDF on it compiles v8 alongside and dies on
`lv_btnmatrix_ctrl_t` and a wall of v8-only symbols.

**ESP-IDF moved silently with the machine.** `dependencies.lock` went 5.3.2 →
5.5.4 because the Windows PlatformIO ships a newer `framework-espidf` than the
old WSL install. LVGL did not move (same 9.5.0, same component hash). Suspect
the IDF minor first if something that used to work stops.

**`src/CMakeLists.txt` globs.** A NEW file in `core/` or `src/` needs
`touch src/CMakeLists.txt` or it is not compiled — the symptom is a confusing
undefined reference.

Touching a header = 8–12 min full ESP-IDF rebuild. A `.cpp` is ~60 s.

## sdkconfig settings that are not free

- **`CONFIG_LV_ATTRIBUTE_FAST_MEM_USE_IRAM=y`** and
  **`CONFIG_ESPTOOLPY_FLASHMODE_QIO=y`** are most of the render win (full-screen
  frame 65 → 29 ms). LVGL's blend/mask inner loops were executing XIP from DIO
  flash at ~20 MB/s alongside 69 KB of 4 bpp glyph `.rodata` re-streamed every
  frame.
- **`CONFIG_LV_DRAW_SW_DRAW_UNIT_CNT=1`.** LVGL creates draw threads with plain
  `xTaskCreate` — no affinity — at prio 3, under `carTask`'s 5 on core 0, so a
  worker lands on core 0 and the link task's blocking UART reads preempt it.
  `uiInit()` pins the esp_lvgl_port task to core 1, but that is the coordinator;
  with `LV_USE_OS` the pixels are drawn on those threads. This was the 11 ms
  frame-to-frame variance.
- **NOT `CONFIG_ESP32S3_INSTRUCTION_CACHE_32KB`.** It links, it flashes, and the
  board then boot-loops silently — the console is native USB-Serial-JTAG so
  every reset tears the port down before anything prints, and the cache is
  carved from internal SRAM at startup so nothing catches it at link time.
  Symptom: `esptool` can still read the flash ID, but a console capture returns
  zero bytes and the COM port keeps dropping.
- **`CONFIG_LV_MEM_SIZE_KILOBYTES` is 96 and BOTH ends are a hard wall.** At 64
  the six-page tree builds and then cannot allocate a DRAW TASK on top of it:
  `lv_draw_add_task` → `lv_malloc_zeroed` → `LV_ASSERT_MALLOC` → `while(1)`, on
  taskLVGL — white screen, no touch, carTask still decoding, `task_wdt` naming
  `CPU 1: taskLVGL` forever. At 128 the pool (it is `.bss`) starves the two
  38 KB draw buffers in the same internal SRAM and `esp_lvgl_port` logs
  `Not enough memory for LVGL buffer (buf2) allocation!` at boot — which then
  presents as `main` wedged in `uiSplash` → `lv_screen_load_anim`, misleadingly,
  because the display was never created. Check `RAM:` in the build output and
  that boot line before trusting any change here.

## Panic addresses

Decode, do not guess:

```bash
~/.platformio/packages/toolchain-xtensa-esp-elf/bin/xtensa-esp32s3-elf-addr2line \
  -pfiaC -e .pio/build/s3/firmware.elf <addr>
```

## Gateway (ATmega328)

ICSP via Arduino-as-ISP on COM19; the radio has to come out. `not in sync:
resp=0x00` on the first avrdude try is the programmer resetting — run it again.
The 328 has no bootloader on these pins, which is why opening the serial port
does not reset it (see the link rules for what that costs `begin()`).
