// Replay scripts/playback.log through core/ on the host, the way carputer's
// onBroadcastFrame() does, and count what the acceleration timer makes of it.
//
// This is the only way to check the launch logic without a car, and it is what
// measured ACCEL_LAUNCH_MS: on this capture -- 126183 frames, 841 s of real
// driving -- rest-then-motion alone STARTED 14 runs and the kickdown gate
// starts 0, while the number that reaches a split in time is 0 either way. So
// the gate costs no real figure and removes fourteen false stopwatches.
//
// The capture has no timestamps, so time is synthesised from the measured
// 151 frames/s. Peak values below double as a check that it is faithful:
// 82.9 km/h, 4163 rpm, pedal 0xEA, 8089 pedal frames.
//
//   clang++ -std=c++17 -O1 -I core scripts/replay-accel.cpp core/Broadcast.cpp //       core/Accel.cpp -o replay-accel && ./replay-accel scripts/playback.log
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "Accel.h"
#include "Broadcast.h"

static int hexv(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

int main(int argc, char** argv) {
  const char* path = (argc > 1) ? argv[1] : "scripts/playback.log";
  FILE* f = fopen(path, "rb");
  if (f == NULL) {
    fprintf(stderr, "cannot open %s\n", path);
    return 1;
  }

  BroadcastState bc;
  broadcastClear(bc);
  AccelSet ac;
  accelInit(ac);

  char line[256];
  unsigned long frames = 0, decoded = 0, pedalFrames = 0, pedalStale = 0;
  unsigned runsStarted = 0, runsLaunched = 0, runsPublished = 0;
  unsigned lastHist = 0;
  bool wasRunning = false, wasLaunched = false;
  uint8_t maxPedal = 0;
  float maxKph = 0.0f;
  unsigned rpmMax = 0;

  while (fgets(line, sizeof(line), f) != NULL) {
    if (line[0] != 't') continue;
    size_t n = strlen(line);
    while (n > 0 && (line[n - 1] == '\n' || line[n - 1] == '\r')) line[--n] = 0;
    if (n < 5) continue;

    CanFrame fr;
    int a = hexv(line[1]), b = hexv(line[2]), c = hexv(line[3]);
    int len = hexv(line[4]);
    if (a < 0 || b < 0 || c < 0 || len < 0 || len > 8) continue;
    fr.id = (uint16_t)((a << 8) | (b << 4) | c);
    fr.len = (uint8_t)len;
    if (n < (size_t)(5 + 2 * len)) continue;
    for (int i = 0; i < len; ++i) {
      int hi = hexv(line[5 + 2 * i]), lo = hexv(line[6 + 2 * i]);
      if (hi < 0 || lo < 0) { hi = 0; lo = 0; }
      fr.data[i] = (uint8_t)((hi << 4) | lo);
    }

    // 151 frames/s, measured on the bus.
    const uint32_t t = (uint32_t)((double)frames * 1000.0 / 151.0);
    ++frames;

    if (broadcastApply(bc, fr, t)) ++decoded;
    if (fr.id == 0x2C0u) ++pedalFrames;

    // main.cpp's onBroadcastFrame order, including the freshness gate.
    accelUpdate(ac, bc.speedKph, bc.rpm, bc.speedMs);
    const bool pedalLive =
        bc.pedalMs != 0u && (uint32_t)(t - bc.pedalMs) < BC_PEDAL_TTL_MS;
    if (fr.id == 0x2C0u && !pedalLive) ++pedalStale;
    accelPedal(ac, pedalLive && bc.pedalRaw >= BC_PEDAL_KICKDOWN);
    accelTick(ac, t);

    const bool running = accelRunning(ac);
    const bool launched = accelLaunched(ac);
    if (running && !wasRunning) ++runsStarted;
    if (launched && !wasLaunched) ++runsLaunched;
    wasRunning = running;
    wasLaunched = launched;
    if (ac.histCount != lastHist) {
      lastHist = ac.histCount;
      ++runsPublished;
    }
    if (bc.pedalRaw > maxPedal) maxPedal = bc.pedalRaw;
    if (bc.speedKph > maxKph) maxKph = bc.speedKph;
    if (bc.rpm > rpmMax) rpmMax = bc.rpm;
  }
  fclose(f);

  printf("frames=%lu decoded=%lu  0x2C0=%lu (stale at arrival %lu)\n", frames,
         decoded, pedalFrames, pedalStale);
  printf("peak %.1f km/h  %u rpm  pedal 0x%02X\n", (double)maxKph, rpmMax,
         maxPedal);
  printf("runs STARTED (old trigger: rest -> motion) = %u\n", runsStarted);
  printf("runs LAUNCHED (new trigger: + kickdown)    = %u\n", runsLaunched);
  printf("runs PUBLISHED (reached a split in time)   = %u\n", runsPublished);
  return 0;
}
