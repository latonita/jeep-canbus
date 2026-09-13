#pragma once
// GENERATED FIXTURE -- do not hand-edit.
//
// The real all-module DTC scan of this vehicle, from
// D:\Repos\car\extracted\live_dtc_scan.csv: 16 fault records over 10 modules
// (the three modules that reported zero faults carry no record rows and are not
// listed here). Copied in full so this project needs nothing outside its tree.
//
// `statusText` is the source CSV's own classification of `status`; it is what
// pins down which bit of the status byte means "currently failing".
#include <stdint.h>

struct DtcScanRecord {
  const char* module;
  uint16_t    reqId;
  uint8_t     countByte;    // the module's own record count for that reply
  uint8_t     recordNo;     // 1-based position inside that reply
  uint16_t    code;
  const char* sae;
  uint8_t     status;
  const char* statusText;   // "stored" | "active"
};

#define DTC_SCAN_RECORD_COUNT 16u

static const DtcScanRecord kDtcScanRecords[DTC_SCAN_RECORD_COUNT] = {
    { "ABS", 0x784u, 3u, 1u, 0xC126u, "U0126", 0x20u, "stored" },
    { "ABS", 0x784u, 3u, 2u, 0xC114u, "U0114", 0x20u, "stored" },
    { "ABS", 0x784u, 3u, 3u, 0xD504u, "U1504", 0x20u, "stored" },
    { "CCN", 0x6A0u, 1u, 1u, 0x9851u, "B1851", 0x20u, "stored" },
    { "ESM", 0x788u, 1u, 1u, 0x0607u, "P0607", 0x30u, "stored" },
    { "FCMCGW", 0x620u, 1u, 1u, 0xA100u, "B2100", 0x20u, "stored" },
    { "FDCM", 0x7B6u, 1u, 1u, 0xC415u, "U0415", 0x20u, "stored" },
    { "PCM", 0x7E0u, 7u, 1u, 0x0460u, "P0460", 0x20u, "stored" },
    { "PCM", 0x7E0u, 7u, 2u, 0xC114u, "U0114", 0x20u, "stored" },
    { "PCM", 0x7E0u, 7u, 3u, 0xC103u, "U0103", 0x20u, "stored" },
    { "PCM", 0x7E0u, 7u, 4u, 0xC114u, "U0114", 0x20u, "stored" },
    { "PCM", 0x7E0u, 7u, 5u, 0xC103u, "U0103", 0x20u, "stored" },
    { "PCM", 0x7E0u, 7u, 6u, 0x0676u, "P0676", 0x60u, "active" },
    { "PCM", 0x7E0u, 7u, 7u, 0x213Au, "P213A", 0xE0u, "active" },
    { "WCM", 0x600u, 2u, 1u, 0x4077u, "C0077", 0x27u, "stored" },
    { "WCM", 0x600u, 2u, 2u, 0xA102u, "B2102", 0x27u, "stored" },
};
