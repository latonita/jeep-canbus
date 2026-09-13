#pragma once
// Static description of the modules and signals we read.
//
// Generated once from the decrypted vendor database in D:\Repos\car\extracted
// and committed as source (tools/gen_signals.py). Deliberately NOT a build-time
// dependency: this project does not require that data to be present, and the
// generated tables are reviewable in a diff.
//
// PURE: no ESP-IDF, no Arduino. All tables are `const` and live in flash.
#include <stdint.h>

// Modules reachable on the interior CAN-B bus. The two CAN-C entries are
// present so the reachability probe can address them; whether the gateway
// forwards a request arriving from the CAN-B side is untested.
enum EcuId : uint8_t {
  ECU_CCN = 0,     // instrument cluster        6A0 / 514
  ECU_FCMCGW,      // central gateway           620 / 504
  ECU_RADIO,       // radio                     6B0 / 516
  ECU_ORC,         // occupant restraint        6E0 / 51C
  ECU_WCM,         // wireless control module   600 / 500
  ECU_SCM,         // steering column           6A8 / 515
  ECU_EOM,         // overhead console          6F0 / 51E
  ECU_PCM,         // engine  -- CAN-C, may be unreachable  7E0 / 7E8
  ECU_TCM,         // transmission -- CAN-C                 7E1 / 7E9
  ECU_ABS,         // brakes -- CAN-C                       784 / 785
  ECU_FDCM,        // final drive / AWD -- CAN-C            7B6 / 7B7
  ECU_ESM,         // electronic shifter -- CAN-C           788 / 789
  ECU_COUNT,
};

// Append new modules at the END, before ECU_COUNT. These values are baked into
// the generated tables in SignalDb.cpp, and kDtcs is sorted by (ecu, code) so
// dtcLookup() can binary-search it -- renumbering an existing entry silently
// breaks that ordering.

struct EcuDef {
  uint16_t    requestId;
  uint16_t    responseId;
  const char* name;
  bool        interiorBus;   // false = CAN-C, reachability unproven
};

extern const EcuDef kEcus[ECU_COUNT];

// How a raw field turns into something displayable.
enum RenderType : uint8_t {
  RENDER_SCALED = 0,   // raw * factor + offset, with a unit
  RENDER_ENUM,         // look the raw value up in an enum group
  RENDER_ASCII,        // a text field (VIN, part number)
  RENDER_RAW,          // show the raw number as-is
};

struct ScalingDef {
  float       factor;
  float       offset;
  const char* unit;      // never NULL; "" when unitless
  uint8_t     decimals;
};

struct EnumEntry {
  uint16_t    group;
  uint32_t    from;
  uint32_t    to;
  const char* label;
};

// One readable field inside one module's response to one local identifier.
//
// Bit conventions, established from real captures and NOT negotiable:
//   * bytes are big-endian
//   * bits within a byte are LSB-first
//   * bitOffset is counted from byte 0 of the FULL response, service byte and
//     local identifier echo included
struct SignalDef {
  uint8_t     ecu;          // EcuId
  uint16_t    localId;      // e.g. 0x213E
  uint16_t    bitOffset;
  uint8_t     bitLength;
  uint8_t     render;       // RenderType
  int8_t      scaling;      // INDEX into kScalings, -1 = none
  int16_t     enumGroup;    // vendor enum group TAG, not an index; -1 = none.
                            // Deliberately not compacted to 0..n: a stray
                            // kEnums[enumGroup] then goes out of bounds loudly
                            // instead of quietly returning the wrong label.
                            // Look groups up by scanning EnumEntry::group.
  const char* name;
};

extern const ScalingDef kScalings[];
extern const uint8_t    kScalingCount;
extern const EnumEntry  kEnums[];
extern const uint16_t   kEnumCount;
extern const SignalDef  kSignals[];
extern const uint16_t   kSignalCount;

// Known DTC descriptions, sorted by (ecu, code) so lookup can binary-search.
struct DtcDef {
  uint8_t     ecu;
  uint16_t    code;
  const char* sae;
  const char* text;
};

extern const DtcDef  kDtcs[];
extern const uint16_t kDtcCount;

// NULL when the code is not in the table -- render it from kwpDtcToSae instead.
const DtcDef* dtcLookup(uint8_t ecu, uint16_t code);

// Find a signal by name; NULL when absent. Linear, used only at setup.
const SignalDef* signalByName(const char* name);
