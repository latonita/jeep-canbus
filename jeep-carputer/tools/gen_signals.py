#!/usr/bin/env python3
"""Generate core/SignalDb.cpp from the decrypted vendor diagnostic database.

This is NOT a build step.  It is run by hand, its output is committed, and the
firmware therefore builds without the extracted CSVs being present anywhere.

    python3 tools/gen_signals.py [EXTRACTED_DIR] [-o core/SignalDb.cpp]

EXTRACTED_DIR defaults to ../../../extracted relative to this script
(i.e. D:\\Repos\\car\\extracted for the usual checkout layout).

Output is deterministic: every table is sorted by an explicit key and every
float is formatted with a fixed precision, so re-running on unchanged input
produces a byte-identical file and a real change shows up cleanly in a diff.

Source render_type -> RenderType mapping (see MAP_RENDER below):

    src 0 "scaling/enum table" + enum_group_id != 0 -> RENDER_ENUM
    src 0 "scaling/enum table" + scaling_id   != 0 -> RENDER_SCALED
    src 0 with neither                             -> RENDER_RAW
    src 1 "ASCII"                                  -> RENDER_ASCII
    src 2 "decimal"                                -> RENDER_RAW
    src 3 "hex"                                    -> RENDER_RAW
    src 4 "binary"                                 -> RENDER_RAW

RenderType has no hex/binary/decimal distinction, so source types 2, 3 and 4
all collapse onto RENDER_RAW; the presentation difference is a UI concern, not
a decoding one.  Note that a row carrying an enum_group_id is RENDER_ENUM even
though its render_type is 0 -- in the source, 0 means "look it up in a table"
and the table is either a scaling row or an enum group.

Metric units (met_*) are used throughout; the imp_* columns are ignored.
"""

import argparse
import csv
import os
import sys

# ---------------------------------------------------------------------------
# The modules that EcuId in core/SignalDb.h can name, in EcuId order.  This
# list must stay in the same order as the enum: the ordinal is what lands in
# SignalDef::ecu and DtcDef::ecu, and kDtcs is sorted on it.  Append only.
#
# interiorBus: True = interior CAN-B, False = the 500 kbps CAN-C side, which
# may not be reachable at all through the gateway.
#
# Request/response ids are NOT hard-coded here -- they are read from ecus.csv
# so the vendor data stays the single source of truth.
# ---------------------------------------------------------------------------
ECUS = [
    ("CCN",    "ECU_CCN",    True),
    ("FCMCGW", "ECU_FCMCGW", True),
    ("RADIO",  "ECU_RADIO",  True),
    ("ORC",    "ECU_ORC",    True),
    ("WCM",    "ECU_WCM",    True),
    ("SCM",    "ECU_SCM",    True),
    ("EOM",    "ECU_EOM",    True),
    ("PCM",    "ECU_PCM",    False),
    ("TCM",    "ECU_TCM",    False),
    ("ABS",    "ECU_ABS",    False),
    ("FDCM",   "ECU_FDCM",   False),
    ("ESM",    "ECU_ESM",    False),
]
ECU_INDEX = {m: i for i, (m, _, _) in enumerate(ECUS)}
ECU_ENUM = {m: e for m, e, _ in ECUS}

# ---------------------------------------------------------------------------
# The curated signal set.  Matched on (module, request_bytes_hex, name_en) with
# an exact, case-sensitive name compare -- a near miss is reported, never
# guessed at, because an approximated name would silently bind to the wrong
# bits.  Emitting all 1159 PIDs is cheap in flash but not in review effort, and
# an unused signal is an untested signal.
# ---------------------------------------------------------------------------
CURATED = [
    # CCN -- instrument cluster, 6A0/514
    ("CCN", "213E", "Engine Revolutions Per Minute"),
    ("CCN", "213E", "Vehicle Speed"),
    ("CCN", "2140", "Engine Coolant Temperature"),
    ("CCN", "2140", "Throttle Pedal Position (Calculated)"),
    ("CCN", "2103", "Fuel Level Percentage"),
    ("CCN", "2134", "Ambient Temperature"),
    ("CCN", "2134", "System Voltage"),
    ("CCN", "213F", "MIL Lamp Request"),
    ("CCN", "213F", "Oil Lamp Request"),
    ("CCN", "213F", "Charging System Failure"),
    ("CCN", "213F", "ABS Lamp Request"),
    ("CCN", "2130", "Tire Pressure Indication Request"),
    ("CCN", "2130", "TPM System Localization Status"),
    ("CCN", "2152", "Compass Direction (Calculated)"),
    ("CCN", "2135", "Engine Coolant Low"),
    ("CCN", "2135", "Washer Fluid Low"),
    ("CCN", "2135", "Brake Fluid Low"),
    ("CCN", "2110", "Parking Brake Input"),
    ("CCN", "2110", "Left Front Door Ajar"),
    # FCMCGW -- central gateway, 620/504
    ("FCMCGW", "21A3", "Odometer"),
    ("FCMCGW", "21B0", "Vehicle Speed"),
    ("FCMCGW", "21B1", "Engine RPM"),
    ("FCMCGW", "21B1", "PRND Status"),
    ("FCMCGW", "2110", "Ambient Temperature Degrees"),
    ("FCMCGW", "2110", "Battery Voltage"),
    ("FCMCGW", "2113", "Coolant Level Sensor"),
    ("FCMCGW", "2113", "Brake Fluid Level"),
    ("FCMCGW", "1A90", "VIN - Current"),
    ("FCMCGW", "1A87", "ECU Part Number"),
]

REQUIRED_FILES = ("pids.csv", "pid_scaling.csv", "pid_enum_values.csv",
                  "dtc_codes.csv", "ecus.csv")

MAP_RENDER = {
    "1": "RENDER_ASCII",
    "2": "RENDER_RAW",
    "3": "RENDER_RAW",
    "4": "RENDER_RAW",
}

HEX = "0123456789abcdefABCDEF"


# ---------------------------------------------------------------------------
# emit helpers
# ---------------------------------------------------------------------------
def cstr(s):
    """A C++ string literal for `s`, with non-ASCII emitted as \\xNN bytes.

    A hex escape in C++ is greedy, so "\\xc2\\xb0C" would parse the trailing C
    as part of the escape.  Whenever a hex-digit character follows an escape we
    break the literal in two and let adjacent-literal concatenation rejoin it.
    """
    out = ['"']
    prev_hex_escape = False
    for b in s.encode("utf-8"):
        ch = chr(b)
        if ch == '"':
            out.append('\\"')
            prev_hex_escape = False
        elif ch == "\\":
            out.append("\\\\")
            prev_hex_escape = False
        elif ch == "\n":
            out.append("\\n")
            prev_hex_escape = False
        elif ch == "\t":
            out.append("\\t")
            prev_hex_escape = False
        elif 0x20 <= b < 0x7F:
            if prev_hex_escape and ch in HEX:
                out.append('" "')
            out.append(ch)
            prev_hex_escape = False
        else:
            out.append("\\x%02x" % b)
            prev_hex_escape = True
    out.append('"')
    return "".join(out)


def cfloat(v):
    """A float literal.  %.9g round-trips a float32 exactly and is stable."""
    s = "%.9g" % float(v)
    if "." not in s and "e" not in s and "E" not in s:
        s += ".0"
    return s + "f"


def num(s, default=0.0):
    s = (s or "").strip()
    if not s:
        return default
    return float(s)


# ---------------------------------------------------------------------------
def load(path, name):
    with open(os.path.join(path, name), "r", encoding="utf-8", newline="") as f:
        return list(csv.DictReader(f))


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("extracted", nargs="?",
                    default=os.path.normpath(os.path.join(here, "..", "..", "..", "extracted")),
                    help="directory holding the extracted vendor CSVs")
    ap.add_argument("-o", "--out",
                    default=os.path.join(os.path.dirname(here), "core", "SignalDb.cpp"),
                    help="output .cpp path")
    args = ap.parse_args()

    src = args.extracted
    missing = [n for n in REQUIRED_FILES if not os.path.isfile(os.path.join(src, n))]
    if missing:
        sys.stderr.write(
            "gen_signals.py: cannot find the extracted vendor database.\n"
            "  looked in: %s\n"
            "  missing:   %s\n"
            "\n"
            "This script is not part of the build -- core/SignalDb.cpp is committed.\n"
            "Only run it when regenerating, and pass the directory holding the CSVs:\n"
            "  python3 tools/gen_signals.py D:/Repos/car/extracted\n" % (src, ", ".join(missing)))
        return 2

    pids = load(src, "pids.csv")
    scalings_src = load(src, "pid_scaling.csv")
    enums_src = load(src, "pid_enum_values.csv")
    dtcs_src = load(src, "dtc_codes.csv")
    ecus_src = load(src, "ecus.csv")

    warnings = []

    # ---- ECUs ------------------------------------------------------------
    by_module = {}
    for r in ecus_src:
        by_module[r["module"]] = r
    ecu_rows = []
    for module, _enum, interior in ECUS:
        r = by_module.get(module)
        if r is None:
            sys.stderr.write("gen_signals.py: ecus.csv has no row for %s\n" % module)
            return 3
        ecu_rows.append((module, int(r["req_can_id_hex"], 16),
                         int(r["resp_can_id_hex"], 16), interior))

    # ---- signals ---------------------------------------------------------
    index = {}
    for r in pids:
        index.setdefault((r["module"], r["request_bytes_hex"], r["name_en"]), []).append(r)

    picked, misses = [], []
    for key in CURATED:
        rows = index.get(key)
        if not rows:
            misses.append(key)
            continue
        if len(rows) > 1:
            warnings.append("pids.csv has %d rows for %s %s %r; took the "
                            "lowest row_id" % (len(rows), key[0], key[1], key[2]))
            rows = sorted(rows, key=lambda r: int(r["row_id"]))
        picked.append(rows[0])

    if misses:
        sys.stderr.write("gen_signals.py: %d curated signal(s) not found in pids.csv:\n" % len(misses))
        for m in misses:
            sys.stderr.write("    %-8s %-6s %r\n" % m)

    # Only the scaling rows and enum groups these signals actually reference
    # get emitted; the rest of the vendor tables are pruned.
    used_scalings = sorted({int(r["scaling_id"]) for r in picked
                            if r["scaling_id"] not in ("", "0")})
    used_enums = sorted({int(r["enum_group_id"]) for r in picked
                         if r["enum_group_id"] not in ("", "0")})

    scaling_by_id = {int(r["scaling_id"]): r for r in scalings_src}
    scaling_rows, scaling_slot = [], {}
    for sid in used_scalings:
        r = scaling_by_id.get(sid)
        if r is None:
            sys.stderr.write("gen_signals.py: pid_scaling.csv has no scaling_id %d\n" % sid)
            return 4
        scaling_slot[sid] = len(scaling_rows)
        scaling_rows.append((sid, num(r["met_factor"], 1.0), num(r["met_offset"]),
                             r["met_unit"] or "", int(num(r["met_decimals"]))))

    # kEnums keeps the SOURCE group ids, not compacted indices: EnumEntry
    # carries a `group` field, so a group is found by scanning for a matching
    # tag rather than by indexing.  Keeping the vendor ids makes a row
    # traceable back to pid_enum_values.csv, and makes an accidental
    # kEnums[sig.enumGroup] fail loudly instead of quietly.
    enum_rows = []
    for r in enums_src:
        g = int(r["enum_group_id"])
        if g in used_enums:
            enum_rows.append((g, int(r["raw_value_from"]), int(r["raw_value_to"]),
                              r["label_en"], int(r["row_id"])))
    enum_rows.sort(key=lambda e: (e[0], e[1], e[2], e[4]))
    have_enum_groups = {e[0] for e in enum_rows}
    for g in used_enums:
        if g not in have_enum_groups:
            sys.stderr.write("gen_signals.py: enum group %d referenced but absent "
                             "from pid_enum_values.csv\n" % g)
            return 5

    # Sanity check: overlapping raw ranges inside one emitted group would make
    # the decoded label depend on scan order.
    per_group = {}
    for g, lo, hi, label, _rid in enum_rows:
        per_group.setdefault(g, []).append((lo, hi, label))
    for g, items in sorted(per_group.items()):
        items = sorted(items)
        for a, b in zip(items, items[1:]):
            if a[1] >= b[0]:
                warnings.append("enum group %d: raw ranges overlap, %r [%d..%d] vs "
                                "%r [%d..%d]" % (g, a[2], a[0], a[1], b[2], b[0], b[1]))

    signal_rows = []
    for r in picked:
        module = r["module"]
        sid = int(r["scaling_id"] or 0)
        gid = int(r["enum_group_id"] or 0)
        rt = r["render_type"]
        if rt == "0":
            render = "RENDER_ENUM" if gid else ("RENDER_SCALED" if sid else "RENDER_RAW")
        else:
            render = MAP_RENDER.get(rt)
            if render is None:
                warnings.append("unknown render_type %r for %s %s %r; using RENDER_RAW"
                                % (rt, module, r["request_bytes_hex"], r["name_en"]))
                render = "RENDER_RAW"
            if gid:
                warnings.append("%s %s %r has render_type %s AND enum_group_id %d"
                                % (module, r["request_bytes_hex"], r["name_en"], rt, gid))
        signal_rows.append((
            ECU_INDEX[module],
            ECU_ENUM[module],
            int(r["request_bytes_hex"], 16),
            int(r["bit_offset"]),
            int(r["bit_length"]),
            render,
            scaling_slot[sid] if sid else -1,
            gid if gid else -1,
            "%s.%s" % (module, r["name_en"]),
            sid,
            int(r["row_id"]),
        ))
    signal_rows.sort(key=lambda s: (s[0], s[2], s[3], s[8]))

    # name_en is not unique across modules ("Vehicle Speed" is both a CCN and
    # an FCMCGW signal), so the emitted name is module-qualified.  signalByName
    # also accepts the bare name and returns the first match, in table order.
    bare = {}
    for s in signal_rows:
        bare.setdefault(s[8].split(".", 1)[1], []).append(s[1])
    for name, owners in sorted(bare.items()):
        if len(owners) > 1:
            warnings.append("name_en %r is used by %s; qualified names keep them "
                            "distinct, and signalByName(%r) resolves to the first"
                            % (name, "/".join(owners), name))

    # A field must fit inside a plausible response.  KWP readDataByLocalId
    # answers with 0x61 + the two identifier bytes echoed for 0x21xx (0x5A + one
    # byte for 0x1Axx), and ISO-TP caps a single response at 4095 bytes.
    for s in signal_rows:
        end_byte = (s[3] + s[4] + 7) // 8
        if end_byte > 4095:
            warnings.append("%s: bitOffset %d + bitLength %d ends past the ISO-TP "
                            "4095-byte limit" % (s[8], s[3], s[4]))
        if s[3] < 16:
            warnings.append("%s: bitOffset %d lands in the service/identifier echo"
                            % (s[8], s[3]))

    # ---- DTCs ------------------------------------------------------------
    dtc_rows, dropped = [], {}
    for r in dtcs_src:
        module = r["module"]
        if module not in ECU_INDEX:
            dropped[module] = dropped.get(module, 0) + 1
            continue
        dtc_rows.append((ECU_INDEX[module], int(r["raw_code_hex_2byte"], 16),
                         r["sae_code"], r["description_en"], ECU_ENUM[module]))
    dtc_rows.sort(key=lambda d: (d[0], d[1]))
    for a, b in zip(dtc_rows, dtc_rows[1:]):
        if a[0] == b[0] and a[1] == b[1]:
            warnings.append("duplicate DTC %s 0x%04X (%s / %s); binary search will "
                            "return one of them arbitrarily" % (a[4], a[1], a[2], b[2]))
    for module, n in sorted(dropped.items()):
        warnings.append("dropped %d DTC(s) for module %s: EcuId in core/SignalDb.h "
                        "cannot name it" % (n, module))

    # ---- write -----------------------------------------------------------
    text = render_cpp(ecu_rows, scaling_rows, enum_rows, signal_rows, dtc_rows,
                      os.path.basename(src), dropped)
    out = args.out
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    with open(out, "w", encoding="utf-8", newline="\n") as f:
        f.write(text)

    sys.stderr.write("\ngen_signals.py: wrote %s\n" % out)
    sys.stderr.write("  kEcus      %4d\n" % len(ecu_rows))
    sys.stderr.write("  kScalings  %4d  (of %d in pid_scaling.csv)\n"
                     % (len(scaling_rows), len(scalings_src)))
    sys.stderr.write("  kEnums     %4d  in %d group(s)  (of %d in pid_enum_values.csv)\n"
                     % (len(enum_rows), len(per_group), len(enums_src)))
    sys.stderr.write("  kSignals   %4d  (of %d curated, %d in pids.csv)\n"
                     % (len(signal_rows), len(CURATED), len(pids)))
    sys.stderr.write("  kDtcs      %4d  (of %d in dtc_codes.csv)\n"
                     % (len(dtc_rows), len(dtcs_src)))
    if warnings:
        sys.stderr.write("\n%d note(s) about the source data:\n" % len(warnings))
        for w in warnings:
            sys.stderr.write("  - %s\n" % w)
    return 1 if misses else 0


# ---------------------------------------------------------------------------
def render_cpp(ecu_rows, scaling_rows, enum_rows, signal_rows, dtc_rows, srcname,
               dropped):
    o = []
    w = o.append
    w("// GENERATED FILE -- DO NOT HAND-EDIT.\n")
    w("//\n")
    w("// Generated by tools/gen_signals.py from the decrypted vendor diagnostic\n")
    w("// database for a 2005 Jeep WK (extracted CSVs: pids.csv, pid_scaling.csv,\n")
    w("// pid_enum_values.csv, dtc_codes.csv, ecus.csv, in .../%s).\n" % srcname)
    w("//\n")
    w("// Regenerate with:\n")
    w("//     python3 tools/gen_signals.py D:/Repos/car/extracted\n")
    w("//\n")
    w("// The generator is deliberately NOT a build step: this file is committed, so\n")
    w("// the firmware builds with no vendor data present.  Edits made here are lost\n")
    w("// on the next regeneration -- change tools/gen_signals.py instead.\n")
    w("//\n")
    w("// render_type (vendor) -> RenderType (ours):\n")
    w("//     0 scaling/enum + enum_group_id -> RENDER_ENUM\n")
    w("//     0 scaling/enum + scaling_id    -> RENDER_SCALED\n")
    w("//     0 with neither                 -> RENDER_RAW\n")
    w("//     1 ASCII                        -> RENDER_ASCII\n")
    w("//     2 decimal / 3 hex / 4 binary   -> RENDER_RAW\n")
    w("// Vendor types 2, 3 and 4 differ only in presentation, which RenderType does\n")
    w("// not model, so they collapse onto RENDER_RAW.  A row with an enum_group_id\n")
    w("// is RENDER_ENUM even though its render_type is 0.\n")
    w("//\n")
    w("// Units and scale factors are the metric (met_*) columns throughout.\n")
    w("//\n")
    w("// dtcLookup() and signalByName() below the tables are hand-written and are\n")
    w("// reproduced verbatim by the generator.\n")
    w("#include \"SignalDb.h\"\n")
    w("\n")
    w("#include <stddef.h>\n")
    w("#include <string.h>\n")
    w("\n")

    # kEcus
    w("// ---------------------------------------------------------------------------\n")
    w("// Modules.  interiorBus false marks the 500 kbps CAN-C side, which the\n")
    w("// gateway may or may not forward a CAN-B request to.\n")
    w("// ---------------------------------------------------------------------------\n")
    w("const EcuDef kEcus[ECU_COUNT] = {\n")
    for module, req, resp, interior in ecu_rows:
        w("    { 0x%03X, 0x%03X, %-10s %-5s },\n"
          % (req, resp, cstr(module) + ",", "true" if interior else "false"))
    w("};\n\n")

    # kScalings
    w("// ---------------------------------------------------------------------------\n")
    w("// Scalings actually referenced by kSignals; the other rows of\n")
    w("// pid_scaling.csv are pruned.  display = raw * factor + offset.\n")
    w("// ---------------------------------------------------------------------------\n")
    w("const ScalingDef kScalings[] = {\n")
    for i, (sid, factor, offset, unit, decimals) in enumerate(scaling_rows):
        w("    { %-14s %-8s %-10s %d },  // [%d] pid_scaling.csv scaling_id %d\n"
          % (cfloat(factor) + ",", cfloat(offset) + ",", cstr(unit) + ",",
             decimals, i, sid))
    w("};\n")
    w("const uint8_t kScalingCount = %d;\n\n" % len(scaling_rows))

    # kEnums
    w("// ---------------------------------------------------------------------------\n")
    w("// Enum labels, sorted by (group, from).  `group` is the vendor\n")
    w("// enum_group_id, NOT an index: SignalDef::enumGroup names a group and the\n")
    w("// matching entries are found by scanning this table for that tag.\n")
    w("// ---------------------------------------------------------------------------\n")
    w("const EnumEntry kEnums[] = {\n")
    prev_group = None
    for group, lo, hi, label, _rid in enum_rows:
        if prev_group is not None and group != prev_group:
            w("\n")
        prev_group = group
        w("    { %4d, %5d, %5d, %s },\n" % (group, lo, hi, cstr(label)))
    w("};\n")
    w("const uint16_t kEnumCount = %d;\n\n" % len(enum_rows))

    # kSignals
    w("// ---------------------------------------------------------------------------\n")
    w("// The curated signal set: %d fields, sorted by (ecu, localId, bitOffset).\n"
      % len(signal_rows))
    w("//\n")
    w("// bitOffset counts from byte 0 of the FULL reassembled response, service\n")
    w("// byte and local-identifier echo included -- so a field in the first data\n")
    w("// byte of a 0x21xx read sits at bit 16.\n")
    w("//\n")
    w("// Names are module-qualified (\"CCN.Vehicle Speed\") because name_en is not\n")
    w("// unique across modules.  signalByName() also accepts the bare name.\n")
    w("// ---------------------------------------------------------------------------\n")
    w("const SignalDef kSignals[] = {\n")
    prev = None
    for (_i, ecu, lid, off, length, render, scaling, group, name, sid, rid) in signal_rows:
        if prev is not None and (ecu, lid) != prev:
            w("\n")
        prev = (ecu, lid)
        note = "pids.csv row %d" % rid
        if scaling >= 0:
            note += ", scaling_id %d" % sid
        w("    { %-12s 0x%04X, %5d, %3d, %-14s %3d, %5d, %s },  // %s\n"
          % (ecu + ",", lid, off, length, render + ",", scaling, group,
             cstr(name), note))
    w("};\n")
    w("const uint16_t kSignalCount = %d;\n\n" % len(signal_rows))

    # kDtcs
    w("// ---------------------------------------------------------------------------\n")
    w("// Known DTC descriptions, sorted by (ecu, code) so dtcLookup() can binary\n")
    w("// search it.  The sort key is the EcuId ORDINAL, so renumbering EcuId in\n")
    w("// SignalDb.h invalidates this ordering -- append new modules at the end.\n")
    if dropped:
        w("//\n")
        w("// Codes for these modules are NOT here, because EcuId cannot name them:\n")
        for module, n in sorted(dropped.items()):
            w("//     %-8s %4d code(s)\n" % (module, n))
    else:
        w("//\n")
        w("// Every module in dtc_codes.csv has an EcuId, so no code is dropped.\n")
    w("// ---------------------------------------------------------------------------\n")
    w("const DtcDef kDtcs[] = {\n")
    prev_ecu = None
    for ecu_i, code, sae, text, ecu_enum in dtc_rows:
        if prev_ecu is not None and ecu_i != prev_ecu:
            w("\n")
        if ecu_i != prev_ecu:
            w("    // %s\n" % ecu_enum)
        prev_ecu = ecu_i
        w("    { %-12s 0x%04X, %-9s %s },\n"
          % (ecu_enum + ",", code, cstr(sae) + ",", cstr(text)))
    w("};\n")
    w("const uint16_t kDtcCount = %d;\n\n" % len(dtc_rows))

    # hand-written lookups
    w(r'''// ---------------------------------------------------------------------------
// Lookups.  Hand-written, but kept in this file so the tables and the code that
// depends on their ordering cannot drift apart.  No allocation, no statics.
// ---------------------------------------------------------------------------

// Binary search on the (ecu, code) ordering the generator guarantees above.
const DtcDef* dtcLookup(uint8_t ecu, uint16_t code) {
  uint16_t lo = 0;
  uint16_t hi = kDtcCount;   // half-open
  while (lo < hi) {
    const uint16_t mid = lo + (hi - lo) / 2;   // no overflow
    const DtcDef& d = kDtcs[mid];
    if (d.ecu < ecu || (d.ecu == ecu && d.code < code)) {
      lo = mid + 1;
    } else if (d.ecu == ecu && d.code == code) {
      return &d;
    } else {
      hi = mid;
    }
  }
  return NULL;
}

// Linear; called only from setup, never from the polling loop.
//
// Accepts either the qualified table name ("CCN.Vehicle Speed") or the bare
// vendor name ("Vehicle Speed").  A bare name that two modules share resolves
// to the first match in table order, i.e. the lowest EcuId -- exact-match on
// the qualified name is the way to be unambiguous.
const SignalDef* signalByName(const char* name) {
  if (name == NULL) return NULL;

  for (uint16_t i = 0; i < kSignalCount; ++i) {
    if (strcmp(kSignals[i].name, name) == 0) return &kSignals[i];
  }
  for (uint16_t i = 0; i < kSignalCount; ++i) {
    const char* dot = strchr(kSignals[i].name, '.');
    if (dot != NULL && strcmp(dot + 1, name) == 0) return &kSignals[i];
  }
  return NULL;
}
''')
    return "".join(o)


if __name__ == "__main__":
    sys.exit(main())
