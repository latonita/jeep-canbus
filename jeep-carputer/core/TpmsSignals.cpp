#include "TpmsSignals.h"

// Response layout: 61 3B <p1> <p2> <p3> <p4>
//
// bitOffset counts from byte 0 of the FULL response, service byte and
// identifier echo included -- so the first pressure sits at bit 16, not bit 0.
// Getting that wrong reads the echo as a pressure and every corner is silently
// off by two bytes.
//
// RENDER_RAW with no scaling row: the value published is raw psi. Converting to
// bar is the presenter's job, so the number stored here stays the number the
// bus sent.
const SignalDef kTpmsSignals[] = {
    {ECU_WCM, 0x213B, 16, 8, RENDER_RAW, -1, -1, "WCM.Tyre Sensor 1"},
    {ECU_WCM, 0x213B, 24, 8, RENDER_RAW, -1, -1, "WCM.Tyre Sensor 2"},
    {ECU_WCM, 0x213B, 32, 8, RENDER_RAW, -1, -1, "WCM.Tyre Sensor 3"},
    {ECU_WCM, 0x213B, 40, 8, RENDER_RAW, -1, -1, "WCM.Tyre Sensor 4"},

    // The spare, from its own request. `21 24` answered `61 24 07 4C 01 83` in
    // the probe log, so the same layout applies: two bytes of echo, then
    // pressure in psi at bit 16. The recorded 7 psi is either a genuinely flat
    // spare or a sensor that is not paired -- which is exactly why it is worth
    // a cell on the screen rather than a footnote.
    {ECU_WCM, 0x2124, 16, 8, RENDER_RAW, -1, -1, "WCM.Tyre Spare"},
};

const uint8_t kTpmsSignalCount = sizeof(kTpmsSignals) / sizeof(kTpmsSignals[0]);
