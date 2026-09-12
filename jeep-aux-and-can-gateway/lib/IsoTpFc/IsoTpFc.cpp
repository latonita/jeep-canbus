// PURE implementation -- see IsoTpFc.h. No Arduino, no AVR, no allocation.
#include "IsoTpFc.h"

void isoTpFcDefaults(IsoTpFcConfig& cfg) {
  cfg.header = ISOTP_FC_HEADER_UNSET;
  cfg.data[0] = ISOTP_FC_DEFAULT_B0;
  cfg.data[1] = ISOTP_FC_DEFAULT_B1;
  cfg.data[2] = ISOTP_FC_DEFAULT_B2;
  for (uint8_t i = ISOTP_FC_DEFAULT_LEN; i < CAN_MAX_DLC; ++i) {
    cfg.data[i] = 0x00u;
  }
  cfg.len = ISOTP_FC_DEFAULT_LEN;
  cfg.mode = ISOTP_FC_MODE_AUTO;
}

IsoTpType isoTpClassify(const CanFrame& frame) {
  // A zero-length frame has no PCI byte; indexing data[0] would be reading
  // whatever the driver left there.
  if (frame.len == 0u) {
    return ISOTP_TYPE_UNKNOWN;
  }
  switch (static_cast<uint8_t>(frame.data[0] >> 4)) {
    case ISOTP_PCI_SINGLE: return ISOTP_TYPE_SINGLE;
    case ISOTP_PCI_FIRST:  return ISOTP_TYPE_FIRST;
    case ISOTP_PCI_CONSEC: return ISOTP_TYPE_CONSEC;
    case ISOTP_PCI_FLOW:   return ISOTP_TYPE_FLOW;
    default:               return ISOTP_TYPE_UNKNOWN;
  }
}

bool isoTpIsFirstFrame(const CanFrame& frame) {
  return frame.len != 0u && (frame.data[0] >> 4) == ISOTP_PCI_FIRST;
}

uint16_t isoTpFirstFrameLen(const CanFrame& frame) {
  // FF_DL spans the low nibble of data[0] and all of data[1].
  if (frame.len < 2u || !isoTpIsFirstFrame(frame)) {
    return 0u;
  }
  return static_cast<uint16_t>((static_cast<uint16_t>(frame.data[0] & 0x0Fu) << 8) |
                               static_cast<uint16_t>(frame.data[1]));
}

uint8_t isoTpSingleFrameLen(const CanFrame& frame) {
  if (frame.len == 0u || (frame.data[0] >> 4) != ISOTP_PCI_SINGLE) {
    return 0u;
  }
  return static_cast<uint8_t>(frame.data[0] & 0x0Fu);
}

uint8_t isoTpConsecPayloadLen(const CanFrame& frame) {
  if (frame.len == 0u || (frame.data[0] >> 4) != ISOTP_PCI_CONSEC) {
    return 0u;
  }
  return static_cast<uint8_t>(frame.len - 1u);
}

IsoTpFcStatus isoTpBuildFc(const IsoTpFcConfig& cfg,
                           const CanFrame& first,
                           uint16_t requestHeader,
                           CanFrame& out) {
  if (!isoTpIsFirstFrame(first)) {
    return ISOTP_FC_NOT_FIRST_FRAME;
  }
  if (cfg.len == 0u || cfg.len > CAN_MAX_DLC || cfg.mode > ISOTP_FC_MODE_MAX) {
    return ISOTP_FC_BAD_CONFIG;
  }

  const uint16_t id = (cfg.header == ISOTP_FC_HEADER_UNSET) ? requestHeader : cfg.header;
  if (id == ISOTP_FC_HEADER_UNSET) {
    return ISOTP_FC_NO_HEADER;
  }
  if (id > 0x7FFu) {
    return ISOTP_FC_BAD_CONFIG;
  }

  out.id = id;
  out.len = cfg.len;
  for (uint8_t i = 0u; i < CAN_MAX_DLC; ++i) {
    out.data[i] = (i < cfg.len) ? cfg.data[i] : 0x00u;
  }
  return ISOTP_FC_OK;
}
