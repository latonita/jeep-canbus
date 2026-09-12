#include "CanDriver.h"

CanDriver::CanDriver()
    : mcp_(PIN_CAN_CS),
      acceptMask_(0),
      acceptCount_(0),
      lastEflg_(0),
      ready_(false),
      rxCount_(0),
      txCount_(0),
      errCount_(0) {
  for (uint8_t i = 0; i < CAN_ACCEPT_MAX; ++i) {
    acceptId_[i] = 0;
  }
}

void CanDriver::bump(uint16_t& counter) {
  // Saturate instead of wrapping: 65535 means "lots", 0 must mean "none".
  if (counter != 0xFFFFu) {
    ++counter;
  }
}

CanStatus CanDriver::begin() {
  ready_ = false;
  lastEflg_ = 0;

  // Bounded attempts. The original firmware did
  //   while (CAN_OK != CAN.begin(...)) delay(250);
  // which starves the AT link and the watchdog. Here a failed begin() just
  // reports CAN_ST_NOT_READY and the caller comes back after
  // CAN_INIT_RETRY_MS with the main loop still turning.
  for (uint8_t attempt = 0; attempt < CAN_BEGIN_ATTEMPTS; ++attempt) {
    if (mcp_.begin(CAN_BITRATE_ID, CAN_CRYSTAL_ID) == CAN_OK) {
      ready_ = true;
      break;
    }
  }

  if (!ready_) {
    bump(errCount_);
    return CAN_ST_NOT_READY;
  }

  // The library leaves RXMn/RXFn untouched (its mask/filter init is commented
  // out and the MCP2515 does not reset those registers) while still enabling
  // filtering, so the acceptance state must be written explicitly here. On a
  // reinit() this restores whatever the modules had asked for.
  return applyAcceptance();
}

CanStatus CanDriver::reinit() {
  // begin() already resets the controller (mcp2515_reset) and re-applies the
  // acceptance table, so there is nothing extra to undo. This is also the only
  // way to clear a latched RXnOVR: the library exposes no EFLG write.
  return begin();
}

CanStatus CanDriver::send(const CanFrame& frame) {
  if (frame.len > CAN_MAX_DLC || frame.id > CAN_STD_ID_MAX) {
    bump(errCount_);
    return CAN_ST_BAD_PARAM;
  }
  if (!ready_) {
    return CAN_ST_NOT_READY;
  }

  // Two reasons this staging copy is not optional:
  //  - sendMsgBuf() takes a non-const INT8U*.
  //  - setMsg() copies MAX_CHAR_IN_MESSAGE (8) bytes regardless of len, so the
  //    source must be 8 bytes and fully initialised or it reads our stack.
  uint8_t buf[CAN_MAX_DLC];
  for (uint8_t i = 0; i < CAN_MAX_DLC; ++i) {
    buf[i] = (i < frame.len) ? frame.data[i] : 0u;
  }

  // The 5-argument overload on purpose. The 4-argument one routes to a setMsg()
  // that never assigns m_nRtr, and m_nRtr is shared with the receive path -- so
  // after receiving one remote-request frame every later send would go out as
  // an RTR frame. Passing rtr=0 explicitly makes that impossible.
  INT8U res = mcp_.sendMsgBuf(static_cast<INT32U>(frame.id), CAN_STDID, 0,
                              frame.len, buf);

  switch (res) {
    case CAN_OK:
      bump(txCount_);
      return CAN_ST_OK;

    case CAN_SENDMSGTIMEOUT:
      // NOT a failure on this bus. sendMsg() writes the frame and sets TXREQ
      // before it starts polling, so the controller will transmit it as soon
      // as it wins arbitration. Its wait is TIMEOUTVALUE (50) SPI register
      // reads, which is well under the ~1.6 ms an 8-byte frame takes at
      // 83.3 kbps -- so this is the NORMAL outcome here, not an error. Booking
      // it as one is what made AT@ST report TX=0 with a climbing ER while the
      // radio was plainly receiving the VES announce.
      bump(txCount_);
      return CAN_ST_OK;

    case CAN_GETTXBFTIMEOUT:   // all three TX buffers still busy: real backlog
      bump(errCount_);
      return CAN_ST_TX_FAIL;

    default:
      bump(errCount_);
      return CAN_ST_TX_FAIL;
  }
}

CanStatus CanDriver::receive(CanFrame& out) {
  if (!ready_) {
    return CAN_ST_NOT_READY;
  }

  // Bounded: frames we cannot represent are skipped rather than returned, and
  // this loop must not become an unbounded drain of a busy bus.
  for (uint8_t attempt = 0; attempt < CAN_RX_SKIP_MAX; ++attempt) {
    if (mcp_.checkReceive() != CAN_MSGAVAIL) {
      return CAN_ST_NO_MSG;
    }

    INT32U rawId = 0;
    INT8U  rawLen = 0;
    uint8_t buf[CAN_MAX_DLC] = {0, 0, 0, 0, 0, 0, 0, 0};

    // readMsgBufID(), not readMsgBuf(): both report the raw 4-bit DLC, but
    // readMsgBuf() then copies that many bytes into the caller's buffer, so a
    // frame with DLC 9..15 (legal on the wire, means 8 bytes) would overrun
    // buf. readMsgBufID() clamps its copy to MAX_CHAR_IN_MESSAGE. It also
    // hands back the id in the same call, which getCanId() would otherwise
    // require a second step for.
    if (mcp_.readMsgBufID(&rawId, &rawLen, buf) != CAN_OK) {
      return CAN_ST_NO_MSG;
    }

    // CanFrame has no room for a 29-bit id or an RTR flag, and the hardware
    // masks are written with the extended bits as don't-care, so both can turn
    // up here. Skip them; they are not errors on this bus.
    if (mcp_.isExtendedFrame() != 0 || mcp_.isRemoteRequest() != 0) {
      continue;
    }

    // Never trust the reported length: it is RXBnDLC & 0x0F, so 0..15.
    uint8_t len = static_cast<uint8_t>(rawLen);
    if (len > CAN_MAX_DLC) {
      len = CAN_MAX_DLC;
    }

    out.id = static_cast<uint16_t>(rawId & CAN_STD_ID_MAX);
    out.len = len;
    for (uint8_t i = 0; i < CAN_MAX_DLC; ++i) {
      out.data[i] = (i < len) ? buf[i] : 0u;
    }

    bump(rxCount_);
    return CAN_ST_OK;
  }

  return CAN_ST_NO_MSG;
}

CanStatus CanDriver::writeMasks(uint16_t mask) {
  // RXM0 gates RXF0/RXF1 into RXB0, RXM1 gates RXF2..RXF5 into RXB1. Both get
  // the same value so a matching frame lands in whichever buffer is free.
  //
  // init_Mask() puts the controller into MODE_CONFIG and back into MODE_NORMAL
  // itself, so we must not do that around it. Its `num` validation is broken
  // (the MCP2515_FAIL it assigns is overwritten by the mode-restore result), so
  // 0 and 1 are hardcoded here rather than passed in.
  //
  // The mask is written as a standard id, which zeroes the EIDn mask bytes and
  // leaves the extended bits as don't-care -- see mcp2515_write_id().
  if (mcp_.init_Mask(0, CAN_STDID, static_cast<INT32U>(mask)) != MCP2515_OK) {
    bump(errCount_);
    ready_ = false;
    return CAN_ST_BUS_ERROR;
  }
  if (mcp_.init_Mask(1, CAN_STDID, static_cast<INT32U>(mask)) != MCP2515_OK) {
    bump(errCount_);
    ready_ = false;
    return CAN_ST_BUS_ERROR;
  }
  return CAN_ST_OK;
}

CanStatus CanDriver::applyAcceptance() {
  if (!ready_) {
    return CAN_ST_NOT_READY;
  }

  if (acceptCount_ == 0) {
    // Mask 0 = every bit is don't-care, so the filters are irrelevant and
    // everything passes.
    return writeMasks(0);
  }

  CanStatus st = writeMasks(acceptMask_);
  if (st != CAN_ST_OK) {
    return st;
  }

  // Every one of the six filters gets written, round-robin over the accepted
  // set, so both filter banks cover the whole set. A filter left alone would
  // sit at an undefined value and, with a narrow mask, waste a slot or accept
  // an id nobody asked for.
  for (uint8_t f = 0; f < CAN_ACCEPT_MAX; ++f) {
    uint16_t id = acceptId_[f % acceptCount_];
    if (mcp_.init_Filt(f, CAN_STDID, static_cast<INT32U>(id)) != MCP2515_OK) {
      bump(errCount_);
      ready_ = false;
      return CAN_ST_BUS_ERROR;
    }
  }

  return CAN_ST_OK;
}

CanStatus CanDriver::setAcceptance(uint16_t id, uint16_t mask) {
  if (id > CAN_STD_ID_MAX || mask > CAN_STD_ID_MAX) {
    bump(errCount_);
    return CAN_ST_BAD_PARAM;
  }

  // Already accepted: nothing to reprogram, and this must stay cheap because
  // several modules may ask for the same id.
  for (uint8_t i = 0; i < acceptCount_; ++i) {
    if (acceptId_[i] == id) {
      return CAN_ST_OK;
    }
  }

  if (acceptCount_ >= CAN_ACCEPT_MAX) {
    bump(errCount_);
    return CAN_ST_BAD_PARAM;
  }

  // The hardware has two masks shared by all six filters, but the interface is
  // per-id. Combining with AND keeps the loosest of the requested masks, which
  // can only let extra frames through -- never drop one a module asked for.
  // Callers still have to check frame.id, which Module::onFrame() does anyway.
  if (acceptCount_ == 0) {
    acceptMask_ = mask;
  } else {
    acceptMask_ &= mask;
  }

  acceptId_[acceptCount_] = id;
  ++acceptCount_;

  // Kept in the table even if programming fails, so the next reinit() retries.
  return applyAcceptance();
}

CanStatus CanDriver::acceptAll() {
  acceptCount_ = 0;
  acceptMask_ = 0;
  return applyAcceptance();
}

bool CanDriver::healthy() {
  if (!ready_) {
    return false;
  }

  INT8U eflg = 0;
  // checkError(INT8U*) is the two-argument overload; the no-argument one
  // collapses everything into CAN_CTRLERROR and we need the individual flags.
  // TEC/REC themselves are not reachable -- mcp2515_readRegister is private --
  // so the error-counter state has to be read off TXEP/RXEP (counter past 127)
  // and EWARN (past 95).
  (void)mcp_.checkError(&eflg);

  const INT8U kFatal = MCP_EFLG_TXBO | MCP_EFLG_TXEP | MCP_EFLG_RXEP;
  const INT8U kOverflow = MCP_EFLG_RX0OVR | MCP_EFLG_RX1OVR;

  // Count each new fault once. healthy() may be called every loop iteration
  // and an unconditional bump here would saturate errCount() immediately.
  const INT8U appeared = static_cast<INT8U>(eflg & ~lastEflg_);
  if ((appeared & (kFatal | kOverflow)) != 0) {
    bump(errCount_);
  }
  lastEflg_ = eflg;

  // RXnOVR only means we drained too slowly, and it is latched with no public
  // way to clear it (mcp2515_modifyRegister is private), so treating it as
  // unhealthy would mark the node dead forever after one late drain. Only
  // bus-off and error-passive count.
  return (eflg & kFatal) == 0;
}
