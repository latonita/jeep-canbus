#pragma once
// Byte-stream abstraction for the AT link. The AT server talks only through
// this, so the whole command layer can be driven by a fake in host tests and
// never mentions HardwareSerial.
//
// PURE: no Arduino, no AVR headers.

class IByteIo {
 public:
  // Next byte from the link, or -1 when nothing is available. Never blocks.
  virtual int  readByte() = 0;

  // NUL-terminated string out. Blocking only for as long as the UART buffer
  // needs to drain -- the caller keeps lines short.
  virtual void writeBytes(const char* s) = 0;

  virtual void writeByte(char c) = 0;

  // Room left in the transmit buffer, in bytes.
  //
  // The background frame stream (AT@SM) uses this to decide whether a line
  // FITS before it starts writing one, because on that path a blocking write
  // is worse than a lost frame: stalling here stalls the main loop, which
  // overflows the MCP2515's two receive buffers and delays the VES announce
  // the car's audio depends on.
  //
  // The default answers "enough for anything", which is right for a host-test
  // fake and for any implementation that cannot block. Only the real UART
  // needs to override it.
  virtual int writableBytes() const { return 0x7FFF; }

 protected:
  // Non-virtual and protected, matching ICanBus: the implementation lives in
  // static storage for the whole program and a virtual dtor would pull in
  // operator delete.
  ~IByteIo() = default;
};
