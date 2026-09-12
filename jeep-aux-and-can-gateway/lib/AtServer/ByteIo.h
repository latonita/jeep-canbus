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

 protected:
  // Non-virtual and protected, matching ICanBus: the implementation lives in
  // static storage for the whole program and a virtual dtor would pull in
  // operator delete.
  ~IByteIo() = default;
};
