#!/usr/bin/env python3
"""Send one raw CAN frame through the gateway (ELM327 AT link) and exit.

Uses ATCAF0 raw mode: the bytes are sent verbatim, DLC = number of bytes.

Examples:
    python scripts/send_frame.py 1BD 85 00 00 00 00 00     # SDAR enable probe
    python scripts/send_frame.py 000 63                    # power-on byte only
    python scripts/send_frame.py --port COM17 --repeat 5 1BD 850000000000
"""
import argparse
import sys
import time

try:
    import serial  # pyserial
except ImportError:
    sys.exit("pyserial is required:  python -m pip install pyserial")


def read_until_prompt(ser, timeout=0.5):
    end = time.monotonic() + timeout
    buf = bytearray()
    while time.monotonic() < end:
        b = ser.read(64)
        if b:
            buf += b
            if b">" in buf:
                break
    return bytes(buf).replace(b"\r", b" ").decode("latin1").strip()


def cmd(ser, line, timeout=0.5):
    ser.reset_input_buffer()
    ser.write(line.encode("ascii") + b"\r")
    ser.flush()
    return read_until_prompt(ser, timeout)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("canid", help="11-bit CAN id in hex, e.g. 1BD")
    ap.add_argument("data", nargs="+", help="data bytes in hex: '85 00 ..' or '850000...'")
    ap.add_argument("--port", default="COM17")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--repeat", type=int, default=1, help="send N times")
    ap.add_argument("--interval", type=float, default=0.1, help="s between repeats")
    args = ap.parse_args()

    # Accept both "85 00 00" and "850000".
    hexstr = "".join(args.data).replace(" ", "").upper()
    if len(hexstr) % 2 or len(hexstr) == 0:
        sys.exit(f"data must be whole bytes (even hex digits): {hexstr!r}")
    nbytes = len(hexstr) // 2
    if nbytes > 8:
        sys.exit(f"a CAN frame holds at most 8 bytes, got {nbytes}")
    canid = args.canid.upper().lstrip("0X")

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.3)
    except Exception as e:
        sys.exit(f"could not open {args.port}: {e}\n-> close any serial terminal on that port first")

    try:
        ser.write(b"\r")
        time.sleep(0.1)
        ser.reset_input_buffer()
        cmd(ser, "ATE0")      # echo off
        cmd(ser, "ATS0")      # no spaces in responses
        cmd(ser, "ATCAF0")    # raw mode: bytes verbatim, DLC = byte count
        print(f"ATSH {canid} -> {cmd(ser, 'ATSH' + canid)}")
        print(f"frame: {canid} / DLC{nbytes} / {' '.join(hexstr[i:i+2] for i in range(0, len(hexstr), 2))}")
        for i in range(args.repeat):
            resp = cmd(ser, hexstr)
            print(f"  send[{i+1}/{args.repeat}] -> {resp}")
            if i + 1 < args.repeat:
                time.sleep(args.interval)
    finally:
        ser.close()


if __name__ == "__main__":
    main()
