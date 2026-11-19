#!/usr/bin/env python3
"""Find the steering servo value where the front wheels point straight.

Sends only VESC COMM_SET_SERVO_POS (steering). The drive motor is never commanded.
Stop teleop first so the serial port is free:  sudo systemctl stop rc-teleop.service

Keys:  a / d  = -/+ 0.01     z / c = -/+ 0.002     r = back to 0.50
       [ / ]  = jump to left / right limit test (0.15 / 0.85)     q = quit (returns to centre)
"""
import struct
import sys
import termios
import tty

import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/vesc"
COMM_SET_SERVO_POS = 12


def crc16(data):
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else crc << 1
            crc &= 0xFFFF
    return crc


def servo_packet(pos):
    payload = struct.pack(">Bh", COMM_SET_SERVO_POS, int(round(pos * 1000)))
    return bytes([2, len(payload)]) + payload + struct.pack(">H", crc16(payload)) + bytes([3])


def key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main():
    pos = 0.50
    steps = {"a": -0.01, "d": 0.01, "z": -0.002, "c": 0.002}
    with serial.Serial(PORT, 115200, timeout=0.1) as vesc:
        while True:
            pos = min(max(pos, 0.0), 1.0)
            vesc.write(servo_packet(pos))
            print(f"\rservo = {pos:.3f}   (a/d 0.01, z/c 0.002, r centre, [ ] limits, q quit) ",
                  end="", flush=True)
            k = key()
            if k in steps:
                pos += steps[k]
            elif k == "r":
                pos = 0.50
            elif k == "[":
                pos = 0.15
            elif k == "]":
                pos = 0.85
            elif k in ("q", "\x03"):
                print(f"\nLast value: {pos:.3f}. Returning servo to 0.50.")
                vesc.write(servo_packet(0.50))
                break


if __name__ == "__main__":
    main()
