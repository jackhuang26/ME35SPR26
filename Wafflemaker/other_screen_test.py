#!/usr/bin/env python3
"""
waffle_timer.py — Raspberry Pi side
Sends a countdown value to the ESP32 over serial,
then waits for "DONE" and prints a message.

Usage:
  python3 waffle_timer.py          # defaults to 60 seconds
  python3 waffle_timer.py 120      # countdown from 120 seconds

Wiring:
  Pi GPIO 14 (TX, Pin 8)  →  ESP32 GPIO16 (RX2)
  Pi GPIO 15 (RX, Pin 10) →  ESP32 GPIO17 (TX2)
  Pi GND (Pin 6)          →  ESP32 GND

First, enable UART on the Pi:
  sudo raspi-config → Interface Options → Serial Port
    "login shell over serial" → No
    "serial port hardware enabled" → Yes
  Then reboot.
"""

import serial
import sys
import time

SERIAL_PORT = "/dev/ttyS0"   # Pi 4 hardware UART
BAUD_RATE   = 115200

def main():
    seconds = int(sys.argv[1]) if len(sys.argv) > 1 else 60

    print(f"Opening {SERIAL_PORT} at {BAUD_RATE} baud...")
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    except serial.SerialException as e:
        print(f"ERROR: Could not open serial port: {e}")
        print("Try: sudo pip install pyserial --break-system-packages")
        sys.exit(1)

    time.sleep(2)  # let ESP32 settle after connection

    # Send the countdown value
    message = f"{seconds}\n"
    ser.write(message.encode())
    print(f"Sent countdown: {seconds} seconds")

    # Wait for ACK
    deadline = time.time() + 5
    while time.time() < deadline:
        line = ser.readline().decode("utf-8", errors="ignore").strip()
        if line.startswith("ACK:"):
            print(f"ESP32 acknowledged: {line}")
            break
    else:
        print("WARNING: No ACK from ESP32 — check wiring and baud rate")

    # Wait for DONE signal
    print(f"Waiting {seconds} seconds for waffle to be ready...")
    while True:
        line = ser.readline().decode("utf-8", errors="ignore").strip()
        if line == "DONE":
            print("=" * 40)
            print("  🧇  WAFFLE IS READY!  🧇")
            print("=" * 40)
            break
        elif line:
            print(f"ESP32: {line}")  # print any other messages

    ser.close()

if __name__ == "__main__":
    main()
