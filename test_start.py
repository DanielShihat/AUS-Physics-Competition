#!/usr/bin/env python3
"""
Simple test: send START and see what happens
"""

import sys
import serial
import time

if len(sys.argv) < 2:
    print("Usage: python3 test_start.py /dev/cu.usbmodemXXXX")
    sys.exit(1)

port = sys.argv[1]

print(f"Connecting to {port} at 115200 baud...")
try:
    ser = serial.Serial(port, 115200, timeout=2)
    print("✓ Connected!")
except Exception as e:
    print(f"✗ Failed: {e}")
    sys.exit(1)

# Wait a bit
time.sleep(1)
ser.reset_input_buffer()

print("\n=== Sending START ===")
ser.write(b"START\n")
time.sleep(0.5)

# Read responses
print("=== Reading responses ===")
start = time.time()
count = 0
while time.time() - start < 3:
    if ser.in_waiting:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if line:
            count += 1
            print(f"  [{count}] {line}")
            if count >= 20:
                print("  ... (more data) ...")
                break
    time.sleep(0.01)

print(f"\n=== Summary ===")
print(f"Lines received: {count}")

if count == 0:
    print("\n⚠ No data received!")
    print("Check:")
    print("  1. Are sensors wired? (look for MPU_INIT messages)")
    print("  2. Did you see 'STREAM_ON'?")
else:
    print("✓ Data is flowing!")

ser.write(b"STOP\n")
ser.close()
