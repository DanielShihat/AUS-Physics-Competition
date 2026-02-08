#!/usr/bin/env python3
"""
Quick test to see if Arduino is responding to START command
"""

import sys
import serial
import time

if len(sys.argv) < 2:
    print("Usage: python3 test_connection.py /dev/cu.usbmodemXXXX")
    sys.exit(1)

port = sys.argv[1]

print(f"Connecting to {port}...")
try:
    ser = serial.Serial(port, 115200, timeout=2)
    print("✓ Connected!\n")
except Exception as e:
    print(f"✗ Failed: {e}")
    sys.exit(1)

# Wait a bit
time.sleep(1)

# Clear buffer
ser.reset_input_buffer()

print("=== Sending START command ===")
ser.write(b"START\n")
time.sleep(0.5)

# Check for STREAM_ON response
print("=== Looking for STREAM_ON response ===")
start = time.time()
found_stream_on = False
while time.time() - start < 2:
    if ser.in_waiting:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if line:
            print(f"  {line}")
            if "STREAM_ON" in line:
                found_stream_on = True
    time.sleep(0.1)

if not found_stream_on:
    print("  ⚠ Did not see STREAM_ON - Arduino may not have received START")
else:
    print("  ✓ Arduino received START and is streaming")

print("\n=== Reading data for 3 seconds ===")
data_count = 0
start = time.time()
while time.time() - start < 3:
    if ser.in_waiting:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if line:
            data_count += 1
            if data_count <= 5:
                print(f"  [{data_count}] {line}")
            elif data_count == 6:
                print("  ... (more data coming) ...")
    
    time.sleep(0.01)

print(f"\n=== Summary ===")
print(f"Total data lines received: {data_count}")

if data_count == 0:
    print("\n⚠ NO DATA!")
    print("Possible reasons:")
    print("  1. Sensors not wired → Arduino can't read sensors")
    print("  2. Check Serial Monitor: type START and see if data appears")
    print("  3. Arduino might need RESET after Python connects")
else:
    print("✓ Data is flowing! The connection works.")

ser.write(b"STOP\n")
ser.close()
