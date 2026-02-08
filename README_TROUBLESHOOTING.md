# Arduino Troubleshooting Guide

## Problem: Arduino not responding over serial

### Step 1: Verify Upload in Arduino IDE

1. Open Arduino IDE
2. Open `test_serial.ino` (or `bridge_stream.ino`)
3. **Tools → Board → Arduino Uno** (must be selected)
4. **Tools → Port → /dev/cu.usbmodem1401** (or whatever port shows)
5. Click **Upload** button (→ arrow)
6. **Watch the bottom of Arduino IDE** - you should see:
   - "Compiling sketch..."
   - "Uploading..."
   - "Done uploading."

### Step 2: Test with Arduino Serial Monitor

1. **Close any Python scripts** using the port
2. In Arduino IDE: **Tools → Serial Monitor**
3. Set baud rate to **9600** (for test_serial.ino) or **460800** (for bridge_stream.ino)
4. You should see messages appear

### Step 3: If Serial Monitor Works But Python Doesn't

- Check baud rate matches (9600 vs 460800)
- Make sure Serial Monitor is **CLOSED** when running Python
- Try the port with `/dev/cu.usbmodem1401` instead of `/dev/tty.usbmodem1401`

### Step 4: If Nothing Works

- Try a different USB cable
- Try a different USB port on your computer
- Check Arduino power LED is on
- Try uploading the basic "Blink" example sketch first
