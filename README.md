# Real-Time Bridge Structural Health Monitoring
A physics competition project that detects structural damage in a model bridge from its vibration signature.
IMU sensors stream acceleration data from the bridge. A Python dashboard computes a live FFT, compares the dominant frequency and amplitude against a healthy baseline, and classifies the structure as healthy, mildly changed, damaged, or critical.
Built for hardware demos and for simulation when the physical setup is not available.
---
## Why it matters
Damage changes how a structure vibrates: resonance typically shifts down, amplitude drops, and damping increases. This system treats those shifts as measurable signals rather than visual inspection.
It is an end-to-end pipeline: embedded sensing → serial protocol → signal processing → live classification UI.
---
## System overview
```
MPU-6050 IMUs  →  TCA9548A I2C mux  →  Arduino Uno  →  USB serial  →  Python GUI
                         ↑
              vibration motor (PWM shaker)
```
**Firmware** (`bridge_stream/bridge_stream.ino`)
- Reads up to three MPU-6050 IMUs through a TCA9548A multiplexer
- Streams CSV samples at a fixed rate (default 200 Hz): `tms,sid,ax,ay,az,gx,gy,gz`
- Command protocol over serial: `START`, `STOP`, `CAL`, `STATUS`, `RATE`, `SENS`, `MOTOR`, `MOTORON`, `MOTOROFF`
- I2C retries and sensor re-init so a glitchy bus does not freeze the stream
- PWM motor on D9 for controlled excitation
**Live monitor** (`bridge_live.py`)
- Background serial reader (does not block the UI)
- Rolling time-domain plot of vertical acceleration (`az`)
- Hann-windowed real FFT in the 1–40 Hz band
- Exponential smoothing of peak frequency and amplitude
- Multi-window averaged baseline, then relative-amplitude classification
- Per-sensor and overall status: healthy / mild / damaged / critical
- Motor power control from the GUI
**Simulator** (`bridge_live_sim.py` + `sim_bridge_source.py`)
- Same dashboard without hardware
- Synthetic IMU traces: shaker (steady sinusoid) or tap (decaying sinusoid)
- Damage levels shift frequency down, reduce amplitude, and increase damping
---
## Signal processing
1. Keep a ~10 s rolling buffer per sensor.
2. Estimate sample rate from inter-sample times.
3. On the last few seconds of `az`, remove DC, apply a Hann window, compute `rfft`.
4. Take the peak magnitude in 1–40 Hz as dominant frequency `f1` and amplitude `A1`.
5. Smooth `f1` / `A1` with EMA so FFT jitter does not flip the status.
6. Capture a baseline by averaging several FFT windows while the bridge is healthy.
7. Classify from relative amplitude change `da = |A1 − A0| / A0`, requiring consecutive bad windows before escalating.
| Relative amplitude change | Status |
|---|---|
| &lt; 35% | Healthy |
| 35–70% | Mild |
| 70–120% | Damaged |
| &gt; 120% | Critical |
---
## Hardware
| Part | Role |
|---|---|
| Arduino Uno | MCU, serial host, PWM |
| MPU-6050 (×2 or ×3) | 6-axis IMU; vertical accel used for FFT |
| TCA9548A | I2C multiplexer (sensors share address `0x68`) |
| Vibration motor on D9 | Shaker excitation |
Default mapping: sensor 0 = left span, sensor 1 = right span.
---
## Setup
**Firmware**
1. Open `bridge_stream/bridge_stream.ino` in Arduino IDE.
2. Board: Arduino Uno. Baud: **115200**.
3. Upload, then close Serial Monitor (the port can only be used by one program).
**Python**
```bash
python3 -m pip install numpy pyqtgraph pyserial
```
PyQtGraph needs a Qt binding (PyQt5 or PySide2).
**Serial port (macOS)**
```bash
ls /dev/cu.usbmodem*
```
---
## Run
Live hardware:
```bash
python3 bridge_live.py /dev/cu.usbmodemXXXX
```
Typical demo:
1. Keep the bridge still → **CAL**
2. Enable the motor and set power
3. **START stream**
4. Wait until the time plot looks stable → **SET BASELINE**
5. Introduce damage (loosen a joint, add a mass, change stiffness) and watch status
Simulation (no Arduino):
```bash
python3 bridge_live_sim.py
```
Use **Damage 0 / 1 / 2** and **Mode: SHAKER / TAP** to see the classifier respond.
Connection checks:
```bash
python3 test_connection.py /dev/cu.usbmodemXXXX
python3 test_start.py /dev/cu.usbmodemXXXX
```
If the board does not respond, see `README_TROUBLESHOOTING.md`.
---
## Repository layout
| File | Description |
|---|---|
| `bridge_stream/bridge_stream.ino` | Arduino firmware: IMU mux, motor, serial protocol |
| `bridge_live.py` | Live FFT dashboard and damage classifier |
| `bridge_live_sim.py` | Same UI driven by the simulator |
| `sim_bridge_source.py` | Synthetic IMU stream (healthy vs damaged) |
| `test_connection.py` / `test_start.py` | Serial smoke tests |
| `README_TROUBLESHOOTING.md` | Arduino / serial debugging |
---
## Skills this project demonstrates
- Embedded I2C (multiplexed IMUs), PWM actuation, and a text command protocol
- Real-time serial streaming and multithreaded desktop UI
- Frequency-domain feature extraction (FFT, windowing, peak picking)
- Baseline comparison and hysteresis so the status is stable under noise
- Hardware-in-the-loop design with a physics-informed simulator for demos and testing
