# to run: python3 bridge_live.py /dev/tty.usbmodem11301


import sys

import time
import threading
from collections import deque

import numpy as np
import serial
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets

# ----------------------------
# Config
# ----------------------------
BAUD = 115200
MAX_SECONDS = 10           # rolling buffer length
DEFAULT_FS_EST = 200       # used for FFT windowing; we estimate real FS too
FFT_WINDOW_SECONDS = 4.0   # use last 4 seconds for FFT
FFT_UPDATE_HZ = 2.0        # FFT recompute rate
TIME_UPDATE_HZ = 30.0      # GUI refresh rate

# Detection thresholds (tune after first run)
DF_THRESHOLD_HZ = 0.30
DA_THRESHOLD_REL = 0.25
CONSEC_WINDOWS = 2

# Severity thresholds as multiplicative factors of baseline
# Example: MILD_FACTOR = 1.2 means measured >= baseline * 1.2 is mild
MILD_FACTOR = 1.2
DAMAGED_FACTOR = 1.5
CRITICAL_FACTOR = 2.0

# Motor control command format (adjust range if your firmware expects different)
MOTOR_POWER_FMT = "MOTOR {}"

# ----------------------------
# Serial reader thread
# ----------------------------
class SerialReader(threading.Thread):
    def __init__(self, port, on_line):
        super().__init__(daemon=True)
        self.port = port
        self.on_line = on_line
        self._stop = threading.Event()
        self.ser = None

    def stop(self):
        self._stop.set()
        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass

    def write(self, s: str):
        if not self.ser:
            return
        # Log outgoing commands for debugging
        try:
            print(f"[OUT] {s.strip()}")
        except Exception:
            pass
        self.ser.write((s.strip() + "\n").encode("utf-8"))
        self.ser.flush()  # Ensure command is sent immediately

    def run(self):
        self.ser = serial.Serial(self.port, BAUD, timeout=1)
        # Small settle time
        time.sleep(1.0)
        
        # Clear any old data in buffer
        self.ser.reset_input_buffer()

        # Optional: stop streaming then start clean
        self.write("STOP")
        time.sleep(0.1)
        self.write("STATUS")

        while not self._stop.is_set():
            try:
                line = self.ser.readline().decode("utf-8", errors="ignore").strip()
                if line:
                    self.on_line(line)
            except Exception:
                # keep going (USB hiccups happen)
                continue


# ----------------------------
# Data model
# ----------------------------
class DataModel:
    def __init__(self):
        # Per sensor buffers
        self.buffers = {
            0: deque(),
            1: deque(),
            2: deque()
        }
        self.maxlen = int(MAX_SECONDS * DEFAULT_FS_EST * 2)  # over-allocate; we cap by time anyway

        # For estimating actual sample rate per sensor
        self.last_tms = {0: None, 1: None, 2: None}
        self.dt_hist = {0: deque(maxlen=400), 1: deque(maxlen=400), 2: deque(maxlen=400)}

        # Baseline
        self.baseline = None  # dict: sid -> (f1, A1)
        self.consec_bad = 0

        # Severity history for smoothing per sensor (use CONSEC_WINDOWS)
        self.sev_hist = {
            0: deque(maxlen=CONSEC_WINDOWS),
            1: deque(maxlen=CONSEC_WINDOWS),
            2: deque(maxlen=CONSEC_WINDOWS),
        }

        # Latest computed features
        self.latest_feats = {}  # sid -> dict

        # Thread safety
        self.lock = threading.Lock()

    def add_sample(self, tms, sid, ax, ay, az, gx, gy, gz):
        with self.lock:
            # Keep only recent by time (rolling)
            t = tms / 1000.0
            self.buffers[sid].append((t, ax, ay, az, gx, gy, gz))

            # Cap by time, not just count
            cutoff = t - MAX_SECONDS
            while self.buffers[sid] and self.buffers[sid][0][0] < cutoff:
                self.buffers[sid].popleft()

            # dt history for FS estimate
            lt = self.last_tms[sid]
            if lt is not None:
                dt = (tms - lt) / 1000.0
                if 0.0 < dt < 0.2:
                    self.dt_hist[sid].append(dt)
            self.last_tms[sid] = tms

    def estimate_fs(self, sid):
        with self.lock:
            dts = list(self.dt_hist[sid])
        if len(dts) < 20:
            return DEFAULT_FS_EST
        med = np.median(dts)
        if med <= 0:
            return DEFAULT_FS_EST
        return float(1.0 / med)

    def get_series(self, sid):
        with self.lock:
            data = list(self.buffers[sid])
        if not data:
            return None
        arr = np.array(data, dtype=np.float64)
        t = arr[:, 0]
        az = arr[:, 3]  # ax,ay,az index: (t,ax,ay,az,...) => az = col 3
        return t, az

    def compute_fft_features(self, sid, fmin=1.0, fmax=40.0):
        series = self.get_series(sid)
        if series is None:
            return None
        t, az = series
        fs = self.estimate_fs(sid)

        # Need enough samples for the FFT window
        window = FFT_WINDOW_SECONDS
        t_end = t[-1]
        t_start = t_end - window
        mask = t >= t_start
        if np.sum(mask) < int(window * fs * 0.6):
            return None

        x = az[mask].astype(np.float64)
        # Remove DC
        x = x - np.mean(x)

        # Hann window
        w = np.hanning(len(x))
        xw = x * w

        # FFT
        X = np.fft.rfft(xw)
        freqs = np.fft.rfftfreq(len(xw), d=1.0/fs)
        mag = np.abs(X)

        # band select
        band = (freqs >= fmin) & (freqs <= fmax)
        if not np.any(band):
            return None
        freqs_b = freqs[band]
        mag_b = mag[band]

        idx = int(np.argmax(mag_b))
        f1 = float(freqs_b[idx])
        A1 = float(mag_b[idx])

        return {
            "fs": fs,
            "freqs": freqs_b,
            "mag": mag_b,
            "f1": f1,
            "A1": A1
        }

    def set_baseline(self):
        base = {}
        for sid in (0, 1, 2):
            feats = self.compute_fft_features(sid)
            if feats is None:
                continue
            base[sid] = (feats["f1"], feats["A1"])
        with self.lock:
            self.baseline = base if base else None
            self.consec_bad = 0

    def health_status(self):
        with self.lock:
            base = self.baseline
        if not base:
            return "NO BASELINE", pg.mkColor("w")

        # Only consider MPU 0 (left) and MPU 1 (right)
        per_sensor_sev = {0: 0, 1: 0}

        def _sev_from_baseline(f0, A0, f1, A1):
            # Returns 0=healthy,1=mild,2=damaged,3=critical
            if f0 is None or A0 is None:
                return 0
            eps = 1e-6
            sev = 0

            # Frequency ratio (consider only if baseline is sensible)
            f_ratio = 1.0
            if f0 > eps and f1 is not None:
                f_ratio = float(f1) / float(f0)

            # Amplitude ratio (only increases count)
            a_ratio = 1.0
            if A0 > eps and A1 is not None:
                a_ratio = float(A1) / float(A0)

            # Determine severity by comparing ratios to factors
            if (f_ratio >= MILD_FACTOR) or (a_ratio >= MILD_FACTOR):
                sev = 1
            if (f_ratio >= DAMAGED_FACTOR) or (a_ratio >= DAMAGED_FACTOR):
                sev = 2
            if (f_ratio >= CRITICAL_FACTOR) or (a_ratio >= CRITICAL_FACTOR):
                sev = 3
            return sev

        for sid in (0, 1):
            base_vals = base.get(sid)
            feats = self.latest_feats.get(sid)
            if (not base_vals) or (not feats):
                sev = 0
            else:
                f0, A0 = base_vals
                f1, A1 = feats.get("f1"), feats.get("A1")
                sev = _sev_from_baseline(f0, A0, f1, A1)

            # update history
            self.sev_hist[sid].append(sev)
            # stable severity is the max over the recent window
            stable_sev = max(self.sev_hist[sid]) if len(self.sev_hist[sid]) > 0 else 0
            per_sensor_sev[sid] = stable_sev

        # Combined severity = worst of the two
        combined = max(per_sensor_sev[0], per_sensor_sev[1])

        sev_map = {0: "HEALTHY", 1: "MILD", 2: "DAMAGED", 3: "CRITICAL"}
        color_map = {0: pg.mkColor("g"), 1: pg.mkColor("#ffd54f"), 2: pg.mkColor("#ff8c00"), 3: pg.mkColor("r")}

        left_text = sev_map.get(per_sensor_sev[0], "?")
        right_text = sev_map.get(per_sensor_sev[1], "?")
        combined_text = sev_map.get(combined, "?")

        label = f"{combined_text} — L:{left_text}, R:{right_text}"
        return label, color_map.get(combined, pg.mkColor("w"))


# ----------------------------
# GUI
# ----------------------------
class MainWindow(QtWidgets.QWidget):
    def __init__(self, port):
        super().__init__()
        self.setWindowTitle("Bridge Vibration Monitor (Live + FFT + Baseline)")

        self.model = DataModel()
        self.reader = SerialReader(port, self.on_serial_line)
        self.reader.start()

        # UI layout
        layout = QtWidgets.QVBoxLayout(self)

        # Top controls
        top = QtWidgets.QHBoxLayout()
        self.btn_cal = QtWidgets.QPushButton("CAL (keep still)")
        self.btn_start = QtWidgets.QPushButton("START stream")
        self.btn_stop = QtWidgets.QPushButton("STOP stream")
        self.btn_base = QtWidgets.QPushButton("SET BASELINE (healthy)")
        # Motor power controls
        self.motor_label = QtWidgets.QLabel("Motor Power:")
        self.motor_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.motor_slider.setRange(0, 255)
        self.motor_slider.setValue(0)
        self.motor_spin = QtWidgets.QSpinBox()
        self.motor_spin.setRange(0, 255)
        self.motor_spin.setValue(0)
        self.btn_motor_set = QtWidgets.QPushButton("Set Motor")
        self.btn_motor_enable = QtWidgets.QPushButton("Enable Motor")
        self.btn_motor_enable.setCheckable(True)
        self.chk_safe_send = QtWidgets.QCheckBox("Safe send (STOP→CMD→START)")
        self.chk_safe_send.setChecked(True)
        self.status = QtWidgets.QLabel("STATUS: --")
        self.status.setStyleSheet("font-size: 18px; font-weight: bold;")

        top.addWidget(self.btn_cal)
        top.addWidget(self.btn_start)
        top.addWidget(self.btn_stop)
        top.addWidget(self.btn_base)
        top.addWidget(self.motor_label)
        top.addWidget(self.motor_slider)
        top.addWidget(self.motor_spin)
        top.addWidget(self.btn_motor_set)
        top.addWidget(self.btn_motor_enable)
        top.addWidget(self.chk_safe_send)
        top.addStretch(1)
        top.addWidget(self.status)
        layout.addLayout(top)

        # Plots
        self.timePlot = pg.PlotWidget(title="Time Domain (az raw)")
        self.timePlot.setLabel("left", "az (raw)")
        self.timePlot.setLabel("bottom", "time (s)")
        layout.addWidget(self.timePlot, 1)

        self.fftPlot = pg.PlotWidget(title="FFT Magnitude (az)")
        self.fftPlot.setLabel("left", "|FFT|")
        self.fftPlot.setLabel("bottom", "frequency (Hz)")
        layout.addWidget(self.fftPlot, 1)

        # Curves: show sensor 0/1/2 (with different colors)
        self.timeCurves = {
            0: self.timePlot.plot(pen='r', name='Sensor 0'),  # Red
            1: self.timePlot.plot(pen='g', name='Sensor 1'),  # Green
            2: self.timePlot.plot(pen='b', name='Sensor 2'),  # Blue
        }
        self.fftCurves = {
            0: self.fftPlot.plot(pen='r', name='Sensor 0'),  # Red
            1: self.fftPlot.plot(pen='g', name='Sensor 1'),  # Green
            2: self.fftPlot.plot(pen='b', name='Sensor 2'),  # Blue
        }

        # Buttons connect
        self.btn_cal.clicked.connect(lambda: self.reader.write("CAL"))
        self.btn_start.clicked.connect(lambda: (print("[DEBUG] START button clicked"), self.reader.write("START")))
        self.btn_stop.clicked.connect(lambda: self.reader.write("STOP"))
        self.btn_base.clicked.connect(self.model.set_baseline)
        # motor control connections
        self.motor_slider.valueChanged.connect(self.motor_spin.setValue)
        self.motor_spin.valueChanged.connect(self.motor_slider.setValue)
        self.btn_motor_set.clicked.connect(self.set_motor_power)
        self.btn_motor_enable.toggled.connect(self.toggle_enable_motor)
        # also send when user releases the slider (convenience)
        self.motor_slider.sliderReleased.connect(lambda: self.reader.write(MOTOR_POWER_FMT.format(self.motor_slider.value())))

        # Timers
        self.timeTimer = QtCore.QTimer()
        self.timeTimer.timeout.connect(self.update_time_plot)
        self.timeTimer.start(int(1000 / TIME_UPDATE_HZ))

        self.fftTimer = QtCore.QTimer()
        self.fftTimer.timeout.connect(self.update_fft_and_status)
        self.fftTimer.start(int(1000 / FFT_UPDATE_HZ))

        # Serial debug label
        self.lastMsg = ""
        # Track whether firmware is currently streaming
        self.streaming_flag = False

    def closeEvent(self, event):
        try:
            # Ensure motor is off before exiting
            try:
                # send motor 0 to stop motor
                if getattr(self, "reader", None):
                    try:
                        self.reader.write(MOTOR_POWER_FMT.format(0))
                    except Exception:
                        pass
            except Exception:
                pass
            self.reader.stop()
        except Exception:
            pass
        event.accept()

    def set_motor_power(self):
        try:
            val = int(self.motor_spin.value())
            # Clamp value to slider/spin range
            val = max(0, min(255, val))
            self.motor_slider.setValue(val)
            cmd = MOTOR_POWER_FMT.format(val)
            if self.chk_safe_send.isChecked():
                threading.Thread(target=self._safe_send_commands, args=([cmd], True), daemon=True).start()
            else:
                self.reader.write(cmd)
        except Exception:
            pass

    def toggle_enable_motor(self, checked: bool):
        # Send MOTORON / MOTOROFF safely if requested
        cmd = "MOTORON" if checked else "MOTOROFF"
        # Update button text immediately
        try:
            self.btn_motor_enable.setText("Disable Motor" if checked else "Enable Motor")
        except Exception:
            pass
        if self.chk_safe_send.isChecked():
            threading.Thread(target=self._safe_send_commands, args=([cmd], True), daemon=True).start()
        else:
            self.reader.write(cmd)

    def _safe_send_commands(self, cmds, restart_stream=True):
        # Run in background thread to avoid blocking UI.
        try:
            was_streaming = getattr(self, "streaming_flag", False)
            if was_streaming:
                self.reader.write("STOP")
                time.sleep(0.12)
            for c in cmds:
                self.reader.write(c)
                time.sleep(0.06)
            if was_streaming and restart_stream:
                time.sleep(0.12)
                self.reader.write("START")
        except Exception:
            pass

    def on_serial_line(self, line: str):
        # Non-data lines: BOOT, CAL_OK, etc.
        # Data lines should be: tms,sid,ax,ay,az,gx,gy,gz
        parts = line.split(",")
        if len(parts) != 8:
            self.lastMsg = line
            # Debug: print all non-data lines (including STREAM messages)
            if line:
                print(f"[DEBUG] Non-data line: {line}")
            return
        try:
            tms = int(parts[0])
            sid = int(parts[1])
            ax = int(parts[2]); ay = int(parts[3]); az = int(parts[4])
            gx = int(parts[5]); gy = int(parts[6]); gz = int(parts[7])
        except ValueError:
            self.lastMsg = line
            return

        if sid not in (0, 1, 2):
            return
        self.model.add_sample(tms, sid, ax, ay, az, gx, gy, gz)
        # Debug: print first few data samples
        if len(self.model.buffers[sid]) <= 3:
            print(f"[DEBUG] Received data: sensor {sid}, samples: {len(self.model.buffers[sid])}")

    def update_time_plot(self):
        # Show last ~5 seconds
        for sid in (0, 1, 2):
            series = self.model.get_series(sid)
            if series is None:
                continue
            t, az = series
            if len(t) < 2:  # Changed from 5 to 2 - show data sooner
                continue
            # Show all data if less than 5 seconds, otherwise last 5 seconds
            if len(t) > 0:
                t0 = max(t[0], t[-1] - 5.0)
                mask = t >= t0
                self.timeCurves[sid].setData(t[mask], az[mask])

    def update_fft_and_status(self):
        # Compute FFT features + update plot
        for sid in (0, 1, 2):
            feats = self.model.compute_fft_features(sid)
            if feats is None:
                continue
            self.model.latest_feats[sid] = feats
            self.fftCurves[sid].setData(feats["freqs"], feats["mag"])

        st, color = self.model.health_status()
        self.status.setText(f"STATUS: {st}")
        # Set label color
        self.status.setStyleSheet(f"font-size: 18px; font-weight: bold; color: {color.name()};")


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 bridge_live.py /dev/tty.usbmodemXXXX")
        sys.exit(1)

    port = sys.argv[1]
    app = QtWidgets.QApplication([])
    w = MainWindow(port)
    w.resize(1200, 700)
    w.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
