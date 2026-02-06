# to run: python3 bridge_live.py /dev/tty.usbmodem1101


import sys
import time
import threading
from collections import dequ

import numpy as np
import serial
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets

# ----------------------------
# Config
# ----------------------------
BAUD = 460800
MAX_SECONDS = 10           # rolling buffer length
DEFAULT_FS_EST = 200       # used for FFT windowing; we estimate real FS too
FFT_WINDOW_SECONDS = 4.0   # use last 4 seconds for FFT
FFT_UPDATE_HZ = 2.0        # FFT recompute rate
TIME_UPDATE_HZ = 30.0      # GUI refresh rate

# Detection thresholds (tune after first run)
DF_THRESHOLD_HZ = 0.30
DA_THRESHOLD_REL = 0.25
CONSEC_WINDOWS = 2

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
        self.ser.write((s.strip() + "\n").encode("utf-8"))

    def run(self):
        self.ser = serial.Serial(self.port, BAUD, timeout=1)
        # Small settle time
        time.sleep(1.0)

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
            consec = self.consec_bad
        if not base:
            return "NO BASELINE", pg.mkColor("w")

        # If any sensor deviates enough, count as bad
        bad = False
        for sid, (f0, A0) in base.items():
            feats = self.latest_feats.get(sid)
            if not feats:
                continue
            df = abs(feats["f1"] - f0)
            da = abs(feats["A1"] - A0) / (A0 + 1e-9)
            if (df > DF_THRESHOLD_HZ) or (da > DA_THRESHOLD_REL):
                bad = True

        if bad:
            consec += 1
        else:
            consec = max(0, consec - 1)

        with self.lock:
            self.consec_bad = consec

        if consec >= CONSEC_WINDOWS:
            return "DAMAGED", pg.mkColor("r")
        return "HEALTHY", pg.mkColor("g")


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
        self.status = QtWidgets.QLabel("STATUS: --")
        self.status.setStyleSheet("font-size: 18px; font-weight: bold;")

        top.addWidget(self.btn_cal)
        top.addWidget(self.btn_start)
        top.addWidget(self.btn_stop)
        top.addWidget(self.btn_base)
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

        # Curves: show sensor 0/1/2
        self.timeCurves = {
            0: self.timePlot.plot(pen=None, symbol=None),
            1: self.timePlot.plot(pen=None, symbol=None),
            2: self.timePlot.plot(pen=None, symbol=None),
        }
        self.fftCurves = {
            0: self.fftPlot.plot(pen=None, symbol=None),
            1: self.fftPlot.plot(pen=None, symbol=None),
            2: self.fftPlot.plot(pen=None, symbol=None),
        }

        # Buttons connect
        self.btn_cal.clicked.connect(lambda: self.reader.write("CAL"))
        self.btn_start.clicked.connect(lambda: self.reader.write("START"))
        self.btn_stop.clicked.connect(lambda: self.reader.write("STOP"))
        self.btn_base.clicked.connect(self.model.set_baseline)

        # Timers
        self.timeTimer = QtCore.QTimer()
        self.timeTimer.timeout.connect(self.update_time_plot)
        self.timeTimer.start(int(1000 / TIME_UPDATE_HZ))

        self.fftTimer = QtCore.QTimer()
        self.fftTimer.timeout.connect(self.update_fft_and_status)
        self.fftTimer.start(int(1000 / FFT_UPDATE_HZ))

        # Serial debug label
        self.lastMsg = ""

    def closeEvent(self, event):
        try:
            self.reader.stop()
        except Exception:
            pass
        event.accept()

    def on_serial_line(self, line: str):
        # Non-data lines: BOOT, CAL_OK, etc.
        # Data lines should be: tms,sid,ax,ay,az,gx,gy,gz
        parts = line.split(",")
        if len(parts) != 8:
            self.lastMsg = line
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

    def update_time_plot(self):
        # Show last ~5 seconds
        for sid in (0, 1, 2):
            series = self.model.get_series(sid)
            if series is None:
                continue
            t, az = series
            if len(t) < 5:
                continue
            t0 = t[-1] - 5.0
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
