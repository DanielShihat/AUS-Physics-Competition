import time
import threading
from collections import deque

import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets

from sim_bridge_source import SimBridgeSource

# ----------------------------
# Config
# ----------------------------
MAX_SECONDS = 10
DEFAULT_FS_EST = 200
FFT_WINDOW_SECONDS = 4.0
FFT_UPDATE_HZ = 2.0
TIME_UPDATE_HZ = 30.0

DF_THRESHOLD_HZ = 0.30
DA_THRESHOLD_REL = 0.25
CONSEC_WINDOWS = 2

# ----------------------------
# Data model (same logic)
# ----------------------------
class DataModel:
    def __init__(self):
        self.buffers = {0: deque(), 1: deque(), 2: deque()}
        self.last_t = {0: None, 1: None, 2: None}
        self.dt_hist = {0: deque(maxlen=400), 1: deque(maxlen=400), 2: deque(maxlen=400)}
        self.baseline = None
        self.consec_bad = 0
        self.latest_feats = {}
        self.lock = threading.Lock()

    def add_sample(self, tsec, sid, ax, ay, az, gx, gy, gz):
        with self.lock:
            self.buffers[sid].append((tsec, az))
            cutoff = tsec - MAX_SECONDS
            while self.buffers[sid] and self.buffers[sid][0][0] < cutoff:
                self.buffers[sid].popleft()

            lt = self.last_t[sid]
            if lt is not None:
                dt = tsec - lt
                if 0.0 < dt < 0.2:
                    self.dt_hist[sid].append(dt)
            self.last_t[sid] = tsec

    def estimate_fs(self, sid):
        with self.lock:
            dts = list(self.dt_hist[sid])
        if len(dts) < 20:
            return DEFAULT_FS_EST
        med = np.median(dts)
        return DEFAULT_FS_EST if med <= 0 else float(1.0 / med)

    def get_series(self, sid):
        with self.lock:
            data = list(self.buffers[sid])
        if not data:
            return None
        arr = np.array(data, dtype=np.float64)
        return arr[:, 0], arr[:, 1]

    def compute_fft_features(self, sid, fmin=1.0, fmax=40.0):
        series = self.get_series(sid)
        if series is None:
            return None
        t, az = series
        fs = self.estimate_fs(sid)

        t_end = t[-1]
        t_start = t_end - FFT_WINDOW_SECONDS
        mask = t >= t_start
        if np.sum(mask) < int(FFT_WINDOW_SECONDS * fs * 0.6):
            return None

        x = az[mask]
        x = x - np.mean(x)
        w = np.hanning(len(x))
        X = np.fft.rfft(x * w)
        freqs = np.fft.rfftfreq(len(x), d=1.0/fs)
        mag = np.abs(X)

        band = (freqs >= fmin) & (freqs <= fmax)
        if not np.any(band):
            return None

        freqs_b = freqs[band]
        mag_b = mag[band]
        idx = int(np.argmax(mag_b))

        return {
            "fs": fs,
            "freqs": freqs_b,
            "mag": mag_b,
            "f1": float(freqs_b[idx]),
            "A1": float(mag_b[idx])
        }

    def set_baseline(self):
        base = {}
        for sid in (0, 1, 2):
            feats = self.compute_fft_features(sid)
            if feats:
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

        bad = False
        for sid, (f0, A0) in base.items():
            feats = self.latest_feats.get(sid)
            if not feats:
                continue
            df = abs(feats["f1"] - f0)
            da = abs(feats["A1"] - A0) / (A0 + 1e-9)
            if (df > DF_THRESHOLD_HZ) or (da > DA_THRESHOLD_REL):
                bad = True

        consec = consec + 1 if bad else max(0, consec - 1)
        with self.lock:
            self.consec_bad = consec

        if consec >= CONSEC_WINDOWS:
            return "DAMAGED", pg.mkColor("r")
        return "HEALTHY", pg.mkColor("g")


# ----------------------------
# Simulator reader thread
# ----------------------------
class SimReader(threading.Thread):
    def __init__(self, sim: SimBridgeSource, on_line):
        super().__init__(daemon=True)
        self.sim = sim
        self.on_line = on_line
        self._stop = threading.Event()

    def stop(self):
        self._stop.set()

    def run(self):
        while not self._stop.is_set():
            line = self.sim.readline().decode("utf-8").strip()
            if line:
                self.on_line(line)


# ----------------------------
# GUI
# ----------------------------
class MainWindow(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Bridge Monitor (SIMULATION)")

        self.model = DataModel()
        self.sim = SimBridgeSource(sensors=3, mode="shaker", fs=200)
        self.reader = SimReader(self.sim, self.on_line)
        self.reader.start()

        layout = QtWidgets.QVBoxLayout(self)

        # Controls
        top = QtWidgets.QHBoxLayout()
        self.btn_mode = QtWidgets.QPushButton("Mode: SHAKER")
        self.btn_d0 = QtWidgets.QPushButton("Damage 0 (Healthy)")
        self.btn_d1 = QtWidgets.QPushButton("Damage 1")
        self.btn_d2 = QtWidgets.QPushButton("Damage 2")
        self.btn_base = QtWidgets.QPushButton("SET BASELINE")
        self.status = QtWidgets.QLabel("STATUS: --")
        self.status.setStyleSheet("font-size: 18px; font-weight: bold;")

        top.addWidget(self.btn_mode)
        top.addWidget(self.btn_d0)
        top.addWidget(self.btn_d1)
        top.addWidget(self.btn_d2)
        top.addWidget(self.btn_base)
        top.addStretch(1)
        top.addWidget(self.status)
        layout.addLayout(top)

        # Plots
        self.timePlot = pg.PlotWidget(title="Time Domain (az raw)")
        self.timePlot.setLabel("left", "az")
        self.timePlot.setLabel("bottom", "time (s)")
        layout.addWidget(self.timePlot, 1)

        self.fftPlot = pg.PlotWidget(title="FFT Magnitude (az)")
        self.fftPlot.setLabel("left", "|FFT|")
        self.fftPlot.setLabel("bottom", "frequency (Hz)")
        layout.addWidget(self.fftPlot, 1)

        self.timeCurves = {sid: self.timePlot.plot() for sid in (0,1,2)}
        self.fftCurves = {sid: self.fftPlot.plot() for sid in (0,1,2)}

        # Connect buttons
        self.btn_mode.clicked.connect(self.toggle_mode)
        self.btn_d0.clicked.connect(lambda: self.sim.set_damage(0))
        self.btn_d1.clicked.connect(lambda: self.sim.set_damage(1))
        self.btn_d2.clicked.connect(lambda: self.sim.set_damage(2))
        self.btn_base.clicked.connect(self.model.set_baseline)

        # Timers
        self.timeTimer = QtCore.QTimer()
        self.timeTimer.timeout.connect(self.update_time)
        self.timeTimer.start(int(1000 / TIME_UPDATE_HZ))

        self.fftTimer = QtCore.QTimer()
        self.fftTimer.timeout.connect(self.update_fft_status)
        self.fftTimer.start(int(1000 / FFT_UPDATE_HZ))

    def closeEvent(self, event):
        try:
            self.reader.stop()
        except Exception:
            pass
        event.accept()

    def toggle_mode(self):
        if self.sim.mode == "shaker":
            self.sim.set_mode("tap")
            self.btn_mode.setText("Mode: TAP")
        else:
            self.sim.set_mode("shaker")
            self.btn_mode.setText("Mode: SHAKER")

    def on_line(self, line: str):
        parts = line.split(",")
        if len(parts) != 8:
            return
        try:
            # use PC time for smooth monotonic display
            tsec = time.time()
            sid = int(parts[1])
            az = int(parts[4])
            ax = int(parts[2]); ay = int(parts[3])
            gx = int(parts[5]); gy = int(parts[6]); gz = int(parts[7])
        except ValueError:
            return

        if sid not in (0,1,2):
            return
        self.model.add_sample(tsec, sid, ax, ay, az, gx, gy, gz)

    def update_time(self):
        for sid in (0,1,2):
            s = self.model.get_series(sid)
            if not s:
                continue
            t, az = s
            if len(t) < 5:
                continue
            t0 = t[-1] - 5.0
            mask = t >= t0
            self.timeCurves[sid].setData(t[mask] - t[mask][0], az[mask])

    def update_fft_status(self):
        for sid in (0,1,2):
            feats = self.model.compute_fft_features(sid)
            if not feats:
                continue
            self.model.latest_feats[sid] = feats
            self.fftCurves[sid].setData(feats["freqs"], feats["mag"])

        st, color = self.model.health_status()
        self.status.setText(f"STATUS: {st}")
        self.status.setStyleSheet(f"font-size: 18px; font-weight: bold; color: {color.name()};")


def main():
    app = QtWidgets.QApplication([])
    w = MainWindow()
    w.resize(1200, 700)
    w.show()
    app.exec_()

if __name__ == "__main__":
    main()
