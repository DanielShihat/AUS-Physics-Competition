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

MAX_SECONDS = 10            # rolling buffer length
DEFAULT_FS_EST = 200        # used for FFT windowing; we estimate real FS too

FFT_WINDOW_SECONDS = 2.0    # use last 2 seconds for FFT (faster detection)
FFT_UPDATE_HZ = 5.0         # update FFT 5x per second (faster damage detection)
TIME_UPDATE_HZ = 20.0       # was 30.0; lower = less UI load

# ---- Robustness / Stability knobs ----
QUIET_NON_DATA = True        # set False if you really want debug spam
PRINT_EVERY_N_DROP = 100     # print 1 per N drops (only if QUIET_NON_DATA=False)

# Smoothing (EMA): larger = more responsive, less smoothing
EMA_ALPHA_F = 0.35   # f1 smoothing (increased for faster response)
EMA_ALPHA_A = 0.35   # A1 smoothing (increased for faster response)

# Baseline averaging: how many FFT windows to average
BASELINE_TARGET_WINDOWS = 5  # reduced from 10 for faster baseline establishment

# Status smoothing: require sustained deviation
CONSEC_WINDOWS = 2  # reduced from 5 for faster detection (2 consecutive bad windows)

# Classification thresholds (amplitude change relative to baseline)
# da = |A1 - A0| / A0
DA_HEALTHY_MAX = 0.35   # <35% change = healthy (less sensitive than before)
DA_MILD_MAX    = 0.70   # 35–70% = mild
DA_DAMAGED_MAX = 1.20   # 70–120% = damaged
# >120% = critical


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
        try:
            self.ser.write((s.strip() + "\n").encode("utf-8"))
            self.ser.flush()
        except Exception:
            pass

    def run(self):
        self.ser = serial.Serial(self.port, BAUD, timeout=1)
        time.sleep(1.0)
        self.ser.reset_input_buffer()

        # Start clean
        self.write("STOP")
        time.sleep(0.1)
        self.write("STATUS")

        while not self._stop.is_set():
            try:
                line = self.ser.readline().decode("utf-8", errors="ignore").strip()
                if line:
                    self.on_line(line)
            except Exception:
                continue


# ----------------------------
# Data model
# ----------------------------
class DataModel:
    def __init__(self):
        self.buffers = {0: deque(), 1: deque(), 2: deque()}
        self.maxlen = int(MAX_SECONDS * DEFAULT_FS_EST * 2)

        # FS estimate
        self.last_tms = {0: None, 1: None, 2: None}
        self.dt_hist = {0: deque(maxlen=400), 1: deque(maxlen=400), 2: deque(maxlen=400)}

        # Smoothed features (EMA)
        self.smooth = {}  # sid -> {"f1":..., "A1":...}
        self.alpha_f = EMA_ALPHA_F
        self.alpha_A = EMA_ALPHA_A

        # Baseline
        self.baseline = None             # dict: sid -> (f0, A0)
        self.baseline_collecting = False
        self.baseline_samples = {0: [], 1: [], 2: []}
        self.baseline_target = BASELINE_TARGET_WINDOWS

        # Latest computed features
        self.latest_feats = {}  # sid -> dict

        # Status smoothing
        self.consec_bad = 0

        # Thread safety
        self.lock = threading.Lock()

    def add_sample(self, tms, sid, ax, ay, az, gx, gy, gz):
        with self.lock:
            t = tms / 1000.0
            self.buffers[sid].append((t, ax, ay, az, gx, gy, gz))

            cutoff = t - MAX_SECONDS
            while self.buffers[sid] and self.buffers[sid][0][0] < cutoff:
                self.buffers[sid].popleft()

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
        az = arr[:, 3]
        return t, az

    def compute_fft_features(self, sid, fmin=1.0, fmax=40.0):
        series = self.get_series(sid)
        if series is None:
            return None
        t, az = series
        fs = self.estimate_fs(sid)

        window = FFT_WINDOW_SECONDS
        t_end = t[-1]
        t_start = t_end - window
        mask = t >= t_start
        if np.sum(mask) < int(window * fs * 0.6):
            return None

        x = az[mask].astype(np.float64)
        x = x - np.mean(x)

        w = np.hanning(len(x))
        xw = x * w

        X = np.fft.rfft(xw)
        freqs = np.fft.rfftfreq(len(xw), d=1.0 / fs)
        mag = np.abs(X)

        band = (freqs >= fmin) & (freqs <= fmax)
        if not np.any(band):
            return None
        freqs_b = freqs[band]
        mag_b = mag[band]

        idx = int(np.argmax(mag_b))
        f1 = float(freqs_b[idx])
        A1 = float(mag_b[idx])

        return {"fs": fs, "freqs": freqs_b, "mag": mag_b, "f1": f1, "A1": A1}

    def update_smoothed(self, sid, feats):
        """EMA smooth f1/A1 so tiny FFT jitters don't flip the status."""
        f1 = feats["f1"]
        A1 = feats["A1"]

        with self.lock:
            s = self.smooth.get(sid)
            if s is None:
                self.smooth[sid] = {"f1": f1, "A1": A1}
                return dict(self.smooth[sid])

            s["f1"] = (1 - self.alpha_f) * s["f1"] + self.alpha_f * f1
            s["A1"] = (1 - self.alpha_A) * s["A1"] + self.alpha_A * A1
            return dict(s)

    def start_baseline(self):
        """Start collecting baseline over multiple FFT windows."""
        with self.lock:
            self.baseline = None
            self.consec_bad = 0
            self.baseline_collecting = True
            for sid in (0, 1, 2):
                self.baseline_samples[sid] = []

    def baseline_progress(self, sids=(0, 1)):
        with self.lock:
            if not self.baseline_collecting and self.baseline:
                return (self.baseline_target, self.baseline_target)
            counts = [len(self.baseline_samples[s]) for s in sids]
        return (min(counts) if counts else 0, self.baseline_target)

    def maybe_collect_baseline(self, sid, f1_s, A1_s, use_sids=(0, 1)):
        """Collect smoothed features until we have enough windows, then average baseline."""
        with self.lock:
            if not self.baseline_collecting:
                return

            if sid in use_sids and len(self.baseline_samples[sid]) < self.baseline_target:
                self.baseline_samples[sid].append((f1_s, A1_s))

            ready = all(len(self.baseline_samples[s]) >= self.baseline_target for s in use_sids)
            if ready:
                base = {}
                for s in use_sids:
                    arr = np.array(self.baseline_samples[s], dtype=np.float64)
                    f0 = float(np.mean(arr[:, 0]))
                    A0 = float(np.mean(arr[:, 1]))
                    base[s] = (f0, A0)
                self.baseline = base
                self.consec_bad = 0
                self.baseline_collecting = False

    def health_status(self):
        with self.lock:
            base = self.baseline
            consec = self.consec_bad
            latest = dict(self.latest_feats)
            collecting = self.baseline_collecting

        if collecting and not base:
            return "BASELINE CAPTURING", pg.mkColor("w"), {0: "n/a", 1: "n/a", 2: "n/a"}

        if not base:
            return "NO BASELINE", pg.mkColor("w"), {0: "n/a", 1: "n/a", 2: "n/a"}

        def classify(da):
            if da < DA_HEALTHY_MAX:
                return 0
            elif da < DA_MILD_MAX:
                return 1
            elif da < DA_DAMAGED_MAX:
                return 2
            else:
                return 3

        per_sid_level = {}
        worst_level = 0

        for sid, (f0, A0) in base.items():
            feats = latest.get(sid)
            if not feats:
                continue

            da = abs(feats["A1"] - A0) / (A0 + 1e-9)
            lvl = classify(da)

            per_sid_level[sid] = lvl
            worst_level = max(worst_level, lvl)

        # smooth status changes over time
        if worst_level > 0:
            consec += 1
        else:
            consec = max(0, consec - 1)

        with self.lock:
            self.consec_bad = consec

        level_to_overall = {
            0: ("HEALTHY", pg.mkColor("g")),
            1: ("MILD", pg.mkColor("y")),
            2: ("DAMAGED", pg.mkColor("orange")),
            3: ("CRITICAL", pg.mkColor("r")),
        }
        level_to_label = {0: "healthy", 1: "mild", 2: "damaged", 3: "critical"}

        if consec >= CONSEC_WINDOWS and worst_level > 0:
            overall_level = worst_level
        else:
            overall_level = 0

        overall_text, overall_color = level_to_overall[overall_level]

        per_sid_labels = {}
        for sid in (0, 1, 2):
            lvl = per_sid_level.get(sid, None)
            per_sid_labels[sid] = "n/a" if lvl is None else level_to_label[lvl]

        return overall_text, overall_color, per_sid_labels


# ----------------------------
# GUI
# ----------------------------
class MainWindow(QtWidgets.QWidget):
    def __init__(self, port):
        super().__init__()
        self.setWindowTitle("Bridge Vibration Monitor (Live + FFT + Smoothed Baseline)")

        self.model = DataModel()
        self.reader = SerialReader(port, self.on_serial_line)
        self.reader.start()

        # log controls
        self.drop_count = 0
        self.last_nondata_print = 0.0

        layout = QtWidgets.QVBoxLayout(self)

        top = QtWidgets.QHBoxLayout()
        self.btn_cal = QtWidgets.QPushButton("CAL (keep still)")
        self.btn_start = QtWidgets.QPushButton("START stream")
        self.btn_stop = QtWidgets.QPushButton("STOP stream")
        self.btn_base = QtWidgets.QPushButton("SET BASELINE (avg)")
        self.status = QtWidgets.QLabel("STATUS: --")
        self.status.setStyleSheet("font-size: 18px; font-weight: bold;")

        top.addWidget(self.btn_cal)
        top.addWidget(self.btn_start)
        top.addWidget(self.btn_stop)
        top.addWidget(self.btn_base)
        top.addStretch(1)
        top.addWidget(self.status)
        layout.addLayout(top)

        # Motor controls
        motor_layout = QtWidgets.QHBoxLayout()
        self.chk_motor = QtWidgets.QCheckBox("Motor enabled")
        self.motorSlider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.motorSlider.setRange(0, 255)
        self.motorSlider.setValue(0)
        self.motorLabel = QtWidgets.QLabel("Power: 0")
        motor_layout.addWidget(self.chk_motor)
        motor_layout.addWidget(self.motorSlider)
        motor_layout.addWidget(self.motorLabel)
        layout.addLayout(motor_layout)

        # Plots
        self.timePlot = pg.PlotWidget(title="Time Domain (az raw)")
        self.timePlot.setLabel("left", "az (raw)")
        self.timePlot.setLabel("bottom", "time (s)")
        layout.addWidget(self.timePlot, 1)

        self.fftPlot = pg.PlotWidget(title="FFT Magnitude (az)")
        self.fftPlot.setLabel("left", "|FFT|")
        self.fftPlot.setLabel("bottom", "frequency (Hz)")
        layout.addWidget(self.fftPlot, 1)

        self.timeCurves = {
            0: self.timePlot.plot(pen='r', name='Sensor 0'),
            1: self.timePlot.plot(pen='g', name='Sensor 1'),
            2: self.timePlot.plot(pen='b', name='Sensor 2'),
        }
        self.fftCurves = {
            0: self.fftPlot.plot(pen='r', name='Sensor 0'),
            1: self.fftPlot.plot(pen='g', name='Sensor 1'),
            2: self.fftPlot.plot(pen='b', name='Sensor 2'),
        }

        # Button hooks
        self.btn_cal.clicked.connect(lambda: self.reader.write("CAL"))
        self.btn_start.clicked.connect(lambda: (print("[DEBUG] START button clicked"), self.reader.write("START")))
        self.btn_stop.clicked.connect(lambda: self.reader.write("STOP"))
        self.btn_base.clicked.connect(self.model.start_baseline)

        self.chk_motor.toggled.connect(self.on_motor_toggled)
        self.motorSlider.valueChanged.connect(self.on_motor_slider)
        self.motorSlider.sliderReleased.connect(self.on_motor_slider_released)

        # Timers
        self.timeTimer = QtCore.QTimer()
        self.timeTimer.timeout.connect(self.update_time_plot)
        self.timeTimer.start(int(1000 / TIME_UPDATE_HZ))

        self.fftTimer = QtCore.QTimer()
        self.fftTimer.timeout.connect(self.update_fft_and_status)
        self.fftTimer.start(int(1000 / FFT_UPDATE_HZ))

        self.lastMsg = ""

    def closeEvent(self, event):
        try:
            self.reader.stop()
        except Exception:
            pass
        event.accept()

    # ----------------------------
    # Motor control callbacks
    # ----------------------------
    def on_motor_toggled(self, checked: bool):
        if checked:
            val = self.motorSlider.value()
            self.reader.write("MOTORON")
            self.reader.write(f"MOTOR {val}")
        else:
            self.reader.write("MOTOROFF")

    def on_motor_slider(self, val: int):
        self.motorLabel.setText(f"Power: {val}")

    def on_motor_slider_released(self):
        if self.chk_motor.isChecked():
            val = self.motorSlider.value()
            self.reader.write(f"MOTOR {val}")

    # ----------------------------
    # Serial parsing (less spam)
    # ----------------------------
    def on_serial_line(self, line: str):
        parts = line.split(",")

        # Non-data lines
        if len(parts) != 8:
            self.lastMsg = line

            # tame the spam so UI doesn't "buffer"
            if line.startswith("DROP"):
                self.drop_count += 1
                if (not QUIET_NON_DATA) and (self.drop_count % PRINT_EVERY_N_DROP == 0):
                    print(f"[DEBUG] DROP x{self.drop_count} (latest: {line})")
                return

            if line.startswith("MPU_RECOVER"):
                if not QUIET_NON_DATA:
                    now = time.time()
                    if now - self.last_nondata_print > 0.5:
                        print(f"[DEBUG] {line}")
                        self.last_nondata_print = now
                return

            if line and not line.startswith("STREAM"):
                if not QUIET_NON_DATA:
                    print(f"[DEBUG] Non-data line: {line}")
            return

        # Data line: tms,sid,ax,ay,az,gx,gy,gz
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

        if not QUIET_NON_DATA:
            with self.model.lock:
                if len(self.model.buffers[sid]) <= 3:
                    print(f"[DEBUG] Received data: sensor {sid}, samples: {len(self.model.buffers[sid])}")

    # ----------------------------
    # Plot updates
    # ----------------------------
    def update_time_plot(self):
        for sid in (0, 1, 2):
            series = self.model.get_series(sid)
            if series is None:
                continue
            t, az = series
            if len(t) < 2:
                continue
            t0 = max(t[0], t[-1] - 5.0)
            mask = t >= t0
            self.timeCurves[sid].setData(t[mask], az[mask])

    def update_fft_and_status(self):
        # Update FFT plot + smoothed features
        for sid in (0, 1, 2):
            feats = self.model.compute_fft_features(sid)
            if feats is None:
                continue

            # plot raw spectrum
            self.fftCurves[sid].setData(feats["freqs"], feats["mag"])

            # smooth f1/A1 for decisions
            s = self.model.update_smoothed(sid, feats)
            feats_sm = dict(feats)
            feats_sm["f1"] = s["f1"]
            feats_sm["A1"] = s["A1"]

            with self.model.lock:
                self.model.latest_feats[sid] = feats_sm

            # collect baseline (average over multiple FFT windows)
            self.model.maybe_collect_baseline(sid, feats_sm["f1"], feats_sm["A1"], use_sids=(0, 1))

        overall, color, per_sid = self.model.health_status()
        left = per_sid.get(0, "n/a")
        right = per_sid.get(1, "n/a")

        # baseline progress text
        got, need = self.model.baseline_progress((0, 1))
        if overall == "BASELINE CAPTURING":
            msg = f"STATUS: {overall} ({got}/{need}) | LEFT: n/a | RIGHT: n/a"
        else:
            msg = f"STATUS: {overall} | LEFT: {left} | RIGHT: {right}"

        self.status.setText(msg)
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
