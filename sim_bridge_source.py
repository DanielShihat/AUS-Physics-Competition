import time
import math
import random

class SimBridgeSource:
    """
    Produces Arduino-like CSV lines:
    tms,sid,ax,ay,az,gx,gy,gz

    Healthy vs damaged changes:
    - resonance frequency shifts downward
    - amplitude decreases
    - damping increases (faster decay in tap mode)
    """
    def __init__(self, sensors=3, mode="shaker", fs=200):
        self.sensors = sensors
        self.mode = mode          # "shaker" or "tap"
        self.fs = fs
        self.dt = 1.0 / fs
        self.t0 = time.time()
        self.sample_index = 0

        # State
        self.damaged = False
        self.damage_level = 0  # 0,1,2

        # Base resonance per sensor (slightly different)
        self.f_base = [8.5, 9.0, 9.3]  # Hz
        self.f_damage_shift = [-0.6, -0.9, -1.2]  # level 1 shift
        self.f_damage_shift2 = [-1.0, -1.5, -2.0] # level 2 shift

        # Amplitudes per sensor
        self.A_base = [1400, 1800, 1600]  # raw az amplitude scale
        self.noise = 120

        # Tap decay
        self.tap_start_t = None

    def set_damage(self, level: int):
        self.damage_level = max(0, min(2, level))
        self.damaged = (self.damage_level > 0)

    def set_mode(self, mode: str):
        assert mode in ("shaker", "tap")
        self.mode = mode
        if mode == "tap":
            self.tap_start_t = None

    def _current_t(self):
        return self.sample_index * self.dt

    def _freq_for(self, sid):
        f = self.f_base[sid]
        if self.damage_level == 1:
            f += self.f_damage_shift[sid]
        elif self.damage_level == 2:
            f += self.f_damage_shift2[sid]
        return max(1.0, f)

    def _amp_for(self, sid):
        A = self.A_base[sid]
        if self.damage_level == 1:
            A *= 0.75
        elif self.damage_level == 2:
            A *= 0.55
        return A

    def _tap_envelope(self, t):
        # Healthy decays slower; damaged decays faster
        if self.tap_start_t is None:
            self.tap_start_t = t
        tau = 1.2 if self.damage_level == 0 else (0.7 if self.damage_level == 1 else 0.45)
        return math.exp(-(t - self.tap_start_t) / tau)

    def readline(self):
        """
        Return one line (bytes) at a time, like serial.readline().
        """
        # pace it like real time
        target = self.t0 + self.sample_index * self.dt
        now = time.time()
        if now < target:
            time.sleep(target - now)

        t = self._current_t()
        tms = int((time.time()) * 1000)

        # cycle through sensors each tick to mimic Arduino sending 0/1/2 each loop
        sid = self.sample_index % self.sensors

        f = self._freq_for(sid)
        A = self._amp_for(sid)

        if self.mode == "shaker":
            # steady-state sinusoid + small harmonics
            az = A * math.sin(2 * math.pi * f * t) + 0.25 * A * math.sin(2 * math.pi * 2*f * t)
        else:
            # tap: decaying sinusoid
            env = self._tap_envelope(t)
            az = (A * 1.2) * env * math.sin(2 * math.pi * f * t)

        # noise + small drift
        az += random.uniform(-self.noise, self.noise)
        az += 30 * math.sin(2 * math.pi * 0.2 * t)

        # mock other axes / gyro (mostly noise)
        ax = random.randint(-80, 80)
        ay = random.randint(-80, 80)
        gx = random.randint(-8, 8)
        gy = random.randint(-8, 8)
        gz = random.randint(-8, 8)

        self.sample_index += 1

        line = f"{tms},{sid},{ax},{ay},{int(az)},{gx},{gy},{gz}\n"
        return line.encode("utf-8")
