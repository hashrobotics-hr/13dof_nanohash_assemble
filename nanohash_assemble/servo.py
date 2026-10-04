import math
import time
from pca9685 import PCA9685


class Servos:
    """
    High-level angle control for up to 16 servos via a PCA9685.
    """

    def __init__(self, i2c, address=0x40, freq=50, min_us=520, max_us=2480,
                 degrees=180, n_channels=16):
        self.period = 1_000_000 / freq
        self.min_duty = self._us2duty(min_us)
        self.max_duty = self._us2duty(max_us)
        self.degrees = degrees
        self.freq = freq
        self.n_channels = n_channels
        self.pca9685 = PCA9685(i2c, address)
        self.pca9685.freq(freq)
        # last known commanded angle per channel, used as the "start" point
        # for timed moves
        self._servopos = [90] * 16

    # ---------- helpers ----------

    def _us2duty(self, value):
        return int(4095 * value / self.period)

    def _check_index(self, index):
        if not 0 <= index <= 15:
            raise ValueError("channel index must be 0-15, got {}".format(index))

    # ---------- low level position ----------

    def position(self, index, degrees=None, radians=None, us=None, duty=None):
        """Directly set (or read) a channel's position. No timing/interpolation."""
        self._check_index(index)
        span = self.max_duty - self.min_duty
        if degrees is not None:
            duty = self.min_duty + span * degrees / self.degrees
        elif radians is not None:
            duty = self.min_duty + span * radians / math.radians(self.degrees)
        elif us is not None:
            duty = self._us2duty(us)
        elif duty is not None:
            pass
        else:
            return self.pca9685.duty(index)
        duty = min(self.max_duty, max(self.min_duty, int(duty)))
        self.pca9685.duty(index, duty)

    def release(self, index):
        """De-energise a single channel (servo goes limp)."""
        self._check_index(index)
        self.pca9685.duty(index, 0)

    def release_all(self):
        self.pca9685.release_all()

    # ---------- main API ----------

    def move_servo(self, index, angle, time_ms=0):
        """
        Move a single servo to `angle` degrees.
        If time_ms > 0, the move is interpolated smoothly over that duration.
        """
        self._check_index(index)
        angle = max(0, min(self.degrees, angle))

        if time_ms <= 0:
            self.position(index, degrees=angle)
            self._servopos[index] = angle
            return

        start_angle = self._servopos[index]
        start_time = time.ticks_ms()
        step_delay = 20  # ms between updates
        steps = max(1, time_ms // step_delay)

        for step in range(1, steps + 1):
            frac = step / steps
            current = start_angle + (angle - start_angle) * frac
            self.position(index, degrees=current)
            target_elapsed = int(time_ms * step / steps)
            while time.ticks_diff(time.ticks_ms(), start_time) < target_elapsed:
                pass

        self._servopos[index] = angle

    def move_all(self, angles, time_ms=0):
        """
        Move several servos together, synchronised over `time_ms`.
        `angles` is a list indexed by channel (0-15). Use None for a channel
        you want left alone.
        If time_ms <= 0, all channels snap instantly to their targets.
        """
        n = len(angles)
        if n > self.n_channels:
            raise ValueError("angles list longer than n_channels ({})".format(self.n_channels))

        clamped = []
        for a in angles:
            if a is None:
                clamped.append(None)
            else:
                clamped.append(max(0, min(self.degrees, a)))

        if time_ms <= 0:
            for i, a in enumerate(clamped):
                if a is not None:
                    self.position(i, degrees=a)
                    self._servopos[i] = a
            return

        start_angles = list(self._servopos[:n])
        start_time = time.ticks_ms()
        step_delay = 20  # ms between updates; raise this if the I2C bus/CPU can't keep up
        steps = max(1, time_ms // step_delay)

        for step in range(1, steps + 1):
            frac = step / steps
            for i in range(n):
                if clamped[i] is None:
                    continue
                current = start_angles[i] + (clamped[i] - start_angles[i]) * frac
                self.position(i, degrees=current)
            target_elapsed = int(time_ms * step / steps)
            while time.ticks_diff(time.ticks_ms(), start_time) < target_elapsed:
                pass

        for i, a in enumerate(clamped):
            if a is not None:
                self._servopos[i] = a

    # ---------- convenience ----------

    def initial_position(self, angles=None, time_ms=2000):
        """Move to a known rest pose. Pass a custom 16-entry list if needed."""
        if angles is None:
            # RH1,RH2,RL1,RL2,LH1,LH2,LL1,LL2,HEAD,DMY1,DMY2,DMY3,DMY4,DMY5,DMY6
            angles = [90, 90, 90, 90, 90, 90, 90, 0, 0, 90, 180, 180, 90, 0, 0, 0]
        self.move_all(angles, time_ms)

    # ---------- backwards-compatible aliases (old buggy names) ----------

    def setservo(self, index, degree):
        """Deprecated: use move_servo(index, degree)."""
        self.move_servo(index, degree)

    def moveservo(self, time_ms, targetangle):
        """Deprecated: old (time, angles) argument order. Use move_all(angles, time_ms)."""
        self.move_all(targetangle, time_ms)