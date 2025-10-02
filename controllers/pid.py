import time
from dataclasses import dataclass


@dataclass
class PIDGains:
    kp: float
    ki: float
    kd: float


class PID:
    def __init__(self, gains: PIDGains, output_limit: float | None = None):
        self.gains = gains
        self.output_limit = output_limit
        self._prev_error = 0.0
        self._integral = 0.0
        self._prev_time = None

    def reset(self):
        self._prev_error = 0.0
        self._integral = 0.0
        self._prev_time = None

    def update(self, error: float) -> float:
        now = time.time()
        dt = 0.0 if self._prev_time is None else (now - self._prev_time)
        de = error - self._prev_error

        self._integral += error * dt

        p = self.gains.kp * error
        i = self.gains.ki * self._integral
        d = self.gains.kd * (de / dt) if dt > 1e-6 else 0.0
        out = p + i + d

        if self.output_limit is not None:
            if out > self.output_limit:
                out = self.output_limit
            elif out < -self.output_limit:
                out = -self.output_limit

        self._prev_error = error
        self._prev_time = now
        return out


