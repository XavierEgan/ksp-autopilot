from Utils.Math.Smoothing import CentralDifference
from Utils.Math.utils import clamp

class PID:
    def __init__(self, kp: float, ki: float, kd: float, kaw: float, min_output: float = -1.0, max_output: float = 1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.kaw = kaw

        self.min_output = min_output
        self.max_output = max_output

        self.accum = 0.0
        self.smoothing = CentralDifference()
    
    def update(self, error: float, dt: float, timewarping: bool = False) -> float:
        self.smoothing.set_next(error)
        current_error = self.smoothing.get_curent()

        # derivative
        derivative = self.smoothing.get_current_derivative(dt)

        raw_output = (self.kp * current_error) + (self.ki * self.accum) + (self.kd * derivative)
        output = clamp(raw_output, self.min_output, self.max_output)
        
        # anti-windup
        self.accum += current_error * dt
        self.accum += (output - raw_output) * self.kaw

        return output

    def reset(self):
        self.accum = 0.0

