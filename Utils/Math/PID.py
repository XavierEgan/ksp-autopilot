

class PID:
    def __init__(self, kp: float, ki: float, kd: float, kaw: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.kaw = kaw

        self.accum = 0.0
        self.previous_error = 0.0
    
    def update(self, error: float, dt: float) -> float:
        # integral and anti-windup
        self.accum += error * dt
        self.accum -= self.kaw * self.previous_error * dt

        # derivative
        derivative = (error - self.previous_error) / dt if dt > 0 else 0.0
        self.previous_error = error

        # PID output
        output = (self.kp * error) + (self.ki * self.accum) + (self.kd * derivative)
        return output

    def reset(self):
        self.accum = 0.0
        self.previous_error = 0.0