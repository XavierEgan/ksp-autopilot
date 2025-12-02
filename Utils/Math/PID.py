

class PID:
    def __init__(self, kp: float, ki: float, kd: float, kaw: float, setpoint: float = 0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.kaw = kaw

        self.setpoint = setpoint
        self.accum = 0.0
        self.previous_error = 0.0
    
    def update(self, measurement: float, dt: float) -> float:
        error = self.setpoint - measurement

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
    
    def set_setpoint(self, setpoint: float, reset: bool = False) -> None:
        self.setpoint = setpoint
        if reset:
            self.reset()