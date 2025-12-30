from Utils.Math import clamp

class CentralDifference:
    def __init__(self):
        self.previous_value = 0.0
        self.curent_value = 0.0
        self.next_value = 0.0
    
    def get_curent(self):
        return self.curent_value
    
    def set_next(self, value: float):
        self.previous_value = self.curent_value
        self.curent_value = self.next_value
        self.next_value = value
    
    def get_current_derivative(self, dt: float):
        if dt <= 0.0:
            return 0.0
        derivative = (self.next_value - self.previous_value) / (2.0 * dt)
        return derivative

class PID2:
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
        self.accum += (output - raw_output) * self.kaw * dt

        return output

    def reset(self):
        self.accum = 0.0


def make_pid_manual(
    kp: float = 0.0, 
    ki: float = 0.0, 
    kd: float = 0.0, 
    kaw: float = 0.0,
    output_min: float = -1.0, output_max: float = 1.0
    ) -> PID2:
    return PID2(
        kp=kp,
        ki=ki,
        kd=kd,
        kaw=kaw,
        min_output=output_min,
        max_output=output_max
    )