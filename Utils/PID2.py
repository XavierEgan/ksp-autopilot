from Utils.Math import clamp
from typing import Literal

class PID2:
    def __init__(self, kp: float = 0.0, ki: float = 0.0, kd: float = 0.0, output_min: float = -1.0, output_max: float = 1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.output_min = output_min
        self.output_max = output_max

        self.integral = 0.0
        self.previous_error = 0.0        
        

    def get_control(self, error: float, delta_time: float) -> float:
        prop = self.kp * error
        derivative = self.kd * ((error - self.previous_error) / delta_time) if delta_time != 0 else 0.0
        integral_contribution = self.ki * self.integral

        raw_output = prop + integral_contribution + derivative
        output = clamp(raw_output, self.output_min, self.output_max)

        # only wind integral if we are not saturated
        if output == raw_output:
            self.integral += error * delta_time

        self.previous_error = error

        return output


def make_pid_manual(
    kp: float = 0.0, 
    ki: float = 0.0, 
    kd: float = 0.0, 
    output_min: float = -1.0, output_max: float = 1.0
    ) -> PID2:
    return PID2(
        kp=kp,
        ki=ki,
        kd=kd,
        output_min=output_min,
        output_max=output_max
    )

def make_pid_ziegler_nichols(
        ku: float, 
        tu: float, 
        output_min: float = -1.0, output_max: float = 1.0, 
        control_type: Literal["classic", "pessen integral rule", "some overshoot", "no overshoot", "chill"] = "classic"
    ) -> PID2:

    if control_type == "classic":
        kp = ku        * 0.6
        ki = ku / tu   * 1.2
        kd = ku * tu   * 0.075
    elif control_type == "pessen integral rule":
        kp = ku        * 0.7
        ki = ku / tu   * 1.75
        kd = ku * tu   * 0.105
    elif control_type == "some overshoot":
        kp = ku        * (1 / 3)
        ki = ku / tu   * (2 / 3)
        kd = ku * tu   * (1 / 9)
    elif control_type == "no overshoot":
        kp = ku        * 0.2
        ki = ku / tu   * 0.4
        kd = ku * tu   * (2 / 30)
    elif control_type == "chill":
        kp = ku        * 0.1
        ki = ku / tu   * 0.4
        kd = ku * tu   * (2 / 30)
    else :
        raise ValueError(f"Unknown control type: {control_type}")
    
    return PID2(
        kp=kp,
        ki=ki,
        kd=kd,
        output_min=output_min,
        output_max=output_max
    )