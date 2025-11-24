import math
from enum import Enum
from Math import clamp

class PID_IntegralWindupMitigation(Enum):
    PASSIVE = 0
    RESET = 1
    DOUBLE_WINDUP_IF_OPPOSITE_PARITY = 2
    QUADRUPLE_WINDUP_IF_OPPOSITE_PARITY = 3

class PID:
    def __init__(self, proportional_gain, derivative_gain, integral_gain, integral_max, min_out, max_out, windup_mitigation=PID_IntegralWindupMitigation.PASSIVE):
        self.proportional_gain = proportional_gain
        self.derivative_gain = derivative_gain
        self.integral_gain = integral_gain

        self.integral_max = integral_max

        self.integral_sum = 0

        self.min_out = min_out
        self.max_out = max_out

        self.previous_error = 0
        self.windup_mitigation = windup_mitigation
    
    def get_control(self, error, delta_time):
        self.integral_sum += error * delta_time

        error_sign = 1 if error >= 0 else -1
        integral_sign = 1 if self.integral_sum >= 0 else -1

        if (self.windup_mitigation == PID_IntegralWindupMitigation.RESET):
            # reset the integral when we hit or exceed the target
            if (error_sign != integral_sign):
                self.integral_sum = 0

        elif (self.windup_mitigation == PID_IntegralWindupMitigation.DOUBLE_WINDUP_IF_OPPOSITE_PARITY):
            # double the mitigation of integral if the error is the opposite sign of integral
            if (error_sign != integral_sign):
                self.integral_sum += error * delta_time  # double the addition
        
        elif (self.windup_mitigation == PID_IntegralWindupMitigation.QUADRUPLE_WINDUP_IF_OPPOSITE_PARITY):
            # quadruple the mitigation of integral if the error is the opposite sign of integral
            if (error_sign != integral_sign):
                self.integral_sum += 3 * error * delta_time  # quadruple the addition
            
        elif (self.windup_mitigation == PID_IntegralWindupMitigation.PASSIVE):
            # let it fix itself over time so we dont do anything
            pass
        
        # always clamp the integral to prevent windup
        self.integral_sum = clamp(self.integral_sum, -self.integral_max, self.integral_max)

        delta_error = error - self.previous_error
        self.previous_error = error

        prop = self.proportional_gain * error
        integral = self.integral_gain * self.integral_sum
        change = self.derivative_gain * (delta_error/delta_time) if delta_time != 0 else 0

        return clamp(prop + integral + change, self.min_out, self.max_out)
    
# Like a PID that you can add a predictive value to (example )
class ForwardPID(PID):
    def __init__(self, proportional_gain, derivative_gain, integral_gain, integral_max, min_out, max_out):
        super().__init__(proportional_gain, derivative_gain, integral_gain, integral_max, min_out, max_out)
    def get_control(self, error, delta_time, predictive_value=0):
        self.integral_sum += error * delta_time
        self.integral_sum = clamp(self.integral_sum, -self.integral_max, self.integral_max)

        delta_error = error - self.previous_error
        self.previous_error = error

        prop = self.proportional_gain * error
        integral = self.integral_gain * self.integral_sum
        change = self.derivative_gain * (delta_error/delta_time) if delta_time != 0 else 0

        return clamp(prop + integral + change + predictive_value, self.min_out, self.max_out)