from Utils.PID2 import PID2
from Utils.Math import clamp

MAX_AIRSPEED_FACTOR = 4.0
MIN_AIRSPEED_FACTOR = 0.5

"""
Wrapper for pid controller that takes into account airspeed
"""
class AxisController:
    def __init__(self, pid: PID2, reference_airspeed: float = 200.0) -> None:
        self.pid = pid
        self.reference_airspeed_squared = reference_airspeed * reference_airspeed
    

    def get_control(self, error: float, delta_time: float, current_airspeed: float) -> float:
        airspeed_factor = self.reference_airspeed_squared / (current_airspeed * current_airspeed) if current_airspeed != 0 else 1.0
        airspeed_factor = clamp(airspeed_factor, MIN_AIRSPEED_FACTOR, MAX_AIRSPEED_FACTOR)

        raw_output = self.pid.get_control(error, delta_time)
        return raw_output * airspeed_factor
