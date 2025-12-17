from dataclasses import dataclass


"""
Details inherent to the plane being controlled.
"""
@dataclass
class PlaneCharacteristics:
    def __init__(self):
        self.version = "1.0"

        self.pitch_pid_gains = {
            "kp" : 0.15,
            "ki" : 0.15,
            "kd" : 0.02,
            "kaw": 2.0,
            "min_output" : -1.0,
            "max_output" : 1.0
        }
        self.roll_pid_gains = {
            "kp" : 0.2,
            "ki" : 0.0,
            "kd" : 0.05,
            "kaw": 1.0
        }
        # needs to be able to output negative because it needs to have the option to override the predicted thrust required
        self.speed_pid_gains = {
            "kp" : 0.5,
            "ki" : 0.1,
            "kd" : 0.2,
            "kaw": 1.0
        }

        self.max_bank_angle_deg = 40.0
        self.max_pitch_angle_deg = 10.0

        self.max_speed_mps = 250.0
        self.stall_speed_mps = 50.0