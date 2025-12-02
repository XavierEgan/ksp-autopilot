from dataclasses import dataclass


"""
Details inherent to the plane being controlled.
"""
@dataclass
class PlaneCharacteristics:
    def __init__(self):
        self.version = "1.0"

        self.pitch_pid_gains = {
            "kp" : 0.0,
            "ki" : 0.0,
            "kd" : 0.0,
            "kaw": 1.0
        }
        self.roll_pid_gains = {
            "kp" : 0.0,
            "ki" : 0.0,
            "kd" : 0.0,
            "kaw": 1.0
        }
        self.speed_pid_gains = {
            "kp" : 0.0,
            "ki" : 0.0,
            "kd" : 0.0,
            "kaw": 1.0
        }

        self.max_bank_angle_deg = 30.0
        self.max_pitch_angle_deg = 15.0

        self.max_speed_mps = 250.0
        self.stall_speed_mps = 50.0        