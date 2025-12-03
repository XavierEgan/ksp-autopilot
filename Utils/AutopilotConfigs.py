
"""
Mostly plane-agnostic autopilot configuration values.
"""
class AutopilotConfigs:
    def __init__(self):
        self.version = "1.0"

        self.turn_rate_pid_gains = {
            "kp" : 0.5,
            "ki" : 0.0,
            "kd" : 0.1,
            "kaw": 1.0
        }
        self.ascend_rate_pid_gains = {
            "kp" : 0.03,
            "ki" : 0.01,
            "kd" : 0.001,
            "kaw": 2.0
        }

        self.max_turn_rate_dps: float = 6.0
        self.max_ascend_rate_mps: float = 20.0