
"""
Mostly plane-agnostic autopilot configuration values.
"""
class AutopilotConfigs:
    def __init__(self):
        self.version = "1.0"

        self.turn_rate_pid_gains = {
            "kp" : 0.0,
            "ki" : 0.0,
            "kd" : 0.0,
            "kaw": 1.0
        }
        self.ascend_rate_pid_gains = {
            
        }

        self.max_turn_rate_dps = 6.0
        self.max_ascend_rate_mps = 20.0