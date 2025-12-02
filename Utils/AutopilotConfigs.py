
"""
Mostly plane-agnostic autopilot configuration values.
"""
class AutopilotConfigs:
    def __init__(self):
        self.version = "1.0"

        self.max_turn_rate_dps = 6.0
        self.max_ascend_rate_mps = 20.0