from Utils.Math.PID import PID
from Utils.PlaneCharacteristics import PlaneCharacteristics
from Utils.AutopilotConfigs import AutopilotConfigs

"""
controls:
    Turn Rate D/s
    Speed m/s
    Asc/Desc rate m/s
"""
class BasicPlaneControl:
    def __init__(self, plane_characteristics: PlaneCharacteristics, autopilot_configs: AutopilotConfigs):
        self.pitch_pid = PID(**plane_characteristics.pitch_pid_gains)
        self.roll_pid = PID(**plane_characteristics.roll_pid_gains)
        self.speed_pid = PID(**plane_characteristics.speed_pid_gains)

        self.turn_rate_pid = PID(kp=0.0, ki=0.0, kd=0.0, kaw=0.0)
        self.ascend_rate_pid = PID(kp=0.0, ki=0.0, kd=0.0, kaw=0.0)

        self.max_turn_rate_dps = autopilot_configs.max_turn_rate_dps
        self.max_ascend_rate_mps = autopilot_configs.max_ascend_rate_mps
        self.max_speed_mps = plane_characteristics.max_speed_mps

    def update(self, dt: float, context: AutopilotContext):
