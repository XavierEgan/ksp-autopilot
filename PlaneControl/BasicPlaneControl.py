from Autopilot.AutopilotContext import AutopilotContext
from Utils.Math.PID import PID
from Utils.Math.utils import clamp
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

        self.turn_rate_pid = PID(**autopilot_configs.turn_rate_pid_gains)
        self.ascend_rate_pid = PID(**autopilot_configs.ascend_rate_pid_gains)

        self.max_bank_angle_deg = plane_characteristics.max_bank_angle_deg
        self.max_pitch_angle_deg = plane_characteristics.max_pitch_angle_deg
        self.max_turn_rate_dps = autopilot_configs.max_turn_rate_dps
        self.max_ascend_rate_mps = autopilot_configs.max_ascend_rate_mps
        self.max_speed_mps = plane_characteristics.max_speed_mps

        self.desired_turn_rate_dps = 3.0
        self.desired_ascend_rate_mps = 0.0
        self.desired_speed_mps = 100.0

    def update(self, dt: float, context: AutopilotContext):
        plane_connection = context.plane_connection

        # turn rate control
        current_turn_rate_dps = plane_connection.get_turn_rate_dps(dt)
        turn_rate_error = self.desired_turn_rate_dps - current_turn_rate_dps
        desired_roll = self.turn_rate_pid.update(turn_rate_error, dt)
        desired_roll = desired_roll * self.max_bank_angle_deg

        # speed control
        current_speed = plane_connection.get_speed()
        speed_error = self.desired_speed_mps - current_speed
        calculated_engine_throttle = plane_connection.calculate_required_engine_throttle(self.desired_speed_mps)
        throttle = calculated_engine_throttle + self.speed_pid.update(speed_error, dt)
        
        # ascend rate control
        current_ascend_rate = plane_connection.get_vertical_speed()
        ascend_rate_error = self.desired_ascend_rate_mps - current_ascend_rate
        desired_pitch = self.ascend_rate_pid.update(ascend_rate_error, dt)
        desired_pitch = desired_pitch * self.max_pitch_angle_deg

        # pitch
        current_pitch = plane_connection.get_pitch()
        pitch_error = desired_pitch - current_pitch
        pitch_control = self.pitch_pid.update(pitch_error, dt)

        # roll
        current_roll = plane_connection.get_roll()
        roll_error = desired_roll - current_roll
        roll_control = self.roll_pid.update(roll_error, dt)

        # apply controls
        context.plane_connection.set_control_surfaces(
            pitch=pitch_control,
            roll=roll_control,
        )
        
        context.plane_connection.set_throttle(throttle)