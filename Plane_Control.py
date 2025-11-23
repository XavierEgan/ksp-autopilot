from PID import PID, ForwardPID, clamp, PID_IntegralWindupMitigation
import krpc
from krpc import client
import time


def cyclic_error(desired_angle, current_angle, period=360.0):
    half_period = period / 2.0
    return (desired_angle - current_angle + half_period) % period - half_period

"""
Controls roll and pitch and thrust using 2 PIDs
you set the desired pitch and roll
it also keeps a handle to the active vessel
"""
class AttitudeController:
    def __init__(self, vessel):
        self.pitch_pid = PID(1/50, 1/100, 1/80, 20, -1, 1, PID_IntegralWindupMitigation.DOUBLE_WINDUP_IF_OPPOSITE_PARITY)
        self.roll_pid = PID(1/50, 1/100, 1/60, 20, -1, 1, PID_IntegralWindupMitigation.RESET)

        self.desired_pitch = 0
        self.desired_roll = 0

        self.vessel = vessel

    def update(self, delta_time):
        reference_frame = self.vessel.surface_reference_frame

        # between -90 and 90
        current_pitch = self.vessel.flight(reference_frame).pitch
        # between -180 and 180
        current_roll = self.vessel.flight(reference_frame).roll

        pitch_error = self.desired_pitch - current_pitch
        roll_error = cyclic_error(self.desired_roll, current_roll)

        pitch_control = self.pitch_pid.get_control(pitch_error, delta_time)
        roll_control = self.roll_pid.get_control(roll_error, delta_time)

        self.vessel.control.pitch = pitch_control
        self.vessel.control.roll = roll_control

"""
Controls the engine throttle to maintain a desired speed
"""
class AutoThrottle:
    def __init__(self, vessel):
        self.thrust_pid = ForwardPID(1/50, 1/20, 0, 0, 0, 1)
        self.desired_speed = 0

        self.vessel = vessel

    def update(self, delta_time):
        # orbit reference frame because surface moves with craft so speed is always 0
        reference_frame = self.vessel.orbit.body.reference_frame
        current_speed = self.vessel.flight(reference_frame).speed
        speed_error = self.desired_speed - current_speed

        drag_vec = self.vessel.flight(reference_frame).drag
        drag_magnitude = (drag_vec[0]**2 + drag_vec[1]**2 + drag_vec[2]**2) ** 0.5
        max_producable_thrust = self.vessel.available_thrust
        predictive_value = clamp(drag_magnitude / max_producable_thrust if max_producable_thrust != 0 else 0, 0, 1)

        throttle_control = self.thrust_pid.get_control(speed_error, delta_time, predictive_value=predictive_value)
        self.vessel.control.throttle = throttle_control
        print(f"Current Speed: {current_speed:.1f}, Throttle: {throttle_control:.2f}")
"""
Controlls heading by turning
"""
class HeadingController:
    def __init__(self, vessel, attitude_controller: AttitudeController):
        self.vessel = vessel
        self.attitude_controller = attitude_controller

        self.desired_heading = 0

        self.roll_pid = PID(1/10, 1/20, 1/50, 10, -1, 1, PID_IntegralWindupMitigation.RESET)
        self.max_bank_angle = 30 

    def update(self, delta_time):
        reference_frame = self.vessel.surface_reference_frame
        current_heading = self.vessel.flight(reference_frame).heading

        heading_error = cyclic_error(self.desired_heading, current_heading)

        roll_control = self.roll_pid.get_control(heading_error, delta_time)
        desired_roll = clamp(roll_control * self.max_bank_angle, -self.max_bank_angle, self.max_bank_angle)

        self.attitude_controller.desired_roll = desired_roll

"""
Controls altitude by adjusting pitch
"""
class AltitudeController:
    def __init__(self, vessel, attitude_controller: AttitudeController):
        self.desired_altitude = 0
        self.altitude_pid = PID(1/30, 1/20, 1/50, 10, -1, 1)
        self.max_pitch = 10
        self.attitude_controller = attitude_controller
        self.vessel = vessel
    
    def update(self, delta_time):
        reference_frame = self.vessel.surface_reference_frame
        current_altitude = self.vessel.flight(reference_frame).mean_altitude

        altitude_error = self.desired_altitude - current_altitude
        pitch_control = self.altitude_pid.get_control(altitude_error, delta_time)
        desired_pitch = clamp(pitch_control * self.max_pitch, -self.max_pitch, self.max_pitch)

        self.attitude_controller.desired_pitch = desired_pitch



if __name__ == "__main__":
    conn = krpc.connect(name='Plane Controller')

    while True:
        try:
            if (conn.space_center is not None):
                vessel = conn.space_center.active_vessel
        except:
            print("No active vessel found, retrying...")
            continue
        break

    attitude_controller = AttitudeController(vessel)
    auto_throttle = AutoThrottle(vessel)
    heading_controller = HeadingController(vessel, attitude_controller)
    altitude_controller = AltitudeController(vessel, attitude_controller)
    attitude_controller.desired_pitch = 0
    attitude_controller.desired_roll = 0
    altitude_controller.desired_altitude = 7000

    auto_throttle.desired_speed = 200

    if (conn.space_center is None):
        quit()

    last_time = conn.space_center.ut
    while True:
        current_time = conn.space_center.ut
        delta_time = current_time - last_time
        last_time = current_time

        auto_throttle.update(delta_time)
        altitude_controller.update(delta_time)
        heading_controller.update(delta_time)
        attitude_controller.update(delta_time)

        time.sleep(0.01)