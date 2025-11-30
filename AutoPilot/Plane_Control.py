from Utils.PID import PID, ForwardPID, PID_IntegralWindupMitigation
from Utils.PID2 import PID2, make_pid_ziegler_nichols, make_pid_manual
from Utils.Axis_Controller import AxisController
from Utils.Math import LatLong, clamp, cyclic_error
from krpc.services.spacecenter import Vessel
from Utils.Runway import Runway

"""
Controls roll and pitch and thrust using 2 PIDs
you set the desired pitch and roll
it also keeps a handle to the active vessel
"""
class AttitudeController:
    def __init__(self, vessel: Vessel):
        # self.pitch_pid: PID2 = make_pid_manual(kp=1/30, ki=1/50, kd=1/50, output_min=-1, output_max=1)
        # self.precise_pitch_pid: PID2 = make_pid_manual(kp=1/15, ki=1/4, kd=1/20, output_min=-1, output_max=1)

        self.pitch_axis_controller: AxisController = AxisController(
            pid=make_pid_ziegler_nichols(ku=0.94099, tu=0.62, output_min=-1, output_max=1, control_type='some overshoot'),
            reference_airspeed=160.0
        )

        self.roll_axis_controller = AxisController(
            pid=make_pid_ziegler_nichols(ku=0.91266, tu=1.36, output_min=-1, output_max=1, control_type='some overshoot'),
            reference_airspeed=170.0
        )

        self.desired_pitch = 0.0
        self.desired_roll = 0.0

        self.vessel = vessel

    def roll_update(self, delta_time: float):
        reference_frame = self.vessel.surface_reference_frame

        # between -180 and 180
        current_roll = self.vessel.flight(reference_frame).roll
        roll_error = cyclic_error(self.desired_roll, current_roll)
        roll_control = self.roll_axis_controller.get_control(roll_error, delta_time)

        self.vessel.control.roll = roll_control
    
    def pitch_update(self, delta_time: float, precise: bool = False):
        reference_frame = self.vessel.surface_reference_frame

        # between -90 and 90
        current_pitch = self.vessel.flight(reference_frame).pitch
        pitch_error = self.desired_pitch - current_pitch

        if precise:
            pitch_control = self.pitch_axis_controller.get_control(pitch_error, delta_time)
        else:
            pitch_control = self.pitch_axis_controller.get_control(pitch_error, delta_time)

        self.vessel.control.pitch = pitch_control

    def update(self, delta_time: float):
        self.roll_update(delta_time)
        self.pitch_update(delta_time)
        
"""
Controls the engine throttle to maintain a desired speed
"""
class AutoThrottle:
    def __init__(self, vessel: Vessel):
        self.thrust_pid = ForwardPID(1/50, 1/20, 0, 0, 0, 1)
        self.desired_speed = 0.0

        self.vessel = vessel

    def update(self, delta_time: float):
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

"""
Controls altitude by adjusting pitch
"""
class AltitudeController:
    def __init__(self, vessel: Vessel, attitude_controller: AttitudeController):
        self.desired_altitude = 0.0
        
        # i tried to do this with ziegler nichols but it does not work well
        self.altitude_pid: PID2 = make_pid_manual(kp=1/30, ki=1/80, kd=1/50, output_min=-1, output_max=1)
        self.precise_pid: PID2 = make_pid_manual(kp=1/10, ki=1/40, kd=1/30, output_min=-1, output_max=1)

        self.max_pitch = 10.0 # can be changed
        self.attitude_controller = attitude_controller
        self.vessel = vessel
    
    def update(self, delta_time: float, precise: bool = False):
        # precise is used during flare to minimise undershoot/overshoot
        reference_frame = self.vessel.surface_reference_frame
        current_altitude = self.vessel.flight(reference_frame).mean_altitude

        altitude_error = self.desired_altitude - current_altitude

        if precise:
            pitch_control = self.precise_pid.get_control(altitude_error, delta_time)
        else:
            pitch_control = self.altitude_pid.get_control(altitude_error, delta_time)


        desired_pitch = pitch_control * self.max_pitch

        self.attitude_controller.desired_pitch = desired_pitch
        self.attitude_controller.pitch_update(delta_time, precise=precise)

"""
Controlls heading by turning
"""
class HeadingController:
    def __init__(self, vessel: Vessel, attitude_controller: AttitudeController):
        self.vessel = vessel
        self.attitude_controller = attitude_controller

        self.desired_heading = 0.0

        self.roll_pid = PID(1/10, 1/20, 1/50, 10, -1, 1, PID_IntegralWindupMitigation.RESET)
        self.max_bank_angle = 30.0

    def update(self, delta_time: float):
        reference_frame = self.vessel.surface_reference_frame
        current_heading = self.vessel.flight(reference_frame).heading

        heading_error = cyclic_error(self.desired_heading, current_heading)

        roll_control = self.roll_pid.get_control(heading_error, delta_time)
        desired_roll = roll_control * self.max_bank_angle

        self.attitude_controller.desired_roll = desired_roll
        self.attitude_controller.roll_update(delta_time)

"""
Controlls the heading on ground
"""
class GroundHeadingController:
    def __init__(self, vessel: Vessel, attitude_controller: AttitudeController):
        self.vessel = vessel
        self.desired_heading = 0.0
        self.attitude_controller = attitude_controller

        self.yaw_pid = PID(1/10, 1/30, 0, 0, -1, 1)
        self.max_yaw_control = 0.2
        self.max_steering_control = 0.2

    def update(self, delta_time: float):
        reference_frame = self.vessel.surface_reference_frame
        current_heading = self.vessel.flight(reference_frame).heading

        heading_error = cyclic_error(self.desired_heading, current_heading)
        yaw_control = self.yaw_pid.get_control(heading_error, delta_time)

        self.vessel.control.yaw = clamp(yaw_control, -self.max_yaw_control, self.max_yaw_control)
        self.vessel.control.wheel_steering = clamp(-yaw_control, -self.max_steering_control, self.max_steering_control)

        # use attitude controller to make sure we dont get a wingtip strike
        self.attitude_controller.desired_roll = 0
        self.attitude_controller.roll_update(delta_time)

"""
Maintains centerline of runway by using a pid which specifies offset from the runway heading.
For example, if we are off to the left then we should turn right to rejoin the centerline
"""
class MaintainCenterlineOnGroundController:
    def __init__(self, vessel: Vessel, ground_heading_controller: GroundHeadingController):
        self.vessel = vessel
        self.runway: Runway | None = None
        self.ground_heading_controller = ground_heading_controller

        self.heading_offset_pid = PID(1/20, 1/10, 0, 0, -1, 1)
        self.max_heading_offset = 5.0 # degrees

    def update(self, delta_time: float):
        if self.runway is None:
            raise Exception("Runway not set for MaintainCenterlineOnGroundController")

        plane_latlong = LatLong.get_plane_latlong(self.vessel)
        cross_track_error = self.runway.cross_track_error(plane_latlong)

        heading_offset_control = self.heading_offset_pid.get_control(cross_track_error, delta_time)
        heading_offset = heading_offset_control * self.max_heading_offset

        desired_heading = (self.runway.line.heading + heading_offset) % 360
        self.ground_heading_controller.desired_heading = desired_heading

        self.ground_heading_controller.update(delta_time)

"""
Intercepts and maintains localiser for approach
"""
class LocaliserController:
    def __init__(self, vessel: Vessel, heading_controller: HeadingController, runway: Runway):
        self.vessel = vessel
        self.heading_controller = heading_controller
        self.runway = runway

        self.localiser_pid = PID(1/200, 1/25, 0, 0, -1, 1)
        self.max_heading_offset = 40.0 # degrees

    def update(self, delta_time: float):
        plane_latlong = LatLong.get_plane_latlong(self.vessel)
        cross_track_error = self.runway.cross_track_error(plane_latlong)

        heading_offset_control = self.localiser_pid.get_control(cross_track_error, delta_time)
        heading_offset = heading_offset_control * self.max_heading_offset

        self.heading_controller.desired_heading = (self.runway.line.heading + heading_offset) % 360.0
        self.heading_controller.update(delta_time)