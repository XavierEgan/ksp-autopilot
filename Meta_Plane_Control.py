import krpc
from Plane_Control import AttitudeController
from PID import PID, PID_IntegralWindupMitigation
from Math import clamp, cyclic_error, LatLong, LatLongLine

"""
contains classes for controlling higher order plane functions
(e.g. maintainging centerline, localiser and glideslope)
"""

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
        desired_roll = roll_control * self.max_bank_angle

        self.attitude_controller.desired_roll = desired_roll
        self.attitude_controller.update(delta_time)

"""
Controlls the heading on ground
"""

class GroundHeadingController:
    def __init__(self, vessel, attitude_controller: AttitudeController):
        self.vessel = vessel
        self.desired_heading = 0
        self.attitude_controller = attitude_controller

        self.yaw_pid = PID(1/10, 1/30, 0, 0, -1, 1)
        self.max_yaw_control = 0.2
        self.max_steering_control = 0.2

    def update(self, delta_time):
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
    def __init__(self, vessel, runway: LatLongLine, ground_heading_controller: GroundHeadingController):
        self.vessel = vessel
        self.runway = runway
        self.ground_heading_controller = ground_heading_controller

        self.heading_offset_pid = PID(1/20, 1/10, 0, 0, -1, 1)
        self.max_heading_offset = 5 # degrees

    def update(self, delta_time):
        plane_latlong = LatLong.get_plane_latlong(self.vessel)
        cross_track_error = self.runway.cross_track_error(plane_latlong)

        heading_offset_control = self.heading_offset_pid.get_control(cross_track_error, delta_time)
        heading_offset = heading_offset_control * self.max_heading_offset

        desired_heading = (self.runway.heading + heading_offset) % 360
        self.ground_heading_controller.desired_heading = desired_heading

        self.ground_heading_controller.update(delta_time)

class LocaliserController:
    def __init__(self, vessel, heading_controller: HeadingController, runway: LatLongLine):
        self.vessel = vessel
        self.heading_controller = heading_controller
        self.runway = runway

        self.localiser_pid = PID(1/200, 1/100, 0, 0, -1, 1)
        self.max_heading_offset = 30 # degrees

    def update(self, delta_time):
        plane_latlong = LatLong.get_plane_latlong(self.vessel)
        cross_track_error = self.runway.cross_track_error(plane_latlong)

        heading_offset_control = self.localiser_pid.get_control(cross_track_error, delta_time)
        heading_offset = heading_offset_control * self.max_heading_offset

        self.heading_controller.desired_heading = (self.runway.heading + heading_offset) % 360
        self.heading_controller.update(delta_time)

class GlideslopeController:
    pass

if __name__ == "__main__":
    conn = krpc.connect(name='Heading Controller Test')
    if not conn.space_center:
        exit()
    vessel = conn.space_center.active_vessel

    attitude_controller = AttitudeController(vessel)
    ghc = GroundHeadingController(vessel, attitude_controller)
    runway = LatLongLine(LatLong(-37.66444129883061, 144.8495527799204), 209.9854278564453)
    mcl = MaintainCenterlineOnGroundController(vessel, runway, ghc)

    ghc.desired_heading = 90

    last_time = conn.space_center.ut
    while True:
        current_time = conn.space_center.ut
        delta_time = current_time - last_time
        last_time = current_time

        mcl.update(delta_time)