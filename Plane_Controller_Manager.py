from Plane_Control import AttitudeController, AutoThrottle, AltitudeController, HeadingController, GroundHeadingController, MaintainCenterlineOnGroundController, LocaliserController
from Flight_Path_Params import FlightPathParams

from krpc.services.spacecenter import Vessel

class PlaneControllerManager:
    def __init__(self, vessel: Vessel, flight_params: FlightPathParams):
        self.attitude_controller = AttitudeController(vessel)
        self.auto_throttle = AutoThrottle(vessel)
        self.heading_controller = HeadingController(vessel, self.attitude_controller)
        self.altitude_controller = AltitudeController(vessel, self.attitude_controller)
        self.ground_heading_controller = GroundHeadingController(vessel, self.attitude_controller)
        self.maintain_centerline_on_ground_controller = MaintainCenterlineOnGroundController(vessel, self.ground_heading_controller)
        self.localiser_controller = LocaliserController(vessel, self.heading_controller, flight_params.arrival_runway)