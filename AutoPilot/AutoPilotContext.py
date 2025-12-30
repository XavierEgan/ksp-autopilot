import krpc
from AutoPilot.Plane_Controller_Manager import PlaneControllerManager
from Utils.Flight_Path_Params import FlightPathParams
from Utils.Telemetry import Telemetry

class AutoPilotContext:
    def __init__(self, flight_params: FlightPathParams) -> None:
        self.connect()

        self.flight_params = flight_params

        self.telemetry = Telemetry(self.conn)

        self.plane_controller_manager = PlaneControllerManager(self.vessel, flight_params=self.flight_params, telemetry=self.telemetry)
    
    def connect(self) -> None:
        self.conn = krpc.connect(name = 'Full AutoPilot')

        if (self.conn.space_center is None):
            print("Could not find space center.")
            quit()

        self.space_center = self.conn.space_center
        self.vessel = self.space_center.active_vessel