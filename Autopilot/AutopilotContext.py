from Utils.FlightPlan import FlightPlan
from Utils.PlaneConnection import PlaneConnection

class AutopilotContext:
    def __init__(self, flight_plan: FlightPlan):
        self.flight_plan = flight_plan
        self.plane_connection = PlaneConnection()