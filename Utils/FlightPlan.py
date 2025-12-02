from Utils.Runway import Runway

class FlightPlan:
    def __init__(
            self, 
            departure_runway: Runway,
            arrival_runway: Runway,

            generate_star: bool = True,
    ):
        self.departure_runway = departure_runway
        self.arrival_runway = arrival_runway
        