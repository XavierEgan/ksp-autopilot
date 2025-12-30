from krpc.client import Client
from Utils.Math import LatLong
from Utils.Math import cyclic_error

class Telemetry:
    def __init__(self, conn: Client) -> None:
        space_center = conn.space_center
        
        if space_center is None:
            print("Could not find space center.")
            quit()

        self.space_center = space_center

        self.vessel = self.space_center.active_vessel

    def get_altitude(self) -> float:
        current_altitude = self.vessel.flight().mean_altitude
        return current_altitude

    def get_speed_relative_to_kerbin(self) -> float:
        speed = self.vessel.flight(self.vessel.orbit.body.reference_frame).speed
        return speed

    def get_heading(self) -> float:
        heading = self.vessel.flight().heading
        return heading
    
    def get_pitch(self) -> float:
        pitch = self.vessel.flight().pitch
        return pitch
    
    def is_grounded(self) -> bool:
        return self.vessel.situation.name in ['landed', 'splashed']
    
    def get_vertical_speed(self) -> float:
        return self.vessel.flight(self.vessel.orbit.body.reference_frame).vertical_speed

    def get_time(self) -> float:
        return self.space_center.ut
    
    def get_latlong(self) -> LatLong:
        return LatLong.get_plane_latlong(self.vessel)
    
    def error_from_heading(self, desired_heading: float) -> float:
        return cyclic_error(desired_heading, self.vessel.flight().heading)