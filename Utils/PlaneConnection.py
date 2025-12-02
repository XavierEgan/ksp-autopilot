import krpc
import time
from Utils.Math.LatLong import LatLong

"""
Handles gathering telemetry from the plane
controlling the plane
and managing a connection with krpc
"""
class PlaneConnection:
    def __init__(self):
        while True:
            try:
                self.conn = krpc.connect(name="Plane Autopilot")
                space_center = self.conn.space_center

                if space_center is None:
                    raise Exception("Space center is None")
                
                self.space_center = space_center

                self.vessel = self.space_center.active_vessel

            except Exception as e:
                print(f"Connection failed with error: {e}. Retrying...")
                time.sleep(5)
                continue
            
            print("Connection established.")
            break

    def get_speed(self) -> float:
        return self.vessel.flight(self.vessel.orbit.body.reference_frame).speed
    
    def get_track(self) -> float:
        return self.vessel.flight().heading
    
    def get_altitude(self) -> float:
        return self.vessel.flight().mean_altitude
    
    def get_terrian_altitude(self) -> float:
        return self.vessel.flight().surface_altitude
    
    def get_time(self) -> float:
        return self.space_center.ut
    
    def get_pitch(self) -> float:
        return self.vessel.flight().pitch
    
    def get_roll(self) -> float:
        return self.vessel.flight().roll

    def get_vertical_speed(self) -> float:
        return self.vessel.flight().vertical_speed
    
    def get_latlong(self) -> LatLong:
        lat = self.vessel.flight().latitude
        long = self.vessel.flight().longitude
        return LatLong(latitude=lat, longitude=long)
    


    def set_control_surfaces(self, pitch: float, roll: float, yaw: float):
        control = self.vessel.control
        control.pitch = pitch
        control.roll = roll
        control.yaw = yaw
    
    def set_throttle(self, throttle: float):
        self.vessel.control.throttle = throttle