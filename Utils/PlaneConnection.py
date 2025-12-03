import krpc
import time
from Utils.Math.LatLong import LatLong
from Utils.Math.Smoothing import CentralDifference

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

        self.turn_rate_smoothing = CentralDifference()
    
    def update(self, dt: float):
        self.turn_rate_smoothing.set_next(self.get_track())

    def get_turn_rate_dps(self, dt: float) -> float:
        return self.turn_rate_smoothing.get_current_derivative(dt)

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
        return self.vessel.flight(self.vessel.orbit.body.reference_frame).vertical_speed
    
    def get_latlong(self) -> LatLong:
        lat = self.vessel.flight().latitude
        long = self.vessel.flight().longitude
        return LatLong(latitude=lat, longitude=long)

    def get_position_vec(self) -> tuple[float, float, float]:
        return self.vessel.position(self.vessel.orbit.body.reference_frame)
    
    def calculate_required_engine_throttle(self, desired_speed_mps: float) -> float:
        body = self.vessel.orbit.body
        reference_frame = self.vessel.surface_velocity_reference_frame
        position = self.vessel.position(reference_frame)

        velocity = (0, desired_speed_mps, 0) # idk if i can get away with this mathematically but it might work
        force = self.vessel.flight(reference_frame).simulate_aerodynamic_force_at(body, position, velocity)
        drag = abs(force[1]) # y value is backwards force (basically drag)

        max_thrust = self.vessel.available_thrust
        required_throttle = drag / max_thrust if max_thrust > 0 else 0.0
        
        return min(max(required_throttle, 0.0), 1.0)



    def set_control_surfaces(self, pitch: float = 0.0, roll: float = 0.0, yaw: float = 0.0):
        control = self.vessel.control
        control.pitch = pitch
        control.roll = roll
        control.yaw = yaw
    
    def set_throttle(self, throttle: float):
        self.vessel.control.throttle = throttle