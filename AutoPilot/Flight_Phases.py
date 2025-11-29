from enum import Enum
from abc import ABC, abstractmethod
from Utils.Flight_Path_Params import FlightPathParams
from Utils.Math import clamp, LatLong, cyclic_error
from Utils.Timer import Timer

from krpc.services.spacecenter import Vessel
from AutoPilot.Plane_Controller_Manager import PlaneControllerManager

def reset_controlls(vessel: Vessel) -> None:
    vessel.control.throttle = 0.0
    vessel.control.pitch = 0.0
    vessel.control.roll = 0.0
    vessel.control.yaw = 0.0
    vessel.control.wheel_steering = 0.0

class FlightPhase(Enum):
    PRE_LAUNCH = 0
    TAKEOFF_ROLL = 1
    ROTATION = 2
    CLIMB = 3
    CRUISE = 4
    DESCENT = 5
    FINAL = 6
    FLARE = 7
    DEROTATION = 8
    ROLLOUT = 9
    FINISHED = 10

"""
Base class for Flight Phase to inherit from
each phase knows the next phase and condiontions to transition
"""
class FlightPhaseBase(ABC):
    def __init__(self, vessel: Vessel, flight_params: FlightPathParams, plane_controller_manager: PlaneControllerManager):
        self.vessel = vessel
        self.flight_params = flight_params
        self.plane_controller_manager = plane_controller_manager
    
    @abstractmethod
    def should_transition(self) -> bool:
        pass

    @abstractmethod
    def next_phase(self) -> FlightPhase:
        pass
    
    @abstractmethod
    def on_enter(self):
        pass

    """
    all phases have the responsibility to call update on the relevant controllers
    e.g. attitude controller, auto throttle, heading controller, altitude controller
    """
    @abstractmethod
    def update(self, delta_time: float):
        pass

class PreLaunch(FlightPhaseBase):
    def should_transition(self) -> bool:
        # we transition if engines are running, throttle is TOGA(max)
        return self.vessel.control.throttle >= .99 and self.vessel.thrust > 0.01
    
    def next_phase(self) -> FlightPhase:
        return FlightPhase.TAKEOFF_ROLL
    
    def on_enter(self):
        print("Entering Pre-Launch Phase")
    
    def update(self, delta_time: float):
        self.vessel.control.throttle = 1.0
        self.vessel.control.activate_next_stage()

        # set flaps
        self.vessel.control.set_action_group(1, True)

class TakeoffRoll(FlightPhaseBase):
    def should_transition(self) -> bool:
        # Switch to rotation when we reach Vr
        frame = self.vessel.orbit.body.reference_frame

        if self.vessel.flight(frame).speed >= self.flight_params.rotation_speed_mps:
            return True
        
        return False
    
    def next_phase(self) -> FlightPhase:
        return FlightPhase.ROTATION
    
    def on_enter(self):
        print("Entering Takeoff Roll Phase")
    
    def update(self, delta_time: float):
        self.plane_controller_manager.maintain_centerline_on_ground_controller.runway = self.flight_params.depature_runway
        self.plane_controller_manager.maintain_centerline_on_ground_controller.update(delta_time)

class Rotation(FlightPhaseBase):
    def should_transition(self) -> bool:
        # if rotation duration exceeded then transition
        return self.rotation_timer.finished and self.vessel.flight(self.vessel.surface_reference_frame).mean_altitude > 50
    
    def next_phase(self) -> FlightPhase:
        return FlightPhase.CLIMB
    
    def on_enter(self):
        print("Entering Rotation Phase")
        self.rotation_timer = Timer(self.flight_params.rotation_length_s)
    
    def update(self, delta_time: float):
        self.rotation_timer.update(delta_time)

        if (not self.rotation_timer.finished):
            desired_pitch: float = (self.rotation_timer.elapsed_s / self.flight_params.rotation_length_s) * self.flight_params.rotation_pitch_deg
            self.plane_controller_manager.attitude_controller.desired_pitch = desired_pitch

        self.plane_controller_manager.attitude_controller.update(delta_time)

        self.plane_controller_manager.maintain_centerline_on_ground_controller.runway = self.flight_params.depature_runway
        self.plane_controller_manager.maintain_centerline_on_ground_controller.update(delta_time)

class Climb(FlightPhaseBase):
    def should_transition(self) -> bool:
        # if we get within 50 meters of cruise altitude then transition
        reference_frame = self.vessel.surface_reference_frame
        current_altitude = self.vessel.flight(reference_frame).mean_altitude

        return abs(self.flight_params.cruise_altitude_m - current_altitude) <= 50
    
    def next_phase(self) -> FlightPhase:
        return FlightPhase.CRUISE
    
    def on_enter(self):
        print("Entering Climb Phase")
        reset_controlls(self.vessel)

        # retract gear
        self.vessel.control.gear = False

        # retract flaps
        self.vessel.control.set_action_group(1, False)
    
    def update(self, delta_time: float):
        self.plane_controller_manager.altitude_controller.desired_altitude = self.flight_params.cruise_altitude_m
        self.plane_controller_manager.altitude_controller.max_pitch = self.flight_params.climb_pitch_deg
        self.plane_controller_manager.altitude_controller.update(delta_time)

        self.plane_controller_manager.auto_throttle.desired_speed = self.flight_params.cruise_speed_mps
        self.plane_controller_manager.auto_throttle.update(delta_time)

        # only turn if we are high enough
        if self.vessel.flight().mean_altitude > self.flight_params.safe_turn_altitude_m:
            self.plane_controller_manager.heading_controller.desired_heading = LatLong.get_plane_latlong(self.vessel).heading_to(self.flight_params.arrival_runway.line.start)
            self.plane_controller_manager.heading_controller.update(delta_time)

class Cruise(FlightPhaseBase):
    def should_transition(self) -> bool:
        distance_to_final = LatLong.get_plane_latlong(self.vessel).distance_to(self.flight_params.final_begin_waypoint)
        return distance_to_final <= self.flight_params.descent_start_distance_m
    
    def next_phase(self) -> FlightPhase:
        return FlightPhase.DESCENT
    
    def on_enter(self):
        print("Entering Cruise Phase")
        reset_controlls(self.vessel)
    
    def update(self, delta_time: float):
        self.plane_controller_manager.altitude_controller.desired_altitude = self.flight_params.cruise_altitude_m
        self.plane_controller_manager.altitude_controller.update(delta_time)

        self.plane_controller_manager.heading_controller.desired_heading = LatLong.get_plane_latlong(self.vessel).heading_to(self.flight_params.final_begin_waypoint)
        self.plane_controller_manager.heading_controller.update(delta_time)
        self.plane_controller_manager.auto_throttle.desired_speed = self.flight_params.cruise_speed_mps
        self.plane_controller_manager.auto_throttle.update(delta_time)

class Descent(FlightPhaseBase):
    def should_transition(self) -> bool:
        altitude_check = self.vessel.flight(self.vessel.surface_reference_frame).mean_altitude <= self.flight_params.final_altitude_m + 50
        distance_check = LatLong.get_plane_latlong(self.vessel).distance_to(self.flight_params.final_begin_waypoint) <= 100
        return altitude_check and distance_check
    
    def next_phase(self) -> FlightPhase:
        return FlightPhase.FINAL
    
    def on_enter(self):
        print("Entering Descent Phase")
        reset_controlls(self.vessel)
    
    def update(self, delta_time: float):
        self.plane_controller_manager.altitude_controller.desired_altitude = self.flight_params.final_altitude_m
        self.plane_controller_manager.altitude_controller.update(delta_time)

        self.plane_controller_manager.heading_controller.desired_heading = LatLong.get_plane_latlong(self.vessel).heading_to(self.flight_params.final_begin_waypoint)
        self.plane_controller_manager.heading_controller.update(delta_time)

        self.plane_controller_manager.auto_throttle.desired_speed = self.flight_params.cruise_speed_mps
        self.plane_controller_manager.auto_throttle.update(delta_time)

class Final(FlightPhaseBase):
    def should_transition(self) -> bool:
        height_off_runway = self.vessel.flight(self.vessel.surface_reference_frame).mean_altitude - self.flight_params.arrival_runway.threashold_altitude

        return height_off_runway <= self.flight_params.flaire_altitude_m
    
    def next_phase(self) -> FlightPhase:
        return FlightPhase.FLARE
    
    def on_enter(self):
        print("Entering Final Phase")

        # 0 for pointing in the same direction as the runway
        # 1 for intercepting the localiser
        # the reason we do final in two phases is because we can begin final at any direciton, and if we are pointing away from the runway enough then we can enter an infinite loop flying away from the runway
        self.final_phase = 0

    def manage_gear_and_flaps(self):
        distance_to_runway = LatLong.get_plane_latlong(self.vessel).distance_to(self.flight_params.arrival_runway.line.start)
        speed = self.vessel.flight(self.vessel.orbit.body.reference_frame).speed

        # manage landing gear and flaps
        if (distance_to_runway <= self.flight_params.landing_gear_deploy_distance_m):
            self.vessel.control.gear = True
        
        if (speed <= self.flight_params.flaps_deploy_speed_mps):
            self.vessel.control.set_action_group(1, True)
        
    def manage_speed(self, delta_time: float):
        # manage speed
        distance_to_runway = LatLong.get_plane_latlong(self.vessel).distance_to(self.flight_params.arrival_runway.line.start)
        fraction_of_final_left = clamp(distance_to_runway / self.flight_params.final_length_m, 0, 1)
        desired_speed_above_landing_speed = (self.flight_params.cruise_speed_mps - self.flight_params.landing_speed_mps) * fraction_of_final_left

        landing_speed_mult_const = .8

        self.plane_controller_manager.auto_throttle.desired_speed = clamp((desired_speed_above_landing_speed + self.flight_params.landing_speed_mps) * landing_speed_mult_const, self.flight_params.landing_speed_mps, self.flight_params.cruise_speed_mps)

        self.plane_controller_manager.auto_throttle.update(delta_time)
    
    def manage_glideslope(self, delta_time: float):
        # manage glideslope
        dist_to_runway_threshold = LatLong.get_plane_latlong(self.vessel).distance_to(self.flight_params.arrival_runway.line.start)
        delta_altitude = self.flight_params.final_altitude_m - self.flight_params.arrival_runway.threashold_altitude

        # since its a similar triangle, delta_altitude decreases linearly with distance to runway threshold
        final_leg_length = self.flight_params.final_begin_waypoint.distance_to(self.flight_params.arrival_runway.line.start)
        final_leg_length = max(final_leg_length, 1) # prevent div by 0
        fraction_along_final = dist_to_runway_threshold / final_leg_length
        fraction_along_final = clamp(fraction_along_final, 0, 1)

        desired_altitude = self.flight_params.arrival_runway.threashold_altitude + (delta_altitude * fraction_along_final)
        self.plane_controller_manager.altitude_controller.desired_altitude = desired_altitude
        self.plane_controller_manager.altitude_controller.update(delta_time)

        print(f"Final Phase: Altitude Error: {desired_altitude - self.vessel.flight(self.vessel.surface_reference_frame).mean_altitude:.1f} Pitch Error: {self.plane_controller_manager.attitude_controller.desired_pitch - self.vessel.flight(self.vessel.surface_reference_frame).pitch:.1f}")

    def manage_localiser(self, delta_time: float):
        if self.final_phase == 0:
            self.plane_controller_manager.heading_controller.desired_heading = self.flight_params.arrival_runway.line.heading
            self.plane_controller_manager.heading_controller.update(delta_time)
        elif self.final_phase == 1:
            self.plane_controller_manager.localiser_controller.runway = self.flight_params.arrival_runway
            self.plane_controller_manager.localiser_controller.update(delta_time)

        if (abs(cyclic_error(self.plane_controller_manager.heading_controller.desired_heading, self.vessel.flight().heading)) < 5.0):
            self.final_phase = 1
    
    def update(self, delta_time: float):
        self.manage_gear_and_flaps()
        self.manage_speed(delta_time)
        self.manage_glideslope(delta_time)
        self.manage_localiser(delta_time)

class Flare(FlightPhaseBase):
    def should_transition(self) -> bool:
        delta = self.flare_timer.elapsed_s / self.flight_params.flaire_duration_s
        return delta >= 1.0
    
    def next_phase(self) -> FlightPhase:
        return FlightPhase.DEROTATION
    
    def on_enter(self):
        print("Entering Flare Phase")
        self.flare_timer = Timer(self.flight_params.flaire_duration_s)
        self.start_throttle = self.vessel.control.throttle

        reset_controlls(self.vessel)

    # returns float from 0 to 1 representing the flare pitch curve
    def flare_pitch_curve(self, fraction_of_flare_left: float) -> float:
        return fraction_of_flare_left ** 2 + .01
    
    def update(self, delta_time: float):
        self.flare_timer.update(delta_time)
        delta = self.flare_timer.elapsed_s / self.flight_params.flaire_duration_s
        height_left_to_flaire = self.flight_params.flaire_altitude_m
        desired_height = height_left_to_flaire * (1 - delta) * self.flare_pitch_curve(1 - delta) + self.flight_params.arrival_runway.threashold_altitude

        self.plane_controller_manager.altitude_controller.desired_altitude = desired_height
        self.plane_controller_manager.altitude_controller.update(delta_time, precise=True)

        self.vessel.control.throttle = clamp(1 - delta, 0, 1) * self.start_throttle

        print(f"Flare Phase: Altitude Error: {desired_height - self.vessel.flight(self.vessel.surface_reference_frame).mean_altitude:.1f} Pitch Error: {self.plane_controller_manager.attitude_controller.desired_pitch - self.vessel.flight(self.vessel.surface_reference_frame).pitch:.1f}")
        
class Derotation(FlightPhaseBase):
    def should_transition(self) -> bool:
        return True
    
    def next_phase(self) -> FlightPhase:
        return FlightPhase.ROLLOUT
    
    def on_enter(self):
        print("Entering Derotation Phase")
        self.vessel.control.throttle = 0.0
        self.derotation_timer = Timer(self.flight_params.derotation_time_s)
        self.start_angle = self.vessel.flight(self.vessel.surface_reference_frame).pitch
    
    def update(self, delta_time: float):
        self.derotation_timer.update(delta_time)
        delta = self.derotation_timer.elapsed_s / self.flight_params.derotation_time_s

        desired_pitch = self.start_angle * (1 - delta)
        self.plane_controller_manager.attitude_controller.desired_pitch = desired_pitch
        self.plane_controller_manager.attitude_controller.update(delta_time)

class Rollout(FlightPhaseBase):
    def should_transition(self) -> bool:
        return False
    
    def next_phase(self) -> FlightPhase:
        return FlightPhase.FINISHED
    
    def on_enter(self):
        print("Entering Rollout Phase")
        reset_controlls(self.vessel)
        self.vessel.control.throttle = 0.0
    
    def update(self, delta_time: float):
        # apply brakes and maintain centerline
        self.vessel.control.brakes = True
        self.plane_controller_manager.maintain_centerline_on_ground_controller.runway = self.flight_params.arrival_runway
        self.plane_controller_manager.maintain_centerline_on_ground_controller.update(delta_time)