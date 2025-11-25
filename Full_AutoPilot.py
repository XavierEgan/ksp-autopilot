from Plane_Control import AttitudeController, AutoThrottle, AltitudeController
from Meta_Plane_Control import HeadingController, MaintainCenterlineOnGroundController, GroundHeadingController, LocaliserController
from PID import PID, ForwardPID
from Math import LatLong, LatLongLine, clamp

import krpc
from enum import Enum
import time
from abc import ABC, abstractmethod

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
class FlightPhaseController(ABC):
    def __init__(self, full_auto_pilot):
        self.full_auto_pilot = full_auto_pilot
    
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
    def update(self, delta_time):
        pass

class PreLaunch(FlightPhaseController):
    def __init__(self, full_auto_pilot):
        super().__init__(full_auto_pilot)
    
    def should_transition(self) -> bool:
        # we transition if engines are running, throttle is TOGA(max)
        return self.full_auto_pilot.vessel.control.throttle >= .99 and self.full_auto_pilot.vessel.thrust > 0.01
    
    def next_phase(self) -> FlightPhase:
        return FlightPhase.TAKEOFF_ROLL
    
    def on_enter(self):
        print("Entering Pre-Launch Phase")
    
    def update(self, delta_time):
        self.full_auto_pilot.vessel.control.throttle = 1.0
        self.full_auto_pilot.vessel.control.activate_next_stage()

        # set flaps
        self.full_auto_pilot.vessel.control.set_action_group(1, True)

class TakeoffRoll(FlightPhaseController):
    def __init__(self, full_auto_pilot):
        super().__init__(full_auto_pilot)
    
    def should_transition(self) -> bool:
        # Switch to rotation when we reach Vr
        frame = self.full_auto_pilot.vessel.orbit.body.reference_frame

        if self.full_auto_pilot.vessel.flight(frame).speed >= self.full_auto_pilot.flight_params.vr_speed:
            return True
        
        return False
    
    def next_phase(self) -> FlightPhase:
        return FlightPhase.ROTATION
    
    def on_enter(self):
        print("Entering Takeoff Roll Phase")
    
    def update(self, delta_time):
        self.full_auto_pilot.maintain_centerline_on_ground_controller.runway = self.full_auto_pilot.flight_params.depature_runway
        self.full_auto_pilot.maintain_centerline_on_ground_controller.update(delta_time)

class Rotation(FlightPhaseController):
    def __init__(self, full_auto_pilot):
        super().__init__(full_auto_pilot)

        self.rotation_duration = 0
    
    def should_transition(self) -> bool:
        # if rotation duration exceeded then transition
        return self.rotation_duration >= self.full_auto_pilot.flight_params.rotation_length and self.full_auto_pilot.vessel.flight(self.full_auto_pilot.vessel.surface_reference_frame).mean_altitude > 50
    
    def next_phase(self) -> FlightPhase:
        return FlightPhase.CLIMB
    
    def on_enter(self):
        print("Entering Rotation Phase")
        self.rotation_duration = 0
    
    def update(self, delta_time):
        self.rotation_duration += delta_time

        if (self.rotation_duration <= self.full_auto_pilot.flight_params.rotation_length):
            desired_pitch = (self.rotation_duration / self.full_auto_pilot.flight_params.rotation_length) * self.full_auto_pilot.flight_params.rotation_pitch
            self.full_auto_pilot.attitude_controller.desired_pitch = desired_pitch

        self.full_auto_pilot.attitude_controller.update(delta_time)

        self.full_auto_pilot.maintain_centerline_on_ground_controller.runway = self.full_auto_pilot.flight_params.depature_runway
        self.full_auto_pilot.maintain_centerline_on_ground_controller.update(delta_time)

class Climb(FlightPhaseController):
    def __init__(self, full_auto_pilot):
        super().__init__(full_auto_pilot)
    
    def should_transition(self) -> bool:
        # if we get within 50 meters of cruise altitude then transition
        reference_frame = self.full_auto_pilot.vessel.surface_reference_frame
        current_altitude = self.full_auto_pilot.vessel.flight(reference_frame).mean_altitude

        return abs(self.full_auto_pilot.flight_params.cruise_altitude - current_altitude) <= 50
    
    def next_phase(self) -> FlightPhase:
        return FlightPhase.CRUISE
    
    def on_enter(self):
        print("Entering Climb Phase")
        self.full_auto_pilot.reset_controlls()

        # retract gear
        self.full_auto_pilot.vessel.control.gear = False

        # retract flaps
        self.full_auto_pilot.vessel.control.set_action_group(1, False)
    
    def update(self, delta_time):
        self.full_auto_pilot.altitude_controller.desired_altitude = self.full_auto_pilot.flight_params.cruise_altitude
        self.full_auto_pilot.altitude_controller.max_pitch = self.full_auto_pilot.flight_params.climb_pitch
        self.full_auto_pilot.altitude_controller.update(delta_time)

        self.full_auto_pilot.auto_throttle.desired_speed = self.full_auto_pilot.flight_params.cruise_speed
        self.full_auto_pilot.auto_throttle.update(delta_time)

        # only turn if we are high enough
        if self.full_auto_pilot.vessel.flight().mean_altitude > self.full_auto_pilot.flight_params.safe_turn_altitude:
            self.full_auto_pilot.heading_controller.desired_heading = LatLong.get_plane_latlong(self.full_auto_pilot.vessel).heading_to(self.full_auto_pilot.flight_params.arrival_runway.start)
            self.full_auto_pilot.heading_controller.update(delta_time)

class Cruise(FlightPhaseController):
    def __init__(self, full_auto_pilot):
        super().__init__(full_auto_pilot)
    
    def should_transition(self) -> bool:
        distance_to_final = LatLong.get_plane_latlong(self.full_auto_pilot.vessel).distance_to(self.full_auto_pilot.flight_params.final_begin_waypoint)
        return distance_to_final <= self.full_auto_pilot.flight_params.descent_start
    
    def next_phase(self) -> FlightPhase:
        return FlightPhase.DESCENT
    
    def on_enter(self):
        print("Entering Cruise Phase")
        self.full_auto_pilot.reset_controlls()
    
    def update(self, delta_time):
        self.full_auto_pilot.altitude_controller.desired_altitude = self.full_auto_pilot.flight_params.cruise_altitude
        self.full_auto_pilot.altitude_controller.update(delta_time)

        self.full_auto_pilot.heading_controller.desired_heading = LatLong.get_plane_latlong(self.full_auto_pilot.vessel).heading_to(self.full_auto_pilot.flight_params.final_begin_waypoint)
        self.full_auto_pilot.heading_controller.update(delta_time)

        self.full_auto_pilot.auto_throttle.desired_speed = self.full_auto_pilot.flight_params.cruise_speed
        self.full_auto_pilot.auto_throttle.update(delta_time)

class Descent(FlightPhaseController):
    def __init__(self, full_auto_pilot):
        super().__init__(full_auto_pilot)
    
    def should_transition(self) -> bool:
        altitude_check = self.full_auto_pilot.vessel.flight(self.full_auto_pilot.vessel.surface_reference_frame).mean_altitude <= self.full_auto_pilot.flight_params.final_altitude + 50
        distance_check = LatLong.get_plane_latlong(self.full_auto_pilot.vessel).distance_to(self.full_auto_pilot.flight_params.final_begin_waypoint) <= 100
        return altitude_check and distance_check
    
    def next_phase(self) -> FlightPhase:
        return FlightPhase.FINAL
    
    def on_enter(self):
        print("Entering Descent Phase")
        self.full_auto_pilot.reset_controlls()
    
    def update(self, delta_time):
        self.full_auto_pilot.altitude_controller.desired_altitude = self.full_auto_pilot.flight_params.final_altitude
        self.full_auto_pilot.altitude_controller.update(delta_time)

        self.full_auto_pilot.heading_controller.desired_heading = LatLong.get_plane_latlong(self.full_auto_pilot.vessel).heading_to(self.full_auto_pilot.flight_params.final_begin_waypoint)
        self.full_auto_pilot.heading_controller.update(delta_time)

        self.full_auto_pilot.auto_throttle.desired_speed = self.full_auto_pilot.flight_params.final_speed
        self.full_auto_pilot.auto_throttle.update(delta_time)

class Final(FlightPhaseController):
    def __init__(self, full_auto_pilot):
        super().__init__(full_auto_pilot)
    
    def should_transition(self) -> bool:
        height_off_ground = self.full_auto_pilot.vessel.flight(self.full_auto_pilot.vessel.surface_reference_frame).mean_altitude - self.full_auto_pilot.flight_params.arrival_threashold_altitude

        return height_off_ground <= self.full_auto_pilot.flight_params.flaire_altitude
    
    def next_phase(self) -> FlightPhase:
        return FlightPhase.FLARE
    
    def on_enter(self):
        print("Entering Final Phase")
    
    def update(self, delta_time):
        self.full_auto_pilot.localiser_controller.runway = self.full_auto_pilot.flight_params.arrival_runway
        self.full_auto_pilot.localiser_controller.update(delta_time)

        self.full_auto_pilot.auto_throttle.desired_speed = self.full_auto_pilot.flight_params.landing_speed
        self.full_auto_pilot.auto_throttle.update(delta_time)

        distance_to_runway = LatLong.get_plane_latlong(self.full_auto_pilot.vessel).distance_to(self.full_auto_pilot.flight_params.arrival_runway.start)
        speed = self.full_auto_pilot.vessel.flight(self.full_auto_pilot.vessel.orbit.body.reference_frame).speed

        if (distance_to_runway <= self.full_auto_pilot.flight_params.landing_gear_deploy_distance):
            self.full_auto_pilot.vessel.control.gear = True
        
        if (speed <= self.full_auto_pilot.flight_params.flaps_deploy_speed):
            self.full_auto_pilot.vessel.control.set_action_group(1, True)

        # manage glideslope
        dist_to_runway_threshold = LatLong.get_plane_latlong(self.full_auto_pilot.vessel).distance_to(self.full_auto_pilot.flight_params.arrival_runway.start)
        delta_altitude = self.full_auto_pilot.flight_params.final_altitude - self.full_auto_pilot.flight_params.arrival_threashold_altitude

        # since its a similar triangle, delta_altitude decreases linearly with distance to runway threshold
        final_leg_length = self.full_auto_pilot.flight_params.final_begin_waypoint.distance_to(self.full_auto_pilot.flight_params.arrival_runway.start)
        final_leg_length = max(final_leg_length, 1) # prevent div by 0
        fraction_along_final = dist_to_runway_threshold / final_leg_length
        fraction_along_final = clamp(fraction_along_final, 0, 1)

        desired_altitude = self.full_auto_pilot.flight_params.arrival_threashold_altitude + (delta_altitude * fraction_along_final)
        self.full_auto_pilot.altitude_controller.desired_altitude = desired_altitude
        self.full_auto_pilot.altitude_controller.update(delta_time)

        print(f"Final Phase: Desired Altitude: {desired_altitude:.1f}, Current Altitude: {self.full_auto_pilot.vessel.flight(self.full_auto_pilot.vessel.surface_reference_frame).mean_altitude:.1f}")

class Flare(FlightPhaseController):
    def __init__(self, full_auto_pilot):
        super().__init__(full_auto_pilot)

        self.flaire_time = 0
    
    def should_transition(self) -> bool:
        delta = self.flaire_time / self.full_auto_pilot.flight_params.flaire_duration
        return delta >= 1.1
    
    def next_phase(self) -> FlightPhase:
        return FlightPhase.DEROTATION
    
    def on_enter(self):
        print("Entering Flare Phase")
        self.full_auto_pilot.reset_controlls()
        self.flaire_time = 0
    
    def update(self, delta_time):
        self.flaire_time += delta_time
        delta = self.flaire_time / self.full_auto_pilot.flight_params.flaire_duration
        height_left_to_flaire = self.full_auto_pilot.flight_params.flaire_altitude
        desired_height = height_left_to_flaire * (1 - delta) + self.full_auto_pilot.flight_params.arrival_threashold_altitude

        self.full_auto_pilot.altitude_controller.desired_altitude = desired_height
        self.full_auto_pilot.altitude_controller.update(delta_time)

        self.full_auto_pilot.auto_throttle.desired_speed = self.full_auto_pilot.flight_params.landing_speed
        self.full_auto_pilot.auto_throttle.update(delta_time)

        print(f"Flare Phase: Desired Altitude: {desired_height:.1f}, Current Altitude: {self.full_auto_pilot.vessel.flight(self.full_auto_pilot.vessel.surface_reference_frame).mean_altitude:.1f}")
        
class Derotation(FlightPhaseController):
    def __init__(self, full_auto_pilot):
        super().__init__(full_auto_pilot)
    
    def should_transition(self) -> bool:
        return True
    
    def next_phase(self) -> FlightPhase:
        return FlightPhase.ROLLOUT
    
    def on_enter(self):
        print("Entering Derotation Phase")
    
    def update(self, delta_time):
        pass

class Rollout(FlightPhaseController):
    def __init__(self, full_auto_pilot):
        super().__init__(full_auto_pilot)
    
    def should_transition(self) -> bool:
        return False
    
    def next_phase(self) -> FlightPhase:
        return FlightPhase.FINISHED
    
    def on_enter(self):
        print("Entering Rollout Phase")
        self.full_auto_pilot.reset_controlls()
        self.full_auto_pilot.vessel.control.throttle = 0.0
    
    def update(self, delta_time):
        # apply brakes and maintain centerline
        self.full_auto_pilot.vessel.control.brakes = True
        self.full_auto_pilot.maintain_centerline_on_ground_controller.runway = self.full_auto_pilot.flight_params.arrival_runway
        self.full_auto_pilot.maintain_centerline_on_ground_controller.update(delta_time)
        

"""
Information and parameters for the flight
Contains depature runway and approach runway info
"""

class FlightPathParams:
    def __init__(self):
        self.depature_runway : LatLongLine = LatLongLine(LatLong(-37.66444129883061, 144.8495527799204), 209.9854278564453)
        self.depature_threashold_altitude = 140.34361927770078
        self.arrival_runway : LatLongLine = LatLongLine(LatLong(-37.66444129883061, 144.8495527799204), 209.9854278564453)
        self.arrival_threashold_altitude = 140.34361927770078

        # self.arrival_runway : LatLongLine = LatLongLine(LatLong(-34.93613994127511, 138.53732454791594), 209.97891235351562)
        # self.arrival_threashold_altitude = 14.020269891247153

        self.vr_speed = 80
        self.rotation_length = 4 # seconds
        self.rotation_pitch = 20 # degrees

        self.climb_pitch = 10 # degrees
        self.safe_turn_altitude = 250 # meters

        self.cruise_altitude = 2000 # meters
        self.cruise_speed = 220 # m/s

        self.descent_start = 50_000 # meters from final entry point, this is arbitrary for now

        # we want to hit these parameters at the start of the final approach, then descend down and slow down accordingly
        self.final_length = 12_000 # meters
        self.final_altitude = 1000 + self.arrival_threashold_altitude # meters
        self.final_speed = 150 # m/s
        self.final_begin_waypoint:LatLong = self.arrival_runway.get_point_at_distance(-self.final_length)

        self.flaps_deploy_speed = 100 # m/s
        self.landing_gear_deploy_distance = 5000 # meters from runway threshold

        self.landing_speed = 60 # m/s

        self.flaire_altitude = 20 # meters
        self.flaire_duration = 20 # seconds

class FullAutoPilot:
    def __init__(self, vessel, flight_params):
        self.flight_params = flight_params

        self.vessel = vessel
        self.attitude_controller = AttitudeController(vessel)
        self.auto_throttle = AutoThrottle(vessel)
        self.heading_controller = HeadingController(vessel, self.attitude_controller)
        self.altitude_controller = AltitudeController(vessel, self.attitude_controller)
        self.ground_heading_controller = GroundHeadingController(vessel, self.attitude_controller)
        self.maintain_centerline_on_ground_controller = MaintainCenterlineOnGroundController(vessel, self.flight_params.depature_runway, self.ground_heading_controller)
        self.localiser_controller = LocaliserController(vessel, self.heading_controller, self.flight_params.arrival_runway)

        self.phase_controller: FlightPhaseController = PreLaunch(self)

        self.just_entered_phase = True

        self.phase_controllers:dict[FlightPhase, FlightPhaseController] = {
            FlightPhase.PRE_LAUNCH: PreLaunch(self),
            FlightPhase.TAKEOFF_ROLL: TakeoffRoll(self),
            FlightPhase.ROTATION: Rotation(self),
            FlightPhase.CLIMB: Climb(self),
            FlightPhase.CRUISE: Cruise(self),
            FlightPhase.DESCENT: Descent(self),
            FlightPhase.FINAL: Final(self),
            FlightPhase.FLARE: Flare(self),
            FlightPhase.DEROTATION: Derotation(self),
            FlightPhase.ROLLOUT: Rollout(self)
        }

    def update(self, delta_time):
        if self.just_entered_phase:
            self.phase_controller.on_enter()
            self.just_entered_phase = False

        elif self.phase_controller.should_transition():
            next_phase = self.phase_controller.next_phase()
            if next_phase == FlightPhase.FINISHED:
                print("Flight Completed")
                quit()

            self.phase_controller = self.phase_controllers[next_phase]
            self.just_entered_phase = True
            
        else:
            self.phase_controller.update(delta_time)

    def update_loop(self, conn):
        last_time = conn.space_center.ut
        while True:
            if (last_time == conn.space_center.ut):
                continue

            current_time = conn.space_center.ut
            delta_time = current_time - last_time
            last_time = current_time

            self.update(delta_time)
    
    def reset_controlls(self):
        self.vessel.control.throttle = 0.0
        self.vessel.control.pitch = 0.0
        self.vessel.control.roll = 0.0
        self.vessel.control.yaw = 0.0
        self.vessel.control.wheel_steering = 0.0

if __name__ == "__main__":
    conn = krpc.connect(name = 'Full AutoPilot')

    while True:
        try:
            if (conn.space_center is not None):
                vessel = conn.space_center.active_vessel
        except:
            print("No active vessel found, retrying...")
            continue
        break

    flight_params = FlightPathParams() # use default params for now

    full_auto_pilot = FullAutoPilot(vessel, flight_params)

    if (conn.space_center is None):
        quit()
    
    full_auto_pilot.update_loop(conn)