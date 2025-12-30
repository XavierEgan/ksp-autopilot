from enum import Enum
from abc import ABC, abstractmethod
from Utils.Math import clamp, LatLong, inverse_lerp, lerp
from Utils.Timer import CountDownTimer
from AutoPilot.AutoPilotContext import AutoPilotContext
from Utils.Logging import Logger

from krpc.services.spacecenter import Vessel

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
    CLIMB_AND_CRUISE = 3
    DESCENT = 4
    FINAL = 5
    FLARE = 6
    DEROTATION = 7
    ROLLOUT = 8
    FINISHED = 9
    TESTING = 10

"""
Base class for Flight Phase to inherit from
each phase knows the next phase and condiontions to transition
"""
class FlightPhaseBase(ABC):
    def __init__(self, context: AutoPilotContext) -> None:
        self.context = context
        self.vessel = context.vessel
        self.telemetry = context.telemetry
        self.flight_params = context.flight_params
        self.plane_controller_manager = context.plane_controller_manager
    
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
        self.wait_timer = CountDownTimer(3.0)  # wait for 3 seconds before takeoff, to let the craft settle down
    
    def update(self, delta_time: float):
        self.wait_timer.update(delta_time)
        if not self.wait_timer.finished:
            return
        
        if self.wait_timer.just_finished():
            # store the pitch we need to derotate down to later when landing
            self.flight_params.derotation_degrees = self.telemetry.get_pitch()

        self.vessel.control.throttle = 1.0
        self.vessel.control.activate_next_stage()

        # set flaps
        self.vessel.control.set_action_group(1, True)

class TakeoffRoll(FlightPhaseBase):
    def should_transition(self) -> bool:
        # Switch to rotation when we reach Vr
        return self.telemetry.get_speed_relative_to_kerbin() >= self.flight_params.rotation_speed_mps
    
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
        return self.rotation_timer.finished and self.telemetry.get_altitude() - self.flight_params.depature_runway.threashold_altitude > 50
    
    def next_phase(self) -> FlightPhase:
        return FlightPhase.CLIMB_AND_CRUISE
    
    def on_enter(self):
        print("Entering Rotation Phase")
        self.rotation_timer = CountDownTimer(self.flight_params.rotation_length_s)
        self.plane_controller_manager.maintain_centerline_on_ground_controller.runway = self.flight_params.depature_runway
    
    def update(self, delta_time: float):
        self.rotation_timer.update(delta_time)

        if (not self.rotation_timer.finished):
            desired_pitch: float = self.rotation_timer.fraction_complete() * self.flight_params.rotation_pitch_deg
            self.plane_controller_manager.attitude_controller.desired_pitch = desired_pitch

        self.plane_controller_manager.attitude_controller.update(delta_time)
        self.plane_controller_manager.maintain_centerline_on_ground_controller.update(delta_time)

class ClimbAndCruise(FlightPhaseBase):
    def should_transition(self) -> bool:
        # transition if were within descent_start_distance_m of the final start point
        close_to_descent: bool = self.telemetry.get_latlong().distance_to(self.flight_params.final_begin_waypoint) <= self.flight_params.descent_start_distance_m
        adequate_altitude: bool = self.telemetry.get_altitude() >= self.flight_params.cruise_altitude_m / 3

        return close_to_descent and adequate_altitude
    
    def next_phase(self) -> FlightPhase:
        return FlightPhase.DESCENT
    
    def on_enter(self):
        print("Entering Climb/Cruise Phase")
        reset_controlls(self.vessel)

        # retract gear
        self.vessel.control.gear = False

        # retract flaps
        self.vessel.control.set_action_group(1, False)

    def update(self, delta_time: float):
        # only turn if we are high enough
        high_enough_to_turn: bool = self.telemetry.get_altitude() > self.flight_params.safe_turn_altitude_m
        if high_enough_to_turn:
            desired_heading = self.telemetry.get_latlong().heading_to(self.flight_params.final_begin_waypoint)
            self.plane_controller_manager.heading_controller.desired_heading = desired_heading
        else:
            self.plane_controller_manager.heading_controller.desired_heading = self.flight_params.depature_runway.line.heading
            
        
        self.plane_controller_manager.altitude_controller.desired_altitude = self.flight_params.cruise_altitude_m
        self.plane_controller_manager.altitude_controller.max_pitch = self.flight_params.climb_pitch_deg
        self.plane_controller_manager.auto_throttle.desired_speed = self.flight_params.cruise_speed_mps

        self.plane_controller_manager.altitude_controller.update(delta_time)
        self.plane_controller_manager.auto_throttle.update(delta_time)
        self.plane_controller_manager.heading_controller.update(delta_time)

class Descent(FlightPhaseBase):
    def should_transition(self) -> bool:
        altitude_check = self.vessel.flight(self.vessel.surface_reference_frame).mean_altitude <= self.flight_params.final_altitude_m + 50
        distance_check = self.telemetry.get_latlong().distance_to(self.flight_params.final_begin_waypoint) <= 100
        return altitude_check and distance_check
    
    def next_phase(self) -> FlightPhase:
        return FlightPhase.FINAL
    
    def on_enter(self):
        print("Entering Descent Phase")
        reset_controlls(self.vessel)

        self.descent_start_distance: float = self.telemetry.get_latlong().distance_to(self.flight_params.final_begin_waypoint)
        self.descent_start_altitude:float = self.telemetry.get_altitude()
        self.descent_start_speed: float = self.telemetry.get_speed_relative_to_kerbin()
    
    def manage_altitude(self, delta_time: float, lerp_t: float):
        self.context.plane_controller_manager.altitude_controller.desired_altitude = lerp(self.descent_start_altitude, self.flight_params.final_altitude_m, lerp_t)
        self.context.plane_controller_manager.altitude_controller.update(delta_time)
        # Logger.get().log(f"desired altitude: {self.context.plane_controller_manager.altitude_controller.desired_altitude}")
    
    def manage_speed(self, delta_time: float, lerp_t: float):
        self.context.plane_controller_manager.auto_throttle.desired_speed = lerp(self.descent_start_speed, self.flight_params.final_speed_mps, lerp_t)
        # Logger.get().log(f"desired speed: {self.context.plane_controller_manager.auto_throttle.desired_speed}")
        self.context.plane_controller_manager.auto_throttle.update(delta_time)
    
    def update(self, delta_time: float):
        heading_to_runway = self.telemetry.get_latlong().heading_to(self.flight_params.final_begin_waypoint)
        self.plane_controller_manager.heading_controller.desired_heading = heading_to_runway

        lerp_t: float = inverse_lerp(self.descent_start_distance, 0, self.telemetry.get_latlong().distance_to(self.flight_params.final_begin_waypoint))
        self.manage_altitude(delta_time, lerp_t)
        self.manage_speed(delta_time, lerp_t)

        self.plane_controller_manager.heading_controller.update(delta_time)

class Final(FlightPhaseBase):
    def should_transition(self) -> bool:
        height_off_runway = self.vessel.flight(self.vessel.surface_reference_frame).mean_altitude - self.flight_params.arrival_runway.threashold_altitude

        return height_off_runway <= self.flight_params.flare_altitude_m
    
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
        
    def manage_speed(self, delta_time: float, lerp_t: float):
        desired_speed = lerp(self.flight_params.final_speed_mps, self.flight_params.landing_speed_mps, lerp_t)
        self.plane_controller_manager.auto_throttle.desired_speed = desired_speed
        self.plane_controller_manager.auto_throttle.update(delta_time)
    
    def manage_glideslope(self, delta_time: float, lerp_t: float):
        desired_altitude = lerp(self.flight_params.final_altitude_m, self.flight_params.arrival_runway.threashold_altitude, lerp_t)
        self.plane_controller_manager.altitude_controller.desired_altitude = desired_altitude
        self.plane_controller_manager.altitude_controller.update(delta_time)

    def manage_localiser(self, delta_time: float):
        if self.final_phase == 0:
            self.plane_controller_manager.heading_controller.desired_heading = self.flight_params.arrival_runway.line.heading
            self.plane_controller_manager.heading_controller.update(delta_time)
        elif self.final_phase == 1:
            self.plane_controller_manager.localiser_controller.runway = self.flight_params.arrival_runway
            self.plane_controller_manager.localiser_controller.update(delta_time)

        if (abs(self.telemetry.error_from_heading(self.flight_params.arrival_runway.line.heading)) < 5.0):
            self.final_phase = 1
    
    def update(self, delta_time: float):
        distance_to_runway = self.telemetry.get_latlong().distance_to(self.flight_params.arrival_runway.line.start)
        t = inverse_lerp(self.flight_params.final_length_m, 0, distance_to_runway)

        self.manage_gear_and_flaps()
        self.manage_speed(delta_time, t)
        self.manage_glideslope(delta_time, t)
        self.manage_localiser(delta_time)

class Flare(FlightPhaseBase):
    def should_transition(self) -> bool:
        return self.telemetry.is_grounded()
    
    def next_phase(self) -> FlightPhase:
        return FlightPhase.DEROTATION
    
    def on_enter(self):
        print("Entering Flare Phase")

        self.setpoint_pitch: float = self.vessel.control.pitch

        reset_controlls(self.vessel)
    
    def update(self, delta_time: float):
        self.plane_controller_manager.flare_controller.update(delta_time, setpoint_control=self.setpoint_pitch)

        # maintain speed
        self.plane_controller_manager.auto_throttle.desired_speed = self.flight_params.landing_speed_mps
        self.plane_controller_manager.auto_throttle.update(delta_time)

        # maintain runway centerline
        self.plane_controller_manager.localiser_controller.runway = self.flight_params.arrival_runway
        self.plane_controller_manager.localiser_controller.update(delta_time)
        
class Derotation(FlightPhaseBase):
    def should_transition(self) -> bool:
        return self.derotation_timer.finished
    
    def next_phase(self) -> FlightPhase:
        return FlightPhase.ROLLOUT
    
    def on_enter(self):
        print("Entering Derotation Phase")
        self.vessel.control.throttle = 0.0
        self.derotation_timer = CountDownTimer(self.flight_params.derotation_time_s)
        self.start_angle = self.vessel.flight(self.vessel.surface_reference_frame).pitch
        self.end_angle = self.flight_params.derotation_degrees
    
    def update(self, delta_time: float):
        self.derotation_timer.update(delta_time)
        delta = self.derotation_timer.fraction_complete()

        desired_pitch = self.start_angle * (1 - delta) + self.end_angle * delta
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
    
class Testing(FlightPhaseBase):
    def should_transition(self) -> bool:
        return False
    
    def next_phase(self) -> FlightPhase:
        return FlightPhase.FINISHED
    
    def on_enter(self):
        print("Entering Testing Phase")
        self.vessel.control.set_action_group(1, True)
    
    def update(self, delta_time: float):
        self.plane_controller_manager.altitude_controller.desired_altitude = 100.0
        self.plane_controller_manager.auto_throttle.desired_speed = 60.0
        self.plane_controller_manager.heading_controller.desired_heading = 0.0

        self.plane_controller_manager.altitude_controller.update(delta_time)
        self.plane_controller_manager.auto_throttle.update(delta_time)
        self.plane_controller_manager.heading_controller.update(delta_time)
