from Plane_Control import AttitudeController, AutoThrottle, HeadingController, AltitudeController
from PID import PID, ForwardPID
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
        # TODO: make sure we are on the centerline (implimented later)
        pass

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
            self.full_auto_pilot.heading_controller.desired_heading = self.full_auto_pilot.flight_params.temp_climb_heading
            self.full_auto_pilot.heading_controller.update(delta_time)

class Cruise(FlightPhaseController):
    def __init__(self, full_auto_pilot):
        super().__init__(full_auto_pilot)
    
    def should_transition(self) -> bool:
        return False
    
    def next_phase(self) -> FlightPhase:
        return FlightPhase.DESCENT
    
    def on_enter(self):
        print("Entering Pre-Launch Phase")
    
    def update(self, delta_time):
        self.full_auto_pilot.altitude_controller.desired_altitude = self.full_auto_pilot.flight_params.cruise_altitude
        self.full_auto_pilot.altitude_controller.update(delta_time)

        self.full_auto_pilot.heading_controller.desired_heading = self.full_auto_pilot.flight_params.temp_climb_heading
        self.full_auto_pilot.heading_controller.update(delta_time)

        self.full_auto_pilot.auto_throttle.desired_speed = self.full_auto_pilot.flight_params.cruise_speed
        self.full_auto_pilot.auto_throttle.update(delta_time)

class Descent(FlightPhaseController):
    def __init__(self, full_auto_pilot):
        super().__init__(full_auto_pilot)
    
    def should_transition(self) -> bool:
        return False
    
    def next_phase(self) -> FlightPhase:
        return FlightPhase.FINAL
    
    def on_enter(self):
        print("Entering Pre-Launch Phase")
    
    def update(self, delta_time):
        pass

class Final(FlightPhaseController):
    def __init__(self, full_auto_pilot):
        super().__init__(full_auto_pilot)
    
    def should_transition(self) -> bool:
        return False
    
    def next_phase(self) -> FlightPhase:
        return FlightPhase.FLARE
    
    def on_enter(self):
        print("Entering Pre-Launch Phase")
    
    def update(self, delta_time):
        pass

class Flare(FlightPhaseController):
    def __init__(self, full_auto_pilot):
        super().__init__(full_auto_pilot)
    
    def should_transition(self) -> bool:
        return False
    
    def next_phase(self) -> FlightPhase:
        return FlightPhase.DEROTATION
    
    def on_enter(self):
        print("Entering Pre-Launch Phase")
    
    def update(self, delta_time):
        pass

class Derotation(FlightPhaseController):
    def __init__(self, full_auto_pilot):
        super().__init__(full_auto_pilot)
    
    def should_transition(self) -> bool:
        return False
    
    def next_phase(self) -> FlightPhase:
        return FlightPhase.ROLLOUT
    
    def on_enter(self):
        print("Entering Pre-Launch Phase")
    
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
        print("Entering Pre-Launch Phase")
    
    def update(self, delta_time):
        pass

class FlightPathParams:
    def __init__(self):
        self.vr_speed = 80
        self.rotation_length = 4 # seconds
        self.rotation_pitch = 20 # degrees

        self.climb_pitch = 10 # degrees
        self.safe_turn_altitude = 250 # meters

        self.temp_climb_heading = 50 # degrees

        self.cruise_altitude = 7000 # meters
        self.cruise_speed = 200 # m/s

class FullAutoPilot:
    def __init__(self, vessel):
        self.vessel = vessel
        self.attitude_controller = AttitudeController(vessel)
        self.auto_throttle = AutoThrottle(vessel)
        self.heading_controller = HeadingController(vessel, self.attitude_controller)
        self.altitude_controller = AltitudeController(vessel, self.attitude_controller)

        self.phase_controller: FlightPhaseController = PreLaunch(self)

        self.just_entered_phase = True

        self.flight_params = FlightPathParams()

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

    full_auto_pilot = FullAutoPilot(vessel)
    full_auto_pilot.flight_params = flight_params

    if (conn.space_center is None):
        quit()

    last_time = conn.space_center.ut
    while True:
        if (last_time == conn.space_center.ut):
            time.sleep(0.001)
            continue

        current_time = conn.space_center.ut
        delta_time = current_time - last_time
        last_time = current_time

        full_auto_pilot.update(delta_time)
        time.sleep(0.001)