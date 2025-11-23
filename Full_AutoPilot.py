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
    ROLLOUT = 8

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

    @abstractmethod
    def update(self, delta_time):
        pass

class FullAutoPilot:
    def __init__(self, vessel):
        self.vessel = vessel
        self.attitude_controller = AttitudeController(vessel)
        self.auto_throttle = AutoThrottle(vessel)
        self.heading_controller = HeadingController(vessel, self.attitude_controller)
        self.altitude_controller = AltitudeController(vessel, self.attitude_controller)

        self.phase = FlightPhase.PRE_LAUNCH
        self.phase_controller = None  # type: FlightPhaseController
    
    def update(self, delta_time):
        pass




if __name__ == "__main__":
    conn = krpc.connect(name='Full AutoPilot')

    while True:
        try:
            if (conn.space_center is not None):
                vessel = conn.space_center.active_vessel
        except:
            print("No active vessel found, retrying...")
            continue
        break

    full_auto_pilot = FullAutoPilot(vessel)

    if (conn.space_center is None):
        quit()

    last_time = conn.space_center.ut
    while True:
        current_time = conn.space_center.ut
        delta_time = current_time - last_time
        last_time = current_time

        full_auto_pilot.update(delta_time)

        time.sleep(0.01)
