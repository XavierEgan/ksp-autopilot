from Plane_Controller_Manager import PlaneControllerManager
from Flight_Path_Params import FlightPathParams, generateFlightPath
from Flight_Phases import FlightPhase, FlightPhaseBase, PreLaunch, TakeoffRoll, Rotation, Climb, Cruise, Descent, Final, Flare, Derotation, Rollout

import krpc
from krpc.services.spacecenter import Vessel, SpaceCenter
from krpc.client import Client
        
class FullAutoPilot:
    def __init__(self, vessel: Vessel, flight_params: FlightPathParams):
        self.plane_controller_manager = PlaneControllerManager(vessel, flight_params=flight_params)

        self.phase_controller:FlightPhaseBase = PreLaunch(vessel, flight_params, self.plane_controller_manager)

        self.just_entered_phase = True

        self.phase_controllers:dict[FlightPhase, FlightPhaseBase] = {
            FlightPhase.PRE_LAUNCH: PreLaunch(vessel, flight_params, self.plane_controller_manager),
            FlightPhase.TAKEOFF_ROLL: TakeoffRoll(vessel, flight_params, self.plane_controller_manager),
            FlightPhase.ROTATION: Rotation(vessel, flight_params, self.plane_controller_manager),
            FlightPhase.CLIMB: Climb(vessel, flight_params, self.plane_controller_manager),
            FlightPhase.CRUISE: Cruise(vessel, flight_params, self.plane_controller_manager),
            FlightPhase.DESCENT: Descent(vessel, flight_params, self.plane_controller_manager),
            FlightPhase.FINAL: Final(vessel, flight_params, self.plane_controller_manager),
            FlightPhase.FLARE: Flare(vessel, flight_params, self.plane_controller_manager),
            FlightPhase.DEROTATION: Derotation(vessel, flight_params, self.plane_controller_manager),
            FlightPhase.ROLLOUT: Rollout(vessel, flight_params, self.plane_controller_manager)
        }

    def update(self, delta_time:float):
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

    def update_loop(self, space_center: SpaceCenter):
        last_time = space_center.ut
        while True:
            if (last_time == space_center.ut):
                continue

            current_time = space_center.ut
            delta_time = current_time - last_time
            last_time = current_time

            self.update(delta_time)

if __name__ == "__main__":
    conn: Client = krpc.connect(name = 'Full AutoPilot')

    if (conn.space_center is None):
        print("Could not find space center.")
        quit()

    space_center: SpaceCenter = conn.space_center

    while True:
        try:
            vessel = space_center.active_vessel
        except:
            print("No active vessel found, retrying...")
            continue
        break

    from Runway import RUNWAY_YMML#, RUNWAY_YPAD

    flight_params: FlightPathParams = generateFlightPath(
        RUNWAY_YMML, RUNWAY_YMML
    )

    full_auto_pilot = FullAutoPilot(vessel, flight_params)
    
    full_auto_pilot.update_loop(space_center)