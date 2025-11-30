from AutoPilot.Plane_Controller_Manager import PlaneControllerManager
from Utils.Flight_Path_Params import FlightPathParams, generateFlightPath
from AutoPilot.Flight_Phases import FlightPhase, FlightPhaseBase, PreLaunch, TakeoffRoll, Rotation, Climb, Cruise, Descent, Final, Flare, Derotation, Rollout

import krpc
from krpc.services.spacecenter import SpaceCenter
from krpc.client import Client
        
class FullAutoPilot:
    def __init__(self, flight_params: FlightPathParams) -> None:
        self.conn: Client = krpc.connect(name = 'Full AutoPilot')

        if (self.conn.space_center is None):
            print("Could not find space center.")
            quit()

        self.space_center: SpaceCenter = self.conn.space_center
        self.vessel = self.space_center.active_vessel

        self.plane_controller_manager = PlaneControllerManager(self.vessel, flight_params=flight_params)
        self.phase_controller:FlightPhaseBase = PreLaunch(self.vessel, flight_params, self.plane_controller_manager)

        self.just_entered_phase = True

        self.phase_controllers: dict[FlightPhase, FlightPhaseBase] = {
            FlightPhase.PRE_LAUNCH: PreLaunch(self.vessel, flight_params, self.plane_controller_manager),
            FlightPhase.TAKEOFF_ROLL: TakeoffRoll(self.vessel, flight_params, self.plane_controller_manager),
            FlightPhase.ROTATION: Rotation(self.vessel, flight_params, self.plane_controller_manager),
            FlightPhase.CLIMB: Climb(self.vessel, flight_params, self.plane_controller_manager),
            FlightPhase.CRUISE: Cruise(self.vessel, flight_params, self.plane_controller_manager),
            FlightPhase.DESCENT: Descent(self.vessel, flight_params, self.plane_controller_manager),
            FlightPhase.FINAL: Final(self.vessel, flight_params, self.plane_controller_manager),
            FlightPhase.FLARE: Flare(self.vessel, flight_params, self.plane_controller_manager),
            FlightPhase.DEROTATION: Derotation(self.vessel, flight_params, self.plane_controller_manager),
            FlightPhase.ROLLOUT: Rollout(self.vessel, flight_params, self.plane_controller_manager)
        }

        self.last_update_time = self.space_center.ut

    def update(self):
        if (self.last_update_time == self.space_center.ut):
            return
        
        current_time = self.space_center.ut
        delta_time = current_time - self.last_update_time
        self.last_update_time = current_time

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
    from Utils.Runway import RUNWAY_YMML, RUNWAY_KSC , RUNWAY_YPAD

    flight_params: FlightPathParams = generateFlightPath(
        RUNWAY_KSC, RUNWAY_KSC
    )

    full_auto_pilot = FullAutoPilot(flight_params)
    
    while True:
        full_auto_pilot.update()