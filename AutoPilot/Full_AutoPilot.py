from AutoPilot.Flight_Phases import *
from AutoPilot.AutoPilotContext import AutoPilotContext

class FullAutoPilot:
    def __init__(self, context: AutoPilotContext) -> None:
        self.context = context  

        self.just_entered_phase = True

        self.phase_controllers: dict[FlightPhase, FlightPhaseBase] = {
            FlightPhase.PRE_LAUNCH: PreLaunch(self.context),
            FlightPhase.TAKEOFF_ROLL: TakeoffRoll(self.context),
            FlightPhase.ROTATION: Rotation(self.context),
            FlightPhase.CLIMB_AND_CRUISE: ClimbAndCruise(self.context),
            FlightPhase.DESCENT: Descent(self.context),
            FlightPhase.FINAL: Final(self.context),
            FlightPhase.FLARE: Flare(self.context),
            FlightPhase.DEROTATION: Derotation(self.context),
            FlightPhase.ROLLOUT: Rollout(self.context)
        }

        self.phase_controller:FlightPhaseBase = self.phase_controllers[FlightPhase.PRE_LAUNCH]

        self.last_update_time = self.context.space_center.ut
        
    def update(self):
        if (self.last_update_time == self.context.telemetry.get_time()):
            return
        
        current_time = self.context.telemetry.get_time()
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