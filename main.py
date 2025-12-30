from AutoPilot.AutoPilotContext import AutoPilotContext
from AutoPilot.Flight_Phases import FlightPhase
from AutoPilot.Full_AutoPilot import FullAutoPilot
from Utils.Runway import RUNWAY_YMML, RUNWAY_KSC , RUNWAY_YPAD, RUNWAY_PBI
from Utils.Flight_Path_Params import FlightPathParams, generateFlightPath

def main() -> None:
    flight_params: FlightPathParams = generateFlightPath(
        RUNWAY_KSC, RUNWAY_KSC,
        cruise_altitude_m=8000,
    )

    autopilot = FullAutoPilot(AutoPilotContext(flight_params))
    autopilot.phase_controller = autopilot.phase_controllers[FlightPhase.FINAL]

    
    while True:
        autopilot.update()

if __name__ == "__main__":
    main()