from AutoPilot.Full_AutoPilot import FullAutoPilot
from Utils.Runway import RUNWAY_YMML, RUNWAY_KSC , RUNWAY_YPAD
from Utils.Flight_Path_Params import FlightPathParams, generateFlightPath

def main() -> None:
    flight_params: FlightPathParams = generateFlightPath(
        RUNWAY_KSC, RUNWAY_KSC
    )

    autopilot = FullAutoPilot(flight_params)
    while True:
        autopilot.update()

if __name__ == "__main__":
    main()