import time
from PlaneControl.PlaneControl import PlaneControl
from Utils.FlightPlan import FlightPlan
from Autopilot.AutopilotContext import AutopilotContext
from Utils.Runway import RUNWAY_YMML, RUNWAY_YPAD, RUNWAY_KSC
from Autopilot.Autopilot import Autopilot
from Utils.AutopilotConfigs import AutopilotConfigs
from Utils.PlaneCharacteristics import PlaneCharacteristics

def main():
    flight_plan = FlightPlan(
        departure_runway = RUNWAY_KSC,
        arrival_runway = RUNWAY_KSC
    )

    plane_characteristics = PlaneCharacteristics()
    autopilot_configs = AutopilotConfigs()
    plane_control = PlaneControl(plane_characteristics, autopilot_configs)

    context = AutopilotContext(flight_plan)

    autopilot = Autopilot(context, plane_control)

    while True:
        autopilot.update()

        time.sleep(0.005)


if __name__ == "__main__":
    main()