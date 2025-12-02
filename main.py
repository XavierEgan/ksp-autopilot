from Utils.FlightPlan import FlightPlan
from Autopilot.AutopilotContext import AutopilotContext
from Utils.Runway import RUNWAY_YMML, RUNWAY_YPAD, RUNWAY_KSC
from Autopilot.Autopilot import Autopilot

def main():
    context = AutopilotContext(FlightPlan(
        departure_runway = RUNWAY_KSC,
        arrival_runway = RUNWAY_KSC
    ))

    autopilot = Autopilot(context)

    while True:
        autopilot.update()


if __name__ == "__main__":
    main()