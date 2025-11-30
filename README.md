# KSP Autopilot
This is an autopilot i made for Kerbal Space Progam using KRPC.
It can take off, climb, cruise, descend and land the plane autonomously.

## How it works
The autopilot is a state machine that goes through different flight phases.
It uses a series of controllers that contain PID controllers that were tuned using a mix of manual and ziegler-nichols methods.

## Drawbacks
- Only works for the specific plane i tuned the PIDs with, it does not work well with any plane with significantly different aerodynamics, resulting in oscillations or instability.
- Only sets heading, ignoring track meaning it theoratically would not work for crosswind (although there is no wind in stock ksp)

## Future improvements
- impliment minimums and ability to go around
- improve PID tuning method
- make centralised spot where the plane performance parameters are stored, currently some of them are hardcoded