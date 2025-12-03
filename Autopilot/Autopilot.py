from Autopilot.AutopilotContext import AutopilotContext
from PlaneControl.PlaneControl import PlaneControl

"""
State machine that handles flying
"""

class Autopilot:
    def __init__(self, context: AutopilotContext, plane_control: PlaneControl):
        self.context = context
        self.plane_control = plane_control

        self.previous_time = self.context.plane_connection.get_time()

    def update(self):
        current_time = self.context.plane_connection.get_time()
        dt = current_time - self.previous_time
        if dt <= 0.0:
            return
        if dt > 1.0:
            # return if delta is absurdly large
            self.previous_time = current_time
            return
        self.previous_time = current_time

        self.context.update(dt)
        self.plane_control.update(dt, self.context)

        print(f"\033[2J\033[HAltitude: {self.context.plane_connection.get_altitude():.2f} m | Turn Rate: {self.context.plane_connection.get_turn_rate_dps(dt):.2f} d/s | Pitch: {self.context.plane_connection.get_pitch():.2f} deg | Roll: {self.context.plane_connection.get_roll():.2f} deg", end="")
        
