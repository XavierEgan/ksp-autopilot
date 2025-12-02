from Autopilot.AutopilotContext import AutopilotContext

"""
State machine that handles flying
"""

class Autopilot:
    def __init__(self, context: AutopilotContext):
        self.context = context

        self.previous_time = self.context.plane_connection.get_time()

    def update(self):
        current_time = self.context.plane_connection.get_time()
        dt = current_time - self.previous_time
        if dt <= 0.0:
            return
        self.previous_time = current_time


        self.context.plane_control.update(dt, self.context)