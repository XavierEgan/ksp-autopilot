from PlaneControl.BasicPlaneControl import BasicPlaneControl
from Utils.AutopilotConfigs import AutopilotConfigs
from Utils.PlaneCharacteristics import PlaneCharacteristics
from Autopilot.AutopilotContext import AutopilotContext

class PlaneControl:
    def __init__(self, plane_characteristics: PlaneCharacteristics, autopilot_configs: AutopilotConfigs):
        self.basic_plane_control = BasicPlaneControl(plane_characteristics, autopilot_configs)

    def update(self, dt: float, context: AutopilotContext):
        self.basic_plane_control.update(dt, context)