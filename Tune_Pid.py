from AutoPilot.Plane_Control import AltitudeController, AttitudeController
import krpc

OSCILATION_THRESHOLD = 1.5
TEST_DELTA = 0.0005

class Oscilation:
    def __init__(self, set_point: float) -> None:
        self.max_value = -float('inf')
        self.max_value_time = 0.0

        self.min_value = float('inf')
        self.min_value_time = 0.0

        self.prev_value_1 = 0.0
        self.prev_value_2 = 0.0
        self.prev_time_2 = 0.0

        self.set_point = 0.0

    def update(self, value: float, current_time: float) -> None:
        # look for local max, prev_value_2 is greater than now and before
        if self.prev_value_2 > self.prev_value_1 and self.prev_value_2 > value:
            self.max_value = self.prev_value_2
            self.max_value_time = self.prev_time_2
            print(f"Max detected: {self.max_value}")
            print(f"Difference: {abs(abs(self.max_value - self.set_point) - abs(self.set_point - self.min_value))}")
        
        # look for local min, prev_value_2 is less than now and before
        if self.prev_value_2 < self.prev_value_1 and self.prev_value_2 < value:
            self.min_value = self.prev_value_2
            self.min_value_time = self.prev_time_2
            print(f"Min detected: {self.min_value}")
            print(f"Difference: {abs(abs(self.max_value - self.set_point) - abs(self.set_point - self.min_value))}")

        self.prev_value_1 = self.prev_value_2
        self.prev_value_2 = value
        self.prev_time_2 = current_time
    
    def is_oscillating(self) -> bool:
        # if the difference between the distance between min and max to the set point is less than threshold
        return abs(abs(self.max_value - self.set_point) - abs(self.set_point - self.min_value)) < OSCILATION_THRESHOLD


def tune_roll() -> None:
    conn = krpc.connect(name='PID Tuning')
    space_center = conn.space_center

    if space_center is None:
        quit()

    vessel = space_center.active_vessel
    controller = AttitudeController(vessel)
    controller.roll_axis_controller.pid.kp = 0.2
    controller.roll_axis_controller.pid.ki = 0.0
    controller.roll_axis_controller.pid.kd = 0.0

    oscilation = Oscilation(0.0)

    prevous_time = space_center.ut
    while True:
        current_time = space_center.ut
        delta_time = current_time - prevous_time

        if delta_time <= 0.0:
            continue

        prevous_time = current_time

        controller.roll_update(delta_time)
        controller.pitch_update(delta_time)

        oscilation.update(vessel.flight().roll, current_time)

        if oscilation.is_oscillating():
            print(f"Oscilating detected, Ku = {controller.roll_axis_controller.pid.kp}, Tu = {oscilation.max_value_time - oscilation.min_value_time}")
            break
            
        controller.roll_axis_controller.pid.kp += TEST_DELTA

def tune_pitch() -> None:
    conn = krpc.connect(name='PID Tuning')
    space_center = conn.space_center

    if space_center is None:
        quit()

    vessel = space_center.active_vessel
    controller = AttitudeController(vessel)
    controller.pitch_axis_controller.pid.kp = 0.2
    controller.pitch_axis_controller.pid.ki = 0.0
    controller.pitch_axis_controller.pid.kd = 0.0

    oscilation = Oscilation(0.0)

    prevous_time = space_center.ut
    while True:
        current_time = space_center.ut
        delta_time = current_time - prevous_time

        if delta_time <= 0.0:
            continue

        prevous_time = current_time

        controller.roll_update(delta_time)
        controller.pitch_update(delta_time)

        oscilation.update(vessel.flight().pitch, current_time)

        if oscilation.is_oscillating():
            print(f"Oscilating detected, Ku = {controller.pitch_axis_controller.pid.kp}, Tu = {oscilation.max_value_time - oscilation.min_value_time}")
            break
            
        controller.pitch_axis_controller.pid.kp += TEST_DELTA

def tune_altitude() -> None:
    conn = krpc.connect(name='PID Tuning')
    space_center = conn.space_center

    if space_center is None:
        quit()

    vessel = space_center.active_vessel
    controller = AltitudeController(vessel, AttitudeController(vessel))
    controller.desired_altitude = vessel.flight().mean_altitude + 50.0
    controller.altitude_pid.kp = 0.0
    controller.altitude_pid.ki = 0.0
    controller.altitude_pid.kd = 0.0

    oscilation = Oscilation(controller.desired_altitude)

    prevous_time = space_center.ut
    while True:
        current_time = space_center.ut
        delta_time = current_time - prevous_time

        if delta_time <= 0.0:
            continue

        prevous_time = current_time

        controller.update(delta_time)

        oscilation.update(vessel.flight().mean_altitude, current_time)

        if oscilation.is_oscillating():
            print(f"Oscilating detected, Ku = {controller.altitude_pid.kp}, Tu = {oscilation.max_value_time - oscilation.min_value_time}")
            break
            
        controller.altitude_pid.kp += TEST_DELTA

if __name__ == "__main__":
    tune_altitude()