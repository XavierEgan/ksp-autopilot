import krpc
import time

def print_depature_airport_info():
    conn = krpc.connect(name='Get Departure Info')
    vessel = conn.space_center.active_vessel
    
    # stage and spool up engines
    vessel.control.activate_next_stage()
    vessel.control.throttle = 1.0

    # wait a few seconds to get stable readings
    time.sleep(3)

    # get the velocity direction
    velocity_vector = list(vessel.flight(vessel.surface_reference_frame).velocity)
    velocity_vector[1] = 0 # ignore vertical component

    position_vector = list(vessel.position(vessel.orbit.body.reference_frame))

    print("")