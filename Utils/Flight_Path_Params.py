from Utils.Runway import Runway
from Utils.Math import LatLong
from dataclasses import dataclass

"""
Information and parameters for the flight
Contains depature runway and approach runway info
"""
@dataclass
class FlightPathParams:
    depature_runway: Runway
    arrival_runway: Runway

    rotation_speed_mps: float
    rotation_length_s: float
    rotation_pitch_deg: float

    climb_pitch_deg: float
    safe_turn_altitude_m: float
    cruise_altitude_m: float
    cruise_speed_mps: float
    descent_start_distance_m: float
    final_begin_waypoint: LatLong

    final_length_m: float
    final_altitude_m: float
    final_speed_mps: float
    flaps_deploy_speed_mps: float
    landing_gear_deploy_distance_m: float
    landing_speed_mps: float

    flare_altitude_m: float
    flare_sink_rate: float
    derotation_time_s: float
    derotation_degrees: float

"""
put in all the params and get a flight path, if values are not included then reasonable defaults are used
"""
def generateFlightPath(
        depature_runway: Runway,
        arrival_runway: Runway,

        rotation_speed_mps: float = 80,
        rotation_length_s: float = 8,
        rotation_pitch_deg: float = 10,

        climb_pitch_deg: float = 10,
        safe_turn_altitude_m: float = 500,
        cruise_altitude_m: float = 10000,
        cruise_speed_mps: float = 220,
        descent_start_distance_m: float = 50_000,
        
        final_length_m: float = 25_000,
        final_altitude_m: float = 2000,
        final_speed_mps: float = 160,
        flaps_deploy_speed_mps: float = 100,
        landing_gear_deploy_distance_m: float = 5000,
        landing_speed_mps: float = 60,

        flare_altitude_m: float = 15,
        flare_sink_rate: float = 1,  # m/s
        derotation_time_s: float = 10,
        derotation_degrees: float = -3
        
) -> FlightPathParams:
    final_begin_waypoint: LatLong = arrival_runway.get_point_at_distance(-final_length_m)

    return FlightPathParams(
        depature_runway=depature_runway,
        arrival_runway=arrival_runway,

        rotation_speed_mps=rotation_speed_mps,
        rotation_length_s=rotation_length_s,
        rotation_pitch_deg=rotation_pitch_deg,

        climb_pitch_deg=climb_pitch_deg,
        safe_turn_altitude_m=safe_turn_altitude_m,

        cruise_altitude_m=cruise_altitude_m,
        cruise_speed_mps=cruise_speed_mps,
        descent_start_distance_m=descent_start_distance_m,
        final_begin_waypoint=final_begin_waypoint,

        final_length_m=final_length_m,
        final_altitude_m=final_altitude_m,
        final_speed_mps=final_speed_mps,
        flaps_deploy_speed_mps=flaps_deploy_speed_mps,
        landing_gear_deploy_distance_m=landing_gear_deploy_distance_m,
        landing_speed_mps=landing_speed_mps,

        flare_altitude_m=flare_altitude_m,
        flare_sink_rate=flare_sink_rate,

        derotation_time_s=derotation_time_s,
        derotation_degrees=derotation_degrees
    )
