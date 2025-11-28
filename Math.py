from math import asin, radians, degrees, sin, cos, atan2

from krpc.services.spacecenter import Vessel

R_earth = 6371000  # radius of earth in meters

class LatLong:
    def __init__(self, latitude: float, longitude: float):
        self.latitude = latitude
        self.longitude = longitude

    def __repr__(self):
        return f"LatLong(latitude={self.latitude}, longitude={self.longitude})"
    
    def __str__(self) -> str:
        return f"({self.latitude}, {self.longitude})"
    
    def to_tuple(self) -> tuple[float, float]:
        return (self.latitude, self.longitude)
    
    def heading_to(self, other: 'LatLong') -> float:
        """
        Calculate the heading from this LatLong to another LatLong in degrees.
        Uses the formula for initial bearing between two points on a sphere.
        """
        lat1 = radians(self.latitude)
        lon1 = radians(self.longitude)
        lat2 = radians(other.latitude)
        lon2 = radians(other.longitude)

        dlon = lon2 - lon1

        x = sin(dlon) * cos(lat2)
        y = cos(lat1) * sin(lat2) - (sin(lat1) * cos(lat2) * cos(dlon))

        initial_bearing = atan2(x, y)
        initial_bearing = degrees(initial_bearing)
        compass_bearing = (initial_bearing + 360) % 360

        return compass_bearing
    
    def distance_to(self, other: 'LatLong') -> float:
        """
        Calculate the great-circle distance between this LatLong and another LatLong in meters.
        Uses the Haversine formula.
        """
        lat1 = radians(self.latitude)
        lon1 = radians(self.longitude)
        lat2 = radians(other.latitude)
        lon2 = radians(other.longitude)

        dlat = lat2 - lat1
        dlon = lon2 - lon1

        a = sin(dlat / 2) ** 2 + cos(lat1) * cos(lat2) * sin(dlon / 2) ** 2
        c = 2 * asin(min(1.0, a ** 0.5))

        distance = R_earth * c
        return distance

    @staticmethod
    def get_plane_latlong(vessel: Vessel) -> 'LatLong':
        lat = vessel.flight().latitude
        long = vessel.flight().longitude
        return LatLong(lat, long)

class LatLongLine:
    def __init__(self, start: LatLong, heading: float):
        self.start = start
        self.heading = heading

    def __repr__(self):
        return f"LatLongLine(start={self.start}, heading={self.heading})"
    
    def __str__(self) -> str:
        return f"Line(start={self.start}, heading={self.heading})"
    
    def cross_track_error(self, point: LatLong) -> float:
        """
        Calculate the shortest distance from this line to a given LatLong point in meters.
        """
        # Convert degrees to radians
        lat1 = radians(self.start.latitude)
        lon1 = radians(self.start.longitude)
        lat2 = radians(point.latitude)
        lon2 = radians(point.longitude)
        heading_rad = radians(self.heading)

        # Calculate differences
        dlon = lon2 - lon1
        dlat = lat2 - lat1

        # Calculate the cross-track distance
        a = sin(dlat / 2) ** 2 + cos(lat1) * cos(lat2) * sin(dlon / 2) ** 2
        c = 2 * asin(min(1.0, a ** 0.5))
        distance_start_to_point = R_earth * c

        # Calculate the bearing from start to point
        y = sin(dlon) * cos(lat2)
        x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon)
        bearing_start_to_point = atan2(y, x)

        # Calculate the angle difference
        angle_diff = atan2(sin(bearing_start_to_point - heading_rad),
                   cos(bearing_start_to_point - heading_rad))

        # Cross-track distance formula (signed: left positive, right negative)
        sin_cross = sin(distance_start_to_point / R_earth) * sin(angle_diff)
        sin_cross = max(-1.0, min(1.0, sin_cross))
        cross_track_distance = -asin(sin_cross) * R_earth

        return cross_track_distance

    def get_point_at_distance(self, distance: float) -> LatLong:
        """
        Calculate the LatLong point at a given distance along the line from the start point.
        """
        # Convert degrees to radians
        lat1 = radians(self.start.latitude)
        lon1 = radians(self.start.longitude)
        heading_rad = radians(self.heading)

        # Angular distance
        angular_distance = distance / R_earth

        lat2 = asin(sin(lat1) * cos(angular_distance) +
                    cos(lat1) * sin(angular_distance) * cos(heading_rad))

        lon2 = lon1 + atan2(sin(heading_rad) * sin(angular_distance) * cos(lat1),
                            cos(angular_distance) - sin(lat1) * sin(lat2))

        # Convert back to degrees
        lat2 = degrees(lat2)
        lon2 = degrees(lon2)

        return LatLong(lat2, lon2)

def cyclic_error(desired_angle: float, current_angle: float, period: float = 360.0) -> float:
    half_period = period / 2.0
    return (desired_angle - current_angle + half_period) % period - half_period

def clamp(value: float, min_value: float, max_value: float) -> float:
    return max(min_value, min(value, max_value))