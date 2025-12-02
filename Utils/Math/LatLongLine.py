from LatLong import LatLong
from math import asin, radians, degrees, sin, cos, atan2

R_EARTH_M = 6371000

class LatLongLine:
    def __init__(self, start: LatLong, heading: float):
        self.start = start
        self.heading = heading
    
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
        distance_start_to_point = R_EARTH_M * c

        # Calculate the bearing from start to point
        y = sin(dlon) * cos(lat2)
        x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon)
        bearing_start_to_point = atan2(y, x)

        # Calculate the angle difference
        angle_diff = atan2(sin(bearing_start_to_point - heading_rad),
                   cos(bearing_start_to_point - heading_rad))

        # Cross-track distance formula (signed: left positive, right negative)
        sin_cross = sin(distance_start_to_point / R_EARTH_M) * sin(angle_diff)
        sin_cross = max(-1.0, min(1.0, sin_cross))
        cross_track_distance = -asin(sin_cross) * R_EARTH_M

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
        angular_distance = distance / R_EARTH_M

        lat2 = asin(sin(lat1) * cos(angular_distance) +
                    cos(lat1) * sin(angular_distance) * cos(heading_rad))

        lon2 = lon1 + atan2(sin(heading_rad) * sin(angular_distance) * cos(lat1),
                            cos(angular_distance) - sin(lat1) * sin(lat2))

        # Convert back to degrees
        lat2 = degrees(lat2)
        lon2 = degrees(lon2)

        return LatLong(lat2, lon2)

    def __str__(self) -> str:
        return f"Line(start={self.start}, heading={self.heading})"