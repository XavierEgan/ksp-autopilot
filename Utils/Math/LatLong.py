from math import asin, radians, degrees, sin, cos, atan2, pi

R_EARTH_M = 6371000

class LatLong:
    def __init__(self, latitude: float, longitude: float):
        self.latitude = latitude
        self.longitude = longitude
    
    def heading_to(self, other: 'LatLong') -> float:
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
        lat1 = radians(self.latitude)
        lon1 = radians(self.longitude)
        lat2 = radians(other.latitude)
        lon2 = radians(other.longitude)

        dlat = lat2 - lat1
        dlon = lon2 - lon1

        a = sin(dlat / 2) ** 2 + cos(lat1) * cos(lat2) * sin(dlon / 2) ** 2
        c = 2 * asin(min(1.0, a ** 0.5))

        distance = R_EARTH_M * c
        return distance

    @staticmethod
    def from_xy(origin: 'LatLong', north_m: float, east_m: float) -> 'LatLong':
        lat1 = radians(origin.latitude)
        lon1 = radians(origin.longitude)

        angular_distance = north_m / R_EARTH_M
        bearing = atan2(east_m, north_m)

        lat2 = asin(sin(lat1) * cos(angular_distance) +
                    cos(lat1) * sin(angular_distance) * cos(bearing))

        lon2 = lon1 + atan2(sin(bearing) * sin(angular_distance) * cos(lat1),
                            cos(angular_distance) - sin(lat1) * sin(lat2))

        lon2 = (lon2 + 3 * pi) % (2 * pi) - pi

        return LatLong(degrees(lat2), degrees(lon2))

    def __str__(self) -> str:
        return f"LatLong(latitude={self.latitude}, longitude={self.longitude})"