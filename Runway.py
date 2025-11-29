from Math import LatLongLine, LatLong

class Runway:
    line: LatLongLine
    threashold_altitude: float
    name: str = ""

    def __init__(self, line: LatLongLine, threashold_altitude: float):
        self.line = line
        self.threashold_altitude = threashold_altitude

    def __repr__(self) -> str:
        return f"Runway(name={self.name}, heading={self.line.heading}, threashold_altitude={self.threashold_altitude})"
    
    def __str__(self) -> str:
        return f"Runway {self.name} (Heading: {self.line.heading:.2f}, Threashold Altitude: {self.threashold_altitude:.2f}m)"

    def cross_track_error(self, plane_latlong: LatLong) -> float:
        return self.line.cross_track_error(plane_latlong)

    def get_point_at_distance(self, distance_meters: float) -> LatLong:
        return self.line.get_point_at_distance(distance_meters)

RUNWAY_YMML = Runway(
    LatLongLine(LatLong(-37.66444129883061, 144.8495527799204), 209.9854278564453),
    140.34361927770078
)

RUNWAY_YPAD = Runway(
    LatLongLine(LatLong(-34.93613994127511, 138.53732454791594), 209.97891235351562),
    14.020269891247153
)

RUNWAY_KSC = Runway(
    LatLongLine(LatLong(28.61285846502337, -80.61755918001734), 89.89300537109375), 80.82715495210141
)