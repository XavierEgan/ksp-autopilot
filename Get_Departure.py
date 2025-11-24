import krpc
import time

from Math import LatLongLine, LatLong

def print_depature_airport_info():
    conn = krpc.connect(name='Get Departure Info')

    if not conn.space_center:
        print("No space center.")
        return

    vessel = conn.space_center.active_vessel

    heading = vessel.flight().heading
    lat = vessel.flight().latitude
    long = vessel.flight().longitude
    altitude = vessel.flight().mean_altitude

    print(f"LatLongLine(LatLong({lat}, {long}), {heading})")
    print(f"{altitude}")

if __name__ == "__main__":
    print_depature_airport_info()