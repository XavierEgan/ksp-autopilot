import krpc

def print_depature_airport_info():
    conn = krpc.connect(name='Get Runway Info')

    if not conn.space_center:
        print("No space center.")
        return

    vessel = conn.space_center.active_vessel

    heading = vessel.flight().heading
    lat = vessel.flight().latitude
    long = vessel.flight().longitude
    altitude = vessel.flight().mean_altitude

    print(f"Runway(LatLongLine(LatLong({lat}, {long}), {heading}), {altitude})")

if __name__ == "__main__":
    print_depature_airport_info()