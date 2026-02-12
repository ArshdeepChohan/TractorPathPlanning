import numpy as np
import csv
import math
import os

EARTH_RADIUS = 6378137


# ======================================================
# CONVERSION FUNCTIONS
# ======================================================

def latlon_to_xy(lat, lon, lat0, lon0):
    dlat = math.radians(lat - lat0)
    dlon = math.radians(lon - lon0)

    x = EARTH_RADIUS * dlon * math.cos(math.radians(lat0))
    y = EARTH_RADIUS * dlat

    return x, y


def xy_to_latlon(x, y, lat0, lon0):
    dlat = y / EARTH_RADIUS
    dlon = x / (EARTH_RADIUS * math.cos(math.radians(lat0)))

    lat = lat0 + math.degrees(dlat)
    lon = lon0 + math.degrees(dlon)

    return lat, lon


# ======================================================
# LAWNMOWER PATH
# ======================================================

def lawnmower_path(field_length, field_width, row_spacing, point_spacing):

    waypoints = []
    y = 0.0
    direction = 1

    while y <= field_width:

        if direction == 1:
            x_vals = np.arange(0, field_length + point_spacing, point_spacing)
        else:
            x_vals = np.arange(field_length, -point_spacing, -point_spacing)

        for x in x_vals:
            waypoints.append((x, y))

        y_next = y + row_spacing

        if y_next <= field_width:
            x_turn = field_length if direction == 1 else 0
            theta = np.linspace(0, np.pi, 40)

            for t in theta:
                x_arc = x_turn + (row_spacing/2) * np.cos(t)
                y_arc = y + (row_spacing/2) * np.sin(t)
                waypoints.append((x_arc, y_arc))

        direction *= -1
        y = y_next

    return waypoints


# ======================================================
# MAIN GPS GENERATOR FUNCTION
# ======================================================

def generate_gps_waypoints(pointA, pointB, pointC, row_spacing, point_spacing):

    lat0, lon0 = pointA

    # Convert field corners to local XY
    xB, yB = latlon_to_xy(pointB[0], pointB[1], lat0, lon0)
    xC, yC = latlon_to_xy(pointC[0], pointC[1], lat0, lon0)

    field_length = abs(xB)
    field_width = abs(yC)

    meter_path = lawnmower_path(
        field_length,
        field_width,
        row_spacing,
        point_spacing
    )

    gps_path = []

    for x, y in meter_path:
        lat, lon = xy_to_latlon(x, y, lat0, lon0)
        gps_path.append((lat, lon))

    os.makedirs("data", exist_ok=True)
    csv_path = "data/gps_waypoints.csv"

    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["latitude", "longitude"])
        writer.writerows(gps_path)

    return csv_path
