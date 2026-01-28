"""
Geographic utility functions for coordinate transformations.
Converts between GPS (Lat/Lon) and Local ENU (East-North-Up) Cartesian coordinates.
"""
import math
from src import config

# Earth radius in meters
R_EARTH = 6378137.0

def gps_to_local(lat, lon):
    """
    Convert GPS coordinates (lat, lon) to local (x, y) meters.
    Origin is defined in config.REF_LAT, config.REF_LON.
    X axis: Easting
    Y axis: Northing
    """
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    ref_lat_rad = math.radians(config.REF_LAT)
    ref_lon_rad = math.radians(config.REF_LON)

    # Equirectangular approximation (valid for small distances)
    x = (lon_rad - ref_lon_rad) * math.cos(ref_lat_rad) * R_EARTH
    y = (lat_rad - ref_lat_rad) * R_EARTH
    
    return x, y

def local_to_gps(x, y):
    """
    Convert local (x, y) meters to GPS coordinates (lat, lon).
    Origin is defined in config.REF_LAT, config.REF_LON.
    """
    ref_lat_rad = math.radians(config.REF_LAT)
    ref_lon_rad = math.radians(config.REF_LON)

    lat_rad = ref_lat_rad + (y / R_EARTH)
    lon_rad = ref_lon_rad + (x / (R_EARTH * math.cos(ref_lat_rad)))

    lat = math.degrees(lat_rad)
    lon = math.degrees(lon_rad)

    return lat, lon

def normalize_angle(angle):
    """
    Normalize angle to be within [-pi, pi].
    """
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

def normalize_angle_deg(angle):
    """
    Normalize angle in degrees to [0, 360).
    """
    return angle % 360.0
