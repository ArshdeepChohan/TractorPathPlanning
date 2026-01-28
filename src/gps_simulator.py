"""
GPS Simulator with Kinematic Bicycle Model.
Simulates tractor movement and generates fake GPS coordinates.
"""
import math
import random
from src.gps_interface import GPSInterface
from src import config
from src import geo_utils

class GPSSimulator(GPSInterface):
    def __init__(self, start_lat=config.REF_LAT, start_lon=config.REF_LON, start_heading=config.INITIAL_HEADING):
        self.lat = start_lat
        self.lon = start_lon
        self.heading = math.radians(start_heading) # Internal state in radians
        self.speed = config.TRACTOR_SPEED
        self.wheelbase = config.WHEELBASE
        
        # Local coordinates state
        self.x, self.y = geo_utils.gps_to_local(self.lat, self.lon)
        
    def update(self, steering_angle, dt):
        """
        Update vehicle state using Kinematic Bicycle Model.
        
        Args:
            steering_angle (float): Steering angle in radians.
            dt (float): Time step in seconds.
        """
        # Limit steering angle
        max_steer = math.radians(config.MAX_STEER_ANGLE)
        steering_angle = max(min(steering_angle, max_steer), -max_steer)
        
        # Bicycle Model Kinematics
        # x_dot = v * cos(heading)
        # y_dot = v * sin(heading)
        # heading_dot = (v / L) * tan(steering_angle)
        
        self.x += self.speed * math.cos(self.heading) * dt
        self.y += self.speed * math.sin(self.heading) * dt
        self.heading += (self.speed / self.wheelbase) * math.tan(steering_angle) * dt
        
        # Normalize heading
        self.heading = geo_utils.normalize_angle(self.heading)
        
        # Update GPS coordinates from new local position
        self.lat, self.lon = geo_utils.local_to_gps(self.x, self.y)
        
        # Add simulated noise (optional)
        # self.lat += random.gauss(0, config.GPS_NOISE_STD / 111320.0) 
        # self.lon += random.gauss(0, config.GPS_NOISE_STD / (111320.0 * math.cos(math.radians(self.lat))))

    def get_state(self):
        """
        Return current simulated state.
        """
        # Convert heading to degrees for interface
        heading_deg = math.degrees(self.heading)
        
        # Add noise to output (sensor noise)
        lat_noise = self.lat + random.gauss(0, config.GPS_NOISE_STD / 111320.0)
        lon_noise = self.lon + random.gauss(0, config.GPS_NOISE_STD / (111320.0 * math.cos(math.radians(self.lat))))
        x_noise, y_noise = geo_utils.gps_to_local(lat_noise, lon_noise)

        return {
            'lat': lat_noise,
            'lon': lon_noise,
            'heading': heading_deg,
            'speed': self.speed,
            'x': x_noise,
            'y': y_noise
        }
