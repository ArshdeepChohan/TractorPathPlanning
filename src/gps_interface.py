"""
Abstract Interface for GPS Data Source.
Ensures the system can switch between Simulation and Real Hardware transparently.
"""
from abc import ABC, abstractmethod

class GPSInterface(ABC):
    @abstractmethod
    def get_state(self):
        """
        Returns the current state of the vehicle.
        
        Returns:
            dict: {
                'lat': float,       # Latitude in degrees
                'lon': float,       # Longitude in degrees
                'heading': float,   # Heading in degrees (0=North, 90=East)
                'speed': float,     # Speed in m/s
                'x': float,         # Local X (East) in meters
                'y': float          # Local Y (North) in meters
            }
        """
        pass
    
    @abstractmethod
    def update(self, steering_angle, dt):
        """
        Update the state based on control inputs (only used in simulation).
        For real hardware, this might be a no-op or a state estimator update.
        
        Args:
            steering_angle (float): Steering angle in radians.
            dt (float): Time step in seconds.
        """
        pass
