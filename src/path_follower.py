"""
Path Follower using Pure Pursuit Algorithm.
Calculates steering angle to follow a set of waypoints.
"""
import math
import numpy as np
from src import config
from src import geo_utils

class PurePursuitController:
    def __init__(self):
        self.lookahead_dist = config.LOOKAHEAD_DISTANCE
        self.wheelbase = config.WHEELBASE
        self.last_idx = 0 # Track progress to avoid searching full path every time

    def get_steering_control(self, state, path):
        """
        Calculates steering angle based on current state and path.
        
        Args:
            state (dict): {'x', 'y', 'heading', ...}
            path (list): List of (x, y) tuples
            
        Returns:
            steering_angle (float): In radians
            cross_track_error (float): Distance to closest path point
            target_idx (int): Index of lookahead point
        """
        if not path:
            return 0.0, 0.0, 0
            
        cx = [p[0] for p in path]
        cy = [p[1] for p in path]
        
        curr_x = state['x']
        curr_y = state['y']
        curr_yaw = math.radians(state['heading']) # Convert to radians
        
        # 1. Find closest point on path
        # Search starting from last_idx to optimize
        # We assume the vehicle moves forward along the path
        
        # Search window (e.g., check next 500 points)
        search_len = 500
        start_search = self.last_idx
        end_search = min(self.last_idx + search_len, len(path))
        
        # If we reached end, search from beginning (loop?) 
        # No, for field coverage we stop.
        
        dx = [curr_x - icx for icx in cx[start_search:end_search]]
        dy = [curr_y - icy for icy in cy[start_search:end_search]]
        d = [np.hypot(idx, idy) for (idx, idy) in zip(dx, dy)]
        
        if not d:
            # End of path reached
            return 0.0, 0.0, len(path)-1
            
        min_dist = min(d)
        closest_idx = start_search + d.index(min_dist)
        self.last_idx = closest_idx
        
        # Cross Track Error is min_dist
        # Sign of XTE? (Left or Right of path)
        # Vector from path to vehicle
        path_dx = cx[min(closest_idx+1, len(path)-1)] - cx[closest_idx]
        path_dy = cy[min(closest_idx+1, len(path)-1)] - cy[closest_idx]
        # Cross product 2D
        # If path vector is A, vehicle vector from closest is B.
        # This is strictly distance.
        cross_track_error = min_dist
        
        # 2. Find Lookahead Point
        # We want a point L distance away from (curr_x, curr_y)
        # We search forward from closest_idx
        
        target_idx = closest_idx
        found_target = False
        
        for i in range(closest_idx, len(path)):
            dist = np.hypot(curr_x - cx[i], curr_y - cy[i])
            if dist > self.lookahead_dist:
                target_idx = i
                found_target = True
                break
                
        if not found_target:
            target_idx = len(path) - 1
            
        target_x = cx[target_idx]
        target_y = cy[target_idx]
        
        # 3. Calculate Steering Angle
        # Transform target to vehicle coordinates
        dx = target_x - curr_x
        dy = target_y - curr_y
        
        # Rotate to vehicle frame
        # x_local = dx * cos(-yaw) - dy * sin(-yaw)
        # y_local = dx * sin(-yaw) + dy * cos(-yaw)
        
        # Standard rotation matrix for aligning x-axis with heading:
        # Rx = x*cos(theta) + y*sin(theta)
        # Ry = -x*sin(theta) + y*cos(theta)
        # Wait, if heading is theta.
        # Local x (forward) = dx * cos(theta) + dy * sin(theta)
        # Local y (lateral) = -dx * sin(theta) + dy * cos(theta)
        
        local_y = -dx * math.sin(curr_yaw) + dy * math.cos(curr_yaw)
        local_x = dx * math.cos(curr_yaw) + dy * math.sin(curr_yaw) # Just for check, should be positive
        
        # Pure Pursuit Curvature
        # k = 2 * y / L^2
        dist_sq = dx**2 + dy**2 # This is roughly L^2
        
        # Steering angle delta = atan(k * wheelbase)
        # delta = atan( 2 * wheelbase * local_y / dist_sq )
        
        # Ensure we don't divide by zero
        if dist_sq < 0.01:
            return 0.0, cross_track_error, target_idx
            
        steer_rad = math.atan2(2.0 * self.wheelbase * local_y, dist_sq)
        
        # Clamp steering
        max_steer = math.radians(config.MAX_STEER_ANGLE)
        steer_rad = max(min(steer_rad, max_steer), -max_steer)
        
        return steer_rad, cross_track_error, target_idx
