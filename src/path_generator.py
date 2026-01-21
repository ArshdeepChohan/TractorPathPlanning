"""
Path Generator for Autonomous Tractor.
Generates Boustrophedon (Lawnmower) coverage paths with smooth U-turns.
Handles cases where row spacing is tighter than the minimum turning radius using Bulb Turns.
"""
import numpy as np
import math
import csv
from src import config
from src import geo_utils

class PathGenerator:
    def __init__(self):
        self.waypoints = [] # List of (x, y) tuples in local coordinates
        self.segments = []  # List of {'type': 'straight'|'turn', 'points': [(x,y), ...]}
    
    def _clamp(self, x, y):
        x = max(0.0, min(x, config.FIELD_LENGTH))
        y = max(0.0, min(y, config.FIELD_WIDTH))
        return x, y
        
    def generate_field_coverage(self):
        """
        Generates a full field coverage pattern.
        """
        self.waypoints = []
        self.segments = []
        
        # Calculate number of passes
        num_rows = int(config.FIELD_WIDTH / config.ROW_SPACING)
        num_rows = max(1, num_rows)
        
        print(f"Generating path for {num_rows} rows...")
        print(f"Field Dimensions: {config.FIELD_LENGTH}x{config.FIELD_WIDTH} m")
        print(f"Row Spacing: {config.ROW_SPACING} m")
        
        for i in range(num_rows):
            y_curr = i * config.ROW_SPACING
            y_curr = min(y_curr, config.FIELD_WIDTH)
            
            # Determine direction of pass
            # Even rows: Left to Right (0 -> Length)
            # Odd rows: Right to Left (Length -> 0)
            if i % 2 == 0:
                start_x = 0.0
                # Reserve margin for U-turn
                margin = min(config.HEADLAND_LENGTH, max(config.MIN_TURN_RADIUS, config.ROW_SPACING / 2.0))
                end_x = config.FIELD_LENGTH - margin
                self._add_straight_line(start_x, end_x, y_curr)
                
                # If not the last row, add U-turn to next row
                if i < num_rows - 1:
                    y_next = (i + 1) * config.ROW_SPACING
                    self._add_u_turn(end_x, y_curr, y_next, turn_direction='left')
            else:
                start_x = config.FIELD_LENGTH
                margin = min(config.HEADLAND_LENGTH, max(config.MIN_TURN_RADIUS, config.ROW_SPACING / 2.0))
                end_x = margin
                self._add_straight_line(start_x, end_x, y_curr)
                
                # If not the last row, add U-turn to next row
                if i < num_rows - 1:
                    y_next = (i + 1) * config.ROW_SPACING
                    self._add_u_turn(end_x, y_curr, y_next, turn_direction='right')
                    
        return self.waypoints

    def _add_straight_line(self, x1, x2, y):
        """
        Adds points for a straight line segment.
        """
        dist = abs(x2 - x1)
        if dist < 1e-3: return # Skip zero length
        
        num_points = max(2, int(dist / config.WAYPOINT_SPACING))
        
        xs = np.linspace(x1, x2, num_points)
        pts = []
        for x in xs:
            cx, cy = self._clamp(x, y)
            pts.append((cx, cy))
            self.waypoints.append((cx, cy))
        self.segments.append({'type': 'straight', 'points': pts})

    def _add_u_turn(self, x_pivot, y_start, y_end, turn_direction='left'):
        """
        Adds a smooth U-turn connecting two parallel rows.
        Automatically chooses between Semicircle and Bulb turn based on geometry.
        """
        row_dist = abs(y_end - y_start)
        min_diameter = 2 * config.MIN_TURN_RADIUS
        
        # Determine if we need a bulb turn (if rows are too close)
        needs_bulb = row_dist < min_diameter
        
        if needs_bulb:
            self._add_bulb_turn(x_pivot, y_start, y_end, turn_direction)
        else:
            self._add_semicircle_turn(x_pivot, y_start, y_end, turn_direction)

    def _add_semicircle_turn(self, x_pivot, y_start, y_end, turn_direction):
        """
        Simple 180 degree turn.
        """
        radius = abs(y_end - y_start) / 2
        
        # Determine heading at start of turn
        # If x_pivot is at config.FIELD_LENGTH (Right side), we arrived heading East (0)
        # If x_pivot is at 0 (Left side), we arrived heading West (180)
        right_side = (x_pivot > config.FIELD_LENGTH/2)
        start_heading = 0 if right_side else 180
        
        # Keep semicircle inside field by shifting pivot inward by radius
        if right_side:
            x_pivot = min(x_pivot, config.FIELD_LENGTH - radius)
        else:
            x_pivot = max(x_pivot, radius)
        
        # If turn_direction is Left: +90, +90
        # If turn_direction is Right: -90, -90
        turn_angle = 180 if turn_direction == 'left' else -180
        
        self._add_arc(x_pivot, y_start, start_heading, turn_angle, radius)

    def _add_bulb_turn(self, x_pivot, y_start, y_end, turn_direction):
        """
        Implements an Asymmetric Bulb Turn for tight row spacings.
        Algorithm:
        1. Extend straight into headland.
        2. Turn 90 deg AWAY from target row (Radius r1 = MIN_TURN_RADIUS).
        3. Turn 270 deg TOWARDS target row (Radius r2 calculated to bridge gap).
        4. Drive straight back to pivot line.
        """
        r1 = config.MIN_TURN_RADIUS * 1.05
        d = abs(y_end - y_start)
        
        # Calculate R2 to bridge the gap
        # Geometry: y_start +/- r1 (after turn 1)
        # Target: y_end
        # Span of Turn 2 (270 deg) is 2 * r2
        # We need: | (y_start +/- r1) - y_end | = 2 * r2
        # | y_start - y_end +/- r1 | = 2 * r2
        # |-d + r1| = 2 * r2  (Assuming y_end > y_start and we turned away (Right/Minus))
        # d + r1 = 2 * r2
        r2 = (d + r1) / 2
        
        if r2 < config.MIN_TURN_RADIUS:
            # Should not happen if d > 0, but safety check
            r2 = config.MIN_TURN_RADIUS
            
        right_side = (x_pivot > config.FIELD_LENGTH/2)
        # Pivot line moved inward to keep arcs inside
        if right_side:
            x_pivot = min(x_pivot, config.FIELD_LENGTH - r1)
        else:
            x_pivot = max(x_pivot, r1)
        x_dir = -1 if right_side else 1 
        start_heading = 0 if right_side else 180
        
        # 1. Straight Extension
        ext = r1
        p1_x = x_pivot + x_dir * ext
        self._add_straight_line(x_pivot, p1_x, y_start)
        
        # 2. Turn 90 deg AWAY
        # If Target is Left (North), Turn Right (South)
        angle1 = -90 if turn_direction == 'left' else 90
        self._add_arc(p1_x, y_start, start_heading, angle1, r1)
        
        # Calculate position after Turn 1 for continuity
        # We need the last point added
        p2_x, p2_y = self.waypoints[-1]
        
        # Heading after Turn 1
        heading_2 = start_heading + angle1
        
        # 3. Turn 270 deg TOWARDS
        # If we turned Right (-90), we now Turn Left (+270)
        angle2 = 270 if turn_direction == 'left' else -270
        self._add_arc(p2_x, p2_y, heading_2, angle2, r2)
        
        # 4. Straight Line Back
        # We should be at y_end and heading West/East (opposite to start)
        p3_x, p3_y = self.waypoints[-1]
        self._add_straight_line(p3_x, x_pivot, p3_y)

    def _add_arc(self, start_x, start_y, heading_start_deg, turn_angle, radius):
        """
        Adds an arc starting from a pose.
        heading_start_deg: 0=East, 90=North.
        turn_angle: +ve for Left, -ve for Right.
        """
        heading_rad = math.radians(heading_start_deg)
        
        # Center calculation
        # If turning Left (+), Center is to the Left of heading vector
        # Left of (cos, sin) is (-sin, cos)
        if turn_angle > 0:
            cx = start_x - radius * math.sin(heading_rad)
            cy = start_y + radius * math.cos(heading_rad)
            start_angle = heading_rad - math.pi/2
        else: # Right
            cx = start_x + radius * math.sin(heading_rad)
            cy = start_y - radius * math.cos(heading_rad)
            start_angle = heading_rad + math.pi/2
            
        end_angle = start_angle + math.radians(turn_angle)
        
        # Generate points
        # Ensure we take the correct direction around the circle
        if turn_angle > 0:
            if end_angle < start_angle: end_angle += 2*math.pi # Should not happen with linspace but good for safety
            steps = int(abs(turn_angle) / 5) + 2
            angles = np.linspace(start_angle, end_angle, steps)
        else:
            steps = int(abs(turn_angle) / 5) + 2
            angles = np.linspace(start_angle, end_angle, steps)
            
        pts = []
        for theta in angles:
            px = cx + radius * math.cos(theta)
            py = cy + radius * math.sin(theta)
            px, py = self._clamp(px, py)
            pts.append((px, py))
            self.waypoints.append((px, py))
        self.segments.append({'type': 'turn', 'points': pts})
            
    def save_to_csv(self, filename):
        """
        Converts local waypoints to GPS and saves to CSV.
        """
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['latitude', 'longitude'])
            
            for x, y in self.waypoints:
                lat, lon = geo_utils.local_to_gps(x, y)
                writer.writerow([lat, lon])
        
        print(f"Path saved to {filename} with {len(self.waypoints)} waypoints.")

    def save_local_csv(self, filename):
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['x', 'y'])
            for x, y in self.waypoints:
                writer.writerow([x, y])
        print(f"Path saved to {filename} with {len(self.waypoints)} waypoints.")

    def save_segments_csv(self, filename):
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['type', 'x', 'y'])
            for seg in self.segments:
                for x, y in seg['points']:
                    writer.writerow([seg['type'], x, y])
        print(f"Segments saved to {filename}")
