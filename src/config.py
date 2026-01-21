"""
Configuration constants for the Autonomous Tractor Guidance System.
"""
import math

# ================= TRACTOR KINEMATICS =================
WHEELBASE = 2.5             # Meters (distance between front and rear axle)
MAX_STEER_ANGLE = 40.0      # Degrees
MIN_TURN_RADIUS = WHEELBASE / math.sin(math.radians(MAX_STEER_ANGLE))
TRACTOR_SPEED = 1.5         # m/s (approx 5.4 km/h)

# ================= CONTROL PARAMETERS =================
LOOKAHEAD_DISTANCE = 3.5    # Meters (Pure Pursuit lookahead)
K_P = 1.0                   # Proportional gain (if using Stanley)
DT = 0.1                    # Time step for simulation (seconds)

# ================= FIELD GEOMETRY =================
# Reference point for local Cartesian system (Origin)
# Set to Pin 1 of your field
REF_LAT = 30.8635559
REF_LON = 75.8582732

FIELD_LENGTH = 78.3         # Meters
FIELD_WIDTH = 38.8          # Meters
# Set pass spacing to realistic value for coverage
ROW_SPACING = 5.0           # Meters (Width between passes)
HEADLAND_LENGTH = 10.0      # Meters (Space for U-turns at ends)

# Optional: Field boundary pins (lat, lon)
FIELD_PINS = [
    (30.8635559, 75.8582732),  # Pin 1
    (30.8635522, 75.8590846),  # Pin 2
    (30.8638520, 75.8590893),  # Pin 3
    (30.8638414, 75.8582699),  # Pin 4
]

# ================= PATH GENERATION =================
WAYPOINT_SPACING = 0.5      # Meters (Distance between path points)
U_TURN_RADIUS = ROW_SPACING / 2.0
# Ensure turn radius is physically possible
if U_TURN_RADIUS < MIN_TURN_RADIUS:
    print(f"WARNING: Desired U-turn radius ({U_TURN_RADIUS}m) is less than min tractor radius ({MIN_TURN_RADIUS:.2f}m).")
    U_TURN_RADIUS = MIN_TURN_RADIUS * 1.1 # Adjust to be feasible

# ================= SIMULATION =================
SIM_DURATION = 300          # Seconds (Max simulation time)
GPS_NOISE_STD = 0.02        # Meters (Standard deviation of GPS noise)
INITIAL_HEADING = 90.0      # Degrees (East)
