"""
Configuration constants for the Autonomous Tractor Guidance System.
"""
import math

# ================= TRACTOR KINEMATICS =================
WHEELBASE = 2.5             # meters (distance between front and rear axle)
MAX_STEER_ANGLE = 40.0      # degrees
MIN_TURN_RADIUS = WHEELBASE / math.sin(math.radians(MAX_STEER_ANGLE))
TRACTOR_SPEED = 1.5        # m/s (≈ 5.4 km/h)

# ================= CONTROL PARAMETERS =================
LOOKAHEAD_DISTANCE = 2.5    # for pure pursuit / Stanley
K_P = 1.0                   # Proportional gain
DT = 0.1                    # Time step (s)

# ================= FIELD GEOMETRY =================
REF_LAT = 30.8635559
REF_LON = 75.8582732

FIELD_LENGTH = 78.0        # meters (X direction) ✔ updated
FIELD_WIDTH = 40.0          # meters (Y direction) ✔ updated
ROW_SPACING = 5.0           # meters between rows ✔ updated
WAYPOINT_SPACING = 0.5      # meters between path points ✔ matches STEP

# Optional: Field boundary pins (lat, lon)
FIELD_PINS = [
    (30.8635559, 75.8582732),
    (30.8635522, 75.8590846),
    (30.8638520, 75.8590893),
    (30.8638414, 75.8582699),
]

# ================= PATH GENERATION =================
DESIRED_TURN_RADIUS = ROW_SPACING / 2.0   # Ideal U-turn radius
U_TURN_RADIUS = max(MIN_TURN_RADIUS, DESIRED_TURN_RADIUS)

if U_TURN_RADIUS > DESIRED_TURN_RADIUS:
    print(f"[INFO] Turn radius adjusted to {U_TURN_RADIUS:.2f}m for tractor feasibility")

# ================= SIMULATION =================
SIM_DURATION = 500          # seconds (enough for full coverage)
GPS_NOISE_STD = 0.02        # meters
INITIAL_HEADING = 0.0       # degrees (facing East)
