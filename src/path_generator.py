# import numpy as np
# import matplotlib.pyplot as plt
# import csv
# import os

# # ================= PARAMETERS =================
# FIELD_L = 78
# FIELD_W = 40
# ROW = 5

# STEP = 0.5
# R = ROW / 2
# ARC_PTS = 80
# LEAD = R

# BASE = os.path.dirname(os.path.dirname(__file__))
# os.makedirs(BASE + "/data", exist_ok=True)
# os.makedirs(BASE + "/plots", exist_ok=True)

# # ================= HELPERS =================
# def straight(x1, x2, y):
#     step = STEP if x2 > x1 else -STEP
#     xs = np.arange(x1, x2 + step, step)
#     return [(x, y) for x in xs]

# def diagonal(x1, y1, x2, y2):
#     dist = np.hypot(x2 - x1, y2 - y1)
#     n = int(dist / STEP)
#     xs = np.linspace(x1, x2, n)
#     ys = np.linspace(y1, y2, n)
#     return list(zip(xs, ys))

# def u_turn(y1, y2, direction):
#     path = []

#     yc = (y1 + y2) / 2

#     if direction == "right":
#         xc = FIELD_L - R
#         theta = np.linspace(-np.pi/2, np.pi/2, ARC_PTS)

#     else:
#         xc = R
#         theta = np.linspace(np.pi/2, 3*np.pi/2, ARC_PTS)

#     for t in theta:
#         x = xc + R * np.cos(t)
#         y = yc + R * np.sin(t)
#         path.append((x, y))

#     return path





# # ================= PATH =================
# path = []

# # 1️⃣ Start A = (0,0)
# path.append((0, 0))

# # 2️⃣ Move 2m diagonally
# path += diagonal(0, 0, 2, 2)

# # 3️⃣ First baseline = FIRST ROW (10m + full row)
# first_row_y = 2
# path += straight(2, FIELD_L, first_row_y)

# # # Now remaining rows (parallel to first baseline)
# # rows = np.arange(first_row_y + ROW, FIELD_W + 0.01, ROW)

# # for i, y in enumerate(rows):
# #     if i % 2 == 0:
# #         path += u_turn(FIELD_L, y-ROW, y, "right")
# #         path += straight(FIELD_L, 2, y)
# #     else:
# #         path += u_turn(2, y-ROW, y, "left")
# #         path += straight(2, FIELD_L, y)

# # Now remaining rows (auto-fit till top boundary)
# num_rows = int((FIELD_W - first_row_y) / ROW)

# rows = np.linspace(first_row_y + ROW, FIELD_W, num_rows)

# for i, y in enumerate(rows):
#     if i % 2 == 0:
#         path += u_turn(FIELD_L, y-ROW, y, "right")
#         path += straight(FIELD_L, 2, y)
#     else:
#         path += u_turn(2, y-ROW, y, "left")
#         path += straight(2, FIELD_L, y)

# # ================= SAVE =================
# with open(BASE + "/data/waypoints.csv", "w", newline="") as f:
#     w = csv.writer(f)
#     w.writerow(["x", "y"])
#     w.writerows(path)

# # ================= PLOT =================
# x, y = zip(*path)

# plt.figure(figsize=(10, 6))
# plt.plot(x, y, label="Tractor Path")
# plt.scatter(x, y, s=6)

# # Field Boundary Rectangle
# plt.plot([0, FIELD_L, FIELD_L, 0, 0],
#          [0, 0, FIELD_W, FIELD_W, 0],
#          linestyle="--", label="Field Boundary")

# plt.title("Tractor Path with Baseline, Diagonal Start & Smooth U-Turns")
# plt.xlabel("X (m)")
# plt.ylabel("Y (m)")
# plt.legend()
# plt.grid(True)
# plt.savefig(BASE + "/plots/path_plot.png", dpi=300)
# plt.show()


import numpy as np
import matplotlib.pyplot as plt
import csv
import os

# ================= PARAMETERS =================
FIELD_L = 78   # Field Length (X)
FIELD_W = 40   # Field Width (Y)
ROW = 5        # Row spacing (swath width)

STEP = 0.5     # Step size for waypoints
R = ROW / 2    # Turning radius
ARC_PTS = 50   # Points in U-turn arc

BASE = os.path.dirname(os.path.dirname(__file__))
os.makedirs(BASE + "/data", exist_ok=True)
os.makedirs(BASE + "/plots", exist_ok=True)

# ================= HELPERS =================
def straight(x1, x2, y):
    """Generates a straight line segment."""
    dist = abs(x2 - x1)
    n = int(dist / STEP) + 1
    xs = np.linspace(x1, x2, n)
    return [(x, y) for x in xs]

def diagonal(x1, y1, x2, y2):
    """Generates a diagonal line segment."""
    dist = np.hypot(x2 - x1, y2 - y1)
    n = max(2, int(dist / STEP) + 1)
    xs = np.linspace(x1, x2, n)
    ys = np.linspace(y1, y2, n)
    return list(zip(xs, ys))

def u_turn(y_start, y_end, direction):
    """Generates a smooth U-turn arc."""
    path = []
    
    # Center of the turn
    yc = (y_start + y_end) / 2
    
    # Identify theta range based on direction and traversal
    # We are moving from y_start to y_end
    # Since we are moving UP the field (increasing Y), y_end > y_start
    
    if direction == "right":
        # Right side turn: Pivot is at x = FIELD_L - R
        xc = FIELD_L - R
        
        # Arc from bottom (-pi/2) to top (pi/2)
        theta = np.linspace(-np.pi/2, np.pi/2, ARC_PTS)
        
        for t in theta:
            x = xc + R * np.cos(t)
            y = yc + R * np.sin(t)
            path.append((x, y))
            
    elif direction == "left":
        # Left side turn: Pivot is at x = R
        xc = R
        
        # Arc from bottom (3pi/2) to top (pi/2)
        # Note: We must traverse the circle along the LEFT side.
        # Angles: 270 deg -> 90 deg (counter-clockwise or clockwise?)
        # We need to trace the arc x < xc. 
        # cos(t) < 0 for t in (pi/2, 3pi/2).
        # But we go from y_start (bottom) to y_end (top).
        # Bottom corresponds to 3pi/2 (270). Top corresponds to pi/2 (90).
        # We should decrease angle: 270 -> 180 -> 90.
        
        theta = np.linspace(3*np.pi/2, np.pi/2, ARC_PTS)
        
        for t in theta:
            x = xc + R * np.cos(t)
            y = yc + R * np.sin(t)
            path.append((x, y))

    return path

# ================= PATH GENERATION =================
path = []

# Helper to add a break in the path (NaN) to prevent connecting lines
def add_break():
    path.append((np.nan, np.nan))

# 1️⃣ Start at A = (0, 0)
path.append((0, 0))

# 2️⃣ Move diagonally to the start of the first row
# First row center should be at Y = ROW/2 to keep implement inside boundary (0 to ROW)
first_row_y = ROW / 2  # 2.5m
path += diagonal(0, 0, R, first_row_y)
add_break()

# 3️⃣ First baseline (Left -> Right)
# Starts at x=R (2.5), Ends at x=FIELD_L-R (75.5)
path += straight(R, FIELD_L - R, first_row_y)
add_break()

# 4️⃣ Generate subsequent rows
# Calculate Y coordinates for all rows
# We want rows centered at 2.5, 7.5, 12.5 ...
# Max Y is FIELD_W - R = 40 - 2.5 = 37.5
rows = np.arange(first_row_y + ROW, FIELD_W, ROW)

prev_y = first_row_y

for i, y in enumerate(rows):
    # i=0 is the first turn (after the first row).
    # First row was L->R. So we need to turn Right.
    
    if i % 2 == 0:
        # Turn Right (upwards)
        path += u_turn(prev_y, y, "right")
        add_break()
        
        # Straight line back (Right -> Left)
        path += straight(FIELD_L - R, R, y)
        add_break()
        
    else:
        # Turn Left (upwards)
        path += u_turn(prev_y, y, "left")
        add_break()
        
        # Straight line forward (Left -> Right)
        path += straight(R, FIELD_L - R, y)
        add_break()

    prev_y = y

# ================= SAVE =================
with open(BASE + "/data/waypoints.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["x", "y"])
    w.writerows(path)

# ================= PLOT =================
# Filter out NaNs for separate segments if needed, but plt.plot handles NaNs by breaking the line.
x, y = zip(*path)

plt.figure(figsize=(10, 6))

# Plot the path
plt.plot(x, y, label="Tractor Path", linewidth=2)

# Scatter start/end points of segments (optional, filtering NaNs)
# valid_x = [px for px in x if not np.isnan(px)]
# valid_y = [py for py in y if not np.isnan(py)]
# plt.scatter(valid_x, valid_y, s=2, c='red', alpha=0.5)

# Field boundary
plt.plot([0, FIELD_L, FIELD_L, 0, 0],
         [0, 0, FIELD_W, FIELD_W, 0],
         'k--', linewidth=1.5, label="Field Boundary")

plt.title("Tractor Path Planning (Smooth U-Turns)")
plt.xlabel("X (m)")
plt.ylabel("Y (m)")
plt.axis("equal") # Ensure 1m x = 1m y for true semicircles
plt.legend(loc="upper right")
plt.grid(True, linestyle=':', alpha=0.6)
plt.tight_layout()
plt.savefig(BASE + "/plots/path_plot.png", dpi=300)
plt.show()
