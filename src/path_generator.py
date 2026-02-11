import numpy as np
import matplotlib.pyplot as plt
import csv
import os

# ================= PARAMETERS =================
FIELD_L = 78   # Field Length (X)
FIELD_W = 40   # Field Width (Y)
ROW = 5        # Row spacing (swath width)

STEP = 0.5     # Waypoint resolution
R = ROW / 2    # Turning radius
ARC_PTS = 60   # Points in U-turn arc

BASE = os.path.dirname(os.path.dirname(__file__))
os.makedirs(BASE + "/data", exist_ok=True)
os.makedirs(BASE + "/plots", exist_ok=True)

# ================= HELPER FUNCTIONS =================
def straight(x1, x2, y):
    """Generate straight horizontal line"""
    dist = abs(x2 - x1)
    n = max(2, int(dist / STEP) + 1)
    xs = np.linspace(x1, x2, n)
    return [(x, y) for x in xs]


def diagonal(x1, y1, x2, y2):
    """Generate diagonal entry path"""
    dist = np.hypot(x2 - x1, y2 - y1)
    n = max(2, int(dist / STEP) + 1)
    xs = np.linspace(x1, x2, n)
    ys = np.linspace(y1, y2, n)
    return list(zip(xs, ys))


def u_turn(y_start, y_end, direction):
    """Generate perfect semicircle U-turn"""
    path = []
    yc = (y_start + y_end) / 2

    if direction == "right":
        xc = FIELD_L - R
        theta = np.linspace(-np.pi/2, np.pi/2, ARC_PTS)
    else:  # left
        xc = R
        theta = np.linspace(np.pi/2, 3*np.pi/2, ARC_PTS)

    for t in theta:
        x = xc + R * np.cos(t)
        y = yc + R * np.sin(t)
        path.append((x, y))

    return path


# ================= PATH GENERATION =================
path = []

def add_break():
    path.append((np.nan, np.nan))

# 1️⃣ Start point
path.append((0, 0))

# 2️⃣ Diagonal Entry
first_row_y = ROW / 2
diag_end = (R, first_row_y)

path += diagonal(0, 0, diag_end[0], diag_end[1])
add_break()

# 3️⃣ First Baseline (Left → Right)
path += straight(diag_end[0], FIELD_L - R, first_row_y)
add_break()

# 4️⃣ Generate Remaining Rows
rows = np.arange(first_row_y + ROW, FIELD_W - R + 0.1, ROW)

prev_y = first_row_y
left_to_right = False  # first row already L→R

for y in rows:

    if not left_to_right:
        # Turn Right
        path += u_turn(prev_y, y, "right")
        add_break()

        # Go Right → Left
        path += straight(FIELD_L - R, R, y)
        left_to_right = True

    else:
        # Turn Left
        path += u_turn(prev_y, y, "left")
        add_break()

        # Go Left → Right
        path += straight(R, FIELD_L - R, y)
        left_to_right = False

    add_break()
    prev_y = y

# ================= SAVE CSV =================
csv_path = BASE + "/data/waypoints.csv"
with open(csv_path, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["x", "y"])
    writer.writerows(path)

# ================= PLOT =================
x, y = zip(*path)

plt.figure(figsize=(12, 6))
plt.plot(x, y, linewidth=2, label="Tractor Path")

# Field boundary
plt.plot([0, FIELD_L, FIELD_L, 0, 0],
         [0, 0, FIELD_W, FIELD_W, 0],
         'k--', linewidth=1.5, label="Field Boundary")

plt.title("Tractor Path Planning (Diagonal Entry + Perfect U-Turns)")
plt.xlabel("X (m)")
plt.ylabel("Y (m)")
plt.axis("equal")
plt.grid(True, linestyle=":")
plt.legend()
plt.tight_layout()

plt.savefig(BASE + "/plots/path_plot.png", dpi=300)
plt.show()
