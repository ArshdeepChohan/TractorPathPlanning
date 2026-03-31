import sys
import os
import csv
import numpy as np
import matplotlib.pyplot as plt

# ==================================================
# INPUT
# ==================================================
L = 100.0
W = 35.0
row_spacing = 5.0
base_len = 50.0
diag = 10.0
direction = "lengthwise"

field_type = "rectangle"
top_width = W
offset = 10.0

if len(sys.argv) >= 7:
    L = float(sys.argv[1])
    W = float(sys.argv[2])
    row_spacing = float(sys.argv[3])
    base_len = float(sys.argv[4])
    diag = float(sys.argv[5])
    direction = sys.argv[6]

if len(sys.argv) >= 8:
    field_type = sys.argv[7]

if len(sys.argv) >= 9:
    top_width = float(sys.argv[8])

if len(sys.argv) >= 10:
    offset = float(sys.argv[9])

# ==================================================
# BOUNDARY FUNCTION
# ==================================================
def get_x_bounds(y):
    if field_type == "rectangle":
        return 0, L

    elif field_type == "trapezium":
        shift = (L - top_width) / 2
        left = (shift / W) * y
        right = L - (shift / W) * y
        return left, right

    elif field_type == "rhombus":
        left = (offset / W) * y
        right = L + (offset / W) * y
        return left, right

    return 0, L

# ==================================================
# EDGE ANGLE (NEW)
# ==================================================
def get_edge_angle(y, side):
    if field_type == "rectangle":
        return 0.0

    elif field_type == "trapezium":
        shift = (L - top_width) / 2
        slope = shift / W
        return np.arctan(slope) if side == "left" else -np.arctan(slope)

    elif field_type == "rhombus":
        slope = offset / W
        return np.arctan(slope)

    return 0.0

# ==================================================
# ROTATION FUNCTION (NEW)
# ==================================================
def rotate_points(x, y, angle, ox, oy):
    x_shift = x - ox
    y_shift = y - oy

    xr = x_shift * np.cos(angle) - y_shift * np.sin(angle)
    yr = x_shift * np.sin(angle) + y_shift * np.cos(angle)

    return xr + ox, yr + oy

# ==================================================
# ROW SETUP
# ==================================================
num_rows = int(W / row_spacing)
POINT_SPACING = min(1.0, row_spacing / 5.0)

waypoints = []
xs, ys = [], []
row_segments = []
turn_segments = []

# ==================================================
# SKIP SEQUENCE
# ==================================================
def generate_skip_sequence(n, k=4):
    visit = []
    visited = set()
    for start in range(n):
        curr = start
        while curr not in visited:
            visit.append(curr)
            visited.add(curr)
            curr = (curr + k) % n
        if len(visit) == n:
            break
    return visit

row_positions = [i * row_spacing for i in range(1, num_rows + 1)]
visit_order = generate_skip_sequence(len(row_positions))

# ==================================================
# DIAGONAL
# ==================================================
diag_x = np.linspace(0, diag, int(diag / POINT_SPACING) + 1)
diag_y = np.linspace(0, diag, int(diag / POINT_SPACING) + 1)

xs.extend(diag_x)
ys.extend(diag_y)

# ==================================================
# MAIN LOOP
# ==================================================
for i in range(len(visit_order)):

    row_idx = visit_order[i]
    y = row_positions[row_idx]

    left, right = get_x_bounds(y)

    # FIRST ROW
    if i == 0:
        baseline_end = min(diag + base_len, right)

        base_x = np.linspace(diag, baseline_end, int(abs(baseline_end - diag) / POINT_SPACING) + 1)
        base_y = np.full_like(base_x, y)

        xs.extend(base_x)
        ys.extend(base_y)
        row_segments.append((base_x, base_y))

        start = diag
        end = right

    else:
        if i % 2 == 0:
            start, end = left, right
        else:
            start, end = right, left

        seg_x = np.linspace(start, end, int(abs(end-start)/POINT_SPACING)+1)
        seg_y = np.full_like(seg_x, y)

        xs.extend(seg_x)
        ys.extend(seg_y)
        row_segments.append((seg_x, seg_y))

    # ==================================================
    # U-TURN (UNCHANGED + ROTATION ONLY)
    # ==================================================
    if i < len(visit_order) - 1:

        next_y = row_positions[visit_order[i+1]]

        row_gap = abs(next_y - y)
        b = min(row_gap * 0.25, 6.0)
        stagger = (i % 3) * 0.5
        a = (b * np.tan(np.radians(30))) + stagger

        exit_x = end
        sign = 1 if end > start else -1

        theta = np.linspace(0, np.pi/2, 25)

        arc1_x = exit_x + sign * a * np.sin(theta)
        arc1_y = y + np.sign(next_y - y) * b * (1 - np.cos(theta))

        y_target_start = arc1_y[-1]
        y_target_end = next_y - np.sign(next_y - y) * b

        straight_x = np.full(20, arc1_x[-1])
        straight_y = np.linspace(y_target_start, y_target_end, 20)

        arc2_x = exit_x + sign * a * np.sin(np.flip(theta))
        arc2_y = next_y - np.sign(next_y - y) * b * (1 - np.cos(np.flip(theta)))

        turn_x = np.concatenate([arc1_x, straight_x, arc2_x])
        turn_y = np.concatenate([arc1_y, straight_y, arc2_y])

        # 🔥 ONLY ADDITION: ROTATE U-TURN
        side = "right" if end > start else "left"
        angle = get_edge_angle(y, side)

        turn_x, turn_y = rotate_points(turn_x, turn_y, angle, exit_x, y)

        xs.extend(turn_x)
        ys.extend(turn_y)
        turn_segments.append((turn_x, turn_y))

# ==================================================
# PLOT
# ==================================================
plt.figure(figsize=(14,8))

# boundary
poly = []
for y in [0, W]:
    l, r = get_x_bounds(y)
    poly.append((l,y))
    poly.append((r,y))
poly = [poly[0], poly[1], poly[3], poly[2], poly[0]]

px = [p[0] for p in poly]
py = [p[1] for p in poly]
plt.plot(px, py, "k--", linewidth=1.5)

# draw
for rx,ry in row_segments:
    plt.plot(rx,ry,color="blue",linewidth=3)

for tx,ty in turn_segments:
    plt.plot(tx,ty,color="red",linewidth=3)

plt.plot(diag_x, diag_y, color="purple",linewidth=4)

plt.scatter(xs[::5],ys[::5], color="black",s=10)

plt.axis("equal")
plt.grid(True)
plt.title("Final Tractor Path (Edge-Aligned U-Turns)")
plt.show()