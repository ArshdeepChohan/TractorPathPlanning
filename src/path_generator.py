import sys
import os
import csv
import json
import numpy as np
import matplotlib.pyplot as plt
import textwrap

from shape_handler import (
    ensure_ccw,
    polygon_bbox,
    row_segment_inside_polygon,
    smooth_turn_points,
)

# ==================================================
# DYNAMIC FIELD INPUT
# ==================================================

L = 100.0
W = 35.0
row_spacing = 5.0
base_len = 50.0
diag = 10.0
direction = "lengthwise"
shape = "rectangle"
turn_radius = 4.0
polygon_points = None
export_json = False


def parse_args():
    global L, W, row_spacing, base_len, diag, direction, shape, turn_radius, polygon_points, export_json

    if len(sys.argv) >= 7:
        L = float(sys.argv[1])
        W = float(sys.argv[2])
        row_spacing = float(sys.argv[3])
        direction = sys.argv[4]
        base_len = float(sys.argv[5])
        diag = float(sys.argv[6])

    idx = 7
    while idx < len(sys.argv):
        token = sys.argv[idx]
        if token == "--shape" and idx + 1 < len(sys.argv):
            shape = sys.argv[idx + 1].lower()
            idx += 2
        elif token == "--turn-radius" and idx + 1 < len(sys.argv):
            turn_radius = max(0.5, float(sys.argv[idx + 1]))
            idx += 2
        elif token == "--polygon-json" and idx + 1 < len(sys.argv):
            polygon_points = json.loads(sys.argv[idx + 1])
            idx += 2
        elif token == "--export-json":
            export_json = True
            idx += 1
        else:
            idx += 1


parse_args()

# ✅ Optional safety (does not affect logic)
if direction not in ["lengthwise", "widthwise"]:
    raise ValueError("Direction must be 'lengthwise' or 'widthwise'")

# ==================================================
# AUTOMATIC ROW CALCULATION
# ==================================================

num_rows = int(W / row_spacing)

ROW = row_spacing
CLI_MODE = False

POINT_SPACING = min(1.0, ROW / 5.0)

print("\nField Configuration")
print("-------------------")
print("Field Length:", L)
print("Field Width:", W)
print("Row Spacing:", row_spacing)
print("Baseline Length:", base_len)
print("Diagonal Entrance:", diag)
print("Direction:", direction)
print("Number of Rows:", num_rows)

# ==================================================
# PATH SETUP
# ==================================================

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATA_DIR = os.path.join(BASE_DIR, "src", "data")
os.makedirs(DATA_DIR, exist_ok=True)

csv_path = os.path.join(DATA_DIR, "waypoints.csv")
img_path = os.path.join(DATA_DIR, "field_path.png")

waypoints = []
xs, ys = [], []
row_segments = []
turn_segments = []
polygon_for_plot = None
metadata = {
    "shape": shape,
    "turn_radius": turn_radius,
}

# ==================================================
# SKIP ROW SEQUENCE
# ==================================================

def generate_skip_sequence(n, k=4):

    if n <= 1:
        return [0]

    visit = []
    visited_set = set()

    for start_node in range(n):

        curr = start_node

        while curr not in visited_set:
            visit.append(curr)
            visited_set.add(curr)
            curr = (curr + k) % n

        if len(visit) == n:
            break

    return visit


# ==================================================
# ROW POSITIONS
# ==================================================

def append_points(points, ptype):
    for px, py in points:
        xs.append(px)
        ys.append(py)
        waypoints.append({"x": px, "y": py, "type": ptype})


def generate_legacy_rectangle():
    if direction == "lengthwise":
        row_positions = [i * row_spacing for i in range(1, num_rows + 1)]
    else:
        row_positions = [i * row_spacing for i in range(1, num_rows + 1)]
    visit_order = generate_skip_sequence(len(row_positions))
    print("Row visiting order:", [i + 1 for i in visit_order])

    diag_x = np.linspace(0, diag, int(diag / POINT_SPACING) + 1)
    diag_y = np.linspace(0, diag, int(diag / POINT_SPACING) + 1)

    xs.extend(diag_x)
    ys.extend(diag_y)
    for x, y in zip(diag_x, diag_y):
        waypoints.append({"x": x, "y": y, "type": 2})

    for i in range(len(visit_order)):
        row_idx = visit_order[i]
        pos_curr = row_positions[row_idx]

        if i == 0:
            baseline_end = min(diag + base_len, L)
            if direction == "lengthwise":
                base_x = np.linspace(diag, baseline_end, int(abs(baseline_end - diag) / POINT_SPACING) + 1)
                base_y = np.full_like(base_x, pos_curr)
            else:
                base_y = np.linspace(diag, baseline_end, int(abs(baseline_end - diag) / POINT_SPACING) + 1)
                base_x = np.full_like(base_y, pos_curr)

            xs.extend(base_x)
            ys.extend(base_y)
            for x, y in zip(base_x, base_y):
                waypoints.append({"x": x, "y": y, "type": 0})
            row_segments.append((base_x, base_y))

            if baseline_end < L:
                seg_x = np.linspace(baseline_end, L, int(abs(L - baseline_end) / POINT_SPACING) + 1)
                seg_y = np.full_like(seg_x, pos_curr)
                xs.extend(seg_x)
                ys.extend(seg_y)
                for x, y in zip(seg_x, seg_y):
                    waypoints.append({"x": x, "y": y, "type": 0})
                row_segments.append((seg_x, seg_y))
            start = diag
            end = L
        else:
            if i % 2 == 0:
                start, end = 0, L
            else:
                start, end = L, 0

            if direction == "lengthwise":
                seg_x = np.linspace(start, end, int(abs(end - start) / POINT_SPACING) + 1)
                seg_y = np.full_like(seg_x, pos_curr)
            else:
                seg_y = np.linspace(start, end, int(abs(end - start) / POINT_SPACING) + 1)
                seg_x = np.full_like(seg_y, pos_curr)
            xs.extend(seg_x)
            ys.extend(seg_y)
            for x, y in zip(seg_x, seg_y):
                waypoints.append({"x": x, "y": y, "type": 0})
            row_segments.append((seg_x, seg_y))

        if i < len(visit_order) - 1:
            next_row_idx = visit_order[i + 1]
            pos_next = row_positions[next_row_idx]
            row_gap = abs(pos_next - pos_curr)
            b = min(row_gap * 0.25, 6.0)
            stagger = (i % 3) * 0.5
            a = (b * np.tan(np.radians(30))) + stagger
            if direction == "lengthwise":
                exit_x = end
                sign = 1 if end > start else -1
            else:
                exit_x = pos_curr
                sign = 1 if end > start else -1

            theta = np.linspace(0, np.pi / 2, 25)
            arc1_x = exit_x + sign * a * np.sin(theta)
            arc1_y = pos_curr + np.sign(pos_next - pos_curr) * b * (1 - np.cos(theta))

            y_target_start = arc1_y[-1]
            y_target_end = pos_next - np.sign(pos_next - pos_curr) * b
            straight_x = np.full(20, arc1_x[-1])
            straight_y = np.linspace(y_target_start, y_target_end, 20)

            arc2_x = exit_x + sign * a * np.sin(np.flip(theta))
            arc2_y = pos_next - np.sign(pos_next - pos_curr) * b * (1 - np.cos(np.flip(theta)))
            turn_x = np.concatenate([arc1_x, straight_x, arc2_x])
            turn_y = np.concatenate([arc1_y, straight_y, arc2_y])
            if direction == "widthwise":
                turn_x, turn_y = turn_y, turn_x

            if direction == "lengthwise":
                xs.extend(diag_x)
                ys.extend(diag_y)
            else:
                xs.extend(diag_y)
                ys.extend(diag_x)

            for tx, ty in zip(turn_x, turn_y):
                waypoints.append({"x": tx, "y": ty, "type": 1})
            turn_segments.append((turn_x, turn_y))

    return row_positions, visit_order


def generate_polygon_mode():
    global L, W, polygon_for_plot
    polygon = ensure_ccw([(float(p[0]), float(p[1])) for p in polygon_points])
    polygon_for_plot = polygon
    min_x, max_x, min_y, max_y = polygon_bbox(polygon)
    L = max_x - min_x
    W = max_y - min_y
    metadata["polygon"] = polygon

    if direction == "lengthwise":
        row_positions = [diag + i * row_spacing for i in range(0, int((max_y - diag) / row_spacing) + 1)]
    else:
        row_positions = [diag + i * row_spacing for i in range(0, int((max_x - diag) / row_spacing) + 1)]
    row_positions = [p for p in row_positions if (p <= max_y if direction == "lengthwise" else p <= max_x)]
    visit_order = generate_skip_sequence(len(row_positions))
    print("Row visiting order:", [i + 1 for i in visit_order])

    if not visit_order:
        return row_positions, visit_order

    first_axis = row_positions[visit_order[0]]
    first_seg = row_segment_inside_polygon(polygon, first_axis, direction)
    if first_seg is None:
        return row_positions, visit_order

    if direction == "lengthwise":
        start_first = (first_seg[0], first_axis)
    else:
        start_first = (first_axis, first_seg[0])

    entry_len = max(np.hypot(start_first[0], start_first[1]), 1e-6)
    ratio = min(1.0, diag / entry_len)
    entry_point = (start_first[0] * ratio, start_first[1] * ratio)
    diag_steps = max(2, int(diag / max(POINT_SPACING, 0.2)) + 1)
    diag_line = np.linspace(0.0, 1.0, diag_steps)
    append_points([(entry_point[0] * t, entry_point[1] * t) for t in diag_line], 2)

    current_end = None
    for i, row_idx in enumerate(visit_order):
        axis_val = row_positions[row_idx]
        seg = row_segment_inside_polygon(polygon, axis_val, direction)
        if seg is None:
            continue

        low, high = seg
        if i % 2 == 0:
            s_axis, e_axis = low, high
        else:
            s_axis, e_axis = high, low

        if direction == "lengthwise":
            row_start = (s_axis, axis_val)
            row_end = (e_axis, axis_val)
            dir_sign = 1.0 if e_axis >= s_axis else -1.0
        else:
            row_start = (axis_val, s_axis)
            row_end = (axis_val, e_axis)
            dir_sign = 1.0 if e_axis >= s_axis else -1.0

        if i == 0:
            row_total = np.hypot(row_end[0] - row_start[0], row_end[1] - row_start[1])
            use_base = min(base_len, row_total)
            ratio_b = 0.0 if row_total < 1e-6 else use_base / row_total
            base_end = (
                row_start[0] + (row_end[0] - row_start[0]) * ratio_b,
                row_start[1] + (row_end[1] - row_start[1]) * ratio_b,
            )
            bpts = max(2, int(use_base / max(POINT_SPACING, 0.2)) + 1)
            append_points(list(zip(np.linspace(row_start[0], base_end[0], bpts),
                                   np.linspace(row_start[1], base_end[1], bpts))), 0)
            row_segments.append(([row_start[0], base_end[0]], [row_start[1], base_end[1]]))
            if use_base < row_total:
                rpts = max(2, int((row_total - use_base) / max(POINT_SPACING, 0.2)) + 1)
                append_points(list(zip(np.linspace(base_end[0], row_end[0], rpts),
                                       np.linspace(base_end[1], row_end[1], rpts))), 0)
                row_segments.append(([base_end[0], row_end[0]], [base_end[1], row_end[1]]))
        else:
            row_total = np.hypot(row_end[0] - row_start[0], row_end[1] - row_start[1])
            pts = max(2, int(row_total / max(POINT_SPACING, 0.2)) + 1)
            append_points(list(zip(np.linspace(row_start[0], row_end[0], pts),
                                   np.linspace(row_start[1], row_end[1], pts))), 0)
            row_segments.append(([row_start[0], row_end[0]], [row_start[1], row_end[1]]))

        current_end = row_end
        if i < len(visit_order) - 1:
            next_axis = row_positions[visit_order[i + 1]]
            next_seg = row_segment_inside_polygon(polygon, next_axis, direction)
            if next_seg is None:
                continue
            if (i + 1) % 2 == 0:
                next_start = (next_seg[0], next_axis) if direction == "lengthwise" else (next_axis, next_seg[0])
            else:
                next_start = (next_seg[1], next_axis) if direction == "lengthwise" else (next_axis, next_seg[1])

            turn_pts = smooth_turn_points(
                current_end,
                next_start,
                dir_sign,
                polygon,
                POINT_SPACING,
                turn_radius,
            )
            append_points(turn_pts, 1)
            if turn_pts:
                tx, ty = [p[0] for p in turn_pts], [p[1] for p in turn_pts]
                turn_segments.append((tx, ty))
    return row_positions, visit_order


if polygon_points and shape != "rectangle":
    row_positions, visit_order = generate_polygon_mode()
else:
    row_positions, visit_order = generate_legacy_rectangle()

# ==================================================
# DIAGONAL ENTRANCE
# ==================================================

# ==================================================
# SAVE CSV
# ==================================================

with open(csv_path, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow([f"# {L}", W, ROW, direction, base_len, diag, json.dumps(metadata)])
    writer.writerow(["x", "y", "type"])

    for wp in waypoints:
        writer.writerow([round(wp["x"], 3), round(wp["y"], 3), wp["type"]])

print("Waypoints saved:",csv_path)

if export_json:
    json_path = os.path.join(DATA_DIR, "waypoints.json")
    payload = {
        "shape": shape,
        "direction": direction,
        "row_spacing": ROW,
        "turn_radius": turn_radius,
        "sequence": [{"order": i + 1, "x": round(wp["x"], 3), "y": round(wp["y"], 3), "type": wp["type"]} for i, wp in enumerate(waypoints)],
    }
    with open(json_path, "w", encoding="utf-8") as jf:
        json.dump(payload, jf, indent=2)
    print("JSON export saved:", json_path)

# ==================================================
# PLOT GRAPH
# ==================================================

plt.figure(figsize=(14,8))

if polygon_for_plot:
    poly_x = [p[0] for p in polygon_for_plot] + [polygon_for_plot[0][0]]
    poly_y = [p[1] for p in polygon_for_plot] + [polygon_for_plot[0][1]]
    plt.plot(poly_x, poly_y, "k--", linewidth=1.5, label="Field Boundary")
else:
    plt.plot([0, L, L, 0, 0], [0, 0, W, W, 0], "k--", linewidth=1.5, label="Field Boundary")

for idx,pos in enumerate(row_positions):
    if polygon_for_plot:
        seg = row_segment_inside_polygon(polygon_for_plot, pos, direction)
        if seg is None:
            continue
        if direction == "lengthwise":
            plt.plot([seg[0], seg[1]], [pos, pos], color="grey", linestyle="--", alpha=0.3)
            plt.text(seg[0] - 0.5, pos, f"Row {idx + 1}", ha="right", va="center", fontsize=9)
        else:
            plt.plot([pos, pos], [seg[0], seg[1]], color="grey", linestyle="--", alpha=0.3)
    else:
        plt.plot([0,L],[pos,pos], color="grey",linestyle="--",alpha=0.3)
        plt.text(-2,pos,f"Row {idx+1}", ha="right",va="center",fontsize=9)

diag_points = [wp for wp in waypoints if wp["type"] == 2]
if diag_points:
    diag_x = [p["x"] for p in diag_points]
    diag_y = [p["y"] for p in diag_points]
    plt.plot(diag_x, diag_y, color="#9467bd", linewidth=4, label="Diagonal Entrance")

for i,(rx,ry) in enumerate(row_segments):

    if i == 0:
        plt.plot(rx,ry,color="#2ca02c",linewidth=4,label="Baseline Path")
    else:
        label="Tractor Path (Rows)" if i==1 else ""
        plt.plot(rx,ry,color="#1f77b4",linewidth=3,label=label)

for i,(tx,ty) in enumerate(turn_segments):
    label="U-Turn Segments" if i==0 else ""
    plt.plot(tx,ty,color="#d62728",linewidth=3,label=label)

plt.scatter(xs[::5],ys[::5],
            color="black",s=15,
            label="GPS Waypoints")

plt.scatter(0,0,color="#2ca02c",
            marker="*",s=300,
            edgecolors="black",
            label="START")

plt.scatter(xs[-1],ys[-1],
            color="#ff7f0e",
            marker="X",s=200,
            edgecolors="black",
            label="END")

plt.title("Professional Tractor Path Planning: 30° Arc Headland Turns", pad=25)

sequence_text = "Row Sequence: " + " → ".join(str(i+1) for i in visit_order)
wrapped_text = "\n".join(textwrap.wrap(sequence_text, width=70))

plt.text(
    0.5, 1.02,
    wrapped_text,
    transform=plt.gca().transAxes,
    ha='center',
    va='bottom',
    fontsize=10,
    fontweight='bold',
    color='darkblue'
)

plt.subplots_adjust(top=0.82)

plt.xlabel("Length (meters)")
plt.ylabel("Width (meters)")

plt.axis("equal")
plt.grid(True,linestyle=":",alpha=0.5)

plt.legend(loc="upper center",
           bbox_to_anchor=(0.5,-0.1),
           ncol=3,frameon=True)

plt.tight_layout()
plt.savefig(img_path,dpi=300)

print("Image saved:",img_path)

if not CLI_MODE:
    plt.show()