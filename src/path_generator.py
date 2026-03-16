import sys
import os
import csv
import numpy as np
import matplotlib.pyplot as plt

# ==================================================
# DYNAMIC FIELD INPUT
# ==================================================

L = 100.0
W = 35.0
row_spacing = 5.0
base_len = 50.0
diag = 10.0
direction = "lengthwise"

if len(sys.argv) >= 7:
    L = float(sys.argv[1])
    W = float(sys.argv[2])
    row_spacing = float(sys.argv[3])
    base_len = float(sys.argv[4])
    diag = float(sys.argv[5])
    direction = sys.argv[6]

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

row_positions = [i * row_spacing for i in range(1, num_rows + 1)]
visit_order = generate_skip_sequence(len(row_positions))

print("Row visiting order:", [i+1 for i in visit_order])

# ==================================================
# DIAGONAL ENTRANCE
# ==================================================

diag_x = np.linspace(0, diag, int(diag / POINT_SPACING) + 1)
diag_y = np.linspace(0, diag, int(diag / POINT_SPACING) + 1)

xs.extend(diag_x)
ys.extend(diag_y)

for x,y in zip(diag_x,diag_y):
    waypoints.append({'x':x,'y':y,'type':2})

# ==================================================
# PATH GENERATION
# ==================================================

for i in range(len(visit_order)):

    row_idx = visit_order[i]
    pos_curr = row_positions[row_idx]

    if i == 0:

        baseline_end = min(diag + base_len, L)

        base_x = np.linspace(diag, baseline_end,
                             int(abs(baseline_end - diag) / POINT_SPACING) + 1)
        base_y = np.full_like(base_x, pos_curr)

        xs.extend(base_x)
        ys.extend(base_y)

        for x,y in zip(base_x,base_y):
            waypoints.append({'x':x,'y':y,'type':0})

        row_segments.append((base_x,base_y))

        if baseline_end < L:

            seg_x = np.linspace(baseline_end, L,
                                int(abs(L - baseline_end) / POINT_SPACING) + 1)
            seg_y = np.full_like(seg_x, pos_curr)

            xs.extend(seg_x)
            ys.extend(seg_y)

            for x,y in zip(seg_x,seg_y):
                waypoints.append({'x':x,'y':y,'type':0})

            row_segments.append((seg_x,seg_y))

        start = diag
        end = L

    else:

        if i % 2 == 0:
            start,end = 0,L
        else:
            start,end = L,0

        seg_x = np.linspace(start,end,
                            int(abs(end-start)/POINT_SPACING)+1)
        seg_y = np.full_like(seg_x,pos_curr)

        xs.extend(seg_x)
        ys.extend(seg_y)

        for x,y in zip(seg_x,seg_y):
            waypoints.append({'x':x,'y':y,'type':0})

        row_segments.append((seg_x,seg_y))

    if i < len(visit_order) - 1:

        next_row_idx = visit_order[i + 1]
        pos_next = row_positions[next_row_idx]

        row_gap = abs(pos_next - pos_curr)

        b = min(row_gap * 0.25, 6.0)

        stagger = (i % 3) * 0.5
        a = (b * np.tan(np.radians(30))) + stagger

        exit_x = end
        sign = 1 if end > start else -1

        theta = np.linspace(0, np.pi/2, 25)

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

        xs.extend(turn_x)
        ys.extend(turn_y)

        for tx, ty in zip(turn_x, turn_y):
            waypoints.append({'x': tx, 'y': ty, 'type': 1})

        turn_segments.append((turn_x, turn_y))

# ==================================================
# SAVE CSV
# ==================================================

with open(csv_path,"w",newline="") as f:

    writer = csv.writer(f)
    writer.writerow([f"# {L}",W,ROW,direction,base_len,diag])
    writer.writerow(["x","y","type"])

    for wp in waypoints:
        writer.writerow([round(wp['x'],3),
                         round(wp['y'],3),
                         wp['type']])

print("Waypoints saved:",csv_path)

# ==================================================
# PLOT GRAPH
# ==================================================

plt.figure(figsize=(14,8))

plt.plot([0,L,L,0,0],[0,0,W,W,0],
         "k--",linewidth=1.5,label="Field Boundary")

for idx,pos in enumerate(row_positions):
    plt.plot([0,L],[pos,pos],
             color="grey",linestyle="--",alpha=0.3)
    plt.text(-2,pos,f"Row {idx+1}",
             ha="right",va="center",fontsize=9)

# Diagonal Entrance Colored
plt.plot(diag_x, diag_y,
         color="#9467bd",linewidth=4,label="Diagonal Entrance")

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

plt.title("Professional Tractor Path Planning: 30° Arc Headland Turns")
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