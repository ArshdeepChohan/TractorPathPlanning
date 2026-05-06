import sys
import os
import csv
import numpy as np
import matplotlib.pyplot as plt
import textwrap  # <-- ONLY added for formatting

# ==================================================
# DYNAMIC FIELD INPUT
# ==================================================

L = 100.0
W = 35.0
row_spacing = 5.0
base_len = 5.0
diag = 5.0
direction = "lengthwise"

if len(sys.argv) >= 7:
    L = float(sys.argv[1])
    W = float(sys.argv[2])
    row_spacing = float(sys.argv[3])
    direction = sys.argv[4]   # FIXED
    base_len = float(sys.argv[5])  # FIXED
    diag = float(sys.argv[6])      # FIXED

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

       # ==================================================
       # PHYSICS-BASED U-TURN (50° steering)
       # ==================================================

        WHEELBASE = 1.6  # meters
        STEERING_ANGLE = 50  # degrees

        R = WHEELBASE / np.tan(np.radians(STEERING_ANGLE))

        exit_x = end
        entry_direction = 1 if end > start else -1
        vertical_dir = np.sign(pos_next - pos_curr)

        # --- First arc ---
        theta1 = np.linspace(0, np.pi/2, 30)

        center1_x = exit_x
        center1_y = pos_curr + vertical_dir * R

        arc1_x = center1_x + entry_direction * R * np.sin(theta1)
        arc1_y = center1_y - vertical_dir * R * np.cos(theta1)

        # --- Straight connector ---
        mid_x = arc1_x[-1]
        target_y = pos_next - vertical_dir * R

        straight_y = np.linspace(arc1_y[-1], target_y, 20)
        straight_x = np.full_like(straight_y, mid_x)

        # --- Second arc ---
        theta2 = np.linspace(np.pi/2, 0, 30)

        center2_x = exit_x
        center2_y = pos_next - vertical_dir * R

        arc2_x = center2_x + entry_direction * R * np.sin(theta2)
        arc2_y = center2_y + vertical_dir * R * np.cos(theta2)

        # --- Combine ---
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

# ==================================================
# TITLE
# ==================================================

plt.title("Tractor Path Planning (50° Steering | Radius-Based)", pad=25)

# ==================================================
# ROW SEQUENCE (BELOW TITLE)
# ==================================================

sequence_text = "Row Sequence: " + " → ".join(str(i+1) for i in visit_order)
wrapped_text = "\n".join(textwrap.wrap(sequence_text, width=70))

plt.text(
    0.5, 1.02,                # position BELOW title
    wrapped_text,
    transform=plt.gca().transAxes,
    ha='center',
    va='bottom',
    fontsize=10,
    fontweight='bold',
    color='darkblue'
)

# Adjust layout for spacing
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