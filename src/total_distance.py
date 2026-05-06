import csv
import math

# ==================================================
# LOAD WAYPOINTS
# ==================================================

def load_waypoints(csv_file):
    waypoints = []

    with open(csv_file, 'r') as f:
        reader = csv.reader(f)
        next(reader)
        next(reader)

        for row in reader:
            x = float(row[0])
            y = float(row[1])
            t = int(row[2])  # 0=row, 1=turn, 2=entry
            waypoints.append((x, y, t))

    return waypoints


# ==================================================
# DETECT ROW CENTERS → AUTOMATIC ROW INDEXING
# ==================================================

def detect_rows(waypoints, tolerance=0.5):
    """
    Detect unique row y-levels and assign row numbers
    """

    row_levels = []

    for x, y, t in waypoints:
        if t == 0:  # only row points
            found = False
            for r in row_levels:
                if abs(y - r) < tolerance:
                    found = True
                    break
            if not found:
                row_levels.append(y)

    row_levels = sorted(row_levels)

    return row_levels


def get_row_index(y, row_levels, tolerance=0.5):
    for i, r in enumerate(row_levels):
        if abs(y - r) < tolerance:
            return i + 1
    return None


# ==================================================
# MAIN DISTANCE + TURN EXTRACTION
# ==================================================

# ==================================================
# MAIN DISTANCE + TURN EXTRACTION (WITH SLIP)
# ==================================================

def compute_exact_turns(waypoints):

    SLIP_FACTOR = 1.25   # 👈 ADD THIS (top of function)

    total_distance = 0.0
    turning_distance_total = 0.0

    transitions = []
    row_sequence = []

    row_levels = detect_rows(waypoints)

    in_turn = False
    current_turn = []
    current_row = None

    for i in range(1, len(waypoints)):

        x1, y1, t1 = waypoints[i-1]
        x2, y2, t2 = waypoints[i]

        dist = math.hypot(x2 - x1, y2 - y1)

        # ==================================================
        # APPLY SLIP ONLY FOR TURNING SEGMENTS
        # ==================================================
        if t2 == 1:
            dist = dist * SLIP_FACTOR

        total_distance += dist

        # =========================
        # DETECT ROW INDEX
        # =========================
        if t2 == 0:
            row_idx = get_row_index(y2, row_levels)

            if current_row != row_idx:
                row_sequence.append(row_idx)
                current_row = row_idx

        # =========================
        # START TURN
        # =========================
        if t1 == 0 and t2 == 1:
            in_turn = True
            current_turn = [(x1, y1)]

        # =========================
        # CONTINUE TURN
        # =========================
        if in_turn:
            current_turn.append((x2, y2))
            turning_distance_total += dist   # 👈 already slip-adjusted

        # =========================
        # END TURN
        # =========================
        if t1 == 1 and t2 == 0 and in_turn:

            turn_dist = 0.0

            for j in range(1, len(current_turn)):
                x_prev, y_prev = current_turn[j-1]
                x_curr, y_curr = current_turn[j]

                seg = math.hypot(x_curr - x_prev, y_curr - y_prev)

                # APPLY SLIP AGAIN FOR SEGMENT (CONSISTENCY)
                seg *= SLIP_FACTOR

                turn_dist += seg

            transitions.append(turn_dist)

            current_turn = []
            in_turn = False

    return total_distance, turning_distance_total, transitions, row_sequence


# ==================================================
# MAIN
# ==================================================

if __name__ == "__main__":

    file_path = "data\waypoints.csv"

    waypoints = load_waypoints(file_path)

    total, turn_total, transitions, row_seq = compute_exact_turns(waypoints)

    print("\n===== EXACT DISTANCE ANALYSIS =====")

    print(f"\nTotal Distance: {total:.2f} meters")
    print(f"Total Turning Distance: {turn_total:.2f} meters")

    print("\nDetected Row Sequence:")
    print(" → ".join(str(r) for r in row_seq))

    print("\nTurning Distance per Row Transition:")

    for i in range(len(transitions)):
        print(f"Row {row_seq[i]} → Row {row_seq[i+1]}: {transitions[i]:.2f} meters")

    if len(transitions) > 0:
        avg_turn = sum(transitions) / len(transitions)
        print(f"\nAverage Turning Distance: {avg_turn:.2f} meters")