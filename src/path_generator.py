import numpy as np
import csv
import os
import sys

STEP = 0.5
ARC_PTS = 60


# ==================================================
# HELPER FUNCTIONS
# ==================================================

def straight_x(x1, x2, y):
    dist = abs(x2 - x1)
    n = int(dist / STEP) + 1
    xs = np.linspace(x1, x2, n)
    return [(x, y) for x in xs]


def straight_y(y1, y2, x):
    dist = abs(y2 - y1)
    n = int(dist / STEP) + 1
    ys = np.linspace(y1, y2, n)
    return [(x, y) for y in ys]


def diagonal(x1, y1, x2, y2):
    dist = np.hypot(x2 - x1, y2 - y1)
    n = max(2, int(dist / STEP) + 1)
    xs = np.linspace(x1, x2, n)
    ys = np.linspace(y1, y2, n)
    return list(zip(xs, ys))


# ==================================================
# MAIN FUNCTION
# ==================================================

def generate_path(FIELD_L, FIELD_W, ROW, direction):

    R = ROW / 2
    path = []

    def add_break():
        path.append((np.nan, np.nan))

    path.append((0, 0))

    # ------------------------------------------------
    # LENGTHWISE MODE
    # ------------------------------------------------
    if direction == "lengthwise":

        first_row_y = ROW / 2

        path += diagonal(0, 0, R, first_row_y)
        add_break()

        path += straight_x(R, FIELD_L - R, first_row_y)
        add_break()

        rows = np.arange(first_row_y + ROW, FIELD_W, ROW)
        prev_y = first_row_y

        for i, y in enumerate(rows):

            yc = (prev_y + y) / 2

            if i % 2 == 0:
                # Right U-turn
                xc = FIELD_L - R
                theta = np.linspace(-np.pi/2, np.pi/2, ARC_PTS)
            else:
                # Left U-turn
                xc = R
                theta = np.linspace(3*np.pi/2, np.pi/2, ARC_PTS)

            for t in theta:
                x = xc + R * np.cos(t)
                y_arc = yc + R * np.sin(t)
                path.append((x, y_arc))

            add_break()

            if i % 2 == 0:
                path += straight_x(FIELD_L - R, R, y)
            else:
                path += straight_x(R, FIELD_L - R, y)

            add_break()
            prev_y = y

    # ------------------------------------------------
    # WIDTHWISE MODE
    # ------------------------------------------------
    else:

        first_row_x = ROW / 2

        path += diagonal(0, 0, first_row_x, R)
        add_break()

        path += straight_y(R, FIELD_W - R, first_row_x)
        add_break()

        cols = np.arange(first_row_x + ROW, FIELD_L, ROW)
        prev_x = first_row_x

        for i, x in enumerate(cols):

            xc = (prev_x + x) / 2

            if i % 2 == 0:
                yc = FIELD_W - R
                theta = np.linspace(0, np.pi, ARC_PTS)
            else:
                yc = R
                theta = np.linspace(np.pi, 2*np.pi, ARC_PTS)

            for t in theta:
                x_arc = xc + R * np.cos(t)
                y = yc + R * np.sin(t)
                path.append((x_arc, y))

            add_break()

            if i % 2 == 0:
                path += straight_y(FIELD_W - R, R, x)
            else:
                path += straight_y(R, FIELD_W - R, x)

            add_break()
            prev_x = x

    return path


# ==================================================
# COMMAND LINE SUPPORT
# ==================================================
if __name__ == "__main__":

    if len(sys.argv) != 5:
        print("Usage: python path_generator.py L W ROW direction")
        sys.exit(1)

    FIELD_L = float(sys.argv[1])
    FIELD_W = float(sys.argv[2])
    ROW = float(sys.argv[3])
    direction = sys.argv[4]

    path = generate_path(FIELD_L, FIELD_W, ROW, direction)

    BASE_DIR = os.path.dirname(os.path.abspath(__file__))
    DATA_DIR = os.path.join(BASE_DIR, "data")
    os.makedirs(DATA_DIR, exist_ok=True)

    csv_path = os.path.join(DATA_DIR, "waypoints.csv")

    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["x", "y"])
        writer.writerows(path)

    print("Path generated successfully.")
