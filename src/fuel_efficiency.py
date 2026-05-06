import csv
import math

# ==============================
# PARAMETERS (adjust if needed)
# ==============================

ROW_FUEL_RATE = 0.0006   # liters per meter
TURN_FUEL_RATE = 0.0008  # liters per meter


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


def compute_fuel(waypoints):

    SLIP_FACTOR = 1.25   # 👈 ADD THIS

    total_fuel = 0.0
    row_fuel = 0.0
    turn_fuel = 0.0

    for i in range(1, len(waypoints)):

        x1, y1, t1 = waypoints[i-1]
        x2, y2, t2 = waypoints[i]

        dist = math.hypot(x2 - x1, y2 - y1)

        # ==================================================
        # APPLY SLIP ONLY FOR TURNING SEGMENTS
        # ==================================================
        if t2 == 1:
            dist *= SLIP_FACTOR

        fuel = 0.0  # always initialize

        if t2 == 0:  # row
            fuel = dist * ROW_FUEL_RATE
            row_fuel += fuel

        elif t2 == 1:  # turn
            fuel = dist * TURN_FUEL_RATE
            turn_fuel += fuel

        elif t2 == 2:  # entry/diagonal
            fuel = dist * ROW_FUEL_RATE

        total_fuel += fuel

    return total_fuel, row_fuel, turn_fuel


if __name__ == "__main__":

    file_path = "data\waypoints.csv"

    waypoints = load_waypoints(file_path)

    total, row, turn = compute_fuel(waypoints)

    print("\n===== FUEL ANALYSIS =====")
    print(f"Total Fuel Used: {total:.3f} L")
    print(f"Row Fuel: {row:.3f} L")
    print(f"Turn Fuel: {turn:.3f} L")