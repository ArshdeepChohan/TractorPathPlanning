import csv
import math

def load_waypoints(csv_file):
    waypoints = []

    with open(csv_file, 'r') as f:
        reader = csv.reader(f)
        next(reader)
        next(reader)

        for row in reader:
            x = float(row[0])
            y = float(row[1])
            t = int(row[2])   # type: 0=row, 1=turn, 2=entry
            waypoints.append((x, y, t))

    return waypoints


def compute_row_transition_distances(waypoints):
    """
    Computes distance between consecutive rows using turn segments
    """

    transitions = []
    current_turn = []

    for wp in waypoints:

        if wp[2] == 1:  # turn segment
            current_turn.append(wp)

        else:
            if len(current_turn) > 1:
                dist = 0.0

                for i in range(1, len(current_turn)):
                    x1, y1, _ = current_turn[i-1]
                    x2, y2, _ = current_turn[i]

                    dist += math.hypot(x2 - x1, y2 - y1)

                transitions.append(dist)
                current_turn = []

    return transitions


if __name__ == "__main__":

    file_path = "data\waypoints.csv"

    waypoints = load_waypoints(file_path)
    transitions = compute_row_transition_distances(waypoints)

    print("\nRow-to-Row Transition Distances:")

    for i, d in enumerate(transitions):
        print(f"Row {i+1} → Row {i+2}: {d:.2f} meters")

    print(f"\nAverage Transition Distance: {sum(transitions)/len(transitions):.2f} meters")