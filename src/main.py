"""
Main Execution Loop for Autonomous Tractor Simulation.
"""

import time
import math
import sys
import os
import csv

# Ensure src is in path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src import config
from src.gps_simulator import GPSSimulator
from src.gps_reader import GPSReader
from src.path_follower import PurePursuitController


def load_path_from_csv(file_path):
    path = []
    with open(file_path, "r") as f:
        reader = csv.reader(f)
        next(reader)
        for row in reader:
            path.append((float(row[0]), float(row[1])))
    return path


def main():
    print("=== AUTONOMOUS TRACTOR GUIDANCE SYSTEM ===")

    USE_SIMULATION = True
    os.makedirs("data", exist_ok=True)

    # GPS
    if USE_SIMULATION:
        gps = GPSSimulator(
            start_lat=config.REF_LAT,
            start_lon=config.REF_LON,
            start_heading=config.INITIAL_HEADING
        )
    else:
        gps = GPSReader()

    # Load Path
    path_file = os.path.join("data", "waypoints.csv")
    if not os.path.exists(path_file):
        print("[ERROR] Run path_generator.py first!")
        return

    path = load_path_from_csv(path_file)

    controller = PurePursuitController()

    print(f"[READY] {len(path)} waypoints loaded")

    start_time = time.time()

    try:
        while True:
            state = gps.get_state()
            steer_rad, xte, idx = controller.get_steering_control(state, path)
            gps.update(steer_rad, config.DT)

            if idx >= len(path) - 1:
                print("[SUCCESS] Path completed")
                break

            if time.time() - start_time > config.SIM_DURATION:
                print("[TIMEOUT]")
                break

            time.sleep(config.DT)

    except KeyboardInterrupt:
        print("[STOPPED]")


if __name__ == "__main__":
    main()
