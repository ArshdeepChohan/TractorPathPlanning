"""
Main Execution Loop for Autonomous Tractor Simulation.
Integrates GPS, Path Generation, and Control Logic.
"""
import time
import math
import sys
import os
import csv

# Ensure src is in path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src import config
from src import geo_utils
from src.gps_simulator import GPSSimulator
from src.gps_reader import GPSReader
from src.path_generator import PathGenerator
from src.path_follower import PurePursuitController

def main():
    print("==================================================")
    print("   AUTONOMOUS TRACTOR GUIDANCE SYSTEM (SIMULATION)")
    print("==================================================")
    
    # 1. Initialize Components
    # Toggle this to switch between Simulation and Real Hardware
    USE_SIMULATION = True
    
    if USE_SIMULATION:
        print("[INIT] Starting GPS Simulator...")
        # Start at (0, 0) facing East (0 deg)
        gps = GPSSimulator(start_lat=config.REF_LAT, start_lon=config.REF_LON, start_heading=0)
    else:
        print("[INIT] Connecting to Real GPS...")
        gps = GPSReader()

    # 2. Generate Path
    print("[INIT] Generating Field Coverage Path...")
    pg = PathGenerator()
    path_local = pg.generate_field_coverage()
    os.makedirs("data", exist_ok=True)
    pg.save_to_csv("data/waypoints_gps.csv")
    pg.save_local_csv("data/waypoints_local.csv")
    
    if not path_local:
        print("[ERROR] No path generated!")
        return

    # 3. Initialize Controller
    controller = PurePursuitController()
    
    print(f"[READY] Path loaded with {len(path_local)} waypoints.")
    print("[READY] Starting Control Loop... Press Ctrl+C to stop.")
    time.sleep(1)
    
    # 4. Control Loop
    try:
        start_time = time.time()
        traj = []
        tel_f = open("data/telemetry.csv", "w", newline="")
        tel_w = csv.writer(tel_f)
        tel_w.writerow(["time", "x", "y", "heading_deg", "steer_deg", "xte", "target_idx"])
        
        while True:
            loop_start = time.time()
            
            # --- READ SENSORS ---
            state = gps.get_state()
            
            # --- CONTROL LOGIC ---
            steer_rad, xte, target_idx = controller.get_steering_control(state, path_local)
            steer_deg = math.degrees(steer_rad)
            
            # --- ACTUATION (SIMULATION) ---
            # In simulation, we feed the control back to the plant
            gps.update(steer_rad, config.DT)
            
            # --- LOGGING / DISPLAY ---
            elapsed = time.time() - start_time
            
            # Clear screen (optional, can be flickering)
            # print("\033[H\033[J", end="") 
            
            print(f"Time: {elapsed:5.1f}s | "
                  f"Pos: ({state['x']:6.1f}, {state['y']:6.1f})m | "
                  f"Head: {state['heading']:5.1f}° | "
                  f"XTE: {xte:5.2f}m | "
                  f"Steer: {steer_deg:5.1f}° | "
                  f"Idx: {target_idx}/{len(path_local)}")
            tel_w.writerow([f"{elapsed:.2f}", f"{state['x']:.3f}", f"{state['y']:.3f}", f"{state['heading']:.2f}", f"{steer_deg:.2f}", f"{xte:.3f}", target_idx])
            traj.append((state['x'], state['y']))
            
            # Check for end of path
            dist_to_end = math.hypot(state['x'] - path_local[-1][0], state['y'] - path_local[-1][1])
            if dist_to_end < 1.0 and target_idx >= len(path_local) - 5:
                print("\n[SUCCESS] Destination Reached!")
                break
            
            # Check for timeout
            if elapsed > config.SIM_DURATION:
                print("\n[TIMEOUT] Simulation ended.")
                break
                
            # Maintain loop rate
            process_time = time.time() - loop_start
            sleep_time = config.DT - process_time
            if sleep_time > 0:
                time.sleep(sleep_time)
                
    except KeyboardInterrupt:
        print("\n[STOP] Simulation stopped by user.")
    finally:
        try:
            tel_f.close()
        except Exception:
            pass
        try:
            import matplotlib.pyplot as plt
            plt.figure(figsize=(10, 6))
            px = [p[0] for p in path_local]
            py = [p[1] for p in path_local]
            pins_local = [geo_utils.gps_to_local(lat, lon) for (lat, lon) in getattr(config, "FIELD_PINS", [])] if hasattr(config, "FIELD_PINS") else []
            all_x = px + [p[0] for p in traj] + [p[0] for p in pins_local]
            all_y = py + [p[1] for p in traj] + [p[1] for p in pins_local]
            min_x = min(all_x) if all_x else 0.0
            min_y = min(all_y) if all_y else 0.0
            off_x = -min_x if min_x < 0 else 0.0
            off_y = -min_y if min_y < 0 else 0.0
            print(f"Transform: ENU from Pin 1 as origin, shift by ({off_x:.2f}, {off_y:.2f}) to remove negatives.")
            px = [x + off_x for x in px]
            py = [y + off_y for y in py]
            px = [max(0.0, min(x, config.FIELD_LENGTH)) for x in px]
            py = [max(0.0, min(y, config.FIELD_WIDTH)) for y in py]
            if pins_local:
                pins_local = [(x + off_x, y + off_y) for (x, y) in pins_local]
            plt.plot([0, config.FIELD_LENGTH, config.FIELD_LENGTH, 0, 0],
                     [0, 0, config.FIELD_WIDTH, config.FIELD_WIDTH, 0],
                     color="black", linewidth=1.0)
            plt.plot(px, py, color="#1f77b4", linewidth=2.0)
            if px and py:
                plt.scatter([px[0]], [py[0]], color="green", zorder=3)
                plt.scatter([px[-1]], [py[-1]], color="red", zorder=3)
            plt.xlim(0, config.FIELD_LENGTH)
            plt.ylim(0, config.FIELD_WIDTH)
            plt.title("Autonomous Tractor Lawn Mower Path – REAL TRACTOR U-TURNS")
            plt.xlabel("X (m)")
            plt.ylabel("Y (m)")
            plt.grid(True, linestyle="--", linewidth=0.5, alpha=0.5)
            plt.tight_layout()
            plt.savefig("data/plot.png", dpi=150)
            print("Visualization saved to data/plot.png")
        except Exception:
            width = int(config.FIELD_LENGTH)
            height = int(config.FIELD_WIDTH)
            scale = 5
            svg_w = width * scale
            svg_h = height * scale
            def to_px(p):
                return f"{int(p[0]*scale)},{int(svg_h - p[1]*scale)}"
            pins_local = [geo_utils.gps_to_local(lat, lon) for (lat, lon) in getattr(config, "FIELD_PINS", [])] if hasattr(config, "FIELD_PINS") else []
            all_x = [p[0] for p in path_local] + [p[0] for p in traj] + [p[0] for p in pins_local]
            all_y = [p[1] for p in path_local] + [p[1] for p in traj] + [p[1] for p in pins_local]
            min_x = min(all_x) if all_x else 0.0
            min_y = min(all_y) if all_y else 0.0
            off_x = -min_x if min_x < 0 else 0.0
            off_y = -min_y if min_y < 0 else 0.0
            print(f"Transform: ENU from Pin 1 as origin, shift by ({off_x:.2f}, {off_y:.2f}) to remove negatives.")
            clamped_path = [(
                max(0.0, min(p[0] + off_x, config.FIELD_LENGTH)),
                max(0.0, min(p[1] + off_y, config.FIELD_WIDTH))
            ) for p in path_local]
            path_points = " ".join(to_px(p) for p in clamped_path)
            if pins_local:
                pins_local = [(x + off_x, y + off_y) for (x, y) in pins_local]
            html = []
            html.append("<!DOCTYPE html><html><head><meta charset='utf-8'><title>Tractor Visualization</title></head><body>")
            html.append(f"<svg width='{svg_w}' height='{svg_h}' viewBox='0 0 {svg_w} {svg_h}' xmlns='http://www.w3.org/2000/svg'>")
            html.append(f"<rect x='0' y='0' width='{svg_w}' height='{svg_h}' fill='white' stroke='black'/>")
            rect_points = [
                (0, 0), (config.FIELD_LENGTH, 0),
                (config.FIELD_LENGTH, config.FIELD_WIDTH),
                (0, config.FIELD_WIDTH), (0, 0)
            ]
            html.append(f"<polyline points='{' '.join(to_px(p) for p in rect_points)}' fill='none' stroke='black' stroke-width='1'/>")
            # Optional: overlay field boundary pins
            if hasattr(config, "FIELD_PINS") and config.FIELD_PINS:
                pins_local.append(pins_local[0])
                boundary_points = " ".join(to_px(p) for p in pins_local)
                html.append(f"<polyline points='{boundary_points}' fill='none' stroke='red' stroke-width='1'/>")
            html.append(f"<polyline points='{path_points}' fill='none' stroke='#1f77b4' stroke-width='2'/>")
            html.append("</svg></body></html>")
            with open("data/plot.html", "w") as f:
                f.write("\n".join(html))
            print("Visualization saved to data/plot.html")
        
    print("==================================================")

if __name__ == "__main__":
    main()
