import csv
import math

# =========================
# PARAMETERS (REALISTIC)
# =========================

V_ROW = 1.4   # m/s
V_TURN = 0.7  # m/s


def load_waypoints(file):
    wp = []

    with open(file, 'r') as f:
        reader = csv.reader(f)
        next(reader)
        next(reader)

        for row in reader:
            x = float(row[0])
            y = float(row[1])
            t = int(row[2])
            wp.append((x, y, t))

    return wp


def compute_speed_efficiency(wp, L, W):

    SLIP_FACTOR = 1.25   # 👈 ADD THIS

    D_row = 0.0
    D_turn = 0.0

    for i in range(1, len(wp)):
        x1, y1, t1 = wp[i-1]
        x2, y2, t2 = wp[i]

        dist = math.hypot(x2 - x1, y2 - y1)

        # ==========================================
        # APPLY SLIP ONLY FOR TURNING
        # ==========================================
        if t2 == 1:
            dist *= SLIP_FACTOR

        if t2 == 0:
            D_row += dist

        elif t2 == 1:
            D_turn += dist

        else:
            D_row += dist  # entry treated as row

    # =========================
    # TIME CALCULATION
    # =========================

    T_row = D_row / V_ROW
    T_turn = D_turn / V_TURN

    T_total = T_row + T_turn
    D_total = D_row + D_turn

    V_eff = D_total / T_total

    # =========================
    # FIELD EFFICIENCY
    # =========================

    efficiency = T_row / T_total

    # =========================
    # FIELD CAPACITY
    # =========================

    area = L * W   # m²

    EFC = area / T_total   # m²/s
    EFC_ha = (EFC * 3600) / 10000

    return D_total, T_total, V_eff, D_row, D_turn, efficiency, EFC_ha

if __name__ == "__main__":

    file = "data\waypoints.csv"

    wp = load_waypoints(file)

    L = 100   # or read from CSV
    W = 35

    D, T, V_eff, D_row, D_turn, eff, efc = compute_speed_efficiency(wp, L, W)

    print("\n===== SPEED ANALYSIS =====")
    print(f"Total Distance: {D:.2f} m")
    print(f"Total Time: {T:.2f} sec")
    print(f"Effective Speed: {V_eff:.2f} m/s")
    print(f"Row Distance: {D_row:.2f} m")
    print(f"Turn Distance: {D_turn:.2f} m")
    print(f"Field Efficiency: {eff*100:.2f}%")
    print(f"Field Capacity: {efc:.2f} ha/hr")