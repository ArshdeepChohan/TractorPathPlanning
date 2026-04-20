import tkinter as tk
from tkinter import messagebox
import subprocess
import os
import sys
import csv
import math
import json
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# ==================================================
# PATHS
# ==================================================
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC_DIR = os.path.join(BASE_DIR, "src")
DATA_DIR = os.path.join(SRC_DIR, "data")

# ==================================================
# THEME
# ==================================================
BG_COLOR = "#e8f5e9"
HEADER_COLOR = "#2e7d32"
BTN_COLOR = "#388e3c"
BTN_HOVER = "#1b5e20"

FONT_HEADER = ("Arial", 22, "bold")
FONT_LABEL = ("Arial", 14)
FONT_BUTTON = ("Arial", 14, "bold")


def styled_button(parent, text, command):
    return tk.Button(parent,
                     text=text,
                     font=FONT_BUTTON,
                     bg=BTN_COLOR,
                     fg="white",
                     activebackground=BTN_HOVER,
                     activeforeground="white",
                     width=18,
                     height=2,
                     bd=0,
                     command=command)

# ==================================================
# DASHBOARD 1 : FIELD BOUNDARY
# ==================================================
class FieldBoundaryGUI:

    def __init__(self, root):
        self.root = root
        root.title("Autonomous Tractor Path Planning")
        root.state("zoomed")
        root.configure(bg=BG_COLOR)

        tk.Label(root,
                 text="🌍 Field Shape + GPS Input",
                 font=FONT_HEADER,
                 fg=HEADER_COLOR,
                 bg=BG_COLOR).pack(pady=30)

        frame = tk.Frame(root, bg=BG_COLOR)
        frame.pack(pady=20)

        tk.Label(frame, text="Field Shape", font=FONT_LABEL, bg=BG_COLOR).grid(row=0, column=0, pady=8)
        self.shape_var = tk.StringVar(value="rectangle")
        shape_menu = tk.OptionMenu(frame, self.shape_var, "rectangle", "square", "trapezium", "triangle", "rhombus")
        shape_menu.config(font=FONT_LABEL, width=22)
        shape_menu.grid(row=0, column=1, pady=8)
        self.shape_var.trace_add("write", lambda *_: self.render_point_entries())

        self.points_frame = tk.Frame(frame, bg=BG_COLOR)
        self.points_frame.grid(row=1, column=0, columnspan=2)
        self.entries = []
        self.render_point_entries()

        styled_button(root,
                      "Generate Field ➜",
                      self.generate_field).pack(pady=40)

    def required_points(self):
        shape_counts = {
            "triangle": 3,
            "rectangle": 4,
            "square": 4,
            "trapezium": 4,
            "rhombus": 4,
        }
        return shape_counts.get(self.shape_var.get(), 4)

    def render_point_entries(self):
        for widget in self.points_frame.winfo_children():
            widget.destroy()
        self.entries = []
        count = self.required_points()
        for idx in range(count):
            label_lat = f"Point {chr(ord('A') + idx)} Latitude"
            label_lon = f"Point {chr(ord('A') + idx)} Longitude"
            tk.Label(self.points_frame, text=label_lat, font=FONT_LABEL, bg=BG_COLOR).grid(row=idx * 2, column=0, pady=4)
            entry_lat = tk.Entry(self.points_frame, font=FONT_LABEL, width=25)
            entry_lat.grid(row=idx * 2, column=1, pady=4)
            tk.Label(self.points_frame, text=label_lon, font=FONT_LABEL, bg=BG_COLOR).grid(row=idx * 2 + 1, column=0, pady=4)
            entry_lon = tk.Entry(self.points_frame, font=FONT_LABEL, width=25)
            entry_lon.grid(row=idx * 2 + 1, column=1, pady=4)
            self.entries.extend([entry_lat, entry_lon])

    def generate_field(self):

        values = []

        for entry in self.entries:
            val = entry.get().strip()

            if val == "":
                messagebox.showerror("Error", "All GPS fields must be filled")
                return

            try:
                values.append(float(val))
            except ValueError:
                messagebox.showerror("Error", f"Invalid number:\n{val}")
                return

        points = []
        for i in range(0, len(values), 2):
            points.append((values[i], values[i + 1]))

        def distance(p1, p2):
            lat1, lon1 = p1
            lat2, lon2 = p2
            avg_lat = math.radians((lat1 + lat2) / 2)
            dlat = (lat2 - lat1) * 111320
            dlon = (lon2 - lon1) * 111320 * math.cos(avg_lat)
            return math.sqrt(dlat**2 + dlon**2)

        A, B = points[0], points[1]
        C = points[2]
        L = distance(A, B)
        W = distance(B, C)
        shape = self.shape_var.get()

        messagebox.showinfo(
            "Field Size",
            f"Shape: {shape.title()}\nLength ≈ {L:.2f} meters\nWidth ≈ {W:.2f} meters"
        )
        BoundaryScreen(self.root, L, W, shape, points)

# ==================================================
# FIELD BOUNDARY VIEW
# ==================================================
class BoundaryScreen:
    def __init__(self, parent, L, W, shape, gps_points):
        self.win = tk.Toplevel(parent)
        self.shape = shape
        self.gps_points = gps_points
        self.win.title("Field Boundary")
        self.win.state("zoomed")
        self.win.configure(bg=BG_COLOR)

        tk.Label(self.win, text="Field Boundary View",
                 font=FONT_HEADER,
                 bg=BG_COLOR,
                 fg=HEADER_COLOR).pack(pady=20)

        fig, ax = plt.subplots(figsize=(10, 5))
        if shape == "rectangle":
            ax.plot([0, L, L, 0, 0], [0, 0, W, W, 0], 'g', linewidth=3)
            ax.set_xlim(0, L)
            ax.set_ylim(0, W)
        else:
            local_points = self._gps_to_local(gps_points)
            poly_x = [p[0] for p in local_points] + [local_points[0][0]]
            poly_y = [p[1] for p in local_points] + [local_points[0][1]]
            ax.plot(poly_x, poly_y, 'g', linewidth=3)
            ax.set_xlim(min(poly_x) - 2, max(poly_x) + 2)
            ax.set_ylim(min(poly_y) - 2, max(poly_y) + 2)
        ax.axis("equal")
        ax.grid(True)

        canvas = FigureCanvasTkAgg(fig, self.win)
        canvas.draw()
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        btn = tk.Frame(self.win, bg=BG_COLOR)
        btn.pack(pady=20)

        styled_button(btn, "⬅ Back", self.win.destroy).grid(row=0, column=0, padx=20)
        styled_button(btn, "Next ➜",
                      lambda: BaselineGUI(parent, L, W, self.shape, self.gps_points)).grid(row=0, column=1, padx=20)

    def _gps_to_local(self, points):
        lat0, lon0 = points[0]
        local = []
        for lat, lon in points:
            avg_lat = math.radians((lat0 + lat) / 2.0)
            x = (lon - lon0) * 111320.0 * math.cos(avg_lat)
            y = (lat - lat0) * 111320.0
            local.append((x, y))
        return local

# ==================================================
# DASHBOARD 2 : BASELINE CONFIG
# ==================================================
class BaselineGUI:
    def __init__(self, root, L, W, shape="rectangle", gps_points=None):
        self.root = root
        self.L = L
        self.W = W
        self.shape = shape
        self.gps_points = gps_points or []

        self.win = tk.Toplevel(root)
        self.win.title("Baseline Configuration")
        self.win.state("zoomed")
        self.win.configure(bg=BG_COLOR)

        tk.Label(self.win, text="🚜 Baseline Configuration",
                 font=FONT_HEADER,
                 bg=BG_COLOR,
                 fg=HEADER_COLOR).pack(pady=30)

        frame = tk.Frame(self.win, bg=BG_COLOR)
        frame.pack(pady=40)

        self.direction = tk.StringVar(value="lengthwise")

        tk.Radiobutton(frame, text="Lengthwise",
                       variable=self.direction,
                       value="lengthwise",
                       bg=BG_COLOR,
                       font=FONT_LABEL).grid(row=0, column=0, padx=20)

        tk.Radiobutton(frame, text="Widthwise",
                       variable=self.direction,
                       value="widthwise",
                       bg=BG_COLOR,
                       font=FONT_LABEL).grid(row=0, column=1, padx=20)

        tk.Label(frame, text="Baseline Length",
                 font=FONT_LABEL, bg=BG_COLOR).grid(row=1, column=0, pady=20)
        self.base_entry = tk.Entry(frame, font=FONT_LABEL)
        self.base_entry.grid(row=1, column=1)

        tk.Label(frame, text="Diagonal Entry",
                 font=FONT_LABEL, bg=BG_COLOR).grid(row=2, column=0, pady=20)
        self.diag_entry = tk.Entry(frame, font=FONT_LABEL)
        self.diag_entry.grid(row=2, column=1)

        btn = tk.Frame(self.win, bg=BG_COLOR)
        btn.pack(pady=30)

        styled_button(btn, "⬅ Back", self.win.destroy).grid(row=0, column=0, padx=20)
        styled_button(btn, "Next ➜", self.preview).grid(row=0, column=1, padx=20)

    def preview(self):
        try:
            base_len = float(self.base_entry.get())
            diag = float(self.diag_entry.get())
        except:
            messagebox.showerror("Error", "Enter valid numbers")
            return

        BaselinePreviewGUI(self.root,
                           self.L,
                           self.W,
                           base_len,
                           diag,
                           self.direction.get(),
                           self.shape,
                           self.gps_points)

# ==================================================
# BASELINE PREVIEW
# ==================================================
class BaselinePreviewGUI:
    def __init__(self, root, L, W, base_len, diag, direction, shape="rectangle", gps_points=None):
        self.shape = shape
        self.gps_points = gps_points or []

        self.win = tk.Toplevel(root)
        self.win.title("Baseline + Diagonal Preview")
        self.win.state("zoomed")
        self.win.configure(bg=BG_COLOR)

        tk.Label(self.win, text="Baseline + Diagonal Entry",
                 font=FONT_HEADER,
                 bg=BG_COLOR,
                 fg=HEADER_COLOR).pack(pady=20)

        fig, ax = plt.subplots(figsize=(10, 5))

        if self.shape == "rectangle":
            ax.plot([0, L, L, 0, 0], [0, 0, W, W, 0], 'black')
        else:
            local_points = self._gps_to_local(self.gps_points)
            bx = [p[0] for p in local_points] + [local_points[0][0]]
            by = [p[1] for p in local_points] + [local_points[0][1]]
            ax.plot(bx, by, 'black')
        ax.plot([0, diag], [0, diag], 'blue', linewidth=3)

        if direction == "lengthwise":
            ax.plot([diag, diag + base_len], [diag, diag], 'red', linewidth=3)
        else:
            ax.plot([diag, diag], [diag, diag + base_len], 'red', linewidth=3)

        ax.set_xlim(0, L)
        ax.set_ylim(0, W)
        ax.axis("equal")
        ax.grid(True)

        canvas = FigureCanvasTkAgg(fig, self.win)
        canvas.draw()
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        btn = tk.Frame(self.win, bg=BG_COLOR)
        btn.pack(pady=20)

        styled_button(btn, "⬅ Back", self.win.destroy).grid(row=0, column=0, padx=20)
        styled_button(btn, "Next ➜",
                      lambda: RowSpacingGUI(root, L, W, direction, base_len, diag, self.shape, self.gps_points)).grid(row=0, column=1, padx=20)

    def _gps_to_local(self, points):
        lat0, lon0 = points[0]
        local = []
        for lat, lon in points:
            avg_lat = math.radians((lat0 + lat) / 2.0)
            x = (lon - lon0) * 111320.0 * math.cos(avg_lat)
            y = (lat - lat0) * 111320.0
            local.append((x, y))
        return local

# ==================================================
# DASHBOARD 3 : ROW SPACING
# ==================================================
class RowSpacingGUI:
    def __init__(self, root, L, W, direction, base_len, diag, shape="rectangle", gps_points=None):
        self.root = root
        self.L = L
        self.W = W
        self.direction = direction
        self.base_len = base_len
        self.diag = diag
        self.shape = shape
        self.gps_points = gps_points or []

        self.win = tk.Toplevel(root)
        self.win.title("Row Spacing")
        self.win.state("zoomed")
        self.win.configure(bg=BG_COLOR)

        tk.Label(self.win, text="🌱 Row Spacing Input",
                 font=FONT_HEADER,
                 bg=BG_COLOR,
                 fg=HEADER_COLOR).pack(pady=30)

        self.row_entry = tk.Entry(self.win, font=FONT_LABEL)
        self.row_entry.pack(pady=20)

        tk.Label(self.win, text="Turning Radius (m)", font=FONT_LABEL, bg=BG_COLOR).pack(pady=10)
        self.turn_entry = tk.Entry(self.win, font=FONT_LABEL)
        self.turn_entry.insert(0, "4.0")
        self.turn_entry.pack(pady=5)

        btn = tk.Frame(self.win, bg=BG_COLOR)
        btn.pack(pady=30)

        styled_button(btn, "⬅ Back", self.win.destroy).grid(row=0, column=0, padx=20)
        styled_button(btn, "Generate 🌾", self.generate).grid(row=0, column=1, padx=20)

    def generate(self):
        try:
            ROW = float(self.row_entry.get())
            turn_radius = float(self.turn_entry.get())
        except:
            messagebox.showerror("Error", "Enter valid row spacing and turning radius")
            return

        script_path = os.path.join(SRC_DIR, "path_generator.py")

        cmd = [
            sys.executable, script_path,
            str(self.L), str(self.W),
            str(ROW), self.direction,
            str(self.base_len), str(self.diag),
            "--shape", self.shape,
            "--turn-radius", str(turn_radius),
            "--export-json",
        ]
        if self.shape != "rectangle" and self.gps_points:
            local_points = self._gps_to_local(self.gps_points)
            cmd.extend(["--polygon-json", json.dumps(local_points)])

        subprocess.run(cmd, check=True)

        FinalPlotScreen(self.root, self.L, self.W, ROW, self.direction, self.base_len, self.diag, self.shape, self.gps_points)

    def _gps_to_local(self, points):
        lat0, lon0 = points[0]
        local = []
        for lat, lon in points:
            avg_lat = math.radians((lat0 + lat) / 2.0)
            x = (lon - lon0) * 111320.0 * math.cos(avg_lat)
            y = (lat - lat0) * 111320.0
            local.append((x, y))
        return local

# ==================================================
# FINAL SCREEN
# ==================================================
class FinalPlotScreen:
    def __init__(self, parent, L, W, ROW, direction, base_len, diag, shape="rectangle", gps_points=None):

        self.parent = parent
        self.L = L
        self.W = W
        self.ROW = ROW
        self.direction = direction
        self.base_len = base_len
        self.diag = diag
        self.shape = shape
        self.gps_points = gps_points or []
        self.win = tk.Toplevel(parent)
        self.win.title("Complete Field Path")
        self.win.state("zoomed")

        csv_path = os.path.join(DATA_DIR, "waypoints.csv")

        waypoints = []
        metadata = {}
        with open(csv_path) as f:
            reader = csv.reader(f)
            for row in reader:
                if row and row[0].startswith("#") and len(row) >= 7:
                    try:
                        metadata = json.loads(row[6])
                    except Exception:
                        metadata = {}
                if not row or row[0].startswith("#") or row[0] == "x":
                    continue
                waypoints.append({'x': float(row[0]), 'y': float(row[1]), 'type': int(row[2])})

        # Get L, W, direction, diag, base_len from self
        L, W, direction = self.L, self.W, self.direction
        ROW = self.ROW
        diag = self.diag
        base_len = self.base_len

        fig, ax = plt.subplots(figsize=(12, 6))
        
        # 1. Plot Field Boundary
        polygon = metadata.get("polygon")
        if polygon:
            bx = [p[0] for p in polygon] + [polygon[0][0]]
            by = [p[1] for p in polygon] + [polygon[0][1]]
            ax.plot(bx, by, "k--", linewidth=2, label="Field Boundary")
        else:
            bx = [0, L, L, 0, 0]
            by = [0, 0, W, W, 0]
            ax.plot(bx, by, "k--", linewidth=2, label="Field Boundary")

        # Plot Diagonal and Baseline for visual reference
        if direction == "lengthwise":
            ax.plot([0, diag], [0, diag], color="blue", linestyle=":", linewidth=2, label="Diagonal")
            ax.plot([diag, diag + base_len], [diag, diag], color="red", linestyle="-", linewidth=2.5, label="Baseline")
        else:
            ax.plot([0, diag], [0, diag], color="blue", linestyle=":", linewidth=2, label="Diagonal")
            ax.plot([diag, diag], [diag, diag + base_len], color="red", linestyle="-", linewidth=2.5, label="Baseline")
        
        # 2. Plot Fixed Crop Rows (Thin grey lines)
        # Re-calculate row positions for display
        if direction == "lengthwise":
            available_width = W - diag
            rows_count = max(2, int(math.floor(available_width / ROW)) + 1)
            row_positions = [diag + i * ROW for i in range(rows_count) if (diag + i * ROW) <= W]
            for idx, pos in enumerate(row_positions):
                label = "Planned Crop Rows" if idx == 0 else ""
                if polygon:
                    xs_intersections = self._line_polygon_intersections(polygon, pos, "lengthwise")
                    if len(xs_intersections) >= 2:
                        ax.plot([xs_intersections[0], xs_intersections[-1]], [pos, pos], color="grey", linestyle="--", linewidth=0.8, alpha=0.3, label=label)
                else:
                    ax.plot([0, L], [pos, pos], color="grey", linestyle="--", linewidth=0.8, alpha=0.3, label=label)
        else:
            available_length = L - diag
            rows_count = max(2, int(math.floor(available_length / ROW)) + 1)
            row_positions = [diag + i * ROW for i in range(rows_count) if (diag + i * ROW) <= L]
            for idx, pos in enumerate(row_positions):
                label = "Planned Crop Rows" if idx == 0 else ""
                if polygon:
                    ys_intersections = self._line_polygon_intersections(polygon, pos, "widthwise")
                    if len(ys_intersections) >= 2:
                        ax.plot([pos, pos], [ys_intersections[0], ys_intersections[-1]], color="grey", linestyle="--", linewidth=0.8, alpha=0.3, label=label)
                else:
                    ax.plot([pos, pos], [0, W],color="grey", linestyle="--", linewidth=0.8, alpha=0.3, label=label)

        # 3. Plot Tractor Travel Path (Differentiated by type)
        # We need to split waypoints into segments for coloring
        current_segment_xs = []
        current_segment_ys = []
        current_type = waypoints[0]['type']
        
        for i, wp in enumerate(waypoints):
            if wp['type'] == current_type:
                current_segment_xs.append(wp['x'])
                current_segment_ys.append(wp['y'])
            else:
                # Plot the segment we just finished
                if current_type == 0:
                    color, label = "#1f77b4", "Tractor Path (Rows)"
                elif current_type == 1:
                    color, label = "#d62728", "U-Turn Segments"
                else: # type 2
                    color, label = "#9467bd", "Initial Diagonal Move"
                
                # Avoid duplicate labels in legend
                if any(l.get_label() == label for l in ax.get_lines()):
                    label = ""
                ax.plot(current_segment_xs, current_segment_ys,
                        color=color, linewidth=3, label=label, zorder=3)
             
                # Start new segment (include last point of previous for continuity)
                current_segment_xs = [current_segment_xs[-1], wp['x']]
                current_segment_ys = [current_segment_ys[-1], wp['y']]
                current_type = wp['type']
        
        # Plot the final segment
        if current_type == 0:
            color, label = "#1f77b4", "Tractor Path (Rows)"
        elif current_type == 1:
            color, label = "#d62728", "U-Turn Segments"
        else: # type 2
            color, label = "#9467bd", "Initial Diagonal Move"
        
        if any(l.get_label() == label for l in ax.get_lines()):
            label = ""
        ax.plot(current_segment_xs, current_segment_ys, color=color, linewidth=3, label=label, zorder=3)

        
        # 4. GPS Waypoints (Clear markers)
        xs = [wp['x'] for wp in waypoints]
        ys = [wp['y'] for wp in waypoints]
        # Keep legacy widthwise swap for rectangle mode only.
        if direction == "widthwise" and not polygon:
            xs, ys = ys, xs

        ax.scatter(xs[::5], ys[::5], color="black", s=15, zorder=4, label="GPS Waypoints")

        # 5. Start and End Positions (Distinct Markers)
        ax.scatter(0, 0, color="#2ca02c", marker="*", s=250, edgecolors="black", zorder=5, label="START")
        ax.scatter(xs[-1], ys[-1], color="#ff7f0e", marker="X", s=150, edgecolors="black", zorder=5, label="END")

        ax.set_title("Autonomous Tractor Path - Skip Row Method", fontsize=14, pad=15)
        ax.set_xlabel("Length (m)")
        ax.set_ylabel("Width (m)")
        ax.axis("equal")
        ax.grid(True, linestyle=":", alpha=0.5)
        ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.15), ncol=3, fontsize=9)

        canvas = FigureCanvasTkAgg(fig, self.win)
        canvas.draw()
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

                # ==================================================
        # 🚜 TRACTOR ANIMATION (FIXED + WORKING)
        # ==================================================

        # Use marker instead of emoji (reliable)
        tractor, = ax.plot(xs[0], ys[0],
                           marker="s",
                           markersize=10,
                           color="green",
                           zorder=6)

        # Trail line
        trail_line, = ax.plot([], [], color="orange", linewidth=2, zorder=5)

        def init_anim():
            tractor.set_data([xs[0]], [ys[0]])
            trail_line.set_data([], [])
            return tractor, trail_line

        def update_anim(frame):
            x = xs[frame]
            y = ys[frame]

            tractor.set_data([x], [y])
            trail_line.set_data(xs[:frame], ys[:frame])

            return tractor, trail_line


        # ✅ FIXED: OUTSIDE function
        self.ani = FuncAnimation(
            fig,
            update_anim,
            frames=len(xs),
            init_func=init_anim,
            interval=20,
            blit=False,
            repeat=False
        )
        # STOP initially (so Start button works)
        if hasattr(self, "ani") and self.ani is not None:
            try:
                self.ani.event_source.stop()
            except:
                pass

                # ===============================
        # 🎮 ANIMATION CONTROLS
        # ===============================

        def start_animation():
            if hasattr(self, "ani"):
                self.ani.event_source.start()

        def pause_animation():
            if hasattr(self, "ani"):
                self.ani.event_source.stop()

        
        # ================= BUTTON FRAME =================
        btn_frame = tk.Frame(self.win)
        btn_frame.pack(pady=20)

        styled_button(
            btn_frame,
            "⬅ Back",
            self.win.destroy
        ).grid(row=0, column=0, padx=15)

        styled_button(
            btn_frame,
            "▶ Start",
            start_animation
        ).grid(row=0, column=1, padx=15)

        styled_button(
            btn_frame,
            "⏸ Pause",
            pause_animation
        ).grid(row=0, column=2, padx=15)

        styled_button(
            btn_frame,
            "Next ➜",
            self.open_gps_dashboard
        ).grid(row=0, column=3, padx=15)

        styled_button(
            btn_frame,
            "Export JSON",
            self.export_json_info
        ).grid(row=0, column=4, padx=15)

    def _line_polygon_intersections(self, polygon, axis_value, direction):
        hits = []
        n = len(polygon)
        for i in range(n):
            x1, y1 = polygon[i]
            x2, y2 = polygon[(i + 1) % n]
            if direction == "lengthwise":
                if abs(y2 - y1) < 1e-9:
                    continue
                if axis_value >= min(y1, y2) and axis_value < max(y1, y2):
                    t = (axis_value - y1) / (y2 - y1)
                    hits.append(x1 + t * (x2 - x1))
            else:
                if abs(x2 - x1) < 1e-9:
                    continue
                if axis_value >= min(x1, x2) and axis_value < max(x1, x2):
                    t = (axis_value - x1) / (x2 - x1)
                    hits.append(y1 + t * (y2 - y1))
        return sorted(hits)

    def export_json_info(self):
        json_path = os.path.join(DATA_DIR, "waypoints.json")
        if os.path.exists(json_path):
            messagebox.showinfo("Export", f"JSON export available at:\n{json_path}")
        else:
            messagebox.showwarning("Export", "JSON file not found. Generate path first.")

    def open_gps_dashboard(self):
        GPSWaypointDashboard(self.win)


# ==================================================
# GPS WAYPOINT DASHBOARD (LAST SCREEN ONLY)
# ==================================================
class GPSWaypointDashboard:
    def __init__(self, parent):

        self.win = tk.Toplevel(parent)
        self.win.title("GPS Waypoint Generator")
        self.win.state("zoomed")
        self.win.configure(bg=BG_COLOR)

        tk.Label(self.win,
                 text="🛰 Generate GPS Waypoints",
                 font=FONT_HEADER,
                 bg=BG_COLOR,
                 fg=HEADER_COLOR).pack(pady=30)

        frame = tk.Frame(self.win, bg=BG_COLOR)
        frame.pack(pady=30)

        labels = [
            "Point A Latitude", "Point A Longitude",
            "Point B Latitude", "Point B Longitude",
            "Point C Latitude", "Point C Longitude"
        ]

        self.entries = []

        for i, text in enumerate(labels):
            tk.Label(frame, text=text,
                     font=FONT_LABEL,
                     bg=BG_COLOR).grid(row=i, column=0, pady=10)

            entry = tk.Entry(frame, font=FONT_LABEL)
            entry.grid(row=i, column=1)
            self.entries.append(entry)

        styled_button(self.win,
                      "Generate GPS 🌍",
                      self.generate_gps).pack(pady=20)

    def generate_gps(self):
        try:
            pointA = (float(self.entries[0].get()), float(self.entries[1].get()))
            pointB = (float(self.entries[2].get()), float(self.entries[3].get()))
            pointC = (float(self.entries[4].get()), float(self.entries[5].get()))

            from gps_waypoint_generator import generate_gps_waypoints

            csv_path = generate_gps_waypoints(
                pointA,
                pointB,
                pointC,
                row_spacing=4.0,
                point_spacing=1.0
            )

            messagebox.showinfo("Success",
                                f"GPS Waypoints Saved:\n{csv_path}")

        except Exception as e:
            messagebox.showerror("Error", str(e))

# ==================================================
# RUN
# ==================================================
if __name__ == "__main__":
    root = tk.Tk()
    FieldBoundaryGUI(root)
    root.mainloop()
