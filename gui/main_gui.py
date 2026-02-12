import tkinter as tk
from tkinter import messagebox
import subprocess
import os
import sys
import csv
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# ==================================================
# PATHS
# ==================================================
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC_DIR = os.path.join(BASE_DIR, "src")
DATA_DIR = os.path.join(SRC_DIR, "data")

# Add SRC to path for GPS import
sys.path.append(SRC_DIR)
from gps_waypoint_generator import generate_gps_waypoints

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
                     width=22,
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

        tk.Label(root, text="🌾 Field Boundary Setup",
                 font=FONT_HEADER, fg=HEADER_COLOR,
                 bg=BG_COLOR).pack(pady=30)

        frame = tk.Frame(root, bg=BG_COLOR)
        frame.pack(pady=40)

        tk.Label(frame, text="Field Length (X)", font=FONT_LABEL,
                 bg=BG_COLOR).grid(row=0, column=0, pady=15)
        self.length = tk.Entry(frame, font=FONT_LABEL)
        self.length.grid(row=0, column=1)

        tk.Label(frame, text="Field Width (Y)", font=FONT_LABEL,
                 bg=BG_COLOR).grid(row=1, column=0, pady=15)
        self.width = tk.Entry(frame, font=FONT_LABEL)
        self.width.grid(row=1, column=1)

        styled_button(root, "View Field ➜", self.view_field).pack(pady=40)

    def view_field(self):
        try:
            L = float(self.length.get())
            W = float(self.width.get())
        except:
            messagebox.showerror("Error", "Enter valid numbers")
            return

        BoundaryScreen(self.root, L, W)


# ==================================================
# FIELD BOUNDARY VIEW
# ==================================================
class BoundaryScreen:
    def __init__(self, parent, L, W):
        win = tk.Toplevel(parent)
        win.title("Field Boundary")
        win.state("zoomed")
        win.configure(bg=BG_COLOR)

        tk.Label(win, text="Field Boundary View",
                 font=FONT_HEADER, bg=BG_COLOR,
                 fg=HEADER_COLOR).pack(pady=20)

        fig, ax = plt.subplots(figsize=(10, 5))
        ax.plot([0, L, L, 0, 0], [0, 0, W, W, 0], 'g', linewidth=3)
        ax.set_xlim(0, L)
        ax.set_ylim(0, W)
        ax.axis("equal")
        ax.grid(True)

        canvas = FigureCanvasTkAgg(fig, win)
        canvas.draw()
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        btn = tk.Frame(win, bg=BG_COLOR)
        btn.pack(pady=20)

        styled_button(btn, "⬅ Back", win.destroy).grid(row=0, column=0, padx=20)
        styled_button(btn, "Next ➜",
                      lambda: BaselineGUI(parent, L, W)).grid(row=0, column=1, padx=20)


# ==================================================
# DASHBOARD 2 : BASELINE CONFIG
# ==================================================
class BaselineGUI:
    def __init__(self, root, L, W):
        self.root = root
        self.L = L
        self.W = W

        win = tk.Toplevel(root)
        win.title("Baseline Configuration")
        win.state("zoomed")
        win.configure(bg=BG_COLOR)

        tk.Label(win, text="🚜 Baseline Configuration",
                 font=FONT_HEADER, bg=BG_COLOR,
                 fg=HEADER_COLOR).pack(pady=30)

        frame = tk.Frame(win, bg=BG_COLOR)
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

        btn = tk.Frame(win, bg=BG_COLOR)
        btn.pack(pady=30)

        styled_button(btn, "⬅ Back", win.destroy).grid(row=0, column=0, padx=20)
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
                           self.direction.get())


# ==================================================
# DASHBOARD 3 : ROW SPACING
# ==================================================
class RowSpacingGUI:
    def __init__(self, root, L, W, direction):
        self.root = root
        self.L = L
        self.W = W
        self.direction = direction

        win = tk.Toplevel(root)
        win.title("Row Spacing")
        win.state("zoomed")
        win.configure(bg=BG_COLOR)

        tk.Label(win, text="🌱 Row Spacing Input",
                 font=FONT_HEADER,
                 bg=BG_COLOR,
                 fg=HEADER_COLOR).pack(pady=30)

        self.row_entry = tk.Entry(win, font=FONT_LABEL)
        self.row_entry.pack(pady=20)

        btn = tk.Frame(win, bg=BG_COLOR)
        btn.pack(pady=30)

        styled_button(btn, "⬅ Back", win.destroy).grid(row=0, column=0, padx=20)
        styled_button(btn, "Generate 🌾", self.generate).grid(row=0, column=1, padx=20)

    def generate(self):
        try:
            ROW = float(self.row_entry.get())
        except:
            messagebox.showerror("Error", "Enter valid row spacing")
            return

        script_path = os.path.join(SRC_DIR, "path_generator.py")

        subprocess.run(
            [sys.executable, script_path,
             str(self.L), str(self.W),
             str(ROW), self.direction],
            check=True
        )

        FinalPlotScreen(self.root)


# ==================================================
# FINAL SCREEN + GPS BUTTON
# ==================================================
class FinalPlotScreen:
    def __init__(self, parent):
        win = tk.Toplevel(parent)
        win.title("Complete Field Path")
        win.state("zoomed")

        csv_path = os.path.join(DATA_DIR, "waypoints.csv")

        xs, ys = [], []
        with open(csv_path) as f:
            reader = csv.DictReader(f)
            for row in reader:
                xs.append(float(row["x"]))
                ys.append(float(row["y"]))

        fig, ax = plt.subplots(figsize=(12, 6))
        ax.plot(xs, ys, linewidth=2, color="green")

        ax.set_xlim(left=0)
        ax.set_ylim(bottom=0)
        ax.axis("equal")
        ax.grid(True)

        canvas = FigureCanvasTkAgg(fig, win)
        canvas.draw()
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # GPS BUTTON
        btn_frame = tk.Frame(win, bg=BG_COLOR)
        btn_frame.pack(pady=20)

        styled_button(btn_frame,
                      "Generate GPS Waypoints 🛰",
                      lambda: GPSInputScreen(win)).pack()


# ==================================================
# GPS INPUT SCREEN
# ==================================================
class GPSInputScreen:
    def __init__(self, parent):

        win = tk.Toplevel(parent)
        win.title("GPS Waypoint Generator")
        win.state("zoomed")
        win.configure(bg=BG_COLOR)

        tk.Label(win,
                 text="🛰 GPS Field Coordinates",
                 font=FONT_HEADER,
                 bg=BG_COLOR,
                 fg=HEADER_COLOR).pack(pady=30)

        frame = tk.Frame(win, bg=BG_COLOR)
        frame.pack(pady=40)

        labels = ["Point A Latitude", "Point A Longitude",
                  "Point B Latitude", "Point B Longitude",
                  "Point C Latitude", "Point C Longitude"]

        self.entries = []

        for i, text in enumerate(labels):
            tk.Label(frame, text=text,
                     font=FONT_LABEL,
                     bg=BG_COLOR).grid(row=i, column=0, pady=10)
            entry = tk.Entry(frame, font=FONT_LABEL)
            entry.grid(row=i, column=1)
            self.entries.append(entry)

        btn_frame = tk.Frame(win, bg=BG_COLOR)
        btn_frame.pack(pady=30)

        styled_button(btn_frame,
                      "Generate GPS 🌍",
                      self.generate_gps).grid(row=0, column=0, padx=20)

        styled_button(btn_frame,
                      "Close",
                      win.destroy).grid(row=0, column=1, padx=20)

    def generate_gps(self):
        try:
            pointA = (float(self.entries[0].get()), float(self.entries[1].get()))
            pointB = (float(self.entries[2].get()), float(self.entries[3].get()))
            pointC = (float(self.entries[4].get()), float(self.entries[5].get()))

            csv_path = generate_gps_waypoints(
                pointA,
                pointB,
                pointC,
                row_spacing=4.0,
                point_spacing=0.5
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
