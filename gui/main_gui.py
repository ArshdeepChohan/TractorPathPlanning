import tkinter as tk
from tkinter import messagebox
import subprocess
import os
import sys
import csv
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# Correct BASE (since GUI and path_generator are in src)
BASE = os.path.dirname(os.path.abspath(__file__))

# ==================================================
# DASHBOARD 1 : FIELD INPUT
# ==================================================
class FieldInputGUI:
    def __init__(self, root):
        self.root = root
        root.title("Autonomous Tractor Path Planning")
        root.state("zoomed")

        tk.Label(root,
                 text="Autonomous Tractor Path Planning",
                 font=("Arial", 22, "bold")).pack(pady=30)

        frame = tk.Frame(root)
        frame.pack(pady=40)

        self.entries = {}
        labels = ["Field Length (X)", "Field Width (Y)", "Row Spacing"]

        for i, lbl in enumerate(labels):
            tk.Label(frame, text=lbl, font=("Arial", 14)).grid(row=i, column=0, pady=15, padx=20)
            e = tk.Entry(frame, width=20, font=("Arial", 14))
            e.grid(row=i, column=1)
            self.entries[lbl] = e

        tk.Button(root,
                  text="Generate Tractor Path",
                  font=("Arial", 16),
                  width=25,
                  command=self.generate).pack(pady=40)

    def generate(self):
        try:
            L = float(self.entries["Field Length (X)"].get())
            W = float(self.entries["Field Width (Y)"].get())
            ROW = float(self.entries["Row Spacing"].get())
        except:
            messagebox.showerror("Error", "Enter valid numeric values")
            return

        script_path = os.path.join(BASE, "path_generator.py")

        try:
            subprocess.run(
                [sys.executable, script_path, str(L), str(W), str(ROW)],
                check=True
            )
        except Exception as e:
            messagebox.showerror("Error", f"Failed to run path_generator.py\n{e}")
            return

        FinalPlotScreen(self.root)


# ==================================================
# DASHBOARD 2 : FINAL PLOT
# ==================================================
class FinalPlotScreen:
    def __init__(self, parent):
        win = tk.Toplevel(parent)
        win.title("Final Tractor Path")
        win.state("zoomed")

        csv_path = os.path.join(BASE, "data", "waypoints.csv")

        if not os.path.exists(csv_path):
            messagebox.showerror("Error", "waypoints.csv not found")
            return

        xs, ys = [], []
        with open(csv_path) as f:
            reader = csv.DictReader(f)
            for row in reader:
                xs.append(float(row["x"]))
                ys.append(float(row["y"]))

        fig, ax = plt.subplots(figsize=(14, 7))
        ax.plot(xs, ys, linewidth=2, label="Tractor Path")

        FIELD_L = max(xs)
        FIELD_W = max(ys)

        ax.plot([0, FIELD_L, FIELD_L, 0, 0],
                [0, 0, FIELD_W, FIELD_W, 0],
                'k--', label="Field Boundary")

        ax.set_title("Tractor Path Planning (Diagonal Entry + Smooth U-Turns)")
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.axis("equal")
        ax.grid(True, linestyle=":")
        ax.legend()

        canvas = FigureCanvasTkAgg(fig, win)
        canvas.draw()
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        tk.Button(win, text="Back", font=("Arial", 14),
                  command=win.destroy).pack(pady=20)


# ==================================================
# RUN
# ==================================================
if __name__ == "__main__":
    root = tk.Tk()
    FieldInputGUI(root)
    root.mainloop()
