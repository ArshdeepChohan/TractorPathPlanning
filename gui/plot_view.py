from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure

class PlotCanvas(FigureCanvasQTAgg):
    def __init__(self, parent=None):
        fig = Figure(figsize=(6,4), dpi=100)
        self.ax = fig.add_subplot(111)
        super().__init__(fig)
        self.setParent(parent)

    def plot_path(self, x, y, FIELD_L, FIELD_W):
        self.ax.clear()
        self.ax.plot(x, y)
        self.ax.set_title("Tractor Path")
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")

        # Field boundary
        self.ax.plot(
            [0, FIELD_L, FIELD_L, 0, 0],
            [0, 0, FIELD_W, FIELD_W, 0],
            linestyle="--"
        )

        self.draw()
