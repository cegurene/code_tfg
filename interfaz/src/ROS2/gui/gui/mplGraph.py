# mplGraph.py
import math

from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as MatplotlibFigureCanvasQTAgg
from matplotlib.figure import Figure


class FigureCanvasQTAgg(MatplotlibFigureCanvasQTAgg):
    def __init__(self, parent=None):
        self.figure = Figure()
        self.ax = self.figure.add_subplot(111)
        super().__init__(self.figure)
        self.setParent(parent)

        self._left_line, = self.ax.plot([], [], label="Motor izquierdo", color="tab:red")
        self._right_line, = self.ax.plot([], [], label="Motor derecho", color="tab:blue")

        self.ax.set_title("Motor position vs time")
        self.ax.set_xlabel("Time [s]")
        self.ax.set_ylabel("Position [deg]")
        self.ax.grid(True)
        self.ax.legend(loc="upper right")
        self.figure.tight_layout()

    def update_data(self, time_data, left_data, right_data, show_left=True, show_right=True):
        self._left_line.set_data(time_data, left_data)
        self._right_line.set_data(time_data, right_data)

        self._left_line.set_visible(show_left)
        self._right_line.set_visible(show_right)

        # Keep legend in sync with visible lines
        current_legend = self.ax.get_legend()
        if current_legend is not None:
            current_legend.remove()

        legend_lines = []
        legend_labels = []
        if show_left:
            legend_lines.append(self._left_line)
            legend_labels.append("Motor izquierdo")
        if show_right:
            legend_lines.append(self._right_line)
            legend_labels.append("Motor derecho")

        if legend_lines:
            self.ax.legend(legend_lines, legend_labels, loc="upper right")

        if time_data:
            xmin = time_data[0]
            xmax = time_data[-1]
            if xmax <= xmin:
                xmax = xmin + 1.0
            self.ax.set_xlim(xmin, xmax)
        else:
            self.ax.set_xlim(0.0, 10.0)

        visible_values = []
        if show_left:
            visible_values.extend([value for value in left_data if isinstance(value, (int, float)) and math.isfinite(value)])
        if show_right:
            visible_values.extend([value for value in right_data if isinstance(value, (int, float)) and math.isfinite(value)])

        if visible_values:
            ymin = min(visible_values)
            ymax = max(visible_values)
            margin = max((ymax - ymin) * 0.1, 1.0)
            self.ax.set_ylim(ymin - margin, ymax + margin)
        else:
            self.ax.set_ylim(-100.0, 100.0)

        self.draw_idle()
