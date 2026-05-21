import math
from collections import deque


class TorqueVelocityGraph:
    def __init__(self, canvas, maxlen=2400):
        self.canvas = canvas
        self.ax = canvas.ax

        self._time = deque(maxlen=maxlen)
        self._left_torque = deque(maxlen=maxlen)
        self._right_torque = deque(maxlen=maxlen)
        self._left_velocity = deque(maxlen=maxlen)
        self._right_velocity = deque(maxlen=maxlen)

        self._line_left_torque = None
        self._line_right_torque = None
        self._line_left_velocity = None
        self._line_right_velocity = None

        self._setup()

    def _setup(self):
        self.ax.cla()

        self._line_left_torque, = self.ax.plot([], [], label='Torque izq', color='tab:orange')
        self._line_right_torque, = self.ax.plot([], [], label='Torque der', color='tab:green')
        self._line_left_velocity, = self.ax.plot([], [], label='Velocidad izq', color='tab:red', linestyle='--')
        self._line_right_velocity, = self.ax.plot([], [], label='Velocidad der', color='tab:blue', linestyle='--')

        self.ax.set_title('Motor torque/velocity vs time')
        self.ax.set_xlabel('Time [s]')
        self.ax.set_ylabel('Value')
        self.ax.grid(True)
        self.ax.legend(loc='upper right')

        self.canvas.draw_idle()

    def update_data(
        self,
        elapsed_time,
        left_torque,
        right_torque,
        left_velocity,
        right_velocity,
        show_left_torque=True,
        show_right_torque=True,
        show_left_velocity=True,
        show_right_velocity=True,
        window_seconds=30.0,
    ):
        self._time.append(elapsed_time)
        self._left_torque.append(left_torque)
        self._right_torque.append(right_torque)
        self._left_velocity.append(left_velocity)
        self._right_velocity.append(right_velocity)

        min_time = elapsed_time - window_seconds
        while self._time and self._time[0] < min_time:
            self._time.popleft()
            self._left_torque.popleft()
            self._right_torque.popleft()
            self._left_velocity.popleft()
            self._right_velocity.popleft()

        time_data = list(self._time)
        left_torque_data = list(self._left_torque)
        right_torque_data = list(self._right_torque)
        left_velocity_data = list(self._left_velocity)
        right_velocity_data = list(self._right_velocity)

        self._line_left_torque.set_data(time_data, left_torque_data)
        self._line_right_torque.set_data(time_data, right_torque_data)
        self._line_left_velocity.set_data(time_data, left_velocity_data)
        self._line_right_velocity.set_data(time_data, right_velocity_data)

        self._line_left_torque.set_visible(show_left_torque)
        self._line_right_torque.set_visible(show_right_torque)
        self._line_left_velocity.set_visible(show_left_velocity)
        self._line_right_velocity.set_visible(show_right_velocity)

        if time_data:
            xmin = time_data[0]
            xmax = time_data[-1]
            if xmax <= xmin:
                xmax = xmin + 1.0
            self.ax.set_xlim(xmin, xmax)
        else:
            self.ax.set_xlim(0.0, 10.0)

        visible_values = []
        if show_left_torque:
            visible_values.extend([
                value for value in left_torque_data if isinstance(value, (int, float)) and math.isfinite(value)
            ])
        if show_right_torque:
            visible_values.extend([
                value for value in right_torque_data if isinstance(value, (int, float)) and math.isfinite(value)
            ])
        if show_left_velocity:
            visible_values.extend([
                value for value in left_velocity_data if isinstance(value, (int, float)) and math.isfinite(value)
            ])
        if show_right_velocity:
            visible_values.extend([
                value for value in right_velocity_data if isinstance(value, (int, float)) and math.isfinite(value)
            ])

        if visible_values:
            ymin = min(visible_values)
            ymax = max(visible_values)
            margin = max((ymax - ymin) * 0.1, 0.5)
            self.ax.set_ylim(ymin - margin, ymax + margin)
        else:
            self.ax.set_ylim(-1.0, 1.0)

        current_legend = self.ax.get_legend()
        if current_legend is not None:
            current_legend.remove()

        legend_lines = []
        legend_labels = []
        if show_left_torque:
            legend_lines.append(self._line_left_torque)
            legend_labels.append(self._line_left_torque.get_label())
        if show_right_torque:
            legend_lines.append(self._line_right_torque)
            legend_labels.append(self._line_right_torque.get_label())
        if show_left_velocity:
            legend_lines.append(self._line_left_velocity)
            legend_labels.append(self._line_left_velocity.get_label())
        if show_right_velocity:
            legend_lines.append(self._line_right_velocity)
            legend_labels.append(self._line_right_velocity.get_label())

        if legend_lines:
            self.ax.legend(legend_lines, legend_labels, loc='upper right')

        self.canvas.draw_idle()
