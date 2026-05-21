"""
Impedance-only GUI for controlling and monitoring an assistive exoskeleton (ROS 2).
Based on gui_assistive_exo.py but stripped of Position/Velocity/Torque modes.
"""

import sys
import threading
import math
import ctypes
import time
from collections import deque

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from PyQt6.QtWidgets import QApplication, QMainWindow
from PyQt6.QtCore import QTimer

from sensor_msgs.msg import Imu, JointState

from candle_ros2.srv import AddMd80s, GenericMd80Msg, SetModeMd80s
from candle_ros2.msg import ImpedanceCommand, MotionCommand

from ..impedance_config_py import ImpedanceConfig, find_config_file
from ..mplGraphTV import TorqueVelocityGraph
from .gui_impedance_data import Ui_MainWindow

# CAN IDs for motors (keep same as gui_assistive_exo.py)
left_hip_can_id = 308
right_hip_can_id = 11
drives_can_id = [left_hip_can_id]  # drives_can_id = [left_hip_can_id, right_hip_can_id]
num_motors = len(drives_can_id)


class MotorData(ctypes.Structure):
    _fields_ = [
        ("voltage", ctypes.c_float),
        ("current", ctypes.c_float),
        ("position", ctypes.c_float),
        ("velocity", ctypes.c_float),
        ("torque", ctypes.c_float),
    ]


class GuiNode(Node):
    def __init__(self):
        super().__init__("simple_gui_node")
        self.publisher_impedance_params = self.create_publisher(ImpedanceCommand, '/md80/impedance_command', 10)
        self.publisher_motion_commands = self.create_publisher(MotionCommand, '/md80/motion_command', 10)
        print("Node and publishers created.")


class MiVentana(QMainWindow):
    def __init__(self, node_):
        super().__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.master = node_

        # ========================================================================
        # LOAD CONFIGURATION
        # ========================================================================
        config_path = find_config_file()
        if config_path:
            self.config = ImpedanceConfig(config_path)
            self.master.get_logger().info(f"📁 GUI: Configuration loaded from {config_path}")
        else:
            self.config = ImpedanceConfig()
            self.master.get_logger().info("📋 GUI: Using default configuration values")

        if hasattr(self.config, 'print_configuration'):
            print("\n" + "=" * 60)
            print("GUI CONFIGURATION LOADED")
            print("=" * 60)
            self.config.print_configuration()

        # Motor data initialization
        self.motor_cadera_izq_data = MotorData()
        self.motor_cadera_der_data = MotorData()

        # Real-time graph buffers (position vs time)
        self._graph_time = deque(maxlen=2400)
        self._graph_left_pos = deque(maxlen=2400)
        self._graph_right_pos = deque(maxlen=2400)
        self._graph_start_time = time.monotonic()
        self._graph_widget = None
        self._graph_display_mode = 'none'
        self._graph_window_seconds = 30.0

        # Real-time graph buffers (torque/velocity vs time)
        self._graph_tv_widget = None
        self._graph_tv_plot = None

        # Motor state
        self._motors_added = False
        self._left_motor_enabled = False
        self._right_motor_enabled = False
        self._motors_enabled = False
        self._current_control_mode = "IMPEDANCE"

        # Set-based motor enable tracking
        self._desired_enabled_motors = set()

        # Impedance control variables
        self._impedance_sinusoidal_left_active = False
        self._impedance_sinusoidal_right_active = False
        self._impedance_sinusoidal_time = 0.0
        self._impedance_sinusoidal_timer = None

        # Impedance trajectory start/stop control (via buttons)
        self._impedance_trajectory_left_active = False
        self._impedance_trajectory_right_active = False

        # IMU data initialization
        self.imu_izq = Imu()
        self.imu_der = Imu()

        # QoS
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT

        # ROS2 Subscribers (guardar referencias para evitar garbage collection)
        self.imu_izq_subscriber = self.master.create_subscription(Imu, 'imu_izq/imu_data', self.imu_izq_callback, qos_profile)
        self.imu_der_subscriber = self.master.create_subscription(Imu, 'imu_der/imu_data', self.imu_der_callback, qos_profile)
        self.motor_izq_subscriber = self.master.create_subscription(JointState, 'md80/joint_states', self.motor_izq_callback, qos_profile)

        # ROS2 Service Clients
        self.addMd80Service = self.master.create_client(AddMd80s, '/candle_ros2_node/add_md80s')
        self.zeroMd80Service = self.master.create_client(GenericMd80Msg, '/candle_ros2_node/zero_md80s')
        self.setModeMd80Service = self.master.create_client(SetModeMd80s, '/candle_ros2_node/set_mode_md80s')
        self.enableMd80Service = self.master.create_client(GenericMd80Msg, '/candle_ros2_node/enable_md80s')
        self.disableMd80Service = self.master.create_client(GenericMd80Msg, '/candle_ros2_node/disable_md80s')

        # Connect GUI elements to functions (impedance-only)
        if hasattr(self.ui, 'enable_motors'):
            self.ui.enable_motors.toggled.connect(self.enable_left_motor)
        if hasattr(self.ui, 'enable_right_motor'):
            self.ui.enable_right_motor.toggled.connect(self.enable_right_motor)
        if hasattr(self.ui, 'add_motors_btn'):
            self.ui.add_motors_btn.pressed.connect(self.add_motors)
        if hasattr(self.ui, 'control_set'):
            self.ui.control_set.pressed.connect(self.set_control)

        # Impedance tab widgets
        try:
            if hasattr(self.ui, 'left_hip_manual_enable'):
                self.ui.left_hip_manual_enable.toggled.connect(self.handle_impedance_manual_enable_left)
            if hasattr(self.ui, 'left_hip_manual_box'):
                self.ui.left_hip_manual_box.valueChanged.connect(self.impedance_position_setpoint)
            if hasattr(self.ui, 'right_hip_manual_enable'):
                self.ui.right_hip_manual_enable.toggled.connect(self.handle_impedance_manual_enable_right)
            if hasattr(self.ui, 'right_hip_manual_box'):
                self.ui.right_hip_manual_box.valueChanged.connect(self.impedance_position_setpoint)

            if hasattr(self.ui, 'left_hip_sine_enable'):
                self.ui.left_hip_sine_enable.toggled.connect(self.start_impedance_sinusoidal_trajectory_left)
            if hasattr(self.ui, 'right_hip_sine_enable'):
                self.ui.right_hip_sine_enable.toggled.connect(self.start_impedance_sinusoidal_trajectory_right)

            if hasattr(self.ui, 'left_start_traj_btn'):
                self.ui.left_start_traj_btn.pressed.connect(self.start_impedance_trajectory_left)
            if hasattr(self.ui, 'left_stop_traj_btn'):
                self.ui.left_stop_traj_btn.pressed.connect(self.stop_impedance_trajectory_left)
            if hasattr(self.ui, 'right_start_traj_btn'):
                self.ui.right_start_traj_btn.pressed.connect(self.start_impedance_trajectory_right)
            if hasattr(self.ui, 'right_stop_traj_btn'):
                self.ui.right_stop_traj_btn.pressed.connect(self.stop_impedance_trajectory_right)

            if hasattr(self.ui, 'update_impedance_parameters_btn_izq'):
                self.ui.update_impedance_parameters_btn_izq.pressed.connect(self.update_impedance_parameters_left)
            if hasattr(self.ui, 'update_impedance_parameters_btn_der'):
                self.ui.update_impedance_parameters_btn_der.pressed.connect(self.update_impedance_parameters_right)

            self.master.get_logger().info('🔗 Connected impedance UI signals')
        except Exception as e:
            self.master.get_logger().warn(f'⚠️ Could not connect impedance UI signals: {e}')

        # Setup real-time position graph widget (promoted widget: graph/widget)
        self._setup_position_graph()
        graph_combo = getattr(self.ui, 'graph_display_combo', None)
        if graph_combo is None:
            graph_combo = getattr(self.ui, 'comboBox', None)
        if graph_combo is not None:
            graph_combo.currentIndexChanged.connect(self._on_graph_display_mode_changed)
            self._set_graph_display_mode_from_index(graph_combo.currentIndex())

        # Setup real-time torque/velocity graph (promoted widget: graph_tv)
        self._setup_torque_velocity_graph()

        # Timer for updating GUI
        self.setpoint_timer = 250
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_display)
        self.timer.start(self.setpoint_timer)

    # Update GUI with latest sensor/motor data
    def update_display(self):
        if hasattr(self.ui, 'imu_izq_v_x'):
            self.ui.imu_izq_v_x.setText(f"{self.imu_izq.linear_acceleration.x:.1f}")
        if hasattr(self.ui, 'imu_izq_v_y'):
            self.ui.imu_izq_v_y.setText(f"{self.imu_izq.linear_acceleration.y:.1f}")
        if hasattr(self.ui, 'imu_izq_v_z'):
            self.ui.imu_izq_v_z.setText(f"{self.imu_izq.linear_acceleration.z:.1f}")

        if hasattr(self.ui, 'imu_der_v_x'):
            self.ui.imu_der_v_x.setText(f"{self.imu_der.linear_acceleration.x:.1f}")
        if hasattr(self.ui, 'imu_der_v_y'):
            self.ui.imu_der_v_y.setText(f"{self.imu_der.linear_acceleration.y:.1f}")
        if hasattr(self.ui, 'imu_der_v_z'):
            self.ui.imu_der_v_z.setText(f"{self.imu_der.linear_acceleration.z:.1f}")

        if hasattr(self.ui, 'cadera_izq_voltaje_data'):
            self.ui.cadera_izq_voltaje_data.setText(f"{self.motor_cadera_izq_data.voltage:.1f}")
        if hasattr(self.ui, 'cadera_izq_corriente_data'):
            self.ui.cadera_izq_corriente_data.setText(f"{self.motor_cadera_izq_data.current:.1f}")
        if hasattr(self.ui, 'cadera_izq_torque_data'):
            self.ui.cadera_izq_torque_data.setText(f"{self.motor_cadera_izq_data.torque:.1f}")
        if hasattr(self.ui, 'cadera_izq_posicion_data'):
            self.ui.cadera_izq_posicion_data.setText(f"{self.motor_cadera_izq_data.position:.1f}")
        if hasattr(self.ui, 'cadera_izq_velocidad_data'):
            self.ui.cadera_izq_velocidad_data.setText(f"{self.motor_cadera_izq_data.velocity:.1f}")

        if hasattr(self.ui, 'cadera_der_voltaje_data'):
            self.ui.cadera_der_voltaje_data.setText(f"{self.motor_cadera_der_data.voltage:.1f}")
        if hasattr(self.ui, 'cadera_der_corriente_data'):
            self.ui.cadera_der_corriente_data.setText(f"{self.motor_cadera_der_data.current:.1f}")
        if hasattr(self.ui, 'cadera_der_torque_data'):
            self.ui.cadera_der_torque_data.setText(f"{self.motor_cadera_der_data.torque:.1f}")
        if hasattr(self.ui, 'cadera_der_posicion_data'):
            self.ui.cadera_der_posicion_data.setText(f"{self.motor_cadera_der_data.position:.1f}")
        if hasattr(self.ui, 'cadera_der_velocidad_data'):
            self.ui.cadera_der_velocidad_data.setText(f"{self.motor_cadera_der_data.velocity:.1f}")

        self._update_position_graph()
        self._update_torque_velocity_graph()

    def _setup_position_graph(self):
        self._graph_widget = getattr(self.ui, 'graph', None)
        if self._graph_widget is None:
            self._graph_widget = getattr(self.ui, 'widget', None)

        if self._graph_widget is None:
            self.master.get_logger().warn('⚠️ Graph widget not found (expected objectName: graph/widget)')
            return

        if hasattr(self._graph_widget, 'update_data'):
            show_left, show_right = self._get_graph_visibility()
            self._graph_widget.update_data([], [], [], show_left=show_left, show_right=show_right)

    def _set_graph_display_mode_from_index(self, index):
        if index == 0:
            self._graph_display_mode = 'none'
        elif index == 1:
            self._graph_display_mode = 'both'
        elif index == 2:
            self._graph_display_mode = 'left'
        elif index == 3:
            self._graph_display_mode = 'right'
        else:
            self._graph_display_mode = 'none'

    def _on_graph_display_mode_changed(self, index):
        self._set_graph_display_mode_from_index(index)
        self._update_position_graph()

    def _get_graph_visibility(self):
        if self._graph_display_mode == 'left':
            return True, False
        if self._graph_display_mode == 'right':
            return False, True
        if self._graph_display_mode == 'none':
            return False, False
        return True, True

    def _update_position_graph(self):
        if self._graph_widget is None or not hasattr(self._graph_widget, 'update_data'):
            return

        elapsed_time = time.monotonic() - self._graph_start_time

        left_pos = self.motor_cadera_izq_data.position
        right_pos = self.motor_cadera_der_data.position
        show_left, show_right = self._get_graph_visibility()

        self._graph_time.append(elapsed_time)
        self._graph_left_pos.append(left_pos)
        self._graph_right_pos.append(right_pos)

        min_time = elapsed_time - self._graph_window_seconds
        while self._graph_time and self._graph_time[0] < min_time:
            self._graph_time.popleft()
            self._graph_left_pos.popleft()
            self._graph_right_pos.popleft()

        self._graph_widget.update_data(
            list(self._graph_time),
            list(self._graph_left_pos),
            list(self._graph_right_pos),
            show_left=show_left,
            show_right=show_right,
        )

    def _setup_torque_velocity_graph(self):
        self._graph_tv_widget = getattr(self.ui, 'graph_tv', None)
        if self._graph_tv_widget is None:
            return

        if not hasattr(self._graph_tv_widget, 'ax'):
            return

        self._graph_tv_plot = TorqueVelocityGraph(self._graph_tv_widget)

        for attr in [
            'graph_tv_left_torque_cb',
            'graph_tv_right_torque_cb',
            'graph_tv_left_velocity_cb',
            'graph_tv_right_velocity_cb',
        ]:
            checkbox = getattr(self.ui, attr, None)
            if checkbox is not None:
                checkbox.toggled.connect(self._update_torque_velocity_graph)

        self._update_torque_velocity_graph()

    def _get_torque_velocity_visibility(self):
        show_left_torque = getattr(self.ui, 'graph_tv_left_torque_cb', None)
        show_right_torque = getattr(self.ui, 'graph_tv_right_torque_cb', None)
        show_left_velocity = getattr(self.ui, 'graph_tv_left_velocity_cb', None)
        show_right_velocity = getattr(self.ui, 'graph_tv_right_velocity_cb', None)

        return (
            show_left_torque.isChecked() if show_left_torque is not None else True,
            show_right_torque.isChecked() if show_right_torque is not None else True,
            show_left_velocity.isChecked() if show_left_velocity is not None else True,
            show_right_velocity.isChecked() if show_right_velocity is not None else True,
        )

    def _update_torque_velocity_graph(self):
        if self._graph_tv_plot is None:
            return

        elapsed_time = time.monotonic() - self._graph_start_time

        show_lt, show_rt, show_lv, show_rv = self._get_torque_velocity_visibility()

        self._graph_tv_plot.update_data(
            elapsed_time=elapsed_time,
            left_torque=self.motor_cadera_izq_data.torque,
            right_torque=self.motor_cadera_der_data.torque,
            left_velocity=self.motor_cadera_izq_data.velocity,
            right_velocity=self.motor_cadera_der_data.velocity,
            show_left_torque=show_lt,
            show_right_torque=show_rt,
            show_left_velocity=show_lv,
            show_right_velocity=show_rv,
            window_seconds=self._graph_window_seconds,
        )

    # Callbacks
    def imu_izq_callback(self, msg):
        self.imu_izq.linear_acceleration.x = msg.linear_acceleration.x
        self.imu_izq.linear_acceleration.y = msg.linear_acceleration.y
        self.imu_izq.linear_acceleration.z = msg.linear_acceleration.z

    def imu_der_callback(self, msg):
        self.imu_der.linear_acceleration.x = msg.linear_acceleration.x
        self.imu_der.linear_acceleration.y = msg.linear_acceleration.y
        self.imu_der.linear_acceleration.z = msg.linear_acceleration.z

    def motor_izq_callback(self, msg):
        try:
            if len(msg.name) >= 2:
                self.motor_cadera_izq_data.position = self.rad2deg(msg.position[0]) if len(msg.position) > 0 else 0.0
                self.motor_cadera_izq_data.velocity = msg.velocity[0] if len(msg.velocity) > 0 else 0.0
                self.motor_cadera_izq_data.torque = msg.effort[0] if len(msg.effort) > 0 else 0.0

                self.motor_cadera_der_data.position = self.rad2deg(msg.position[1]) if len(msg.position) > 1 else 0.0
                self.motor_cadera_der_data.velocity = msg.velocity[1] if len(msg.velocity) > 1 else 0.0
                self.motor_cadera_der_data.torque = msg.effort[1] if len(msg.effort) > 1 else 0.0
        except Exception as e:
            self.master.get_logger().error(f'Error processing motor data: {str(e)}')

    # Degree/radian conversion
    def deg2rad(self, deg):
        return deg * 2 * math.pi / 360

    def rad2deg(self, rad):
        return rad * 360 / (2 * math.pi)

    # ------------------------- MOTOR CONTROL -------------------------

    def add_motors(self):
        print()
        print("=" * 60)
        self.master.get_logger().info('🔧 DEBUG: add_motors function called')
        print("ADD MD80")

        if not self.addMd80Service.service_is_ready():
            self.master.get_logger().warn('⚠️ WARNING: AddMd80s service is not available!')
            return

        add_request = AddMd80s.Request()
        add_request.drive_ids = drives_can_id
        self.master.get_logger().info(f'🔧 DEBUG: Requesting to add motors with CAN IDs: {drives_can_id}')
        add_future = self.addMd80Service.call_async(add_request)
        add_future.add_done_callback(self.add_motors_callback)

    def add_motors_callback(self, future):
        try:
            response = future.result()
            self.master.get_logger().info(f'Add motors service response: {response}')

            if hasattr(response, 'drives_success'):
                self.master.get_logger().info('🔍 Motor detection results:')
                for drive_id, success in zip(drives_can_id, response.drives_success):
                    status = "✅ DETECTED" if success else "❌ NOT FOUND"
                    self.master.get_logger().info(f'  Motor ID {drive_id}: {status}')

                successful_motors = sum(response.drives_success)
                self.master.get_logger().info(f'📊 Total: {successful_motors}/{len(drives_can_id)} motors detected')

                if hasattr(self.ui, 'enable_motors'):
                    self.ui.enable_motors.setText(f"Enable Left Motor: id=  {left_hip_can_id}")
                if hasattr(self.ui, 'enable_right_motor'):
                    self.ui.enable_right_motor.setText("Enable Right Motor: id=  ")

                if any(response.drives_success):
                    self._motors_added = True
                    if successful_motors == len(drives_can_id):
                        self.master.get_logger().info("🎯 All motors successfully added")
                    else:
                        self.master.get_logger().warn(f"⚠️ Only {successful_motors} out of {len(drives_can_id)} motors added")
                else:
                    self._motors_added = False
                    self.master.get_logger().warn("❌ No motors were successfully added")

                print("=" * 60)
            else:
                self._motors_added = False
                self.master.get_logger().warn("No drives_success field in response")

        except Exception as e:
            self.master.get_logger().error(f'Add motors service call failed: {e}')
            self._motors_added = False

    def enable_left_motor(self, state):
        print()
        print("=" * 60)
        self.master.get_logger().info(f'🔧 DEBUG: enable_left_motor called with state: {state}')

        if state:
            self._desired_enabled_motors.add(left_hip_can_id)
            self.master.get_logger().info(f'🔄 Adding left motor {left_hip_can_id} to desired set')
        else:
            self._desired_enabled_motors.discard(left_hip_can_id)
            self.master.get_logger().info(f'🔄 Removing left motor {left_hip_can_id} from desired set')

        left_checked = hasattr(self.ui, 'enable_motors') and self.ui.enable_motors.isChecked()
        right_checked = hasattr(self.ui, 'enable_right_motor') and self.ui.enable_right_motor.isChecked()

        self.master.get_logger().info(f'🔧 DEBUG: Current desired motors: {self._desired_enabled_motors}')
        self.master.get_logger().info(f'🔧 DEBUG: Checkbox states - Left: {left_checked}, Right: {right_checked}')

        if not hasattr(self, '_motors_added') or not self._motors_added:
            self.master.get_logger().warn('⚠️ Cannot enable/disable left motor: Motor not added yet!')
            if hasattr(self.ui, 'enable_motors'):
                self.ui.enable_motors.blockSignals(True)
                self.ui.enable_motors.setChecked(False)
                self.ui.enable_motors.blockSignals(False)
            return

        # Automatically set IMPEDANCE mode if not already set
        if self._current_control_mode is None and state:
            self.master.get_logger().info('🎛️ Auto-setting IMPEDANCE mode before enabling motor')
            self.set_control()

        print(60 * "=")
        self._apply_motor_state()

    def enable_right_motor(self, state):
        self.master.get_logger().info(f'🔧 DEBUG: enable_right_motor called with state: {state}')

        if right_hip_can_id not in drives_can_id:
            self.master.get_logger().warn('⚠️ Right motor not configured in drives_can_id; ignoring enable request')
            if hasattr(self.ui, 'enable_right_motor'):
                self.ui.enable_right_motor.blockSignals(True)
                self.ui.enable_right_motor.setChecked(False)
                self.ui.enable_right_motor.blockSignals(False)
            return

        if state:
            self._desired_enabled_motors.add(right_hip_can_id)
            self.master.get_logger().info(f'🔄 Adding right motor {right_hip_can_id} to desired set')
        else:
            self._desired_enabled_motors.discard(right_hip_can_id)
            self.master.get_logger().info(f'🔄 Removing right motor {right_hip_can_id} from desired set')

        left_checked = hasattr(self.ui, 'enable_motors') and self.ui.enable_motors.isChecked()
        right_checked = hasattr(self.ui, 'enable_right_motor') and self.ui.enable_right_motor.isChecked()

        self.master.get_logger().info(f'🔧 DEBUG: Current desired motors: {self._desired_enabled_motors}')
        self.master.get_logger().info(f'🔧 DEBUG: Checkbox states - Left: {left_checked}, Right: {right_checked}')

        if not hasattr(self, '_motors_added') or not self._motors_added:
            self.master.get_logger().warn('⚠️ Cannot enable/disable right motor: Motor not added yet!')
            if right_widget is not None:
                right_widget.blockSignals(True)
                right_widget.setChecked(False)
                right_widget.blockSignals(False)
            return

        # Automatically set IMPEDANCE mode if not already set
        if self._current_control_mode is None and state:
            self.master.get_logger().info('🎛️ Auto-setting IMPEDANCE mode before enabling motor')
            self.set_control()

        print(60 * "=")
        self._apply_motor_state()

    def _apply_motor_state(self):
        self.master.get_logger().info(f'🎯 Applying motor state - Desired: {sorted(self._desired_enabled_motors)}')

        currently_enabled = set()
        if self._left_motor_enabled:
            currently_enabled.add(left_hip_can_id)
        if self._right_motor_enabled:
            currently_enabled.add(right_hip_can_id)

        self.master.get_logger().info(f'🔄 Current state: {sorted(currently_enabled)}, Target state: {sorted(self._desired_enabled_motors)}')

        if currently_enabled == self._desired_enabled_motors:
            self.master.get_logger().info('✅ No motor state changes needed - all motors already in correct state')
            return

        all_motor_ids = set(drives_can_id)
        print()
        self.master.get_logger().info('🔄 Resetting state: Disabling all motors')
        disable_request = GenericMd80Msg.Request()
        disable_request.drive_ids = sorted(list(all_motor_ids))
        disable_future = self.disableMd80Service.call_async(disable_request)
        disable_future.add_done_callback(lambda f: self._handle_reset_then_enable(f))

    def _handle_reset_then_enable(self, disable_future):
        try:
            disable_response = disable_future.result()
            self.master.get_logger().info(f'Reset disable response: {disable_response}')

            if self._desired_enabled_motors:
                desired_motors = sorted(list(self._desired_enabled_motors))
                self.master.get_logger().info(f'🔄 Enabling desired motors: {desired_motors}')

                if self._current_control_mode is None:
                    self.master.get_logger().warn('⚠️ No control mode set! Enabling motors anyway, but you should select a control mode.')
                    self.master.get_logger().warn('🎛️ Recommended: Select a control mode (Position, Velocity, or Impedance) for proper operation.')
                else:
                    self.master.get_logger().info(f'✅ Control mode is {self._current_control_mode}, proceeding with motor enable...')

                enable_request = GenericMd80Msg.Request()
                enable_request.drive_ids = desired_motors
                enable_future = self.enableMd80Service.call_async(enable_request)
                enable_future.add_done_callback(lambda f: self._apply_motor_state_callback(f, desired_motors, True))
            else:
                self.master.get_logger().info('✅ No motors to enable after reset')
                self._left_motor_enabled = False
                self._right_motor_enabled = False
                self._motors_enabled = False
                self._update_motor_state_tracking()

        except Exception as e:
            self.master.get_logger().error(f'Reset disable failed: {e}')
            self._desired_enabled_motors.clear()
            if hasattr(self.ui, 'enable_motors'):
                self.ui.enable_motors.blockSignals(True)
                self.ui.enable_motors.setChecked(False)
                self.ui.enable_motors.blockSignals(False)
            if hasattr(self.ui, 'enable_right_motor'):
                self.ui.enable_right_motor.blockSignals(True)
                self.ui.enable_right_motor.setChecked(False)
                self.ui.enable_right_motor.blockSignals(False)
            self._update_motor_state_tracking()

    def _apply_motor_state_callback(self, future, motor_ids, enabling):
        try:
            response = future.result()
            action = "enable" if enabling else "disable"
            print()
            self.master.get_logger().info(f'Apply motor state ({action}) response: {response}')
            
            # Check if the response has drives_success field
            if hasattr(response, 'drives_success'):
                for i, (motor_id, success) in enumerate(zip(motor_ids, response.drives_success)):
                    if success:
                        self.master.get_logger().info(f'✅ Motor {motor_id}: Successfully {action}d')
                    else:
                        self.master.get_logger().warn(f'❌ Motor {motor_id}: Failed to {action}')
                        # If enabling failed, remove from desired set
                        if enabling:
                            self._desired_enabled_motors.discard(motor_id)
                            # Update corresponding checkbox
                            if motor_id == left_hip_can_id:
                                if hasattr(self.ui, 'enable_motors'):
                                    self.ui.enable_motors.blockSignals(True)
                                    self.ui.enable_motors.setChecked(False)
                                    self.ui.enable_motors.blockSignals(False)
                            elif motor_id == right_hip_can_id:
                                if hasattr(self.ui, 'enable_right_motor'):
                                    self.ui.enable_right_motor.blockSignals(True)
                                    self.ui.enable_right_motor.setChecked(False)
                                    self.ui.enable_right_motor.blockSignals(False)
            
            # Update motor state tracking
            self._update_motor_state_tracking()
                
        except Exception as e:
            self.master.get_logger().error(f'Apply motor state service call failed: {e}')
            # On exception, only reset the motors that were part of this operation
            for motor_id in motor_ids:
                if enabling:
                    # Remove from desired set only the motors that failed to enable
                    self._desired_enabled_motors.discard(motor_id)
                    # Update corresponding checkbox only for the failed motor
                    if motor_id == left_hip_can_id:
                        if hasattr(self.ui, 'enable_motors'):
                            self.ui.enable_motors.blockSignals(True)
                            self.ui.enable_motors.setChecked(False)
                            self.ui.enable_motors.blockSignals(False)
                    elif motor_id == right_hip_can_id:
                        if hasattr(self.ui, 'enable_right_motor'):
                            self.ui.enable_right_motor.blockSignals(True)
                            self.ui.enable_right_motor.setChecked(False)
                            self.ui.enable_right_motor.blockSignals(False)
            self._update_motor_state_tracking()

    def _update_motor_state_tracking(self):
        self._left_motor_enabled = left_hip_can_id in self._desired_enabled_motors
        self._right_motor_enabled = right_hip_can_id in self._desired_enabled_motors
        self._motors_enabled = self._left_motor_enabled or self._right_motor_enabled
        
        self.master.get_logger().info(f'🔧 Updated motor states - Left: {self._left_motor_enabled}, Right: {self._right_motor_enabled}, Overall: {self._motors_enabled}')
        self.master.get_logger().info(f'🔧 Desired set: {sorted(self._desired_enabled_motors)}')
        
        # Check if UI widgets exist before accessing them
        enable_left_checked = False
        enable_right_checked = False
        if hasattr(self.ui, 'enable_motors'):
            enable_left_checked = self.ui.enable_motors.isChecked()
        if hasattr(self.ui, 'enable_right_motor'):
            enable_right_checked = self.ui.enable_right_motor.isChecked()
        
        self.master.get_logger().info(f'🔧 Checkbox states - Left: {enable_left_checked}, Right: {enable_right_checked}')
        print(60*"=")
        print()

    # ------------------------- IMPEDANCE HELPERS -------------------------
    def _verify_impedance_parameters_for_both_motors(self, kp_values, kd_values, offset_values):
        num_motors = len(drives_can_id)
        self.master.get_logger().info(f'🔍 VERIFICATION: Impedance parameters for {num_motors} motor(s):')
        motor_names = ['Left', 'Right']
        for i in range(num_motors):
            motor_name = motor_names[i] if i < len(motor_names) else f'Motor{i}'
            self.master.get_logger().info(f'   {motor_name} (CAN ID {drives_can_id[i]}): kP={kp_values[i]:.3f}, kD={kd_values[i]:.3f}, offset={offset_values[i]:.3f}')
        self.master.get_logger().info(f'   Drive IDs: {drives_can_id}')

    def _send_initial_impedance_offset(self):
        try:
            offset_values = self.config.get_offset_torque()
            num_configured_motors = len(drives_can_id)
            msg = MotionCommand()
            msg.drive_ids = [int(d) for d in drives_can_id]
            msg.target_position = [0.0] * num_configured_motors
            msg.target_velocity = [0.0] * num_configured_motors
            msg.target_torque = [float(offset_values[i]) for i in range(num_configured_motors)]

            self.master.get_logger().info(f'🔧 Sending initial impedance offset torque for {num_configured_motors} motor(s): {[f"{offset_values[i]:.2f}" for i in range(num_configured_motors)]} Nm')
            self.master.get_logger().info(f'🔧 Drive IDs: {list(msg.drive_ids)}')
            self.master.publisher_motion_commands.publish(msg)
            print("=" * 60)

        except Exception as e:
            self.master.get_logger().error(f'❌ Failed to send initial impedance offset: {str(e)}')

    # Reads kp/kd/offset from UI spinboxes and publishes impedance parameters (manual button).
    def update_impedance_parameters(self):
        self.master.get_logger().info("🔧 Manual impedance parameter update triggered")
        try:
            kp_defaults = self.config.get_kp_gains()
            kd_defaults = self.config.get_kd_gains()
            max_output_defaults = self.config.get_max_output_torque()
            offset_defaults = self.config.get_offset_torque()

            num_configured_motors = len(drives_can_id)

            kp_values = []
            kd_values = []
            offset_values = []

            for i in range(num_configured_motors):
                if i == 0:
                    kp_values.append(float(self.ui.left_hip_kp_box.value()) if hasattr(self.ui, 'left_hip_kp_box') else kp_defaults[0])
                    kd_values.append(float(self.ui.left_hip_kd_box.value()) if hasattr(self.ui, 'left_hip_kd_box') else kd_defaults[0])
                    offset_values.append(float(self.ui.left_hip_offset_box.value()) if hasattr(self.ui, 'left_hip_offset_box') else offset_defaults[0])
                elif i == 1:
                    kp_values.append(float(self.ui.right_hip_kp_box.value()) if hasattr(self.ui, 'right_hip_kp_box') else kp_defaults[1])
                    kd_values.append(float(self.ui.right_hip_kd_box.value()) if hasattr(self.ui, 'right_hip_kd_box') else kd_defaults[1])
                    offset_values.append(float(self.ui.right_hip_offset_box.value()) if hasattr(self.ui, 'right_hip_offset_box') else offset_defaults[1])

            msg = ImpedanceCommand()
            msg.drive_ids = [int(d) for d in drives_can_id]
            msg.kp = [float(kp) for kp in kp_values]
            msg.kd = [float(kd) for kd in kd_values]
            msg.max_output = [float(max_output_defaults[i]) for i in range(num_configured_motors)]

            motor_names = ['Left', 'Right']
            for i in range(num_configured_motors):
                motor_name = motor_names[i] if i < len(motor_names) else f'Motor{i}'
                self.master.get_logger().info(f'🔧 Publishing impedance params: {motor_name}(kp={kp_values[i]:.2f}, kd={kd_values[i]:.3f})')
            self.master.get_logger().info(f'🔧 Max outputs: {[f"{max_output_defaults[i]:.1f}" for i in range(num_configured_motors)]} Nm')

            self.master.publisher_impedance_params.publish(msg)

            motion_msg = MotionCommand()
            motion_msg.drive_ids = [int(d) for d in drives_can_id]
            motion_msg.target_position = [0.0] * num_configured_motors
            motion_msg.target_velocity = [0.0] * num_configured_motors
            motion_msg.target_torque = [float(offset) for offset in offset_values]

            for i in range(num_configured_motors):
                motor_name = motor_names[i] if i < len(motor_names) else f'Motor{i}'
                self.master.get_logger().info(f'🔧 Publishing offset torque: {motor_name}={offset_values[i]:.2f} Nm')

            self.master.publisher_motion_commands.publish(motion_msg)

        except Exception as e:
            self.master.get_logger().error(f'❌ Manual impedance parameter update failed: {str(e)}')

    def update_impedance_parameters_left(self):
        print()
        print("=" * 60)
        self.master.get_logger().info("🔧 Left motor impedance parameter update triggered (button)")
        try:
            if not hasattr(self, '_current_control_mode') or self._current_control_mode != "IMPEDANCE":
                self.master.get_logger().warn('⚠️ Not in IMPEDANCE mode; skipping left motor parameter update')
                return

            if not (hasattr(self, '_left_motor_enabled') and self._left_motor_enabled):
                self.master.get_logger().warn('⚠️ Left motor not enabled; skipping parameter update')
                return

            kp_defaults = self.config.get_kp_gains()
            kd_defaults = self.config.get_kd_gains()
            max_output_defaults = self.config.get_max_output_torque()
            offset_defaults = self.config.get_offset_torque()

            left_kp = float(self.ui.left_hip_kp_box.value()) if hasattr(self.ui, 'left_hip_kp_box') else kp_defaults[0]
            left_kd = float(self.ui.left_hip_kd_box.value()) if hasattr(self.ui, 'left_hip_kd_box') else kd_defaults[0]
            left_offset = float(self.ui.left_hip_offset_box.value()) if hasattr(self.ui, 'left_hip_offset_box') else offset_defaults[0]
            left_max_output = float(max_output_defaults[0])

            msg = ImpedanceCommand()
            msg.drive_ids = [left_hip_can_id]
            msg.kp = [left_kp]
            msg.kd = [left_kd]
            msg.max_output = [left_max_output]

            self.master.publisher_impedance_params.publish(msg)
            self.master.get_logger().info(f'🔧 Left motor impedance: kp={left_kp:.2f}, kd={left_kd:.3f}, max_output={left_max_output:.1f}')

            motion_msg = MotionCommand()
            motion_msg.drive_ids = [left_hip_can_id]
            motion_msg.target_position = [0.0]
            motion_msg.target_velocity = [0.0]
            motion_msg.target_torque = [left_offset]

            self.master.publisher_motion_commands.publish(motion_msg)
            self.master.get_logger().info(f'🔧 Left motor offset torque: {left_offset:.2f} Nm')
            print(60 * "=")
            print()

        except Exception as e:
            self.master.get_logger().error(f'❌ Left motor impedance parameter update failed: {str(e)}')

    def update_impedance_parameters_right(self):
        print()
        print("=" * 60)
        self.master.get_logger().info("🔧 Right motor impedance parameter update triggered (button)")
        try:
            if not hasattr(self, '_current_control_mode') or self._current_control_mode != "IMPEDANCE":
                self.master.get_logger().warn('⚠️ Not in IMPEDANCE mode; skipping right motor parameter update')
                return

            if not (hasattr(self, '_right_motor_enabled') and self._right_motor_enabled):
                self.master.get_logger().warn('⚠️ Right motor not enabled; skipping parameter update')
                return

            kp_defaults = self.config.get_kp_gains()
            kd_defaults = self.config.get_kd_gains()
            max_output_defaults = self.config.get_max_output_torque()
            offset_defaults = self.config.get_offset_torque()

            offset_index = min(1, len(kp_defaults) - 1)
            right_kp = float(self.ui.right_hip_kp_box.value()) if hasattr(self.ui, 'right_hip_kp_box') else kp_defaults[offset_index]
            right_kd = float(self.ui.right_hip_kd_box.value()) if hasattr(self.ui, 'right_hip_kd_box') else kd_defaults[offset_index]
            right_offset = float(self.ui.right_hip_offset_box.value()) if hasattr(self.ui, 'right_hip_offset_box') else offset_defaults[offset_index]
            right_max_output = float(max_output_defaults[offset_index])

            msg = ImpedanceCommand()
            msg.drive_ids = [right_hip_can_id]
            msg.kp = [right_kp]
            msg.kd = [right_kd]
            msg.max_output = [right_max_output]

            self.master.publisher_impedance_params.publish(msg)
            self.master.get_logger().info(f'🔧 Right motor impedance: kp={right_kp:.2f}, kd={right_kd:.3f}, max_output={right_max_output:.1f}')

            motion_msg = MotionCommand()
            motion_msg.drive_ids = [right_hip_can_id]
            motion_msg.target_position = [0.0]
            motion_msg.target_velocity = [0.0]
            motion_msg.target_torque = [right_offset]

            self.master.publisher_motion_commands.publish(motion_msg)
            self.master.get_logger().info(f'🔧 Right motor offset torque: {right_offset:.2f} Nm')
            print(60 * "=")
            print()

        except Exception as e:
            self.master.get_logger().error(f'❌ Right motor impedance parameter update failed: {str(e)}')

    def update_impedance_parameters_realtime(self):
        current_mode = getattr(self, '_current_control_mode', 'None')

        if not hasattr(self, '_current_control_mode') or self._current_control_mode != "IMPEDANCE":
            self.master.get_logger().warn(f'⚠️ Impedance parameter update skipped - not in IMPEDANCE mode (current: {current_mode})')
            return

        left_enabled = hasattr(self, '_left_motor_enabled') and self._left_motor_enabled
        right_enabled = hasattr(self, '_right_motor_enabled') and self._right_motor_enabled

        if not left_enabled and not right_enabled:
            self.master.get_logger().warn('⚠️ Impedance parameter update skipped - no motors enabled')
            return

        try:
            kp_defaults = self.config.get_kp_gains()
            kd_defaults = self.config.get_kd_gains()
            max_output_defaults = self.config.get_max_output_torque()
            offset_defaults = self.config.get_offset_torque()

            enabled_motor_ids = []
            kp_values = []
            kd_values = []
            max_output_values = []
            offset_values = []
            motor_descriptions = []

            if left_enabled:
                enabled_motor_ids.append(left_hip_can_id)
                left_kp = float(self.ui.left_hip_kp_box.value()) if hasattr(self.ui, 'left_hip_kp_box') else kp_defaults[0]
                left_kd = float(self.ui.left_hip_kd_box.value()) if hasattr(self.ui, 'left_hip_kd_box') else kd_defaults[0]
                left_offset = float(self.ui.left_hip_offset_box.value()) if hasattr(self.ui, 'left_hip_offset_box') else offset_defaults[0]

                kp_values.append(float(left_kp))
                kd_values.append(float(left_kd))
                max_output_values.append(float(max_output_defaults[0]))
                offset_values.append(float(left_offset))
                motor_descriptions.append(f'Left(kp={left_kp:.2f}, kd={left_kd:.3f}, offset={left_offset:.2f})')

            if right_enabled:
                enabled_motor_ids.append(right_hip_can_id)
                right_kp = float(self.ui.right_hip_kp_box.value()) if hasattr(self.ui, 'right_hip_kp_box') else kp_defaults[1]
                right_kd = float(self.ui.right_hip_kd_box.value()) if hasattr(self.ui, 'right_hip_kd_box') else kd_defaults[1]
                right_offset = float(self.ui.right_hip_offset_box.value()) if hasattr(self.ui, 'right_hip_offset_box') else offset_defaults[1]

                kp_values.append(float(right_kp))
                kd_values.append(float(right_kd))
                max_output_values.append(float(max_output_defaults[1]))
                offset_values.append(float(right_offset))
                motor_descriptions.append(f'Right(kp={right_kp:.2f}, kd={right_kd:.3f}, offset={right_offset:.2f})')

            if enabled_motor_ids:
                msg = ImpedanceCommand()
                msg.drive_ids = [int(d) for d in enabled_motor_ids]
                msg.kp = kp_values
                msg.kd = kd_values
                msg.max_output = max_output_values
                self.master.publisher_impedance_params.publish(msg)

                motion_msg = MotionCommand()
                motion_msg.drive_ids = [int(d) for d in enabled_motor_ids]
                motion_msg.target_position = [0.0] * len(enabled_motor_ids)
                motion_msg.target_velocity = [0.0] * len(enabled_motor_ids)
                motion_msg.target_torque = offset_values
                self.master.publisher_motion_commands.publish(motion_msg)

                motor_status = ', '.join(motor_descriptions)
                self.master.get_logger().info(f'🔧 Real-time impedance parameters sent to enabled motors: {motor_status}')

        except Exception as e:
            self.master.get_logger().error(f'❌ Real-time impedance parameter update failed: {str(e)}')
            self.master.get_logger().error(f'Error updating impedance parameters: {str(e)}')

    # ------------------------- IMPEDANCE POSITION REFERENCE -------------------------
    def _check_impedance_mode_conflict(self, motor_side):
        if motor_side == 'left':
            manual_enabled = hasattr(self.ui, 'left_hip_manual_enable') and self.ui.left_hip_manual_enable.isChecked()
            sine_enabled = hasattr(self.ui, 'left_hip_sine_enable') and self.ui.left_hip_sine_enable.isChecked()
            if manual_enabled and sine_enabled:
                self.master.get_logger().error('❌ [IMP-LEFT] CONFLICT: Both manual_enable and sine_enable are active! Disable one. Motor stopped.')
                return True
        elif motor_side == 'right':
            manual_enabled = hasattr(self.ui, 'right_hip_manual_enable') and self.ui.right_hip_manual_enable.isChecked()
            sine_enabled = hasattr(self.ui, 'right_hip_sine_enable') and self.ui.right_hip_sine_enable.isChecked()
            if manual_enabled and sine_enabled:
                self.master.get_logger().error('❌ [IMP-RIGHT] CONFLICT: Both manual_enable and sine_enable are active! Disable one. Motor stopped.')
                return True
        return False

    def start_impedance_trajectory_left(self):
        if not hasattr(self, '_motors_added') or not self._motors_added:
            self.master.get_logger().warn('⚠️ [IMP-LEFT] Cannot start trajectory: Motors not added yet!')
            return

        if not hasattr(self, '_left_motor_enabled') or not self._left_motor_enabled:
            self.master.get_logger().warn('⚠️ [IMP-LEFT] Cannot start trajectory: Left motor not enabled!')
            return

        if not hasattr(self, '_current_control_mode') or self._current_control_mode != "IMPEDANCE":
            self.master.get_logger().warn('⚠️ [IMP-LEFT] Cannot start trajectory: Not in IMPEDANCE mode!')
            return

        sine_enabled = hasattr(self.ui, 'left_hip_sine_enable') and self.ui.left_hip_sine_enable.isChecked()

        if sine_enabled:
            if self._check_impedance_mode_conflict('left'):
                self.master.get_logger().error('❌ [IMP-LEFT] Cannot start: manual and sine both enabled!')
                return

            self._impedance_trajectory_left_active = True
            self._impedance_sinusoidal_left_active = True
            self._impedance_sinusoidal_time = 0.0
            self.start_impedance_sinusoidal_timer()
            self.master.get_logger().info('✅ [IMP-LEFT] Sinusoidal trajectory started with button')
        else:
            self._impedance_trajectory_left_active = True
            self.master.get_logger().info('✅ [IMP-LEFT] Manual trajectory started - motor will respond to manual control')

    def stop_impedance_trajectory_left(self):
        self._impedance_trajectory_left_active = False
        self._impedance_sinusoidal_left_active = False

        if hasattr(self.ui, 'left_hip_sine_enable'):
            self.ui.left_hip_sine_enable.blockSignals(True)
            self.ui.left_hip_sine_enable.setChecked(False)
            self.ui.left_hip_sine_enable.blockSignals(False)

        self.check_stop_impedance_sinusoidal_timer()
        self.master.get_logger().info('🛑 [IMP-LEFT] Trajectory stopped - ready to restart')

    def start_impedance_trajectory_right(self):
        if not hasattr(self, '_motors_added') or not self._motors_added:
            self.master.get_logger().warn('⚠️ [IMP-RIGHT] Cannot start trajectory: Motors not added yet!')
            return

        if not hasattr(self, '_right_motor_enabled') or not self._right_motor_enabled:
            self.master.get_logger().warn('⚠️ [IMP-RIGHT] Cannot start trajectory: Right motor not enabled!')
            return

        if not hasattr(self, '_current_control_mode') or self._current_control_mode != "IMPEDANCE":
            self.master.get_logger().warn('⚠️ [IMP-RIGHT] Cannot start trajectory: Not in IMPEDANCE mode!')
            return

        sine_enabled = hasattr(self.ui, 'right_hip_sine_enable') and self.ui.right_hip_sine_enable.isChecked()

        if sine_enabled:
            if self._check_impedance_mode_conflict('right'):
                self.master.get_logger().error('❌ [IMP-RIGHT] Cannot start: manual and sine both enabled!')
                return

            self._impedance_trajectory_right_active = True
            self._impedance_sinusoidal_right_active = True
            self._impedance_sinusoidal_time = 0.0
            self.start_impedance_sinusoidal_timer()
            self.master.get_logger().info('✅ [IMP-RIGHT] Sinusoidal trajectory started with button')
        else:
            self._impedance_trajectory_right_active = True
            self.master.get_logger().info('✅ [IMP-RIGHT] Manual trajectory started - motor will respond to manual control')

    def stop_impedance_trajectory_right(self):
        self._impedance_trajectory_right_active = False
        self._impedance_sinusoidal_right_active = False

        if hasattr(self.ui, 'right_hip_sine_enable'):
            self.ui.right_hip_sine_enable.blockSignals(True)
            self.ui.right_hip_sine_enable.setChecked(False)
            self.ui.right_hip_sine_enable.blockSignals(False)

        self.check_stop_impedance_sinusoidal_timer()
        self.master.get_logger().info('🛑 [IMP-RIGHT] Trajectory stopped - ready to restart')

    def handle_impedance_manual_enable_left(self, checked):
        print()
        print(60 * "=")
        if checked:
            self._impedance_trajectory_left_active = True
            self.master.get_logger().info('✅ [IMP-LEFT] Manual mode activated - trajectory flag enabled automatically')
        else:
            self._impedance_trajectory_left_active = False
            self.master.get_logger().info('🛑 [IMP-LEFT] Manual mode deactivated - trajectory flag disabled')

        self.impedance_position_setpoint()

    def handle_impedance_manual_enable_right(self, checked):
        print()
        print(60 * "=")
        if checked:
            self._impedance_trajectory_right_active = True
            self.master.get_logger().info('✅ [IMP-RIGHT] Manual mode activated - trajectory flag enabled automatically')
        else:
            self._impedance_trajectory_right_active = False
            self.master.get_logger().info('🛑 [IMP-RIGHT] Manual mode deactivated - trajectory flag disabled')

        self.impedance_position_setpoint()

    def impedance_position_setpoint(self):
        self.master.get_logger().info('[IMP] impedance_position_setpoint called')
        current_mode = getattr(self, '_current_control_mode', None)
        motors_added = getattr(self, '_motors_added', False)
        motors_enabled = getattr(self, '_motors_enabled', False)
        self.master.get_logger().info(f'[IMP] mode={current_mode}, motors_added={motors_added}, motors_enabled={motors_enabled}')

        if not hasattr(self, '_current_control_mode') or self._current_control_mode != "IMPEDANCE":
            self.master.get_logger().info('[IMP] Skipping: not in IMPEDANCE mode')
            return

        if not hasattr(self, '_motors_added') or not self._motors_added:
            self.master.get_logger().warn('⚠️ Cannot send impedance position reference: Motors not added yet!')
            return

        if not hasattr(self, '_motors_enabled') or not self._motors_enabled:
            self.master.get_logger().warn('⚠️ Cannot send impedance position reference: Motors not enabled!')
            return

        if getattr(self, '_impedance_sinusoidal_left_active', False) or getattr(self, '_impedance_sinusoidal_right_active', False):
            self.master.get_logger().info('[IMP] Skipping: impedance sinusoidal active')
            return

        try:
            setpoint_left = 0.0
            setpoint_right = 0.0

            left_manual_enabled = hasattr(self.ui, 'left_hip_manual_enable') and self.ui.left_hip_manual_enable.isChecked()
            if left_manual_enabled:
                if hasattr(self.ui, 'left_hip_manual_box'):
                    setpoint_left = self.ui.left_hip_manual_box.value()

            right_manual_enabled = hasattr(self.ui, 'right_hip_manual_enable') and self.ui.right_hip_manual_enable.isChecked()
            if right_manual_enabled:
                if hasattr(self.ui, 'right_hip_manual_box'):
                    setpoint_right = self.ui.right_hip_manual_box.value()

            left_conflict = self._check_impedance_mode_conflict('left')
            right_conflict = self._check_impedance_mode_conflict('right')

            left_trajectory_active = getattr(self, '_impedance_trajectory_left_active', False)
            right_trajectory_active = getattr(self, '_impedance_trajectory_right_active', False)

            left_to_send = left_manual_enabled and left_trajectory_active and not left_conflict
            right_to_send = right_manual_enabled and right_trajectory_active and not right_conflict

            self.master.get_logger().info(f'[IMP] Manual enabled: left={left_manual_enabled}, right={right_manual_enabled}; Trajectory active: left={left_trajectory_active}, right={right_trajectory_active}; To send: left={left_to_send}, right={right_to_send}')

            if left_to_send or right_to_send:
                self.master.get_logger().info(f'[IMP] Sending impedance reference: left={setpoint_left}, right={setpoint_right}')
                self.send_impedance_position_reference(setpoint_left, setpoint_right)
            else:
                self.master.get_logger().info('[IMP] No valid motor commands to send (trajectory not active or conflict detected)')
                print("=" * 60)
                print()

        except Exception as e:
            self.master.get_logger().error(f'Error sending impedance position reference: {str(e)}')

    def send_impedance_position_reference(self, setpoint_left, setpoint_right):
        enabled_drives = []
        target_positions = []
        target_torques = []
        setpoint_info = []

        offset_defaults = self.config.get_offset_torque()

        left_enabled = hasattr(self, '_left_motor_enabled') and self._left_motor_enabled
        right_enabled = hasattr(self, '_right_motor_enabled') and self._right_motor_enabled

        motor_index = 0
        if left_enabled:
            enabled_drives.append(left_hip_can_id)
            target_positions.append(float(self.deg2rad(setpoint_left)))
            left_offset = float(self.ui.left_hip_offset_box.value()) if hasattr(self.ui, 'left_hip_offset_box') else offset_defaults[0]
            target_torques.append(float(left_offset))
            setpoint_info.append(f'Left={setpoint_left}°({self.deg2rad(setpoint_left):.3f}rad, offset={left_offset:.2f}Nm)')
            motor_index += 1

        if right_enabled:
            enabled_drives.append(right_hip_can_id)
            target_positions.append(float(self.deg2rad(setpoint_right)))
            offset_index = min(motor_index, len(offset_defaults) - 1)
            right_offset = float(self.ui.right_hip_offset_box.value()) if hasattr(self.ui, 'right_hip_offset_box') else offset_defaults[offset_index]
            target_torques.append(float(right_offset))
            setpoint_info.append(f'Right={setpoint_right}°({self.deg2rad(setpoint_right):.3f}rad, offset={right_offset:.2f}Nm)')

        if not enabled_drives:
            self.master.get_logger().warn('⚠️ Cannot send impedance position reference: No motors are enabled!')
            return

        msg = MotionCommand()
        msg.drive_ids = enabled_drives
        msg.target_position = target_positions
        msg.target_velocity = [0.0] * len(enabled_drives)
        msg.target_torque = target_torques

        self.master.get_logger().info(f'🏗️ Sending impedance position reference to enabled motors: {", ".join(setpoint_info)}')
        self.master.get_logger().info(f'🏗️ Drive IDs: {msg.drive_ids}, Positions: {msg.target_position}, Torques: {msg.target_torque}')
        self.master.publisher_motion_commands.publish(msg)

    # ------------------------- IMPEDANCE SINE -------------------------
    def start_impedance_sinusoidal_trajectory_left(self, checked):
        if checked:
            if not hasattr(self, '_motors_added') or not self._motors_added:
                self.master.get_logger().warn('⚠️ Cannot enable sinusoidal: Motors not added yet!')
                if hasattr(self.ui, 'left_hip_sine_enable'):
                    self.ui.left_hip_sine_enable.setChecked(False)
                return

            if not hasattr(self, '_left_motor_enabled') or not self._left_motor_enabled:
                self.master.get_logger().warn('⚠️ Cannot enable sinusoidal: Left motor not enabled!')
                if hasattr(self.ui, 'left_hip_sine_enable'):
                    self.ui.left_hip_sine_enable.setChecked(False)
                return

            if not hasattr(self, '_current_control_mode') or self._current_control_mode != "IMPEDANCE":
                self.master.get_logger().warn('⚠️ Cannot enable sinusoidal: Not in Impedance Control mode!')
                if hasattr(self.ui, 'left_hip_sine_enable'):
                    self.ui.left_hip_sine_enable.setChecked(False)
                return

            if self._check_impedance_mode_conflict('left'):
                if hasattr(self.ui, 'left_hip_sine_enable'):
                    self.ui.left_hip_sine_enable.setChecked(False)
                return

            print()
            print(60 * "=")
            self.master.get_logger().info('🌊 [IMP-LEFT] Sinusoidal mode enabled - press start button to begin trajectory')
        else:
            if self._impedance_sinusoidal_left_active:
                self._impedance_sinusoidal_left_active = False
                self.check_stop_impedance_sinusoidal_timer()
            self.master.get_logger().info('🛑 [IMP-LEFT] Sinusoidal mode disabled')

    def start_impedance_sinusoidal_trajectory_right(self, checked):
        if checked:
            if not hasattr(self, '_motors_added') or not self._motors_added:
                self.master.get_logger().warn('⚠️ Cannot enable sinusoidal: Motors not added yet!')
                if hasattr(self.ui, 'right_hip_sine_enable'):
                    self.ui.right_hip_sine_enable.setChecked(False)
                return

            if not hasattr(self, '_right_motor_enabled') or not self._right_motor_enabled:
                self.master.get_logger().warn('⚠️ Cannot enable sinusoidal: Right motor not enabled!')
                if hasattr(self.ui, 'right_hip_sine_enable'):
                    self.ui.right_hip_sine_enable.setChecked(False)
                return

            if not hasattr(self, '_current_control_mode') or self._current_control_mode != "IMPEDANCE":
                self.master.get_logger().warn('⚠️ Cannot enable sinusoidal: Not in Impedance Control mode!')
                if hasattr(self.ui, 'right_hip_sine_enable'):
                    self.ui.right_hip_sine_enable.setChecked(False)
                return

            if self._check_impedance_mode_conflict('right'):
                if hasattr(self.ui, 'right_hip_sine_enable'):
                    self.ui.right_hip_sine_enable.setChecked(False)
                return

            print()
            print(60 * "=")
            self.master.get_logger().info('🌊 [IMP-RIGHT] Sinusoidal mode enabled - press start button to begin trajectory')
        else:
            if self._impedance_sinusoidal_right_active:
                self._impedance_sinusoidal_right_active = False
                self.check_stop_impedance_sinusoidal_timer()
            self.master.get_logger().info('🛑 [IMP-RIGHT] Sinusoidal mode disabled')

    def start_impedance_sinusoidal_timer(self):
        if self._impedance_sinusoidal_timer is None:
            self._impedance_sinusoidal_timer = QTimer()
            self._impedance_sinusoidal_timer.timeout.connect(self.update_impedance_sinusoidal_trajectory)
            self._impedance_sinusoidal_timer.start(50)
            self.master.get_logger().info('⏰ Impedance sinusoidal timer started (20 Hz)')

    def check_stop_impedance_sinusoidal_timer(self):
        if not self._impedance_sinusoidal_left_active and not self._impedance_sinusoidal_right_active:
            if self._impedance_sinusoidal_timer is not None:
                self._impedance_sinusoidal_timer.stop()
                self._impedance_sinusoidal_timer = None
                self.master.get_logger().info('⏰ Impedance sinusoidal timer stopped')

    def update_impedance_sinusoidal_trajectory(self):
        self._impedance_sinusoidal_time += 0.05

        setpoint_left = 0.0
        setpoint_right = 0.0

        if self._impedance_sinusoidal_left_active and not getattr(self, '_impedance_trajectory_left_active', False):
            self.master.get_logger().warn('⚠️ [IMP-LEFT] Stopping sine: trajectory stopped via button')
            self._impedance_sinusoidal_left_active = False
            if hasattr(self.ui, 'left_hip_sine_enable'):
                self.ui.left_hip_sine_enable.setChecked(False)
            self.check_stop_impedance_sinusoidal_timer()

        if self._impedance_sinusoidal_right_active and not getattr(self, '_impedance_trajectory_right_active', False):
            self.master.get_logger().warn('⚠️ [IMP-RIGHT] Stopping sine: trajectory stopped via button')
            self._impedance_sinusoidal_right_active = False
            if hasattr(self.ui, 'right_hip_sine_enable'):
                self.ui.right_hip_sine_enable.setChecked(False)
            self.check_stop_impedance_sinusoidal_timer()

        if self._impedance_sinusoidal_left_active and self._check_impedance_mode_conflict('left'):
            self.master.get_logger().error('❌ [IMP-LEFT] Conflict detected during sine, stopping')
            self._impedance_sinusoidal_left_active = False
            if hasattr(self.ui, 'left_hip_sine_enable'):
                self.ui.left_hip_sine_enable.setChecked(False)
            self.check_stop_impedance_sinusoidal_timer()

        if self._impedance_sinusoidal_right_active and self._check_impedance_mode_conflict('right'):
            self.master.get_logger().error('❌ [IMP-RIGHT] Conflict detected during sine, stopping')
            self._impedance_sinusoidal_right_active = False
            if hasattr(self.ui, 'right_hip_sine_enable'):
                self.ui.right_hip_sine_enable.setChecked(False)
            self.check_stop_impedance_sinusoidal_timer()

        if self._impedance_sinusoidal_left_active:
            try:
                freq = self.ui.left_hip_sine_freq.value() if hasattr(self.ui, 'left_hip_sine_freq') else 0.5
                amplitude = self.ui.left_hip_sine_amp.value() if hasattr(self.ui, 'left_hip_sine_amp') else 10.0
                offset = self.ui.left_hip_sine_offset.value() if hasattr(self.ui, 'left_hip_sine_offset') else 0.0
                cycles = self.ui.left_hip_sine_cycles.value() if hasattr(self.ui, 'left_hip_sine_cycles') else 0

                if self._impedance_sinusoidal_time <= 0.1:
                    self.master.get_logger().info(f'📊 Left motor sine parameters: freq={freq}Hz, amp={amplitude}°, offset={offset}°, cycles={cycles}')

                if cycles > 0 and self._impedance_sinusoidal_time * freq >= cycles:
                    self._impedance_sinusoidal_left_active = False
                    if hasattr(self.ui, 'left_hip_sine_enable'):
                        self.ui.left_hip_sine_enable.setChecked(False)
                    self.check_stop_impedance_sinusoidal_timer()
                    self.master.get_logger().info('✅ Left motor sinusoidal trajectory completed')
                    print(60 * "=")
                    print()
                    return

                setpoint_left = offset + amplitude * math.sin(2 * math.pi * freq * self._impedance_sinusoidal_time)
            except Exception as e:
                self.master.get_logger().warn(f'⚠️ Error reading left sine parameters, using defaults: {e}')
                setpoint_left = 10.0 * math.sin(2 * math.pi * 0.5 * self._impedance_sinusoidal_time)

        if self._impedance_sinusoidal_right_active:
            try:
                freq = self.ui.right_hip_sine_freq.value() if hasattr(self.ui, 'right_hip_sine_freq') else 0.5
                amplitude = self.ui.right_hip_sine_amp.value() if hasattr(self.ui, 'right_hip_sine_amp') else 10.0
                offset = self.ui.right_hip_sine_offset.value() if hasattr(self.ui, 'right_hip_sine_offset') else 0.0
                cycles = self.ui.right_hip_sine_cycles.value() if hasattr(self.ui, 'right_hip_sine_cycles') else 0

                if self._impedance_sinusoidal_time <= 0.1:
                    self.master.get_logger().info(f'📊 Right motor sine parameters: freq={freq}Hz, amp={amplitude}°, offset={offset}°, cycles={cycles}')

                if cycles > 0 and self._impedance_sinusoidal_time * freq >= cycles:
                    self._impedance_sinusoidal_right_active = False
                    if hasattr(self.ui, 'right_hip_sine_enable'):
                        self.ui.right_hip_sine_enable.setChecked(False)
                    self.check_stop_impedance_sinusoidal_timer()
                    self.master.get_logger().info('✅ Right motor sinusoidal trajectory completed')
                    print(60 * "=")
                    print()
                    return

                setpoint_right = offset + amplitude * math.sin(2 * math.pi * freq * self._impedance_sinusoidal_time)
            except Exception as e:
                self.master.get_logger().warn(f'⚠️ Error reading right sine parameters, using defaults: {e}')
                setpoint_right = 10.0 * math.sin(2 * math.pi * 0.5 * self._impedance_sinusoidal_time)

        left_traj_active = self._impedance_sinusoidal_left_active and getattr(self, '_impedance_trajectory_left_active', False)
        right_traj_active = self._impedance_sinusoidal_right_active and getattr(self, '_impedance_trajectory_right_active', False)

        if left_traj_active or right_traj_active:
            if int(self._impedance_sinusoidal_time * 20) % 20 == 0:
                self.master.get_logger().info(f'🌊 Impedance sinusoidal trajectory t={self._impedance_sinusoidal_time:.1f}s: Left={setpoint_left:.1f}°, Right={setpoint_right:.1f}°')

            self.send_impedance_position_reference(setpoint_left, setpoint_right)

    # Send impedance trajectory command for left motor only
    def send_impedance_trajectory_left(self, state):
        if state:
            if hasattr(self.ui, 'left_hip_sine_enable'):
                self.ui.left_hip_sine_enable.setChecked(True)
        else:
            if hasattr(self.ui, 'left_hip_sine_enable'):
                self.ui.left_hip_sine_enable.setChecked(False)

        action = "Started" if state else "Stopped"
        self.master.get_logger().info(f'🎯 {action} impedance trajectory for LEFT motor')

    # Send impedance trajectory command for right motor only
    def send_impedance_trajectory_right(self, state):
        if state:
            if hasattr(self.ui, 'right_hip_sine_enable'):
                self.ui.right_hip_sine_enable.setChecked(True)
        else:
            if hasattr(self.ui, 'right_hip_sine_enable'):
                self.ui.right_hip_sine_enable.setChecked(False)

        action = "Started" if state else "Stopped"
        self.master.get_logger().info(f'🎯 {action} impedance trajectory for RIGHT motor')

    def set_control(self, force_mode=None, callback=None):
        """Set control mode. For impedance-only GUI, IMPEDANCE is the only option."""
        # For impedance-only GUI, always use IMPEDANCE mode
        control = 4.0
        mode_ = "IMPEDANCE"

        print()
        print("="*60)
        self.master.get_logger().info('🎛️ Impedance mode selected - parameters will be published directly')

        # Set the control mode early for immediate parameter updates
        self._current_control_mode = mode_

        request = SetModeMd80s.Request()
        request.drive_ids = drives_can_id
        request.mode = [mode_] * len(drives_can_id)
        self.master.get_logger().info(f'🔧 Sending SetMode request: drive_ids={request.drive_ids}, mode={request.mode}')
        future = self.setModeMd80Service.call_async(request)

        # Use custom callback if provided, otherwise use default
        if callback:
            future.add_done_callback(lambda f: self._set_control_with_callback(f, mode_, callback))
        else:
            future.add_done_callback(lambda f: self.set_control_callback(f, mode_))

    def _set_control_with_callback(self, future, mode_, custom_callback):
        try:
            # First, handle the normal set_control response
            self.set_control_callback(future, mode_)
            # Then call the custom callback
            custom_callback()
        except Exception as e:
            self.master.get_logger().error(f'Custom callback execution failed: {e}')

    def set_control_callback(self, future, mode_):
        try:
            response = future.result()
            self.master.get_logger().info(f'SetMode response: {response}')

            if hasattr(response, 'drives_success'):
                for i, (drive_id, success) in enumerate(zip(drives_can_id, response.drives_success)):
                    status = "✅ SUCCESS" if success else "❌ FAILED"
                    self.master.get_logger().info(f'  Drive {drive_id}: Mode set to {mode_} - {status}')

                if all(response.drives_success):
                    self.master.get_logger().info(f'🎯 All motors successfully set to {mode_} mode')
                else:
                    self.master.get_logger().warn(f'⚠️ Some motors failed to set mode')
            else:
                self.master.get_logger().warn('No drives_success field in response')

            print("="*60)
            print()

        except Exception as e:
            self.master.get_logger().error(f'SetMode service call failed: {e}')
            self._current_control_mode = None


def spin_ros_node(node):
    rclpy.spin(node)


def main(args=None):
    rclpy.init(args=args)
    ros_node = GuiNode()
    executor = MultiThreadedExecutor()
    try:
        app = QApplication(sys.argv)
        ventana = MiVentana(ros_node)
        ventana.show()
        ros_spin_thread = threading.Thread(target=spin_ros_node, args=(ros_node,))
        ros_spin_thread.start()
        sys.exit(app.exec())
    finally:
        ros_node.get_logger().info("Shutting down")
        ros_node.destroy_node()
        executor.shutdown()


if __name__ == '__main__':
    main()
