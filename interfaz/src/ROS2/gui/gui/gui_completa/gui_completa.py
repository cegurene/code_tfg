"""
This script implements a PyQt6-based GUI for controlling and monitoring an assistive exoskeleton using ROS 2.
It provides real-time visualization and control of motor data, allows the user to
send commands to the exoskeleton motors, update control parameters, and record data using rosbag.
The GUI interacts with ROS 2 topics and services to send commands and receive feedback from the exoskeleton hardware.

Main variables and classes:
- left_hip_can_id, right_hip_can_id: CAN IDs for the left and right hip motors.
- drives_can_id: List of CAN IDs for all motors.
- num_motors: Number of motors in the system.
- MotorData: Structure to hold motor telemetry (voltage, current, position, velocity, torque).
- MplCanvas: Matplotlib canvas for future plotting of data.
- GuiNode: ROS 2 node that manages publishers for sending motor control commands.
- MiVentana: Main GUI window class, handles user interaction, ROS 2 subscriptions, service clients, and data display.
    - self.motor_cadera_izq_data, self.motor_cadera_der_data: Store telemetry for left and right hip motors.
    - self.rosbag_process: Manages rosbag recording process.
    - self.ui: The Qt Designer generated UI class instance.
- spin_ros_node: Function to spin the ROS node in a separate thread.
- main: Entry point for the application, initializes ROS 2, the GUI, and starts the event loop.
"""

import rclpy
import sys
import threading
import subprocess
import time
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from PyQt6.QtWidgets import QApplication, QMainWindow, QFileDialog
from PyQt6.QtCore import QTimer, QProcess, QProcessEnvironment
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
import os
from datetime import datetime
import math
import ctypes
from collections import deque

from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, ReliabilityPolicy

# ROS2 Bag and Serialization imports
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

# Candle ROS2 service and message imports
from candle_ros2.srv import AddMd80s, GenericMd80Msg, SetLimitsMd80, SetModeMd80s
from candle_ros2.msg import ImpedanceCommand, MotionCommand, Pid, PositionPidCommand, VelocityPidCommand

# Note: No longer using ImpedanceParameters service - direct publishing instead
from std_srvs.srv import SetBool

# Import centralized configuration management (parent package)
from ..impedance_config_py import ImpedanceConfig, find_config_file
from ..mplGraphTV import TorqueVelocityGraph

from .gui_completa_data import Ui_MainWindow  # Qt Designer generated class for the new UI

from pathlib import Path

import re
from PyQt6.QtGui import QTextCursor

# CAN IDs for motors (updated to match actual hardware)
left_hip_can_id = 348
right_hip_can_id = 349
drives_can_id = [left_hip_can_id, right_hip_can_id]  # drives_can_id = [left_hip_can_id, right_hip_can_id]
num_motors = len(drives_can_id)

# Structure to hold motor data
class MotorData(ctypes.Structure):
    _fields_ = [
        ("voltage", ctypes.c_float),
        ("current", ctypes.c_float),
        ("position", ctypes.c_float),
        ("velocity", ctypes.c_float),
        ("torque", ctypes.c_float)
    ]

# Matplotlib canvas for plotting (not used in this code, but left for future use)
class MplCanvas(FigureCanvas):
    def __init__(self, parent=None, width=5, height=4, dpi=100):
        import matplotlib.pyplot as plt
        self.fig, self.axes = plt.subplots(figsize=(width, height), dpi=dpi)
        super().__init__(self.fig)
        self.axes.set_xlim(0, 10)
        self.axes.set_ylim(-100, 100)
        self.line_pos, = self.axes.plot([], [], label="Position", color='r')
        self.line_setpoint, = self.axes.plot([], [], label="Setpoint", color='b')
        self.axes.legend()
        self.draw()

# ROS2 Node for GUI
class GuiNode(Node):
    def __init__(self):
        super().__init__("simple_gui_node")
        # Publishers for motor control (both motors)
        self.publisher_position_pid_params = self.create_publisher(PositionPidCommand, '/md80/position_pid_command', 10)
        self.publisher_velocity_pid_params = self.create_publisher(VelocityPidCommand, '/md80/velocity_pid_command', 10)
        self.publisher_impedance_params = self.create_publisher(ImpedanceCommand, '/md80/impedance_command', 10)
        self.publisher_motion_commands = self.create_publisher(MotionCommand, '/md80/motion_command', 10)
        print("Node and publishers created.")

# Main GUI Window
class MiVentana(QMainWindow):
    def __init__(self, node_):
        super().__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.master = node_

        # ========================================================================
        # LOAD CONFIGURATION
        # ========================================================================
        
        # Try to find and load configuration file
        config_path = find_config_file()
        if config_path:
            self.config = ImpedanceConfig(config_path)
            self.master.get_logger().info(f"📁 GUI: Configuration loaded from {config_path}")
        else:
            self.config = ImpedanceConfig()
            self.master.get_logger().info("📋 GUI: Using default configuration values")
        
        # Print configuration for debugging
        if hasattr(self.config, 'print_configuration'):
            print("\n" + "="*60)
            print("GUI CONFIGURATION LOADED")
            print("="*60)
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
        
        # Track motor addition status
        self._motors_added = False
        self._left_motor_enabled = False
        self._right_motor_enabled = False
        self._motors_enabled = False  # True only when both motors are enabled
        self._current_control_mode = None
        
        # Set-based motor enable tracking
        self._desired_enabled_motors = set()  # Set of motor IDs we want enabled
        
        # Sinusoidal trajectory variables
        self._sinusoidal_left_active = False
        self._sinusoidal_right_active = False
        self._sinusoidal_time = 0.0
        self._sinusoidal_timer = None
        
        # Impedance control variables
        self._impedance_sinusoidal_left_active = False
        self._impedance_sinusoidal_right_active = False
        self._impedance_sinusoidal_time_left = 0.0
        self._impedance_sinusoidal_time_right = 0.0
        self._impedance_sinusoidal_timer = None
        
        # Impedance trajectory start/stop control (via buttons)
        self._impedance_trajectory_left_active = False  # True only when start_traj_btn is pressed
        self._impedance_trajectory_right_active = False  # True only when start_traj_btn is pressed

        # Archive replay variables
        self._replay_timer = None
        self._replay_data = []
        self._replay_index = 0

        # Set QoS profile for subscriptions
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT

        # ROS2 Subscribers
        self.motor_izq_subscriber = self.master.create_subscription(JointState, 'md80/joint_states', self.motor_izq_callback, qos_profile)
        self.motor_der_subscriber = self.master.create_subscription(JointState, 'md80/joint_states', self.motor_der_callback, qos_profile)  

        # ROS2 Service Clients
        self.addMd80Service = self.master.create_client(AddMd80s, '/candle_ros2_node/add_md80s')
        self.zeroMd80Service = self.master.create_client(GenericMd80Msg, '/candle_ros2_node/zero_md80s')
        self.setModeMd80Service = self.master.create_client(SetModeMd80s, '/candle_ros2_node/set_mode_md80s')
        self.enableMd80Service = self.master.create_client(GenericMd80Msg, '/candle_ros2_node/enable_md80s')
        self.disableMd80Service = self.master.create_client(GenericMd80Msg, '/candle_ros2_node/disable_md80s')
        # Note: Impedance parameters are now published directly to /md80/impedance_command (no service needed)

        # Connect GUI elements to functions
        #self.ui.enable_motors.toggled.connect(self.enable_left_motor)  # Changed to left motor control
        #self.ui.enable_right_motor.toggled.connect(self.enable_right_motor)  # New right motor control
        self.ui.botonHabilitarMotores.clicked.connect(self.enable_selected_motors_button_clicked)

        # TEST: Add button for simultaneous enable (temporary)
        # self.ui.some_button.pressed.connect(self.enable_both_motors_test)  # Uncomment when we add button
        self.ui.control_set.pressed.connect(self.set_control)
        if hasattr(self.ui, 'cadera_der_setpoint_slider_manual'):
            self.ui.cadera_der_setpoint_slider_manual.valueChanged.connect(self.slider_setpoint)
        self.ui.carpeta_guardado_btn.pressed.connect(self.select_folder_to_save)
        self.ui.save_play_button.pressed.connect(self.start_recording)
        self.ui.save_stop_button.pressed.connect(self.stop_recording)
        self.ui.add_motors_btn.pressed.connect(self.add_motors)

        # Connect MIMo
        self.ui.buttonLaunchSimMimo.pressed.connect(self.launch_sim_mimo)

        # Connect training
        self.ui.button_empezar_entrenamiento.pressed.connect(self.launch_training)
        self.ui.texto_salida_entrenamiento.setReadOnly(True)
        
        # Impedance control layout is now created in Qt Designer
        # Connect impedance tab widgets (created in Qt Designer)
        try:
            # Manual reference controls
            if hasattr(self.ui, 'left_hip_manual_enable'):
                self.ui.left_hip_manual_enable.toggled.connect(self.handle_impedance_manual_enable_left)
            if hasattr(self.ui, 'left_hip_manual_box'):
                self.ui.left_hip_manual_box.valueChanged.connect(self.impedance_position_setpoint)
            if hasattr(self.ui, 'right_hip_manual_enable'):
                self.ui.right_hip_manual_enable.toggled.connect(self.handle_impedance_manual_enable_right)
            if hasattr(self.ui, 'right_hip_manual_box'):
                self.ui.right_hip_manual_box.valueChanged.connect(self.impedance_position_setpoint)

            # Sinusoidal trajectory enable (impedance tab)
            if hasattr(self.ui, 'left_hip_sine_enable'):
                self.ui.left_hip_sine_enable.toggled.connect(self.start_impedance_sinusoidal_trajectory_left)
            if hasattr(self.ui, 'right_hip_sine_enable'):
                self.ui.right_hip_sine_enable.toggled.connect(self.start_impedance_sinusoidal_trajectory_right)
            
            # Start/Stop trajectory buttons (impedance tab)
            if hasattr(self.ui, 'left_start_traj_btn'):
                self.ui.left_start_traj_btn.pressed.connect(self.start_impedance_trajectory_left)
            if hasattr(self.ui, 'left_stop_traj_btn'):
                self.ui.left_stop_traj_btn.pressed.connect(self.stop_impedance_trajectory_left)
            if hasattr(self.ui, 'right_start_traj_btn'):
                self.ui.right_start_traj_btn.pressed.connect(self.start_impedance_trajectory_right)
            if hasattr(self.ui, 'right_stop_traj_btn'):
                self.ui.right_stop_traj_btn.pressed.connect(self.stop_impedance_trajectory_right)

            # Manual update buttons for impedance parameters (kp/kd/offset)
            # Left motor
            if hasattr(self.ui, 'update_impedance_parameters_btn_izq'):
                self.ui.update_impedance_parameters_btn_izq.pressed.connect(self.update_impedance_parameters_left)
            # Right motor
            if hasattr(self.ui, 'update_impedance_parameters_btn_der'):
                self.ui.update_impedance_parameters_btn_der.pressed.connect(self.update_impedance_parameters_right)

            self.master.get_logger().info('🔗 Connected impedance UI signals')
        except Exception as e:
            self.master.get_logger().warn(f'⚠️ Could not connect impedance UI signals: {e}')
        
        # Hide the "Sit To Stand" tab (tab_3)
        try:
            sit_to_stand_index = self.ui.tabWidget.indexOf(self.ui.tab_3)
            if sit_to_stand_index >= 0:
                self.ui.tabWidget.removeTab(sit_to_stand_index)
                self.master.get_logger().info('🚫 Removed "Sit To Stand" tab from GUI')
        except AttributeError:
            pass  # Tab doesn't exist
            
        # Hide torque control radio button (if it exists)
        try:
            if hasattr(self.ui, 'torque_control'):
                self.ui.torque_control.setVisible(False)
                self.master.get_logger().info('🚫 Hidden torque control option')
        except AttributeError:
            pass  # Torque control doesn't exist

        # Hide position control radio button (if it exists)
        try:
            if hasattr(self.ui, 'position_control'):
                self.ui.position_control.setVisible(False)
                self.master.get_logger().info('🚫 Hidden position control option')
        except AttributeError:
            pass  # Position control doesn't exist

        # Hide velocity control radio button (if it exists)
        try:
            if hasattr(self.ui, 'velocity_control'):
                self.ui.velocity_control.setVisible(False)
                self.master.get_logger().info('🚫 Hidden velocity control option')
        except AttributeError:
            pass  # Velocity control doesn't exist

        # Setup real-time position graph widget (promoted widget: graph/widget)
        self._setup_position_graph()
        if hasattr(self.ui, 'graph_display_combo'):
            self.ui.graph_display_combo.currentIndexChanged.connect(self._on_graph_display_mode_changed)
            self._set_graph_display_mode_from_index(self.ui.graph_display_combo.currentIndex())

        # Setup real-time torque/velocity graph (promoted widget: graph_tv)
        self._setup_torque_velocity_graph()


        # Timer for updating GUI
        self.setpoint_timer = 250
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(self.setpoint_timer)

        # For rosbag recording
        self.rosbag_process = None

        print("GUI initialized and ready.")
        print("="*60)

    # Update GUI with latest sensor/motor data
    def update_plot(self):
        # Motor data - Left motor
        self.ui.cadera_izq_voltaje_data.setText(f"{self.motor_cadera_izq_data.voltage:.1f}")
        self.ui.cadera_izq_corriente_data.setText(f"{self.motor_cadera_izq_data.current:.1f}")
        self.ui.cadera_izq_torque_data.setText(f"{self.motor_cadera_izq_data.torque:.1f}")
        self.ui.cadera_izq_posicion_data.setText(f"{self.motor_cadera_izq_data.position:.1f}")
        self.ui.cadera_izq_velocidad_data.setText(f"{self.motor_cadera_izq_data.velocity:.1f}")
        
        # Motor data - Right motor (if UI elements exist)
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

    # Processes JointState: updates one/two motor telemetry and logs significant position changes.
    def motor_izq_callback(self, msg):
        try:
            # Log the raw message structure for debugging (commented out to reduce spam)
            # self.master.get_logger().info(f'📊 Received joint_states: pos={len(msg.position)}, vel={len(msg.velocity)}, eff={len(msg.effort)}, names={msg.name}')

            if len(msg.position) >= 2 and len(msg.velocity) >= 2 and len(msg.effort) >= 2:
                # Store previous positions for change detection
                prev_left_pos = getattr(self.motor_cadera_izq_data, 'position', 0.0)
                prev_right_pos = getattr(self.motor_cadera_der_data, 'position', 0.0)
                
                # Left motor (index 0)
                self.motor_cadera_izq_data.voltage = msg.effort[0]  # Using effort as voltage for now
                self.motor_cadera_izq_data.current = msg.effort[0]  # Using effort as current for now
                self.motor_cadera_izq_data.position = self.rad2deg(msg.position[0])
                self.motor_cadera_izq_data.velocity = msg.velocity[0]
                self.motor_cadera_izq_data.torque = msg.effort[0]
                
                # Right motor (index 1)  
                self.motor_cadera_der_data.voltage = msg.effort[1]
                self.motor_cadera_der_data.current = msg.effort[1]
                self.motor_cadera_der_data.position = self.rad2deg(msg.position[1])
                self.motor_cadera_der_data.velocity = msg.velocity[1]
                self.motor_cadera_der_data.torque = msg.effort[1]
                
                # Log significant position changes (more than 1 degree)
                left_change = abs(self.motor_cadera_izq_data.position - prev_left_pos)
                right_change = abs(self.motor_cadera_der_data.position - prev_right_pos)
                
                if left_change > 1.0 or right_change > 1.0:
                    self.master.get_logger().info(f'🔄 Motor positions: Left={self.motor_cadera_izq_data.position:.1f}° (Δ{left_change:.1f}°), Right={self.motor_cadera_der_data.position:.1f}° (Δ{right_change:.1f}°)')
                    self.master.get_logger().info(f'🔄 Motor velocities: Left={self.motor_cadera_izq_data.velocity:.2f}rad/s, Right={self.motor_cadera_der_data.velocity:.2f}rad/s')
                    
            elif len(msg.position) == 1 and len(msg.velocity) == 1 and len(msg.effort) == 1:
                # Only one motor detected
                #self.master.get_logger().warn(f'⚠️ Only one motor detected in joint_states! Name: {msg.name[0] if msg.name else "unknown"}')
                
                # Assume it's the left motor for now
                prev_left_pos = getattr(self.motor_cadera_izq_data, 'position', 0.0)
                self.motor_cadera_izq_data.voltage = msg.effort[0]
                self.motor_cadera_izq_data.current = msg.effort[0]
                self.motor_cadera_izq_data.position = self.rad2deg(msg.position[0])
                self.motor_cadera_izq_data.velocity = msg.velocity[0]
                self.motor_cadera_izq_data.torque = msg.effort[0]
                
                left_change = abs(self.motor_cadera_izq_data.position - prev_left_pos)
                if left_change > 1.0:
                    self.master.get_logger().info(f'🔄 Single motor position: {self.motor_cadera_izq_data.position:.1f}° (Δ{left_change:.1f}°)')
                    
            elif len(msg.position) > 0 or len(msg.velocity) > 0 or len(msg.effort) > 0:
                # Partial data received
                if hasattr(self, '_motors_added') and self._motors_added:
                    self.master.get_logger().warn(f'⚠️ Received partial motor data: pos={len(msg.position)}, vel={len(msg.velocity)}, eff={len(msg.effort)}')
            # Silently ignore completely empty messages when motors aren't added
        except Exception as e:
            self.master.get_logger().error(f'Error processing motor data: {str(e)}')

    def motor_der_callback(self, msg):
        try:
            # Log the raw message structure for debugging (commented out to reduce spam)
            # self.master.get_logger().info(f'📊 Received joint_states: pos={len(msg.position)}, vel={len(msg.velocity)}, eff={len(msg.effort)}, names={msg.name}')

            if len(msg.position) >= 2 and len(msg.velocity) >= 2 and len(msg.effort) >= 2:
                # Store previous positions for change detection
                #prev_left_pos = getattr(self.motor_cadera_izq_data, 'position', 0.0)
                prev_right_pos = getattr(self.motor_cadera_der_data, 'position', 0.0)
                
                # Left motor (index 0)
                """self.motor_cadera_izq_data.voltage = msg.effort[0]  # Using effort as voltage for now
                self.motor_cadera_izq_data.current = msg.effort[0]  # Using effort as current for now
                self.motor_cadera_izq_data.position = self.rad2deg(msg.position[0])
                self.motor_cadera_izq_data.velocity = msg.velocity[0]
                self.motor_cadera_izq_data.torque = msg.effort[0]"""
                
                # Right motor (index 1)  
                self.motor_cadera_der_data.voltage = msg.effort[1]
                self.motor_cadera_der_data.current = msg.effort[1]
                self.motor_cadera_der_data.position = self.rad2deg(msg.position[1])
                self.motor_cadera_der_data.velocity = msg.velocity[1]
                self.motor_cadera_der_data.torque = msg.effort[1]
                
                # Log significant position changes (more than 1 degree)
                #left_change = abs(self.motor_cadera_izq_data.position - prev_left_pos)
                right_change = abs(self.motor_cadera_der_data.position - prev_right_pos)
                
                if right_change > 1.0:
                    self.master.get_logger().info(f'🔄 Motor positions: Right={self.motor_cadera_der_data.position:.1f}° (Δ{right_change:.1f}°)')
                    self.master.get_logger().info(f'🔄 Motor velocities: Right={self.motor_cadera_der_data.velocity:.2f}rad/s')
                    
            elif len(msg.position) == 1 and len(msg.velocity) == 1 and len(msg.effort) == 1:
                # Only one motor detected
                #self.master.get_logger().warn(f'⚠️ Only one motor detected in joint_states! Name: {msg.name[0] if msg.name else "unknown"}')
                
                # Assume it's the left motor for now
                prev_right_pos = getattr(self.motor_cadera_der_data, 'position', 0.0)
                self.motor_cadera_der_data.voltage = msg.effort[0]
                self.motor_cadera_der_data.current = msg.effort[0]
                self.motor_cadera_der_data.position = self.rad2deg(msg.position[0])
                self.motor_cadera_der_data.velocity = msg.velocity[0]
                self.motor_cadera_der_data.torque = msg.effort[0]
                
                right_change = abs(self.motor_cadera_der_data.position - prev_right_pos)
                if right_change > 1.0:
                    self.master.get_logger().info(f'🔄 Single motor position: {self.motor_cadera_der_data.position:.1f}° (Δ{right_change:.1f}°)')
                    
            elif len(msg.position) > 0 or len(msg.velocity) > 0 or len(msg.effort) > 0:
                # Partial data received
                if hasattr(self, '_motors_added') and self._motors_added:
                    self.master.get_logger().warn(f'⚠️ Received partial motor data: pos={len(msg.position)}, vel={len(msg.velocity)}, eff={len(msg.effort)}')
            # Silently ignore completely empty messages when motors aren't added
        except Exception as e:
            self.master.get_logger().error(f'Error processing motor data: {str(e)}')

    # Degree/radian conversion
    def deg2rad(self, deg):
        return deg * 2 * math.pi / 360

    def rad2deg(self, rad):
        return rad * 360 / (2 * math.pi)
    
    # Sends async service call to add motors by CAN IDs and registers add_motors_callback.
    def add_motors(self):
        print()
        print("="*60)
        self.master.get_logger().info('🔧 DEBUG: add_motors function called')
        print("ADD MD80")
        
        # Check if service is available
        if not self.addMd80Service.service_is_ready():
            self.master.get_logger().warn('⚠️ WARNING: AddMd80s service is not available!')
            return
        
        add_request = AddMd80s.Request()
        add_request.drive_ids = drives_can_id
        self.master.get_logger().info(f'🔧 DEBUG: Requesting to add motors with CAN IDs: {drives_can_id}')
        add_future = self.addMd80Service.call_async(add_request)
        add_future.add_done_callback(self.add_motors_callback)

    # Callback to process add motors service response
    def add_motors_callback(self, future):
        try:
            response = future.result()
            self.master.get_logger().info(f'Add motors service response: {response}')
            
            # Enhanced motor detection logging
            if hasattr(response, 'drives_success'):
                self.master.get_logger().info(f'🔍 Motor detection results:')
                for i, (drive_id, success) in enumerate(zip(drives_can_id, response.drives_success)):
                    status = "✅ DETECTED" if success else "❌ NOT FOUND"
                    self.master.get_logger().info(f'  Motor ID {drive_id}: {status}')
                
                successful_motors = sum(response.drives_success)
                self.master.get_logger().info(f'📊 Total: {successful_motors}/{len(drives_can_id)} motors detected')
                
                # Update button texts to reflect CAN IDs
                self.ui.enable_motors.setText(f"Enable Left Motor: id= {left_hip_can_id}")
                self.ui.enable_right_motor.setText(f"Enable Right Motor: id= {right_hip_can_id}")

                if any(response.drives_success):
                    self._motors_added = True
                    if successful_motors == len(drives_can_id):
                        self.master.get_logger().info("🎯 All motors successfully added")
                    else:
                        self.master.get_logger().warn(f"⚠️ Only {successful_motors} out of {len(drives_can_id)} motors added")
                else:
                    self._motors_added = False
                    self.master.get_logger().warn("❌ No motors were successfully added")
            
                print("="*60)

            else:
                self._motors_added = False
                self.master.get_logger().warn("No drives_success field in response")
                
        except Exception as e:
            self.master.get_logger().error(f'Add motors service call failed: {e}')
            self._motors_added = False

    # Enable/disable left motor  
    def enable_left_motor(self, state):
        """Enable/disable left motor using set-based approach"""
        print()
        print("="*60)
        self.master.get_logger().info(f'🔧 DEBUG: enable_left_motor called with state: {state}')

        # Update desired motors set
        if state:  # Checkbox checked - add left motor to desired set
            self._desired_enabled_motors.add(left_hip_can_id)
            self.master.get_logger().info(f'🔄 Adding left motor {left_hip_can_id} to desired set')
        else:  # Checkbox unchecked - remove left motor from desired set
            self._desired_enabled_motors.discard(left_hip_can_id)  # discard won't raise error if not present
            self.master.get_logger().info(f'🔄 Removing left motor {left_hip_can_id} from desired set')

        self.master.get_logger().info(f'🔧 DEBUG: Current desired motors: {self._desired_enabled_motors}')
        self.master.get_logger().info(f'🔧 DEBUG: Checkbox states - Left: {self.ui.enable_motors.isChecked()}, Right: {self.ui.enable_right_motor.isChecked()}')

        # Check if motors are added first
        if not hasattr(self, '_motors_added') or not self._motors_added:
            self.master.get_logger().warn('⚠️ Cannot enable/disable left motor: Motor not added yet!')
            self.ui.enable_motors.blockSignals(True)
            self.ui.enable_motors.setChecked(False)  # Reset checkbox
            self.ui.enable_motors.blockSignals(False)
            return
        
        print(60*"=")
        # Apply the complete motor state
        self._apply_motor_state()

    # Enable/disable right motor
    def enable_right_motor(self, state):
        """Enable/disable right motor using set-based approach"""
        self.master.get_logger().info(f'🔧 DEBUG: enable_right_motor called with state: {state}')

        # Block enabling right motor if it is not part of the configured drives
        if right_hip_can_id not in drives_can_id:
            self.master.get_logger().warn('⚠️ Right motor not configured in drives_can_id; ignoring enable request')
            self.ui.enable_right_motor.blockSignals(True)
            self.ui.enable_right_motor.setChecked(False)
            self.ui.enable_right_motor.blockSignals(False)
            return

        # Update desired motors set
        if state:  # Checkbox checked - add right motor to desired set
            self._desired_enabled_motors.add(right_hip_can_id)
            self.master.get_logger().info(f'🔄 Adding right motor {right_hip_can_id} to desired set')
        else:  # Checkbox unchecked - remove right motor from desired set
            self._desired_enabled_motors.discard(right_hip_can_id)  # discard won't raise error if not present
            self.master.get_logger().info(f'🔄 Removing right motor {right_hip_can_id} from desired set')

        self.master.get_logger().info(f'🔧 DEBUG: Current desired motors: {self._desired_enabled_motors}')
        self.master.get_logger().info(f'🔧 DEBUG: Checkbox states - Left: {self.ui.enable_motors.isChecked()}, Right: {self.ui.enable_right_motor.isChecked()}')
        
        # Check if motors are added first
        if not hasattr(self, '_motors_added') or not self._motors_added:
            self.master.get_logger().warn('⚠️ Cannot enable/disable right motor: Motor not added yet!')
            self.ui.enable_right_motor.blockSignals(True)
            self.ui.enable_right_motor.setChecked(False)  # Reset checkbox
            self.ui.enable_right_motor.blockSignals(False)
            return
        
        print(60*"=")
        # Apply the complete motor state
        self._apply_motor_state()

    # Apply the current desired motor state by enabling desired motors and disabling others
    def _apply_motor_state(self):
        self.master.get_logger().info(f'🎯 Analyzing motor state changes - Desired: {sorted(self._desired_enabled_motors)}')
        
        # Track current actual state based on variables
        currently_enabled = set()
        if getattr(self, '_left_motor_enabled', False):
            currently_enabled.add(left_hip_can_id)
        if getattr(self, '_right_motor_enabled', False):
            currently_enabled.add(right_hip_can_id)
        
        self.master.get_logger().info(f'🔄 Current: {sorted(currently_enabled)} -> Target: {sorted(self._desired_enabled_motors)}')
        
        # 1. Check if we actually need any state changes
        if currently_enabled == self._desired_enabled_motors:
            self.master.get_logger().info('✅ No motor state changes needed - all motors already in correct state')
            return

        # 2. Calculate differentials (What to enable, what to disable)
        motors_to_enable = self._desired_enabled_motors - currently_enabled
        motors_to_disable = currently_enabled - self._desired_enabled_motors

        # 3. Process Disables FIRST (Only to specific motors that were turned off)
        if motors_to_disable:
            self.master.get_logger().info(f'🛑 Disabling specific motors: {sorted(list(motors_to_disable))}')
            disable_request = GenericMd80Msg.Request()
            disable_request.drive_ids = sorted(list(motors_to_disable))
            
            # Al apagar, actualizamos sus flags inmediatamente para reflejar el cambio en la GUI
            for motor_id in motors_to_disable:
                if motor_id == left_hip_can_id: self._left_motor_enabled = False
                if motor_id == right_hip_can_id: self._right_motor_enabled = False
                
            self.disableMd80Service.call_async(disable_request)
            # Damos un pequeño margen para que el bus CAN procese el disable antes de mandar un enable
            time.sleep(0.02) 

        # 4. Process Enables (Only to specific motors that need to turn on)
        if motors_to_enable:
            self.master.get_logger().info(f'⚡ Enabling specific motors: {sorted(list(motors_to_enable))}')
            enable_request = GenericMd80Msg.Request()
            enable_request.drive_ids = sorted(list(motors_to_enable))
            
            # Llamamos al servicio asíncrono para activar SOLO los nuevos motores
            enable_future = self.enableMd80Service.call_async(enable_request)
            enable_future.add_done_callback(lambda f, ids=motors_to_enable: self._handle_specific_enable_done(f, ids))
    
    def _handle_specific_enable_done(self, future, motor_ids):
        try:
            response = future.result()
            self.master.get_logger().info(f'✅ ROS 2 Service response received for enabling motors: {sorted(list(motor_ids))}')
            
            # Confirmamos la activación en nuestros estados internos individuales
            for motor_id in motor_ids:
                if motor_id == left_hip_can_id:
                    self._left_motor_enabled = True
                    self.master.get_logger().info("➡ Left motor state marked as ENABLED")
                if motor_id == right_hip_can_id:
                    self._right_motor_enabled = True
                    self.master.get_logger().info("➡ Right motor state marked as ENABLED")
                    
        except Exception as e:
            self.master.get_logger().error(f'❌ Failed to enable motors {sorted(list(motor_ids))}: {str(e)}')

    def enable_selected_motors_button_clicked(self):
        """Recoge los motores seleccionados en los checkboxes y los habilita simultáneamente"""
        
        print(60*"=")

        # Validación de seguridad: verificar si Candle ya conoce los motores
        if not hasattr(self, '_motors_added') or not self._motors_added:
            self.master.get_logger().warn('⚠️ No se pueden habilitar: ¡Los motores no han sido añadidos al driver todavía!')
            return

        # 1. Crear la lista de motores a activar leyendo directamente la interfaz
        motores_a_activar = []
        
        if self.ui.enable_motors.isChecked():       # Checkbox de la pierna izquierda
            motores_a_activar.append(left_hip_can_id)
            
        if self.ui.enable_right_motor.isChecked():  # Checkbox de la pierna derecha
            motores_a_activar.append(right_hip_can_id)

        # 2. Validar si el usuario se ha olvidado de marcar algún checkbox
        if not motores_a_activar:
            self.master.get_logger().warn('⚠️ Ningún motor seleccionado. Marca al menos un checkbox antes de pulsar el botón.')
            return

        self.master.get_logger().info(f'🚀 Botón pulsado. Enviando petición unificada para activar IDs: {sorted(motores_a_activar)}')

        # 3. Construir el mensaje de ROS 2 con la lista completa de una sola vez
        request = GenericMd80Msg.Request()
        request.drive_ids = sorted(motores_a_activar)

        # 4. Realizar la llamada asíncrona al servicio de Candle
        future = self.enableMd80Service.call_async(request)
        future.add_done_callback(self._handle_unified_enable_done)

        print(60*"=")

    def _handle_unified_enable_done(self, future):
        """Callback que procesa la respuesta del nodo C++ de Candle"""
        try:
            response = future.result()
            self.master.get_logger().info('✅ ¡Respuesta de Candle exitosa! Bucle de tiempo real iniciado para los motores seleccionados.')
            
            # Sincronizar de forma segura nuestros flags booleanos internos con lo que está marcado en la UI
            self._left_motor_enabled = self.ui.enable_motors.isChecked()
            self._right_motor_enabled = self.ui.enable_right_motor.isChecked()
            self._motors_enabled = self._left_motor_enabled or self._right_motor_enabled
            
            self.master.get_logger().info(f'📊 Flags internos actualizados -> IZQ: {self._left_motor_enabled} | DER: {self._right_motor_enabled}')
            
        except Exception as e:
            self.master.get_logger().error(f'❌ Error crítico en el servicio de habilitación conjunta: {str(e)}')

    # Handle the reset-disable response, then enable desired motors
    def _handle_reset_then_enable(self, disable_future):
        try:
            disable_response = disable_future.result()
            self.master.get_logger().info(f'Reset disable response: {disable_response}')
            
            # Now enable desired motors (if any)
            if self._desired_enabled_motors:
                desired_motors = sorted(list(self._desired_enabled_motors))
                self.master.get_logger().info(f'🔄 Enabling desired motors: {desired_motors}')
                
                # Control mode check
                if self._current_control_mode is None:
                    self.master.get_logger().warn(f'⚠️ No control mode set! Enabling motors anyway, but you should select a control mode.')
                    self.master.get_logger().warn(f'🎛️ Recommended: Select a control mode (Position, Velocity, or Impedance) for proper operation.')
                else:
                    self.master.get_logger().info(f'✅ Control mode is {self._current_control_mode}, proceeding with motor enable...')
                
                # Enable all desired motors simultaneously
                enable_request = GenericMd80Msg.Request()
                enable_request.drive_ids = desired_motors
                enable_future = self.enableMd80Service.call_async(enable_request)
                enable_future.add_done_callback(lambda f: self._apply_motor_state_callback(f, desired_motors, True))
            else:
                # No motors to enable, just update state
                self.master.get_logger().info('✅ No motors to enable after reset')
                # Set all motors as disabled
                self._left_motor_enabled = False
                self._right_motor_enabled = False
                self._motors_enabled = False
                self._update_motor_state_tracking()
                
        except Exception as e:
            self.master.get_logger().error(f'Reset disable failed: {e}')
            # Reset all on error
            self._desired_enabled_motors.clear()
            self.ui.enable_motors.blockSignals(True)
            self.ui.enable_motors.setChecked(False)
            self.ui.enable_motors.blockSignals(False)
            self.ui.enable_right_motor.blockSignals(True)
            self.ui.enable_right_motor.setChecked(False)
            self.ui.enable_right_motor.blockSignals(False)
            self._update_motor_state_tracking()

    # Callback for motor state application
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
                                self.ui.enable_motors.blockSignals(True)
                                self.ui.enable_motors.setChecked(False)
                                self.ui.enable_motors.blockSignals(False)
                            elif motor_id == right_hip_can_id:
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
                        self.ui.enable_motors.blockSignals(True)
                        self.ui.enable_motors.setChecked(False)
                        self.ui.enable_motors.blockSignals(False)
                    elif motor_id == right_hip_can_id:
                        self.ui.enable_right_motor.blockSignals(True)
                        self.ui.enable_right_motor.setChecked(False)
                        self.ui.enable_right_motor.blockSignals(False)
            self._update_motor_state_tracking()

    # Update the legacy motor state tracking variables based on desired set
    def _update_motor_state_tracking(self):
        self._left_motor_enabled = left_hip_can_id in self._desired_enabled_motors
        self._right_motor_enabled = right_hip_can_id in self._desired_enabled_motors
        self._motors_enabled = self._left_motor_enabled or self._right_motor_enabled
        
        self.master.get_logger().info(f'🔧 Updated motor states - Left: {self._left_motor_enabled}, Right: {self._right_motor_enabled}, Overall: {self._motors_enabled}')
        self.master.get_logger().info(f'🔧 Desired set: {sorted(self._desired_enabled_motors)}')
        self.master.get_logger().info(f'🔧 Checkbox states - Left: {self.ui.enable_motors.isChecked()}, Right: {self.ui.enable_right_motor.isChecked()}')
        print(60*"=")
        print()

    # Generic service callback that logs response or error from ROS2 service calls.
    def service_callback(self, future):
        try:
            response = future.result()
            self.master.get_logger().info(f'Service response: {response}')
        except Exception as e:
            self.master.get_logger().info(f'Service call failed: {e}')

    # Handles SetMode service response, logging results and maintaining control mode state.
    def set_control_callback(self, future, mode_):
        try:
            response = future.result()
            self.master.get_logger().info(f'SetMode service response: {response}')
            
            # Convert mode string to numeric value for publish_pid_params_all_motors
            mode_numeric = 0.0
            if mode_ == "POSITION_PID":
                mode_numeric = 3.0
            elif mode_ == "VELOCITY_PID":
                mode_numeric = 2.0
            elif mode_ == "IMPEDANCE":
                mode_numeric = 4.0
            
            # Check if the response has drives_success field
            if hasattr(response, 'drives_success') and response.drives_success:
                # Ensure we don't go out of bounds
                num_responses = len(response.drives_success)
                num_drives = len(drives_can_id)
                
                if num_responses != num_drives:
                    self.master.get_logger().warn(f'⚠️ Response count mismatch: {num_responses} responses for {num_drives} drives')
                
                success_count = 0
                # Use the minimum to avoid index errors
                for i in range(min(num_responses, num_drives)):
                    drive_id = drives_can_id[i]
                    success = response.drives_success[i]
                    if success:
                        success_count += 1
                        self.master.get_logger().info(f'✅ Motor {drive_id}: Successfully set to {mode_} mode')
                    else:
                        self.master.get_logger().warn(f'❌ Motor {drive_id}: Failed to set {mode_} mode')
                
                if success_count == num_drives:
                    self.master.get_logger().info(f'🎯 All motors successfully set to {mode_} mode')
                    # Control mode was already set early, no need to set again
                    
                    # Send appropriate initial parameters based on mode
                    if mode_numeric == 4.0:  # IMPEDANCE
                        self.master.get_logger().info('🔧 Impedance mode set successfully - sending initial parameters automatically')
                        self.publish_pid_params_all_motors(4.0)
                    elif mode_numeric == 3.0:  # POSITION_PID
                        self.master.get_logger().info('🔧 Position mode set successfully - sending initial PID parameters automatically')
                        self.publish_pid_params_all_motors(3.0)
                    elif mode_numeric == 2.0:  # VELOCITY_PID
                        self.master.get_logger().info('🔧 Velocity mode set successfully - sending initial PID parameters automatically')
                        self.publish_pid_params_all_motors(2.0)
                else:
                    self.master.get_logger().warn(f'⚠️ Only {success_count}/{num_drives} motors set to {mode_} mode')
                    self._current_control_mode = None  # Reset if not all motors succeeded
            else:
                # Fallback if no drives_success field
                self.master.get_logger().info(f'SetMode service completed for mode: {mode_}')
                # Still try to send parameters even without explicit success confirmation
                if mode_numeric > 0:
                    self.publish_pid_params_all_motors(mode_numeric)
                
        except Exception as e:
            self.master.get_logger().error(f'SetMode service call failed: {e}')
            import traceback
            self.master.get_logger().error(f'Traceback: {traceback.format_exc()}')
            self._current_control_mode = None  # Reset on exception

    # TEST FUNCTION: Try enabling both motors simultaneously  
    def enable_both_motors_test(self):
        """Test function to see if enabling both motors in one request works"""
        self.master.get_logger().info('🧪 TESTING: Attempting to enable both motors simultaneously...')
        
        if not hasattr(self, '_motors_added') or not self._motors_added:
            self.master.get_logger().warn('⚠️ Cannot test: Motors not added yet!')
            return
            
        # Send CAN command for BOTH motors in one request
        request = GenericMd80Msg.Request()
        request.drive_ids = [left_hip_can_id, right_hip_can_id]  # Both motors
        
        future = self.enableMd80Service.call_async(request)
        future.add_done_callback(self._handle_both_motors_test_response)

    # Handle the response from simultaneous enable test
    def _handle_both_motors_test_response(self, future):
        try:
            response = future.result()
            self.master.get_logger().info(f'🧪 Both motors enable test response: {response}')
            
            if hasattr(response, 'drives_success') and len(response.drives_success) >= 2:
                left_success = response.drives_success[0]
                right_success = response.drives_success[1]
                
                self.master.get_logger().info(f'🧪 Test Results:')
                self.master.get_logger().info(f'  Left Motor (ID {left_hip_can_id}): {"✅ SUCCESS" if left_success else "❌ FAILED"}')
                self.master.get_logger().info(f'  Right Motor (ID {right_hip_can_id}): {"✅ SUCCESS" if right_success else "❌ FAILED"}')
                
                if left_success and right_success:
                    self.master.get_logger().info('🎉 SOLUTION FOUND: Both motors can be enabled simultaneously!')
                    self._left_motor_enabled = True
                    self._right_motor_enabled = True
                    # Update checkboxes to reflect actual state
                    self.ui.enable_motors.blockSignals(True)
                    self.ui.enable_right_motor.blockSignals(True)
                    self.ui.enable_motors.setChecked(True)
                    self.ui.enable_right_motor.setChecked(True)
                    self.ui.enable_motors.blockSignals(False)
                    self.ui.enable_right_motor.blockSignals(False)
                else:
                    self.master.get_logger().warn('❌ Simultaneous enable also failed - this might be a hardware/firmware limitation')
            else:
                self.master.get_logger().warn('🧪 Test response format unexpected')
                
        except Exception as e:
            self.master.get_logger().error(f'🧪 Both motors test failed: {e}')

    # Sends zeroing request for currently enabled motors, setting their current position as reference.
    def zero_motors(self):
        # Check which motors are enabled and zero only those
        motors_to_zero = []
        if hasattr(self, '_left_motor_enabled') and self._left_motor_enabled:
            motors_to_zero.append(left_hip_can_id)
        if hasattr(self, '_right_motor_enabled') and self._right_motor_enabled:
            motors_to_zero.append(right_hip_can_id)
            
        if not motors_to_zero:
            self.master.get_logger().warn('⚠️ Cannot zero motors: No motors are enabled!')
            return
            
        motor_names = []
        if left_hip_can_id in motors_to_zero:
            motor_names.append("left")
        if right_hip_can_id in motors_to_zero:
            motor_names.append("right")
            
        self.master.get_logger().info(f'🔄 Zeroing {" and ".join(motor_names)} motor(s) (setting current position as reference)...')
        request = GenericMd80Msg.Request()
        request.drive_ids = motors_to_zero
        future = self.zeroMd80Service.call_async(request)
        future.add_done_callback(lambda f: self.zero_motors_callback(f, motors_to_zero))

    # Handles zero-motors service response and logs per-motor success.
    def zero_motors_callback(self, future, motors_zeroed):
        try:
            response = future.result()
            self.master.get_logger().info(f'Zero motors service response: {response}')
            
            if hasattr(response, 'drives_success'):
                success_count = 0
                for i, (drive_id, success) in enumerate(zip(motors_zeroed, response.drives_success)):
                    if success:
                        success_count += 1
                        motor_name = "left" if drive_id == left_hip_can_id else "right"
                        self.master.get_logger().info(f'✅ {motor_name.title()} motor ({drive_id}): Successfully zeroed')
                    else:
                        motor_name = "left" if drive_id == left_hip_can_id else "right"
                        self.master.get_logger().warn(f'❌ {motor_name.title()} motor ({drive_id}): Failed to zero')
                
                if success_count == len(motors_zeroed):
                    motor_names = []
                    for drive_id in motors_zeroed:
                        if drive_id == left_hip_can_id:
                            motor_names.append("left")
                        elif drive_id == right_hip_can_id:
                            motor_names.append("right")
                    self.master.get_logger().info(f'🎯 {" and ".join(motor_names).title()} motor(s) successfully zeroed')
                else:
                    self.master.get_logger().warn(f'⚠️ Only {success_count}/{len(motors_zeroed)} motors zeroed successfully')
            else:
                self.master.get_logger().info('Motors zeroed')
                
        except Exception as e:
            self.master.get_logger().error(f'Zero motors service call failed: {e}')

    # Controller selection for left hip
    def set_control(self, force_mode=None, callback=None):
        """Set control mode. Can be called with explicit mode or from UI selection."""
        if force_mode:
            # Called with explicit mode (for motor enable sequence)
            if force_mode.lower() == "impedance":
                control = 4.0
                mode_ = "IMPEDANCE"
            else:
                self.master.get_logger().warn(f'⚠️ Unsupported forced mode: {force_mode}')
                return
        else:
            # Called from UI selection
            if self.ui.position_control.isChecked():
                control = 3.0
                mode_ = "POSITION_PID"
            elif self.ui.velocity_control.isChecked():
                control = 2.0
                mode_ = "VELOCITY_PID"
            elif self.ui.impedance_control.isChecked():
                control = 4.0
                mode_ = "IMPEDANCE"
            else:
                return

        print()
        print("="*60)
        # Handle impedance mode selection (no separate node needed - direct publishing like position control)
        if control == 4.0:  # Impedance control
            self.master.get_logger().info('🎛️ Impedance mode selected - parameters will be published directly')
        else:
            self.master.get_logger().info(f'🎛️ Mode {mode_} selected')

        # Reset all sliders to 0 when changing control mode (only for UI-initiated changes)
        if not force_mode:
            self.reset_all_sliders()

        # Set the control mode early for immediate parameter updates
        self._current_control_mode = mode_

        request = SetModeMd80s.Request()
        request.drive_ids = drives_can_id
        request.mode = [mode_] * len(drives_can_id)  # Send mode name as string ("POSITION_PID", "VELOCITY_PID", "IMPEDANCE")
        self.master.get_logger().info(f'🔧 Sending SetMode request: drive_ids={request.drive_ids}, mode={request.mode}')
        future = self.setModeMd80Service.call_async(request)
        
        # Use custom callback if provided, otherwise use default
        if callback:
            future.add_done_callback(lambda f: self._set_control_with_callback(f, mode_, callback))
        else:
            future.add_done_callback(lambda f: self.set_control_callback(f, mode_))

    # Handle set_control response when a custom callback is provided
    def _set_control_with_callback(self, future, mode_, custom_callback):
        try:
            # First, handle the normal set_control response
            self.set_control_callback(future, mode_)
            # Then call the custom callback
            custom_callback()
        except Exception as e:
            self.master.get_logger().error(f'Custom callback execution failed: {e}')

    # Publish PID parameters for all motors depending on control mode
    def publish_pid_params_all_motors(self, control):
        if control == 3.0:  # Position
            # Position PID parameters - more conservative for safety
            pos_pid_params = Pid()
            pos_pid_params.kp = 20.0  # Reduced from 40.0 for stability
            pos_pid_params.ki = 0.2   # Reduced from 0.5
            pos_pid_params.kd = 0.5   # Added some derivative for damping
            pos_pid_params.i_windup = 5.0  # Reduced windup
            pos_pid_params.max_output = 2.0  # Reduced max output

            # Velocity PID parameters
            vel_pid_params = Pid()
            vel_pid_params.kp = 0.15  # Slightly reduced
            vel_pid_params.ki = 0.2   # Reduced
            vel_pid_params.kd = 0.0
            vel_pid_params.i_windup = 10.0  # Reduced windup
            vel_pid_params.max_output = 1.5  # Reduced max output

            msg = PositionPidCommand()
            msg.drive_ids = drives_can_id  # Send to both motors
            msg.position_pid = [pos_pid_params] * len(drives_can_id)
            msg.velocity_pid = [vel_pid_params] * len(drives_can_id)
            
            self.master.get_logger().info(f'🎛️ Publishing PID params: Pos(kp={pos_pid_params.kp}, ki={pos_pid_params.ki}, kd={pos_pid_params.kd})')
            self.master.get_logger().info(f'🎛️ Publishing PID params: Vel(kp={vel_pid_params.kp}, ki={vel_pid_params.ki}, kd={vel_pid_params.kd})')
            self.master.publisher_position_pid_params.publish(msg)
            print("="*60)
            print()

        elif control == 2.0:  # Velocity
            vel_pid_params = Pid()
            vel_pid_params.kp = 0.2
            vel_pid_params.ki = 0.3
            vel_pid_params.kd = 0.0
            vel_pid_params.i_windup = 2.0
            vel_pid_params.max_output = 2.0

            msg = VelocityPidCommand()
            msg.drive_ids = drives_can_id
            msg.velocity_pid = [vel_pid_params] * len(drives_can_id)

            self.master.publisher_velocity_pid_params.publish(msg)

        elif control == 4.0:  # Impedance
            # Get impedance parameters from config
            kp_defaults = self.config.get_kp_gains()
            kd_defaults = self.config.get_kd_gains() 
            max_output_defaults = self.config.get_max_output_torque()
            
            msg = ImpedanceCommand()
            msg.drive_ids = [int(d) for d in drives_can_id]  # Ensure uint16 type
            # Use only the number of motors we have configured
            num_configured_motors = len(drives_can_id)
            msg.kp = [float(kp_defaults[i]) for i in range(num_configured_motors)]  # Match motor count
            msg.kd = [float(kd_defaults[i]) for i in range(num_configured_motors)]  # Match motor count
            msg.max_output = [float(max_output_defaults[i]) for i in range(num_configured_motors)]  # Match motor count

            self.master.get_logger().info(f'🎛️ Publishing initial impedance params for {num_configured_motors} motor(s): kp={msg.kp}, kd={msg.kd}, max_output={msg.max_output}')
            self.master.get_logger().info(f'🎛️ Drive IDs: {list(msg.drive_ids)}')
            self.master.publisher_impedance_params.publish(msg)
            
            # Also send initial offset torque via MotionCommand if impedance mode
            self._send_initial_impedance_offset()

    # Verify that impedance parameters are correctly set for configured motors
    def _verify_impedance_parameters_for_both_motors(self, kp_values, kd_values, offset_values):
        num_motors = len(drives_can_id)
        self.master.get_logger().info(f'🔍 VERIFICATION: Impedance parameters for {num_motors} motor(s):')
        motor_names = ['Left', 'Right']
        for i in range(num_motors):
            motor_name = motor_names[i] if i < len(motor_names) else f'Motor{i}'
            self.master.get_logger().info(f'   {motor_name} (CAN ID {drives_can_id[i]}): kP={kp_values[i]:.3f}, kD={kd_values[i]:.3f}, offset={offset_values[i]:.3f}')
        self.master.get_logger().info(f'   Drive IDs: {drives_can_id}')

    # Send initial offset torque for impedance mode via MotionCommand
    def _send_initial_impedance_offset(self):
        try:
            # Get offset torque from config
            offset_values = self.config.get_offset_torque()
            
            # Create motion command for impedance offset torque
            num_configured_motors = len(drives_can_id)
            msg = MotionCommand()
            msg.drive_ids = [int(d) for d in drives_can_id]  # Ensure uint32 type
            msg.target_position = [0.0] * num_configured_motors  # Not used in impedance mode
            msg.target_velocity = [0.0] * num_configured_motors  # Not used in impedance mode  
            msg.target_torque = [float(offset_values[i]) for i in range(num_configured_motors)]  # Match motor count
            
            self.master.get_logger().info(f'🔧 Sending initial impedance offset torque for {num_configured_motors} motor(s): {[f"{offset_values[i]:.2f}" for i in range(num_configured_motors)]} Nm')
            self.master.get_logger().info(f'🔧 Drive IDs: {list(msg.drive_ids)}')
            self.master.publisher_motion_commands.publish(msg)
            print("="*60)

        except Exception as e:
            self.master.get_logger().error(f'❌ Failed to send initial impedance offset: {str(e)}')

    # Legacy function for single motor (keeping for compatibility)
    def publish_pid_params(self, can_id, control):
        # Redirect to new function
        self.publish_pid_params_all_motors(control)

    # Reads kp/kd/offset from UI spinboxes and publishes impedance parameters (manual button).
    def update_impedance_parameters(self):
        """Publish impedance parameters directly to candle_ros2 (like position control)"""
        self.master.get_logger().info("🔧 Manual impedance parameter update triggered")
        try:
            # Get all current impedance parameters from UI (with config fallbacks)
            kp_defaults = self.config.get_kp_gains()
            kd_defaults = self.config.get_kd_gains() 
            max_output_defaults = self.config.get_max_output_torque()
            offset_defaults = self.config.get_offset_torque()
            
            num_configured_motors = len(drives_can_id)
            
            # Collect parameters for each configured motor
            kp_values = []
            kd_values = []
            offset_values = []
            
            for i in range(num_configured_motors):
                # Get UI boxes if they exist (left motor = index 0, right motor = index 1)
                if i == 0:  # Left motor
                    kp_values.append(float(self.ui.left_hip_kp_box.value()) if hasattr(self.ui, 'left_hip_kp_box') else kp_defaults[0])
                    kd_values.append(float(self.ui.left_hip_kd_box.value()) if hasattr(self.ui, 'left_hip_kd_box') else kd_defaults[0])
                    offset_values.append(float(self.ui.left_hip_offset_box.value()) if hasattr(self.ui, 'left_hip_offset_box') else offset_defaults[0])
                elif i == 1:  # Right motor
                    kp_values.append(float(self.ui.right_hip_kp_box.value()) if hasattr(self.ui, 'right_hip_kp_box') else kp_defaults[1])
                    kd_values.append(float(self.ui.right_hip_kd_box.value()) if hasattr(self.ui, 'right_hip_kd_box') else kd_defaults[1])
                    offset_values.append(float(self.ui.right_hip_offset_box.value()) if hasattr(self.ui, 'right_hip_offset_box') else offset_defaults[1])
            
            # Create and publish impedance command directly (like position control)
            msg = ImpedanceCommand()
            msg.drive_ids = [int(d) for d in drives_can_id]  # Ensure uint16 type
            msg.kp = [float(kp) for kp in kp_values]  # Match motor count
            msg.kd = [float(kd) for kd in kd_values]  # Match motor count
            msg.max_output = [float(max_output_defaults[i]) for i in range(num_configured_motors)]  # Match motor count
            
            # Log parameters for all configured motors
            motor_names = ['Left', 'Right']
            for i in range(num_configured_motors):
                motor_name = motor_names[i] if i < len(motor_names) else f'Motor{i}'
                self.master.get_logger().info(f'🔧 Publishing impedance params: {motor_name}(kp={kp_values[i]:.2f}, kd={kd_values[i]:.3f})')
            self.master.get_logger().info(f'🔧 Max outputs: {[f"{max_output_defaults[i]:.1f}" for i in range(num_configured_motors)]} Nm')
            
            # Direct publish - no service needed (consistent with position control approach)
            self.master.publisher_impedance_params.publish(msg)
            
            # Also send offset torque via MotionCommand
            motion_msg = MotionCommand()
            motion_msg.drive_ids = [int(d) for d in drives_can_id]  # Ensure uint32 type
            motion_msg.target_position = [0.0] * num_configured_motors  # Not used in impedance mode
            motion_msg.target_velocity = [0.0] * num_configured_motors  # Not used in impedance mode
            motion_msg.target_torque = [float(offset) for offset in offset_values]  # Match motor count
            
            # Log offset torques for all configured motors
            for i in range(num_configured_motors):
                motor_name = motor_names[i] if i < len(motor_names) else f'Motor{i}'
                self.master.get_logger().info(f'🔧 Publishing offset torque: {motor_name}={offset_values[i]:.2f} Nm')
            
            self.master.publisher_motion_commands.publish(motion_msg)
            
        except Exception as e:
            self.master.get_logger().error(f'❌ Manual impedance parameter update failed: {str(e)}')

    # Legacy function for single motor (keeping for compatibility)
    def publish_pid_params(self, can_id, control):
        # Redirect to new function
        self.publish_pid_params_all_motors(control)

    # Update impedance parameters for left motor only (button-triggered)
    def update_impedance_parameters_left(self):
        """Update impedance parameters for left motor only when button is pressed"""

        print()
        print("="*60)
        self.master.get_logger().info("🔧 Left motor impedance parameter update triggered (button)")
        try:
            # Check control mode
            if not hasattr(self, '_current_control_mode') or self._current_control_mode != "IMPEDANCE":
                self.master.get_logger().warn('⚠️ Not in IMPEDANCE mode; skipping left motor parameter update')
                return
            
            # Check if left motor is enabled
            if not (hasattr(self, '_left_motor_enabled') and self._left_motor_enabled):
                self.master.get_logger().warn('⚠️ Left motor not enabled; skipping parameter update')
                return
            
            # Get configuration defaults
            kp_defaults = self.config.get_kp_gains()
            kd_defaults = self.config.get_kd_gains()
            max_output_defaults = self.config.get_max_output_torque()
            offset_defaults = self.config.get_offset_torque()
            
            # Get left motor values from UI (index 0)
            left_kp = float(self.ui.left_hip_kp_box.value()) if hasattr(self.ui, 'left_hip_kp_box') else kp_defaults[0]
            left_kd = float(self.ui.left_hip_kd_box.value()) if hasattr(self.ui, 'left_hip_kd_box') else kd_defaults[0]
            left_offset = float(self.ui.left_hip_offset_box.value()) if hasattr(self.ui, 'left_hip_offset_box') else offset_defaults[0]
            left_max_output = float(max_output_defaults[0])
            
            # Create impedance command for left motor only
            msg = ImpedanceCommand()
            msg.drive_ids = [left_hip_can_id]
            msg.kp = [left_kp]
            msg.kd = [left_kd]
            msg.max_output = [left_max_output]
            
            # Publish impedance parameters
            self.master.publisher_impedance_params.publish(msg)
            self.master.get_logger().info(f'🔧 Left motor impedance: kp={left_kp:.2f}, kd={left_kd:.3f}, max_output={left_max_output:.1f}')
            
            # Send offset torque via MotionCommand
            motion_msg = MotionCommand()
            motion_msg.drive_ids = [left_hip_can_id]
            motion_msg.target_position = [0.0]
            motion_msg.target_velocity = [0.0]
            motion_msg.target_torque = [left_offset]
            
            self.master.publisher_motion_commands.publish(motion_msg)
            self.master.get_logger().info(f'🔧 Left motor offset torque: {left_offset:.2f} Nm')
            print(60*"=")
            print()
            
        except Exception as e:
            self.master.get_logger().error(f'❌ Left motor impedance parameter update failed: {str(e)}')
    
    # Update impedance parameters for right motor only (button-triggered)
    def update_impedance_parameters_right(self):
        """Update impedance parameters for right motor only when button is pressed"""

        print()
        print("="*60)
        self.master.get_logger().info("🔧 Right motor impedance parameter update triggered (button)")
        try:
            # Check control mode
            if not hasattr(self, '_current_control_mode') or self._current_control_mode != "IMPEDANCE":
                self.master.get_logger().warn('⚠️ Not in IMPEDANCE mode; skipping right motor parameter update')
                return
            
            # Check if right motor is enabled
            if not (hasattr(self, '_right_motor_enabled') and self._right_motor_enabled):
                self.master.get_logger().warn('⚠️ Right motor not enabled; skipping parameter update')
                return
            
            # Get configuration defaults
            kp_defaults = self.config.get_kp_gains()
            kd_defaults = self.config.get_kd_gains()
            max_output_defaults = self.config.get_max_output_torque()
            offset_defaults = self.config.get_offset_torque()
            
            # Get right motor values from UI (index 1 or fallback to index 0)
            offset_index = min(1, len(kp_defaults) - 1)
            right_kp = float(self.ui.right_hip_kp_box.value()) if hasattr(self.ui, 'right_hip_kp_box') else kp_defaults[offset_index]
            right_kd = float(self.ui.right_hip_kd_box.value()) if hasattr(self.ui, 'right_hip_kd_box') else kd_defaults[offset_index]
            right_offset = float(self.ui.right_hip_offset_box.value()) if hasattr(self.ui, 'right_hip_offset_box') else offset_defaults[offset_index]
            right_max_output = float(max_output_defaults[offset_index])
            
            # Create impedance command for right motor only
            msg = ImpedanceCommand()
            msg.drive_ids = [right_hip_can_id]
            msg.kp = [right_kp]
            msg.kd = [right_kd]
            msg.max_output = [right_max_output]
            
            # Publish impedance parameters
            self.master.publisher_impedance_params.publish(msg)
            self.master.get_logger().info(f'🔧 Right motor impedance: kp={right_kp:.2f}, kd={right_kd:.3f}, max_output={right_max_output:.1f}')
            
            # Send offset torque via MotionCommand
            motion_msg = MotionCommand()
            motion_msg.drive_ids = [right_hip_can_id]
            motion_msg.target_position = [0.0]
            motion_msg.target_velocity = [0.0]
            motion_msg.target_torque = [right_offset]
            
            self.master.publisher_motion_commands.publish(motion_msg)
            self.master.get_logger().info(f'🔧 Right motor offset torque: {right_offset:.2f} Nm')
            print(60*"=")
            print()
            
        except Exception as e:
            self.master.get_logger().error(f'❌ Right motor impedance parameter update failed: {str(e)}')

    # Manual setpoint for motors (handles different control modes)
    def slider_setpoint(self):
        # Check if motors are added and enabled before sending setpoints
        if not hasattr(self, '_motors_added') or not self._motors_added:
            self.master.get_logger().warn('⚠️ Cannot send setpoints: Motors not added yet!')
            return
            
        # Check if at least one motor is enabled
        left_enabled = hasattr(self, '_left_motor_enabled') and self._left_motor_enabled
        right_enabled = hasattr(self, '_right_motor_enabled') and self._right_motor_enabled
        
        if not left_enabled and not right_enabled:
            self.master.get_logger().warn('⚠️ Cannot send setpoints: No motors are enabled!')
            return
            
        # Check control mode and handle accordingly
        current_mode = getattr(self, '_current_control_mode', None)
        if current_mode == "POSITION_PID":
            # Skip manual setpoints if sinusoidal trajectories are active
            if getattr(self, '_sinusoidal_left_active', False) or getattr(self, '_sinusoidal_right_active', False):
                return  # Sinusoidal trajectories take precedence
            
            if self.ui.cadera_izq_setpoint_manual_btn.isChecked():
                setpoint_left = self.ui.cadera_izq_setpoint_box_manual.value()
            else:
                setpoint_left = 0.0

            if self.ui.cadera_der_setpoint_manual_btn.isChecked():
                setpoint_right = self.ui.cadera_der_setpoint_box_manual.value()
            else:
                setpoint_right = 0.0

            self.send_position_setpoint(setpoint_left, setpoint_right)
            
        elif current_mode == "VELOCITY_PID":
            if self.ui.cadera_izq_setpoint_manual_btn.isChecked():
                velocity_left = self.ui.cadera_izq_setpoint_box_manual.value()  # deg/s
            else:
                velocity_left = 0.0

            if self.ui.cadera_der_setpoint_manual_btn.isChecked():
                velocity_right = self.ui.cadera_der_setpoint_box_manual.value()  # deg/s
            else:
                velocity_right = 0.0

            self.send_velocity_setpoint(velocity_left, velocity_right)
            
        elif current_mode == "IMPEDANCE":
            # Impedance control is handled by the impedance tab, not manual tab
            self.master.get_logger().info('ℹ️ Impedance control active - use the Impedance tab for position references')
            
        else:
            self.master.get_logger().warn(f'⚠️ Cannot send setpoints: Current mode is {current_mode}. Switch to Position, Velocity, or Impedance Control first!')

    # Send position setpoints to motors
    def send_position_setpoint(self, setpoint_left, setpoint_right):
        # Determine which motors are enabled and should receive commands
        enabled_drives = []
        target_positions = []
        setpoint_info = []
        
        left_enabled = hasattr(self, '_left_motor_enabled') and self._left_motor_enabled
        right_enabled = hasattr(self, '_right_motor_enabled') and self._right_motor_enabled
        
        if left_enabled:
            enabled_drives.append(left_hip_can_id)
            target_positions.append(float(self.deg2rad(setpoint_left)))
            setpoint_info.append(f'Left={setpoint_left}°({self.deg2rad(setpoint_left):.3f}rad)')
            
        if right_enabled:
            enabled_drives.append(right_hip_can_id)
            target_positions.append(float(self.deg2rad(setpoint_right)))
            setpoint_info.append(f'Right={setpoint_right}°({self.deg2rad(setpoint_right):.3f}rad)')
        
        if not enabled_drives:
            self.master.get_logger().warn('⚠️ Cannot send position setpoint: No motors are enabled!')
            return
            
        msg = MotionCommand()
        msg.drive_ids = [int(d) for d in enabled_drives]  # Ensure int type
        msg.target_position = [float(p) for p in target_positions]  # Ensure float type
        msg.target_velocity = [0.0] * len(enabled_drives)
        msg.target_torque = [0.0] * len(enabled_drives)
        
        # Enhanced debugging information
        self.master.get_logger().info(f'📍 Sending position setpoint to enabled motors: {", ".join(setpoint_info)}')
        self.master.get_logger().info(f'📍 Drive IDs (type={type(msg.drive_ids[0]) if msg.drive_ids else "empty"}): {msg.drive_ids}')
        self.master.get_logger().info(f'📍 Target Positions: {msg.target_position}')
        self.master.get_logger().info(f'📍 Target Velocities: {msg.target_velocity}')
        self.master.get_logger().info(f'📍 Target Torques: {msg.target_torque}')
        if left_enabled:
            self.master.get_logger().info(f'📍 Left Current Position: {self.motor_cadera_izq_data.position:.1f}°, Error: {setpoint_left - self.motor_cadera_izq_data.position:.1f}°')
        if right_enabled:
            self.master.get_logger().info(f'📍 Right Current Position: {self.motor_cadera_der_data.position:.1f}°, Error: {setpoint_right - self.motor_cadera_der_data.position:.1f}°')
        
        self.master.get_logger().info('📍 Publishing to /md80/motion_command...')
        self.master.publisher_motion_commands.publish(msg)
        self.master.get_logger().info('📍 Message published successfully')

    # Send velocity setpoints to motors
    def send_velocity_setpoint(self, velocity_left, velocity_right):
        # Determine which motors are enabled and should receive commands
        enabled_drives = []
        target_velocities = []
        velocity_info = []
        
        left_enabled = hasattr(self, '_left_motor_enabled') and self._left_motor_enabled
        right_enabled = hasattr(self, '_right_motor_enabled') and self._right_motor_enabled
        
        if left_enabled:
            enabled_drives.append(left_hip_can_id)
            target_velocities.append(float(self.deg2rad(velocity_left)))
            velocity_info.append(f'Left={velocity_left}°/s({self.deg2rad(velocity_left):.3f}rad/s)')
            
        if right_enabled:
            enabled_drives.append(right_hip_can_id)
            target_velocities.append(float(self.deg2rad(velocity_right)))
            velocity_info.append(f'Right={velocity_right}°/s({self.deg2rad(velocity_right):.3f}rad/s)')
        
        if not enabled_drives:
            self.master.get_logger().warn('⚠️ Cannot send velocity setpoint: No motors are enabled!')
            return
            
        msg = MotionCommand()
        msg.drive_ids = enabled_drives
        msg.target_position = [0.0] * len(enabled_drives)  # Not used in velocity mode
        msg.target_velocity = target_velocities
        msg.target_torque = [0.0] * len(enabled_drives)
        
        # Enhanced debugging information
        self.master.get_logger().info(f'🏃 Sending velocity setpoint to enabled motors: {", ".join(velocity_info)}')
        self.master.get_logger().info(f'🏃 Drive IDs: {msg.drive_ids}, Target Velocities: {msg.target_velocity}')
        if left_enabled:
            self.master.get_logger().info(f'🏃 Left Current Velocity: {self.rad2deg(self.motor_cadera_izq_data.velocity):.1f}°/s')
        if right_enabled:
            self.master.get_logger().info(f'🏃 Right Current Velocity: {self.rad2deg(self.motor_cadera_der_data.velocity):.1f}°/s')
        
        self.master.publisher_motion_commands.publish(msg)

    # Legacy function for compatibility
    def send_setpoint_manual(self, setpoint_left, setpoint_right):
        """Legacy function - redirects to position setpoint"""
        self.send_position_setpoint(setpoint_left, setpoint_right)

    # Resets all manual control sliders to zero when switching control modes.
    def reset_all_sliders(self):
        """Reset all manual control sliders to 0 position"""
        try:
            # Reset right hip sliders  
            if hasattr(self.ui, 'cadera_der_setpoint_slider_manual'):
                self.ui.cadera_der_setpoint_slider_manual.setValue(0)
            if hasattr(self.ui, 'cadera_der_setpoint_box_manual'):
                self.ui.cadera_der_setpoint_box_manual.setValue(0)
                
            # Reset sinusoidal parameters
            if hasattr(self.ui, 'cadera_izq_amplitud_manual'):
                self.ui.cadera_izq_amplitud_manual.setValue(0)
            if hasattr(self.ui, 'cadera_izq_frecuencia_manual'):
                self.ui.cadera_izq_frecuencia_manual.setValue(0)
            if hasattr(self.ui, 'cadera_izq_offset_manual'):
                self.ui.cadera_izq_offset_manual.setValue(0)
                
            if hasattr(self.ui, 'cadera_der_amplitud_manual'):
                self.ui.cadera_der_amplitud_manual.setValue(0)
            if hasattr(self.ui, 'cadera_der_frecuencia_manual'):
                self.ui.cadera_der_frecuencia_manual.setValue(0)
            if hasattr(self.ui, 'cadera_der_offset_manual'):
                self.ui.cadera_der_offset_manual.setValue(0)
                
            # Reset cycles parameter
            if hasattr(self.ui, 'cadera_ciclos_manual'):
                self.ui.cadera_ciclos_manual.setValue(0)
                
            # Uncheck manual control radio buttons
            if hasattr(self.ui, 'cadera_izq_setpoint_manual_btn'):
                self.ui.cadera_izq_setpoint_manual_btn.setChecked(False)
            if hasattr(self.ui, 'cadera_der_setpoint_manual_btn'):
                self.ui.cadera_der_setpoint_manual_btn.setChecked(False)
                
            # Stop any active sinusoidal trajectories
            if hasattr(self.ui, 'cadera_der_setpoint_senoidal_btn'):
                self.ui.cadera_der_setpoint_senoidal_btn.setChecked(False)
                
            # Stop impedance sinusoidal trajectories
            self._impedance_sinusoidal_left_active = False
            self._impedance_sinusoidal_right_active = False
            self.check_stop_impedance_sinusoidal_timer()
                
            self.master.get_logger().info('🔄 All sliders and parameters reset to 0')
            
        except Exception as e:
            self.master.get_logger().error(f'Error resetting sliders: {str(e)}')    

    # Start the sinusoidal trajectory timer if not already running
    def start_sinusoidal_timer(self):
        if self._sinusoidal_timer is None:
            self._sinusoidal_timer = QTimer()
            self._sinusoidal_timer.timeout.connect(self.update_sinusoidal_trajectory)
            self._sinusoidal_timer.start(50)  # 20 Hz update rate
            self.master.get_logger().info('⏰ Sinusoidal timer started (20 Hz)')

    # Stop the sinusoidal timer if no trajectories are active
    def check_stop_sinusoidal_timer(self):
        if not self._sinusoidal_left_active and not self._sinusoidal_right_active:
            if self._sinusoidal_timer is not None:
                self._sinusoidal_timer.stop()
                self._sinusoidal_timer = None
                self.master.get_logger().info('⏰ Sinusoidal timer stopped')

    # Update sinusoidal trajectory positions
    def update_sinusoidal_trajectory(self):
        import math
        import time
        
        # Update time
        self._sinusoidal_time += 0.05  # 50ms increment
        
        # Calculate positions for active trajectories
        setpoint_left = 0.0
        setpoint_right = 0.0
        
        if self._sinusoidal_left_active:
            freq = self.ui.cadera_izq_frecuencia_manual.value()  # Hz
            amplitude = self.ui.cadera_izq_amplitud_manual.value()  # degrees
            offset = self.ui.cadera_izq_offset_manual.value()  # degrees
            cycles = self.ui.cadera_ciclos_manual.value()  # total cycles
            
            # Check if trajectory should end
            if cycles > 0 and self._sinusoidal_time * freq >= cycles:
                self.ui.cadera_setpoint_senoidal_btn.setChecked(False)
                return
                
            setpoint_left = offset + amplitude * math.sin(2 * math.pi * freq * self._sinusoidal_time)
            
        if self._sinusoidal_right_active:
            freq = self.ui.cadera_der_frecuencia_manual.value()  # Hz
            amplitude = self.ui.cadera_der_amplitud_manual.value()  # degrees
            offset = self.ui.cadera_der_offset_manual.value()  # degrees (assuming this exists)
            cycles = self.ui.cadera_ciclos_manual.value()  # total cycles (shared with left)
            
            # Check if trajectory should end
            if cycles > 0 and self._sinusoidal_time * freq >= cycles:
                self.ui.cadera_der_setpoint_senoidal_btn.setChecked(False)
                return
                
            # If right offset doesn't exist, use 0
            try:
                offset = self.ui.cadera_der_offset_manual.value()
            except AttributeError:
                offset = 0.0
                
            setpoint_right = offset + amplitude * math.sin(2 * math.pi * freq * self._sinusoidal_time)

        # Send the combined trajectory command
        if self._sinusoidal_left_active or self._sinusoidal_right_active:
            # Optional: Log trajectory progress every 1 second
            if int(self._sinusoidal_time * 20) % 20 == 0:  # Every 1 second at 20Hz
                self.master.get_logger().info(f'🌊 Sinusoidal trajectory t={self._sinusoidal_time:.1f}s: Left={setpoint_left:.1f}°, Right={setpoint_right:.1f}°')
            
            # Use the position setpoint function
            self.send_position_setpoint(setpoint_left, setpoint_right)

    # Updates impedance kp/kd/offset in real time when spinboxes change (impedance mode only).
    def update_impedance_parameters_realtime(self):
        """Update impedance parameters (kp, kd) in real-time when sliders change"""
        # Debug current control mode
        current_mode = getattr(self, '_current_control_mode', 'None')
        
        # Only update if we're actually in impedance mode
        if not hasattr(self, '_current_control_mode') or self._current_control_mode != "IMPEDANCE":
            self.master.get_logger().warn(f'⚠️ Impedance parameter update skipped - not in IMPEDANCE mode (current: {current_mode})')
            return
            
        # Check if motors are enabled (at least one)
        left_enabled = hasattr(self, '_left_motor_enabled') and self._left_motor_enabled
        right_enabled = hasattr(self, '_right_motor_enabled') and self._right_motor_enabled
        
        if not left_enabled and not right_enabled:
            self.master.get_logger().warn(f'⚠️ Impedance parameter update skipped - no motors enabled')
            return
            
        try:
            # Get current impedance parameters from UI (with config fallbacks)
            kp_defaults = self.config.get_kp_gains()
            kd_defaults = self.config.get_kd_gains() 
            max_output_defaults = self.config.get_max_output_torque()
            offset_defaults = self.config.get_offset_torque()
            
            # Build lists only for enabled motors
            enabled_motor_ids = []
            kp_values = []
            kd_values = []
            max_output_values = []
            offset_values = []
            motor_descriptions = []
            
            # Add left motor if enabled
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
                
            # Add right motor if enabled  
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
            
            # Only send parameters to enabled motors
            if enabled_motor_ids:
                # Create and publish impedance command for enabled motors only
                msg = ImpedanceCommand()
                msg.drive_ids = [int(d) for d in enabled_motor_ids]  # Only enabled motors
                msg.kp = kp_values
                msg.kd = kd_values
                msg.max_output = max_output_values
                
                # Direct publish - no service needed (consistent with position control approach)
                self.master.publisher_impedance_params.publish(msg)
                
                # Also send offset torque via MotionCommand for enabled motors only
                motion_msg = MotionCommand()
                motion_msg.drive_ids = [int(d) for d in enabled_motor_ids]  # Only enabled motors
                motion_msg.target_position = [0.0] * len(enabled_motor_ids)  # Not used in impedance mode
                motion_msg.target_velocity = [0.0] * len(enabled_motor_ids)  # Not used in impedance mode
                motion_msg.target_torque = offset_values
                
                self.master.publisher_motion_commands.publish(motion_msg)
                
                # Log the published parameters for enabled motors only
                motor_status = ', '.join(motor_descriptions)
                self.master.get_logger().info(f'🔧 Real-time impedance parameters sent to enabled motors: {motor_status}')
            
        except Exception as e:
            self.master.get_logger().error(f'❌ Real-time impedance parameter update failed: {str(e)}')
            self.master.get_logger().error(f'Error updating impedance parameters: {str(e)}')

    # Send position reference for impedance control from impedance tab UI.
    def _check_impedance_mode_conflict(self, motor_side):
        """Check if both manual and sinusoidal are enabled for same motor. Return True if conflict detected."""
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
        """Start impedance trajectory for left motor (button press).
        If sinusoidal is enabled, starts sinusoidal trajectory; otherwise enables manual control."""
        if not hasattr(self, '_motors_added') or not self._motors_added:
            self.master.get_logger().warn('⚠️ [IMP-LEFT] Cannot start trajectory: Motors not added yet!')
            return
            
        if not hasattr(self, '_left_motor_enabled') or not self._left_motor_enabled:
            self.master.get_logger().warn('⚠️ [IMP-LEFT] Cannot start trajectory: Left motor not enabled!')
            return
            
        if not hasattr(self, '_current_control_mode') or self._current_control_mode != "IMPEDANCE":
            self.master.get_logger().warn('⚠️ [IMP-LEFT] Cannot start trajectory: Not in IMPEDANCE mode!')
            return
        
        # Check if sinusoidal is enabled - if so, start sinusoidal trajectory
        sine_enabled = hasattr(self.ui, 'left_hip_sine_enable') and self.ui.left_hip_sine_enable.isChecked()
        
        if sine_enabled:
            # Check for conflict first
            if self._check_impedance_mode_conflict('left'):
                self.master.get_logger().error('❌ [IMP-LEFT] Cannot start: manual and sine both enabled!')
                return
            
            # CRITICAL: Activate trajectory flag so timer doesn't immediately stop it
            self._impedance_trajectory_left_active = True
            self._impedance_sinusoidal_left_active = True
            self._impedance_sinusoidal_time_left = 0.0
            self.start_impedance_sinusoidal_timer_left()
            self.master.get_logger().info('✅ [IMP-LEFT] Sinusoidal trajectory started with button')
        else:
            # Manual mode - just enable trajectory flag
            self._impedance_trajectory_left_active = True
            self.master.get_logger().info('✅ [IMP-LEFT] Manual trajectory started - motor will respond to manual control')
    
    def stop_impedance_trajectory_left(self):
        """Stop impedance trajectory for left motor (button press)."""
        self._impedance_trajectory_left_active = False
        # Also stop any running sinusoidal on left side
        self._impedance_sinusoidal_left_active = False
        
        # Block signals to prevent cascading callbacks during cleanup
        if hasattr(self.ui, 'left_hip_sine_enable'):
            self.ui.left_hip_sine_enable.blockSignals(True)
            self.ui.left_hip_sine_enable.setChecked(False)
            self.ui.left_hip_sine_enable.blockSignals(False)
        
        self.check_stop_impedance_sinusoidal_timer()
        self.master.get_logger().info('🛑 [IMP-LEFT] Trajectory stopped - ready to restart')
    
    def start_impedance_trajectory_right(self):
        """Start impedance trajectory for right motor (button press).
        If sinusoidal is enabled, starts sinusoidal trajectory; otherwise enables manual control."""
        if not hasattr(self, '_motors_added') or not self._motors_added:
            self.master.get_logger().warn('⚠️ [IMP-RIGHT] Cannot start trajectory: Motors not added yet!')
            return
            
        if not hasattr(self, '_right_motor_enabled') or not self._right_motor_enabled:
            self.master.get_logger().warn('⚠️ [IMP-RIGHT] Cannot start trajectory: Right motor not enabled!')
            return
            
        if not hasattr(self, '_current_control_mode') or self._current_control_mode != "IMPEDANCE":
            self.master.get_logger().warn('⚠️ [IMP-RIGHT] Cannot start trajectory: Not in IMPEDANCE mode!')
            return
        
        # Check if sinusoidal is enabled - if so, start sinusoidal trajectory
        sine_enabled = hasattr(self.ui, 'right_hip_sine_enable') and self.ui.right_hip_sine_enable.isChecked()
        
        if sine_enabled:
            # Check for conflict first
            if self._check_impedance_mode_conflict('right'):
                self.master.get_logger().error('❌ [IMP-RIGHT] Cannot start: manual and sine both enabled!')
                return
            
            # CRITICAL: Activate trajectory flag so timer doesn't immediately stop it
            self._impedance_trajectory_right_active = True
            self._impedance_sinusoidal_right_active = True
            self._impedance_sinusoidal_time_right = 0.0
            self.start_impedance_sinusoidal_timer_right()
            self.master.get_logger().info('✅ [IMP-RIGHT] Sinusoidal trajectory started with button')
        else:
            # Manual mode - just enable trajectory flag
            self._impedance_trajectory_right_active = True
            self.master.get_logger().info('✅ [IMP-RIGHT] Manual trajectory started - motor will respond to manual control')
    
    def stop_impedance_trajectory_right(self):
        """Stop impedance trajectory for right motor (button press)."""
        self._impedance_trajectory_right_active = False
        # Also stop any running sinusoidal on right side
        self._impedance_sinusoidal_right_active = False
        
        # Block signals to prevent cascading callbacks during cleanup
        if hasattr(self.ui, 'right_hip_sine_enable'):
            self.ui.right_hip_sine_enable.blockSignals(True)
            self.ui.right_hip_sine_enable.setChecked(False)
            self.ui.right_hip_sine_enable.blockSignals(False)
        
        self.check_stop_impedance_sinusoidal_timer()
        self.master.get_logger().info('🛑 [IMP-RIGHT] Trajectory stopped - ready to restart')

    def handle_impedance_manual_enable_left(self, checked):
        """When manual enable checkbox is toggled, auto-enable trajectory flag."""
        print()
        print(60*"=")
        if checked:
            # User enabled manual control - automatically activate trajectory flag
            self._impedance_trajectory_left_active = True
            self.master.get_logger().info('✅ [IMP-LEFT] Manual mode activated - trajectory flag enabled automatically')
        else:
            # User disabled manual control - deactivate trajectory flag
            self._impedance_trajectory_left_active = False
            self.master.get_logger().info('🛑 [IMP-LEFT] Manual mode deactivated - trajectory flag disabled')
        
        # Trigger position setpoint update
        self.impedance_position_setpoint()

    def handle_impedance_manual_enable_right(self, checked):
        """When manual enable checkbox is toggled, auto-enable trajectory flag."""
        print()
        print(60*"=")
        if checked:
            # User enabled manual control - automatically activate trajectory flag
            self._impedance_trajectory_right_active = True
            self.master.get_logger().info('✅ [IMP-RIGHT] Manual mode activated - trajectory flag enabled automatically')
        else:
            # User disabled manual control - deactivate trajectory flag
            self._impedance_trajectory_right_active = False
            self.master.get_logger().info('🛑 [IMP-RIGHT] Manual mode deactivated - trajectory flag disabled')
        
        # Trigger position setpoint update
        self.impedance_position_setpoint()

    def impedance_position_setpoint(self):
        if not hasattr(self, '_current_control_mode') or self._current_control_mode != "IMPEDANCE":
            return
            
        motors_added = getattr(self, '_motors_added', False)
        motors_enabled = getattr(self, '_motors_enabled', False)

        if not motors_added or not motors_enabled:
            self.master.get_logger().info(f'[IMP] Blocked: motors_added={motors_added}, motors_enabled={motors_enabled}')
            return

        try:
            setpoint_left = None
            setpoint_right = None

            # Left Hip Logic
            left_manual_enabled = hasattr(self.ui, 'left_hip_manual_enable') and self.ui.left_hip_manual_enable.isChecked()
            left_trajectory_active = getattr(self, '_impedance_trajectory_left_active', False)
            left_sine_active = getattr(self, '_impedance_sinusoidal_left_active', False)
            left_conflict = self._check_impedance_mode_conflict('left')

            if left_manual_enabled and left_trajectory_active and not left_sine_active and not left_conflict:
                if hasattr(self.ui, 'left_hip_manual_box'):
                    setpoint_left = self.ui.left_hip_manual_box.value()

            # Right Hip Logic
            right_manual_enabled = hasattr(self.ui, 'right_hip_manual_enable') and self.ui.right_hip_manual_enable.isChecked()
            right_trajectory_active = getattr(self, '_impedance_trajectory_right_active', False)
            right_sine_active = getattr(self, '_impedance_sinusoidal_right_active', False)
            right_conflict = self._check_impedance_mode_conflict('right')

            if right_manual_enabled and right_trajectory_active and not right_sine_active and not right_conflict:
                if hasattr(self.ui, 'right_hip_manual_box'):
                    setpoint_right = self.ui.right_hip_manual_box.value()

            # Only publish if at least one side has a new valid manual setpoint
            if setpoint_left is not None or setpoint_right is not None:
                self.master.get_logger().info(f'[IMP] Sending Manual Refs: L={setpoint_left if setpoint_left is not None else "SKIP"}, R={setpoint_right if setpoint_right is not None else "SKIP"}')
                self.send_impedance_position_reference(setpoint_left, setpoint_right)
            else:
                # Helpful debug to see which flag is blocking the update
                self.master.get_logger().info(f'[IMP] No command sent. L_man={left_manual_enabled}, L_traj={left_trajectory_active}, L_sine={left_sine_active} | R_man={right_manual_enabled}, R_traj={right_trajectory_active}, R_sine={right_sine_active}')

        except Exception as e:
            self.master.get_logger().error(f'Error in impedance_position_setpoint: {str(e)}')
            import traceback
            self.master.get_logger().error(traceback.format_exc())

    # Send position reference commands for impedance control
    def send_impedance_position_reference(self, setpoint_left=None, setpoint_right=None):        
        # Determine which motors are enabled and should receive commands
        enabled_drives = []
        target_positions = []
        target_torques = []
        
        # Get offset torque values from config
        offset_defaults = self.config.get_offset_torque()
        
        left_enabled = hasattr(self, '_left_motor_enabled') and self._left_motor_enabled
        right_enabled = hasattr(self, '_right_motor_enabled') and self._right_motor_enabled
        
        # SOLUCIÓN: Solo agregamos el motor al mensaje si su setpoint NO es None
        if left_enabled and setpoint_left is not None:
            enabled_drives.append(left_hip_can_id)
            target_positions.append(float(self.deg2rad(setpoint_left)))
            left_offset = float(self.ui.left_hip_offset_box.value()) if hasattr(self.ui, 'left_hip_offset_box') else offset_defaults[0]
            target_torques.append(float(left_offset))
            
        if right_enabled and setpoint_right is not None:
            enabled_drives.append(right_hip_can_id)
            target_positions.append(float(self.deg2rad(setpoint_right)))
            offset_index = min(1, len(offset_defaults) - 1)
            right_offset = float(self.ui.right_hip_offset_box.value()) if hasattr(self.ui, 'right_hip_offset_box') else offset_defaults[offset_index]
            target_torques.append(float(right_offset))
        
        if not enabled_drives:
            # Si no hay motores que actualizar, no publicamos para evitar saturar el topic
            return
        
        msg = MotionCommand()
        msg.drive_ids = enabled_drives
        msg.target_position = target_positions
        msg.target_velocity = [0.0] * len(enabled_drives)
        msg.target_torque = target_torques 
        
        self.master.publisher_motion_commands.publish(msg)

    # Start or stop sinusoidal trajectory for left motor in impedance mode
    def start_impedance_sinusoidal_trajectory_left(self, checked):
        if checked:
            if not hasattr(self, '_motors_added') or not self._motors_added:
                self.master.get_logger().warn('⚠️ Cannot enable sinusoidal: Motors not added yet!')
                self.ui.left_hip_sine_enable.setChecked(False)
                return
                
            if not hasattr(self, '_left_motor_enabled') or not self._left_motor_enabled:
                self.master.get_logger().warn('⚠️ Cannot enable sinusoidal: Left motor not enabled!')
                self.ui.left_hip_sine_enable.setChecked(False)
                return
                
            if not hasattr(self, '_current_control_mode') or self._current_control_mode != "IMPEDANCE":
                self.master.get_logger().warn('⚠️ Cannot enable sinusoidal: Not in Impedance Control mode!')
                self.ui.left_hip_sine_enable.setChecked(False)
                return
            
            # Check for conflict: if manual is also enabled
            if self._check_impedance_mode_conflict('left'):
                self.ui.left_hip_sine_enable.setChecked(False)
                return
            
            print()
            print(60*"=")
            self.master.get_logger().info('🌊 [IMP-LEFT] Sinusoidal mode enabled - press start button to begin trajectory')
        else:
            # Disable sinusoidal
            if self._impedance_sinusoidal_left_active:
                self._impedance_sinusoidal_left_active = False
                self.check_stop_impedance_sinusoidal_timer()
            self.master.get_logger().info('🛑 [IMP-LEFT] Sinusoidal mode disabled')

    # Start or stop sinusoidal trajectory for right motor in impedance mode
    def start_impedance_sinusoidal_trajectory_right(self, checked):
        if checked:
            if not hasattr(self, '_motors_added') or not self._motors_added:
                self.master.get_logger().warn('⚠️ Cannot enable sinusoidal: Motors not added yet!')
                self.ui.right_hip_sine_enable.setChecked(False)
                return
                
            if not hasattr(self, '_right_motor_enabled') or not self._right_motor_enabled:
                self.master.get_logger().warn('⚠️ Cannot enable sinusoidal: Right motor not enabled!')
                self.ui.right_hip_sine_enable.setChecked(False)
                return
                
            if not hasattr(self, '_current_control_mode') or self._current_control_mode != "IMPEDANCE":
                self.master.get_logger().warn('⚠️ Cannot enable sinusoidal: Not in Impedance Control mode!')
                self.ui.right_hip_sine_enable.setChecked(False)
                return
            
            # Check for conflict: if manual is also enabled
            if self._check_impedance_mode_conflict('right'):
                self.ui.right_hip_sine_enable.setChecked(False)
                return
            
            print()
            print(60*"=")
            self.master.get_logger().info('🌊 [IMP-RIGHT] Sinusoidal mode enabled - press start button to begin trajectory')
        else:
            # Disable sinusoidal
            if self._impedance_sinusoidal_right_active:
                self._impedance_sinusoidal_right_active = False
                self.check_stop_impedance_sinusoidal_timer()
            self.master.get_logger().info('🛑 [IMP-RIGHT] Sinusoidal mode disabled')

    # Start the impedance sinusoidal trajectory timer if not already running
    def start_impedance_sinusoidal_timer_left(self):
        if self._impedance_sinusoidal_timer is None:
            self._impedance_sinusoidal_timer = QTimer()
            self._impedance_sinusoidal_timer.timeout.connect(self.update_impedance_sinusoidal_trajectory)
            self._impedance_sinusoidal_timer.start(50)  # 20 Hz update rate
            self.master.get_logger().info('⏰ Impedance sinusoidal timer started (20 Hz)')

    def start_impedance_sinusoidal_timer_right(self):
        if self._impedance_sinusoidal_timer is None:
            self._impedance_sinusoidal_timer = QTimer()
            self._impedance_sinusoidal_timer.timeout.connect(self.update_impedance_sinusoidal_trajectory)
            self._impedance_sinusoidal_timer.start(50)  # 20 Hz update rate
            self.master.get_logger().info('⏰ Impedance sinusoidal timer started (20 Hz)')

    # Stop the impedance sinusoidal timer if no trajectories are active
    def check_stop_impedance_sinusoidal_timer(self):
        if not self._impedance_sinusoidal_left_active and not self._impedance_sinusoidal_right_active:
            if self._impedance_sinusoidal_timer is not None:
                self._impedance_sinusoidal_timer.stop()
                self._impedance_sinusoidal_timer = None
                self.master.get_logger().info('⏰ Impedance sinusoidal timer stopped')

    # Update impedance sinusoidal trajectory positions
    def update_impedance_sinusoidal_trajectory(self):
        import math
        
        setpoint_left = None
        setpoint_right = None
        
        # Checks de parada...
        if self._impedance_sinusoidal_left_active and not getattr(self, '_impedance_trajectory_left_active', False):
            self._impedance_sinusoidal_left_active = False
            if hasattr(self.ui, 'left_hip_sine_enable'): self.ui.left_hip_sine_enable.setChecked(False)
            self.check_stop_impedance_sinusoidal_timer()

        if self._impedance_sinusoidal_right_active and not getattr(self, '_impedance_trajectory_right_active', False):
            self._impedance_sinusoidal_right_active = False
            if hasattr(self.ui, 'right_hip_sine_enable'): self.ui.right_hip_sine_enable.setChecked(False)
            self.check_stop_impedance_sinusoidal_timer()

        # Izquierda
        if self._impedance_sinusoidal_left_active and not self._check_impedance_mode_conflict('left'):
            self._impedance_sinusoidal_time_left += 0.05
            try:
                freq = self.ui.left_hip_sine_freq.value() if hasattr(self.ui, 'left_hip_sine_freq') else 0.5
                amplitude = self.ui.left_hip_sine_amp.value() if hasattr(self.ui, 'left_hip_sine_amp') else 10.0
                offset = self.ui.left_hip_sine_offset.value() if hasattr(self.ui, 'left_hip_sine_offset') else 0.0
                cycles = self.ui.left_hip_sine_cycles.value() if hasattr(self.ui, 'left_hip_sine_cycles') else 0
                
                if cycles > 0 and self._impedance_sinusoidal_time_left * freq >= cycles:
                    self._impedance_sinusoidal_left_active = False
                    self.ui.left_hip_sine_enable.setChecked(False)
                    self.check_stop_impedance_sinusoidal_timer()
                else:
                    setpoint_left = offset + amplitude * math.sin(2 * math.pi * freq * self._impedance_sinusoidal_time_left)
            except Exception:
                setpoint_left = 10.0 * math.sin(2 * math.pi * 0.5 * self._impedance_sinusoidal_time_left)

        # Derecha
        if self._impedance_sinusoidal_right_active and not self._check_impedance_mode_conflict('right'):
            self._impedance_sinusoidal_time_right += 0.05
            try:
                freq = self.ui.right_hip_sine_freq.value() if hasattr(self.ui, 'right_hip_sine_freq') else 0.5
                amplitude = self.ui.right_hip_sine_amp.value() if hasattr(self.ui, 'right_hip_sine_amp') else 10.0
                offset = self.ui.right_hip_sine_offset.value() if hasattr(self.ui, 'right_hip_sine_offset') else 0.0
                cycles = self.ui.right_hip_sine_cycles.value() if hasattr(self.ui, 'right_hip_sine_cycles') else 0
                
                if cycles > 0 and self._impedance_sinusoidal_time_right * freq >= cycles:
                    self._impedance_sinusoidal_right_active = False
                    self.ui.right_hip_sine_enable.setChecked(False)
                    self.check_stop_impedance_sinusoidal_timer()
                else:
                    setpoint_right = offset + amplitude * math.sin(2 * math.pi * freq * self._impedance_sinusoidal_time_right)
            except Exception:
                setpoint_right = 10.0 * math.sin(2 * math.pi * 0.5 * self._impedance_sinusoidal_time_right)

        # Publicar solo los válidos
        if setpoint_left is not None or setpoint_right is not None:
            self.send_impedance_position_reference(setpoint_left, setpoint_right)

        # Send the combined impedance trajectory command only if trajectories remain active
        left_traj_active = self._impedance_sinusoidal_left_active and getattr(self, '_impedance_trajectory_left_active', False)
        right_traj_active = self._impedance_sinusoidal_right_active and getattr(self, '_impedance_trajectory_right_active', False)

        if left_traj_active or right_traj_active:
            # Log trajectory progress every 1 second
            # Every 1 second at 20Hz print info using the separate times
            if self._impedance_sinusoidal_left_active and int(self._impedance_sinusoidal_time_left * 20) % 20 == 0:
                self.master.get_logger().info(f"[LEFT] Impedance trajectory active. Ref: {setpoint_left:.2f} deg")
                
            if self._impedance_sinusoidal_right_active and int(self._impedance_sinusoidal_time_right * 20) % 20 == 0:
                self.master.get_logger().info(f"[RIGHT] Impedance trajectory active. Ref: {setpoint_right:.2f} deg")
            
            # Use the impedance position reference function
            self.send_impedance_position_reference(setpoint_left, setpoint_right)

    def launch_sim_mimo(self):
        ciclos = self.ui.cyclesValueBox.value()
        kp = self.ui.kpValueBox.value()
        ki = self.ui.kiValueBox.value()
        kd = self.ui.kdValueBox.value()
        mass_scale = self.ui.massValueBox.value()
        joint_stiffness = self.ui.jointStiffnessValueBox.value()
        joint_damping = self.ui.jointDampingValueBox.value()
        joint_frictionloss = self.ui.jointFrictionlossValueBox.value()
        joint_armature = self.ui.jointArmatureValueBox.value()
        
        self.mimo_sim_process = QProcess(self)
        conda_python = "/home/carlos/miniconda3/envs/mimo/bin/python"
        script_path = (
            Path(__file__).resolve().parent / ".." / ".." / ".." / ".." / ".." / ".." / ".." / ".." / "mimo" / "scripts" / "visualiza_standup_trajectory_tracking.py"
        ).resolve()
        
        print(f"Launching MIMO simulation with script: {script_path}")
        
        self.mimo_sim_process.errorOccurred.connect(self.handle_mimo_error)
        self.mimo_sim_process.finished.connect(self.handle_mimo_finished)
        
        self.mimo_sim_process.setProcessChannelMode(QProcess.ProcessChannelMode.MergedChannels)
        self.mimo_sim_process.readyReadStandardOutput.connect(self.handle_mimo_output)
        
        script_dir = script_path.parent
        self.mimo_sim_process.setWorkingDirectory(str(script_dir))
        
        self.mimo_sim_process.start(
            conda_python,
            [
                str(script_path),
                "--cycles", str(ciclos),
                "--kp", str(kp),
                "--ki", str(ki),
                "--kd", str(kd),
                "--mass-scale", str(mass_scale),
                "--joint-stiffness-scale", str(joint_stiffness),
                "--joint-damping-scale", str(joint_damping),
                "--joint-frictionloss-scale", str(joint_frictionloss),
                "--joint-armature-scale", str(joint_armature)
            ]
        )
        
        if not self.mimo_sim_process.waitForStarted():
            print("ERROR: No se pudo iniciar el proceso")
            print(f"Error: {self.mimo_sim_process.errorString()}")

    def handle_mimo_error(self, error):
        print(f"QProcess ERROR: {error}")
        print(f"Error string: {self.mimo_sim_process.errorString()}")

    def handle_mimo_finished(self):
        exit_code = self.mimo_sim_process.exitCode()
        exit_status = self.mimo_sim_process.exitStatus()
        print(f"MIMO simulation finished. Exit code: {exit_code}, Status: {exit_status}")

    def handle_mimo_output(self):
        output = self.mimo_sim_process.readAllStandardOutput().data().decode()
        print(f"MIMO OUTPUT: {output}")

    def launch_real_mimo(self):
        motor_id = self.ui.selector_motor_id_mimo_real.value()
        if self.ui.selector_apply_torque_true.isChecked():
            apply_torque = True
        else:
            apply_torque = False
        
        path_csv = self.ui.carpeta_csv_origen_mimo_real.text() + self.ui.nombre_csv_origen_mimo_real.text()
        print(f"Launching real MIMO with motor_id={motor_id}, apply_torque={apply_torque}, path_csv={path_csv}")

        conda_python = "/home/carlos/miniconda3/envs/mimo312/bin/python"

        script_path = (
            Path(__file__).resolve().parent / ".." / ".." / ".." / ".." / ".." / ".." / ".." / ".." / "mimo" / "scripts" / "hip_motor_torque_replay_runner_pycandle.py"
        ).resolve()

        self.mimo_real_process = QProcess(self)
        script_dir = script_path.parent
        self.mimo_real_process.setWorkingDirectory(str(script_dir))

        self.mimo_real_process.start(
            conda_python,
            [
                str(script_path),
                "--source-csv", str(path_csv),
                "--motor-id", str(motor_id),
                "--apply-torque", str(apply_torque),
                "--source-hip right",
                "--interface pycandle"
            ]
        )
        
        if not self.mimo_real_process.waitForStarted():
            print("ERROR: No se pudo iniciar el proceso")
            print(f"Error: {self.mimo_real_process.errorString()}")

    def select_folder_mimo_real(self):
        folder_path = QFileDialog.getExistingDirectory(self, "Select folder", "")
        if folder_path:
            self.ui.carpeta_csv_origen_mimo_real.setText(folder_path)

    def launch_training(self):
        timesteps = self.ui.episodios_training.value()
        checkpoints_freq = self.ui.checkpoints_freq_training.value()

        conda_python = "/home/carlos/miniconda3/envs/scone/bin/python"

        script_path = (
            Path(__file__).resolve().parent / ".." / ".." / ".." / ".." / ".." / ".." / ".." / ".." / "entrenamiento" / "training.py"
        ).resolve()

        self.training_process = QProcess(self)
        script_dir = script_path.parent
        self.training_process.setWorkingDirectory(str(script_dir))
        
        # ✅ Configurar variables de entorno para unbuffered output
        env = QProcessEnvironment.systemEnvironment()
        env.insert("PYTHONUNBUFFERED", "1")
        self.training_process.setProcessEnvironment(env)
        
        self.training_process.readyReadStandardOutput.connect(self.handle_training_output)
        self.training_process.readyReadStandardError.connect(self.handle_training_output)
        self.training_process.finished.connect(self.handle_training_finished)

        self.training_process.start(
            conda_python,
            [
                str(script_path),
                "--total-timesteps", str(timesteps),
                "--checkpoint-freq", str(checkpoints_freq),
                "--save-scone-episodes",
                "--progress-bar"
            ]
        )

    def handle_training_output(self):
        """Captura y muestra la salida en tiempo real"""
        output = self.training_process.readAllStandardOutput().data().decode()
        
        if not output:
            return
        
        # Limpiar códigos ANSI
        ansi_escape = re.compile(r'\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])')
        clean = ansi_escape.sub('', output)
        
        text_edit = self.ui.texto_salida_entrenamiento
        
        # Si hay \r (carriage return), actualizar última línea
        if '\r' in clean:
            # Obtener la última línea visible
            cursor = text_edit.textCursor()
            cursor.movePosition(QTextCursor.MoveOperation.End)
            
            # Ir a inicio de línea
            cursor.movePosition(QTextCursor.MoveOperation.StartOfLine)
            cursor.movePosition(QTextCursor.MoveOperation.End, QTextCursor.MoveMode.KeepAnchor)
            
            # Obtener texto de última línea
            last_line_text = cursor.selectedText()
            
            # Actualizar con el nuevo progress
            parts = clean.split('\r')
            new_progress = parts[-1].strip()
            
            cursor.removeSelectedText()
            cursor.insertText(new_progress)
            text_edit.setTextCursor(cursor)
        else:
            # Agregar como nueva línea
            if clean.strip():
                text_edit.appendPlainText(clean.strip())
        
        # Auto-scroll
        scrollbar = text_edit.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

    def handle_training_finished(self):
        """Se ejecuta cuando el proceso termina"""
        exit_code = self.training_process.exitCode()
        self.ui.texto_salida_entrenamiento.appendPlainText(f"\n✅ Entrenamiento finalizado (Exit code: {exit_code})")

    def update_last_line(self, new_text):
        """Actualiza la última línea sin crear una nueva"""
        text_edit = self.ui.texto_salida_entrenamiento
        cursor = text_edit.textCursor()
        
        # Mover al final del documento
        cursor.movePosition(QTextCursor.MoveOperation.End)
        
        # Seleccionar toda la última línea
        cursor.select(QTextCursor.SelectionType.LineUnderCursor)
        
        # Reemplazar con el nuevo texto del progress bar
        cursor.removeSelectedText()
        cursor.insertText(new_text)
        
        text_edit.setTextCursor(cursor)

    # Starts rosbag recording with auto-numbered filename in the chosen folder.
    def start_recording(self):
        print("start_recording")
        if self.rosbag_process is None:
            filename = self.ui.nombre_archivo_guardado.text()
            if self.ui.nombre_carpeta_guardado.text() == "":
                path = "/home/carlos/Escritorio/assistive_exo/src/rosbags/rosbags"
            else:
                path = self.ui.nombre_carpeta_guardado.text()

            if filename[-1] != "_":
                filename = self.ui.nombre_archivo_guardado.text() + "_"

            file_number = self.get_next_rosbag_number(path, filename)
            self.rosbag_process = QProcess(self)
            command = ['ros2', 'bag', 'record', '-a', '-o', os.path.join(path, f"{filename}{file_number:03}")]
            self.rosbag_process.start(command[0], command[1:])
            
            # Wait 1 second for rosbag to create the directory, then write metadata.txt inside
            QTimer.singleShot(1000, lambda: self._write_rosbag_metadata_txt(path, filename, file_number))
        print(self.rosbag_process)

    # Stop rosbag recording
    def stop_recording(self):
        print("stop_recording")
        if self.rosbag_process is not None:
            self.rosbag_process.terminate()
            self.rosbag_process = None
        print(self.rosbag_process)

    def _read_ui_value(self, attr):
        if hasattr(self.ui, attr):
            try:
                return float(getattr(self.ui, attr).value())
            except Exception:
                return "N/A"
        return "N/A"

    def _write_rosbag_metadata_txt(self, path, filename, file_number):
        try:
            # rosbag creates the directory, we just write inside it
            rosbag_dir = os.path.join(path, f"{filename}{file_number:03}")
            txt_path = os.path.join(rosbag_dir, "metadata.txt")

            mode = getattr(self, '_current_control_mode', 'UNKNOWN')
            timestamp = datetime.now().isoformat(timespec="seconds")

            left_kp = self._read_ui_value('left_hip_kp_box')
            left_kd = self._read_ui_value('left_hip_kd_box')
            left_offset = self._read_ui_value('left_hip_offset_box')
            right_kp = self._read_ui_value('right_hip_kp_box')
            right_kd = self._read_ui_value('right_hip_kd_box')
            right_offset = self._read_ui_value('right_hip_offset_box')

            max_outputs = self.config.get_max_output_torque() if hasattr(self, 'config') else []
            left_max_output = max_outputs[0] if len(max_outputs) > 0 else "N/A"
            right_max_output = max_outputs[1] if len(max_outputs) > 1 else "N/A"

            pos_left_freq = self._read_ui_value('cadera_izq_frecuencia_manual')
            pos_left_amp = self._read_ui_value('cadera_izq_amplitud_manual')
            pos_left_offset = self._read_ui_value('cadera_izq_offset_manual')
            pos_right_freq = self._read_ui_value('cadera_der_frecuencia_manual')
            pos_right_amp = self._read_ui_value('cadera_der_amplitud_manual')
            pos_right_offset = self._read_ui_value('cadera_der_offset_manual')
            pos_cycles = self._read_ui_value('cadera_ciclos_manual')

            imp_left_freq = self._read_ui_value('left_hip_sine_freq')
            imp_left_amp = self._read_ui_value('left_hip_sine_amp')
            imp_left_offset = self._read_ui_value('left_hip_sine_offset')
            imp_left_cycles = self._read_ui_value('left_hip_sine_cycles')
            imp_right_freq = self._read_ui_value('right_hip_sine_freq')
            imp_right_amp = self._read_ui_value('right_hip_sine_amp')
            imp_right_offset = self._read_ui_value('right_hip_sine_offset')
            imp_right_cycles = self._read_ui_value('right_hip_sine_cycles')

            lines = [
                "RECORDING_METADATA",
                f"timestamp: {timestamp}",
                f"control_mode: {mode}",
                "IMPEDANCE_GAINS:",
                f"  left_kp: {left_kp}",
                f"  left_kd: {left_kd}",
                f"  left_offset: {left_offset}",
                f"  left_max_output: {left_max_output}",
                f"  -------------",
                f"  right_kp: {right_kp}",
                f"  right_kd: {right_kd}",
                f"  right_offset: {right_offset}",
                f"  right_max_output: {right_max_output}",
                "IMPEDANCE_SINE:",
                f"  left_amp_deg: {imp_left_amp}",
                f"  left_freq_hz: {imp_left_freq}",
                f"  left_offset_deg: {imp_left_offset}",
                f"  left_cycles: {imp_left_cycles}",
                f"  -------------",
                f"  right_amp_deg: {imp_right_amp}",
                f"  right_freq_hz: {imp_right_freq}",
                f"  right_offset_deg: {imp_right_offset}",
                f"  right_cycles: {imp_right_cycles}",
            ]

            with open(txt_path, 'w', encoding='utf-8') as txt_file:
                txt_file.write("\n".join(lines) + "\n")

            self.master.get_logger().info(f'Metadata txt saved: {txt_path}')
        except Exception as e:
            self.master.get_logger().warn(f'Failed to write metadata txt: {e}')

    # Scans folder for matching prefix and returns next sequential rosbag number.
    def get_next_rosbag_number(self, folder, name):
        files = [f for f in os.listdir(folder) if f.startswith(name)]
        if not files:
            return 1
        numbers = []
        for f in files:
            try:
                number = int(f.split('_')[-1])
                numbers.append(number)
            except ValueError:
                continue
        if not numbers:
            return 1
        next_number = max(numbers) + 1
        return next_number

    # Open folder selection dialog
    def select_folder_to_save(self):
        folder_path = QFileDialog.getExistingDirectory(self, "Select folder", "")
        if folder_path:
            self.ui.nombre_carpeta_guardado.setText(folder_path)

    # Send impedance trajectory command for left motor only
    def send_impedance_trajectory_left(self, state):
        if state:
            # Start left motor impedance trajectory
            if hasattr(self.ui, 'left_hip_sine_enable'):
                self.ui.left_hip_sine_enable.setChecked(True)
        else:
            # Stop left motor impedance trajectory
            if hasattr(self.ui, 'left_hip_sine_enable'):
                self.ui.left_hip_sine_enable.setChecked(False)
        
        action = "Started" if state else "Stopped"
        self.master.get_logger().info(f'🎯 {action} impedance trajectory for LEFT motor')

    # Send impedance trajectory command for right motor only
    def send_impedance_trajectory_right(self, state):
        if state:
            # Start right motor impedance trajectory
            if hasattr(self.ui, 'right_hip_sine_enable'):
                self.ui.right_hip_sine_enable.setChecked(True)
        else:
            # Stop right motor impedance trajectory
            if hasattr(self.ui, 'right_hip_sine_enable'):
                self.ui.right_hip_sine_enable.setChecked(False)
        
        action = "Started" if state else "Stopped"
        self.master.get_logger().info(f'🎯 {action} impedance trajectory for RIGHT motor')

    # Handle window close event - cleanup resources
    def closeEvent(self, event):
        self.master.get_logger().info('🔄 GUI closing, cleaning up...')
        
        # Note: All impedance control is now handled directly by GUI
        
        # Stop rosbag if running
        if hasattr(self, 'rosbag_process') and self.rosbag_process is not None:
            try:
                self.rosbag_process.terminate()
                self.rosbag_process.wait(timeout=3.0)
            except:
                pass
        
        event.accept()

# Function to spin the ROS node in a separate thread
def spin_ros_node(node):
    rclpy.spin(node)

# Main entry point
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