"""Constants and enums for the GUI application."""

from enum import Enum
from typing import Dict, Any

# CAN IDs for motors
LEFT_HIP_CAN_ID = 10
RIGHT_HIP_CAN_ID = 11
DRIVES_CAN_IDS = [LEFT_HIP_CAN_ID, RIGHT_HIP_CAN_ID]
NUM_MOTORS = len(DRIVES_CAN_IDS)

# Topic and service names
class RosTopics:
    """ROS topic names used in the application."""
    IMU_LEFT = 'imu_izq/imu_data'
    IMU_RIGHT = 'imu_der/imu_data'
    JOINT_STATES = 'md80/joint_states'
    POSITION_PID_COMMAND = '/md80/position_pid_command'
    VELOCITY_PID_COMMAND = '/md80/velocity_pid_command'
    IMPEDANCE_COMMAND = '/md80/impedance_command'
    MOTION_COMMAND = '/md80/motion_command'

class RosServices:
    """ROS service names used in the application."""
    ADD_MD80S = '/candle_ros2_node/add_md80s'
    ZERO_MD80S = '/candle_ros2_node/zero_md80s'
    SET_MODE_MD80S = '/candle_ros2_node/set_mode_md80s'
    ENABLE_MD80S = '/candle_ros2_node/enable_md80s'
    DISABLE_MD80S = '/candle_ros2_node/disable_md80s'
    UPDATE_IMPEDANCE = '/update_impedance_parameters'
    IMPEDANCE_TRAJECTORY = '/start_stop_impedance'

# Motor control modes
class MotorControlMode(Enum):
    """Enum for motor control modes."""
    POSITION_PID = (3.0, "POSITION_PID")
    VELOCITY_PID = (2.0, "VELOCITY_PID")
    TORQUE = (1.0, "RAW_TORQUE")
    IMPEDANCE = (4.0, "IMPEDANCE")

    def __init__(self, id_value: float, ros_string: str):
        self.id = id_value
        self.ros_string = ros_string

# Error messages
class ErrorMessages:
    """Error messages used throughout the application."""
    INVALID_IMPEDANCE_PARAMS = "Invalid impedance parameters: Kp={}, Kd={}"
    MOTOR_DATA_INCOMPLETE = "Received incomplete motor data"
    SERVICE_CALL_FAILED = "Service call failed: {}"
    SETPOINT_ERROR = "Error sending setpoints: {}"
    UPDATE_PLOT_ERROR = "Error updating plot: {}"
    MOTOR_DATA_ERROR = "Error processing motor data: {}"
    CONFIG_LOAD_ERROR = "Error loading configuration: {}"
    CONFIG_SAVE_ERROR = "Error saving configuration: {}"