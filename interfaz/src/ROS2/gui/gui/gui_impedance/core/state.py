"""State management for the GUI application."""

from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional, Dict, Any
from PyQt6.QtCore import QObject, pyqtSignal

class MotorState(Enum):
    """Enum representing possible motor states."""
    DISABLED = auto()
    ENABLED = auto()
    ERROR = auto()

@dataclass
class RecordingState:
    """State representing recording status and details."""
    is_recording: bool = False
    recording_folder: Optional[str] = None
    recording_filename: Optional[str] = None

class MotionState(Enum):
    """Enum representing possible motion states."""
    IDLE = auto()
    RUNNING = auto()
    PAUSED = auto()

@dataclass
class MotorSettings:
    """Settings for a motor."""
    enabled: bool = False
    control_mode: Optional[str] = None
    setpoint: float = 0.0
    position: float = 0.0
    velocity: float = 0.0
    torque: float = 0.0
    current: float = 0.0
    voltage: float = 0.0

class StateManager(QObject):
    """Manages state for the GUI application."""
    
    # Signals for various state changes
    state_changed = pyqtSignal(str, object)  # (state_name, new_value)
    motor_state_changed = pyqtSignal(int, object)  # (motor_id, new_state)
    recording_state_changed = pyqtSignal(object)  # (new_state)
    motion_state_changed = pyqtSignal(object)  # (new_state)
    motor_mode_changed = pyqtSignal(str)  # (new_mode)
    settings_changed = pyqtSignal(int, object)  # (motor_id, settings)

    def __init__(self):
        """Initialize state manager."""
        super().__init__()
        self._motor_states: Dict[int, MotorState] = {}
        self._motor_settings: Dict[int, MotorSettings] = {}
        self._recording_state = RecordingState()
        self._motion_state = MotionState.IDLE
        self._motor_mode: Optional[str] = None

    def update_motor_state(self, motor_id: int, state: MotorState) -> None:
        """Update the state of a motor."""
        if self._motor_states.get(motor_id) != state:
            self._motor_states[motor_id] = state
            self.state_changed.emit(f'motor_state_{motor_id}', state)
            self.motor_state_changed.emit(motor_id, state)

    def update_motor_settings(self, motor_id: int, settings: MotorSettings) -> None:
        """Update settings for a motor."""
        old_settings = self._motor_settings.get(motor_id)
        if old_settings != settings:
            self._motor_settings[motor_id] = settings
            self.state_changed.emit(f'motor_settings_{motor_id}', settings)
            self.settings_changed.emit(motor_id, settings)

    @property
    def recording_state(self) -> RecordingState:
        """Get the current recording state."""
        return self._recording_state

    @recording_state.setter
    def recording_state(self, state: RecordingState) -> None:
        """Set the recording state."""
        if self._recording_state != state:
            self._recording_state = state
            self.state_changed.emit('recording_state', state)
            self.recording_state_changed.emit(state)

    def update_recording_state(self, is_recording: bool = None, recording_folder: str = None, recording_filename: str = None) -> None:
        """Update the recording state with new values."""
        updated = False
        if is_recording is not None and self._recording_state.is_recording != is_recording:
            self._recording_state.is_recording = is_recording
            updated = True
        if recording_folder is not None and self._recording_state.recording_folder != recording_folder:
            self._recording_state.recording_folder = recording_folder
            updated = True
        if recording_filename is not None and self._recording_state.recording_filename != recording_filename:
            self._recording_state.recording_filename = recording_filename
            updated = True
        
        if updated:
            self.state_changed.emit('recording_state', self._recording_state)
            self.recording_state_changed.emit(self._recording_state)

    @property
    def motion_state(self) -> MotionState:
        """Get the current motion state."""
        return self._motion_state

    @motion_state.setter
    def motion_state(self, state: MotionState) -> None:
        """Set the motion state."""
        if self._motion_state != state:
            self._motion_state = state
            self.state_changed.emit('motion_state', state)
            self.motion_state_changed.emit(state)

    @property
    def motor_mode(self) -> Optional[str]:
        """Get the current motor control mode."""
        return self._motor_mode

    @motor_mode.setter
    def motor_mode(self, mode: Optional[str]) -> None:
        """Set the motor control mode."""
        if self._motor_mode != mode:
            self._motor_mode = mode
            self.state_changed.emit('motor_mode', mode)
            if mode is not None:
                self.motor_mode_changed.emit(mode)

    def set_control_mode(self, mode) -> None:
        """Set control mode from enum or string."""
        from .constants import MotorControlMode
        if isinstance(mode, MotorControlMode):
            mode_str = mode.value  # Convert enum to its value
        else:
            mode_str = str(mode) if mode is not None else None
        self.motor_mode = mode_str

    def get_motor_state(self, motor_id: int) -> Optional[MotorState]:
        """Get the state of a specific motor."""
        return self._motor_states.get(motor_id)

    def get_motor_settings(self, motor_id: int) -> Optional[MotorSettings]:
        """Get the settings for a specific motor."""
        return self._motor_settings.get(motor_id)

    def all_motors_enabled(self) -> bool:
        """Check if all motors are enabled."""
        return all(state == MotorState.ENABLED for state in self._motor_states.values())

    def any_motor_error(self) -> bool:
        """Check if any motor is in error state."""
        return any(state == MotorState.ERROR for state in self._motor_states.values())

    def clear_motor_errors(self) -> None:
        """Clear error states for all motors."""
        for motor_id in self._motor_states:
            if self._motor_states[motor_id] == MotorState.ERROR:
                self.update_motor_state(motor_id, MotorState.DISABLED)

    def set_motors_enabled(self, enabled: bool) -> None:
        """Enable or disable all motors."""
        new_state = MotorState.ENABLED if enabled else MotorState.DISABLED
        for motor_id in self._motor_states:
            self.update_motor_state(motor_id, new_state)
                
    def update_setpoints(self, left_setpoint: float = 0.0, right_setpoint: float = 0.0) -> None:
        """Update setpoints for both motors."""
        # Update left hip setpoint
        if self._motor_settings.get(10):  # 10 is LEFT_HIP_CAN_ID
            left_settings = self._motor_settings[10]
            if left_settings.setpoint != left_setpoint:
                left_settings.setpoint = left_setpoint
                self.settings_changed.emit(10, left_settings)
                
        # Update right hip setpoint
        if self._motor_settings.get(11):  # 11 is RIGHT_HIP_CAN_ID
            right_settings = self._motor_settings[11]
            if right_settings.setpoint != right_setpoint:
                right_settings.setpoint = right_setpoint
                self.settings_changed.emit(11, right_settings)