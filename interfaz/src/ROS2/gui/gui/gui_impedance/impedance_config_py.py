#!/usr/bin/env python3
"""
Impedance Control Configuration Loader (Python)

This module provides centralized configuration management for the impedance control
system GUI. It reads the same YAML configuration file as the C++ impedance_node
to ensure parameter consistency across all system components.

Author: Professional Development Team  
Date: September 26, 2025
Version: 1.0

Usage:
    from impedance_config_py import ImpedanceConfig
    
    config = ImpedanceConfig('/path/to/impedance_config.yaml')
    kp_gains = config.get_kp_gains()
    gui_ranges = config.get_gui_parameter_ranges()
"""

import os
import yaml
import logging
from typing import List, Dict, Tuple, Any, Optional
from dataclasses import dataclass

@dataclass
class ParameterRange:
    """Parameter range configuration for GUI widgets"""
    minimum: float
    maximum: float  
    step: float
    decimals: int

@dataclass  
class GuiSettings:
    """GUI-specific configuration settings"""
    kp_range: ParameterRange
    kd_range: ParameterRange
    offset_range: ParameterRange
    real_time_updates: bool
    update_rate_hz: int

class ImpedanceConfig:
    """
    Centralized configuration management for impedance control system.
    
    This class loads configuration from a YAML file and provides type-safe
    access to all impedance control parameters. It ensures consistency 
    between C++ nodes and Python GUI components.
    """
    
    def __init__(self, config_file_path: Optional[str] = None):
        """
        Initialize configuration loader with mandatory YAML loading.
        
        Args:
            config_file_path: Path to impedance_config.yaml file.
                            If None, attempts to find config in standard locations.
                            
        Raises:
            FileNotFoundError: If config file cannot be found
            RuntimeError: If config file parsing fails
            
        Safety Note:
            No fallback values are used. Configuration MUST be loaded from
            a valid YAML file to ensure consistency with C++ components.
        """
        self._logger = logging.getLogger(__name__)
        
        # Determine config file path
        if config_file_path is None:
            config_file_path = self._find_default_config()
            
        if not config_file_path or not os.path.exists(config_file_path):
            error_msg = f"❌ CRITICAL: Config file not found: {config_file_path or 'None'}"
            self._logger.error(error_msg)
            self._logger.error("🛑 SAFETY REQUIREMENT: impedance_config.yaml file is mandatory")
            self._logger.error("📁 Tip: set IMPEDANCE_CONFIG_PATH to your YAML file, e.g. $WS/src/ROS2/config/impedance_config.yaml")
            raise FileNotFoundError(error_msg)
        
        # Load configuration (no fallbacks - fail fast for safety)
        try:
            self._config_file_path = config_file_path
            self._load_from_file(config_file_path)
            self._loaded_from_file = True
            self._logger.info(f"✅ Configuration loaded from: {config_file_path}")
        except Exception as e:
            error_msg = f"❌ CRITICAL: Failed to parse config file '{config_file_path}': {e}"
            self._logger.error(error_msg)
            self._logger.error("🛑 SAFETY REQUIREMENT: Config file must be valid YAML")
            raise RuntimeError(error_msg) from e
    
    # ========================================================================
    # IMPEDANCE CONTROL PARAMETERS
    # ========================================================================
    
    def get_kp_gains(self) -> List[float]:
        """Get proportional gains [left_motor, right_motor] in Nm/rad"""
        return self._kp_gains.copy()
    
    def get_kd_gains(self) -> List[float]:
        """Get derivative gains [left_motor, right_motor] in Nm·s/rad"""
        return self._kd_gains.copy()
        
    def get_offset_torque(self) -> List[float]:
        """Get torque offsets [left_motor, right_motor] in Nm"""
        return self._offset_torque.copy()
        
    def get_max_output_torque(self) -> List[float]:
        """Get maximum output torques [left_motor, right_motor] in Nm"""
        return self._max_output_torque.copy()
    
    # ========================================================================
    # MOTOR CONFIGURATION
    # ========================================================================
    
    def get_left_motor_id(self) -> int:
        """Get left motor CAN ID"""
        return self._left_motor_id
        
    def get_right_motor_id(self) -> int:
        """Get right motor CAN ID"""
        return self._right_motor_id
        
    def get_motor_ids(self) -> List[int]:
        """Get motor IDs as list [left, right]"""
        return [self._left_motor_id, self._right_motor_id]
    
    # ========================================================================
    # GUI CONFIGURATION
    # ========================================================================
    
    def get_gui_settings(self) -> GuiSettings:
        """Get complete GUI settings configuration"""
        return self._gui_settings
        
    def get_gui_parameter_ranges(self) -> Dict[str, ParameterRange]:
        """Get parameter ranges for GUI spin boxes"""
        return {
            'kp': self._gui_settings.kp_range,
            'kd': self._gui_settings.kd_range, 
            'offset': self._gui_settings.offset_range
        }
    
    def get_kp_range(self) -> ParameterRange:
        """Get kP parameter range for GUI widgets"""
        return self._gui_settings.kp_range
        
    def get_kd_range(self) -> ParameterRange:
        """Get kD parameter range for GUI widgets"""
        return self._gui_settings.kd_range
        
    def get_offset_range(self) -> ParameterRange:
        """Get offset parameter range for GUI widgets"""
        return self._gui_settings.offset_range
    
    # ========================================================================
    # SAFETY PARAMETERS
    # ========================================================================
    
    def get_emergency_torque_limit(self) -> float:
        """Get emergency torque limit (absolute maximum) in Nm"""
        return self._emergency_torque_limit
        
    def get_kp_max_safe(self) -> float:
        """Get maximum safe kP value"""
        return self._kp_max_safe
        
    def get_kd_max_safe(self) -> float:
        """Get maximum safe kD value"""
        return self._kd_max_safe
        
    def get_offset_max_safe(self) -> float:
        """Get maximum safe offset magnitude in Nm"""
        return self._offset_max_safe
    
    # ========================================================================
    # ROS2 INTERFACE NAMES
    # ========================================================================
    
    def get_node_name(self) -> str:
        """Get ROS2 node name"""
        return self._node_name
        
    def get_impedance_command_topic(self) -> str:
        """Get impedance command topic name"""
        return self._impedance_command_topic
        
    def get_motion_command_topic(self) -> str:
        """Get motion command topic name"""  
        return self._motion_command_topic
        
    def get_update_parameters_service(self) -> str:
        """Get parameter update service name"""
        return self._update_parameters_service
        
    def get_start_stop_service(self) -> str:
        """Get start/stop service name"""
        return self._start_stop_service
    
    # ========================================================================
    # PARAMETER VALIDATION
    # ========================================================================
    
    def validate_parameters(self, kp: List[float], kd: List[float], 
                          offset: List[float], max_output: List[float]) -> bool:
        """
        Validate impedance parameters against safety limits.
        
        Args:
            kp: Proportional gains to validate
            kd: Derivative gains to validate  
            offset: Offset torques to validate
            max_output: Max output torques to validate
            
        Returns:
            True if all parameters are within safe ranges
        """
        
        # Check array lengths
        if not all(len(arr) == 2 for arr in [kp, kd, offset, max_output]):
            return False
            
        # Validate kP parameters
        if any(kp_val < 0.0 or kp_val > self._kp_max_safe for kp_val in kp):
            return False
            
        # Validate kD parameters  
        if any(kd_val < 0.0 or kd_val > self._kd_max_safe for kd_val in kd):
            return False
            
        # Validate offset torques
        if any(abs(offset_val) > self._offset_max_safe for offset_val in offset):
            return False
            
        # Validate max output torques
        if any(max_val <= 0.0 or max_val > self._emergency_torque_limit for max_val in max_output):
            return False
            
        return True
    
    def clamp_parameters(self, kp: List[float], kd: List[float],
                        offset: List[float], max_output: List[float]) -> Tuple[List[float], List[float], List[float], List[float]]:
        """
        Clamp parameters to safe ranges.
        
        Args:
            kp: Proportional gains to clamp
            kd: Derivative gains to clamp
            offset: Offset torques to clamp  
            max_output: Max output torques to clamp
            
        Returns:
            Tuple of clamped parameter lists (kp, kd, offset, max_output)
        """
        
        # Clamp kP parameters
        kp_clamped = [max(0.0, min(kp_val, self._kp_max_safe)) for kp_val in kp]
        
        # Clamp kD parameters
        kd_clamped = [max(0.0, min(kd_val, self._kd_max_safe)) for kd_val in kd]
        
        # Clamp offset torques  
        offset_clamped = [max(-self._offset_max_safe, min(offset_val, self._offset_max_safe)) for offset_val in offset]
        
        # Clamp max output torques
        max_output_clamped = [max(0.1, min(max_val, self._emergency_torque_limit)) for max_val in max_output]
        
        return kp_clamped, kd_clamped, offset_clamped, max_output_clamped
    
    # ========================================================================
    # UTILITY METHODS
    # ========================================================================
    
    def print_configuration(self):
        """Print all configuration parameters to console"""
        
        print("=" * 60)
        print("IMPEDANCE CONTROL CONFIGURATION (YAML-ONLY)")
        print("=" * 60)
        print(f"📁 Loaded from file: {self._config_file_path}")
        print("🛡️  No fallback values - config file is mandatory for safety")
            
        print()
        print("🔧 IMPEDANCE PARAMETERS:")
        print(f"   kP gains:        {self._kp_gains} Nm/rad")
        print(f"   kD gains:        {self._kd_gains} Nm·s/rad")
        print(f"   Offset torques:  {self._offset_torque} Nm") 
        print(f"   Max output:      {self._max_output_torque} Nm")
        
        print()
        print("🤖 MOTOR CONFIGURATION:")
        print(f"   Left motor ID:   {self._left_motor_id}")
        print(f"   Right motor ID:  {self._right_motor_id}")
        
        print()
        print("🖥️  GUI SETTINGS:")
        print(f"   kP range:        {self._gui_settings.kp_range.minimum} - {self._gui_settings.kp_range.maximum} (step: {self._gui_settings.kp_range.step})")
        print(f"   kD range:        {self._gui_settings.kd_range.minimum} - {self._gui_settings.kd_range.maximum} (step: {self._gui_settings.kd_range.step})")
        print(f"   Offset range:    {self._gui_settings.offset_range.minimum} - {self._gui_settings.offset_range.maximum} (step: {self._gui_settings.offset_range.step})")
        print(f"   Real-time:       {self._gui_settings.real_time_updates}")
        print(f"   Update rate:     {self._gui_settings.update_rate_hz} Hz")
        
        print()
        print("🛡️  SAFETY LIMITS:")
        print(f"   Emergency limit: {self._emergency_torque_limit} Nm")
        print(f"   Max safe kP:     {self._kp_max_safe}")
        print(f"   Max safe kD:     {self._kd_max_safe}")
        print(f"   Max safe offset: {self._offset_max_safe} Nm")
        
        print()
        print("📡 ROS2 INTERFACE:")
        print(f"   Node name:       {self._node_name}")
        print(f"   Impedance topic: {self._impedance_command_topic}")
        print(f"   Motion topic:    {self._motion_command_topic}")
        print(f"   Update service:  {self._update_parameters_service}")
        print(f"   Start/stop svc:  {self._start_stop_service}")
        
        print("=" * 60)
    
    def is_loaded_from_file(self) -> bool:
        """Check if configuration was loaded from file successfully"""
        return self._loaded_from_file
        
    def get_config_file_path(self) -> Optional[str]:
        """Get configuration file path that was loaded"""
        return self._config_file_path if self._loaded_from_file else None
    
    # ========================================================================
    # PRIVATE METHODS
    # ========================================================================
    
    def _find_default_config(self) -> Optional[str]:
        """
        Find the default config file in standard locations.

        Resolution order:
        1. Environment override: IMPEDANCE_CONFIG_PATH or ASSISTIVE_EXO_CONFIG_PATH
        2. Workspace-relative paths derived from current working directory
        3. Common local paths within the repository

        Returns:
            Path to config file if found, None otherwise
        """
        # 1) Environment override
        env_paths = [
            os.environ.get("IMPEDANCE_CONFIG_PATH"),
            os.environ.get("ASSISTIVE_EXO_CONFIG_PATH"),
        ]
        for p in env_paths:
            if p and os.path.exists(p):
                return os.path.abspath(p)

        # 2) Derive workspace-relative candidates from CWD
        cwd = os.getcwd()
        candidates = [
            os.path.join(cwd, "src/ROS2/config/impedance_config.yaml"),
            os.path.join(cwd, "config/impedance_config.yaml"),
            os.path.abspath(os.path.join(cwd, "../src/ROS2/config/impedance_config.yaml")),
            os.path.abspath(os.path.join(cwd, "../config/impedance_config.yaml")),
        ]

        # 3) Fallbacks near execution paths (relative to this file may not work post-install)
        file_dir = os.path.dirname(__file__)
        candidates.extend([
            os.path.abspath(os.path.join(file_dir, "../../config/impedance_config.yaml")),
            os.path.abspath(os.path.join(file_dir, "../config/impedance_config.yaml")),
        ])

        for path in candidates:
            if os.path.exists(path):
                return os.path.abspath(path)

        return None
    
    def _load_from_file(self, file_path: str):
        """Load configuration from YAML file (mandatory - no fallbacks)"""
        
        with open(file_path, 'r', encoding='utf-8') as f:
            config_data = yaml.safe_load(f)
        
        if not config_data:
            raise RuntimeError("Config file is empty or invalid YAML")
        
        # Parse impedance control section (MANDATORY)
        if 'impedance_control' not in config_data:
            raise RuntimeError("Missing required 'impedance_control' section in config file")
        
        impedance_config = config_data['impedance_control']
        
        # Load kP gains (mandatory)
        if 'kp_gains' not in impedance_config:
            raise RuntimeError("Missing required 'kp_gains' in impedance_control section")
        kp_gains = impedance_config['kp_gains']
        if not isinstance(kp_gains, list) or len(kp_gains) != 2:
            raise RuntimeError("kp_gains must be a list of exactly 2 values")
        self._kp_gains = [float(x) for x in kp_gains]
        
        # Load kD gains (mandatory)
        if 'kd_gains' not in impedance_config:
            raise RuntimeError("Missing required 'kd_gains' in impedance_control section")
        kd_gains = impedance_config['kd_gains']
        if not isinstance(kd_gains, list) or len(kd_gains) != 2:
            raise RuntimeError("kd_gains must be a list of exactly 2 values")
        self._kd_gains = [float(x) for x in kd_gains]
        
        # Load offset torque (mandatory)
        if 'offset_torque' not in impedance_config:
            raise RuntimeError("Missing required 'offset_torque' in impedance_control section")
        offset_torque = impedance_config['offset_torque']
        if not isinstance(offset_torque, list) or len(offset_torque) != 2:
            raise RuntimeError("offset_torque must be a list of exactly 2 values")
        self._offset_torque = [float(x) for x in offset_torque]
        
        # Load max output torque (mandatory)
        if 'max_output_torque' not in impedance_config:
            raise RuntimeError("Missing required 'max_output_torque' in impedance_control section")
        max_output = impedance_config['max_output_torque']
        if not isinstance(max_output, list) or len(max_output) != 2:
            raise RuntimeError("max_output_torque must be a list of exactly 2 values")
        self._max_output_torque = [float(x) for x in max_output]
        
        # Parse motor configuration (mandatory)
        if 'motors' not in config_data:
            raise RuntimeError("Missing required 'motors' section in config file")
        
        motors_config = config_data['motors']
        
        if 'left_motor_id' not in motors_config:
            raise RuntimeError("Missing required 'left_motor_id' in motors section")
        self._left_motor_id = int(motors_config['left_motor_id'])
        
        if 'right_motor_id' not in motors_config:
            raise RuntimeError("Missing required 'right_motor_id' in motors section")
        self._right_motor_id = int(motors_config['right_motor_id'])
        
        # Parse safety configuration (mandatory)
        if 'safety' not in config_data:
            raise RuntimeError("Missing required 'safety' section in config file")
        
        safety_config = config_data['safety']
        
        if 'emergency_torque_limit' not in safety_config:
            raise RuntimeError("Missing required 'emergency_torque_limit' in safety section")
        self._emergency_torque_limit = float(safety_config['emergency_torque_limit'])
        
        if 'validation' not in safety_config:
            raise RuntimeError("Missing required 'validation' subsection in safety section")
        
        validation = safety_config['validation']
        
        if 'kp_max_safe' not in validation:
            raise RuntimeError("Missing required 'kp_max_safe' in safety.validation section")
        self._kp_max_safe = float(validation['kp_max_safe'])
        
        if 'kd_max_safe' not in validation:
            raise RuntimeError("Missing required 'kd_max_safe' in safety.validation section")
        self._kd_max_safe = float(validation['kd_max_safe'])
        
        if 'offset_max_safe' not in validation:
            raise RuntimeError("Missing required 'offset_max_safe' in safety.validation section")
        self._offset_max_safe = float(validation['offset_max_safe'])
        
        # Parse ROS2 interface (mandatory)
        if 'ros2_interface' not in config_data:
            raise RuntimeError("Missing required 'ros2_interface' section in config file")
        
        ros2_interface = config_data['ros2_interface']
        
        # Load topic names
        if 'topics' not in ros2_interface:
            raise RuntimeError("Missing required 'topics' subsection in ros2_interface")
        topics = ros2_interface['topics']
        
        if 'impedance_command' not in topics:
            raise RuntimeError("Missing required 'impedance_command' in ros2_interface.topics")
        self._impedance_command_topic = str(topics['impedance_command'])
        
        if 'motion_command' not in topics:
            raise RuntimeError("Missing required 'motion_command' in ros2_interface.topics")
        self._motion_command_topic = str(topics['motion_command'])
        
        # Load service names
        if 'services' not in ros2_interface:
            raise RuntimeError("Missing required 'services' subsection in ros2_interface")
        services = ros2_interface['services']
        
        if 'update_parameters' not in services:
            raise RuntimeError("Missing required 'update_parameters' in ros2_interface.services")
        self._update_parameters_service = str(services['update_parameters'])
        
        if 'start_stop_control' not in services:
            raise RuntimeError("Missing required 'start_stop_control' in ros2_interface.services")
        self._start_stop_service = str(services['start_stop_control'])
        
        # Load node name
        if 'ros2' in config_data and 'node_name' in config_data['ros2']:
            self._node_name = str(config_data['ros2']['node_name'])
        else:
            raise RuntimeError("Missing required 'node_name' in ros2 section")
        
        # Parse GUI settings (mandatory)
        if 'gui_settings' not in config_data:
            raise RuntimeError("Missing required 'gui_settings' section in config file")
        
        gui_config = config_data['gui_settings']
        
        if 'parameter_ranges' not in gui_config:
            raise RuntimeError("Missing required 'parameter_ranges' in gui_settings section")
        ranges = gui_config['parameter_ranges']
        
        # Load kP range (mandatory)
        if 'kp_range' not in ranges:
            raise RuntimeError("Missing required 'kp_range' in gui_settings.parameter_ranges")
        kp_range = ranges['kp_range']
        kp_range_obj = ParameterRange(
            minimum=float(kp_range['minimum']),
            maximum=float(kp_range['maximum']),
            step=float(kp_range['step']),
            decimals=int(kp_range['decimals'])
        )
        
        # Load kD range (mandatory)
        if 'kd_range' not in ranges:
            raise RuntimeError("Missing required 'kd_range' in gui_settings.parameter_ranges")
        kd_range = ranges['kd_range']
        kd_range_obj = ParameterRange(
            minimum=float(kd_range['minimum']),
            maximum=float(kd_range['maximum']),
            step=float(kd_range['step']),
            decimals=int(kd_range['decimals'])
        )
        
        # Load offset range (mandatory)
        if 'offset_range' not in ranges:
            raise RuntimeError("Missing required 'offset_range' in gui_settings.parameter_ranges")
        offset_range = ranges['offset_range']
        offset_range_obj = ParameterRange(
            minimum=float(offset_range['minimum']),
            maximum=float(offset_range['maximum']),
            step=float(offset_range['step']),
            decimals=int(offset_range['decimals'])
        )
        
        # Load GUI behavior settings (mandatory)
        if 'real_time_updates' not in gui_config:
            raise RuntimeError("Missing required 'real_time_updates' in gui_settings")
        if 'update_rate_hz' not in gui_config:
            raise RuntimeError("Missing required 'update_rate_hz' in gui_settings")
        
        # Create GUI settings object
        self._gui_settings = GuiSettings(
            kp_range=kp_range_obj,
            kd_range=kd_range_obj,
            offset_range=offset_range_obj,
            real_time_updates=bool(gui_config['real_time_updates']),
            update_rate_hz=int(gui_config['update_rate_hz'])
        )


# ============================================================================
# CONVENIENCE FUNCTIONS  
# ============================================================================

def load_config(config_file_path: Optional[str] = None) -> ImpedanceConfig:
    """
    Convenience function to load impedance configuration.
    
    Args:
        config_file_path: Path to YAML config file. If None, uses defaults.
        
    Returns:
        Loaded ImpedanceConfig instance
    """
    return ImpedanceConfig(config_file_path)


def find_config_file() -> Optional[str]:
    """
    Automatically find the impedance config file in common locations.
    
    Returns:
        Path to config file if found, None otherwise
    """
    
    # Common search paths
    search_paths = [
        # Current directory
        "./impedance_config.yaml",
        # Config directory relative to current
        "./config/impedance_config.yaml", 
        # ROS2 workspace config
        "../config/impedance_config.yaml",
        # Absolute workspace path
        "/home/legreplica/assistive_exo_ws/src/ROS2/config/impedance_config.yaml"
    ]
    
    for path in search_paths:
        if os.path.exists(path):
            return os.path.abspath(path)
    
    return None


if __name__ == "__main__":
    """Example usage and testing"""
    
    # Setup logging
    logging.basicConfig(level=logging.INFO)
    
    # Try to find config file automatically
    config_path = find_config_file()
    
    if config_path:
        print(f"🔍 Found config file: {config_path}")
        config = ImpedanceConfig(config_path)
    else:
        print("🔍 No config file found, using defaults")
        config = ImpedanceConfig()
    
    # Print configuration
    config.print_configuration()
    
    # Example parameter validation
    test_kp = [5.0, 5.0]
    test_kd = [0.2, 0.2]
    test_offset = [1.0, -1.0]
    test_max = [8.0, 8.0]
    
    print("\n🧪 Testing parameter validation:")
    is_valid = config.validate_parameters(test_kp, test_kd, test_offset, test_max)
    print(f"   Parameters valid: {is_valid}")
    
    if not is_valid:
        clamped = config.clamp_parameters(test_kp, test_kd, test_offset, test_max)
        print(f"   Clamped values: kp={clamped[0]}, kd={clamped[1]}, offset={clamped[2]}, max={clamped[3]}")