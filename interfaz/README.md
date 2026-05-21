# Assistive Exoskeleton Workspace

A comprehensive ROS2 workspace for the development, control, and monitoring of a wearable assistive exoskeleton system. This workspace integrates motor control, wearable biosensors, IMU sensors, trajectory generation, and a user-friendly GUI for real-time operation.

---

## 📋 Table of Contents

- [Overview](#overview)
- [Project Structure](#project-structure)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Quick Start](#quick-start)
- [Main Components](#main-components)
- [Detailed Documentation](#detailed-documentation)
- [Usage Examples](#usage-examples)
- [Troubleshooting](#troubleshooting)
- [Licenses](#license)
- [Acknowledgments](#acknowledgments)

---

## 🎯 Overview

This workspace provides a complete solution for assistive exoskeleton control and monitoring, featuring:

- **Motor Control**: MD80 motor drivers integration via CANdle interface
- **Biosensor Integration**: Support for multiple wearable biosensors (Empatica E4, Emotiv Insight, Polar H10, Shimmer3 GSR, etc.)
- **IMU Support**: Real-time inertial measurement unit data acquisition
- **Impedance Control**: Configurable impedance parameters for safe and adaptive control
- **Graphical Interface**: PyQt-based GUI for real-time monitoring and control
- **Trajectory Generation**: Real-time trajectory generation capabilities

---

## 📁 Project Structure

```
assistive_exo_ws/
├── src/
│   ├── candle_ros2/                        # MD80 motor driver package
│   ├── ROS2/                               # Main exoskeleton control packages
│   │   ├── gui/                            # Graphical user interface
│   │   ├── imu_ap/                         # IMU data acquisition
│   │   ├── msg_interface/                  # Custom message definitions
│   │   ├── srv_interface/                  # Custom service definitions
│   │   ├── startup_pkg/                    # System startup utilities
│   │   └── trajectory_generation/          # Trajectory generation (deprecated)
│   └── ros2-foxy-wearable-biosensors/      # Wearable biosensor packages
│       ├── empatica_e4/
│       ├── emotiv_insight/
│       ├── polar_h10/
│       ├── shimmer3_gsr_unit/
│       └── vernier_respiration_belt/
├── README.md                               # This file
├── commands_motor.txt                      # File showing some candle commands, used to control de motor
├── gui.png                                 # Image which show impedance control mode of the gui
├── launch_gui.sh                           # Quick launch script for GUI
├── terminal_text_gui.txt                   # File showing the text printed at the terminal when the gui is launched
├── test_enable_disable_cycle.py            # Motor testing script
└── test_motor_enable.py                    # Motor enable testing script
```

---

## 🔧 Prerequisites

### System Requirements
- **OS**: Ubuntu 20.04 or newer
- **ROS2**: Foxy or newer
- **Python**: 3.8+

### Required Software
- ROS2 (Foxy/Galactic/Humble)
- Python 3.8+
- PyQt6 or PyQt5
- colcon build tools
- Git

### Python Dependencies
```bash
pip3 install PyQt6 matplotlib open-e4-client pexpect websocket-client
```

### Additional Requirements
- **Emotiv App**: Required for Emotiv Insight sensor ([Download](https://www.emotiv.com/my-account/downloads/))
- **Empatica E4 Streaming Server**: Required for E4 wristband ([Download](https://developer.empatica.com/windows-streaming-server-usage.html))

---

## 🚀 Installation

### 1. Clone the Repository
```bash
cd ~/
git clone <repository-url> assistive_exo_ws
cd assistive_exo_ws
```

### 2. Source ROS2
```bash
source /opt/ros/foxy/setup.bash
```

### 3. Install Dependencies
```bash
# Install Python dependencies
pip3 install PyQt6 matplotlib open-e4-client pexpect websocket-client

# Install ROS2 dependencies (from workspace root)
rosdep install --from-paths src --ignore-src -r -y
```

### 4. Build the Workspace
```bash
colcon build --symlink-install
```

### 5. Source the Workspace
```bash
source install/setup.bash
```

---

## ⚡ Quick Start

### Launch the GUI
The easiest way to start the system is using the provided launch script:

```bash
./launch_gui.sh
```

This script will:
1. Source the ROS2 environment
2. Source the workspace
3. Launch the graphical user interface
4. Initialize motor controllers
5. Start sensor data streams

### Manual Launch
Alternatively, you can launch components individually:

```bash
# Terminal 1: Source and launch GUI
source install/setup.bash
ros2 run gui gui_assistive_exo.py

# Terminal 2: Launch motor drivers (if needed)
source install/setup.bash
ros2 run candle_ros2 candle_ros2_node
```

---

## 🔩 Main Components

### 1. Motor Control - `candle_ros2`
MD80 motor driver package for low-level motor commands via CAN bus.

**Key Features:**
- Real-time motor position/velocity control
- Impedance control support
- Motor enable/disable services
- Joint state feedback

**📚 Documentation**: [src/candle_ros2/README.md](src/candle_ros2/README.md)

**Services:**
- `/add_md80s` - Add motors to the system
- `/zero_md80s` - Zero motor positions
- `/set_mode_md80s` - Set control mode
- `/enable_md80s` - Enable motors
- `/disable_md80s` - Disable motors

**Topics:**
- `/md80/motion_command` - Send motion commands
- `/md80/impedance_command` - Send impedance parameters
- `/md80/joint_states` - Receive joint feedback

### 2. Graphical User Interface - `gui`
PyQt-based GUI for real-time exoskeleton control and monitoring.

**Key Features:**
- Real-time impedance parameter adjustment
- Motor position/velocity control
- Sensor data visualization
- Safety limits monitoring
- Configuration file management

**📚 Documentation**: [src/ROS2/gui/README.md](src/ROS2/gui/README.md)

**Files:**
- `gui_assistive_exo.ui` - Qt Designer layout
- `gui_assistive_exo_data.py` - Auto-generated UI code
- `gui_assistive_exo.py` - Backend logic and ROS2 integration

### 3. Wearable Biosensors - `ros2-foxy-wearable-biosensors`
Comprehensive package for integrating wearable biosensors into ROS2.

**Supported Sensors:**
- Empatica E4 wristband (HR, BVP, GSR, temperature)
- Emotiv Insight (EEG)
- Polar H10 (Heart rate)
- Shimmer3 GSR Unit+ (Galvanic skin response)
- Vernier Respiration Belt

**📚 Documentation**: [src/ros2-foxy-wearable-biosensors/README.md](src/ros2-foxy-wearable-biosensors/README.md)

### 4. IMU Interface - `imu_ap`
Real-time IMU data acquisition for multiple sensors.

**Key Features:**
- Multiple IMU support
- Configurable serial interfaces
- Euler angle publishing
- Adjustable publish rates

**📚 Documentation**: [src/ROS2/imu_ap/README.md](src/ROS2/imu_ap/README.md)

### 5. Message & Service Interfaces
Custom ROS2 messages and services for standardized communication.

**📚 Documentation**: 
- [src/ROS2/msg_interface/README.md](src/ROS2/msg_interface/README.md)
- [src/ROS2/srv_interface/README.md](src/ROS2/srv_interface/README.md)

### 6. Startup Package
System initialization and startup utilities.

**📚 Documentation**: [src/ROS2/startup_pkg/README.md](src/ROS2/startup_pkg/README.md)

---

## 📖 Detailed Documentation

For comprehensive information about each component, please refer to the individual README files:

| Component | Documentation Link |
|-----------|-------------------|
| **Main ROS2 Packages** | [src/ROS2/README.md](src/ROS2/README.md) |
| **Motor Control** | [src/candle_ros2/README.md](src/candle_ros2/README.md) |
| **GUI** | [src/ROS2/gui/README.md](src/ROS2/gui/README.md) |
| **IMU Interface** | [src/ROS2/imu_ap/README.md](src/ROS2/imu_ap/README.md) |
| **Biosensors** | [src/ros2-foxy-wearable-biosensors/README.md](src/ros2-foxy-wearable-biosensors/README.md) |
| **Empatica E4** | [src/ros2-foxy-wearable-biosensors/src/empatica_e4/README.md](src/ros2-foxy-wearable-biosensors/src/empatica_e4/README.md) |
| **Emotiv Insight** | [src/ros2-foxy-wearable-biosensors/src/emotiv_insight/README.md](src/ros2-foxy-wearable-biosensors/src/emotiv_insight/README.md) |
| **Polar H10** | [src/ros2-foxy-wearable-biosensors/src/polar_h10/README.md](src/ros2-foxy-wearable-biosensors/src/polar_h10/README.md) |
| **Shimmer3 GSR** | [src/ros2-foxy-wearable-biosensors/src/shimmer3_gsr_unit/README.md](src/ros2-foxy-wearable-biosensors/src/shimmer3_gsr_unit/README.md) |
| **Message Interface** | [src/ROS2/msg_interface/README.md](src/ROS2/msg_interface/README.md) |
| **Service Interface** | [src/ROS2/srv_interface/README.md](src/ROS2/srv_interface/README.md) |

---

## 💡 Usage Examples

### Running Individual Nodes

**Launch GUI only:**
```bash
source install/setup.bash
ros2 run gui gui_assistive_exo.py
```

**Launch motor control:**
```bash
source install/setup.bash
ros2 run candle_ros2 candle_ros2_node
```

**Launch IMU node:**
```bash
source install/setup.bash
ros2 run imu_ap ros_ap_euler --ros-args -p serial_port:=/dev/ttyUSB0
```

**Launch biosensor (example: Polar H10):**
```bash
source install/setup.bash
ros2 run polar_h10 polar_h10_node
```

### Modifying the GUI
If you need to modify the GUI layout:

1. Open Qt Designer:
```bash
designer src/ROS2/gui/gui_assistive_exo.ui
```

2. Make your changes and save

3. Regenerate Python code:
```bash
cd src/ROS2/gui/
pyuic6 -x gui_assistive_exo.ui -o gui_assistive_exo_data.py
```

---

## 🔍 Troubleshooting

### Build Errors
If you encounter build errors:
```bash
# Clean the workspace
rm -rf build/ install/ log/

# Rebuild
colcon build --symlink-install
```

### Permission Denied on USB Devices
```bash
# Add user to dialout group for serial port access
sudo usermod -a -G dialout $USER
# Log out and log back in for changes to take effect
```

### GUI Not Launching
```bash
# Check PyQt installation
pip3 show PyQt6

# If not installed:
pip3 install PyQt6

# If using PyQt5, modify accordingly
```

### Motor Communication Issues
- Ensure CANdle device is properly connected
- Check CAN bus termination
- Verify motor IDs in configuration files
- See [src/candle_ros2/README.md](src/candle_ros2/README.md) for detailed troubleshooting

### Biosensor Connection Issues
- Check that required software is installed (Emotiv App, E4 Streaming Server)
- Verify Bluetooth/USB connections
- Ensure devices are paired and connected
- Check individual sensor documentation in [src/ros2-foxy-wearable-biosensors/](src/ros2-foxy-wearable-biosensors/)

---

## 📄 License

This project integrates multiple packages with different licenses:
- `candle_ros2`: Check [src/candle_ros2/README.md](src/candle_ros2/README.md)
- `ros2-foxy-wearable-biosensors`: Check [src/ros2-foxy-wearable-biosensors/README.md](src/ros2-foxy-wearable-biosensors/README.md)
- Internal packages: Please refer to individual package documentation

---

## 🙏 Acknowledgments

This workspace integrates contributions from:
- **MABrobotics** - CANdle and MD80 motor driver packages
- **SMART Lab, Purdue University** - Wearable biosensor packages
- **Assistive Exoskeleton Development Team** - GUI, control systems, and integration

---

**Last Updated**: December 2025
