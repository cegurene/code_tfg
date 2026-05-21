#!/bin/bash

# Assistive Exoskeleton Complete GUI Launcher
# This script launches the complete GUI with a clean environment, avoiding snap library conflicts

# Flujo recomendado para crear y mantener la nueva GUI:
# conda activate ros2rl
# cd interfaz/src/ROS2/gui/gui/gui_completa
# pyuic6 -x gui_completa.ui -o gui_completa_data.py
# cd ../../../../..
# colcon build --packages-select gui
# ./launch_gui_completa.sh

# Navigate to workspace (directory containing this script)
WS_DIR="$(dirname "$(realpath "$0")")"
cd "$WS_DIR"

# Source ROS2 environment
source install/setup.bash

# Launch with clean environment
env -i \
    HOME="$HOME" \
    USER="$USER" \
    SHELL="/bin/bash" \
    DISPLAY="$DISPLAY" \
    XAUTHORITY="$XAUTHORITY" \
    XDG_RUNTIME_DIR="$XDG_RUNTIME_DIR" \
    PATH="/opt/ros/jazzy/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin" \
    LD_LIBRARY_PATH="/opt/ros/jazzy/lib:$WS_DIR/install/candle_ros2/lib:$WS_DIR/install/srv_interface/lib:$WS_DIR/install/msg_interface/lib:/usr/lib/x86_64-linux-gnu:/lib/x86_64-linux-gnu" \
    LD_PRELOAD="/lib/x86_64-linux-gnu/libpthread.so.0" \
    PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:/opt/ros/jazzy/local/lib/python3.12/dist-packages:$WS_DIR/install/gui/lib/python3.12/site-packages:$WS_DIR/build/gui:$WS_DIR/install/srv_interface/lib/python3.12/site-packages:$WS_DIR/install/msg_interface/lib/python3.12/site-packages:$WS_DIR/install/candle_ros2/lib/python3.12/site-packages" \
    ROS_DISTRO="jazzy" \
    AMENT_PREFIX_PATH="/opt/ros/jazzy:$WS_DIR/install/gui:$WS_DIR/install/srv_interface:$WS_DIR/install/msg_interface:$WS_DIR/install/candle_ros2" \
    ROS_VERSION="2" \
    IMPEDANCE_CONFIG_PATH="$WS_DIR/src/ROS2/config/impedance_config.yaml" \
    /opt/ros/jazzy/bin/ros2 run gui gui_completa