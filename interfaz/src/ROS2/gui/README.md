# gui

This folder contains the graphical user interface (GUI) for the Assistive Exoskeleton ROS2 project. The GUI is designed to provide real-time control and monitoring of the exoskeleton hardware using ROS2 topics and services.

---

## Main Files

- **gui_assistive_exo.ui:**  
  The main graphical layout file, created and edited using Qt Designer. This file defines the GUI's visual structure.
- **gui_assistive_exo_data.py:**  
  The Python module generated automatically from `gui_assistive_exo.ui` using the Qt `pyuic` tool. It contains the classes and logic to instantiate the GUI defined in the `.ui` file.
- **gui_assistive_exo.py:**  
  The backend logic of the application. This script connects the GUI elements to ROS2 nodes, topics, and services, manages data flow, and implements control and visualization logic.

---

## Workflow: Generating Python Code from the .ui File

Whenever you modify `gui_assistive_exo.ui` using Qt Designer, regenerate the Python interface code by running:

```bash
pyuic6 -x gui_assistive_exo.ui -o gui_assistive_exo_data.py
```

If you use PyQt5, substitute `pyuic6` with `pyuic5`.

---

## Dependencies

- **Python 3.8+**
- **PyQt6** (or PyQt5)
    - Install with:  
      ```bash
      pip install PyQt6
      ```
      or for PyQt5:
      ```bash
      pip install PyQt5
      ```
- **matplotlib**
    - For plotting support (used in MplCanvas, potential/future use):
      ```bash
      pip install matplotlib
      ```
- **ROS2** (Humble/Foxy/Galactic, depending on your workspace)
- **rclpy** (ROS2 Python client library)
- **sensor_msgs, std_srvs** (from ROS2)
- **candle_ros2** and **srv_interface** packages (for exoskeleton and service integration)

---

## Backend Logic (`gui_assistive_exo.py`)

- Implements a PyQt6-based GUI for controlling and monitoring the exoskeleton.
- Visualizes real-time motor and IMU data.
- Allows sending commands to exoskeleton motors and updating control parameters.
- Supports recording and managing data with rosbag.
- Interacts with ROS2 topics and services for bi-directional communication with hardware.
- Key classes and variables:
    - **CAN IDs and MotorData:** Manages identification and telemetry for left/right hip motors.
    - **MplCanvas:** (For future data plotting)
    - **GuiNode:** ROS2 node for command publishing.
    - **MiVentana:** Main window class, handles user interaction, ROS2 subscriptions, service clients, and UI updates.
    - **rosbag_process:** Manages rosbag recording.
    - **Ui_MainWindow:** The Qt Designer-generated UI class loaded from `gui_assistive_exo_data.py`.

You can view the backend code for more details:  
[gui_assistive_exo.py](https://github.com/AssistiveExoskeleton/ROS2/blob/main/gui/gui/gui_assistive_exo.py)

---

## Example Launch

To run the GUI (inside your ROS2 workspace, with dependencies installed):

```bash
source install/setup.bash
bash ./launch_gui.sh
```

If you prefer a direct call (without the script), export the config path to avoid hard-coded paths:

```bash
export IMPEDANCE_CONFIG_PATH="$PWD/src/ROS2/config/impedance_config.yaml"
LD_PRELOAD=/lib/x86_64-linux-gnu/libpthread.so.0 ros2 run gui gui_assistive_exo
```

> Why: Some systems pull `libpthread` from Snap core images, which lack `GLIBC_PRIVATE` symbols. Forcing the system `libpthread` and a clean `LD_LIBRARY_PATH` avoids the symbol lookup error.

---

## Notes

- Always regenerate `gui_assistive_exo_data.py` after editing the `.ui` file in Qt Designer.
- Make sure your ROS2 environment is properly sourced before launching the GUI.
- For further details, review the code and comments in `gui_assistive_exo.py`.

---

## TODO

- In the impedance section, the interface must be completed to allow sending more data to the trajectory generation module.  
  Please also review or edit the logic in `trajectory_generation/impedance_node` to ensure full compatibility and data flow for trajectory generation.

---

