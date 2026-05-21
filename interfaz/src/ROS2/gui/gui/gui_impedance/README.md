# Impedance-Only GUI Setup Instructions

This GUI is a simplified version focused exclusively on impedance control mode.

## Files Created

```
src/ROS2/gui/gui/gui_impedance/
├── __init__.py                    # Package init
├── gui_impedance.py               # Main GUI application (⚠️ needs custom .ui file)
├── gui_impedance_data.py          # Auto-generated from .ui (placeholder, will be overwritten)
├── gui_impedance.ui               # ⚠️ CREATE THIS IN QT DESIGNER
├── impedance_config_py.py         # (copied from parent gui/)
├── core/                          # (copied from parent gui/)
│   ├── constants.py
│   └── state.py
```

launch_gui_impedance.sh            # Launcher script in workspace root

## Next Steps

### 1. **Create the GUI Design File**

You need to create `gui_impedance.ui` in Qt Designer with impedance-specific controls. Place the file at:
```
src/ROS2/gui/gui/gui_impedance/gui_impedance.ui
```

**Important:** Use the same widget names as the main GUI for consistency:
- `enable_left_motor` (QCheckBox)
- `enable_right_motor` (QCheckBox)
- `add_motors_btn` (QPushButton)
- `left_hip_manual_enable` (QCheckBox)
- `left_hip_manual_box` (QSpinBox/QDoubleSpinBox)
- `right_hip_manual_enable` (QCheckBox)
- `right_hip_manual_box` (QSpinBox/QDoubleSpinBox)
- `left_hip_sine_enable` (QCheckBox)
- `right_hip_sine_enable` (QCheckBox)
- `left_start_traj_btn` (QPushButton)
- `left_stop_traj_btn` (QPushButton)
- `right_start_traj_btn` (QPushButton)
- `right_stop_traj_btn` (QPushButton)
- `update_impedance_parameters_btn_izq` (QPushButton)
- `update_impedance_parameters_btn_der` (QPushButton)
- Motor telemetry labels (optional but recommended)

### 2. **Generate Python Code from UI**

After creating `gui_impedance.ui` in Qt Designer, run:

```bash
cd src/ROS2/gui/gui/gui_impedance/
pyuic6 -x gui_impedance.ui -o gui_impedance_data.py
```

This will generate the `gui_impedance_data.py` file (replacing the placeholder).

### 3. **Build the Package**

```bash
cd /path/to/assistive_exo_ws
colcon build --packages-select gui
```

### 4. **Run the GUI**

Option A - Using the launcher script:
```bash
./launch_gui_impedance.sh
```

Option B - Using ROS2 directly:
```bash
source install/setup.bash
ros2 run gui gui_impedance
```

## Key Differences from Main GUI

- ❌ **Removed:** Velocity control, Position control tabs
- ❌ **Removed:** Motion command publishers
- ✅ **Kept:** Only impedance control logic
- ✅ **Kept:** Motor telemetry display
- ✅ **Kept:** IMU data display
- ✅ **Kept:** Configuration system
- ✅ **Kept:** Motor enable/disable

## Configuration

The impedance parameters are loaded from:
```
src/ROS2/config/impedance_config.yaml
```

The launcher script automatically sets the `IMPEDANCE_CONFIG_PATH` environment variable.

## Widget Signal Connections

The main application (`gui_impedance.py`) tries to connect to UI widgets dynamically using `hasattr()`. This means:

- ✅ UI widgets are **optional** (won't crash if missing)
- ✅ Can add/remove widgets without changing Python code
- ✅ Safe to use with different UI layouts

However, for full functionality, ensure all control widgets listed above are present in your `.ui` file.

## Notes

- The `impedance_config_py.py` and `core/` files are copied from the main GUI
- Widget names use Spanish naming convention (e.g., `cadera_izq` = left hip)
- All functionality is logged to ROS2 logger
- Motor telemetry updates every 250ms (configurable via `setpoint_timer`)
