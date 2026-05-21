#!/usr/bin/env bash
set -eo pipefail

# Usage:
#   ./run_rl_env.sh
#
# What it does:
# 1) Sanitizes Python user site-packages interference
# 2) Sources ROS Jazzy + candle workspace
# 3) Builds rl_interfaces + rl_package (compatible with existing install layout)
# 4) Sources this workspace
# 5) Verifies ROS package visibility

WS_RL="${HOME}/Escritorio/TFG/code_tfg/inferencia"
WS_CANDLE="${HOME}/Escritorio/ros2_candle_ws"

cd "${WS_RL}"

export PYTHONNOUSERSITE=1

# Source setup scripts with nounset disabled (ROS/colcon scripts may read unset vars)
set +u
source /opt/ros/jazzy/setup.bash
source "${WS_CANDLE}/install/setup.bash"
set -u

COLCON_ARGS=(--packages-select rl_interfaces rl_package)

# Respect existing install layout to avoid colcon conflicts
if [[ -f install/.colcon_install_layout ]]; then
	layout="$(cat install/.colcon_install_layout)"
	if [[ "${layout}" == "merged" ]]; then
		COLCON_ARGS=(--merge-install "${COLCON_ARGS[@]}")
	fi
fi

colcon build "${COLCON_ARGS[@]}"

set +u
source install/setup.bash
set -u

echo
printf "[OK] Environment ready. Visible packages:\n"
ros2 pkg list | grep -E '^rl_interfaces$|^rl_package$' || true
