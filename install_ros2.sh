#!/usr/bin/env bash
set -euo pipefail

# Script para instalar ROS2 (metapackages o lista de paquetes APT desde archivo)
# Uso:
#   ROS_DISTRO=jazzy ./install_ros2.sh
# o simplemente
#   ./install_ros2.sh

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
WORKDIR="$(cd "$(dirname "$0")" && pwd)"
PKG_FILE="$WORKDIR/ros2-system-packages.txt"

if [ "$(id -u)" -eq 0 ]; then
  echo "No ejecutes este script como root. Ejecuta sin sudo, el script pedirá sudo cuando haga falta."
  exit 1
fi

echo "Instalando ROS2 distro: $ROS_DISTRO"

echo "1) Instalando utilidades básicas..."
sudo apt update
sudo apt install -y locales curl gnupg lsb-release ca-certificates

echo "Configurando locale (por si acaso)..."
sudo locale-gen en_US en_US.UTF-8 || true
sudo update-locale LANG=en_US.UTF-8 || true
export LANG=en_US.UTF-8

echo "2) Añadiendo repositorio de ROS 2 y clave..."
sudo mkdir -p /etc/apt/keyrings
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /etc/apt/keyrings/ros-archive-keyring.gpg
echo "deb [signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

echo "3) Actualizando índices APT..."
sudo apt update

if [ -f "$PKG_FILE" ] && [ -s "$PKG_FILE" ]; then
  echo "Archivo de paquetes APT detectado: $PKG_FILE"
  echo "Instalando paquetes listados (puede tardar)..."
  # instalar por lotes (xargs instala todos los paquetes listados)
  sudo xargs -a "$PKG_FILE" apt install -y || {
    echo "Instalación de lista de paquetes falló; intentando instalar metapaquete ros-$ROS_DISTRO-desktop"
    sudo apt install -y "ros-$ROS_DISTRO-desktop"
  }
else
  echo "No se encontró $PKG_FILE, instalando metapaquete 'ros-$ROS_DISTRO-desktop' por defecto."
  sudo apt install -y "ros-$ROS_DISTRO-desktop"
fi

echo "4) Instalando rosdep (si no está) y actualizando rosdep..."
sudo apt install -y python3-rosdep2 || true
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init || true
fi
rosdep update || true

echo "Instalación completada (si no hubo errores)."
echo "Para usar ROS desde tu entorno conda, actívalo y luego source:/opt/ros/$ROS_DISTRO/setup.bash"
echo "Ejemplo:"
echo "  conda activate ros2rl && source /opt/ros/$ROS_DISTRO/setup.bash"

cat <<'USAGE'

Notas:
- El script instala paquetes del sistema con sudo (APT). No afecta al entorno conda.
- Si quieres más aislamiento, considera usar Docker o instalar ROS2 en un workspace y usar colcon.

USAGE

exit 0
