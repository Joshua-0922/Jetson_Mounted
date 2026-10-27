#!/usr/bin/env bash
# Companion-computer setup for Jetson Orin Nano (JetPack 7.x, Ubuntu 24.04, aarch64).
# Run as the normal user (not root); sudo is invoked where needed. Safe to re-run.
#
#   bash scripts/setup_jetson.sh            # ROS 2 Jazzy + uXRCE-DDS Agent + px4_msgs + YOLO
#   WITH_GZ=1 bash scripts/setup_jetson.sh  # also ros_gz_bridge / Gazebo Harmonic (SITL only)
#
# Ubuntu 24.04 cannot run ROS 2 Humble natively, so Jazzy is used. Nodes only need rclpy/px4_msgs
# and are unaffected by the distro change.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO_NAME=jazzy
PX4_MSGS_BRANCH=release/1.16            # must match the Pixhawk firmware (PX4 v1.16.x)
XRCE_AGENT_TAG=v2.4.2                   # version recommended by PX4 docs
YOLO_VENV="$HOME/yolo-venv"
SRC_DIR="$HOME/src"

log() { printf '\n\033[1;32m==> %s\033[0m\n' "$*"; }

[ "$(id -u)" -ne 0 ] || { echo "run as normal user, not root"; exit 1; }
sudo -v

log "Base build tools"
sudo apt-get update
sudo apt-get install -y build-essential cmake git curl wget gnupg lsb-release software-properties-common \
  python3-pip python3-venv python3-dev libasio-dev libtinyxml2-dev libssl-dev v4l-utils

log "ROS 2 $ROS_DISTRO_NAME apt source"
if ! dpkg -s ros2-apt-source >/dev/null 2>&1; then
  sudo add-apt-repository -y universe
  VER=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F '"tag_name"' | cut -d'"' -f4)
  curl -fL -o /tmp/ros2-apt-source.deb \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${VER}/ros2-apt-source_${VER}.$(. /etc/os-release && echo "$VERSION_CODENAME")_all.deb"
  sudo dpkg -i /tmp/ros2-apt-source.deb
  sudo apt-get update
fi

log "ROS 2 $ROS_DISTRO_NAME + build tools + camera/vision bridge"
sudo apt-get install -y ros-${ROS_DISTRO_NAME}-ros-base ros-dev-tools \
  ros-${ROS_DISTRO_NAME}-cv-bridge ros-${ROS_DISTRO_NAME}-image-transport ros-${ROS_DISTRO_NAME}-v4l2-camera \
  ros-${ROS_DISTRO_NAME}-ament-flake8 ros-${ROS_DISTRO_NAME}-ament-pep257 ros-${ROS_DISTRO_NAME}-ament-copyright \
  python3-colcon-common-extensions python3-rosdep python3-pytest
if [ "${WITH_GZ:-0}" = 1 ]; then
  sudo apt-get install -y ros-${ROS_DISTRO_NAME}-ros-gz
fi

log "Serial access for Pixhawk (USB -> /dev/pixhawk, TELEM2 -> /dev/ttyTHS1)"
sudo usermod -aG dialout "$USER"
# ModemManager probes new ttyACM ports with AT commands, which corrupts the uXRCE-DDS stream.
# 3185 = Auterion/Holybro PX4 FMU (e.g. Pixhawk 6C), 26ac = 3D Robotics/PX4 legacy.
sudo tee /etc/udev/rules.d/99-pixhawk.rules >/dev/null <<'EOF'
SUBSYSTEM=="tty", ATTRS{idVendor}=="3185", ENV{ID_MM_DEVICE_IGNORE}="1", SYMLINK+="pixhawk", MODE="0660", GROUP="dialout"
SUBSYSTEM=="tty", ATTRS{idVendor}=="26ac", ENV{ID_MM_DEVICE_IGNORE}="1", SYMLINK+="pixhawk", MODE="0660", GROUP="dialout"
EOF
sudo udevadm control --reload-rules && sudo udevadm trigger

log "Micro XRCE-DDS Agent $XRCE_AGENT_TAG (uXRCE-DDS bridge to PX4)"
if ! command -v MicroXRCEAgent >/dev/null; then
  mkdir -p "$SRC_DIR"
  [ -d "$SRC_DIR/Micro-XRCE-DDS-Agent" ] || git clone -b "$XRCE_AGENT_TAG" https://github.com/eProsima/Micro-XRCE-DDS-Agent.git "$SRC_DIR/Micro-XRCE-DDS-Agent"
  # Upstream deleted the Fast-DDS "2.12.x" branch the superbuild fetches; pin the last 2.12 tag instead.
  sed -i 's/set(_fastdds_tag 2\.12\.x)/set(_fastdds_tag v2.12.2)/' "$SRC_DIR/Micro-XRCE-DDS-Agent/CMakeLists.txt"
  if [ ! -x "$SRC_DIR/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent" ]; then
    rm -rf "$SRC_DIR/Micro-XRCE-DDS-Agent/build"   # drop stamps from a failed fetch/build
    # Fast-DDS 2.12 omits <cstdint>, which GCC 13 (Ubuntu 24.04) no longer pulls in transitively.
    # Exported (not -DCMAKE_CXX_FLAGS) so the superbuild's ExternalProjects inherit it.
    export CXXFLAGS="${CXXFLAGS:-} -include cstdint"
    cmake -S "$SRC_DIR/Micro-XRCE-DDS-Agent" -B "$SRC_DIR/Micro-XRCE-DDS-Agent/build" -DCMAKE_BUILD_TYPE=Release
    cmake --build "$SRC_DIR/Micro-XRCE-DDS-Agent/build" -j"$(nproc)"
    unset CXXFLAGS
  fi
  sudo cmake --install "$SRC_DIR/Micro-XRCE-DDS-Agent/build"
  sudo ldconfig /usr/local/lib/
fi

log "px4_msgs ($PX4_MSGS_BRANCH) + colcon build"
# shellcheck disable=SC1091
set +u; source /opt/ros/${ROS_DISTRO_NAME}/setup.bash; set -u
# A checkout on another branch means a different message ABI: replace it and its build outputs.
if [ -d "$REPO/ros2_ws/src/px4_msgs" ] && \
   [ "$(git -C "$REPO/ros2_ws/src/px4_msgs" rev-parse --abbrev-ref HEAD)" != "$PX4_MSGS_BRANCH" ]; then
  rm -rf "$REPO/ros2_ws/src/px4_msgs" "$REPO/ros2_ws/build/px4_msgs" "$REPO/ros2_ws/install/px4_msgs"
fi
if [ ! -d "$REPO/ros2_ws/src/px4_msgs" ]; then
  git clone -b "$PX4_MSGS_BRANCH" https://github.com/PX4/px4_msgs.git "$REPO/ros2_ws/src/px4_msgs"
fi
(cd "$REPO/ros2_ws" && colcon build --packages-select px4_msgs drone_controller drop_policy --symlink-install)

log "YOLO: PyTorch (CUDA 13) + Ultralytics in $YOLO_VENV"
# System-site-packages so the venv can import rclpy/cv_bridge from ROS. numpy<2 keeps cv_bridge ABI-compatible.
[ -d "$YOLO_VENV" ] || python3 -m venv --system-site-packages "$YOLO_VENV"
"$YOLO_VENV/bin/pip" install -U pip wheel
"$YOLO_VENV/bin/pip" install torch torchvision --index-url https://download.pytorch.org/whl/cu130
"$YOLO_VENV/bin/pip" install ultralytics "numpy<2" onnx pymavlink   # pymavlink: scripts/px4_usb_xrce.py
# TensorRT 10.x python bindings come from JetPack apt (python3-libnvinfer), visible via system-site-packages.
sudo apt-get install -y python3-libnvinfer python3-libnvinfer-dev || true

log "Shell environment"
grep -q "opt/ros/${ROS_DISTRO_NAME}/setup.bash" ~/.bashrc || cat >> ~/.bashrc <<EOF

# drone companion
source /opt/ros/${ROS_DISTRO_NAME}/setup.bash
[ -f "$REPO/ros2_ws/install/setup.bash" ] && source "$REPO/ros2_ws/install/setup.bash"
EOF

log "Done. Log out/in (or reboot) for the dialout group, then run scripts/verify_jetson.sh"
