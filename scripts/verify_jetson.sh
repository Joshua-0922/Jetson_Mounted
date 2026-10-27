#!/usr/bin/env bash
# Checks that the companion-computer stack installed by setup_jetson.sh works.
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
set +u
source /opt/ros/jazzy/setup.bash 2>/dev/null
source "$REPO/ros2_ws/install/setup.bash" 2>/dev/null
ok() { printf '  [ok]   %s\n' "$1"; }
no() { printf '  [FAIL] %s\n' "$1"; }
chk() { local d=$1; shift; if "$@" >/dev/null 2>&1; then ok "$d"; else no "$d"; fi; }

chk "ros2 CLI"                 ros2 --help
chk "MicroXRCEAgent"           command -v MicroXRCEAgent   # -h exits non-zero
chk "px4_msgs importable"      python3 -c "from px4_msgs.msg import TrajectorySetpoint, VehicleOdometry"
chk "drop_policy installed"    ros2 pkg prefix drop_policy
chk "drone_controller installed" ros2 pkg prefix drone_controller
chk "policy golden tests"      python3 -m pytest "$REPO/ros2_ws/src/drop_policy/test" -q
chk "TensorRT (system python)" python3 -c "import tensorrt"
chk "dialout group active"     bash -c "id -nG | grep -qw dialout"
echo "  torch / CUDA:"
"$HOME/yolo-venv/bin/python" -c "
import torch, ultralytics
print('   torch', torch.__version__, 'cuda', torch.cuda.is_available(), torch.cuda.get_device_name(0) if torch.cuda.is_available() else '')
print('   ultralytics', ultralytics.__version__)
import numpy; print('   numpy', numpy.__version__)
" 2>&1 | sed 's/^/  /'
