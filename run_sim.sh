#!/usr/bin/env bash
set -euo pipefail

# 可选：如果需要从任意路径运行，自动找到脚本所在目录作为基准
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$SCRIPT_DIR"   # 如果你把脚本放在leo_exploration_ws的父目录，这样可直接找到工作区
# 如果脚本放在其他地方，修改 WS_ROOT 指向包含 leo_exploration_ws 的目录

if [ ! -d "$WS_ROOT/leo_exploration_ws" ]; then
  echo "错误：在 $WS_ROOT 中未找到 leo_exploration_ws，请把脚本放在正确位置或修改 WS_ROOT。"
  exit 1
fi

echo "清理旧进程（如果存在）..."
pkill -f "gz sim"  || true
pkill -f nav2      || true
pkill -f slam      || true
pkill -f frontier  || true
pkill -f rviz2     || true

echo "运行依赖安装脚本..."
pushd "$WS_ROOT/leo_exploration_ws/src/leo_exploration/scripts" > /dev/null
chmod +x install_sim_deps.sh
./install_sim_deps.sh
popd > /dev/null

echo "构建包..."
pushd "$WS_ROOT/leo_exploration_ws" > /dev/null
colcon build --packages-select leo_exploration

echo "source setup.bash（仅在此脚本的进程内生效）..."
# 下面的 source 在本脚本的 shell 中生效，因此随后的 ros2 launch 可以使用环境
source install/setup.bash

echo "启动仿真（前台运行）..."
ros2 launch leo_exploration sim_exploration_launch.py
# 如果想在后台运行改成:
# ros2 launch leo_exploration sim_exploration_launch.py &

popd > /dev/null || true