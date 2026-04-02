#!/bin/bash
set -e

# ROS2 Humble 환경 설정 로드
source "/opt/ros/humble/setup.bash"

# 빌드된 워크스페이스가 있다면 로드 (선택 사항)
if [ -f "/ros2_ws/install/setup.bash" ]; then
  source "/ros2_ws/install/setup.bash"
fi

exec "$@"