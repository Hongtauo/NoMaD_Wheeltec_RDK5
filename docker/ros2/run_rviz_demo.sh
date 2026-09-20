#!/usr/bin/env bash
# Run inside ros2-dev; start_rviz_viewer.ps1 uses the same local-only DDS profile.
set -eo pipefail
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
export ROS_DOMAIN_ID=180
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$project_root/docker/ros2/rviz_udp.xml"
source /opt/ros/humble/setup.bash
source "$project_root/ros2_ws/install/setup.bash"
# Refuse to mix replay with an already running pipeline in this preview domain.
python3 - <<'PY'
import time
import rclpy
rclpy.init()
node = rclpy.create_node('nomad_demo_preflight')
deadline = time.monotonic() + 2.0
while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
topics = ('/camera/image_raw', '/waypoint_stamped', '/cmd_vel_preview', '/nomad/visualization', '/cmd_vel')
busy = [topic for topic in topics if node.count_publishers(topic)]
node.destroy_node()
rclpy.shutdown()
if busy:
    raise SystemExit('Preview domain is busy. Stop its existing launch first: ' + ', '.join(busy))
PY
exec ros2 launch nomad_navigation rviz_demo.launch.py \
  project_root:="$project_root" use_rviz:=false
