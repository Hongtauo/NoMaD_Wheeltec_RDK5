# Source only in nomad-fastdds-extlab-20260909 (or a reproduced lab).
# Do not add to ros2-dev or robot shell startup files.
source /opt/ros/humble/setup.bash
source /opt/extlab/rmw_ws/install/local_setup.bash
export LD_LIBRARY_PATH="/opt/extlab/fastdds/lib:${LD_LIBRARY_PATH:-}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
