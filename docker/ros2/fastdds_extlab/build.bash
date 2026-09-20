#!/usr/bin/env bash
# Execute ONLY in an independently created osrf/ros:humble-desktop-full-jammy lab.
# This is an experimental 2.10.7 overlay, not a production upgrade recommendation.
set -eo pipefail
source /opt/ros/humble/setup.bash
mkdir -p /opt/extlab/src
if [[ ! -d /opt/extlab/src/Fast-DDS ]]; then
  git clone --depth 1 --branch v2.10.7 https://github.com/eProsima/Fast-DDS.git /opt/extlab/src/Fast-DDS
fi
if [[ ! -d /opt/extlab/src/rmw_fastrtps ]]; then
  git clone --depth 1 --branch 6.2.10 https://github.com/ros2/rmw_fastrtps.git /opt/extlab/src/rmw_fastrtps
fi
test "$(git -C /opt/extlab/src/Fast-DDS rev-parse HEAD)" = 92a890c1dea08682d2fc83974d96cebc4b7feb9b
test "$(git -C /opt/extlab/src/rmw_fastrtps rev-parse HEAD)" = cbe12977497c9c536317ca6488edf2f23b8c1830
cmake -S /opt/extlab/src/Fast-DDS -B /opt/extlab/build/fastdds \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/extlab/fastdds \
  -DBUILD_TESTING=OFF -DCOMPILE_TOOLS=OFF -DCOMPILE_EXAMPLES=OFF \
  -DSECURITY=ON -DTHIRDPARTY=ON
cmake --build /opt/extlab/build/fastdds --parallel 4
cmake --install /opt/extlab/build/fastdds
export CMAKE_PREFIX_PATH="/opt/extlab/fastdds:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="/opt/extlab/fastdds/lib:${LD_LIBRARY_PATH:-}"
export CMAKE_BUILD_PARALLEL_LEVEL=4
export MAKEFLAGS=-j4
mkdir -p /opt/extlab/rmw_ws
cd /opt/extlab/rmw_ws
colcon build --base-paths /opt/extlab/src/rmw_fastrtps \
  --packages-select rmw_fastrtps_shared_cpp rmw_fastrtps_cpp --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DSECURITY=ON \
  -Dfastrtps_DIR=/opt/extlab/fastdds/share/fastrtps/cmake
