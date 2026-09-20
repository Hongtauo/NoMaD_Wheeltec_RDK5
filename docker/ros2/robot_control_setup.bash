# Source this file on the robot after deployment to nomad_live_control/setup.bash.
# It only configures ROS/Python/DDS; it never starts nodes or arms motion.
source /opt/ros/humble/setup.bash || return 1
source /opt/tros/humble/setup.bash || return 1
source /home/wheeltec/wheeltec_ros2/install/setup.bash || return 1
source /home/wheeltec/nomad_interfaces_ws/install/local_setup.bash || return 1
export PYTHONPATH="/home/wheeltec/nomad_live_control${PYTHONPATH:+:$PYTHONPATH}"
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/wheeltec/nomad_live_control/robot_onboard_udp.xml
unset FASTDDS_DEFAULT_PROFILES_FILE ROS_DISCOVERY_SERVER SKIP_DEFAULT_XML
