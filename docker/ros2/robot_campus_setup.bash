# Source on the enrolled robot as wheeltec. Does not start a node or arm motion.
# Installed as /home/wheeltec/nomad_live_control/campus/setup.bash.
source /home/wheeltec/nomad_live_control/setup.bash || return 1
export NOMAD_TRANSPORT=api
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE FASTDDS_ENVIRONMENT_FILE ROS_DISCOVERY_SERVER
unset SKIP_DEFAULT_XML RMW_FASTRTPS_USE_QOS_FROM_XML ROS_STATIC_PEERS ROS_AUTOMATIC_DISCOVERY_RANGE
