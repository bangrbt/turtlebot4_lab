# Source this file in each terminal; no user-specific path in source.
if [[ -z "${BASH_VERSION:-}" ]]; then
    echo "LOI: hay dung bash de source lab_env.bash" >&2
    return 1
fi
export TB4_LAB_WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH ROS_PACKAGE_PATH
unset IGN_GAZEBO_RESOURCE_PATH IGN_GUI_PLUGIN_PATH GAZEBO_MODEL_PATH
source /opt/ros/humble/setup.bash || return 1
if [[ ! -f "$TB4_LAB_WS/install/setup.bash" ]]; then
    echo "LOI: chua build workspace. Hay colcon build sau khi source Humble." >&2
    return 1
fi
source "$TB4_LAB_WS/install/setup.bash" || return 1
export ROS_DOMAIN_ID=64
export ROS_LOCALHOST_ONLY=1
export IGN_PARTITION=turtlebot4_lab_64
export ROS_LOG_DIR="$TB4_LAB_WS/log/ros"
mkdir -p "$ROS_LOG_DIR" "$TB4_LAB_WS/evidence"
