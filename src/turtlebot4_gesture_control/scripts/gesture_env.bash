# Source in each bash terminal for this package; isolated from Lab04/Lab05.
_gesture_pkg_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export TB4_GESTURE_WS="$(cd -- "$_gesture_pkg_dir/../.." && pwd)"
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH ROS_PACKAGE_PATH
unset IGN_GAZEBO_RESOURCE_PATH IGN_GUI_PLUGIN_PATH
source /opt/ros/humble/setup.bash || return 1
if [[ -f "$TB4_GESTURE_WS/install/setup.bash" ]]; then source "$TB4_GESTURE_WS/install/setup.bash" || return 1; fi
export ROS_DOMAIN_ID=73
export ROS_LOCALHOST_ONLY=1
export IGN_PARTITION=turtlebot4_gesture_73
export ROS_LOG_DIR="$TB4_GESTURE_WS/log/gesture_ros"
export TB4_GESTURE_CACHE="$TB4_GESTURE_WS/.cache_gesture"
export MPLCONFIGDIR="$TB4_GESTURE_CACHE/matplotlib"
export JAX_PLATFORMS=cpu
export OMP_NUM_THREADS=1
export TF_CPP_MIN_LOG_LEVEL=2
mkdir -p "$ROS_LOG_DIR" "$TB4_GESTURE_CACHE"
if [[ -x "$TB4_GESTURE_WS/.venv_gesture/bin/python" ]]; then
    source "$TB4_GESTURE_WS/.venv_gesture/bin/activate"
    _gesture_site="$(python -c 'import sysconfig; print(sysconfig.get_path("purelib"))')"
    export PYTHONPATH="$_gesture_site:${PYTHONPATH:-}"
fi
unset _gesture_pkg_dir _gesture_site
