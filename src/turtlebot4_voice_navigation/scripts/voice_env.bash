# Source in each bash terminal for TurtleBot 4 Voice Navigation (Lab 07).
_voice_pkg_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export TB4_VOICE_WS="$(cd -- "$_voice_pkg_dir/../.." && pwd)"

# Clean stale paths to avoid collision with other assignments
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH ROS_PACKAGE_PATH
unset IGN_GAZEBO_RESOURCE_PATH IGN_GUI_PLUGIN_PATH

# 1. Source ROS 2 Humble base
source /opt/ros/humble/setup.bash || return 1

# 2. Source Workspace overlay if built
if [[ -f "$TB4_VOICE_WS/install/setup.bash" ]]; then
    source "$TB4_VOICE_WS/install/setup.bash" || return 1
fi

# 3. Isolated domain and Ignition partition for Lab 07
export ROS_DOMAIN_ID=77
export ROS_LOCALHOST_ONLY=1
export IGN_PARTITION=turtlebot4_voice_77

# 4. Cache and model directories
export TB4_VOICE_CACHE="$TB4_VOICE_WS/.cache_voice"
export IGN_GAZEBO_RESOURCE_PATH="$TB4_VOICE_WS/src/turtlebot4_voice_navigation/models:${IGN_GAZEBO_RESOURCE_PATH:-}"
mkdir -p "$TB4_VOICE_CACHE"

# 5. Activate Python environment (reusing .venv_gesture which contains OpenCV, NumPy, Sounddevice)
if [[ -x "$TB4_VOICE_WS/.venv_gesture/bin/python" ]]; then
    source "$TB4_VOICE_WS/.venv_gesture/bin/activate"
    _py_site="$(python -c 'import sysconfig; print(sysconfig.get_path("purelib"))')"
    export PYTHONPATH="$_py_site:${PYTHONPATH:-}"
fi

unset _voice_pkg_dir _py_site
echo "[OK] Sourced TurtleBot 4 Voice Navigation environment (ROS_DOMAIN_ID=77, IGN_PARTITION=turtlebot4_voice_77)."
