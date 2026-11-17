#!/usr/bin/env bash
# Source ROS 2 Humble
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi

# Source workspace install
WS_DIR="/home/anhbang/turtlebot4_lab_ws"
if [ -f "$WS_DIR/install/setup.bash" ]; then
    source "$WS_DIR/install/setup.bash"
fi

# Activate Python virtual environment for DeepFace / OpenCV / MediaPipe
if [ -f "$WS_DIR/.venv_gesture/bin/activate" ]; then
    source "$WS_DIR/.venv_gesture/bin/activate"
fi

# Multi-robot / Localhost Isolation
export ROS_DOMAIN_ID=88
export ROS_LOCALHOST_ONLY=1
export IGN_IP=127.0.0.1
export IGN_PARTITION=turtlebot4_emotion_88

# DeepFace and Matplotlib Cache Locations
export DEEPFACE_HOME="$WS_DIR/.cache_emotion/deepface"
export MPLCONFIGDIR="/tmp/matplotlib"

echo "================================================================="
echo " [✓] Đã nạp thành công môi trường TurtleBot 4 Facial Emotion Control!"
echo "     • ROS_DOMAIN_ID=88 | ROS_LOCALHOST_ONLY=1"
echo "     • IGN_IP=127.0.0.1  | IGN_PARTITION=turtlebot4_emotion_88"
echo "     • Python venv: $WS_DIR/.venv_gesture"
echo "================================================================="
