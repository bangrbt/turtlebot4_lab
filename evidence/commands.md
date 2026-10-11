# Lệnh thực tế đã chạy

Môi trường ban đầu: đọc /etc/os-release; df -h; dpkg-query; ign gazebo --versions;
xdpyinfo trong và ngoài sandbox; apt-get -s install hai package TurtleBot 4.
Người dùng đã chạy sudo apt-get install ros-humble-turtlebot4-simulator
ros-humble-turtlebot4-navigation, sau đó xác minh package thật tại /opt/ros/humble.

Đã đối chiếu launch bằng ros2 pkg prefix và ros2 launch --show-args; source từ
package Humble đã cài và gói .deb chính thức được đọc trước khi cài.

```bash
colcon build --packages-select turtlebot4_lab --symlink-install
source install/setup.bash
ros2 launch turtlebot4_lab simulation.launch.py rviz:=true
# Sửa scope rviz sau khi phát hiện official include làm rviz=false rò ra parent.
ros2 launch turtlebot4_lab view.launch.py frame:=odom
ros2 run turtlebot4_lab check_runtime.py --stage simulation --output evidence/simulation_check.json
ros2 topic echo /dock_status --once
ros2 action list -t
ros2 node info /motion_control
ros2 action send_goal /undock irobot_create_msgs/action/Undock '{}' --feedback
ros2 run turtlebot4_lab motion_probe.py --output evidence/motion_probe.json
```

Mọi lệnh runtime source Humble + install bài, ROS_DOMAIN_ID=64. Giai đoạn 1 log
ở simulation.log, rviz_phase1.log, undock_phase1.log; ảnh thật PNG từ cửa sổ GUI.

Dừng launch bằng SIGINT đúng PID do agent tạo. Ignition để lại server/GUI;
xác minh từng PID/cmdline/domain trước khi dừng residual. Lần startup mapping
trước reset được giữ dưới startup_attempt_mapping*.log; không dùng làm bản đồ
hoàn chỉnh. Graph đã trống trước phiên sạch cuối cùng.

Phiên cuối: thêm IGN_PARTITION=turtlebot4_lab_64, làm sạch overlay môi trường,
khởi chạy simulation.launch.py rviz:=false và mapping.launch.py rviz:=true.
Các PID launch sống được ghi ở sessions.json; không khởi chạy thêm simulator/SLAM.
Chưa lưu map, chưa chạy AMCL/Nav2 với map tự tạo, chưa có ba goal.

Sau sửa lần cuối: simulation.launch.py rviz:=true xác minh lại wrapper mở đúng
RViz và controller spawner hoạt động (simulation_after_fix_check.json). Đóng RViz
odom riêng, giữ simulator; mapping.launch.py rviz:=true mở SLAM/RViz map.
Mapping check passed, Undock mapping SUCCEEDED. Terminal teleop đã mở để người dùng
khảo sát; hai override repeat_rate/key_timeout trong lần mở đầu không được bản
teleop 2.4.1 khai báo nên không có hiệu lực; speed/turn đúng, README đã sửa.
