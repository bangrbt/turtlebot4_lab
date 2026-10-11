# Nội dung báo cáo — chờ mẫu LaTeX

Đây là bản nội dung để đưa vào mẫu người dùng cung cấp, chưa phải báo cáo hoàn tất.
Chỉ điền “đạt” khi có log/ảnh thực tế. Xem evidence/RESULTS.md và status.json.

## Mục tiêu và kiến trúc

Ubuntu 22.04, ROS 2 Humble, Ignition Gazebo Fortress; world maze chính thức;
package ament_cmake turtlebot4_lab với bốn wrapper launch, RViz config và scripts.
Simulator/Create 3/driver/controller, SLAM, AMCL và Nav2 dùng package chính thức.

Pipeline: Gazebo → bridge → /scan + /odom + TF → SLAM Toolbox → /map → lưu
map.yaml/map.pgm → restart cùng world → map server + AMCL → map→odom→base_link
→ Nav2 planner (global plan) → controller (local command) → velocity smoother
→ /cmd_vel → Create 3 motion_control → controller/bridge → robot Gazebo.
Đường topic vận tốc cuối phải đối chiếu endpoint thực tế, không chỉ theo sơ đồ.

## Topic/node chính (xác minh runtime trước khi nộp)

| Topic | Kiểu dự kiến | Vai trò |
|---|---|---|
| /clock | rosgraph_msgs/msg/Clock | thời gian mô phỏng |
| /scan | sensor_msgs/msg/LaserScan | LiDAR cho SLAM/AMCL/costmap |
| /odom | nav_msgs/msg/Odometry | odometry từ Create 3 |
| /tf, /tf_static | tf2_msgs/msg/TFMessage | quan hệ frame |
| /robot_description | std_msgs/msg/String | URDF hiển thị RViz |
| /map | nav_msgs/msg/OccupancyGrid | bản đồ occupancy |
| /amcl_pose | geometry_msgs/msg/PoseWithCovarianceStamped | pose localization |
| /initialpose | geometry_msgs/msg/PoseWithCovarianceStamped | initial pose RViz |
| /plan, /local_plan | nav_msgs/msg/Path | đường global/local |
| /global_costmap/costmap, /local_costmap/costmap | nav_msgs/msg/OccupancyGrid | costmap |
| /cmd_vel | geometry_msgs/msg/Twist | lệnh vận tốc vào motion control |
| /diffdrive_controller/cmd_vel_unstamped | geometry_msgs/msg/Twist | lệnh controller đã qua motion control |
| /navigate_to_pose | nav2_msgs/action/NavigateToPose (action) | giao nhiệm vụ và nhận kết quả |

Node chính: clock/lidar/pose/cmd_vel bridge, robot_state_publisher,
motion_control, slam_toolbox (mapping), map_server và amcl (localization),
planner_server/controller_server/bt_navigator/behavior_server/velocity_smoother (Nav2).
Không chạy SLAM và AMCL đồng thời.

## Quá trình và kết quả

- Môi trường: đã kiểm tra Ubuntu 22.04.5, Humble, Fortress 6.18.0, ổ còn khoảng 34 GB.
- Build/package: 1 package finished; underlay chỉ install bài + /opt/ros/humble (evidence/build.txt, underlay.txt).
- Simulation/LiDAR/odom/TF/chuyển động: topic/TF check passed, scan 640 tia, undock SUCCEEDED; motion probe đi 0.114 m và vận tốc cuối gần 0; ảnh thật 01_gazebo.png, 02_rviz_lidar.png.
- Mapping: SLAM readiness passed, map ban đầu 341×340 ô, resolution 0.05 m (03_slam_initial.png); CHƯA khảo sát đủ và CHƯA lưu cặp map.
- Restart/AMCL: [CHƯA CHẠY với map tự tạo].
- Ba goal: [CHƯA CHẠY; không ghi thành công].

| Goal | x/y/yaw trên map | UUID/action result | Tới đích quan sát | Tránh vật cản quan sát | Log/ảnh |
|---|---|---|---|---|---|
| 1 | CHƯA CÓ | CHƯA CÓ | CHƯA QUAN SÁT | CHƯA QUAN SÁT | CHƯA CÓ |
| 2 | CHƯA CÓ | CHƯA CÓ | CHƯA QUAN SÁT | CHƯA QUAN SÁT | CHƯA CÓ |
| 3 | CHƯA CÓ | CHƯA CÓ | CHƯA QUAN SÁT | CHƯA QUAN SÁT | CHƯA CÓ |

## Hình minh họa

[CHỜ ảnh THẬT]: Gazebo robot/world, RViz robot/LiDAR, mapping và map hoàn chỉnh,
map được load lại/AMCL khớp scan, ba goal. Không dùng ảnh dựng hoặc ảnh tài liệu
để thay bằng chứng bài thực hành.

## Nội dung cần hiểu

[CHỜ đề bài/mục câu hỏi do người dùng cung cấp khi hoàn thiện báo cáo].

## Liên kết

GitHub: [CHƯA CÓ — chưa push]. Video: [CHƯA CÓ — chưa quay/upload].
Mẫu LaTeX/PDF: [CHỜ mẫu người dùng].
