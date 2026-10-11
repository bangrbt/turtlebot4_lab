# Cấu hình

SLAM, AMCL và Nav2 dùng nguyên cấu hình TurtleBot 4 Humble chính thức:
`$(ros2 pkg prefix turtlebot4_navigation)/share/turtlebot4_navigation/config/`.
Không sao chép driver/controller/simulator.

Hai RViz config tối giản dành cho bài thực hành: `odom.rviz` ở bước kiểm tra
robot chưa có map; `map.rviz` ở SLAM/AMCL/Nav2. Chúng có RobotModel, LaserScan,
TF, Map, costmap, plan, AMCL pose và công cụ 2D Pose Estimate/Nav2 Goal.
Robot description dùng Transient Local để nhận URDF đã phát trước khi RViz mở.
Cấu hình mặc định chính thức có Fixed Frame `map`, nên không đủ cho bước 1
khi chưa tồn tại map → odom; cấu hình riêng giải quyết trường hợp này.
