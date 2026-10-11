# Cấu hình

AMCL và Nav2 dùng cấu hình TurtleBot 4 Humble chính thức. SLAM dùng
`slam_mapping.yaml` được tạo từ `turtlebot4_navigation/config/slam.yaml`, giữ solver,
frame, resolution 0.05 m và loop closure. Sau log sync queue hơn 4000 scan và TF
extrapolation, mapping mặc định chuyển async, throttle_scans=4, khoảng di chuyển/
quay tối thiểu 0.10, map update 1 s, scan_queue_size=1, min laser range 0.2 m.
Không thay driver/controller/simulator.

RViz: odom.rviz cho kiểm tra chưa có map; map.rviz cho SLAM/AMCL/Nav2.
Robot description dùng Transient Local, LaserScan dùng Best Effort.
