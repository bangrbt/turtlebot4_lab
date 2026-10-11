# Đối chiếu yêu cầu

| Yêu cầu | Tệp/bằng chứng | Điều kiện hoàn tất |
|---|---|---|
| Workspace độc lập | source dưới turtlebot4_lab_ws; install/setup.bash | underlay chỉ /opt/ros/humble |
| Package/launch Humble | package.xml, CMakeLists.txt, launch/; evidence/launch_args.txt | build và launch chạy |
| Robot/Gazebo/RViz | simulation.launch.py, config/odom.rviz; simulation.log, ảnh thật | robot/LiDAR nhìn thấy |
| Clock/scan/odom/TF | scripts/check_runtime.py; simulation_check.json | dữ liệu/clock tăng, TF đúng |
| Lệnh vận tốc/di chuyển/dừng | motion_probe.py, motion_probe.json, endpoints | dịch chuyển và dừng có bằng chứng |
| Mapping thật | mapping.launch.py; mapping.log, mapping_check.json, ảnh | người dùng teleop khảo sát |
| Map YAML/PGM portable | save_map.py, validate_map.py; maps/map.* | map thật, image tương đối |
| Dừng SLAM/restart cùng world | README giai đoạn 2–3; node list/log restart | không slam_toolbox khi AMCL |
| AMCL/localization | navigation.launch.py nav2:=false; localization_check.json | pose/TF/scan khớp map |
| Nav2 readiness | nav2 chính thức, navigation_check.json | lifecycle/costmap/action sẵn sàng |
| Ba goal RViz khác nhau | record_goals.py; goals.jsonl, ảnh/video | action + tới đích, không teleop |
| README A–Z/dependency | README.md | kiểm chứng theo phiên thực tế |
| Repository Git | .gitignore, Git local | chưa push nếu chưa có remote |
| Báo cáo LaTeX/PDF | docs/report_content.md | chờ mẫu và kết quả thật |
| Video/GitHub | docs/demo_checklist.md | người dùng quay, cung cấp link/remote |

Trạng thái thực tế cập nhật ở evidence/RESULTS.md; bảng này không tự xác nhận test đạt.
