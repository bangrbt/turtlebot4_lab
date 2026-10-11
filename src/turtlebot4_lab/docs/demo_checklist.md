# Checklist và kịch bản quay video

Video chưa được quay. Không tick bước chưa thực hiện.

1. (0:00–0:30) Giới thiệu OS/ROS, world maze và package độc lập. Hiện terminal source/build.
2. (0:30–1:00) Gazebo robot chạy, RViz LiDAR, /scan /odom clock/TF; undock nếu cần.
3. (1:00–3:30 hoặc timelapse rõ nhãn) Teleop tốc độ vừa phải, cho thấy map mở rộng,
   khảo sát hành lang/nhánh, loop closure; dùng dữ liệu mapping thật.
4. (3:30–4:00) Dừng teleop, map hoàn chỉnh, chạy saver/validator, hiện cặp YAML/PGM.
5. (4:00–4:30) Ctrl+C SLAM và simulation, node list không còn slam_toolbox; restart maze.
6. (4:30–5:00) Load map tự tạo, 2D Pose Estimate, scan khớp tường, TF map→odom→base.
7. (5:00–8:00 hoặc dài hơn nếu cần) Lifecycle active; gửi lần lượt ba Nav2 Goal qua RViz
   đến ba vị trí khác nhau, không teleop; giữ cảnh robot đi, đường/costmap và kết quả
   từng goal. Cho thấy tránh vật cản khi đường thực tế cần tránh; không suy diễn.
8. (cuối) Hiện bảng ba UUID/result + ảnh tới đích; link repo/video khi đã upload.

- [ ] Chuẩn bị screen recorder của Ubuntu, chọn cả Gazebo/RViz/terminal cần thiết.
- [ ] Ghi log simulation/SLAM/AMCL/Nav2; mở recorder JSONL trước goal đầu tiên.
- [ ] Không ghi mật khẩu/sensitive terminal lên video.
- [ ] Chụp 01_gazebo, 02_rviz_lidar, 03_mapping, 04_saved_map, 05_localization,
      06_goal1, 07_goal2, 08_goal3 PNG thực tế trong evidence/.
- [ ] Kiểm tra map/goal bằng log + quan sát, ghi cả lỗi/thử lại.
- [ ] Không commit video/rosbag lớn; upload sau và kiểm tra link quyền xem.
- [ ] Điền báo cáo theo mẫu LaTeX khi được cung cấp.
