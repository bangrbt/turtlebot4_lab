# TurtleBot 4 Lab: Simulation, SLAM Mapping, Localization & Nav2

Package ROS 2 Humble cung cấp môi trường mô phỏng, mapping, định vị và điều hướng tự động cho **TurtleBot 4** trong Ignition Gazebo (thế giới `maze`).

---

## 1. Mục tiêu bài tập

1. **Phần 1 – Simulation & Sensor Verification:**
   - Khởi chạy TurtleBot 4 Standard trong môi trường mô phỏng thế giới `maze`.
   - Kiểm tra robot hoạt động, dữ liệu cảm biến LiDAR (`/scan`), odometry (`/odom`), lệnh vận tốc (`/cmd_vel`) và hiển thị trên RViz2.
2. **Phần 2 – Xây dựng bản đồ (SLAM Toolbox):**
   - Khởi chạy SLAM Toolbox (chế độ async tối ưu).
   - Điều khiển robot khảo sát môi trường bằng teleoperation.
   - Xây dựng bản đồ 2D Occupancy Grid không bị méo/nhân đôi tường và lưu lại thành cặp file `map.yaml` và `map.pgm`.
3. **Phần 3 – Định vị robot (Localization với AMCL):**
   - Dừng SLAM, khởi động lại mô phỏng cùng môi trường.
   - Nạp bản đồ đã lưu từ Phần 2 và khởi chạy node AMCL.
   - Sử dụng công cụ *2D Pose Estimate* trên RViz2 để định vị robot, xác nhận chùm hạt hội tụ và laser scan khớp hoàn toàn với tường bản đồ.
4. **Phần 4 – Điều hướng tự động (Autonomous Navigation với Nav2):**
   - Khởi chạy ngăn xếp Nav2 với bản đồ đã tạo.
   - Sử dụng công cụ *Nav2 Goal* trên RViz2 để giao nhiệm vụ di chuyển đến ít nhất **03 vị trí khác nhau**.
   - Robot tự lập kế hoạch toàn cục (Global Path), né tránh chướng ngại vật thông qua Costmaps và về đích an toàn (`SUCCEEDED`).

---

## 2. Cấu trúc package

```text
turtlebot4_lab/
├── CMakeLists.txt                # Build configuration (ament_cmake)
├── package.xml                   # Package metadata và ROS 2 dependencies
├── README.md                     # Tài liệu hướng dẫn chi tiết gói package
├── launch/
│   ├── simulation.launch.py      # Launch TurtleBot 4 trong Gazebo + RViz2
│   ├── mapping.launch.py         # Launch SLAM Toolbox + RViz2 cấu hình map
│   ├── navigation.launch.py      # Launch Map Server, AMCL và Nav2
│   └── view.launch.py            # Mở nhanh giao diện RViz2 tùy chọn
├── config/
│   ├── odom.rviz                 # Cấu hình RViz2 cho frame odom & cảm biến ban đầu
│   ├── map.rviz                  # Cấu hình RViz2 đầy đủ (Map, Costmaps, Paths, AMCL pose)
│   └── slam_mapping.yaml         # Cấu hình tham số SLAM Toolbox tối ưu (async)
├── maps/
│   ├── map.yaml                  # Metadata bản đồ (độ phân giải 0.05m/pixel, origin)
│   └── map.pgm                   # Dữ liệu ảnh bản đồ maze đã khảo sát
└── scripts/
    ├── check_runtime.py          # Kiểm tra tự động trạng thái topics, TF, lifecycles
    ├── cleanup_simulation.py     # Dọn dẹp an toàn các tiến trình Gazebo/ROS còn sót
    ├── motion_probe.py           # Kiểm tra an toàn chuyển động và nhận lệnh /cmd_vel
    ├── record_goals.py           # Ghi nhận và theo dõi kết quả các goal Nav2
    ├── save_map.py               # Tiện ích lưu bản đồ chuẩn từ topic /map
    ├── teleop_hold.py            # Giao diện teleop đọc phím X11 giữ/thả phím mượt mà
    └── validate_map.py           # Kiểm tra tính toàn vẹn của cặp file bản đồ
```

---

## 3. Cài đặt và chuẩn bị môi trường

### 3.1. Cài đặt các gói phụ thuộc (Dependencies)
Mở terminal và cài đặt các package chính thức của TurtleBot 4 và công cụ hỗ trợ:

```bash
sudo apt-get update
sudo apt-get install -y \
  ros-humble-turtlebot4-simulator \
  ros-humble-turtlebot4-navigation \
  ros-humble-teleop-twist-keyboard \
  python3-colcon-common-extensions \
  python3-tk python3-xlib python3-yaml python3-pil
```

### 3.2. Thiết lập môi trường và Build Package
Để tránh xung đột với các robot khác cùng mạng, dự án sử dụng `ROS_DOMAIN_ID=64` và `ROS_LOCALHOST_ONLY=1`.

```bash
cd ~/turtlebot4_lab_ws

# 1. Source ROS 2 Humble
source /opt/ros/humble/setup.bash

# 2. Source script cấu hình môi trường
source lab_env.bash

# 3. Build workspace
colcon build --packages-select turtlebot4_lab --symlink-install

# 4. Source package vừa build
source install/setup.bash
```

> **Lưu ý:** Trước khi chạy bất kỳ terminal nào trong các bước tiếp theo, hãy luôn chạy:
> ```bash
> source ~/turtlebot4_lab_ws/lab_env.bash
> ```

---

## 4. Hướng dẫn chạy từng phần

### Phần 1 – Khởi chạy mô phỏng & Kiểm tra cảm biến

1. **Terminal 1: Khởi chạy Gazebo và RViz2:**
   ```bash
   source ~/turtlebot4_lab_ws/lab_env.bash
   ros2 launch turtlebot4_lab simulation.launch.py rviz:=true
   ```
   - Gazebo Fortress sẽ mở thế giới `maze` và nạp robot TurtleBot 4 tại vị trí dock sạc `(0, 0, 0)`.
   - Bấm nút **Play** ở góc dưới Gazebo nếu mô phỏng đang tạm dừng.

2. **Terminal 2: Undock robot khỏi trạm sạc:**
   ```bash
   source ~/turtlebot4_lab_ws/lab_env.bash
   ros2 action send_goal /undock irobot_create_msgs/action/Undock '{}' --feedback
   ```
   Robot sẽ tự động lùi ra khỏi trạm sạc và quay 180°.

3. **Kiểm tra dữ liệu trên terminal:**
   ```bash
   # Kiểm tra cảm biến LiDAR
   ros2 topic echo /scan sensor_msgs/msg/LaserScan --once --qos-reliability best_effort

   # Kiểm tra Odometry
   ros2 topic echo /odom nav_msgs/msg/Odometry --once --qos-reliability best_effort

   # Kiểm tra cây biến đổi tọa độ (TF)
   ros2 run tf2_ros tf2_echo odom base_link

   # Thử nghiệm di chuyển ngắn an toàn
   ros2 run turtlebot4_lab motion_probe.py
   ```
   Trên RViz2, xác nhận mô hình robot hiển thị đầy đủ và các tia quét LiDAR màu đỏ bao quanh các bức tường gần kề.

---

### Phần 2 – Xây dựng bản đồ (SLAM Toolbox)

1. **Terminal 1: Mở mô phỏng (tắt RViz để tránh mở 2 cửa sổ):**
   ```bash
   source ~/turtlebot4_lab_ws/lab_env.bash
   ros2 launch turtlebot4_lab simulation.launch.py rviz:=false
   ```

2. **Terminal 2: Khởi chạy SLAM Toolbox + RViz2 cấu hình Map:**
   ```bash
   source ~/turtlebot4_lab_ws/lab_env.bash
   ros2 launch turtlebot4_lab mapping.launch.py
   ```

3. **Terminal 3: Undock và mở công cụ điều khiển Teleop:**
   ```bash
   source ~/turtlebot4_lab_ws/lab_env.bash
   ros2 action send_goal /undock irobot_create_msgs/action/Undock '{}' --feedback
   ros2 run turtlebot4_lab teleop_hold.py --speed 0.40 --turn 0.8
   ```
   - Click vào cửa sổ **TurtleBot 4 - HOLD TELEOP**.
   - **Cách lái:** Giữ phím `W/A/S/D` hoặc các phím mũi tên để di chuyển, nhả phím robot sẽ tự động dừng ngay lập tức.
   - Điều khiển robot di chuyển chậm qua các hành lang và nhánh của mê cung để LiDAR quét toàn bộ không gian.
   - Quay lại các khu vực đã quét để thuật toán đóng vòng lặp (Loop Closure), giúp bản đồ khớp chính xác.

4. **Terminal 4: Lưu bản đồ:**
   Khi bản đồ trên RViz2 đã đầy đủ và không bị lỗi nhân đôi tường, chạy lệnh lưu:
   ```bash
   source ~/turtlebot4_lab_ws/lab_env.bash
   ros2 run turtlebot4_lab save_map.py --output ~/turtlebot4_lab_ws/src/turtlebot4_lab/maps/map
   ```
   Hai file bản đồ sẽ được tạo ra tại `src/turtlebot4_lab/maps/`:
   - `map.yaml`
   - `map.pgm`

5. **Dọn dẹp trước khi sang Phần 3:**
   - Tắt cửa sổ Teleop.
   - Nhấn `Ctrl+C` tại Terminal 2 (SLAM) và Terminal 1 (Simulation).
   - Đảm bảo node SLAM đã dừng hẳn:
     ```bash
     ros2 run turtlebot4_lab cleanup_simulation.py --stop
     ```

---

### Phần 3 – Localization (Định vị AMCL)

1. **Terminal 1: Khởi động lại mô phỏng sạch:**
   ```bash
   source ~/turtlebot4_lab_ws/lab_env.bash
   ros2 launch turtlebot4_lab simulation.launch.py rviz:=false
   ```

2. **Terminal 2: Khởi chạy Localization với bản đồ vừa lưu:**
   ```bash
   source ~/turtlebot4_lab_ws/lab_env.bash
   ros2 launch turtlebot4_lab navigation.launch.py nav2:=false \
     map:=$HOME/turtlebot4_lab_ws/src/turtlebot4_lab/maps/map.yaml
   ```

3. **Terminal 3: Undock robot:**
   ```bash
   source ~/turtlebot4_lab_ws/lab_env.bash
   ros2 action send_goal /undock irobot_create_msgs/action/Undock '{}' --feedback
   ```

4. **Ước lượng vị trí ban đầu (Initial Pose):**
   - Trên RViz2, chọn công cụ **2D Pose Estimate** trên thanh công cụ trên cùng.
   - Click chuột vào vị trí tương ứng với vị trí thực của robot trong Gazebo, giữ và kéo mũi tên theo hướng robot đang quay.
   - Quan sát chùm mũi tên xanh (hạt định vị AMCL) hội tụ xung quanh robot và các điểm laser scan màu đỏ khớp khít với các bức tường màu đen trên bản đồ.

---

### Phần 4 – Navigation với Nav2

1. **Terminal 3: Khởi chạy ngăn xếp điều hướng Nav2:**
   *(Giữ Terminal 1 Gazebo và Terminal 2 Localization đang chạy)*
   ```bash
   source ~/turtlebot4_lab_ws/lab_env.bash
   ros2 launch turtlebot4_navigation nav2.launch.py use_sim_time:=true
   ```

   *(Tùy chọn: Có thể chạy gộp cả Localization và Nav2 ngay từ đầu bằng một lệnh duy nhất ở Terminal 2:)*
   ```bash
   ros2 launch turtlebot4_lab navigation.launch.py nav2:=true \
     map:=$HOME/turtlebot4_lab_ws/src/turtlebot4_lab/maps/map.yaml
   ```

2. **Giao mục tiêu di chuyển tự động (Nav2 Goal):**
   - Trên thanh công cụ RViz2, chọn **Nav2 Goal**.
   - Click vào một điểm trống (Free Space) trên bản đồ và kéo mũi tên để chỉ định hướng dừng (Goal Pose).
   - **Quan sát quá trình di chuyển:**
     - Đường màu xanh lá (`/plan`) biểu thị quỹ đạo toàn cục (Global Plan).
     - Đường màu xanh dương (`/local_plan`) biểu thị quỹ đạo cục bộ tránh vật cản.
     - Lớp màu mờ xung quanh tường là Global/Local Costmaps (vùng nguy hiểm có nguy cơ va chạm).
     - Robot sẽ tự động tính toán vận tốc đưa về `/cmd_vel` và di chuyển đến điểm đích mà không va chạm.
   - Đặt lần lượt **ít nhất 03 Goal** tại 3 khu vực/ngã rẽ khác nhau trong mê cung để hoàn thành toàn bộ yêu cầu của bài.

---

## 5. Khắc phục sự cố thường gặp (Troubleshooting)

| Sự cố | Nguyên nhân | Cách xử lý |
|---|---|---|
| **Robot không nhận lệnh di chuyển** | Robot đang gắn trên trạm dock và tính năng an toàn khoá di chuyển | Chạy lệnh `ros2 action send_goal /undock irobot_create_msgs/action/Undock '{}' --feedback` trước khi teleop hoặc navigation. |
| **Không tìm thấy Topic / Node** | Sai ROS Domain hoặc biến môi trường bị phân mảnh | Đảm bảo tất cả các terminal đều chạy `source ~/turtlebot4_lab_ws/lab_env.bash` trước khi chạy lệnh. |
| **Bản đồ SLAM bị lệch hoặc nhân đôi tường** | Robot di chuyển hoặc xoay góc quá nhanh làm quá tải bộ đệm quét laser | Cấu hình `slam_mapping.yaml` đã được thiết lập `async` với `throttle_scans: 4`. Khi lái hãy giảm tốc độ xoay cua chậm để loop closure xử lý chuẩn. |
| **Tiến trình Gazebo cũ bị treo ngầm** | Đóng terminal đột ngột khiến daemon `ign gazebo` chưa giải phóng socket | Chạy script dọn dẹp: `ros2 run turtlebot4_lab cleanup_simulation.py --stop`. |
