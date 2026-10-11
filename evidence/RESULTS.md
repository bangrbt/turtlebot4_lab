# Kết quả chạy thực tế — 11/10/2026 (giờ Việt Nam)

## Kiểm tra máy và triển khai package

Ubuntu 22.04.5, ROS 2 Humble, Ignition Fortress 6.18.0; khoảng 34 GB trống lúc kiểm tra.
GUI display :1 truy cập được ngoài sandbox; sandbox không kết nối được display.
Ban đầu thiếu TurtleBot 4. Người dùng cài simulator/navigation bằng sudo; sau đó
đã xác minh package/launch thực tại /opt/ros/humble bằng ros2 pkg/--show-args.
Build cuối: **1 package finished** (`build.txt`). Underlay chỉ install bài và
/opt/ros/humble (`underlay.txt`). CLI help và syntax Python đã kiểm tra; package
launch/script/config được install. Git đã init local; chưa push/upload.

## Giai đoạn 1: đạt kiểm tra cơ bản

- Simulator maze và robot được quan sát trong GUI, ảnh `01_gazebo.png`.
- Robot/LiDAR trong RViz odom, ảnh `02_rviz_lidar.png`; robot nhỏ ở giữa hình.
- `simulation_check.json` và `simulation_after_fix_check.json`: clock/scan/odom
  timestamps tăng, scan 640 tia, TF odom→base_link→turtlebot4/rplidar_link/rplidar.
- `/cmd_vel`: bridge phát, motion_control nhận; motion_control phát
  `/diffdrive_controller/cmd_vel_unstamped`, controller và bridge nhận.
- Undock **SUCCEEDED** (`undock_phase1.log`).
- Movement probe: dịch chuyển **0.114 m**, final linear x ≈ -1.19e-16 m/s,
  angular z ≈ -5.92e-17 rad/s, `passed: true` (`motion_probe.json`).

## Giai đoạn 2: SLAM sẵn sàng, CHƯA hoàn thành khảo sát/lưu map

Simulation/SLAM đã reset sạch, dùng IGN_PARTITION=turtlebot4_lab_64 và domain 64.
Phiên đang giữ chạy: simulation launch PID 13902, mapping launch PID 15025;
PID chỉ là snapshot, phải xác minh trước khi dừng. RViz odom kiểm tra wrapper đã
đóng, RViz map đang mở từ mapping wrapper. Undock phiên này **SUCCEEDED**.

`mapping_check.json`: `passed: true`, map ban đầu **341 × 340**, resolution **0.05 m**,
map→odom TF tồn tại, không AMCL/map server. Đây là kiểm tra readiness, không xác
nhận phủ phần lớn khu vực hay chất lượng map. Ảnh `03_slam_initial.png` chỉ là map
ban đầu. Terminal teleop đã mở tốc độ 0.15 m/s, 0.4 rad/s. Chờ người dùng khảo sát.

`maps/` hiện chỉ có README; **chưa có map.yaml/map.pgm**, không map giả/map mẫu.
Missing-map guard đã xác minh từ chối trước AMCL (`missing_map_guard.log`).

## Giai đoạn 3–4 và báo cáo/video: CHƯA chạy/hoàn tất

AMCL/Nav2 với map tự tạo chưa chạy; không có ba goal hoặc bằng chứng tránh vật cản.
Source/wrapper/recorder và hướng dẫn đã chuẩn bị, chưa coi là test runtime đạt.
Nội dung báo cáo/checklist có placeholder rõ; chờ mẫu LaTeX, câu hỏi đề, ảnh và link.

## Lỗi thực tế đã xử lý và giới hạn

- Build đầu thiếu docs/: tạo docs rồi build đạt.
- Official include rviz=false làm rò argument: sửa dùng argument riêng cho view.
- Scope cả official launch làm delayed spawner mất namespace: giữ context chính
  thức, tách argument view; runtime sau sửa có controller active và RViz.
- Ignition để lại server/GUI sau shutdown: dừng đúng PID đã xác minh; thêm partition
  và cleanup_simulation.py chỉ xử lý process có partition/domain của bài.
- RViz 2 Humble segfault/hang khi đóng cửa sổ được ghi ở log; phiên mới mở và nhận
  map bình thường. Không gọi shutdown GUI đạt sạch nếu log chưa chứng minh.
- Log có cảnh báo rendering/material, sensor range SLAM và controller update
  period 0.001 s so với simulation step 0.003 s của package chính thức. Không sửa
  simulator/controller; dữ liệu và movement đã đạt trong phép thử ngắn, không
  suy ra toàn bộ navigation đáng tin cậy từ kiểm tra này.
- Bản teleop 2.4.1 chỉ có speed/turn/stamped/frame_id, không repeat_rate/key_timeout.
  README đã sửa theo bản cài; terminal đang mở nhận đúng speed 0.15/turn 0.4.

- ROS CLI daemon đôi lúc báo Node not found hoặc endpoint chưa đủ; dùng discovery
  trực tiếp `--no-daemon --spin-time 3` và script check_runtime để xác minh lại.

## Thao tác người dùng hiện cần thực hiện

1. Khảo sát bằng terminal TELEOP: i tiến, j/l quay, , lùi; nhấn k trước đổi focus.
2. Khi map đủ, k rồi Ctrl+C dừng teleop, giữ simulator/SLAM và báo agent để lưu map.
3. Sau khi agent kiểm tra map/restart/AMCL, đặt initial pose và gửi ba goal qua RViz.
4. Quay video, cung cấp mẫu LaTeX và phần Nội dung cần hiểu; link repo/video sau upload.

README A–Z và bảng `docs/requirements.md` chỉ đường tới source/bằng chứng từng yêu cầu.

## Cập nhật teleop giữ/thả phím

Thay teleop terminal bằng scripts/teleop_hold.py: đọc trực tiếp trạng thái phím
X11, không chờ chữ từ bộ gõ Bamboo đang bật. Giữ WASD/mũi tên để chạy, thả để dừng;
chỉ nhận điều khiển khi cửa sổ của bài có focus; phát /cmd_vel 20 Hz. Mặc định
0.25 m/s, quay 0.6 rad/s. Không đổi driver/controller. Dependency Tkinter/Xlib đã có.

Build: build_hold_teleop.txt đạt. Test X11 + ROS thật: giữ W 0.832 s không Enter,
15 lệnh nonzero, command speed 0.25 m/s, đi 0.144 m; lệnh về zero sau 0.021 s từ
release và final odom velocity gần 0 (hold_teleop_check.json). Các lần kiểm tra
chưa đạt được giữ riêng initial_/focus_attempt_ để không lẫn với test cuối.
Teleop cũ dừng, chỉ giữ cửa sổ HOLD TELEOP mới; map chưa lưu và AMCL/Nav2 chưa chạy.

## Cập nhật tốc độ teleop và tải SLAM

Mặc định teleop mới 0.40 m/s, quay 0.8 rad/s. Giới hạn tiến/lùi 0.46 m/s đã đọc
trực tiếp từ diffdrive_controller linear.x.max_velocity. Mapping chuyển async
với config/slam_mapping.yaml dựa cấu hình Humble; throttle_scans=4, scan_queue_size=1,
minimum_travel_distance/heading=0.10, map_update_interval=1 s. Build đạt trong
build_speed_update.txt. Chưa suy ra chất lượng toàn bộ map từ kiểm tra readiness.

SLAM/teleop cũ đã tắt khi bắt đầu lượt tăng tốc, chỉ còn Gazebo server orphan;
không có map YAML/PGM được lưu. Log trước đó có hơn 4000 scan tồn đọng/TF lỗi,
được giữ trong before_speed_mapping.log. Phiên mới ban đầu lỗi GLX, log riêng
failed_glx_mapping_simulation.log và failed_glx_mapping.log.

Driver NVIDIA kernel 595.91.07 lệch thư viện 595.99.02: graphics_driver_check.json.
Phương án Mesa chỉ áp dụng cho tiến trình mới, không sửa hệ thống. RViz khởi động,
check_runtime stage mapping đạt scan/odom/clock/TF map->odom: mapping_fast_check.json.
Đo clock/wall RTF ~0.092: simulation_speed.json. Cần reboot để nạp driver mới rồi
kiểm tra lại tốc độ thực; không tuyên bố vận tốc tăng 1.6 lần theo thời gian thực.
Bài vẫn chờ người dùng khảo sát map thật, lưu map và các bước AMCL/Nav2/3 goal.

Test X11 thực với bản nhanh: giữ W 4.007 s, 74 lệnh nonzero ở 0.40 m/s,
đi 0.123 m; thả phím đưa /cmd_vel về zero sau 0.031 s. Test tổng hợp chưa đạt
vì final odom sau 3 s đồng hồ thực vẫn 0.03465 m/s (speed_teleop_check.json);
ở RTF ~0.092, cửa sổ chờ này quá ngắn theo thời gian mô phỏng. Bằng chứng kiểm
tra dừng riêng sau đó ở speed_stop_check.json; không sửa kết quả thất bại gốc.
