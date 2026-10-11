# Bài thực hành 04 — TurtleBot 4 Simulation, SLAM, Localization và Nav2

Dự án độc lập trên Ubuntu 22.04 + ROS 2 Humble + Ignition Gazebo Fortress.
World cố định: **maze**, model **standard**, namespace rỗng, spawn `(0, 0, 0)`.
Không phụ thuộc workspace UR3 hoặc workspace robot khác. Không dùng launch Jazzy.

## Trạng thái kết quả

Xem `../../evidence/status.json` và `../../evidence/RESULTS.md` trong repository workspace.
Source được chuẩn bị trước; chỉ log/ảnh thực tế mới xác nhận một bước đã đạt.
`maps/` không chứa bản đồ mẫu. Map, AMCL và ba goal chưa được xem là hoàn thành
chỉ vì launch đã được viết hoặc build thành công.

## Cấu trúc

```text
turtlebot4_lab_ws/                 # gốc repository Git
├── .gitignore
├── evidence/                      # log kiểm tra, JSON/JSONL, ảnh thực tế
└── src/turtlebot4_lab/
    ├── package.xml
    ├── CMakeLists.txt
    ├── launch/                    # simulation, mapping, navigation, view
    ├── config/                    # odom.rviz và map.rviz
    ├── maps/                      # map.yaml/map.pgm sau khi mapping thật
    ├── scripts/                   # kiểm tra, movement probe, lưu map, ghi goal
    ├── docs/                      # kịch bản video, nội dung báo cáo, đối chiếu
    └── README.md
```

## Dependency và cài đặt (chạy một lần)

Hệ điều hành và ROS Desktop đã có, không cài lại. Kiểm tra trước:

```bash
cat /etc/os-release
df -h "$HOME"
source /opt/ros/humble/setup.bash
ign gazebo --versions
ros2 pkg prefix turtlebot4_ignition_bringup
ros2 pkg prefix turtlebot4_navigation
xdpyinfo -display "$DISPLAY" | head
```

Chỉ cài phần còn thiếu:

```bash
sudo apt-get install ros-humble-turtlebot4-simulator ros-humble-turtlebot4-navigation
```

Simulator bổ sung mô hình TurtleBot/Create 3, plugin Ignition và bridge ROS/Gazebo;
navigation bổ sung cấu hình/launch chính thức SLAM, AMCL, Nav2; dependency apt tự giải quyết.
Máy thực hành đã có Nav2, SLAM Toolbox, RViz2, teleop, colcon, PyYAML và Pillow.
Nếu máy tái lập thiếu công cụ: cài `python3-colcon-common-extensions`,
`ros-humble-teleop-twist-keyboard`, `python3-yaml`, `python3-pil` tương ứng.
Không nâng cấp toàn bộ hệ thống, không build lại simulator/controller.

## Môi trường sạch cho MỖI terminal

Shell máy có thể tự source workspace khác trong `.bashrc`. Mở shell sạch trước:

```bash
env -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH -u CMAKE_PREFIX_PATH \
    -u PYTHONPATH -u LD_LIBRARY_PATH -u ROS_PACKAGE_PATH \
    -u IGN_GAZEBO_RESOURCE_PATH -u IGN_GUI_PLUGIN_PATH -u GAZEBO_MODEL_PATH \
    bash --noprofile --norc
source /opt/ros/humble/setup.bash
export TB4_LAB_WS="$HOME/turtlebot4_lab_ws"
export ROS_DOMAIN_ID=64
export ROS_LOCALHOST_ONLY=1
export IGN_PARTITION=turtlebot4_lab_64
export ROS_LOG_DIR="$TB4_LAB_WS/log/ros"
mkdir -p "$ROS_LOG_DIR" "$TB4_LAB_WS/evidence"
# Sau khi build mới chạy dòng sau:
source "$TB4_LAB_WS/install/setup.bash"
```

Domain 64 tách bài thực hành khỏi các robot đang chạy ở domain khác. MỌI terminal
của bài phải có cùng domain và IGN_PARTITION; không đổi domain khi các node đang hoạt động.
`AMENT_PREFIX_PATH` sau source chỉ nên chứa package bài và `/opt/ros/humble`.

Sau khi build, có thể thay các dòng source/export trên bằng helper trong repository:

```bash
source "$HOME/turtlebot4_lab_ws/lab_env.bash"
```

Helper suy ra workspace từ vị trí file, không hardcode tên tài khoản; chỉ làm sạch
môi trường trong terminal hiện tại.

## Build (chạy một lần và sau khi thêm map vào source)

Trong terminal sạch ở trên, bỏ dòng source install ở lần đầu:

```bash
cd "$TB4_LAB_WS"
colcon build --packages-select turtlebot4_lab --symlink-install
source "$TB4_LAB_WS/install/setup.bash"
ros2 pkg prefix turtlebot4_lab
ros2 launch turtlebot4_lab simulation.launch.py --show-args
ros2 launch turtlebot4_navigation slam.launch.py --show-args
ros2 launch turtlebot4_navigation localization.launch.py --show-args
ros2 launch turtlebot4_navigation nav2.launch.py --show-args
```

Wrapper chỉ include launch Humble chính thức. `simulation` luôn tắt SLAM/AMCL/Nav2;
`mapping` chỉ mở SLAM; `navigation` chỉ mở map server + AMCL + Nav2 tùy `nav2`.
Không chạy hai wrapper mapping/navigation đồng thời. `view` chỉ mở RViz, không tạo
robot_state_publisher thứ hai. Có thể dùng `rviz:=false` để tránh hai cửa sổ RViz.
Không để simulator tự dùng map mẫu `warehouse.yaml`: wrapper navigation yêu cầu map thật.

## Giai đoạn 1 — simulation và kiểm tra

**Terminal A** (giữ chạy):

```bash
ros2 launch turtlebot4_lab simulation.launch.py rviz:=true \
  > >(tee "$TB4_LAB_WS/evidence/simulation.log") 2>&1
```

Trong Gazebo: chờ robot/world hiện; bấm Play nếu đang pause. Launch chính thức
có `-r` nhưng vẫn phải xác minh `/clock` tăng. RViz có Fixed Frame `odom`, Robot
và LiDAR bật. Pan/zoom để thấy robot. Map chưa có ở bước này là bình thường.

**Terminal B** (kiểm tra, chạy xong trả về prompt):

```bash
ros2 run turtlebot4_lab check_runtime.py --stage simulation \
  --output "$TB4_LAB_WS/evidence/simulation_check.json"
ros2 topic info /cmd_vel --verbose --no-daemon --spin-time 3
ros2 topic info /diffdrive_controller/cmd_vel_unstamped --verbose --no-daemon --spin-time 3
ros2 node info /motion_control --no-daemon --spin-time 3
ros2 topic echo /scan sensor_msgs/msg/LaserScan --once --qos-reliability best_effort
ros2 topic echo /odom nav_msgs/msg/Odometry --once --qos-reliability best_effort
ros2 topic echo /clock --once
ros2 run tf2_ros tf2_echo odom base_link
# Ctrl+C dừng tf2_echo (chỉ lệnh kiểm tra).
```

Đường vận tốc phải được xác nhận trực tiếp qua endpoint. Bridge chính thức đưa
lệnh GUI Gazebo vào `/cmd_vel`; Create 3 `motion_control` nhận `/cmd_vel` và xử lý
an toàn, xuất lệnh sang `/diffdrive_controller/cmd_vel_unstamped`, bridge đưa sang
`/model/turtlebot4/cmd_vel` trong Ignition. Không bypass safety bằng cách phát thẳng
vào controller. Script motion probe từ chối nếu không có subscriber `motion_control`.

Robot mặc định spawn tại dock. Nếu dock status đang docked, kiểm tra và undock
bằng action chính thức (chỉ chạy trước khi thử movement):

```bash
ros2 topic echo /dock_status --once
ros2 action list -t
ros2 action send_goal /undock irobot_create_msgs/action/Undock '{}' --feedback
```

Chờ action hoàn tất, xác nhận phía trước thoáng, rồi thử tốc độ 0.08 m/s trong
2 giây và dừng. Script kiểm tra scan, odom và clock, luôn phát zero khi kết thúc:

```bash
ros2 run turtlebot4_lab motion_probe.py --output "$TB4_LAB_WS/evidence/motion_probe.json"
```

Chỉ ghi đạt khi odom cho thấy dịch chuyển và vận tốc cuối gần 0. Chụp ảnh thật
Gazebo và RViz vào `evidence/` (`01_gazebo.png`, `02_rviz_lidar.png`).
**Ctrl+C Terminal A** khi kết thúc kiểm tra để chuyển sang phiên mapping sạch.

### Robot bò chậm sau cập nhật NVIDIA

Cập nhật sau reboot 11/10/2026: NVIDIA 595.99.02 đã hoạt động; phiên GPU
`evidence/post_reboot_20261011_230015/` đạt kiểm tra mapping và hold teleop.
Tỷ lệ thời gian mô phỏng đo được 0.679 (so với 0.092 ở Mesa). Gazebo/SLAM/teleop
đang mở để khảo sát map; chưa lưu YAML/PGM, chưa chạy AMCL/Nav2.

Nếu `nvidia-smi` báo `Driver/library version mismatch`, so sánh với
`cat /proc/driver/nvidia/version`. Phiên kiểm tra tăng tốc ghi nhận kernel driver
595.91.07 và thư viện 595.99.02; Gazebo/RViz không tạo được GLX context. Khởi động
lại máy để nạp driver đã cập nhật, rồi kiểm tra `nvidia-smi` và chạy lại các terminal
simulation, mapping, hold teleop ở dưới. Nếu vẫn lệch sau reboot, kiểm tra lại cài
đặt driver trước khi tiếp tục.

Mesa software chỉ là phương án tạm theo từng tiến trình:
`LIBGL_ALWAYS_SOFTWARE=1 __GLX_VENDOR_LIBRARY_NAME=mesa`; không thêm các biến này
vào `lab_env.bash`. Phép đo `/clock` trong `evidence/simulation_speed.json` chỉ đạt
0.092 lần thời gian thực, nên vận tốc lệnh 0.40 m/s vẫn rất chậm theo đồng hồ thực.
Tăng vận tốc vượt giới hạn controller 0.46 m/s không giải quyết được nguyên nhân này.
Bản đồ khảo sát chưa được lưu thành YAML/PGM; sau reboot cần chạy SLAM và khảo sát lại.

## Giai đoạn 2 — SLAM + teleoperation

Mapping mặc định dùng launch SLAM chính thức với `sync:=false` và
`config/slam_mapping.yaml` dựa trên cấu hình Humble: throttle_scans=4,
minimum_travel_distance/heading=0.10, map_update_interval=1.0 s. Giữ độ phân giải
0.05 m và loop closure. Điều chỉnh này xử lý lỗi queue >4000 scans/TF extrapolation
đã xuất hiện khi sync xử lý không kịp. Có thể chọn `sync:=true` nếu cần, nhưng phải
quan sát log tránh queue tăng trở lại. 0.40 m/s dành cho hành lang thoáng; giảm bằng
spinbox khi vào góc hẹp hoặc map/scan bắt đầu lệch.


**Terminal A** khởi động lại simulator cùng world, tắt RViz ở wrapper này:

```bash
ros2 launch turtlebot4_lab simulation.launch.py rviz:=false \
  > >(tee "$TB4_LAB_WS/evidence/mapping_simulation.log") 2>&1
```

**Terminal B** (giữ chạy SLAM + RViz, không AMCL):

```bash
ros2 launch turtlebot4_lab mapping.launch.py \
  > >(tee "$TB4_LAB_WS/evidence/mapping.log") 2>&1
```

**Terminal C**, kiểm tra/undock nếu cần, sau đó teleop tương tác:

```bash
ros2 action send_goal /undock irobot_create_msgs/action/Undock '{}' --feedback
ros2 run turtlebot4_lab teleop_hold.py --speed 0.40 --turn 0.8 \
  --log "$TB4_LAB_WS/evidence/teleop_hold.jsonl"
```

Click cửa sổ **TurtleBot 4 - HOLD TELEOP**, giữ `W/A/S/D` hoặc mũi tên để chạy,
thả phím để dừng; không cần Enter. `W/↑/i` tiến, `S/↓/,` lùi, `A/←/j` quay trái,
`D/→/l` quay phải; giữ W+A hoặc W+D để đi cong. `Space`, `k`, `Esc` hoặc nút
DỪNG ROBOT dừng ngay. Chuyển focus sang cửa sổ khác cũng dừng. Lệnh phát 20 Hz,
trạng thái phím được đọc trực tiếp từ X11, không qua bộ gõ Bamboo/IME, mỗi 50 ms;
release được xử lý sau tối đa khoảng 50 ms (trong điều kiện GUI không bị treo).
Tốc độ mặc định 0.40 m/s, quay 0.8 rad/s; spinbox giới hạn 0.46 m/s và 1 rad/s.
Script chỉ phát `/cmd_vel` vào Create 3 motion_control, không thay controller/driver;
không cho chạy khi Nav2 đang hoạt động hoặc simulation clock đứng. Dùng Tkinter
và Xlib (`python3-tk`, `python3-xlib`, đã có trên máy thực hành). Đóng cửa sổ để dừng teleop trước Nav2.

Bản `teleop_twist_keyboard` 2.4.1 chính thức vẫn dùng được làm lựa chọn terminal,
nhưng không hỗ trợ `repeat_rate`/`key_timeout`; cửa sổ giữ/thả phím ở trên là cách
điều khiển mặc định của bài này. Chỉ mở **một** teleop mỗi lần.
Khảo sát phần lớn hành lang có thể đi, quay chậm, tránh va tường và quay quá gấp.
Đi vòng trở lại khu vực đã quét để có loop closure. Theo dõi LaserScan khớp tường,
map tăng vùng đã biết; nếu tường đôi/lệch, dừng và kiểm tra TF/time, không tiếp tục
lưu một map lỗi. Không coi diện tích chưa quét là free space.

**Terminal D** (trong lúc SLAM chạy):

```bash
ros2 run turtlebot4_lab check_runtime.py --stage mapping \
  --output "$TB4_LAB_WS/evidence/mapping_check.json"
ros2 topic info /map --verbose
ros2 run tf2_ros tf2_echo map odom
# Ctrl+C dừng tf2_echo.
```

Khi map đủ tốt: nhấn `k`, đóng cửa sổ HOLD TELEOP, chụp `03_mapping.png`, rồi **Terminal D**:

```bash
ros2 run turtlebot4_lab save_map.py --output "$TB4_LAB_WS/src/turtlebot4_lab/maps/map"
ros2 run turtlebot4_lab validate_map.py "$TB4_LAB_WS/src/turtlebot4_lab/maps/map.yaml" \
  | tee "$TB4_LAB_WS/evidence/map_validation.json"
ls -lh "$TB4_LAB_WS/src/turtlebot4_lab/maps/map.yaml" \
       "$TB4_LAB_WS/src/turtlebot4_lab/maps/map.pgm"
```

Script chỉ lưu `/map` trực tiếp bằng `nav2_map_server map_saver_cli`, không tạo map
hay thay thế khảo sát teleop; từ chối ghi đè. Nếu đã có output cũ, sao lưu cặp map
hoặc chọn prefix khác. Cặp ảnh/YAML giữ `image: map.pgm` để clone sang máy khác.
Chụp `04_saved_map.png`. Build lại để map được install vào share package.

**Dừng trước localization:** đóng cửa sổ HOLD TELEOP hoặc Ctrl+C Terminal C, Terminal B (SLAM/RViz),
Terminal A (simulation). Trong D xác minh SLAM đã biến mất:

```bash
ros2 node list --no-daemon
# Không được còn /slam_toolbox trước khi mở AMCL.
ros2 run turtlebot4_lab cleanup_simulation.py --stop
ros2 node list --no-daemon
# Không được còn controller_manager/diffdrive_controller của phiên cũ.
```

Ignition có thể để lại server/GUI sau Ctrl+C. `cleanup_simulation.py --stop` chỉ
dừng process Ignition có partition `turtlebot4_lab_64` và ROS domain 64, không dừng
robot khác. Không dùng `pkill` toàn máy. Nếu node còn, xác định terminal/process thuộc bài
và dừng đúng tiến trình đó. Đợi các node của phiên cũ thoát hẳn.

## Giai đoạn 3 — restart, load map và localization

**Terminal A** khởi động lại robot trong đúng `maze`:

```bash
ros2 launch turtlebot4_lab simulation.launch.py rviz:=false \
  > >(tee "$TB4_LAB_WS/evidence/localization_simulation.log") 2>&1
```

**Terminal B** mở AMCL/map/RViz, trước hết chưa chạy Nav2:

```bash
ros2 launch turtlebot4_lab navigation.launch.py nav2:=false \
  map:="$TB4_LAB_WS/src/turtlebot4_lab/maps/map.yaml" \
  > >(tee "$TB4_LAB_WS/evidence/localization.log") 2>&1
```

Gazebo Play; RViz Fixed Frame `map`. Chọn **2D Pose Estimate**, click vị trí robot
ban đầu trên map đã lưu và kéo mũi tên hướng đúng theo Gazebo. Không mặc định map
origin trùng tọa độ Gazebo; dùng hình dạng tường và pose khi mapping để đối chiếu.
Undock bằng action trước khi đặt initial pose nếu còn ở dock: action không chỉ
lùi khỏi dock mà còn quay gần 180°. Phải đặt hướng mũi tên theo robot SAU undock,
không giả định yaw vẫn bằng 0. Phiên thực tế đã có goal ABORTED do đặt sai hướng;
log giữ nguyên trong evidence/navigation_20261011_232532. Trong simulation có thể
đối chiếu `ros2 topic echo /sim_ground_truth_pose --once --qos-reliability best_effort`
để biết pose thật; ground truth chỉ hỗ trợ kiểm tra/initial pose, không điều khiển goal.
Kiểm tra scan khớp tường/map, pose và covariance AMCL trước navigation.

**Terminal C**:

```bash
ros2 run turtlebot4_lab check_runtime.py --stage localization \
  --output "$TB4_LAB_WS/evidence/localization_check.json"
ros2 topic echo /amcl_pose --once --qos-reliability reliable --qos-durability transient_local
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
ros2 run tf2_ros tf2_echo map base_link
# Ctrl+C dừng tf2_echo.
```

`map → odom → base_link` phải có và scan khớp map trước khi gửi goal. AMCL không
nhất thiết phát pose liên tục khi robot đứng yên. Publisher /amcl_pose dùng
Transient Local; check_runtime.py và config RViz dùng đúng durability để nhận pose
cuối khi subscribe muộn. Có thể gọi `ros2 service call /request_nomotion_update std_srvs/srv/Empty '{}'`
để AMCL xử lý scan mới lúc robot đứng yên. Chụp `05_localization.png`.

## Giai đoạn 4 — Nav2 và ba goal qua RViz

Giữ simulation A và AMCL/RViz B. **Terminal C** chỉ mở navigation chính thức
(đã có AMCL, không chạy wrapper navigation lần hai):

```bash
ros2 launch turtlebot4_navigation nav2.launch.py use_sim_time:=true \
  > >(tee "$TB4_LAB_WS/evidence/nav2.log") 2>&1
```

**Terminal D** kiểm tra, sau đó giữ recorder chạy **trước goal thứ nhất**:

```bash
ros2 run turtlebot4_lab check_runtime.py --stage navigation \
  --output "$TB4_LAB_WS/evidence/navigation_check.json"
ros2 action info /navigate_to_pose
ros2 run turtlebot4_lab record_goals.py --output "$TB4_LAB_WS/evidence/goals.jsonl"
```

Có thể thay B+C bằng **một** wrapper `navigation.launch.py nav2:=true` nếu chưa
mở AMCL; tuyệt đối không thêm C khi wrapper này đã mở Nav2.

Tắt teleop hoàn toàn trước goal. Trong RViz chọn **Nav2 Goal**, click và kéo
hướng tại một vùng free đã quét, cách tường đủ xa. Thực hiện lần lượt **ba vị trí
khác nhau**, đợi goal trước kết thúc rồi mới gửi goal tiếp theo. Mỗi goal cần:
đích x/y/yaw (ghi từ RViz/log hoặc ảnh), action result `SUCCEEDED` (status 4), robot
thực sự tới đích, quan sát đường/costmap và tránh vật cản. Chụp `06_goal1.png`,
`07_goal2.png`, `08_goal3.png`, ghi bảng thực tế ở docs. **Không teleop sau khi gửi
goal để đưa robot tới đích.** Recorder không gửi goal/velocity; status 5 canceled,
6 aborted phải ghi đúng. GoalTool Humble dùng GoalUpdater của panel và có thể không phát `/goal_pose`.
Recorder cũng ghi endpoint `/plan`; endpoint chỉ là bằng chứng bổ sung từ planner,
không phải request action gốc. Đối chiếu UUID, dòng `Begin navigating` trong log,
ảnh và endpoint khi chỉ một goal đang chạy; khi đánh giá. Không suy ra tránh vật cản chỉ từ SUCCEEDED.

Goal lỗi: giữ log, xem `ros2 lifecycle get /planner_server`, `/controller_server`,
TF, initial pose, map, costmap và `/scan`; đọc `nav2.log` để phân biệt lỗi lập
đường, controller, thời gian hoặc recovery. Cancel goal đang lỗi bằng RViz, sửa
nguyên nhân rồi thử lại, ghi cả lần lỗi. Sau demo Ctrl+C D, C, B, A theo thứ tự.

## Lỗi thường gặp

| Triệu chứng | Kiểm tra và xử lý |
|---|---|
| Package not found | source đúng Humble và install bài; xác minh apt đã cài TurtleBot 4 |
| Không thấy node/topic | mọi terminal dùng domain 64, localhost 1 và partition bài; bỏ overlay robot khác; thử CLI `--no-daemon --spin-time 3` vì daemon có thể cache graph cũ |
| `/clock` không tăng | bấm Play, đọc log Gazebo, kiểm tra clock bridge |
| Gazebo tải mãi | maze không cần Fuel; kiểm tra robot spawn và OpenGL; đọc simulation.log |
| GUI lỗi display/OpenGL | kiểm tra DISPLAY/XAUTHORITY, driver; thử `LIBGL_ALWAYS_SOFTWARE=1` cho riêng terminal nếu cần |
| Scan không có | xem lidar_bridge, sensor Ignition, QoS Best Effort, tên world maze |
| Robot không đi | dock status, undock action, safety hazards, endpoint `/cmd_vel`, odom; không bypass safety |
| RViz robot mất | RobotModel bật, description Transient Local, TF odom/base/sensor |
| Fixed Frame map lỗi ở bước 1 | dùng odom.rviz, chưa có SLAM/AMCL là bình thường |
| Map chồng tường | dừng, kiểm tra simulation time và TF, giảm tốc, khảo sát lại nếu cần |
| Saver timeout | SLAM còn chạy, `/map` có data, QoS Transient Local, clock không pause |
| Map YAML thiếu ảnh | ảnh tương đối cùng thư mục; chạy validate_map.py |
| AMCL chưa có map→odom | đặt initial pose, scan/time đúng, không chạy SLAM đồng thời |
| Nav2 inactive | đọc lỗi cấu hình/TF; lifecycle chỉ active khi khởi động thành công |
| Goal ngoài map/đụng costmap | chọn free đã quét, initial pose khớp, inflation/costmap đúng |

## Báo cáo, video và repository

`docs/report_content.md` là nội dung thực tế/chỗ trống có nhãn; sẽ dàn LaTeX sau
khi nhận mẫu báo cáo và phần “Nội dung cần hiểu”. Không tạo ảnh/goal giả hay PDF
khẳng định kết quả chưa chạy. `docs/demo_checklist.md` có kịch bản quay.
Log/ảnh/map có thể commit; build/install/log/video/rosbag lớn bị gitignore.
Repository Git local có thể tạo tại gốc workspace; không tự nhận đã push/upload.
Điền link GitHub/video sau khi thực sự tạo và kiểm tra quyền truy cập.

## Tài liệu chính thức đã đối chiếu cho Humble

- https://github.com/turtlebot/turtlebot4_simulator/tree/humble
- https://turtlebot.github.io/turtlebot4-user-manual/software/turtlebot4_simulator.html
- https://turtlebot.github.io/turtlebot4-user-manual/tutorials/generate_map.html
- https://turtlebot.github.io/turtlebot4-user-manual/tutorials/navigation.html

Ưu tiên tên và arguments từ `ros2 launch ... --show-args` và source package đã
cài. Tài liệu có tab Galactic/Humble/Jazzy; phần Humble dùng
`turtlebot4_ignition_bringup/turtlebot4_ignition.launch.py`, SLAM dùng
`turtlebot4_navigation/slam.launch.py` (`sync:=true`).
