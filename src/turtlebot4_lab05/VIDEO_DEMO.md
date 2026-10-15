# Kịch bản quay video Bài 05 — chạy đúng thứ tự

Mục tiêu video: thấy **nạp map cũ + AMCL trên RViz**, **code tự chạy 3 goal liên
tiếp**, **cửa sổ camera YOLO cập nhật trong lúc robot di chuyển**. Quay màn hình
thật; không cần quay quá trình cài thư viện/build. Không cần upload video trong
các lệnh dưới đây. Khi quay xong mới tự đưa link Drive vào README.

## 0. Chuẩn bị trước khi bấm quay

Nếu đang có phiên simulator cũ: Ctrl+C terminal navigation trước, rồi YOLO,
localization/Nav2, simulator; giữ lại source/map. Không mở thêm một bộ chạy trùng.
Khi các launch cũ đã dừng có thể dọn Ignition còn sót bằng:

```bash
cd ~/turtlebot4_lab_ws
source lab_env.bash
ros2 run turtlebot4_lab cleanup_simulation.py --stop
```

Build/tải tài nguyên **một lần, trước khi quay**, nhất là asset STOP và weights YOLO11s mới:

```bash
cd ~/turtlebot4_lab_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select turtlebot4_lab05
source lab_env.bash
source src/turtlebot4_lab05/scripts/lab05_env.bash
python src/turtlebot4_lab05/scripts/download_resources.py --cache "$TB4_LAB05_CACHE"
python src/turtlebot4_lab05/scripts/check_scene.py
```

`check_scene.py` phải passed=true: kiểm tra cả thân vật thể với tường thật,
map/unknown, khoảng cách giữa các vật thể. Không chỉ kiểm tra tâm. Bố trí hiện tại
có 2 người, 1 ngựa, 2 ghế, 1 biển STOP. Bản đồ gốc và ba waypoint giữ nguyên.

Mở các terminal A, B, C, D, E, F. **Đầu mỗi terminal**, chạy ba dòng sau:

```bash
cd ~/turtlebot4_lab_ws
source lab_env.bash
source src/turtlebot4_lab05/scripts/lab05_env.bash
```

Terminal A/B/D/E giữ chạy. C dùng lệnh kiểm tra một lần. F chạy chương trình
navigation. Có thể thu nhỏ terminal không dùng, nhưng không đóng tiến trình.

## 1. Terminal A — bật simulator, chưa bấm quay

```bash
ros2 launch turtlebot4_lab simulation.launch.py model:=standard rviz:=false
```

Chờ Gazebo hiện robot/world maze, simulation ở Play. Không dùng world khác,
không chạy SLAM. Không cần vội gửi lệnh khi Gazebo còn đang tải model.

## 2. Terminal C — chuẩn bị vật thể và undock, vẫn chưa bấm quay

Chạy từng lệnh; chờ lệnh trước hoàn tất:

```bash
ros2 run turtlebot4_lab05 spawn_demo_objects
ros2 action send_goal /undock irobot_create_msgs/action/Undock '{}' --feedback
```

Chờ undock **SUCCEEDED**. Quan sát Gazebo: tám vật thể đứng riêng, ba ghế không
chạm tường. Chỉ spawn một lần cho world mới. Nếu đang cập nhật một scene đã có
vật thể, dùng lệnh dưới **thay cho** lệnh spawn thường:

```bash
ros2 run turtlebot4_lab05 spawn_demo_objects --ros-args -p replace_existing:=true
```

Lệnh replace gỡ cả `lab05_backpack` cũ, rồi thay các model `lab05_` trong objects.yaml, giữ nguyên robot,
tường và nền. Không chạy trong lúc đang thực hiện mission.

## 3. BẮT ĐẦU QUAY, rồi Terminal B — nạp map cũ

**Bấm Record tại đây, trước khi chạy lệnh terminal B** để video thấy quá trình
nạp map. Chọn vùng quay chứa RViz, cửa sổ camera và terminal F; có thể quay cả
màn hình rồi sắp xếp các cửa sổ cạnh nhau. Giữ quay liên tục tới hết Goal 3.

```bash
ros2 launch turtlebot4_lab05 bringup.launch.py start_yolo:=false start_navigation:=false
```

Đợi RViz hiện map. Chỉ rõ bản đồ từ tuần trước đang được map_server nạp; Fixed
Frame là map, AMCL chạy localization, không phải SLAM quét map mới. Chưa chạy F.
Có thể chuyển qua Gazebo vài giây để giới thiệu sáu vật thể rồi trở lại RViz.

## 4. Terminal C/RViz — thực hiện localization, tiếp tục quay

**Cách thao tác trên RViz:** chọn **2D Pose Estimate**, click vị trí robot trên
bản đồ, kéo mũi tên đúng hướng robot rồi thả. Với simulator mới và đã undock,
vị trí đo trước đây gần x=-0.35, y=0; robot thường hướng sang trái (yaw gần pi).
Đây chỉ là gợi ý nhận biết vị trí spawn của đúng maze; phải đối chiếu robot thật
trong Gazebo và chùm scan, không mặc định đúng cho map/phiên khác.

Nếu cần cách đặt chính xác, dùng helper này **thay cho thao tác click**:

```bash
ros2 run turtlebot4_lab05 initialize_pose --ros-args \
  -p use_sim_time:=true -p source:=sim_ground_truth
```

Helper chỉ dùng ground truth mô phỏng để khởi tạo AMCL **trước mission**, có
kiểm tra map/alignment đã xác thực. Navigation sau đó vẫn dùng AMCL/Nav2; helper
không phát vận tốc, không điều khiển robot đến các goal. Khi thuyết minh, nói rõ
nếu đã chọn cách helper này.

Sau cách đã chọn, chạy:

```bash
ros2 run turtlebot4_lab check_runtime.py --stage navigation
```

Phải passed=true; trên RViz Navigation/Localization active, scan đỏ khớp các
tường lân cận, pose/hạt hội tụ quanh vị trí robot. Giữ hình 5–10 giây để nhìn rõ.
Nếu scan lệch tường, sửa Initial Pose; chưa chạy chương trình navigation.

## 5. Terminal D — mở camera YOLO, tiếp tục quay

```bash
ros2 launch turtlebot4_lab05 yolo.launch.py device:=0 show_window:=true
```

Chờ cửa sổ **TurtleBot 4 Lab05 - YOLO RGB** hiện ảnh, lần đầu warm-up có thể lâu
hơn các ảnh sau. Đặt cửa sổ camera cạnh RViz, không che toàn bộ map. Trong C:

```bash
ros2 topic echo /yolo/status --once --qos-durability transient_local --full-length
```

Cần state=ready và actual_model_device=cuda:0. Khi có vật thể trong tầm nhìn,
YOLO hiện bbox, nhãn, confidence. Khi nhìn tường hoặc không thấy vật thể,
không có bbox là bình thường; ảnh vẫn phải cập nhật. Bản hiện tại dùng YOLO11s với 8 vật thể rải đều (xe đạp, cây sồi thu nhỏ, cột cứu hỏa đỏ, ngựa, 3 ghế, người). Đã kích hoạt bộ lọc hình học + ngưỡng `person_confidence: 0.55` để triệt tiêu việc nhận nhầm tường/góc thành người, và hạ `plant_confidence: 0.15` để nhận diện cây ổn định.
Không nhấn q/Esc hoặc đóng cửa sổ camera trong đoạn mission cần quay.

## 6. Terminal E — ghi bằng chứng tự động (khuyên dùng)

Chạy **trước F**, vẫn đang quay:

```bash
python src/turtlebot4_lab05/scripts/record_demo.py \
  --output-directory "$TB4_LAB_WS/log/video_$(date +%Y%m%d_%H%M%S)"
```

Observer chỉ đọc topic, không lái robot. Nó tự lưu ảnh thật, các kết quả action,
model device, inference khi odometry có chuyển động và hazard. Tự kết thúc vài
giây sau mission_completed. Log/ảnh/video không tự upload.

## 7. Terminal F — chạy code tự đi 3 goal, giữ quay liên tục

```bash
export LAB05_VIDEO_LOG="$TB4_LAB_WS/log/navigation_video_$(date +%Y%m%d_%H%M%S).jsonl"
ros2 run turtlebot4_lab05 sequential_navigation --ros-args \
  --params-file src/turtlebot4_lab05/config/navigation.yaml \
  -p localization_confirmed:=true \
  -p log_file:="$LAB05_VIDEO_LOG"
```

Giữ RViz + camera + terminal F nhìn thấy được. Node đọc YAML và tự gửi action,
không cần bấm Enter thêm. Quay đủ các mốc:

1. readiness_passed, goal_sent/goal_accepted Goal_1; robot đi trên map, camera cập nhật.
2. Goal_1 goal_result status=4, dừng 8 giây; code tự gửi Goal_2.
3. Goal_2 status=4, dừng 8 giây; code tự gửi Goal_3.
4. Goal_3 status=4, hết dwell, mission_completed với completed=3; robot đứng lại.

Trong mission không teleop, không click Nav2 Goal, không đặt lại Initial Pose,
không đổi scene. Có thể thay đổi góc nhìn/cửa sổ GUI để quay rõ, không gửi lệnh
chuyển động khác. Cần dừng khẩn thì Ctrl+C ở F để node cancel; lượt chưa đủ ba
SUCCEEDED không tính là video hoàn thành, hãy xử lý nguyên nhân rồi quay lại.

## 8. Terminal F — hiện bảng kết quả thật, rồi mới dừng quay

Sau khi chương trình F trả về prompt, dùng **cùng terminal F** để biến
LAB05_VIDEO_LOG vẫn còn giá trị:

```bash
python3 - "$LAB05_VIDEO_LOG" <<'PY'
import json, sys
records = [json.loads(line) for line in open(sys.argv[1])]
for r in records:
    if r['event'] == 'goal_result':
        status = 'SUCCEEDED' if r['status'] == 4 else 'status=' + str(r['status'])
        print(r['name'], status, r['goal_uuid'])
completed = next((r for r in reversed(records) if r['event'] == 'mission_completed'), None)
print('mission_completed:', completed is not None,
      'completed:', completed['completed'] if completed else 0)
PY
```

Quay thêm 5–10 giây thấy ba SUCCEEDED, completed=3, RViz/camera vẫn mở rồi **Stop
Recording**. Không chỉ quay bảng cuối: phần robot thực sự đi qua cả ba goal và
camera chạy đồng thời phải có trong video. Có thể cho xem thêm person, horse,
chair/stop sign đã nhận diện trong các khung hình thật; không ghép ảnh tĩnh thành
camera giả. Thời gian chạy phụ thuộc tải máy, không cắt lượt chỉ vì chạy chậm.

Sau khi đã dừng quay: Ctrl+C D (YOLO), B (Nav2), A (simulator); E tự xong hoặc
Ctrl+C khi cần. Không dừng Nav2/simulator khi F còn active. Lưu video, kiểm tra
âm thanh/hình ảnh và tự bổ sung link Drive vào README.
