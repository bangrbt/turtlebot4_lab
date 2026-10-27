# TurtleBot 4 — điều khiển bằng cử chỉ tay

Bài tập 3, nhánh **`assignments_3`**, package **`turtlebot4_gesture_control`**.
Chạy trên Ubuntu 22.04, ROS 2 Humble và Gazebo Fortress. Webcam laptop được đọc
bằng OpenCV; MediaPipe Gesture Recognizer phân loại tay và ROS 2 điều khiển robot.

**Cách dùng:** giơ 1 hoặc 2 ngón để chọn → giơ ngón cái xác nhận một lần → hạ tay.
Robot tự hoàn thành hành vi rồi dừng. **Vẫy tay để robot chào ngay, không cần xác nhận.**

## 1. Khởi chạy trên máy hiện tại

Nếu đang chạy bản cũ, nhấn **Ctrl+C** ở terminal launch và đợi webcam/Gazebo đóng.
Mở **một terminal mới**, chạy lần lượt:

```bash
cd ~/turtlebot4_lab_ws
source src/turtlebot4_gesture_control/scripts/gesture_env.bash
colcon build --symlink-install --packages-select turtlebot4_gesture_control
source src/turtlebot4_gesture_control/scripts/gesture_env.bash
ros2 launch turtlebot4_gesture_control demo.launch.py
```

Lệnh cuối mở **Gazebo + cửa sổ webcam + hai node điều khiển**. Giữ terminal này
chạy. Chờ dòng `Arena ready` và trạng thái `"ready": true` trong terminal.
Robot đứng yên cho tới khi nhận lệnh tay phù hợp.

World riêng `gesture_arena` là sân 12 × 12 m có tường bao, trống ở giữa.
Startup chỉ dời trạm sạc ra góc, không tự lái robot. Không cần chạy lệnh undock.
Môi trường riêng dùng `ROS_DOMAIN_ID=73`, `IGN_PARTITION=turtlebot4_gesture_73`;
mọi terminal của bài này đều phải source **gesture_env.bash**.

Nếu chọn sai webcam, thêm `camera_index:=1` vào lệnh launch. Không mở hai node
camera hoặc hai phiên `demo.launch.py` cùng lúc.

## 2. Cử chỉ và cách robot phản hồi

Đưa một bàn tay vào khung hình, lòng bàn tay hướng về webcam, đủ sáng.
Giữ cử chỉ chọn/xác nhận khoảng nửa giây tới khi cửa sổ đổi trạng thái.

| Cử chỉ | Kết quả |
|---|---|
| **Giơ 1 ngón trỏ** — `Pointing_Up` | Chọn **tiến–lùi 4 lần**, chờ ngón cái |
| **Giơ 2 ngón** — `Victory` | Chọn **đi một vòng tròn**, chờ ngón cái |
| **Giơ ngón cái** — `Thumb_Up` | Xác nhận lựa chọn; sau khi hiện `RUNNING` có thể hạ tay |
| **Vẫy bàn tay trái–phải** — `Wave` | **Chào ngay** bằng động tác xoay trái–phải nhanh; không cần ngón cái |
| **Xòe bàn tay đứng yên** — `Open_Palm` | Dừng và xóa lựa chọn |
| **Nắm tay** — `Closed_Fist` | **Hủy ngay cả khi đang di chuyển**, xóa hành vi/lựa chọn, về `IDLE` |

**Ví dụ tiến–lùi:** giơ 1 ngón → thấy `SELECTED [back_and_forth]` → giơ ngón cái
→ thấy `RUNNING` → hạ tay. Robot thực hiện **4 lượt tiến và 4 lượt lùi**, rồi
trở về `IDLE`. Không cần giữ ngón cái và không tự lặp lại lượt thứ hai.

**Ví dụ vòng tròn:** giơ 2 ngón → thấy `SELECTED [circle]` → giơ ngón cái → hạ tay.
Robot đi một vòng rồi dừng.

**Ví dụ chào:** vẫy tay ngang rõ một nhịp qua–lại; khi hiện `RUNNING [greeting]`,
hạ tay để robot hoàn thành. Wave cần vài frame chuyển động để phân biệt với
Open Palm đứng yên; sau khi nhận ra Wave, robot bắt đầu ngay, không chờ giữ
cử chỉ hay xác nhận lần nữa. Có thể vẫy bằng cổ tay; bộ lọc theo dõi cả đầu ngón
tay, chịu được vài frame nhãn AI chập chờn. Nếu giữ bàn tay xòe đứng yên sau đó,
robot hiểu là dừng. Khi đang chào, bàn tay đang chuyển động không tự hủy chào;
Open Palm đứng yên khoảng 0.25 s được phân biệt thành lệnh dừng.

### Khi robot đang chạy

- **Hạ tay / AI tạm không nhận ra tay:** robot tiếp tục hoàn thành lượt đã xác nhận.
- **Giơ cử chỉ chọn khác:** robot dừng lượt cũ, chọn hành vi mới và chờ Thumb Up.
  Giơ lại cử chỉ của chính hành vi đang chạy không khởi động lại nó.
- **Vẫy tay:** chuyển sang chào ngay. Các frame của cùng một lần vẫy không làm
  robot khởi động lại động tác chào liên tục.
- **Open Palm / Closed Fist:** dừng/hủy. Muốn chạy tiếp phải chọn và xác nhận lại;
  riêng Wave luôn dùng quy tắc chào ngay.
- **Giữ nguyên Thumb Up sau khi xong:** robot đứng yên; không tự lặp hành vi.

Không dùng bàn phím để lái trong demo. Nhấn `q`, `Esc` hoặc đóng cửa sổ webcam
chỉ để kết thúc nhận diện; robot dừng do mất nguồn điều khiển. `Ctrl+C` trong
terminal launch kết thúc cả phiên.

## 3. Tốc độ và giới hạn chuyển động

Cấu hình tại [config/behavior.yaml](config/behavior.yaml):

| Tham số | Giá trị mặc định |
|---|---:|
| Tốc độ tiến/lùi tối đa | **0.30 m/s** |
| Tốc độ xoay tối đa | **1.10 rad/s** |
| Quãng đường mục tiêu mỗi đoạn tiến/lùi | 0.12 m |
| Số chu kỳ tiến–lùi | 4 |
| Bán kính vòng tròn | 0.85 m |
| Góc chào trái/phải so với hướng ban đầu | ±0.55 rad |
| Nghỉ giữa các đoạn | 0.10 s |

Đây là giới hạn lệnh, tốc độ thực còn phụ thuộc tăng/giảm tốc và khoảng cách tới
đích. Robot giảm tốc gần đích; chuyển động kết thúc theo **odometry**, không chỉ
canh thời gian. Động tác chào dùng điều khiển góc đáp ứng nhanh và trở về hướng đầu.
Quãng tiến–lùi được giữ ngắn để hoàn thành đủ 4 chu kỳ trong giới hạn lùi gốc
của Create 3; dung sai vị trí 0.02 m. Không tăng đoạn lùi dài tùy ý vì bộ điều
khiển gốc có thể chặn trước khi tới đích. Sau khi sửa YAML, dừng rồi chạy lại launch.

**Hạ tay khác với mất webcam:** frame mới không có tay vẫn chứng minh camera
đang hoạt động. Nếu webcam/node ngắt, ảnh quá cũ/đóng băng, payload hỏng hoặc
session camera đổi, robot dừng và xóa lệnh; không tự chạy tiếp khi kết nối trở lại.
Timeout nhận frame mới là **0.80 s**. Lỗi camera được báo rõ có thể gây dừng sớm hơn.

LiDAR, odometry, simulation clock, hazard và arena heartbeat vẫn được giám sát.
Trước khi chạy cần khoảng trống ít nhất 2.2 m quanh robot; khi chạy, khoảng cách
vật cản dưới 0.45 m hoặc cảm biến không hợp lệ/cũ sẽ dừng hành vi. Bộ giám sát
chạy bằng steady time nên vẫn dừng khi Gazebo bị pause. Giữ các giới hạn an toàn
gốc của Create 3; không cấu hình `safety_override` để tăng tốc.

## 4. Chạy riêng từng phần và xử lý lỗi

**Chỉ xem webcam, chưa chạy AI:**

```bash
source ~/turtlebot4_lab_ws/src/turtlebot4_gesture_control/scripts/gesture_env.bash
ros2 launch turtlebot4_gesture_control webcam.launch.py
```

**Thử nhận diện nhưng khóa chuyển động:**

```bash
ros2 launch turtlebot4_gesture_control control.launch.py enable_motion:=false
```

**Mở Gazebo và điều khiển ở hai terminal riêng:** mỗi terminal source môi trường,
sau đó terminal A chạy simulation, terminal B chạy control. Không mở thêm demo.

```bash
# Terminal A
ros2 launch turtlebot4_gesture_control simulation.launch.py

# Terminal B
ros2 launch turtlebot4_gesture_control control.launch.py
```

| Hiện tượng | Cách xử lý |
|---|---|
| `Package not found` | Source môi trường, build package, source lại rồi launch |
| Webcam không mở | Đóng ứng dụng đang dùng camera; thử `camera_index:=1` |
| `Detected` có nhãn nhưng `accepted: None` | Giữ tay rõ, hướng lòng bàn tay vào camera; nhãn chưa đủ confidence/hình học |
| Chỉ hiện `SELECTED` | Giơ ngón cái tới khi thấy `RUNNING`, sau đó hạ tay |
| Đang chào lại dừng | Tránh giữ bàn tay xòe đứng yên; sau khi Wave đã nhận, hạ tay |
| `ready: false` | Xem `safety_reason` qua topic dưới đây; chờ cảm biến/arena sẵn sàng |
| `conflicting_controller` | Dừng teleop/Nav2; chỉ chạy một node hành vi |
| `camera_signal_lost` / `camera_source_failed` | Kiểm tra cửa sổ webcam và node nhận diện, sau đó chọn/xác nhận lại |

Quan sát trạng thái ở terminal khác đã source môi trường:

```bash
ros2 topic echo /gesture/behavior_status --qos-durability transient_local
ros2 topic echo /detected_gesture
```

Không thể cam kết AI đúng ở mọi ánh sáng/góc tay. Bộ lọc vẫn kiểm tra landmark,
confidence và độ ổn định trước khi chọn/xác nhận. Ngưỡng hiện tại giữ theo cấu hình
webcam đã dùng: chọn 0.55, xác nhận 0.60, 3 frame và 0.20 s; controller lọc thêm
cử chỉ chọn/xác nhận. **Cancel không chờ bộ lọc ổn định và không cần xác nhận:**
Closed Fist từ 0.50 được đưa thẳng tới lệnh hủy ở cả hai node, không có ngưỡng
0.65 khác ở phía điều khiển. Khi AI mất nhãn nhưng landmark cho thấy bốn ngón
khép và ngón cái gập sát lòng bàn tay, hai frame rõ liên tiếp cũng phát Cancel.
Điều kiện ngón cái giúp phân biệt với Thumb Up; fallback này không giả lập
confidence AI. Cửa sổ ghi `[landmarks]` nếu dùng đường nhận diện này.

Wave dùng chuyển động ngang của cả đầu ngón và tâm bàn tay qua thời gian,
không đòi mọi frame đều có nhãn AI Open Palm. Một nhịp qua–lại rõ được chấp nhận
sau ít nhất 4 frame / 0.18 s; không có nghĩa mọi webcam đều đạt độ trễ 0.18 s.
Mặc định chỉ theo dõi một tay (`max_hands: 1`) và chỉ giữ message mới nhất để
giảm chờ xử lý. Không có lớp Wave pretrained trong model Google.

## 5. Cài đặt lần đầu trên máy khác

Cần ROS Humble và các package simulator TurtleBot 4:

```bash
sudo apt-get install ros-humble-turtlebot4-simulator python3-venv \
  python3-colcon-common-extensions python3-pytest
```

Sau khi clone nhánh `assignments_3` vào `~/turtlebot4_lab_ws`:

```bash
cd ~/turtlebot4_lab_ws
bash src/turtlebot4_gesture_control/scripts/setup_environment.bash
source src/turtlebot4_gesture_control/scripts/gesture_env.bash
colcon build --symlink-install --packages-select turtlebot4_gesture_control
source src/turtlebot4_gesture_control/scripts/gesture_env.bash
ros2 launch turtlebot4_gesture_control demo.launch.py
```

Script tạo `.venv_gesture`, cài dependency pin version và tải model Tasks Google
về `.cache_gesture`, kiểm tra SHA256 trong manifest. Không tải model giữa phiên.
Venv, model, log, video và báo cáo ở local, không đưa vào Git.

## 6. Luồng chương trình và kiểm thử

```text
Webcam → OpenCV → MediaPipe + landmark + temporal Wave
       → /detected_gesture (String JSON: nhãn, confidence, nguồn frame)
       → robot_behavior_node (chọn/xác nhận hoặc Wave chạy ngay)
       → quỹ đạo odometry + kiểm tra cảm biến → /cmd_vel → TurtleBot 4
```

| File | Trách nhiệm |
|---|---|
| `gesture_recognition_node.py`, `camera.py` | Đọc frame mới nhất, inference, cửa sổ webcam, nguồn ảnh |
| `gestures.py` | Kiểm tra landmark, lọc tĩnh, Wave động và schema JSON |
| `behavior.py` | Chọn → xác nhận một lần → thực hiện một lượt; Stop/Cancel/watchdog |
| `motion.py` | Tiến–lùi, vòng tròn, chào theo odometry |
| `robot_behavior_node.py`, `safety.py` | ROS topics, vận tốc, trạng thái và kiểm tra cảm biến |
| `prepare_simulation.py` | Dời dock, phát `/gesture/arena_ready` |
| `launch/`, `config/`, `worlds/` | Khởi chạy, tham số và sân mô phỏng |
| `test/`, `scripts/validate_behavior.py` | Kiểm thử logic/ROS và hành vi trên simulator |

Các topic chính: `/detected_gesture` và `/gesture/behavior_status`
(`std_msgs/msg/String`, payload JSON), `/cmd_vel` (`geometry_msgs/msg/Twist`),
`/odom`, `/scan`, `/clock`, `/hazard_detection`, `/gesture/arena_ready`.
Observation chứa session, sequence và thời điểm chụp để từ chối ảnh cũ/message lặp.
Không publish chuỗi trần `Victory` vào topic để thay thế schema.

Bản hiện tại đã đạt **110 test** và **16 kiểm tra tích hợp** trên Gazebo bằng
cử chỉ tổng hợp; chào, đủ 4 chu kỳ tiến–lùi và vòng tròn đều hoàn thành khi hạ tay.
Các test mới tái hiện Fist score 0.50–0.64, Cancel bằng landmark khi nhãn None,
Wave bằng cổ tay, mất nhãn từng frame và frame Open Palm không được tự ngắt chào.
Đây là kiểm chứng điều khiển, không phải phép đo độ chính xác từ tay người.
Webcam thực cũng đã nhận được Wave và Closed Fist trong lượt kiểm tra có người dùng;
không coi số frame nhận được là accuracy hoặc ma trận nhầm lẫn.

Chạy kiểm thử trong domain riêng sau khi source môi trường:

```bash
ROS_DOMAIN_ID=75 python -m pytest src/turtlebot4_gesture_control/test -q
```

Kiểm chứng Gazebo bằng **cử chỉ tổng hợp**, không phải demo nhận diện tay người:
chạy simulation + node hành vi, chưa mở camera trong cùng domain, rồi:

```bash
ros2 run turtlebot4_gesture_control robot_behavior_node --ros-args \
  --params-file src/turtlebot4_gesture_control/config/behavior.yaml
# Ở terminal khác, cùng môi trường, khi simulator và node hành vi đã sẵn sàng:
python src/turtlebot4_gesture_control/scripts/validate_behavior.py \
  --output log/gesture_behavior_validation.json
```

Kiểm tra perception với MediaPipe thật trên ảnh mẫu xoay và video chuyển động
nhân tạo (cần ảnh mẫu cache từ `validate_model_samples.py`):

```bash
python src/turtlebot4_gesture_control/scripts/validate_priority_perception.py \
  --output log/gesture_priority_perception.json
```

Script này kiểm tra Fist ở các góc xoay, không nhầm Thumb Up thành hủy, đường
landmark khi mất nhãn và phát hiện Wave trên chuỗi ảnh tổng hợp. Không thay thế
kiểm thử webcam/tay người.

Script hành vi kiểm tra hạ tay vẫn chạy, Wave không cần xác nhận, đổi lựa chọn, Stop/Cancel,
mất camera và hoàn thành từng quỹ đạo đúng một lượt. Kết quả sinh vào `log/`, không
đưa artifact kiểm thử lên Git. `scripts/check_recognition.py` chỉ đọc topic để
đếm nhãn cho từng cử chỉ do người dùng thực sự giữ trước webcam.

Nguồn: [MediaPipe Gesture Recognizer](https://developers.google.com/edge/mediapipe/solutions/vision/gesture_recognizer/python),
[TurtleBot 4 simulator](https://turtlebot.github.io/turtlebot4-user-manual/software/turtlebot4_simulator.html).
