# TurtleBot 4 — Nhận diện Biểu cảm Khuôn mặt và Điều khiển Robot (Bài thực hành)

Package **`turtlebot4_facial_emotion_control`** (ROS 2 Humble / DeepFace / OpenCV / PyQt5 Studio / Ignition Gazebo Fortress).  
Hệ thống cho phép TurtleBot 4 nhận diện thời gian thực **06 trạng thái biểu cảm khuôn mặt** của người điều khiển qua webcam máy tính bằng mô hình AI DeepFace (kết hợp OpenCV Haar Cascade và MediaPipe), phân tích xác suất cảm xúc, và tự động điều khiển TurtleBot 4 di chuyển theo các quỹ đạo động học tương ứng qua topic `/cmd_vel`.

---

## 1. Thông tin Đề tài

* **Sinh viên thực hiện:** Lê Anh Tuấn Bằng — MSSV: 23020723
* **Môn học:** Tương tác Người — Robot (HRI) — Đại học Công nghệ (VNU-UET)
* **Giảng viên hướng dẫn:** PGS. TS. Hoàng Văn Xiêm, KS. Nguyễn Quốc Bảo
* **Nhánh Git:** `assignments_5`
* **Package chính:** `turtlebot4_facial_emotion_control`

---

## 2. Bảng Quy tắc Ánh xạ Biểu cảm và Quỹ đạo Điều khiển

| STT | Biểu cảm (Emotion) | Tên tiếng Việt | Hành động của TurtleBot 4 | Vận tốc Dài ($v_x$) | Vận tốc Góc ($\omega_z$) | Đặc tả Quỹ đạo |
| :---: | :--- | :--- | :--- | :---: | :---: | :--- |
| **1** | **Happy** | Vui vẻ | **Di chuyển cong sang trái** | $+0.20\,\text{m/s}$ | $+0.60\,\text{rad/s}$ | Quỹ đạo vòng cung bán kính $R \approx 0.33\,\text{m}$ rẽ trái |
| **2** | **Sad** | Buồn | **Di chuyển cong sang phải** | $+0.20\,\text{m/s}$ | $-0.60\,\text{rad/s}$ | Quỹ đạo vòng cung bán kính $R \approx 0.33\,\text{m}$ rẽ phải |
| **3** | **Neutral** | Bình thường | **Dừng tại chỗ** | $0.00\,\text{m/s}$ | $0.00\,\text{rad/s}$ | Dừng tĩnh ổn định, chờ lệnh tiếp theo |
| **4** | **Angry** | Tức giận | **Lùi nhẹ về phía sau** | $-0.15\,\text{m/s}$ | $0.00\,\text{rad/s}$ | Phản ứng phòng vệ: lùi thẳng an toàn |
| **5** | **Surprise** | Ngạc nhiên | **Xoay một vòng tại chỗ** | $0.00\,\text{m/s}$ | $+1.20\,\text{rad/s}$ | Xoay tròn đều tại chỗ $360^\circ$ quan sát môi trường |
| **6** | **Fear** | Sợ hãi | **Tiến nhẹ về phía trước** | $+0.18\,\text{m/s}$ | $0.00\,\text{rad/s}$ | Tiến thẳng nhẹ nhàng về phía trước |
| --- | *Mất dấu khuôn mặt* | Không có mặt | **Dừng khẩn cấp (Watchdog)** | $0.00\,\text{m/s}$ | $0.00\,\text{rad/s}$ | Tự động phanh sau $1.0\,\text{s}$ bảo vệ an toàn |

---

## 3. Kiến trúc Hệ thống

```text
+-----------------------------------------------------------------------------------+
|                               WEBCAM / CAMERA STREAM                              |
|             Webcam phần cứng /dev/video0 hoặc Bộ sinh khuôn mặt Mock              |
+-----------------------------------------------------------------------------------+
                                       |
                                       v
                     +-----------------------------------+
                     |      EmotionDetector (AI)         |
                     |  OpenCV Haar Cascade + DeepFace   |
                     | (ONNX Weights + Softmax + Filter) |
                     +-----------------------------------+
                                       |
                   +-------------------+-------------------+
                   | /detected_emotion (JSON String)       |
                   v                                       v
+------------------------------------+   +------------------------------------+
|        robot_behavior_node         |   |       emotion_gui (PyQt5)          |
| • Ánh xạ 6 biểu cảm sang quỹ đạo   |   | • Màn hình Live Camera thời gian   |
| • Bộ lọc gia tốc trơn mượt (Ramp)  |   |   thực với bounding box & nhãn     |
| • Watchdog timeout an toàn (1.0s)  |   | • 6 cột đo xác suất biểu cảm       |
+------------------------------------+   | • Đồng hồ đo vận tốc vx, wz        |
                   |                     | • 6 nút bấm thử nghiệm nhanh       |
                   | /cmd_vel (Twist)    +------------------------------------+
                   v                                       ^
+------------------------------------+                     | /emotion/behavior_status
|       TurtleBot 4 Robot /          |---------------------+
|    Mô phỏng Ignition Gazebo        |
+------------------------------------+
```

---

## 4. Danh mục ROS 2 Topics

| Tên Topic | Kiểu Dữ liệu | Node Gửi $\rightarrow$ Nhận | Ý nghĩa Chức năng |
| :--- | :--- | :--- | :--- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | `robot_behavior_node` $\rightarrow$ Robot/Gazebo | Vận tốc dài $v_x$ và vận tốc góc $\omega_z$ điều khiển 2 bánh vi sai. |
| `/detected_emotion` | `std_msgs/msg/String` | `emotion_recognition_node` $\rightarrow$ Behavior, GUI | Chuỗi JSON chứa biểu cảm, độ tin cậy, xác suất 6 cảm xúc, hộp khuôn mặt và FPS. |
| `/emotion/behavior_status` | `std_msgs/msg/String` | `robot_behavior_node` $\rightarrow$ GUI | Trạng thái điều khiển thực tế, vận tốc tức thời, trạng thái Watchdog. |
| `/emotion/manual_override` | `std_msgs/msg/String` | `emotion_gui` $\rightarrow$ `robot_behavior_node` | Lệnh kích hoạt nhanh biểu cảm thủ công để kiểm thử độc lập. |

---

## 5. Hướng dẫn Khởi chạy Thực nghiệm

### Bước 1: Build package trong ROS 2 Workspace
```bash
cd ~/turtlebot4_lab_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select turtlebot4_facial_emotion_control --symlink-install
source install/setup.bash
```

### Bước 2: Chạy Hệ thống Nhận diện Biểu cảm & Giao diện Studio GUI
Mở terminal và chạy lệnh:
```bash
ros2 launch turtlebot4_facial_emotion_control emotion_control.launch.py
```
* Cửa sổ **PyQt5 Emotion Robotics Studio** sẽ xuất hiện, tự động nhận diện webcam, hiển thị khung nhận diện khuôn mặt và các đồng hồ đo xác suất biểu cảm.
* Bạn có thể bấm trực tiếp các nút thử nghiệm nhanh trên giao diện (`Happy`, `Sad`, `Neutral`, `Angry`, `Surprise`, `Fear`) để kiểm tra tức thì các lệnh `/cmd_vel`.

### Bước 3: Chạy cùng Mô phỏng 3D Gazebo TurtleBot 4 (Tùy chọn)
Mở một terminal mới:
```bash
ros2 launch turtlebot4_facial_emotion_control simulation.launch.py
```
TurtleBot 4 sẽ xuất hiện trong sân tập rộng $14\,\text{m} \times 14\,\text{m}$ và di chuyển linh hoạt theo từng biểu cảm khuôn mặt của bạn!

---

## 6. Chạy Kiểm thử Tự động (Unit Tests)
Gói phần mềm tích hợp đầy đủ bộ unit tests kiểm tra toán học và logic ánh xạ:
```bash
pytest src/turtlebot4_facial_emotion_control/test/ -v
```
Toàn bộ các ca kiểm thử bảo đảm kiểm chứng tính chính xác 100% của 6 quỹ đạo và cơ chế an toàn Watchdog.
