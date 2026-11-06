# TurtleBot 4 — Phân vùng bản đồ và điều hướng robot bằng giọng nói (Lab 07)

Package **`turtlebot4_voice_navigation`**, ROS 2 Humble / Gazebo Fortress / Nav2.  
Hệ thống cho phép TurtleBot 4 tự động phân vùng bản đồ 2D Occupancy Grid, gắn nhãn ngữ nghĩa các phòng, nhận dạng câu lệnh giọng nói (tiếng Việt & tiếng Anh), và điều hướng tự hành qua Nav2 với khả năng xử lý lộ trình đa điểm và từ chối khu vực không tồn tại.

---

## 1. Kiến trúc hệ thống

```
+-----------------------------------------------------------------------------------+
|                              NGƯỜI DÙNG / GIỌNG NÓI                               |
|        Microphone laptop / File âm thanh mẫu / Terminal nhập lệnh văn bản         |
+-----------------------------------------------------------------------------------+
                                       |
                                       v
                     +-----------------------------------+
                     |       voice_recognizer_node       |
                     |  Offline STT: Vosk / Whisper / VAD |
                     +-----------------------------------+
                                       | /voice/recognized_text (String)
                                       v
+------------------------+   +-----------------------------------+   +-----------------------+
|      BẢN ĐỒ GỐC        |   |     voice_navigation_manager      |   |  zone_visualizer_node |
| map.pgm + map.yaml     |-->|   NLU Intent Parser & Routing     |<--|  3D Markers on /map   |
| (OpenCV Segmentation)  |   | (Single / From-To / Multi-Waypt)  |   | (Labels, Boundaries)  |
+------------------------+   +-----------------------------------+   +-----------------------+
            |                                  |
            v                                  | Nav2 NavigateToPose / NavigateThroughPoses
+------------------------+                     v
|      zones.yaml        |       +---------------------------+
| Tọa độ, Đa giác phòng, |       |   Nav2 Navigation Stack   |
| Aliases tiếng Việt/Anh |       | (Planner, Costmap, AMCL)  |
+------------------------+       +---------------------------+
                                               |
                                               v /cmd_vel
                                 +---------------------------+
                                 |  TurtleBot 4 Simulation   |
                                 |   Gazebo Smart Campus     |
                                 +---------------------------+
```

### Các Module & Node cốt lõi:
1. **`map_segmentation.py`**:
   - Sử dụng OpenCV xử lý ảnh Occupancy Grid (`map.pgm`).
   - Ứng dụng lọc hình thái học (Morphological opening), biến đổi khoảng cách (Distance Transform), và thuật toán **Watershed** để phân chia không gian tự do thành các phòng riêng biệt qua các nút thắt cửa ra vào (doorways).
   - Chuyển đổi chính xác tọa độ pixel $(u, v) \leftrightarrow$ tọa độ thực $(x, y)$ trong hệ trục `/map` của ROS 2.
   - Tìm điểm cực đại khoảng cách tới tường (max clearance) làm điểm đích an toàn nhất cho robot.
   - Xuất bản đồ trực quan `segmented_map.png` và cấu hình ngữ nghĩa `zones.yaml`.
2. **`semantic_map.py`**:
   - Quản lý metadata các khu vực: Tên, ID (A, B, C, D), mô tả, tọa độ đích $(x, y, \theta)$, ranh giới đa giác (polygon), màu sắc hiển thị.
   - Hỗ trợ tra cứu từ đồng nghĩa (aliases) đa dạng: tiếng Việt có dấu, không dấu, viết tắt, và tiếng Anh.
   - Thuật toán **Ray-Casting Point-in-Polygon (PIP)**: Kiểm tra vị trí hiện tại của robot đang nằm trong phòng nào trong thời gian thực.
3. **`nlu_parser.py`**:
   - Phân tích ngữ pháp và trích xuất ý định (Intent Extraction) từ câu nói:
     - **Điều hướng trực tiếp (NAVIGATE_TO)**: *"Đi đến khu vực A."*, *"Đi đến phòng thí nghiệm."*
     - **Điều hướng có gốc và đích (NAVIGATE_FROM_TO)**: *"Di chuyển từ khu vực A đến khu vực B."* (Nếu robot chưa ở A $\rightarrow$ đi qua A trước rồi mới đến B).
     - **Lộ trình đa điểm (NAVIGATE_WAYPOINTS - Mở rộng)**: *"Đi từ phòng A qua hành lang D rồi đến phòng B."*
     - **Dừng khẩn cấp (STOP)**: *"Dừng lại"*, *"Hủy lệnh"*.
     - **Hỏi vị trí (QUERY_LOCATION)**: *"Robot đang ở đâu?"*, *"Vị trí hiện tại"*.
     - **Bắt lỗi phòng không tồn tại (INVALID_ZONE)**: Nhận diện và từ chối các phòng lạ (*"Đi đến nhà ăn"*, *"Khu vực Z"*), xuất thông báo giải thích rõ ràng và giữ robot đứng yên an toàn.
4. **`voice_recognizer_node`**:
   - Nhận dạng giọng nói ngoại tuyến (offline) bằng **Vosk** hoặc **Whisper**.
   - Hỗ trợ đa kênh đầu vào: Microphone laptop trực tiếp, file âm thanh `.wav` mẫu, hoặc kênh văn bản `/voice/command_text`.
5. **`voice_navigation_manager`**:
   - Đóng vai trò Action Client kết nối với Nav2 (`NavigateToPose` và `NavigateThroughPoses`).
   - Theo dõi trạng thái robot qua `/tf` (`map` $\to$ `base_link`) và `/amcl_pose`.
   - Điều phối thứ tự di chuyển và xuất trạng thái `/voice/navigation_status`.
6. **`zone_visualizer_node`**:
   - Xuất `MarkerArray` lên topic `/semantic_zones_markers`.
   - Hiển thị nhãn 3D chữ nổi trên không gian phòng trong RViz2, vẽ đường viền phòng đổi màu, đánh dấu tâm đích và vẽ đường lộ trình dự kiến màu vàng.

---

## 2. Bản đồ & Khu vực ngữ nghĩa (Smart Campus)

Môi trường mô phỏng Gazebo Fortress diện tích compact **12.0m × 9.0m**, bao gồm 4 khu vực chức năng được bố trí vật cản (bàn ghế, bục giảng, tủ tài liệu, cây cảnh, cột trang trí) với hành lang thông thoáng:

| Khu vực | Mã ID | Tên chức năng | Tọa độ đích $(x, y, \theta)$ | Đặc điểm nhận diện & Vật cản |
|---|:---:|---|:---:|---|
| **Khu vực A** | `A` | **Phòng học (Classroom)** | $(-3.0\text{m}, 2.0\text{m}, 0.0\text{ rad})$ | Bục giảng, bàn ghế sinh viên, cột chịu lực |
| **Khu vực B** | `B` | **Phòng thí nghiệm (Laboratory)** | $(2.8\text{m}, 2.0\text{m}, 1.57\text{ rad})$ | Bàn thí nghiệm, tủ điều khiển thiết bị |
| **Khu vực C** | `C` | **Văn phòng (Office)** | $(2.8\text{m}, -2.0\text{m}, -1.57\text{ rad})$ | Bàn làm việc, tủ sách, chậu cây cảnh |
| **Khu vực D** | `D` | **Hành lang & Sảnh (Corridor & Lobby)** | $(-3.0\text{m}, -1.8\text{m}, 0.0\text{ rad})$ | Ghế chờ sảnh, sảnh thông tầng, chậu cây |

* Vị trí xuất phát ban đầu của robot: $(x = 0.0, y = 0.0, \text{yaw} = 0.0)$ tại ngã tư trung tâm hành lang.

---

## 3. Hướng dẫn chạy chương trình

Hệ thống sử dụng môi trường độc lập: **`ROS_DOMAIN_ID=77`** và Ignition partition `turtlebot4_voice_77`.  
Mỗi terminal mới luôn chạy lệnh `source src/turtlebot4_voice_navigation/scripts/voice_env.bash`.

### Cách 1: Chạy theo từng Terminal (Khuyến nghị cho kiểm tra & chấm demo)

* **Terminal 1: Khởi động Mô phỏng Gazebo (Smart Campus World)**
  ```bash
  cd ~/turtlebot4_lab_ws
  source src/turtlebot4_voice_navigation/scripts/voice_env.bash
  ros2 launch turtlebot4_voice_navigation simulation.launch.py
  ```

* **Terminal 2: Khởi động Bản đồ, Định vị AMCL, Nav2 & RViz2**
  ```bash
  cd ~/turtlebot4_lab_ws
  source src/turtlebot4_voice_navigation/scripts/voice_env.bash
  ros2 launch turtlebot4_voice_navigation navigation.launch.py
  ```
  *(Cửa sổ RViz2 sẽ hiện lên với đầy đủ Bản đồ, Laser Scan, Robot, Costmap và nhãn chữ nổi 3D của 4 khu vực).*

* **Terminal 3: Khởi động Hệ thống Nhận dạng Giọng nói & Điều hướng**
  ```bash
  cd ~/turtlebot4_lab_ws
  source src/turtlebot4_voice_navigation/scripts/voice_env.bash
  ros2 launch turtlebot4_voice_navigation voice_nav.launch.py
  ```

* **Terminal 4: Bảng điều khiển lệnh tương tác (Voice Terminal CLI)**
  ```bash
  cd ~/turtlebot4_lab_ws
  source src/turtlebot4_voice_navigation/scripts/voice_env.bash
  python3 src/turtlebot4_voice_navigation/scripts/voice_terminal.py
  ```

---

### Cách 2: Khởi động All-in-One (1 lệnh duy nhất)
```bash
cd ~/turtlebot4_lab_ws
source src/turtlebot4_voice_navigation/scripts/voice_env.bash
ros2 launch turtlebot4_voice_navigation system.launch.py
```

---

## 4. Các câu lệnh thử nghiệm (Test Cases)

Bạn có thể nói trực tiếp vào microphone hoặc gõ vào `voice_terminal.py` (hoặc publish topic `/voice/command_text`):

1. **Test Case 1: Lệnh điều hướng trực tiếp 1 điểm**
   ```text
   Nhập lệnh > Đi đến khu vực A.
   ```
   $\rightarrow$ Robot nhận diện đích là `Khu vực A (Phòng học)`, lập kế hoạch đường đi tránh vật cản qua cửa phòng A và di chuyển đến đích.

2. **Test Case 2: Lệnh sử dụng tên ngữ nghĩa**
   ```text
   Nhập lệnh > Đi đến phòng thí nghiệm.
   ```
   $\rightarrow$ Robot di chuyển vào `Khu vực B (Phòng thí nghiệm)`.

3. **Test Case 3: Lệnh di chuyển từ gốc đến đích ("Từ A đến B")**
   ```text
   Nhập lệnh > Di chuyển từ khu vực A đến khu vực B.
   ```
   $\rightarrow$ Hệ thống kiểm tra: Nếu robot chưa ở phòng A, robot sẽ tự động lái đến phòng A trước, dừng xác nhận 2 giây, sau đó tiếp tục hành trình đến phòng B!

4. **Test Case 4: Lệnh đa điểm qua trạm trung gian (Gợi ý mở rộng)**
   ```text
   Nhập lệnh > Đi từ phòng A qua hành lang D rồi đến phòng B.
   ```
   $\rightarrow$ RViz vẽ đường nối lộ trình màu vàng [A $\rightarrow$ D $\rightarrow$ B]. Robot lần lượt di chuyển tuần tự qua 3 phòng.

5. **Test Case 5: Bắt lỗi khu vực không tồn tại**
   ```text
   Nhập lệnh > Đi đến nhà ăn
   ```
   $\rightarrow$ Phản hồi: `[REJECTED] Khu vực 'nhà ăn' không tồn tại trên bản đồ. Các khu vực hợp lệ: A (Khu vực A), B (Khu vực B), C (Khu vực C), D (Khu vực D).` Robot giữ nguyên trạng thái đứng yên!

6. **Test Case 6: Kiểm tra vị trí hiện tại**
   ```text
   Nhập lệnh > Robot đang ở đâu?
   ```
   $\rightarrow$ Trả lời chính xác phòng mà robot đang đứng dựa trên thuật toán Point-in-Polygon.

7. **Test Case 7: Dừng khẩn cấp**
   ```text
   Nhập lệnh > Dừng lại
   ```
   $\rightarrow$ Lập tức hủy action goal của Nav2, phanh dừng robot về 0 m/s.

---

## 5. Kiểm thử độc lập & Unit Tests

Chạy toàn bộ 77 kiểm thử tự động (không cần mở Gazebo):
```bash
PYTHONPATH=src/turtlebot4_voice_navigation .venv_gesture/bin/python -m pytest src/turtlebot4_voice_navigation/test -v
```

Kiểm tra phân vùng lại bản đồ bất kỳ:
```bash
python3 src/turtlebot4_voice_navigation/scripts/segment_map.py --map src/turtlebot4_voice_navigation/maps/map.yaml
```
