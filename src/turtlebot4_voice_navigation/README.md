# TurtleBot 4 — Phân vùng bản đồ và điều hướng robot bằng giọng nói (Bài thực hành 7)

Package **`turtlebot4_voice_navigation`**, ROS 2 Humble / Gazebo Fortress / Nav2.  
Hệ thống cho phép TurtleBot 4 tự động phân vùng bản đồ 2D Occupancy Grid, gắn nhãn ngữ nghĩa **05 khu vực chức năng**, nhận dạng câu lệnh giọng nói (tiếng Việt & tiếng Anh qua microphone hoặc terminal), và tự hành né tránh vật cản 3D thực tế bằng Nav2 với khả năng xử lý lộ trình đơn trạm, đa điểm và từ chối khu vực không tồn tại.

---

## 1. Kiến trúc hệ thống

```
+-----------------------------------------------------------------------------------+
|                              NGƯỜI DÙNG / GIỌNG NÓI                               |
|       Microphone laptop / voice_terminal.py / Kênh văn bản / File mẫu .wav        |
+-----------------------------------------------------------------------------------+
                                       |
                                       v
                     +-----------------------------------+
                     |       voice_recognizer_node       |
                     |  Vosk (Offline) / Google / VAD    |
                     +-----------------------------------+
                                       | /voice/recognized_text (String)
                                       v
+------------------------+   +-----------------------------------+   +-----------------------+
|      BẢN ĐỒ GỐC        |   |     voice_navigation_manager      |   |  zone_visualizer_node |
| map.pgm + map.yaml     |-->|   NLU Intent Parser & Routing     |<--|  3D Markers on /map   |
| (Tường & Vật cản 3D)   |   | (Single / From-To / Multi-Waypt)  |   | (Labels, Boundaries)  |
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
                                 | (Bàn ghế, Cây, Người 3D)  |
                                 +---------------------------+
```

### Các Module & Node cốt lõi:
1. **`generate_environment.py` & `map_segmentation.py`**:
   - Thiết lập bản đồ 14.0m × 11.0m, phân chia thành **05 khu vực chức năng** rõ rệt nối với trục hành lang sảnh chính rộng 1.6m và các cửa ra vào 1.2m thông thoáng.
   - Bố trí các vật thể 3D thực tế (người, ghế, xe đạp, cây xanh, bình chữa cháy, balo), không sử dụng các khối lập phương đơn sơ.
   - Tính toán mặt cắt va chạm tại độ cao tia quét laser LiDAR ($z = 0.18$m) để vẽ trực tiếp lên `map.pgm` và xuất `segmented_map.png` có in chữ tên phòng to rõ, màu sắc trực quan.
2. **`semantic_map.py`**:
   - Quản lý metadata 5 khu vực: Tên, ID (A, B, C, D, E), mô tả, tọa độ đích $(x, y, \theta)$, ranh giới đa giác, màu sắc hiển thị.
   - Cơ chế tự động mở rộng tra cứu từ đồng nghĩa (aliases): hỗ trợ mọi biến thể tiếng Việt có dấu, không dấu, viết tắt, và tiếng Anh.
   - Thuật toán **Ray-Casting Point-in-Polygon (PIP)**: Xác định tức thì vị trí robot đang ở phòng nào trong thời gian thực.
3. **`nlu_parser.py`**:
   - Phân tích ngữ pháp và trích xuất ý định (Intent Extraction) từ câu nói:
     - **Điều hướng trực tiếp (NAVIGATE_TO)**: *"Đi đến khu vực A."*, *"Đi đến phòng thí nghiệm."*, *"Đi đến thư viện."*
     - **Điều hướng có điểm xuất phát (NAVIGATE_FROM_TO)**: *"Di chuyển từ khu vực A đến khu vực B."* (Nếu robot chưa ở A $\rightarrow$ đi qua A trước rồi mới đến B).
     - **Lộ trình đa điểm (NAVIGATE_WAYPOINTS)**: *"Đi từ phòng học qua hành lang rồi đến phòng thí nghiệm."*
     - **Dừng khẩn cấp (STOP)**: *"Dừng lại"*, *"Hủy lệnh"*.
     - **Hỏi vị trí (QUERY_LOCATION)**: *"Robot đang ở đâu?"*, *"Vị trí hiện tại"*.
     - **Bắt lỗi phòng không tồn tại (INVALID_ZONE)**: Bắt các phòng lạ (*"Đi đến phòng tổng thống"*, *"Đi đến nhà ăn"*), từ chối di chuyển và in danh sách phòng hợp lệ.
4. **`voice_recognizer_node`**:
   - Nhận dạng giọng nói linh hoạt: Hỗ trợ offline **Vosk** (`vosk-model-small-vn`), online Google SpeechRecognition, và đo ngưỡng năng lượng RMS âm thanh microphone.
   - Tự động quét và chọn thiết bị micro mặc định, hỗ trợ kèm kênh tiêm lệnh văn bản `/voice/command_text`.
5. **`prepare_voice_simulation`**:
   - Tự động di chuyển `standard_dock` ra ngoài sau khi khởi tạo để robot được nhả sạc (undock) ngay lập tức, bánh xe không bị khóa và Nav2 tự do điều khiển `/cmd_vel`.
6. **`zone_visualizer_node`**:
   - Xuất `MarkerArray` lên topic `/semantic_zones_markers`.
   - Hiển thị nhãn chữ nổi 3D đa sắc cho cả 5 phòng, vẽ khung viền ranh giới, đánh dấu tâm đích an toàn trong RViz2.

---

## 2. Bản đồ & 05 Khu vực ngữ nghĩa (Smart Campus)

Môi trường mô phỏng Gazebo Fortress diện tích **14.0m × 11.0m**, bao gồm 5 khu vực chức năng với các vật thể 3D thực tế:

| Khu vực | Mã ID | Tên chức năng | Tọa độ đích $(x, y, \theta)$ | Vật thể 3D đặc trưng |
|---|:---:|---|:---:|---|
| **Khu A** | `A` | **Phòng học (Classroom)** | $(-3.2\text{m}, 2.6\text{m}, 0.0\text{ rad})$ | Bàn giáo viên, 2 ghế học viên, người đứng, balo |
| **Khu B** | `B` | **Phòng thí nghiệm (Laboratory)** | $(3.2\text{m}, 2.6\text{m}, 0.0\text{ rad})$ | Bàn thí nghiệm, ghế xoay, nhà nghiên cứu, bình cứu hỏa |
| **Khu C** | `C` | **Văn phòng (Office)** | $(3.2\text{m}, -2.6\text{m}, 0.0\text{ rad})$ | Bàn làm việc, ghế văn phòng, nhân viên, balo |
| **Khu D** | `D` | **Thư viện (Library)** | $(-3.2\text{m}, -2.6\text{m}, 0.0\text{ rad})$ | Cây xanh nội thất (Oak tree), bàn ghế đọc, độc giả |
| **Khu E** | `E` | **Hành lang & Sảnh (Corridor & Lobby)** | $(0.0\text{m}, 0.0\text{m}, 0.0\text{ rad})$ | Xe đạp dựng tường, bình cứu hỏa, điểm xuất phát TB4 |

* **Vị trí xuất phát của robot:** $(x = 0.0, y = 0.0, \text{yaw} = 0.0)$ tại trung tâm sảnh Zone E, hoàn toàn không bị vướng vật cản.

---

## 3. Hướng dẫn chạy chương trình chi tiết

Môi trường chạy sử dụng Domain ID độc lập: **`ROS_DOMAIN_ID=77`** và Partition `turtlebot4_voice_77`.  
Mỗi terminal mới luôn chạy lệnh:
```bash
source src/turtlebot4_voice_navigation/scripts/voice_env.bash
```

### Bước 0: Chuẩn bị & Kiểm tra Microphone (Chỉ cần chạy 1 lần đầu)

1. **Cài đặt thư viện giọng nói & mô hình tiếng Việt Vosk (chạy khi có mạng):**
   ```bash
   cd ~/turtlebot4_lab_ws
   bash src/turtlebot4_voice_navigation/scripts/setup_environment.bash
   ```

2. **Kiểm tra phần cứng Microphone (3 giây):**
   ```bash
   cd ~/turtlebot4_lab_ws
   source src/turtlebot4_voice_navigation/scripts/voice_env.bash
   python3 src/turtlebot4_voice_navigation/scripts/test_microphone.py
   ```
   *(Nói vào mic một câu; màn hình sẽ hiển thị thanh âm lượng RMS và xác nhận `[OK] MICROPHONE HOẠT ĐỘNG TỐT!`)*

---

### Quy trình chạy Demo (Khuyến nghị mở 4 Terminal)

#### Terminal 1: Khởi động Mô phỏng Gazebo (Thế giới Smart Campus 5 phòng & Robot)
```bash
cd ~/turtlebot4_lab_ws
source src/turtlebot4_voice_navigation/scripts/voice_env.bash
ros2 launch turtlebot4_voice_navigation simulation.launch.py
```
*(Chờ Gazebo hiển thị và xuất dòng thông báo: `Arena Ready! TurtleBot 4 is undocked at (0, 0) in Zone E (Lobby). Ready for voice navigation!`)*

#### Terminal 2: Khởi động Định vị AMCL, Ngăn xếp Nav2 & RViz2
```bash
cd ~/turtlebot4_lab_ws
source src/turtlebot4_voice_navigation/scripts/voice_env.bash
ros2 launch turtlebot4_voice_navigation navigation.launch.py
```
*(Cửa sổ RViz2 sẽ hiện lên với bản đồ 5 khu vực, chùm hạt AMCL và các nhãn chữ 3D trên từng phòng).*

#### Terminal 3: Khởi động Bộ nhận diện giọng nói & Điều phối hành vi
```bash
cd ~/turtlebot4_lab_ws
source src/turtlebot4_voice_navigation/scripts/voice_env.bash
ros2 launch turtlebot4_voice_navigation voice_nav.launch.py
```
*(Node voice recognizer sẽ bật micro lắng nghe và kết nối Action Client tới Nav2).*

#### Terminal 4: Bật Giao diện tương tác Voice Terminal (Nói vào mic hoặc chọn số)
```bash
cd ~/turtlebot4_lab_ws
source src/turtlebot4_voice_navigation/scripts/voice_env.bash
python3 src/turtlebot4_voice_navigation/scripts/voice_terminal.py
```

Tại Terminal 4, bạn có thể:
- **Nói trực tiếp vào microphone laptop** các câu lệnh tiếng Việt.
- **Hoặc chọn nhanh phím số từ 1 đến 9**:
  - `1`: "Đi đến khu vực A" (Phòng học)
  - `2`: "Đi đến phòng thí nghiệm" (Khu vực B)
  - `3`: "Di chuyển từ khu vực A đến khu vực B"
  - `4`: "Đi đến văn phòng" (Khu vực C)
  - `5`: "Đi đến thư viện" (Khu vực D)
  - `6`: "Đi từ phòng học qua hành lang rồi đến phòng thí nghiệm" (Lộ trình đa trạm)
  - `7`: "Đi đến phòng tổng thống" (Kiểm tra bắt lỗi phòng không tồn tại)
  - `8`: "Robot đang ở đâu?" (Truy vấn vị trí hiện tại)
  - `9`: "Dừng lại" (Dừng khẩn cấp)

---

## 4. Chạy kiểm thử tự động (Unit Tests)

Toàn bộ 90 ca kiểm thử đã được tự động hóa bằng `pytest`:
```bash
cd ~/turtlebot4_lab_ws
source src/turtlebot4_voice_navigation/scripts/voice_env.bash
python3 -m pytest src/turtlebot4_voice_navigation/test/ -v
```
Kết quả: **`90 passed in 1.76s`** (100% Passed).

---

## 5. Tài liệu nộp bài
- **Bản đồ gốc & bản đồ phân vùng:** `maps/map.pgm`, `maps/map.yaml`, `maps/segmented_map.png`
- **Tệp cấu hình 5 khu vực:** `config/zones.yaml`, `config/zones.json`
- **Mô hình 3D thực tế:** `models/chair`, `models/person`, `models/bicycle`, `models/oak_tree`, `models/fire_hydrant`, `models/backpack`
- **Báo cáo PDF hoàn chỉnh (2 trang chuẩn academic):** `docs/report.pdf`
- **File âm thanh kiểm thử mẫu:** `sample_audio/01_di_den_khu_vuc_a.wav` ... `sample_audio/06_khu_vuc_khong_ton_tai.wav`
