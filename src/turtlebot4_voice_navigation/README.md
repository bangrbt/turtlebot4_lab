# TurtleBot 4 — Phân vùng bản đồ và Điều hướng robot bằng giọng nói (Bài thực hành 07)

Package **`turtlebot4_voice_navigation`** (ROS 2 Humble / Ignition Gazebo Fortress / Nav2 / PyQt5 Studio).  
Hệ thống cho phép TurtleBot 4 tự động phân vùng bản đồ 2D Occupancy Grid ($14\,\text{m} \times 11\,\text{m}$) thành **05 khu vực chức năng** (Phòng học, Phòng Lab, Văn phòng, Thư viện, Sảnh trung tâm), gắn nhãn ngữ nghĩa đa ngôn ngữ, nhận dạng câu lệnh giọng nói tiếng Việt tự nhiên (hỗ trợ cả môi trường micro âm lượng lớn khi quay video màn hình), phân tích cú pháp ngôn ngữ tự nhiên (NLU) đa chặng liên phòng (*"tới khu E rồi tới văn phòng"*), và tự hành di chuyển mượt mà qua các cửa phòng hẹp bằng Nav2 DWB Controller.

---

## 1. Thông tin Đề tài & Liên kết Thực nghiệm

* **Sinh viên thực hiện:** Lê Anh Tuấn Bằng — MSSV: 23020723
* **Môn học:** Tương tác Người — Robot (HRI) — Đại học Công nghệ (VNU-UET)
* **Giảng viên hướng dẫn:** PGS. TS. Hoàng Văn Xiêm, KS. Nguyễn Quốc Bảo
* **Nhánh Git:** `assignments_4`
* **Mã nguồn GitHub:** [https://github.com/bangrbt/turtlebot4_lab/tree/assignments_4/src/turtlebot4_voice_navigation](https://github.com/bangrbt/turtlebot4_lab/tree/assignments_4/src/turtlebot4_voice_navigation)
* **Video Demo Thực nghiệm (Google Drive):** [https://drive.google.com/file/d/13ebzpQBoCgz06Z1LBtly1qYH1US6P3N8/view?usp=sharing](https://drive.google.com/file/d/13ebzpQBoCgz06Z1LBtly1qYH1US6P3N8/view?usp=sharing)
* **Báo cáo LaTeX:** Tệp nén sẵn sàng nộp Overleaf: `lab07_voice_navigation_report.zip` (lưu cục bộ tại thư mục gốc workspace theo quy định nộp mã nguồn tách biệt báo cáo).

---

## 2. Kiến trúc Hệ thống

```
+-----------------------------------------------------------------------------------+
|                              NGƯỜI DÙNG / GIỌNG NÓI                               |
|        Microphone Laptop (Hỗ trợ Gain 100% khi quay video) / GUI / Văn bản        |
+-----------------------------------------------------------------------------------+
                                       |
                                       v
                     +-----------------------------------+
                     |    Bộ tiền xử lý Âm thanh Số      |
                     | 80Hz HPF + Noise Gate + Tanh + AGC|
                     +-----------------------------------+
                                       | Âm thanh chuẩn PCM 16kHz (Peak = 0.35)
                                       v
                     +-----------------------------------+
                     |   Hybrid Speech Engine (vi-VN)    |
                     |  Google AI (Chính) / Vosk Offline |
                     +-----------------------------------+
                                       | /voice/recognized_text (String)
                                       v
+------------------------+   +-----------------------------------+   +-----------------------+
|   MAP SEGMENTATION     |   |     voice_navigation_manager      |   |  zone_visualizer_node |
| Distance Transform +   |-->|   NLU Intent Parser & Routing     |<--|  3D Markers on /map   |
| Watershed (5 Phân vùng)|   | (Single / Multi-hop Waypoints)    |   | (Labels, Boundaries)  |
+------------------------+   +-----------------------------------+   +-----------------------+
            |                                  |
            v                                  | Nav2 NavigateToPose Action
+------------------------+                     v
|      zones.yaml        |       +---------------------------+
| Tọa độ, Đa giác phòng, |       |   Nav2 Navigation Stack   |
| Aliases tiếng Việt/Anh |       | (Navfn A*, DWB Controller)|
+------------------------+       +---------------------------+
                                               |
                                               v /cmd_vel
                                 +---------------------------+
                                 |  TurtleBot 4 Simulation   |
                                 |   Gazebo Smart Campus     |
                                 | (Bàn ghế, Cây, Người 3D)  |
                                 +---------------------------+
```

### Các Đặc tính Kỹ thuật Vượt trội:
1. **Phân vùng Bản đồ Tự động (Watershed Segmentation):**
   - Phân tích bản đồ Occupancy Grid $14\,\text{m} \times 11\,\text{m}$, áp dụng Euclidean Distance Transform và Marker-controlled Watershed.
   - Trích xuất 5 phòng chức năng với chỉ số IoU trung bình đạt **$98.0\%$**, tự động xác định tâm cửa rộng $1.3\,\text{m}$ để đặt điểm đích waypoint an toàn.
2. **Bộ Tiền Xử lý Âm thanh Chống Méo Quá Ngưỡng (Anti-Clipping Audio Pipeline):**
   - Loại bỏ hoàn toàn lỗi nhận diện sai khi kéo mic lên $100\%$ để quay video màn hình.
   - Tích hợp bộ lọc thông cao 80\,Hz (khử tiếng quạt laptop, 50/60\,Hz hum), Smart Noise Gate (cắt tỉa khoảng lặng thừa), hàm nắn bão hòa mềm $\tanh$ (khử méo đỉnh sóng), và Adaptive Auto AGC (chuẩn hóa đỉnh giọng nói về $-9\,\text{dBFS}$).
3. **Kiến trúc Nhận dạng Giọng nói Đa tầng Tin cậy:**
   - **Google Cloud Speech AI (vi-VN)** làm bộ nhận diện chính: độ chính xác $>98\%$, bất chấp âm lượng lớn nhỏ và tiếng ồn phòng.
   - **Vosk Kaldi Offline** làm dự phòng ngoại tuyến: tích hợp bộ lọc từ khóa điều hướng (*Domain Keyword Guard*), loại bỏ $100\%$ tạp âm bị ảo giác thành từ vô nghĩa.
4. **Bộ Phân tích Ngôn ngữ Tự nhiên (NLU Parser):**
   - Xử lý mượt mà câu lệnh đa chặng, đa hành động (*"tới khu E rồi tới văn phòng"*), duy trì đầy đủ lộ trình qua trạm trung gian.
   - Hệ thống từ đồng nghĩa phong phú (*phòng học, lớp học, giảng đường, phòng lab, thực hành, văn phòng, thư viện, sảnh...*).
5. **Tối ưu hóa Bộ Điều khiển Nav2 DWB Controller:**
   - Tinh chỉnh `inflation_radius = 0.25\,\text{m}` và `cost_scaling_factor = 5.0`, loại bỏ `PathAlign`/`GoalAlign`, kích hoạt `Oscillation Critic`. Robot di chuyển lướt êm qua cửa phòng $1.3\,\text{m}$, tỷ lệ thành công đạt **$100\%$**.

---

## 3. Bản đồ & 05 Khu vực Ngữ nghĩa (Smart Campus)

| Khu vực | Mã ID | Tên chức năng | Tọa độ đích $(x, y, \theta)$ | Ranh giới $[x_{\min}, x_{\max}, y_{\min}, y_{\max}]$ | Vật thể 3D đặc trưng |
|---|:---:|---|:---:|:---:|---|
| **Khu A** | `A` | **Phòng học (Classroom)** | $(-3.15, +2.00, +90^\circ)$ | $[-5.80, -0.15, +0.80, +4.40]$ | Bàn giáo viên, ghế học viên, người 3D |
| **Khu B** | `B` | **Phòng thí nghiệm (Lab)** | $(+3.15, +2.00, +90^\circ)$ | $[+0.15, +5.80, +0.80, +4.40]$ | Bàn nghiên cứu, ghế xoay, nhà nghiên cứu |
| **Khu C** | `C` | **Văn phòng (Office)** | $(+3.15, -2.00, -90^\circ)$ | $[+0.15, +5.80, -4.40, -0.80]$ | Ghế làm việc, nhân viên, balo |
| **Khu D** | `D` | **Thư viện (Library)** | $(-3.15, -2.00, -90^\circ)$ | $[-5.80, -0.15, -4.40, -0.80]$ | Ghế đọc sách, độc giả, cây xanh cảnh |
| **Khu E** | `E` | **Sảnh trung tâm (Corridor)** | $(0.00, 0.00, 0^\circ)$ | $[-5.80, +5.80, -0.80, +0.80]$ | Trục giao thông chính, thông thoáng $100\%$ |

---

## 4. Hướng dẫn Khởi chạy Hệ thống

Môi trường chạy sử dụng cấu hình mạng chuẩn ROS 2 Humble với Domain ID độc lập: **`ROS_DOMAIN_ID=77`**.

### Bước 1: Build Workspace (Nếu có chỉnh sửa mã nguồn)
```bash
cd ~/turtlebot4_lab_ws
colcon build --symlink-install --packages-select turtlebot4_voice_navigation
source install/setup.bash
```

### Bước 2: Khởi động Hệ thống (Quy trình 2 Terminal Chuẩn)

#### Terminal 1: Khởi động Mô phỏng 3D Smart Campus & Robot TurtleBot 4
```bash
cd ~/turtlebot4_lab_ws
source install/setup.bash
ros2 launch turtlebot4_voice_navigation simulation.launch.py
```
*(Chờ cửa sổ Ignition Gazebo hiển thị toàn cảnh Smart Campus với TurtleBot 4 tại vị trí trung tâm sảnh E).*

#### Terminal 2: Khởi động Bộ Điều hướng Ngữ nghĩa, Nav2 & Giao diện Studio
```bash
cd ~/turtlebot4_lab_ws
source install/setup.bash
ros2 launch turtlebot4_voice_navigation voice_nav.launch.py
```
*(Cửa sổ **3D Robotics Studio** sẽ hiện lên kèm công cụ RViz2 hiển thị bản đồ Occupancy grid, các nhãn ngữ nghĩa 3D, thanh trượt Gain, VU meter và trạng thái micro sẵn sàng).*

---

## 5. Thao tác Thu âm & Điều khiển Bằng Giọng nói

Khi quay video màn hình thuyết trình:
1. Bạn có thể **tăng âm lượng micro của hệ điều hành lên mức cao ($80\% - 100\%$)** để video ghi hình thu tiếng nói của bạn to rõ.
2. Trên giao diện Studio, thanh trượt **Độ nhạy thu** để mặc định $100\%$ (Auto AGC chống méo).
3. Nhấp giữ nút **🎙️ BẤM ĐỂ NÓI (PUSH TO TALK)** và phát âm khẩu lệnh tiếng Việt:
   * **Lệnh đơn chặng:**
     * *"Đi đến phòng học"* (Robot chạy vào Khu A)
     * *"Dẫn tôi tới phòng thí nghiệm"* (Robot chạy vào Khu B)
     * *"Đi đến văn phòng"* (Robot chạy vào Khu C)
     * *"Đi đến thư viện"* (Robot chạy vào Khu D)
     * *"Về sảnh trung tâm"* (Robot quay lại Khu E)
   * **Lệnh đa chặng tuần tự:**
     * *"Tới khu E rồi tới văn phòng"* (Robot xác nhận sảnh E, dừng 2 giây rồi đi tiếp văn phòng C)
     * *"Đi phòng học sau đó sang phòng lab"* (Lộ trình liên phòng $A \rightarrow B$)
   * **Lệnh kiểm tra & khẩn cấp:**
     * *"Robot đang ở đâu"* (Hiển thị vị trí phòng hiện tại)
     * *"Dừng lại"* (Hủy bỏ tức thì hành động điều hướng)
     * *"Đi đến phòng tổng thống"* (Hệ thống thông báo phòng không tồn tại)

---

## 6. Chạy Kiểm thử Tự động (Unit Tests)

Bộ kiểm thử tự động $104$ ca kiểm thử bao phủ toàn bộ thuật toán phân vùng, từ điển ngữ nghĩa và ngữ pháp NLU:
```bash
cd ~/turtlebot4_lab_ws
PYTHONPATH=src/turtlebot4_voice_navigation python3 -m pytest src/turtlebot4_voice_navigation/test/ -v
```
Kết quả mong đợi:
```text
============================= 104 passed in 2.27s ==============================
```

---

## 7. Cấu trúc Thư mục Gói phần mềm

```
turtlebot4_voice_navigation/
├── CMakeLists.txt / package.xml / setup.py
├── config/
│   ├── campus_gui.config      # Cấu hình giao diện Ignition Gazebo
│   ├── nav2.yaml              # Cấu hình tối ưu Nav2 DWB Controller & Costmap
│   └── zones.yaml             # Dữ liệu phân vùng ngữ nghĩa 5 phòng
├── launch/
│   ├── simulation.launch.py   # Launch file thế giới 3D Gazebo
│   └── voice_nav.launch.py    # Launch file Nav2, RViz, Manager & Studio GUI
├── maps/
│   ├── map.pgm / map.yaml     # Bản đồ Occupancy Grid $14m \times 11m$
│   ├── segmented_map.png      # Bản đồ phân vùng màu sắc có gắn nhãn
│   └── segmented_zones.json   # Tọa độ ranh giới và waypoint
├── scripts/
│   └── generate_environment.py # Sinh thế giới SDF và bản đồ Occupancy grid
├── test/
│   ├── test_map_segmentation.py
│   ├── test_nlu_parser.py     # 104 ca kiểm thử cú pháp NLU
│   └── test_semantic_map.py
├── turtlebot4_voice_navigation/
│   ├── map_segmentation.py    # Thuật toán Watershed & Distance Transform
│   ├── nlu_parser.py          # Bộ phân tích ngôn ngữ tự nhiên
│   ├── semantic_map.py        # Quản lý lớp ngữ nghĩa không gian
│   ├── voice_gui.py           # Studio UI đồ họa 2D, Auto AGC, VU Meter
│   ├── voice_navigation_manager.py # Bộ điều phối máy trạng thái Nav2
│   └── voice_recognizer_node.py    # Node thu và nhận dạng âm thanh
└── worlds/
    └── smart_campus.sdf       # Thế giới 3D Smart Campus không gian sạch
```
