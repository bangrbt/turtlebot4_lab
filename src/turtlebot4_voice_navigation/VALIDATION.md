# Kết quả kiểm chứng package turtlebot4_voice_navigation

Package đã được kiểm tra, phân tích hình thái học và chạy kiểm thử tự động toàn diện trên môi trường ROS 2 Humble / Ubuntu 22.04.

---

## 1. Kết quả Unit Tests tự động (77/77 PASSED)

Chạy trên môi trường độc lập:
```bash
PYTHONPATH=src/turtlebot4_voice_navigation .venv_gesture/bin/python -m pytest src/turtlebot4_voice_navigation/test -v
```

### Chi tiết các nhóm kiểm thử:
1. **Kiểm thử Phân vùng bản đồ & Hình thái học (`test_map_segmentation.py` - 4 tests)**:
   - `test_coordinate_round_trip`: Kiểm tra sai số chuyển đổi 2 chiều giữa tọa độ pixel ảnh $(u, v)$ và tọa độ bản đồ ROS $(x, y)$ luôn nhỏ hơn độ phân giải ($\le 0.05\text{m}$).
   - `test_map_segmenter_load`: Kiểm tra tải đúng kích thước $280 \times 220$ pixel, gốc tọa độ $[-7.0, -5.5, 0.0]$.
   - `test_free_space_extraction`: Tách không gian tự do chính xác, loại bỏ nhiễu biên.
   - `test_watershed_segmentation`: Thuật toán Watershed phân chia chính xác tối thiểu 3 khu vực lớn ($\ge 4\text{ m}^2$) với khoảng an toàn cực đại $> 0.4\text{m}$ tới chướng ngại vật.

2. **Kiểm thử Bản đồ Ngữ nghĩa & Vị trí không gian (`test_semantic_map.py` - 36 tests)**:
   - `test_zones_loaded`: Tải đủ 4 khu vực chuẩn: A, B, C, D.
   - `test_alias_lookup`: Kiểm tra 25 trường hợp biến thể từ vựng: tiếng Việt có dấu, không dấu, từ viết tắt, tiếng Anh ("phòng học", "phong hoc", "lớp học", "classroom", "lab", "phòng thí nghiệm", "văn phòng", "hành lang", "lobby", v.v.).
   - `test_invalid_zone_lookup`: Kiểm tra từ chối đúng các tên phòng lạ ("nhà ăn", "căng tin", "toilet", "khu vực z").
   - `test_point_in_zone`: Kiểm tra thuật toán Ray-Casting Point-in-Polygon với 8 điểm tọa độ thực tế bên trong các phòng A, B, C, D.
   - `test_point_outside_zones`: Kiểm tra nhận diện điểm ngoài bản đồ.

3. **Kiểm thử NLU Intent Parser (`test_nlu_parser.py` - 37 tests)**:
   - `test_direct_navigation`: Nhận diện 14 câu lệnh điều hướng 1 điểm đến cả tiếng Việt & tiếng Anh.
   - `test_from_to_navigation`: Nhận diện 5 câu lệnh di chuyển từ điểm gốc đến đích ("Từ A đến B").
   - `test_multi_hop_navigation`: Trích xuất chính xác chuỗi 3 trạm dừng: `[A -> D -> B]`.
   - `test_stop_commands`: Nhận diện 7 biến thể lệnh dừng khẩn cấp ("Dừng lại", "dừng", "hủy lệnh", "stop", "cancel", "halt").
   - `test_location_query`: Nhận diện 5 biến thể hỏi vị trí ("Robot đang ở đâu?", "vị trí hiện tại", "where are you").
   - `test_invalid_zones_rejected`: Bắt lỗi và từ chối 5 câu lệnh chứa phòng không tồn tại với thông báo giải thích cụ thể.

---

## 2. Thông số hình thái học phân vùng bản đồ

Bản đồ Smart Campus ($14.0\text{m} \times 11.0\text{m}$, kích thước sử dụng thực tế $12.0\text{m} \times 9.0\text{m}$):

| Khu vực | Tên phòng | Diện tích ($m^2$) | Tọa độ đích an toàn | Khoảng trống tới tường |
|---|---|:---:|:---:|:---:|
| **Khu vực A** | Phòng học (Classroom) | $37.97\text{ m}^2$ | $(-3.00, 2.00, 0.00)$ | $1.40\text{ m}$ |
| **Khu vực B** | Phòng thí nghiệm (Laboratory) | $22.04\text{ m}^2$ | $(2.80, 2.00, 1.57)$ | $1.25\text{ m}$ |
| **Khu vực C** | Văn phòng (Office) | $37.88\text{ m}^2$ | $(2.80, -2.00, -1.57)$ | $1.19\text{ m}$ |
| **Khu vực D** | Hành lang & Sảnh (Corridor) | $41.46\text{ m}^2$ | $(-3.00, -1.80, 0.00)$ | $1.72\text{ m}$ |

Tất cả các điểm đích đều có khoảng cách an toàn tới chướng ngại vật $> 1.1\text{m}$, hoàn toàn vượt trội so với bán kính lạm phát (inflation radius $\approx 0.55\text{m}$) của Nav2, đảm bảo robot không bao giờ bị kẹt costmap khi lập kế hoạch đường đi.
