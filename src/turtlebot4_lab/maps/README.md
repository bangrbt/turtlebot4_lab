# Bản đồ thực

Thư mục này cố ý chưa có `map.yaml`/`map.pgm` khi chưa mapping thật.
Không sử dụng bản đồ mẫu làm kết quả bài thực hành.

Sau khi teleop khảo sát và kiểm tra bản đồ, lưu vào source bằng:

```bash
ros2 run turtlebot4_lab save_map.py --output "$TB4_LAB_WS/src/turtlebot4_lab/maps/map"
ros2 run turtlebot4_lab validate_map.py "$TB4_LAB_WS/src/turtlebot4_lab/maps/map.yaml"
```

`image: map.pgm` phải là đường dẫn tương đối. Build lại để cài bản đồ vào package,
hoặc truyền `map:=` với đường dẫn tuyệt đối đến bản đồ trong source.
