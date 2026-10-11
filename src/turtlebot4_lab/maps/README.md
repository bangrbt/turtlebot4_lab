# Bản đồ thực nghiệm

map.yaml và map.pgm được lưu trực tiếp từ /map bằng Nav2 map_saver_cli sau khi
người dùng teleop khảo sát world maze trên TurtleBot 4 Standard, ROS 2 Humble.
405 x 404 ô, resolution 0.05 m, origin [-10.1, -10.2, 0]. YAML trỏ `image: map.pgm`.
Log nguồn: evidence/post_reboot_20261011_230015/save_map.log và map_validation.json.
Không tạo/chỉnh ảnh occupancy để thay dữ liệu khảo sát. Bản đồ có một vài ô chưa biết
trong vùng vật cản; kiểm tra free/costmap trước khi gửi goal.
