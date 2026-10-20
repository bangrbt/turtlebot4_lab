# Báo cáo Bài thực hành 05 — Lê Anh Tuấn Bằng, 23020723

- [PDF hoàn chỉnh](report.pdf)
- [Tệp LaTeX chính](report.tex); nội dung tách trong `01_system.tex` … `05_discussion.tex`.
- [Video demo](https://drive.google.com/file/d/1L9oFTGC_8RS8bYuAewitxUT4dYWiAlEM/view)
- [Mã nguồn](https://github.com/bangrbt/turtlebot4_lab/tree/assignments_2/src/turtlebot4_lab05)

Bìa được điều chỉnh theo mẫu người thực hiện cung cấp: bài 05, nhánh assignments_2,
package turtlebot4_lab05, đúng tên sinh viên/MSSV/giảng viên. Dùng chữ VNU--UET dự
phòng khi không có tệp logo, theo mẫu. Không thay báo cáo bài 04 ở root workspace.

## Biên dịch

Chạy từ thư mục chứa `report.tex`, giữ nguyên các tệp `.tex` con và `figures/`:

```bash
cd ~/turtlebot4_lab_ws/src/turtlebot4_lab05/report
tectonic report.tex
```

Hoặc TeX Live/Overleaf (pdfLaTeX), cài bộ tiếng Việt, TikZ, các package trong preamble:

```bash
pdflatex -interaction=nonstopmode -halt-on-error report.tex
pdflatex -interaction=nonstopmode -halt-on-error report.tex
```

XeLaTeX/Tectonic dùng Liberation Serif/Sans/Mono có hỗ trợ tiếng Việt. pdfLaTeX
sử dụng UTF-8, fontenc T5, babel vietnamese theo mẫu. Trên Overleaf chọn pdfLaTeX
nếu không có Liberation fonts. Chạy lại để cập nhật mục lục/tham chiếu nếu trình
biên dịch yêu cầu. Các hình đã được đính kèm, không cần chạy ROS để biên dịch.

## Nguồn hình và số liệu

- `map_goals_objects.pdf`: dựng từ map PGM và YAML goal/8 vật thể hiện tại.
  Không phải ảnh của một lượt chạy runtime mới.
- `rviz_navigation.png`, `yolo_observed.png`, `trajectory_historical.png`: sao
  nguyên ảnh kiểm chứng trong `../validation/`; chú thích rõ scene/phiên bản lịch sử.
- `parallel_evidence.pdf`: số đếm lấy từ `scattered_demo_summary.json` (15/10/2026).
- `video_navigation.jpg`: frame tại 484.6667 s (08:04.7) của video người thực hiện,
  cho thấy RViz active và camera YOLO bicycle 0.91 đồng thời.
- `video_bicycle.jpg`: crop nguyên cửa sổ camera tại 949.9467 s (15:49.9), bicycle
  0.94, chair 0.86/0.87; không sửa bbox/nhãn. Đây là crop được ghi rõ, không ghép
  thành một màn hình runtime khác.
- `figure_provenance.json`: URL, SHA256 video, metadata, thời điểm/crop ảnh.
  Video gốc 2.43 GB không đưa vào Git. Chỉ trích và đối chiếu các khung đại diện;
  không coi FPS màn hình là FPS inference hoặc suy ra metrics tự động mới từ ảnh.
- `layout_checks.json`: số trang, kiểm tra overfull/missing glyph/reference và biên
  trang PDF. Đã xem trực quan từng trang, đặc biệt sơ đồ, bảng và ảnh; không coi
  kiểm tra bbox tự động là chứng minh mọi loại chồng lấn trong ảnh chụp gốc.

Tái dựng hình từ dữ liệu nguồn với môi trường YOLO đã chuẩn bị:

```bash
python generate_figures.py --package ..
# Có video gốc cục bộ thì tái tạo luôn hai frame đã trích:
python generate_figures.py --package .. --video /path/to/video.mp4
```

Report giải thích model/config hiện tại theo source nền `a90f949` và kiểm tra
20/10/2026. Bảng runtime được ghi đúng phiên bản lịch sử, không lấy confidence
cực đại thay accuracy/mAP và không gán tốc độ YOLO cũ cho YOLO11s mới.
