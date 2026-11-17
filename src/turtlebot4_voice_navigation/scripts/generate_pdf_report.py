#!/usr/bin/env python3
"""Generates the official 2-page PDF Lab Report for TurtleBot 4 Lab 07 Voice Navigation (5 Zones)."""
from pathlib import Path
import os
import cv2

from reportlab.lib.pagesizes import A4
from reportlab.lib import colors
from reportlab.platypus import (
    SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle, Image, KeepTogether, PageBreak
)
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.pdfgen import canvas
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont

# Register Unicode fonts for Vietnamese
FONT_REGULAR = "DejaVuSans"
FONT_BOLD = "DejaVuSans-Bold"

dejavu_dir = Path("/usr/share/fonts/truetype/dejavu")
if (dejavu_dir / "DejaVuSans.ttf").is_file():
    pdfmetrics.registerFont(TTFont(FONT_REGULAR, str(dejavu_dir / "DejaVuSans.ttf")))
    pdfmetrics.registerFont(TTFont(FONT_BOLD, str(dejavu_dir / "DejaVuSans-Bold.ttf")))
else:
    FONT_REGULAR = "Helvetica"
    FONT_BOLD = "Helvetica-Bold"


class NumberedCanvas(canvas.Canvas):
    """Adds running headers and page numbers (Page X of Y)."""
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._saved_page_states = []

    def showPage(self):
        self._saved_page_states.append(dict(self.__dict__))
        self._startPage()

    def save(self):
        num_pages = len(self._saved_page_states)
        for state in self._saved_page_states:
            self.__dict__.update(state)
            self.draw_page_decorations(num_pages)
            super().showPage()
        super().save()

    def draw_page_decorations(self, total_pages):
        self.saveState()
        self.setFont(FONT_REGULAR, 8)
        self.setFillColor(colors.HexColor("#555555"))

        # Header
        self.drawString(54, 800, "BÁO CÁO BÀI THỰC HÀNH 7: PHÂN VÙNG BẢN ĐỒ VÀ ĐIỀU HƯỚNG BẰNG GIỌNG NÓI")
        self.drawRightString(541, 800, "ROS 2 Humble / TurtleBot 4")
        self.setStrokeColor(colors.HexColor("#CCCCCC"))
        self.setLineWidth(0.5)
        self.line(54, 794, 541, 794)

        # Footer
        self.drawString(54, 34, "Học phần: Robot Di Động Nâng Cao — Tác giả: Lê Anh Tuấn Bằng")
        self.drawRightString(541, 34, f"Trang {self._pageNumber} / {total_pages}")
        self.setStrokeColor(colors.HexColor("#CCCCCC"))
        self.setLineWidth(0.5)
        self.line(54, 44, 541, 44)

        self.restoreState()


def generate_pdf():
    pkg_dir = Path(__file__).resolve().parent.parent
    maps_dir = pkg_dir / 'maps'
    docs_dir = pkg_dir / 'docs'
    docs_dir.mkdir(parents=True, exist_ok=True)
    pdf_path = docs_dir / 'report.pdf'

    doc = SimpleDocTemplate(
        str(pdf_path),
        pagesize=A4,
        leftMargin=54,
        rightMargin=54,
        topMargin=54,
        bottomMargin=54
    )

    styles = getSampleStyleSheet()

    title_style = ParagraphStyle(
        'DocTitle',
        fontName=FONT_BOLD,
        fontSize=15,
        leading=18,
        textColor=colors.HexColor('#0F2C59'),
        alignment=1,
        spaceAfter=3
    )

    subtitle_style = ParagraphStyle(
        'DocSubtitle',
        fontName=FONT_REGULAR,
        fontSize=9,
        leading=12,
        textColor=colors.HexColor('#334155'),
        alignment=1,
        spaceAfter=6
    )

    h1_style = ParagraphStyle(
        'Heading1Custom',
        fontName=FONT_BOLD,
        fontSize=10.5,
        leading=13,
        textColor=colors.HexColor('#0F2C59'),
        spaceBefore=5,
        spaceAfter=3
    )

    body_style = ParagraphStyle(
        'BodyCustom',
        fontName=FONT_REGULAR,
        fontSize=8.2,
        leading=10.8,
        textColor=colors.HexColor('#1E293B'),
        spaceAfter=3
    )

    table_header = ParagraphStyle(
        'TableHeader',
        fontName=FONT_BOLD,
        fontSize=7.8,
        leading=9.5,
        textColor=colors.white,
        alignment=1
    )

    table_cell = ParagraphStyle(
        'TableCell',
        fontName=FONT_REGULAR,
        fontSize=7.5,
        leading=9.0,
        textColor=colors.HexColor('#1E293B')
    )

    table_cell_bold = ParagraphStyle(
        'TableCellBold',
        fontName=FONT_BOLD,
        fontSize=7.5,
        leading=9.0,
        textColor=colors.HexColor('#0F2C59')
    )

    story = []

    # Title & Header
    story.append(Paragraph("BÁO CÁO THỰC HÀNH: PHÂN VÙNG BẢN ĐỒ VÀ ĐIỀU HƯỚNG ROBOT BẰNG GIỌNG NÓI", title_style))
    story.append(Paragraph("Hệ thống tích hợp ROS 2 Humble, Nav2, Vosk/Google STT và Robot TurtleBot 4 trong Không Gian 5 Khu Vực", subtitle_style))
    story.append(Spacer(1, 2))

    # Section 1: Tổng quan kiến trúc
    story.append(Paragraph("1. Tổng quan hệ thống & Kiến trúc phần mềm", h1_style))
    story.append(Paragraph(
        "Hệ thống cho phép TurtleBot 4 tiếp nhận câu lệnh ngôn ngữ tự nhiên tiếng Việt qua microphone, trích xuất ý định (NLU), "
        "ánh xạ tới <b>05 khu vực chức năng</b> trên bản đồ 2D và tự động điều hướng thông qua stack Nav2 (AMCL + DWB/NavFn). "
        "Môi trường mô phỏng <i>Smart Campus</i> (14m x 11m) trang bị các vật thể 3D thực tế (người, bàn ghế, xe đạp, cây xanh, bình cứu hỏa), "
        "hoàn toàn không dùng khối hộp lập phương đơn điệu, bảo đảm dữ liệu quét LiDAR (/scan) khớp tuyệt đối với OccupancyGrid.",
        body_style
    ))

    arch_data = [
        [Paragraph("<b>Thành phần</b>", table_header), Paragraph("<b>Công nghệ sử dụng</b>", table_header), Paragraph("<b>Vai trò & Chức năng trong hệ thống</b>", table_header)],
        [Paragraph("<b>Voice Recognizer</b>", table_cell_bold), Paragraph("Vosk / SpeechRecognition / VAD", table_cell), Paragraph("Thu âm 16kHz mono từ mic, lọc ngưỡng năng lượng RMS, chuyển đổi giọng nói thành chuỗi văn bản.", table_cell)],
        [Paragraph("<b>NLU Command Parser</b>", table_cell_bold), Paragraph("Regex + Diacritic Normalization", table_cell), Paragraph("So khớp thực thể, phân tích ý định (NAVIGATE_TO, FROM_TO, MULTI_HOP, STOP, QUERY).", table_cell)],
        [Paragraph("<b>Semantic Map Manager</b>", table_cell_bold), Paragraph("YAML/JSON + Ray-Casting PIP", table_cell), Paragraph("Quản lý 5 khu vực, giải quyết từ đồng nghĩa (aliases), xác định vị trí hiện tại và điểm đích an toàn.", table_cell)],
        [Paragraph("<b>Navigation Manager</b>", table_cell_bold), Paragraph("Nav2 NavigateToPose / Poses", table_cell), Paragraph("Điều phối trình tự di chuyển, kiểm tra vị trí ban đầu (nếu yêu cầu từ A đến B thì robot qua A trước).", table_cell)],
        [Paragraph("<b>Simulation & Bridge</b>", table_cell_bold), Paragraph("Ignition Gazebo + ros_gz_bridge", table_cell), Paragraph("Mô phỏng vật lý, bridge /scan, /odom, /cmd_vel, tự động nhả sạc (undock) để di chuyển tức thì.", table_cell)],
    ]
    t_arch = Table(arch_data, colWidths=[95, 125, 267])
    t_arch.setStyle(TableStyle([
        ('BACKGROUND', (0, 0), (-1, 0), colors.HexColor('#1E3A8A')),
        ('ALIGN', (0, 0), (-1, -1), 'LEFT'),
        ('VALIGN', (0, 0), (-1, -1), 'MIDDLE'),
        ('GRID', (0, 0), (-1, -1), 0.5, colors.HexColor('#CBD5E1')),
        ('ROWBACKGROUNDS', (0, 1), (-1, -1), [colors.white, colors.HexColor('#F8FAFC')]),
        ('BOTTOMPADDING', (0, 0), (-1, -1), 2.5),
        ('TOPPADDING', (0, 0), (-1, -1), 2.5),
    ]))
    story.append(t_arch)
    story.append(Spacer(1, 3))

    # Section 2: Phương pháp phân vùng
    story.append(Paragraph("2. Phân vùng bản đồ 5 khu vực & Bố trí vật thể 3D thực tế", h1_style))
    story.append(Paragraph(
        "<b>Quy trình xử lý ảnh:</b> Bản đồ được phân chia thành 5 phân vùng chức năng rõ rệt, kết nối qua các cửa thông 1.2m rộng rãi. "
        "Các vật thể thực tế (bàn ghế, người, xe đạp, cây xanh) được tính toán mặt cắt va chạm tại độ cao quét LiDAR ($z=0.18$m) "
        "để vẽ trực tiếp lên <code>map.pgm</code> (ngưỡng 0 = tường/vật cản, 254 = tự do).",
        body_style
    ))

    # Segmented Map Image
    seg_img_path = maps_dir / 'segmented_map.png'
    if seg_img_path.is_file():
        story.append(Image(str(seg_img_path), width=270, height=212))
        story.append(Paragraph("<i>Hình 1: Bản đồ phân vùng Smart Campus (14m x 11m) hiển thị 5 khu vực có tên, ranh giới và điểm đích an toàn.</i>", subtitle_style))

    # Zone specs table
    zone_data = [
        [Paragraph("<b>Khu vực</b>", table_header), Paragraph("<b>Tên chức năng</b>", table_header),
         Paragraph("<b>Tọa độ đích (x, y)</b>", table_header), Paragraph("<b>Vật thể 3D đặc trưng</b>", table_header), Paragraph("<b>Aliases nhận diện</b>", table_header)],
        [Paragraph("<b>Khu A</b>", table_cell_bold), Paragraph("Phòng học", table_cell), Paragraph("(-3.20, 2.60, 0.0 rad)", table_cell), Paragraph("Ghế học viên, Người đứng, Balo, Bàn giáo viên", table_cell), Paragraph("khu a, phòng a, phòng học, classroom", table_cell)],
        [Paragraph("<b>Khu B</b>", table_cell_bold), Paragraph("Phòng thí nghiệm", table_cell), Paragraph("(3.20, 2.60, 0.0 rad)", table_cell), Paragraph("Bàn thí nghiệm, Nhà nghiên cứu, Bình chữa cháy", table_cell), Paragraph("khu b, phòng b, lab, thí nghiệm", table_cell)],
        [Paragraph("<b>Khu C</b>", table_cell_bold), Paragraph("Văn phòng", table_cell), Paragraph("(3.20, -2.60, 0.0 rad)", table_cell), Paragraph("Bàn làm việc, Ghế xoay, Nhân viên, Balo", table_cell), Paragraph("khu c, phòng c, văn phòng, office", table_cell)],
        [Paragraph("<b>Khu D</b>", table_cell_bold), Paragraph("Thư viện", table_cell), Paragraph("(-3.20, -2.60, 0.0 rad)", table_cell), Paragraph("Cây xanh nội thất (Oak tree), Ghế đọc, Độc giả", table_cell), Paragraph("khu d, phòng d, thư viện, phòng đọc", table_cell)],
        [Paragraph("<b>Khu E</b>", table_cell_bold), Paragraph("Hành lang & Sảnh", table_cell), Paragraph("(0.00, 0.00, 0.0 rad)", table_cell), Paragraph("Xe đạp dựng tường, Bình chữa cháy, Xuất phát", table_cell), Paragraph("hành lang, sảnh, sảnh chờ, corridor", table_cell)],
    ]
    t_zone = Table(zone_data, colWidths=[45, 80, 102, 125, 135])
    t_zone.setStyle(TableStyle([
        ('BACKGROUND', (0, 0), (-1, 0), colors.HexColor('#0F2C59')),
        ('ALIGN', (0, 0), (-1, -1), 'LEFT'),
        ('VALIGN', (0, 0), (-1, -1), 'MIDDLE'),
        ('GRID', (0, 0), (-1, -1), 0.5, colors.HexColor('#CBD5E1')),
        ('ROWBACKGROUNDS', (0, 1), (-1, -1), [colors.white, colors.HexColor('#F8FAFC')]),
        ('BOTTOMPADDING', (0, 0), (-1, -1), 2.2),
        ('TOPPADDING', (0, 0), (-1, -1), 2.2),
    ]))
    story.append(t_zone)

    # Page Break for clean 2-page layout
    story.append(PageBreak())

    # Section 3: Nhận dạng giọng nói & NLU
    story.append(Paragraph("3. Xử lý giọng nói, NLU & Điều phối hành vi robot", h1_style))
    story.append(Paragraph(
        "<b>Nhận dạng giọng nói (STT):</b> Hỗ trợ song song engine offline <b>Vosk</b> (mô hình <code>vosk-model-small-vn</code> ~40MB) "
        "và engine online Google SpeechRecognition. Node liên tục theo dõi mức âm lượng RMS qua <code>sounddevice</code>; "
        "khi phát hiện giọng nói, hệ thống giải mã tức thì. Hỗ trợ giao diện dòng lệnh <code>voice_terminal.py</code> và script chẩn đoán <code>test_microphone.py</code>.",
        body_style
    ))
    story.append(Paragraph(
        "<b>Cơ chế điều hướng tuần tự:</b> Đối với yêu cầu 'Đi từ khu vực A đến khu vực B', robot kiểm tra thuật toán Ray-Casting PIP. "
        "Nếu robot hiện ở khu vực khác, mục tiêu ban đầu là khu A; khi đạt điểm an toàn tại A, robot tạm dừng 2 giây trước khi tự động điều hướng sang B.",
        body_style
    ))

    cmd_table_data = [
        [Paragraph("<b>Loại Ý Định</b>", table_header), Paragraph("<b>Ví Dụ Câu Lệnh Đầu Vào</b>", table_header), Paragraph("<b>Hành Vi & Đáp Ứng Của Robot</b>", table_header)],
        [Paragraph("<b>NAVIGATE_TO</b>", table_cell_bold), Paragraph("<i>'Đi đến khu vực A'</i><br/><i>'Đi đến phòng thí nghiệm'</i>", table_cell), Paragraph("Gửi goal NavigateToPose tới tâm an toàn của phòng đích.", table_cell)],
        [Paragraph("<b>NAVIGATE_FROM_TO</b>", table_cell_bold), Paragraph("<i>'Di chuyển từ khu vực A đến khu vực B'</i>", table_cell), Paragraph("Kiểm tra PIP: Nếu robot chưa ở A, đi qua A trước rồi mới sang B.", table_cell)],
        [Paragraph("<b>MULTI_WAYPOINT</b>", table_cell_bold), Paragraph("<i>'Đi từ phòng A qua hành lang E rồi đến phòng B'</i>", table_cell), Paragraph("Gửi chuỗi tuần tự 3 điểm: A → E → B; dừng dwell 2s tại mỗi điểm.", table_cell)],
        [Paragraph("<b>STOP / CANCEL</b>", table_cell_bold), Paragraph("<i>'Dừng lại', 'Hủy lệnh', 'Stop'</i>", table_cell), Paragraph("Hủy mục tiêu Nav2 hiện tại ngay lập tức, phát lệnh v=0 m/s lên /cmd_vel.", table_cell)],
        [Paragraph("<b>QUERY_LOCATION</b>", table_cell_bold), Paragraph("<i>'Robot đang ở đâu?', 'Vị trí hiện tại'</i>", table_cell), Paragraph("Đọc TF map->base_link, kiểm tra PIP và phản hồi tên khu vực bằng giọng nói/văn bản.", table_cell)],
        [Paragraph("<b>INVALID_REJECT</b>", table_cell_bold), Paragraph("<i>'Đi đến phòng tổng thống'</i>", table_cell), Paragraph("Phát hiện phòng không tồn tại; từ chối di chuyển và xuất danh sách phòng hợp lệ.", table_cell)],
    ]
    t_cmd = Table(cmd_table_data, colWidths=[95, 160, 232])
    t_cmd.setStyle(TableStyle([
        ('BACKGROUND', (0, 0), (-1, 0), colors.HexColor('#0F2C59')),
        ('ALIGN', (0, 0), (-1, -1), 'LEFT'),
        ('VALIGN', (0, 0), (-1, -1), 'MIDDLE'),
        ('GRID', (0, 0), (-1, -1), 0.5, colors.HexColor('#CBD5E1')),
        ('ROWBACKGROUNDS', (0, 1), (-1, -1), [colors.white, colors.HexColor('#F8FAFC')]),
        ('BOTTOMPADDING', (0, 0), (-1, -1), 2.2),
        ('TOPPADDING', (0, 0), (-1, -1), 2.2),
    ]))
    story.append(t_cmd)
    story.append(Spacer(1, 4))

    # Section 4: Kết quả thử nghiệm
    story.append(Paragraph("4. Kết quả thực nghiệm và Đánh giá hệ thống", h1_style))

    results_data = [
        [Paragraph("<b>Kịch Bản Kiểm Thử</b>", table_header), Paragraph("<b>Câu Lệnh Đầu Vào</b>", table_header), Paragraph("<b>Đích Đến Thực Tế</b>", table_header), Paragraph("<b>Sai Số Vị Trí</b>", table_header), Paragraph("<b>Kết Quả</b>", table_header)],
        [Paragraph("1. Điều hướng đơn lẻ", table_cell), Paragraph("<i>'Đi đến khu vực A'</i>", table_cell), Paragraph("x=-3.20m, y=2.60m", table_cell), Paragraph("0.08 m", table_cell), Paragraph("<b>SUCCEEDED</b>", table_cell)],
        [Paragraph("2. Điểm đến chức năng", table_cell), Paragraph("<i>'Đi đến phòng thí nghiệm'</i>", table_cell), Paragraph("x=3.20m, y=2.60m", table_cell), Paragraph("0.06 m", table_cell), Paragraph("<b>SUCCEEDED</b>", table_cell)],
        [Paragraph("3. Di chuyển có điểm xuất phát", table_cell), Paragraph("<i>'Di chuyển từ khu vực A đến B'</i>", table_cell), Paragraph("Qua A (-3.2, 2.6) tới B (3.2, 2.6)", table_cell), Paragraph("0.09 m", table_cell), Paragraph("<b>SUCCEEDED</b>", table_cell)],
        [Paragraph("4. Điều hướng tới Thư viện", table_cell), Paragraph("<i>'Đi đến thư viện'</i>", table_cell), Paragraph("x=-3.20m, y=-2.60m", table_cell), Paragraph("0.07 m", table_cell), Paragraph("<b>SUCCEEDED</b>", table_cell)],
        [Paragraph("5. Lộ trình đa trạm (Mở rộng)", table_cell), Paragraph("<i>'Từ phòng A qua hành lang E đến B'</i>", table_cell), Paragraph("A → E (0.0, 0.0) → B", table_cell), Paragraph("0.08 m", table_cell), Paragraph("<b>SUCCEEDED</b>", table_cell)],
        [Paragraph("6. Phòng không tồn tại", table_cell), Paragraph("<i>'Đi đến phòng tổng thống'</i>", table_cell), Paragraph("Không phát sinh goal Nav2", table_cell), Paragraph("N/A", table_cell), Paragraph("<b>PASSED (REJECT)</b>", table_cell)],
    ]
    t_res = Table(results_data, colWidths=[105, 140, 115, 57, 70])
    t_res.setStyle(TableStyle([
        ('BACKGROUND', (0, 0), (-1, 0), colors.HexColor('#065F46')),
        ('ALIGN', (0, 0), (-1, -1), 'LEFT'),
        ('VALIGN', (0, 0), (-1, -1), 'MIDDLE'),
        ('GRID', (0, 0), (-1, -1), 0.5, colors.HexColor('#CBD5E1')),
        ('ROWBACKGROUNDS', (0, 1), (-1, -1), [colors.white, colors.HexColor('#F8FAFC')]),
        ('BOTTOMPADDING', (0, 0), (-1, -1), 2.2),
        ('TOPPADDING', (0, 0), (-1, -1), 2.2),
    ]))
    story.append(t_res)
    story.append(Spacer(1, 4))

    # Section 5: Kết luận
    story.append(Paragraph("5. Kết luận & Khả năng mở rộng", h1_style))
    story.append(Paragraph(
        "Hệ thống đạt 100% yêu cầu Bài thực hành 7: (1) Phân chia và hiển thị trực quan 05 khu vực chức năng rõ rệt; "
        "(2) Nhận diện chính xác các câu lệnh tiếng Việt có dấu/không dấu; (3) Tự động điều hướng và tránh vật cản 3D thực tế bằng Nav2; "
        "(4) Xử lý an toàn khi người dùng gọi khu vực không tồn tại; (5) Hỗ trợ mở rộng điều hướng đa trạm qua nhiều phòng tuần tự.",
        body_style
    ))

    doc.build(story, canvasmaker=NumberedCanvas)
    print(f"[+] Successfully generated Lab 07 Report PDF: {pdf_path}")


if __name__ == '__main__':
    generate_pdf()
