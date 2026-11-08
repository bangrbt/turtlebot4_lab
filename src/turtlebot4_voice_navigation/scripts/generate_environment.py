#!/usr/bin/env python3
"""Generates the Smart Campus SDF world, ground truth OccupancyGrid map.pgm/yaml,
and segmented visual map with 5 semantic zones and realistic 3D objects.
"""
from pathlib import Path
import json
import math
import cv2
import numpy as np
import yaml

# Map parameters: 14.0m x 11.0m, resolution 0.05m/pixel (280 x 220 px)
MAP_WIDTH_M = 14.0
MAP_HEIGHT_M = 11.0
RESOLUTION = 0.05
ORIGIN_X = -7.0
ORIGIN_Y = -5.5

PX_W = int(round(MAP_WIDTH_M / RESOLUTION))   # 280
PX_H = int(round(MAP_HEIGHT_M / RESOLUTION))  # 220


def world_to_px(x, y):
    """Convert metric (x, y) in map frame to pixel (col, row)."""
    col = int(round((x - ORIGIN_X) / RESOLUTION))
    row = int(round((ORIGIN_Y + MAP_HEIGHT_M - y) / RESOLUTION))
    return col, row


def px_to_world(col, row):
    """Convert pixel (col, row) back to metric (x, y)."""
    x = ORIGIN_X + col * RESOLUTION
    y = ORIGIN_Y + (PX_H - 1 - row) * RESOLUTION
    return round(x, 3), round(y, 3)


def generate_environment():
    pkg_dir = Path(__file__).resolve().parent.parent
    worlds_dir = pkg_dir / 'worlds'
    maps_dir = pkg_dir / 'maps'
    config_dir = pkg_dir / 'config'
    models_dir = pkg_dir / 'models'

    worlds_dir.mkdir(parents=True, exist_ok=True)
    maps_dir.mkdir(parents=True, exist_ok=True)
    config_dir.mkdir(parents=True, exist_ok=True)

    # 1. Prepare Occupancy Grid: 205 = unknown/exterior, 254 = free space, 0 = occupied
    grid = np.full((PX_H, PX_W), 205, dtype=np.uint8)

    # Interior boundary: x in [-5.925, 5.925], y in [-4.425, 4.425]
    c1, r1 = world_to_px(-5.925, 4.425)
    c2, r2 = world_to_px(5.925, -4.425)
    grid[min(r1, r2):max(r1, r2)+1, min(c1, c2):max(c1, c2)+1] = 254

    # 2. Structural walls: (name, x, y, dx, dy, color_rgba)
    # Wall thickness: 0.15m.
    walls = [
        # Outer boundary
        ("north_wall", 0.0, 4.5, 12.15, 0.15, (0.2, 0.3, 0.4, 1.0)),
        ("south_wall", 0.0, -4.5, 12.15, 0.15, (0.2, 0.3, 0.4, 1.0)),
        ("west_wall", -6.0, 0.0, 0.15, 9.15, (0.2, 0.3, 0.4, 1.0)),
        ("east_wall", 6.0, 0.0, 0.15, 9.15, (0.2, 0.3, 0.4, 1.0)),

        # Central vertical dividers (dividing North rooms A & B, and South rooms D & C)
        ("divider_north_v", 0.0, 2.65, 0.15, 3.7, (0.25, 0.35, 0.45, 1.0)),
        ("divider_south_v", 0.0, -2.65, 0.15, 3.7, (0.25, 0.35, 0.45, 1.0)),

        # Corridor North wall (y = 0.8) with doorways at x in [-3.8, -2.6] (Zone A) and x in [2.6, 3.8] (Zone B)
        ("corridor_n_west", -4.9, 0.8, 2.2, 0.15, (0.25, 0.35, 0.45, 1.0)),
        ("corridor_n_mid", 0.0, 0.8, 5.0, 0.15, (0.25, 0.35, 0.45, 1.0)),
        ("corridor_n_east", 4.9, 0.8, 2.2, 0.15, (0.25, 0.35, 0.45, 1.0)),

        # Corridor South wall (y = -0.8) with doorways at x in [-3.8, -2.6] (Zone D) and x in [2.6, 3.8] (Zone C)
        ("corridor_s_west", -4.9, -0.8, 2.2, 0.15, (0.25, 0.35, 0.45, 1.0)),
        ("corridor_s_mid", 0.0, -0.8, 5.0, 0.15, (0.25, 0.35, 0.45, 1.0)),
        ("corridor_s_east", 4.9, -0.8, 2.2, 0.15, (0.25, 0.35, 0.45, 1.0)),
    ]

    # Draw walls on OccupancyGrid
    for name, wx, wy, dx, dy, _ in walls:
        c_min, r_min = world_to_px(wx - dx/2.0, wy + dy/2.0)
        c_max, r_max = world_to_px(wx + dx/2.0, wy - dy/2.0)
        r_start = max(0, min(r_min, r_max))
        r_end = min(PX_H - 1, max(r_min, r_max))
        c_start = max(0, min(c_min, c_max))
        c_end = min(PX_W - 1, max(c_min, c_max))
        grid[r_start:r_end+1, c_start:c_end+1] = 0

    # 3. Realistic 3D Objects specification
    # All objects are placed with large clearances (>1.0m) from walls and doorways.
    # The bicycle is completely removed as requested.
    objects = [
        # --- ZONE A: PHÒNG HỌC (Classroom) ---
        {
            "name": "zone_a_teacher_desk",
            "asset": "furniture",
            "pose": (-3.6, 2.7, 0.375, 0, 0, 0),
            "collision_type": "box",
            "collision_size": (0.7, 1.4, 0.75),
            "color_rgba": (0.6, 0.4, 0.25, 1.0)
        },
        {
            "name": "zone_a_chair_teacher",
            "asset": "chair",
            "pose": (-4.3, 2.7, 0.0, 0, 0, 0),
            "mesh_scale": (0.00817, 0.00817, 0.00817),
            "collision_type": "box",
            "collision_size": (0.65, 0.65, 0.9)
        },
        {
            "name": "zone_a_chair_student1",
            "asset": "chair",
            "pose": (-2.9, 2.7, 0.0, 0, 0, 3.1415),
            "mesh_scale": (0.00817, 0.00817, 0.00817),
            "collision_type": "box",
            "collision_size": (0.65, 0.65, 0.9)
        },
        {
            "name": "zone_a_person_student",
            "asset": "person",
            "pose": (-1.8, 3.0, 0.0, 0, 0, -1.57),
            "mesh_scale": (0.85, 0.85, 0.85),
            "collision_type": "box",
            "collision_size": (0.45, 0.4, 1.7)
        },
        {
            "name": "zone_a_backpack",
            "asset": "backpack",
            "pose": (-4.8, 3.6, 0.0, 0, 0, 0),
            "mesh_scale": (0.8, 0.8, 0.8),
            "collision_type": "box",
            "collision_size": (0.35, 0.35, 0.45)
        },

        # --- ZONE B: PHÒNG THÍ NGHIỆM (Laboratory) ---
        {
            "name": "zone_b_lab_bench",
            "asset": "furniture",
            "pose": (3.6, 2.7, 0.4, 0, 0, 0),
            "collision_type": "box",
            "collision_size": (0.8, 1.6, 0.8),
            "color_rgba": (0.2, 0.4, 0.6, 1.0)
        },
        {
            "name": "zone_b_chair_lab",
            "asset": "chair",
            "pose": (4.4, 2.7, 0.0, 0, 0, 3.1415),
            "mesh_scale": (0.00817, 0.00817, 0.00817),
            "collision_type": "box",
            "collision_size": (0.65, 0.65, 0.9)
        },
        {
            "name": "zone_b_person_researcher",
            "asset": "person",
            "pose": (1.8, 3.0, 0.0, 0, 0, -1.57),
            "mesh_scale": (0.85, 0.85, 0.85),
            "collision_type": "box",
            "collision_size": (0.45, 0.4, 1.7)
        },
        {
            "name": "zone_b_fire_extinguisher",
            "asset": "fire_hydrant",
            "pose": (4.8, 1.8, 0.0, 0, 0, 0),
            "mesh_scale": (0.85, 0.85, 0.85),
            "collision_type": "cylinder",
            "collision_size": (0.2, 0.8)
        },

        # --- ZONE C: VĂN PHÒNG (Office) ---
        {
            "name": "zone_c_office_desk",
            "asset": "furniture",
            "pose": (3.6, -2.7, 0.375, 0, 0, 0),
            "collision_type": "box",
            "collision_size": (0.8, 1.6, 0.75),
            "color_rgba": (0.45, 0.28, 0.18, 1.0)
        },
        {
            "name": "zone_c_chair_exec",
            "asset": "chair",
            "pose": (4.4, -2.7, 0.0, 0, 0, 3.1415),
            "mesh_scale": (0.00817, 0.00817, 0.00817),
            "collision_type": "box",
            "collision_size": (0.65, 0.65, 0.9)
        },
        {
            "name": "zone_c_person_staff",
            "asset": "person",
            "pose": (1.8, -3.0, 0.0, 0, 0, 1.57),
            "mesh_scale": (0.85, 0.85, 0.85),
            "collision_type": "box",
            "collision_size": (0.45, 0.4, 1.7)
        },
        {
            "name": "zone_c_backpack",
            "asset": "backpack",
            "pose": (5.0, -3.6, 0.0, 0, 0, 0),
            "mesh_scale": (0.8, 0.8, 0.8),
            "collision_type": "box",
            "collision_size": (0.35, 0.35, 0.45)
        },

        # --- ZONE D: THƯ VIỆN (Library / Reading Room) ---
        {
            "name": "zone_d_reading_table",
            "asset": "furniture",
            "pose": (-3.6, -2.7, 0.35, 0, 0, 0),
            "collision_type": "box",
            "collision_size": (0.8, 1.4, 0.7),
            "color_rgba": (0.55, 0.45, 0.35, 1.0)
        },
        {
            "name": "zone_d_chair_reading",
            "asset": "chair",
            "pose": (-4.3, -2.7, 0.0, 0, 0, 0),
            "mesh_scale": (0.00817, 0.00817, 0.00817),
            "collision_type": "box",
            "collision_size": (0.65, 0.65, 0.9)
        },
        {
            "name": "zone_d_indoor_potted_plant",
            "asset": "potted_plant",
            "pose": (-4.6, -1.8, 0.0, 0, 0, 0),
            "collision_type": "cylinder",
            "collision_size": (0.24, 1.1)
        },
        {
            "name": "zone_d_person_reader",
            "asset": "person",
            "pose": (-1.8, -3.0, 0.0, 0, 0, 1.57),
            "mesh_scale": (0.85, 0.85, 0.85),
            "collision_type": "box",
            "collision_size": (0.45, 0.4, 1.7)
        },

        # --- ZONE E: HÀNH LANG & SẢNH CHỜ (Corridor & Lobby) ---
        # Bicycle is removed completely. Fire hydrant placed at far west end.
        {
            "name": "zone_e_lobby_hydrant",
            "asset": "fire_hydrant",
            "pose": (-5.2, 0.0, 0.0, 0, 0, 0),
            "mesh_scale": (0.85, 0.85, 0.85),
            "collision_type": "cylinder",
            "collision_size": (0.2, 0.8)
        },

        # --- SMALL PASSAGEWAY OBSTACLES: CORRIDOR & ROOM ENTRANCES ---
        # Carefully positioned along corridor and doorway thresholds to test dynamic navigation and obstacle avoidance
        {
            "name": "obs_wet_floor_sign_corridor",
            "asset": "wet_floor_sign",
            "pose": (-1.2, 0.35, 0.0, 0, 0, 0),
            "collision_type": "box",
            "collision_size": (0.28, 0.20, 0.48)
        },
        {
            "name": "obs_caution_cone_corridor",
            "asset": "caution_cone",
            "pose": (1.2, -0.35, 0.0, 0, 0, 0),
            "collision_type": "cylinder",
            "collision_size": (0.15, 0.45)
        },
        {
            "name": "obs_recycling_bin_door_a",
            "asset": "trash_bin",
            "pose": (-2.6, 1.25, 0.0, 0, 0, 0),
            "collision_type": "cylinder",
            "collision_size": (0.16, 0.58)
        },
        {
            "name": "obs_parcel_box_door_b",
            "asset": "parcel_box",
            "pose": (2.6, 1.25, 0.0, 0, 0, 0.2),
            "collision_type": "box",
            "collision_size": (0.32, 0.28, 0.24)
        },
        {
            "name": "obs_caution_cone_door_c",
            "asset": "caution_cone",
            "pose": (2.6, -1.25, 0.0, 0, 0, 0),
            "collision_type": "cylinder",
            "collision_size": (0.15, 0.45)
        },
        {
            "name": "obs_supply_box_door_d",
            "asset": "parcel_box",
            "pose": (-2.6, -1.25, 0.0, 0, 0, -0.2),
            "collision_type": "box",
            "collision_size": (0.32, 0.28, 0.24)
        },
    ]

    # Draw obstacles onto Occupancy Grid matching exact collision footprint at LiDAR height
    for obj in objects:
        ox, oy, oz, _, _, yaw = obj["pose"]
        c_type = obj["collision_type"]
        c_size = obj["collision_size"]

        if c_type == "box":
            dx, dy, _ = c_size
            # Handle yaw rotation if significant
            if abs(yaw) > 1.0 and abs(yaw) < 2.5:
                # Swapped dx, dy for ~90 degree rotations
                dx, dy = dy, dx
            c_min, r_min = world_to_px(ox - dx/2.0, oy + dy/2.0)
            c_max, r_max = world_to_px(ox + dx/2.0, oy - dy/2.0)
            r_start = max(0, min(r_min, r_max))
            r_end = min(PX_H - 1, max(r_min, r_max))
            c_start = max(0, min(c_min, c_max))
            c_end = min(PX_W - 1, max(c_min, c_max))
            grid[r_start:r_end+1, c_start:c_end+1] = 0

        elif c_type == "cylinder":
            radius, _ = c_size
            cc, cr = world_to_px(ox, oy)
            pr = int(math.ceil(radius / RESOLUTION))
            cv2.circle(grid, (cc, cr), pr, 0, -1)

    # 4. Save Occupancy Grid map.pgm and map.yaml
    map_pgm_path = maps_dir / 'map.pgm'
    cv2.imwrite(str(map_pgm_path), grid)
    print(f"[+] Saved Occupancy Grid: {map_pgm_path}")

    map_yaml_path = maps_dir / 'map.yaml'
    yaml_content = {
        'image': 'map.pgm',
        'resolution': RESOLUTION,
        'origin': [ORIGIN_X, ORIGIN_Y, 0.0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.25
    }
    with open(map_yaml_path, 'w', encoding='utf-8') as f:
        yaml.dump(yaml_content, f, sort_keys=False)
    print(f"[+] Saved Map YAML: {map_yaml_path}")

    # 5. Define 5 Semantic Zones with metadata and waypoints
    zones_data = {
        "metadata": {
            "environment_name": "Smart Campus TurtleBot 4 Lab 07",
            "map_width_m": MAP_WIDTH_M,
            "map_height_m": MAP_HEIGHT_M,
            "resolution": RESOLUTION,
            "origin": [ORIGIN_X, ORIGIN_Y, 0.0],
            "total_zones": 5
        },
        "zones": [
            {
                "id": "A",
                "name": "Phòng học",
                "name_en": "Classroom",
                "color_bgr": [140, 210, 245],       # Soft warm yellow/orange
                "color_hex": "#F5D28C",
                "bounds": [-5.8, -0.15, 0.8, 4.4],   # [x_min, x_max, y_min, y_max]
                "center": [-3.2, 2.6],
                "waypoint": [-2.2, 2.0, 0.0],       # [x, y, yaw]
                "description": "Khu vực giảng dạy, phòng học trang bị bàn ghế và bảng giảng viên.",
                "aliases": ["khu vuc a", "khu a", "phong hoc", "lop hoc", "classroom", "room a", "a"]
            },
            {
                "id": "B",
                "name": "Phòng thí nghiệm",
                "name_en": "Laboratory",
                "color_bgr": [235, 205, 140],       # Soft cyan/teal
                "color_hex": "#8CCDDE",
                "bounds": [0.15, 5.8, 0.8, 4.4],
                "center": [3.2, 2.6],
                "waypoint": [2.2, 2.0, 0.0],
                "description": "Phòng thực hành robot, thiết bị đo kiểm và hệ thống an toàn.",
                "aliases": ["khu vuc b", "khu b", "phong thi nghiem", "phong lab", "lab", "thi nghiem", "laboratory", "room b", "b"]
            },
            {
                "id": "C",
                "name": "Văn phòng",
                "name_en": "Office",
                "color_bgr": [215, 160, 230],       # Soft lavender/purple
                "color_hex": "#E6A0D7",
                "bounds": [0.15, 5.8, -4.4, -0.8],
                "center": [3.2, -2.6],
                "waypoint": [2.2, -2.0, 0.0],
                "description": "Khu vực bàn làm việc quản lý, máy tính văn phòng và tài liệu hành chính.",
                "aliases": ["khu vuc c", "khu c", "van phong", "phong lam viec", "office", "room c", "c"]
            },
            {
                "id": "D",
                "name": "Thư viện",
                "name_en": "Library",
                "color_bgr": [160, 225, 170],       # Soft green
                "color_hex": "#AAD5A0",
                "bounds": [-5.8, -0.15, -4.4, -0.8],
                "center": [-3.2, -2.6],
                "waypoint": [-2.2, -2.0, 0.0],
                "description": "Không gian yên tĩnh đọc sách, cây xanh trang trí và bàn đọc cá nhân.",
                "aliases": ["khu vuc d", "khu d", "thu vien", "phong doc", "library", "reading room", "room d", "d"]
            },
            {
                "id": "E",
                "name": "Hành lang & Sảnh chờ",
                "name_en": "Corridor & Lobby",
                "color_bgr": [200, 200, 245],       # Soft peach/salmon
                "color_hex": "#F5C8C8",
                "bounds": [-5.8, 5.8, -0.8, 0.8],
                "center": [0.0, 0.0],
                "waypoint": [0.0, 0.0, 0.0],
                "description": "Trục giao thông chính kết nối 4 phòng chức năng, sảnh đón và bãi đỗ xe đạp.",
                "aliases": ["khu vuc e", "khu e", "hanh lang", "sanh", "sanh cho", "corridor", "lobby", "hallway", "room e", "e"]
            }
        ]
    }

    # Save zones.yaml and zones.json
    with open(config_dir / 'zones.yaml', 'w', encoding='utf-8') as f:
        yaml.dump(zones_data, f, sort_keys=False, allow_unicode=True)
    with open(config_dir / 'zones.json', 'w', encoding='utf-8') as f:
        json.dump(zones_data, f, indent=2, ensure_ascii=False)
    with open(maps_dir / 'segmented_zones.json', 'w', encoding='utf-8') as f:
        json.dump(zones_data, f, indent=2, ensure_ascii=False)
    print(f"[+] Saved Zones config to {config_dir / 'zones.yaml'}")

    # 6. Generate High-Quality Visual Segmented Map with Printed Names & Colors
    # Create high-res RGB canvas (scale 3x: 840 x 660 px)
    scale = 3
    vis_h, vis_w = PX_H * scale, PX_W * scale
    vis_img = np.full((vis_h, vis_w, 3), 245, dtype=np.uint8)

    # Colorize zones
    for zone in zones_data["zones"]:
        xmin, xmax, ymin, ymax = zone["bounds"]
        c1, r1 = world_to_px(xmin, ymax)
        c2, r2 = world_to_px(xmax, ymin)
        sc1, sr1 = c1 * scale, r1 * scale
        sc2, sr2 = c2 * scale, r2 * scale
        color = zone["color_bgr"]
        cv2.rectangle(vis_img, (min(sc1, sc2), min(sr1, sr2)), (max(sc1, sc2), max(sr1, sr2)), color, -1)

    # Overlay walls and obstacles in dark charcoal
    for r in range(PX_H):
        for c in range(PX_W):
            if grid[r, c] == 0:
                cv2.rectangle(vis_img, (c*scale, r*scale), ((c+1)*scale, (r+1)*scale), (30, 30, 35), -1)

    # Draw Zone Labels and Waypoints prominently
    for zone in zones_data["zones"]:
        wx, wy, _ = zone["waypoint"]
        wc, wr = world_to_px(wx, wy)
        sc, sr = wc * scale, wr * scale

        # Draw Goal Waypoint Marker: Target circle with crosshairs
        cv2.circle(vis_img, (sc, sr), 14, (0, 0, 180), 2)
        cv2.circle(vis_img, (sc, sr), 4, (0, 0, 200), -1)
        cv2.line(vis_img, (sc - 18, sr), (sc + 18, sr), (0, 0, 180), 2)
        cv2.line(vis_img, (sc, sr - 18), (sc, sr + 18), (0, 0, 180), 2)

        # Coordinate badge
        coord_txt = f"({wx:+.1f}, {wy:+.1f})"
        cv2.putText(vis_img, coord_txt, (sc - 30, sr + 28), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (40, 40, 40), 1, cv2.LINE_AA)

        # Main Label Banner
        title_txt = f"[{zone['id']}] {zone['name_en'].upper()}"
        sub_txt = zone['name']

        # Determine label position based on room location
        lbl_y = sr - 40 if wy > 0 else sr + 55
        if zone['id'] == 'E':
            lbl_y = sr - 25

        # Measure text box
        (tw, th), _ = cv2.getTextSize(title_txt, cv2.FONT_HERSHEY_SIMPLEX, 0.65, 2)
        (stw, sth), _ = cv2.getTextSize(sub_txt, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
        bw = max(tw, stw) + 20
        bh = th + sth + 18
        bx1 = sc - bw // 2
        by1 = lbl_y - th - 6
        bx2 = bx1 + bw
        by2 = by1 + bh

        # Semi-transparent background banner
        cv2.rectangle(vis_img, (bx1, by1), (bx2, by2), (255, 255, 255), -1)
        cv2.rectangle(vis_img, (bx1, by1), (bx2, by2), (40, 40, 50), 2)

        # Text in banner
        cv2.putText(vis_img, title_txt, (sc - tw // 2, by1 + th + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (10, 10, 20), 2, cv2.LINE_AA)
        cv2.putText(vis_img, sub_txt, (sc - stw // 2, by1 + th + sth + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (30, 30, 120), 1, cv2.LINE_AA)

    # Title Banner at Top
    cv2.rectangle(vis_img, (0, 0), (vis_w, 36), (30, 35, 45), -1)
    title_banner = "SMART CAMPUS - 5 SEMANTIC ZONES (BAI THUC HANH 7)"
    (tbw, tbh), _ = cv2.getTextSize(title_banner, cv2.FONT_HERSHEY_SIMPLEX, 0.65, 2)
    cv2.putText(vis_img, title_banner, ((vis_w - tbw) // 2, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2, cv2.LINE_AA)

    segmented_png_path = maps_dir / 'segmented_map.png'
    cv2.imwrite(str(segmented_png_path), vis_img)
    print(f"[+] Saved Segmented Visualization: {segmented_png_path}")

    # 7. Generate Ignition Gazebo SDF World File (worlds/smart_campus.sdf)
    sdf_lines = [
        '<?xml version="1.0" encoding="utf-8"?>',
        '<sdf version="1.8">',
        '  <world name="smart_campus">',
        '    <physics name="default_physics" type="ignored">',
        '      <max_step_size>0.003</max_step_size>',
        '      <real_time_factor>1.0</real_time_factor>',
        '    </physics>',
        '    <plugin name="ignition::gazebo::systems::Physics" filename="ignition-gazebo-physics-system" />',
        '    <plugin name="ignition::gazebo::systems::UserCommands" filename="ignition-gazebo-user-commands-system" />',
        '    <plugin name="ignition::gazebo::systems::SceneBroadcaster" filename="ignition-gazebo-scene-broadcaster-system" />',
        '    <plugin name="ignition::gazebo::systems::Contact" filename="ignition-gazebo-contact-system" />',
        '    <plugin name="ignition::gazebo::systems::Imu" filename="ignition-gazebo-imu-system" />',
        '    <gravity>0 0 -9.8</gravity>',
        '    <scene>',
        '      <ambient>0.7 0.7 0.7 1.0</ambient>',
        '      <background>0.78 0.84 0.90 1.0</background>',
        '    </scene>',
        '    <light name="sun" type="directional">',
        '      <pose>0 0 10 0 0 0</pose>',
        '      <diffuse>0.9 0.9 0.9 1</diffuse>',
        '      <specular>0.2 0.2 0.2 1</specular>',
        '      <direction>-0.4 0.2 -0.9</direction>',
        '      <cast_shadows>true</cast_shadows>',
        '    </light>',
        '    <!-- Ground plane floor -->',
        '    <model name="ground">',
        '      <static>true</static>',
        '      <pose>0 0 -0.05 0 0 0</pose>',
        '      <link name="ground_link">',
        '        <collision name="collision">',
        '          <geometry><box><size>14.0 11.0 0.1</size></box></geometry>',
        '        </collision>',
        '        <visual name="visual">',
        '          <geometry><box><size>14.0 11.0 0.1</size></box></geometry>',
        '          <material>',
        '            <ambient>0.88 0.88 0.90 1</ambient>',
        '            <diffuse>0.88 0.88 0.90 1</diffuse>',
        '          </material>',
        '        </visual>',
        '      </link>',
        '    </model>'
    ]

    # Add walls
    for name, wx, wy, dx, dy, color in walls:
        sdf_lines += [
            f'    <model name="{name}">',
            '      <static>true</static>',
            f'      <pose>{wx:.3f} {wy:.3f} 0.5 0 0 0</pose>',
            '      <link name="link">',
            '        <collision name="collision">',
            f'          <geometry><box><size>{dx:.3f} {dy:.3f} 1.0</size></box></geometry>',
            '        </collision>',
            '        <visual name="visual">',
            f'          <geometry><box><size>{dx:.3f} {dy:.3f} 1.0</size></box></geometry>',
            '          <material>',
            f'            <ambient>{color[0]} {color[1]} {color[2]} {color[3]}</ambient>',
            f'            <diffuse>{color[0]} {color[1]} {color[2]} {color[3]}</diffuse>',
            '          </material>',
            '        </visual>',
            '      </link>',
            '    </model>'
        ]

    # Add realistic objects
    for obj in objects:
        name = obj["name"]
        asset = obj["asset"]
        ox, oy, oz, roll, pitch, yaw = obj["pose"]
        c_type = obj["collision_type"]
        c_size = obj["collision_size"]

        sdf_lines += [
            f'    <model name="{name}">',
            '      <static>true</static>',
            f'      <pose>{ox:.3f} {oy:.3f} {oz:.3f} {roll:.4f} {pitch:.4f} {yaw:.4f}</pose>',
            '      <link name="link">'
        ]

        # Visual representation: Realistic Table (Top + 4 Legs)
        if asset == "furniture":
            dx, dy, dz = c_size
            color = obj.get("color_rgba", (0.5, 0.4, 0.3, 1.0))
            top_th = 0.04
            leg_r = 0.03
            leg_h = dz - top_th
            lx = dx / 2.0 - 0.06
            ly = dy / 2.0 - 0.06
            sdf_lines += [
                '        <!-- Realistic Table: Wooden Top + 4 Legs -->',
                '        <visual name="table_top">',
                f'          <pose>0 0 {dz/2.0 - top_th/2.0:.3f} 0 0 0</pose>',
                f'          <geometry><box><size>{dx:.3f} {dy:.3f} {top_th:.3f}</size></box></geometry>',
                '          <material>',
                f'            <ambient>{color[0]} {color[1]} {color[2]} {color[3]}</ambient>',
                f'            <diffuse>{color[0]} {color[1]} {color[2]} {color[3]}</diffuse>',
                '          </material>',
                '        </visual>',
                '        <visual name="leg_fl">',
                f'          <pose>{lx:.3f} {ly:.3f} {-top_th/2.0:.3f} 0 0 0</pose>',
                f'          <geometry><cylinder><radius>{leg_r}</radius><length>{leg_h:.3f}</length></cylinder></geometry>',
                '          <material><ambient>0.2 0.2 0.2 1</ambient><diffuse>0.2 0.2 0.2 1</diffuse></material>',
                '        </visual>',
                '        <visual name="leg_fr">',
                f'          <pose>{lx:.3f} {-ly:.3f} {-top_th/2.0:.3f} 0 0 0</pose>',
                f'          <geometry><cylinder><radius>{leg_r}</radius><length>{leg_h:.3f}</length></cylinder></geometry>',
                '          <material><ambient>0.2 0.2 0.2 1</ambient><diffuse>0.2 0.2 0.2 1</diffuse></material>',
                '        </visual>',
                '        <visual name="leg_bl">',
                f'          <pose>{-lx:.3f} {ly:.3f} {-top_th/2.0:.3f} 0 0 0</pose>',
                f'          <geometry><cylinder><radius>{leg_r}</radius><length>{leg_h:.3f}</length></cylinder></geometry>',
                '          <material><ambient>0.2 0.2 0.2 1</ambient><diffuse>0.2 0.2 0.2 1</diffuse></material>',
                '        </visual>',
                '        <visual name="leg_br">',
                f'          <pose>{-lx:.3f} {-ly:.3f} {-top_th/2.0:.3f} 0 0 0</pose>',
                f'          <geometry><cylinder><radius>{leg_r}</radius><length>{leg_h:.3f}</length></cylinder></geometry>',
                '          <material><ambient>0.2 0.2 0.2 1</ambient><diffuse>0.2 0.2 0.2 1</diffuse></material>',
                '        </visual>'
            ]
        elif asset == "chair":
            scale = obj.get("mesh_scale", (0.00817, 0.00817, 0.00817))
            sdf_lines += [
                '        <visual name="visual">',
                '          <geometry>',
                '            <mesh>',
                f'              <uri>model://chair/meshes/Chair.obj</uri>',
                f'              <scale>{scale[0]} {scale[1]} {scale[2]}</scale>',
                '            </mesh>',
                '          </geometry>',
                '        </visual>'
            ]
        elif asset == "person":
            scale = obj.get("mesh_scale", (0.85, 0.85, 0.85))
            sdf_lines += [
                '        <visual name="visual">',
                '          <pose>0 0 0 0 0 0</pose>',
                '          <geometry>',
                '            <mesh>',
                f'              <uri>model://person/meshes/standing.dae</uri>',
                f'              <scale>{scale[0]} {scale[1]} {scale[2]}</scale>',
                '            </mesh>',
                '          </geometry>',
                '        </visual>'
            ]
        elif asset == "potted_plant":
            sdf_lines += [
                '        <!-- Realistic Decorative Potted Plant -->',
                '        <visual name="pot_body">',
                '          <pose>0 0 0.20 0 0 0</pose>',
                '          <geometry><cylinder><radius>0.22</radius><length>0.40</length></cylinder></geometry>',
                '          <material><ambient>0.72 0.38 0.24 1</ambient><diffuse>0.72 0.38 0.24 1</diffuse></material>',
                '        </visual>',
                '        <visual name="pot_rim">',
                '          <pose>0 0 0.40 0 0 0</pose>',
                '          <geometry><cylinder><radius>0.24</radius><length>0.05</length></cylinder></geometry>',
                '          <material><ambient>0.62 0.32 0.18 1</ambient><diffuse>0.62 0.32 0.18 1</diffuse></material>',
                '        </visual>',
                '        <visual name="soil">',
                '          <pose>0 0 0.41 0 0 0</pose>',
                '          <geometry><cylinder><radius>0.20</radius><length>0.04</length></cylinder></geometry>',
                '          <material><ambient>0.18 0.12 0.08 1</ambient><diffuse>0.18 0.12 0.08 1</diffuse></material>',
                '        </visual>',
                '        <visual name="stem">',
                '          <pose>0 0 0.55 0 0 0</pose>',
                '          <geometry><cylinder><radius>0.035</radius><length>0.28</length></cylinder></geometry>',
                '          <material><ambient>0.35 0.25 0.15 1</ambient><diffuse>0.35 0.25 0.15 1</diffuse></material>',
                '        </visual>',
                '        <visual name="foliage_main">',
                '          <pose>0 0 0.85 0 0 0</pose>',
                '          <geometry><sphere><radius>0.35</radius></sphere></geometry>',
                '          <material><ambient>0.15 0.55 0.18 1</ambient><diffuse>0.15 0.55 0.18 1</diffuse></material>',
                '        </visual>',
                '        <visual name="foliage_top">',
                '          <pose>0 0 1.05 0 0 0</pose>',
                '          <geometry><sphere><radius>0.26</radius></sphere></geometry>',
                '          <material><ambient>0.22 0.65 0.25 1</ambient><diffuse>0.22 0.65 0.25 1</diffuse></material>',
                '        </visual>'
            ]
        elif asset == "fire_hydrant":
            scale = obj.get("mesh_scale", (0.85, 0.85, 0.85))
            sdf_lines += [
                '        <visual name="visual">',
                '          <geometry>',
                '            <mesh>',
                f'              <uri>model://fire_hydrant/meshes/fire_hydrant.dae</uri>',
                f'              <scale>{scale[0]} {scale[1]} {scale[2]}</scale>',
                '            </mesh>',
                '          </geometry>',
                '        </visual>'
            ]
        elif asset == "backpack":
            scale = obj.get("mesh_scale", (0.8, 0.8, 0.8))
            sdf_lines += [
                '        <visual name="visual">',
                '          <geometry>',
                '            <mesh>',
                f'              <uri>model://backpack/meshes/Backpack.dae</uri>',
                f'              <scale>{scale[0]} {scale[1]} {scale[2]}</scale>',
                '            </mesh>',
                '          </geometry>',
                '        </visual>'
            ]
        elif asset == "wet_floor_sign":
            sdf_lines += [
                '        <visual name="sign_board">',
                '          <pose>0 0 0.24 0 0 0</pose>',
                '          <geometry><box><size>0.28 0.16 0.48</size></box></geometry>',
                '          <material><ambient>0.98 0.80 0.08 1</ambient><diffuse>0.98 0.80 0.08 1</diffuse></material>',
                '        </visual>',
                '        <visual name="sign_stripe">',
                '          <pose>0 0 0.24 0 0 0</pose>',
                '          <geometry><box><size>0.29 0.17 0.12</size></box></geometry>',
                '          <material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material>',
                '        </visual>'
            ]
        elif asset == "caution_cone":
            sdf_lines += [
                '        <visual name="cone_base">',
                '          <pose>0 0 0.02 0 0 0</pose>',
                '          <geometry><box><size>0.28 0.28 0.04</size></box></geometry>',
                '          <material><ambient>0.96 0.45 0.08 1</ambient><diffuse>0.96 0.45 0.08 1</diffuse></material>',
                '        </visual>',
                '        <visual name="cone_body">',
                '          <pose>0 0 0.23 0 0 0</pose>',
                '          <geometry><cylinder><radius>0.12</radius><length>0.42</length></cylinder></geometry>',
                '          <material><ambient>0.96 0.45 0.08 1</ambient><diffuse>0.96 0.45 0.08 1</diffuse></material>',
                '        </visual>',
                '        <visual name="cone_stripe">',
                '          <pose>0 0 0.25 0 0 0</pose>',
                '          <geometry><cylinder><radius>0.13</radius><length>0.10</length></cylinder></geometry>',
                '          <material><ambient>0.95 0.95 0.95 1</ambient><diffuse>0.95 0.95 0.95 1</diffuse></material>',
                '        </visual>'
            ]
        elif asset == "trash_bin":
            sdf_lines += [
                '        <visual name="bin_body">',
                '          <pose>0 0 0.28 0 0 0</pose>',
                '          <geometry><cylinder><radius>0.16</radius><length>0.56</length></cylinder></geometry>',
                '          <material><ambient>0.15 0.35 0.55 1</ambient><diffuse>0.15 0.35 0.55 1</diffuse></material>',
                '        </visual>',
                '        <visual name="bin_lid">',
                '          <pose>0 0 0.57 0 0 0</pose>',
                '          <geometry><cylinder><radius>0.17</radius><length>0.03</length></cylinder></geometry>',
                '          <material><ambient>0.7 0.7 0.7 1</ambient><diffuse>0.7 0.7 0.7 1</diffuse></material>',
                '        </visual>'
            ]
        elif asset == "parcel_box":
            sdf_lines += [
                '        <visual name="box_body">',
                '          <pose>0 0 0.12 0 0 0</pose>',
                '          <geometry><box><size>0.32 0.28 0.24</size></box></geometry>',
                '          <material><ambient>0.75 0.50 0.25 1</ambient><diffuse>0.75 0.50 0.25 1</diffuse></material>',
                '        </visual>',
                '        <visual name="box_tape">',
                '          <pose>0 0 0.12 0 0 0</pose>',
                '          <geometry><box><size>0.33 0.06 0.245</size></box></geometry>',
                '          <material><ambient>0.88 0.70 0.35 1</ambient><diffuse>0.88 0.70 0.35 1</diffuse></material>',
                '        </visual>'
            ]

        # Accurate collision matching the Occupancy Grid
        if c_type == "box":
            dx, dy, dz = c_size
            sdf_lines += [
                '        <collision name="collision">',
                f'          <pose>0 0 {dz/2.0:.3f} 0 0 0</pose>',
                f'          <geometry><box><size>{dx:.3f} {dy:.3f} {dz:.3f}</size></box></geometry>',
                '        </collision>'
            ]
        elif c_type == "cylinder":
            radius, length = c_size
            sdf_lines += [
                '        <collision name="collision">',
                f'          <pose>0 0 {length/2.0:.3f} 0 0 0</pose>',
                f'          <geometry><cylinder><radius>{radius:.3f}</radius><length>{length:.3f}</length></cylinder></geometry>',
                '        </collision>'
            ]

        sdf_lines += [
            '      </link>',
            '    </model>'
        ]

    sdf_lines += [
        '  </world>',
        '</sdf>',
        ''
    ]

    sdf_path = worlds_dir / 'smart_campus.sdf'
    with open(sdf_path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(sdf_lines))
    print(f"[+] Saved Smart Campus SDF World: {sdf_path}")
    print("[OK] Environment generation complete!")


if __name__ == '__main__':
    generate_environment()
