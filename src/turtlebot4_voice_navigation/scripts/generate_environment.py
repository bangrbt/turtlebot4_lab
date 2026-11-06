#!/usr/bin/env python3
"""Generates the Smart Campus SDF world, ground truth OccupancyGrid map.pgm/yaml,
and semantic zones for TurtleBot 4 Lab 07 Voice Navigation.
"""
from pathlib import Path
import math
import cv2
import numpy as np
import yaml
import json

MAP_WIDTH_M = 14.0
MAP_HEIGHT_M = 11.0
RESOLUTION = 0.05  # 5cm per pixel
ORIGIN_X = -7.0
ORIGIN_Y = -5.5

PX_W = int(round(MAP_WIDTH_M / RESOLUTION))   # 280
PX_H = int(round(MAP_HEIGHT_M / RESOLUTION))  # 220

def world_to_px(x, y):
    """Convert metric (x, y) in map frame to pixel (col, row).
    Standard ROS OccupancyGrid: row 0 is bottom (y = ORIGIN_Y),
    in image coordinates row 0 is top (y = ORIGIN_Y + MAP_HEIGHT_M).
    So row = (ORIGIN_Y + MAP_HEIGHT_M - y) / RESOLUTION.
    """
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

    worlds_dir.mkdir(parents=True, exist_ok=True)
    maps_dir.mkdir(parents=True, exist_ok=True)
    config_dir.mkdir(parents=True, exist_ok=True)

    # 1. Prepare Occupancy Grid: default 205 (unknown/outside), 254 (free), 0 (occupied wall/obstacle)
    grid = np.full((PX_H, PX_W), 205, dtype=np.uint8)

    # Interior room boundary: x in [-5.9, 5.9], y in [-4.4, 4.4]
    c1, r1 = world_to_px(-5.925, 4.425)
    c2, r2 = world_to_px(5.925, -4.425)
    grid[min(r1, r2):max(r1, r2)+1, min(c1, c2):max(c1, c2)+1] = 254

    # Structure walls list: (name, x, y, dx, dy, color_rgba)
    walls = [
        # Outer boundary walls (thickness 0.15m)
        ("north_wall", 0.0, 4.5, 12.15, 0.15, (0.2, 0.3, 0.4, 1.0)),
        ("south_wall", 0.0, -4.5, 12.15, 0.15, (0.2, 0.3, 0.4, 1.0)),
        ("west_wall", -6.0, 0.0, 0.15, 9.15, (0.2, 0.3, 0.4, 1.0)),
        ("east_wall", 6.0, 0.0, 0.15, 9.15, (0.2, 0.3, 0.4, 1.0)),

        # Internal dividers
        ("divider_north_v", 0.0, 2.5, 0.15, 4.0, (0.25, 0.35, 0.45, 1.0)),
        ("divider_south_v", 0.0, -2.5, 0.15, 4.0, (0.25, 0.35, 0.45, 1.0)),

        # Corridor North wall (y = 0.5) with doorways at x = -3.0 and x = +3.0
        ("corridor_n_left", -4.85, 0.5, 2.3, 0.15, (0.25, 0.35, 0.45, 1.0)),
        ("corridor_n_mid_l", -1.15, 0.5, 2.3, 0.15, (0.25, 0.35, 0.45, 1.0)),
        ("corridor_n_mid_r", 1.15, 0.5, 2.3, 0.15, (0.25, 0.35, 0.45, 1.0)),
        ("corridor_n_right", 4.85, 0.5, 2.3, 0.15, (0.25, 0.35, 0.45, 1.0)),

        # Corridor South wall (y = -0.5) with doorway at x = +3.0 (West side open to lobby)
        ("corridor_s_mid_r", 1.15, -0.5, 2.3, 0.15, (0.25, 0.35, 0.45, 1.0)),
        ("corridor_s_right", 4.85, -0.5, 2.3, 0.15, (0.25, 0.35, 0.45, 1.0)),
    ]

    # Obstacles: (name, type, x, y, size/radius, height, color_rgba)
    obstacles = [
        # Zone A - Classroom
        ("podium_a", "box", -4.6, 3.4, (0.8, 1.4), 0.85, (0.55, 0.35, 0.2, 1.0)),
        ("desk_a1", "box", -3.0, 3.2, (0.7, 1.4), 0.75, (0.7, 0.5, 0.3, 1.0)),
        ("desk_a2", "box", -1.4, 3.2, (0.7, 1.4), 0.75, (0.7, 0.5, 0.3, 1.0)),
        ("column_a", "cylinder", -5.0, 1.6, 0.22, 1.0, (0.3, 0.4, 0.5, 1.0)),

        # Zone B - Laboratory
        ("lab_bench_b1", "box", 4.6, 3.4, (0.8, 1.5), 0.85, (0.2, 0.4, 0.55, 1.0)),
        ("console_b", "box", 1.4, 3.4, (0.8, 1.2), 1.0, (0.25, 0.3, 0.35, 1.0)),
        ("island_b", "box", 4.6, 1.6, (0.8, 1.2), 0.85, (0.2, 0.4, 0.55, 1.0)),
        ("column_b", "cylinder", 1.4, 1.6, 0.22, 1.0, (0.3, 0.4, 0.5, 1.0)),

        # Zone C - Office
        ("exec_desk_c", "box", 4.6, -2.8, (0.8, 1.6), 0.8, (0.5, 0.3, 0.15, 1.0)),
        ("cabinet_c", "box", 1.4, -3.5, (0.6, 1.4), 1.2, (0.35, 0.35, 0.35, 1.0)),
        ("plant_c", "cylinder", 5.2, -1.2, 0.25, 0.7, (0.15, 0.55, 0.2, 1.0)),
        ("column_c", "cylinder", 4.6, -1.2, 0.22, 1.0, (0.3, 0.4, 0.5, 1.0)),

        # Zone D - Lobby & Corridor
        ("bench_d", "box", -4.6, -3.2, (0.7, 1.6), 0.5, (0.45, 0.35, 0.25, 1.0)),
        ("column_d1", "cylinder", -1.5, -2.5, 0.25, 1.0, (0.3, 0.4, 0.5, 1.0)),
        ("plant_d1", "cylinder", -5.2, -1.2, 0.25, 0.7, (0.15, 0.55, 0.2, 1.0)),
        ("plant_d2", "cylinder", -5.2, 0.0, 0.25, 0.7, (0.15, 0.55, 0.2, 1.0)),
        ("notice_board_d", "box", 0.0, -0.6, (0.8, 0.18), 1.1, (0.6, 0.4, 0.2, 1.0)),
    ]

    # Draw walls on grid (0 = occupied)
    for name, x, y, dx, dy, _ in walls:
        c1, r1 = world_to_px(x - dx/2, y + dy/2)
        c2, r2 = world_to_px(x + dx/2, y - dy/2)
        col_min, col_max = max(0, min(c1, c2)), min(PX_W - 1, max(c1, c2))
        row_min, row_max = max(0, min(r1, r2)), min(PX_H - 1, max(r1, r2))
        grid[row_min:row_max+1, col_min:col_max+1] = 0

    # Draw obstacles on grid (0 = occupied)
    for item in obstacles:
        name, otype, x, y = item[0], item[1], item[2], item[3]
        if otype == "box":
            dx, dy = item[4]
            c1, r1 = world_to_px(x - dx/2, y + dy/2)
            c2, r2 = world_to_px(x + dx/2, y - dy/2)
            col_min, col_max = max(0, min(c1, c2)), min(PX_W - 1, max(c1, c2))
            row_min, row_max = max(0, min(r1, r2)), min(PX_H - 1, max(r1, r2))
            grid[row_min:row_max+1, col_min:col_max+1] = 0
        elif otype == "cylinder":
            radius = item[4]
            center_c, center_r = world_to_px(x, y)
            rad_px = int(round(radius / RESOLUTION))
            cv2.circle(grid, (center_c, center_r), rad_px, 0, -1)

    # Save map.pgm and map.yaml
    cv2.imwrite(str(maps_dir / 'map.pgm'), grid)
    map_meta = {
        'image': 'map.pgm',
        'resolution': RESOLUTION,
        'origin': [ORIGIN_X, ORIGIN_Y, 0.0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.25,
        'mode': 'trinary'
    }
    with open(maps_dir / 'map.yaml', 'w') as f:
        yaml.dump(map_meta, f, sort_keys=False)

    print(f"[OK] Generated {maps_dir / 'map.pgm'} ({PX_W}x{PX_H}) and map.yaml")

    # 2. Generate Semantic Zones
    zones = {
        'A': {
            'id': 'A',
            'code': 'ZONE_A',
            'name': 'Khu vực A (Phòng học)',
            'english_name': 'Classroom (Zone A)',
            'aliases': [
                'a', 'khu a', 'khu vực a', 'phòng a', 'phòng học', 'lớp học', 'phong hoc', 'lop hoc',
                'zone a', 'area a', 'room a', 'classroom'
            ],
            'description': 'Phòng học lý thuyết và thuyết trình, góc Tây Bắc của toà nhà.',
            'target_pose': {
                'x': -3.0,
                'y': 2.0,
                'yaw': 0.0
            },
            'polygon': [
                [-5.9, 0.6],
                [-0.1, 0.6],
                [-0.1, 4.4],
                [-5.9, 4.4]
            ],
            'color_rgb': [0, 220, 100],   # Emerald Green
            'marker_z': 1.0
        },
        'B': {
            'id': 'B',
            'code': 'ZONE_B',
            'name': 'Khu vực B (Phòng thí nghiệm)',
            'english_name': 'Laboratory (Zone B)',
            'aliases': [
                'b', 'khu b', 'khu vực b', 'phòng b', 'phòng thí nghiệm', 'phòng lab', 'phong thi nghiem', 'lab',
                'zone b', 'area b', 'room b', 'laboratory', 'research lab'
            ],
            'description': 'Phòng thí nghiệm nghiên cứu robotics và cảm biến, góc Đông Bắc toà nhà.',
            'target_pose': {
                'x': 2.8,
                'y': 2.0,
                'yaw': 1.5708
            },
            'polygon': [
                [0.1, 0.6],
                [5.9, 0.6],
                [5.9, 4.4],
                [0.1, 4.4]
            ],
            'color_rgb': [0, 180, 255],   # Sky Blue
            'marker_z': 1.0
        },
        'C': {
            'id': 'C',
            'code': 'ZONE_C',
            'name': 'Khu vực C (Văn phòng)',
            'english_name': 'Office (Zone C)',
            'aliases': [
                'c', 'khu c', 'khu vực c', 'phòng c', 'văn phòng', 'phòng làm việc', 'van phong', 'phong lam viec',
                'zone c', 'area c', 'room c', 'office', 'study'
            ],
            'description': 'Văn phòng làm việc giảng viên và nghiên cứu sinh, góc Đông Nam toà nhà.',
            'target_pose': {
                'x': 2.8,
                'y': -2.0,
                'yaw': -1.5708
            },
            'polygon': [
                [0.1, -4.4],
                [5.9, -4.4],
                [5.9, -0.6],
                [0.1, -0.6]
            ],
            'color_rgb': [220, 80, 220],  # Magenta
            'marker_z': 1.0
        },
        'D': {
            'id': 'D',
            'code': 'ZONE_D',
            'name': 'Khu vực D (Hành lang & Sảnh)',
            'english_name': 'Corridor & Lobby (Zone D)',
            'aliases': [
                'd', 'khu d', 'khu vực d', 'hành lang', 'sảnh', 'sảnh chính', 'khu tiếp đón', 'hanh lang', 'sanh',
                'zone d', 'area d', 'corridor', 'hallway', 'lobby', 'reception'
            ],
            'description': 'Hành lang trung tâm kết nối các phòng và sảnh chờ tiếp đón phía Tây Nam.',
            'target_pose': {
                'x': -3.0,
                'y': -1.8,
                'yaw': 0.0
            },
            'polygon': [
                [-5.9, -4.4],
                [-0.1, -4.4],
                [-0.1, 0.5],
                [5.9, 0.5],
                [5.9, -0.5],
                [-0.1, -0.5],
                [-5.9, -0.5]
            ],
            'color_rgb': [255, 160, 40],  # Amber / Orange
            'marker_z': 1.0
        }
    }

    # Save zones.yaml and zones.json
    with open(config_dir / 'zones.yaml', 'w', encoding='utf-8') as f:
        yaml.dump(zones, f, allow_unicode=True, sort_keys=False)
    with open(config_dir / 'zones.json', 'w', encoding='utf-8') as f:
        json.dump(zones, f, ensure_ascii=False, indent=2)

    print(f"[OK] Generated {config_dir / 'zones.yaml'} and zones.json")

    # 3. Generate segmented visual map image (segmented_map.png)
    vis = cv2.cvtColor(grid, cv2.COLOR_GRAY2BGR)
    overlay = vis.copy()

    for zid, zinfo in zones.items():
        color = zinfo['color_rgb'][::-1]  # BGR
        poly_pts = [world_to_px(pt[0], pt[1]) for pt in zinfo['polygon']]
        cv2.fillPoly(overlay, [np.array(poly_pts, dtype=np.int32)], color)

    # Blend overlay with original map (alpha = 0.35)
    cv2.addWeighted(overlay, 0.35, vis, 0.65, 0, vis)

    # Redraw walls and obstacles crisply over the blend
    vis[grid == 0] = [30, 30, 30]

    # Draw zone boundaries and text annotations
    for zid, zinfo in zones.items():
        poly_pts = [world_to_px(pt[0], pt[1]) for pt in zinfo['polygon']]
        cv2.polylines(vis, [np.array(poly_pts, dtype=np.int32)], True, (20, 20, 20), 2)

        # Draw target waypoint star/circle
        tx, ty = zinfo['target_pose']['x'], zinfo['target_pose']['y']
        tc, tr = world_to_px(tx, ty)
        cv2.circle(vis, (tc, tr), 6, (0, 0, 255), -1)
        cv2.circle(vis, (tc, tr), 8, (255, 255, 255), 1)

        # Label box
        label_text = f"Zone {zid}: {zinfo['name'].split('(')[1].replace(')', '')}"
        cv2.putText(vis, label_text, (tc - 45, tr - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (10, 10, 10), 2, cv2.LINE_AA)
        cv2.putText(vis, label_text, (tc - 45, tr - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255, 255, 255), 1, cv2.LINE_AA)

    # Draw start position (0, 0)
    sc, sr = world_to_px(0.0, 0.0)
    cv2.circle(vis, (sc, sr), 6, (255, 50, 50), -1)
    cv2.putText(vis, "START (0,0)", (sc - 35, sr + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (255, 255, 255), 1, cv2.LINE_AA)

    cv2.imwrite(str(maps_dir / 'segmented_map.png'), vis)
    print(f"[OK] Generated {maps_dir / 'segmented_map.png'}")

    # 4. Generate Gazebo SDF world (smart_campus.sdf)
    sdf_content = generate_sdf(walls, obstacles)
    with open(worlds_dir / 'smart_campus.sdf', 'w', encoding='utf-8') as f:
        f.write(sdf_content)
    print(f"[OK] Generated {worlds_dir / 'smart_campus.sdf'}")

def generate_sdf(walls, obstacles):
    models_xml = []

    # Floor ground
    models_xml.append('''    <model name="ground">
      <static>true</static>
      <pose>0 0 -0.05 0 0 0</pose>
      <link name="ground_link">
        <collision name="collision">
          <geometry><box><size>14.0 11.0 0.1</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>14.0 11.0 0.1</size></box></geometry>
          <material>
            <ambient>0.88 0.88 0.90 1</ambient>
            <diffuse>0.88 0.88 0.90 1</diffuse>
          </material>
        </visual>
      </link>
    </model>''')

    # Walls
    for name, x, y, dx, dy, col in walls:
        r, g, b, a = col
        models_xml.append(f'''    <model name="{name}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{dx:.3f} {dy:.3f} 1.0</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{dx:.3f} {dy:.3f} 1.0</size></box></geometry>
          <material>
            <ambient>{r} {g} {b} {a}</ambient>
            <diffuse>{r} {g} {b} {a}</diffuse>
          </material>
        </visual>
      </link>
    </model>''')

    # Obstacles
    for item in obstacles:
        name, otype, x, y = item[0], item[1], item[2], item[3]
        if otype == "box":
            (dx, dy), h, col = item[4], item[5], item[6]
            r, g, b, a = col
            z = h / 2.0
            models_xml.append(f'''    <model name="{name}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{dx:.3f} {dy:.3f} {h:.3f}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{dx:.3f} {dy:.3f} {h:.3f}</size></box></geometry>
          <material>
            <ambient>{r} {g} {b} {a}</ambient>
            <diffuse>{r} {g} {b} {a}</diffuse>
          </material>
        </visual>
      </link>
    </model>''')
        elif otype == "cylinder":
            radius, h, col = item[4], item[5], item[6]
            r, g, b, a = col
            z = h / 2.0
            models_xml.append(f'''    <model name="{name}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><cylinder><radius>{radius:.3f}</radius><length>{h:.3f}</length></cylinder></geometry>
        </collision>
        <visual name="visual">
          <geometry><cylinder><radius>{radius:.3f}</radius><length>{h:.3f}</length></cylinder></geometry>
          <material>
            <ambient>{r} {g} {b} {a}</ambient>
            <diffuse>{r} {g} {b} {a}</diffuse>
          </material>
        </visual>
      </link>
    </model>''')

    body = "\n".join(models_xml)
    return f"""<?xml version="1.0" encoding="utf-8"?>
<sdf version="1.8">
  <world name="smart_campus">
    <physics name="default_physics" type="ignored">
      <max_step_size>0.003</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin name="ignition::gazebo::systems::Physics" filename="ignition-gazebo-physics-system" />
    <plugin name="ignition::gazebo::systems::UserCommands" filename="ignition-gazebo-user-commands-system" />
    <plugin name="ignition::gazebo::systems::SceneBroadcaster" filename="ignition-gazebo-scene-broadcaster-system" />
    <plugin name="ignition::gazebo::systems::Contact" filename="ignition-gazebo-contact-system" />
    <plugin name="ignition::gazebo::systems::Imu" filename="ignition-gazebo-imu-system" />
    <plugin name="ignition::gazebo::systems::Sensors" filename="ignition-gazebo-sensors-system">
      <render_engine>ogre2</render_engine>
    </plugin>
    <gravity>0 0 -9.8</gravity>
    <scene>
      <ambient>0.65 0.65 0.65 1.0</ambient>
      <background>0.75 0.82 0.88 1.0</background>
    </scene>
    <light name="sun" type="directional">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.4 0.2 -0.9</direction>
      <cast_shadows>true</cast_shadows>
    </light>
{body}
  </world>
</sdf>
"""

if __name__ == '__main__':
    generate_environment()
