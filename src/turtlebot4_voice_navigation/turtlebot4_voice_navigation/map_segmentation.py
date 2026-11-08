"""OpenCV-based Map Segmentation and Pixel-to-World Coordinate Transformations.
Implements morphological filtering, distance transform, and watershed segmentation
for ROS 2 Occupancy Grid maps.
"""
from pathlib import Path
from typing import Dict, List, Tuple, Any, Optional
import math
import cv2
import numpy as np
import yaml
import json


def pixel_to_world(col: float, row: float, resolution: float, origin: List[float], image_height: int) -> Tuple[float, float]:
    """Convert pixel coordinates (col, row) to ROS 2 metric map coordinates (x, y).
    In standard OccupancyGrid: row 0 is bottom (y = origin[1]),
    in OpenCV image: row 0 is top.
    Therefore:
        x = origin[0] + col * resolution
        y = origin[1] + (image_height - 1 - row) * resolution
    """
    x = origin[0] + col * resolution
    y = origin[1] + (image_height - 1 - row) * resolution
    return round(float(x), 3), round(float(y), 3)


def world_to_pixel(x: float, y: float, resolution: float, origin: List[float], image_height: int) -> Tuple[int, int]:
    """Convert ROS 2 metric map coordinates (x, y) to image pixel coordinates (col, row)."""
    col = int(round((x - origin[0]) / resolution))
    row = int(round(image_height - 1 - (y - origin[1]) / resolution))
    return col, row


class MapSegmenter:
    """Performs topological and morphological segmentation of a 2D occupancy grid."""

    def __init__(self, map_yaml_path: str):
        self.yaml_path = Path(map_yaml_path).resolve()
        if not self.yaml_path.is_file():
            raise FileNotFoundError(f"Map YAML not found: {self.yaml_path}")

        with open(self.yaml_path, 'r', encoding='utf-8') as f:
            self.meta = yaml.safe_load(f)

        self.resolution = float(self.meta.get('resolution', 0.05))
        self.origin = [float(v) for v in self.meta.get('origin', [0.0, 0.0, 0.0])]
        self.occupied_thresh = float(self.meta.get('occupied_thresh', 0.65))
        self.free_thresh = float(self.meta.get('free_thresh', 0.25))

        image_path = self.yaml_path.parent / self.meta.get('image', 'map.pgm')
        if not image_path.is_file():
            raise FileNotFoundError(f"Map image not found: {image_path}")

        self.image = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
        if self.image is None:
            raise ValueError(f"Failed to load image: {image_path}")

        self.height, self.width = self.image.shape

    def get_free_space_mask(self) -> np.ndarray:
        """Extract binary mask of navigable free space.
        In standard ROS 8-bit PGM:
            Occupied: pixel < (1 - occupied_thresh) * 255 (dark, near 0)
            Free: pixel > (1 - free_thresh) * 255 (light, near 254)
            Unknown: around 205
        """
        threshold_val = int((1.0 - self.free_thresh) * 255.0)
        _, free_mask = cv2.threshold(self.image, threshold_val, 255, cv2.THRESH_BINARY)
        return free_mask

    def segment_watershed(self, min_room_area_m2: float = 4.0) -> Tuple[np.ndarray, List[Dict[str, Any]]]:
        """Segments free space into rooms using morphological operations,
        distance transform, and the Watershed algorithm.
        """
        free_mask = self.get_free_space_mask()

        # 1. Clean small speckles with morphological opening
        kernel_clean = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        clean_free = cv2.morphologyEx(free_mask, cv2.MORPH_OPEN, kernel_clean)

        # 2. Distance transform: pixel intensity = distance to closest obstacle/wall
        dist_transform = cv2.distanceTransform(clean_free, cv2.DIST_L2, 5)

        # 3. Find sure foreground room centers (peaks in distance transform)
        # Dilate doorways to separate connected rooms
        max_dist = dist_transform.max()
        if max_dist <= 0:
            return np.zeros_like(free_mask), []

        # Local maxima as seeds: threshold at 45% of max room clearance
        _, sure_fg = cv2.threshold(dist_transform, 0.45 * max_dist, 255, cv2.THRESH_BINARY)
        sure_fg = np.uint8(sure_fg)

        # 4. Unknown boundary region
        kernel_dilate = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        sure_bg = cv2.dilate(clean_free, kernel_dilate, iterations=1)
        unknown = cv2.subtract(sure_bg, sure_fg)

        # 5. Marker labeling for Watershed
        num_markers, markers = cv2.connectedComponents(sure_fg)
        markers = markers + 1  # background is 1
        markers[unknown == 255] = 0  # unknown region for watershed to decide

        # Run Watershed
        color_img = cv2.cvtColor(self.image, cv2.COLOR_GRAY2BGR)
        cv2.watershed(color_img, markers)

        min_pixel_area = int((min_room_area_m2 / (self.resolution ** 2)))

        segments: List[Dict[str, Any]] = []
        raw_labels = np.unique(markers)

        # Filter and extract region data
        valid_regions = []
        for label_val in raw_labels:
            if label_val <= 1:  # 0: boundary, 1: background
                continue
            region_mask = np.uint8(markers == label_val)
            area_px = int(np.sum(region_mask))
            if area_px < min_pixel_area:
                continue

            # Centroid
            moments = cv2.moments(region_mask)
            if moments["m00"] == 0:
                continue
            cx_px = moments["m10"] / moments["m00"]
            cy_px = moments["m01"] / moments["m00"]
            cx_m, cy_m = pixel_to_world(cx_px, cy_px, self.resolution, self.origin, self.height)

            # Max clearance point in this region (best navigation target)
            masked_dist = dist_transform.copy()
            masked_dist[region_mask == 0] = 0
            min_val, peak_val, min_loc, peak_loc = cv2.minMaxLoc(masked_dist)
            peak_x_m, peak_y_m = pixel_to_world(peak_loc[0], peak_loc[1], self.resolution, self.origin, self.height)

            # Contours
            contours, _ = cv2.findContours(region_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            poly_points = []
            if contours:
                largest_c = max(contours, key=cv2.contourArea)
                # Simplify polygon
                epsilon = 0.015 * cv2.arcLength(largest_c, True)
                approx = cv2.approxPolyDP(largest_c, epsilon, True)
                for pt in approx:
                    px_c, px_r = pt[0][0], pt[0][1]
                    wx, wy = pixel_to_world(px_c, px_r, self.resolution, self.origin, self.height)
                    poly_points.append([wx, wy])

            valid_regions.append({
                'label': int(label_val),
                'area_m2': round(area_px * (self.resolution ** 2), 2),
                'area_pixels': area_px,
                'centroid_m': [cx_m, cy_m],
                'target_pose': {'x': peak_x_m, 'y': peak_y_m, 'yaw': 0.0},
                'clearance_m': round(float(peak_val) * self.resolution, 2),
                'polygon': poly_points,
                'mask': region_mask
            })

        # Sort regions geographically:
        # Central/corridor area (|y - center_y| < 0.9m) -> Zone E
        # Top-Left -> Zone A, Top-Right -> Zone B
        # Bottom-Right -> Zone C, Bottom-Left -> Zone D
        center_x = self.origin[0] + (self.width * self.resolution) / 2.0
        center_y = self.origin[1] + (self.height * self.resolution) / 2.0

        for r in valid_regions:
            x, y = r['centroid_m']
            if abs(y - center_y) <= 0.85 and (x > -5.0 and x < 5.0):
                r['quadrant'] = 'CEN'
            elif x < center_x and y >= center_y:
                r['quadrant'] = 'TL'
            elif x >= center_x and y >= center_y:
                r['quadrant'] = 'TR'
            elif x >= center_x and y < center_y:
                r['quadrant'] = 'BR'
            else:
                r['quadrant'] = 'BL'

        # Map to Zone IDs A, B, C, D, E
        id_map = {'TL': 'A', 'TR': 'B', 'BR': 'C', 'BL': 'D', 'CEN': 'E'}
        names = {
            'A': ('Khu vực A (Phòng học)', 'Classroom'),
            'B': ('Khu vực B (Phòng thí nghiệm)', 'Laboratory'),
            'C': ('Khu vực C (Văn phòng)', 'Office'),
            'D': ('Khu vực D (Thư viện)', 'Library'),
            'E': ('Khu vực E (Hành lang & Sảnh)', 'Corridor & Lobby')
        }

        # Fallback alphabetical assignment if quadrants conflict
        assigned_ids = set()
        for r in valid_regions:
            cand_id = id_map.get(r['quadrant'], 'A')
            if cand_id in assigned_ids:
                for alt in ['A', 'B', 'C', 'D', 'E']:
                    if alt not in assigned_ids:
                        cand_id = alt
                        break
            assigned_ids.add(cand_id)
            r['id'] = cand_id
            r['name'] = names.get(cand_id, (f"Khu vực {cand_id}", cand_id))[0]
            r['english_name'] = names.get(cand_id, (cand_id, cand_id))[1]
            del r['mask']  # non-serializable
            segments.append(r)

        # Sort segments alphabetically by ID
        segments.sort(key=lambda s: s['id'])
        return markers, segments

    def export_visualization(self, output_png_path: str, zones_dict: Optional[Dict[str, Any]] = None):
        """Draws colored segmentation overlay with labels and target poses."""
        color_map = {
            'A': (140, 210, 245),   # Soft warm yellow/orange
            'B': (235, 205, 140),   # Soft cyan/teal
            'C': (215, 160, 230),   # Soft lavender
            'D': (160, 225, 170),   # Soft green
            'E': (200, 200, 245)    # Soft salmon/peach
        }

        vis = cv2.cvtColor(self.image, cv2.COLOR_GRAY2BGR)
        overlay = vis.copy()

        if zones_dict:
            for zid, info in zones_dict.items():
                col = color_map.get(zid, (200, 200, 200))
                poly = info.get('polygon', [])
                if len(poly) >= 3:
                    pts = [world_to_pixel(p[0], p[1], self.resolution, self.origin, self.height) for p in poly]
                    cv2.fillPoly(overlay, [np.array(pts, dtype=np.int32)], col)
                    cv2.polylines(vis, [np.array(pts, dtype=np.int32)], True, (30, 30, 30), 2)

                tpose = info.get('target_pose', {})
                tx, ty = tpose.get('x', 0.0), tpose.get('y', 0.0)
                tc, tr = world_to_pixel(tx, ty, self.resolution, self.origin, self.height)
                cv2.circle(vis, (tc, tr), 6, (0, 0, 255), -1)
                cv2.circle(vis, (tc, tr), 8, (255, 255, 255), 1)

                name_short = info.get('name', f'Zone {zid}')
                cv2.putText(vis, f"[{zid}] {name_short}", (tc - 40, tr - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 0, 0), 2, cv2.LINE_AA)
                cv2.putText(vis, f"[{zid}] {name_short}", (tc - 40, tr - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255, 255, 255), 1, cv2.LINE_AA)

        cv2.addWeighted(overlay, 0.35, vis, 0.65, 0, vis)
        # Restore solid obstacles
        vis[self.image == 0] = [30, 30, 30]

        cv2.imwrite(str(output_png_path), vis)
        return output_png_path
