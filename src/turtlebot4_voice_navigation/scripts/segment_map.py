#!/usr/bin/env python3
"""CLI utility to segment a 2D map into semantic zones using OpenCV watershed/morphology.

Usage:
    python3 segment_map.py --map src/turtlebot4_voice_navigation/maps/map.yaml
"""
import argparse
from pathlib import Path
import json
import yaml
from turtlebot4_voice_navigation.map_segmentation import MapSegmenter

def main():
    parser = argparse.ArgumentParser(description="Segment a ROS 2 Occupancy Grid map into semantic zones.")
    parser.add_argument('--map', type=str, default='maps/map.yaml', help='Path to map.yaml')
    parser.add_argument('--output-dir', type=str, default=None, help='Output directory for zones.yaml and segmented_map.png')
    parser.add_argument('--min-area', type=float, default=4.0, help='Minimum zone area in m^2')
    args = parser.parse_args()

    map_path = Path(args.map).resolve()
    out_dir = Path(args.output_dir).resolve() if args.output_dir else map_path.parent

    print(f"[*] Loading map: {map_path}")
    segmenter = MapSegmenter(str(map_path))

    print(f"[*] Map dimensions: {segmenter.width}x{segmenter.height} px, Resolution: {segmenter.resolution} m/px")
    print(f"[*] Running Watershed morphological segmentation (min_area: {args.min_area} m^2)...")
    markers, segments = segmenter.segment_watershed(min_room_area_m2=args.min_area)

    print(f"[+] Detected {len(segments)} distinct zones:")
    zones_dict = {}
    for s in segments:
        zid = s['id']
        zones_dict[zid] = {
            'id': zid,
            'code': f"ZONE_{zid}",
            'name': s['name'],
            'english_name': s['english_name'],
            'area_m2': s['area_m2'],
            'target_pose': s['target_pose'],
            'clearance_m': s['clearance_m'],
            'polygon': s['polygon']
        }
        print(f"    - Zone [{zid}] {s['name']}: Center={s['centroid_m']}, Target={s['target_pose']}, Area={s['area_m2']} m^2, Clearance={s['clearance_m']} m")

    # Export visualization
    vis_path = out_dir / 'segmented_map.png'
    segmenter.export_visualization(str(vis_path), zones_dict)
    print(f"[+] Saved visualization overlay: {vis_path}")

    # Export json
    json_path = out_dir / 'segmented_zones.json'
    with open(json_path, 'w', encoding='utf-8') as f:
        json.dump(zones_dict, f, ensure_ascii=False, indent=2)
    print(f"[+] Saved semantic zones: {json_path}")

if __name__ == '__main__':
    main()
