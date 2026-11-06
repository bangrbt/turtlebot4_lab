"""Semantic Map Manager: loads zone definitions, resolves natural language aliases,
and performs spatial point-in-polygon containment queries.
"""
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Any
import unicodedata
import re
import yaml
import json


def normalize_text(text: str) -> str:
    """Normalize text for robust keyword matching: lowercase, strip punctuation,
    collapse spaces, and handle Vietnamese diacritics.
    """
    if not text:
        return ""
    text = text.lower().strip()
    # Replace common punctuation with space
    text = re.sub(r'[\.,!?_\-\:;\"\'\(\)\[\]]', ' ', text)
    # Collapse multiple whitespace
    text = re.sub(r'\s+', ' ', text).strip()
    return text


def strip_accents(text: str) -> str:
    """Strip Vietnamese diacritics for fallback matching (e.g., 'phòng học' -> 'phong hoc')."""
    text = unicodedata.normalize('NFD', text)
    text = ''.join(ch for ch in text if unicodedata.category(ch) != 'Mn')
    text = text.replace('đ', 'd').replace('Đ', 'D')
    return unicodedata.normalize('NFC', text)


def point_in_polygon(x: float, y: float, polygon: List[List[float]]) -> bool:
    """Standard Ray-Casting algorithm to test if point (x, y) is inside polygon."""
    n = len(polygon)
    if n < 3:
        return False
    inside = False
    p1x, p1y = polygon[0]
    for i in range(1, n + 1):
        p2x, p2y = polygon[i % n]
        if min(p1y, p2y) < y <= max(p1y, p2y):
            if x <= max(p1x, p2x):
                if p1y != p2y:
                    xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                if p1x == p2x or x <= xinters:
                    inside = not inside
        p1x, p1y = p2x, p2y
    return inside


@dataclass
class TargetPose:
    x: float
    y: float
    yaw: float = 0.0


@dataclass
class Zone:
    id: str
    code: str
    name: str
    english_name: str
    description: str
    target_pose: TargetPose
    polygon: List[List[float]] = field(default_factory=list)
    aliases: List[str] = field(default_factory=list)
    color_rgb: List[int] = field(default_factory=lambda: [128, 128, 128])
    marker_z: float = 1.0


class SemanticMap:
    """Manages semantic zones and provides fast alias resolution and spatial queries."""

    def __init__(self, zones_config_path: str):
        path = Path(zones_config_path).resolve()
        if not path.is_file():
            raise FileNotFoundError(f"Zones config not found: {path}")

        with open(path, 'r', encoding='utf-8') as f:
            if path.suffix in ('.yaml', '.yml'):
                raw = yaml.safe_load(f)
            else:
                raw = json.load(f)

        self.zones: Dict[str, Zone] = {}
        self._alias_map: Dict[str, str] = {}  # normalized alias -> zone_id

        for zid, data in raw.items():
            t_data = data.get('target_pose', {})
            target = TargetPose(
                x=float(t_data.get('x', 0.0)),
                y=float(t_data.get('y', 0.0)),
                yaw=float(t_data.get('yaw', 0.0))
            )
            zone = Zone(
                id=str(data.get('id', zid)),
                code=str(data.get('code', f"ZONE_{zid}")),
                name=str(data.get('name', f"Khu vực {zid}")),
                english_name=str(data.get('english_name', zid)),
                description=str(data.get('description', '')),
                target_pose=target,
                polygon=data.get('polygon', []),
                aliases=[str(a) for a in data.get('aliases', [])],
                color_rgb=data.get('color_rgb', [128, 128, 128]),
                marker_z=float(data.get('marker_z', 1.0))
            )
            self.zones[zone.id] = zone

            # Register standard aliases
            self._register_alias(zone.id, zone.id)
            self._register_alias(zone.id.lower(), zone.id)
            self._register_alias(zone.name, zone.id)
            self._register_alias(zone.english_name, zone.id)
            for alias in zone.aliases:
                self._register_alias(alias, zone.id)

    def _register_alias(self, alias_text: str, zone_id: str):
        norm = normalize_text(alias_text)
        if norm:
            self._alias_map[norm] = zone_id
            self._alias_map[strip_accents(norm)] = zone_id

    def lookup_zone(self, text: str) -> Optional[Zone]:
        """Looks up a zone by natural language text or alias (exact or stripped accents)."""
        norm = normalize_text(text)
        if norm in self._alias_map:
            return self.zones[self._alias_map[norm]]

        unaccented = strip_accents(norm)
        if unaccented in self._alias_map:
            return self.zones[self._alias_map[unaccented]]

        # Substring / fuzzy search in known aliases
        for alias, zid in self._alias_map.items():
            if alias == norm or alias == unaccented:
                return self.zones[zid]

        # Heuristic: check if the text ends with or contains a single zone letter (a, b, c, d)
        # e.g., "hành lang d", "phòng học a", "khu vực b"
        match_suffix = re.search(r'\b(?:phòng|khu|khu vực|hành lang|sảnh|room|area|zone)\s+([a-d])\b', unaccented)
        if match_suffix:
            letter = match_suffix.group(1).upper()
            if letter in self.zones:
                return self.zones[letter]

        return None

    def get_zone_by_id(self, zone_id: str) -> Optional[Zone]:
        return self.zones.get(zone_id.upper()) or self.zones.get(zone_id)

    def get_zone_at_point(self, x: float, y: float) -> Optional[Zone]:
        """Finds which zone contains coordinates (x, y) via point-in-polygon."""
        for zone in self.zones.values():
            if zone.polygon and point_in_polygon(x, y, zone.polygon):
                return zone
        return None

    def get_all_zones(self) -> Dict[str, Zone]:
        return self.zones

    def get_zone_ids(self) -> List[str]:
        return sorted(list(self.zones.keys()))

    def get_zone_summary_text(self) -> str:
        """Human-readable listing of valid zones for prompt/error feedback."""
        parts = [f"{z.id} ({z.name})" for z in self.zones.values()]
        return ", ".join(parts)
