"""Natural Language Understanding (NLU) & Intent Parser for Voice Navigation.
Extracts navigation targets, waypoints, routes, and control commands from spoken or typed text.
"""
from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional, Tuple
import re
from .semantic_map import SemanticMap, Zone, normalize_text, strip_accents


class IntentType(str, Enum):
    NAVIGATE_TO = "NAVIGATE_TO"              # Single destination ("Đi đến phòng học")
    NAVIGATE_FROM_TO = "NAVIGATE_FROM_TO"    # From X to Y ("Đi từ khu vực A đến khu vực B")
    NAVIGATE_WAYPOINTS = "NAVIGATE_WAYPOINTS"# Multi-hop route ("Từ A qua C rồi đến B")
    STOP = "STOP"                            # Emergency halt ("Dừng lại", "Hủy")
    QUERY_LOCATION = "QUERY_LOCATION"        # Location query ("Đang ở đâu")
    INVALID_ZONE = "INVALID_ZONE"            # Command targets a non-existent room ("Đi đến khu Z")
    UNKNOWN = "UNKNOWN"                      # Unrecognized intent


@dataclass
class VoiceIntent:
    intent_type: IntentType
    raw_text: str
    target_zone: Optional[Zone] = None
    origin_zone: Optional[Zone] = None
    waypoint_zones: List[Zone] = field(default_factory=list)
    unrecognized_names: List[str] = field(default_factory=list)
    error_message: str = ""
    is_valid: bool = True

    @property
    def summary(self) -> str:
        if self.intent_type == IntentType.NAVIGATE_TO:
            return f"NAVIGATE_TO -> {self.target_zone.name}"
        elif self.intent_type == IntentType.NAVIGATE_FROM_TO:
            return f"NAVIGATE_FROM_TO -> From {self.origin_zone.name} To {self.target_zone.name}"
        elif self.intent_type == IntentType.NAVIGATE_WAYPOINTS:
            hops = " -> ".join([z.name for z in self.waypoint_zones])
            return f"NAVIGATE_WAYPOINTS -> {hops}"
        elif self.intent_type == IntentType.STOP:
            return "STOP_COMMAND"
        elif self.intent_type == IntentType.QUERY_LOCATION:
            return "QUERY_LOCATION"
        elif self.intent_type == IntentType.INVALID_ZONE:
            return f"INVALID_ZONE: {self.error_message}"
        return f"UNKNOWN: '{self.raw_text}'"


class CommandParser:
    """Parses natural language commands in Vietnamese and English, extracting single destinations,
    sequential routes (From A to B), and multi-hop itineraries (A -> B -> C -> ...).
    """

    def __init__(self, semantic_map: SemanticMap):
        self.map = semantic_map
        self._build_alias_patterns()

    def _build_alias_patterns(self):
        self._patterns = []
        for zid, z in self.map.get_all_zones().items():
            zid_lower = zid.lower()
            aliases = {
                zid_lower,
                f"khu {zid_lower}",
                f"khu vuc {zid_lower}",
                f"phong {zid_lower}",
                f"sanh {zid_lower}",
                f"hanh lang {zid_lower}",
                f"room {zid_lower}",
                f"zone {zid_lower}",
                strip_accents(normalize_text(z.name)),
                strip_accents(normalize_text(z.english_name)),
            }
            for a in z.aliases:
                aliases.add(strip_accents(normalize_text(a)))

            for a in aliases:
                if a:
                    self._patterns.append((a, zid, len(a)))

        # Sort descending by length so longer phrases match first ("phòng thí nghiệm" before "phòng")
        self._patterns.sort(key=lambda x: x[2], reverse=True)

    def parse(self, text: str) -> VoiceIntent:
        raw = text.strip()
        norm = normalize_text(raw)
        unaccented = strip_accents(norm)

        if not norm:
            return VoiceIntent(
                intent_type=IntentType.UNKNOWN,
                raw_text=raw,
                is_valid=False,
                error_message="Câu lệnh rỗng."
            )

        # 1. Stop / Cancel command
        if re.search(r'\b(dừng lại|dừng|ngừng|đứng lại|hủy lệnh|hủy bỏ|hủy|thôi|stop|halt|cancel|abort)\b', norm):
            return VoiceIntent(intent_type=IntentType.STOP, raw_text=raw, is_valid=True)

        # 2. Query Location command
        if re.search(r'\b(đang ở đâu|vị trí hiện tại|vị trí|ở đâu|robot ở đâu|where are you|current location|where are we)\b', norm):
            return VoiceIntent(intent_type=IntentType.QUERY_LOCATION, raw_text=raw, is_valid=True)

        # 3. Check for explicitly non-existent single-letter zones (e.g. "khu Z", "phòng F", "zone X")
        for m_inv in re.finditer(r'\b(?:khu vuc|khu|phong|room|zone)\s+([a-z])\b', unaccented):
            cand = m_inv.group(1).upper()
            if cand not in self.map.get_all_zones():
                inv_name = m_inv.group(0)
                err = f"Khu vực '{inv_name}' không tồn tại trên bản đồ. Các khu vực hợp lệ: {self.map.get_zone_summary_text()}"
                return VoiceIntent(
                    intent_type=IntentType.INVALID_ZONE,
                    raw_text=raw,
                    unrecognized_names=[inv_name],
                    error_message=err,
                    is_valid=False
                )

        # 4. Extract all valid zone mentions in sentence appearance order
        matches: List[Tuple[int, int, str, str]] = []  # (start, end, zone_id, pattern)
        occupied_spans = []
        for pat, zid, plen in self._patterns:
            for m_obj in re.finditer(rf'\b{re.escape(pat)}\b', unaccented):
                st, en = m_obj.span()
                if not any(ost <= st < oen or ost < en <= oen for ost, oen in occupied_spans):
                    occupied_spans.append((st, en))
                    matches.append((st, en, zid, pat))
        matches.sort(key=lambda x: x[0])

        # 5. Check invalid destinations following navigation prepositions
        # (e.g., "đi đến nhà ăn", "từ phòng A đến phòng ngủ", "chạy tới canteen")
        segments = re.split(r'\b(?:tu|from|den|toi|sang|qua|via|through|roi|sau do|tiep theo|then|va)\b', unaccented)
        for seg in segments:
            seg_s = seg.strip()
            seg_clean = re.sub(r'^(?:di|di chuyen|chay|hay|move|go|navigate|lan luot|cac|khu vuc|phong)\s*', '', seg_s).strip()
            if len(seg_clean) >= 3 and seg_clean not in ['robot', 'ngay', 'lap tuc', 'cho toi']:
                has_match = any(zid.lower() == seg_clean or pat in seg_s for _, _, zid, pat in matches)
                if not has_match and self.map.lookup_zone(seg_clean) is None:
                    if re.search(rf'\b(?:den|toi|sang|qua|tu)\s+{re.escape(seg_s)}', unaccented):
                        err = f"Khu vực '{seg_s}' không tồn tại trên bản đồ. Các khu vực hợp lệ: {self.map.get_zone_summary_text()}"
                        return VoiceIntent(
                            intent_type=IntentType.INVALID_ZONE,
                            raw_text=raw,
                            unrecognized_names=[seg_s],
                            error_message=err,
                            is_valid=False
                        )

        # 6. If no valid zones matched
        if not matches:
            if re.search(r'\b(di|di chuyen|chay|go|move|navigate|den|toi|sang)\b', unaccented):
                m_t = re.search(r'(?:den|toi|sang|to)\s+(.+)', unaccented)
                target = m_t.group(1).strip() if m_t else raw
                err = f"Khu vực '{target}' không tồn tại trên bản đồ. Các khu vực hợp lệ: {self.map.get_zone_summary_text()}"
                return VoiceIntent(
                    intent_type=IntentType.INVALID_ZONE,
                    raw_text=raw,
                    unrecognized_names=[target],
                    error_message=err,
                    is_valid=False
                )
            err = f"Không nhận diện được ý định trong câu: '{raw}'. Vui lòng nói ví dụ: 'Đi đến phòng học' hoặc 'Đi lần lượt từ A đến B đến C'."
            return VoiceIntent(
                intent_type=IntentType.UNKNOWN,
                raw_text=raw,
                error_message=err,
                is_valid=False
            )

        # 7. Deduplicate consecutive identical zones (e.g. "đi tới A rồi ở A sang B" -> [A, B])
        zone_objs: List[Zone] = []
        for _, _, zid, _ in matches:
            z = self.map.get_zone_by_id(zid)
            if z and (not zone_objs or zone_objs[-1].id != z.id):
                zone_objs.append(z)

        # 8. Construct Intent based on zone count and sentence structure
        if len(zone_objs) == 1:
            return VoiceIntent(
                intent_type=IntentType.NAVIGATE_TO,
                raw_text=raw,
                target_zone=zone_objs[0],
                waypoint_zones=zone_objs,
                is_valid=True
            )
        elif len(zone_objs) == 2:
            # Check if explicit "from ... to ..."
            if re.search(r'\b(tu|from)\b.+\b(den|toi|sang|to)\b', unaccented):
                return VoiceIntent(
                    intent_type=IntentType.NAVIGATE_FROM_TO,
                    raw_text=raw,
                    origin_zone=zone_objs[0],
                    target_zone=zone_objs[1],
                    waypoint_zones=zone_objs,
                    is_valid=True
                )
            else:
                return VoiceIntent(
                    intent_type=IntentType.NAVIGATE_WAYPOINTS,
                    raw_text=raw,
                    origin_zone=zone_objs[0],
                    target_zone=zone_objs[-1],
                    waypoint_zones=zone_objs,
                    is_valid=True
                )
        else:
            # Multi-hop route (3 or more stops, e.g. A -> B -> C or A -> E -> C)
            return VoiceIntent(
                intent_type=IntentType.NAVIGATE_WAYPOINTS,
                raw_text=raw,
                origin_zone=zone_objs[0],
                target_zone=zone_objs[-1],
                waypoint_zones=zone_objs,
                is_valid=True
            )
