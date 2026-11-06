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
    """Parses natural language commands in Vietnamese and English."""

    def __init__(self, semantic_map: SemanticMap):
        self.map = semantic_map

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

        # 1. Check Stop / Cancel command
        if re.search(r'\b(dừng lại|dừng|ngừng|đứng lại|hủy lệnh|hủy bỏ|hủy|thôi|stop|halt|cancel|abort)\b', norm):
            return VoiceIntent(intent_type=IntentType.STOP, raw_text=raw, is_valid=True)

        # 2. Check Query Location command
        if re.search(r'\b(đang ở đâu|vị trí hiện tại|vị trí|ở đâu|robot ở đâu|where are you|current location|where are we)\b', norm):
            return VoiceIntent(intent_type=IntentType.QUERY_LOCATION, raw_text=raw, is_valid=True)

        # 3. Check Multi-waypoint command (Gợi ý mở rộng):
        # Example: "Đi từ phòng A qua hành lang D rồi đến phòng B"
        # Example: "Go from room A via corridor D to room B"
        m_multi = re.search(
            r'(?:đi|di chuyển|chạy|navigate|move)\s+(?:từ|from)\s+(.+?)\s+(?:qua|đi qua|via|through)\s+(.+?)\s+(?:rồi|sau đó|tiếp tục|then)?\s*(?:đến|tới|sang|to)\s+(.+)',
            norm
        )
        if m_multi:
            raw_orig, raw_via, raw_dest = m_multi.group(1).strip(), m_multi.group(2).strip(), m_multi.group(3).strip()
            z_orig = self.map.lookup_zone(raw_orig)
            z_via = self.map.lookup_zone(raw_via)
            z_dest = self.map.lookup_zone(raw_dest)

            invalid = []
            if not z_orig: invalid.append(raw_orig)
            if not z_via: invalid.append(raw_via)
            if not z_dest: invalid.append(raw_dest)

            if invalid:
                err = f"Khu vực '{', '.join(invalid)}' không tồn tại trên bản đồ. Các khu vực hợp lệ: {self.map.get_zone_summary_text()}"
                return VoiceIntent(
                    intent_type=IntentType.INVALID_ZONE,
                    raw_text=raw,
                    unrecognized_names=invalid,
                    error_message=err,
                    is_valid=False
                )

            return VoiceIntent(
                intent_type=IntentType.NAVIGATE_WAYPOINTS,
                raw_text=raw,
                origin_zone=z_orig,
                target_zone=z_dest,
                waypoint_zones=[z_orig, z_via, z_dest],
                is_valid=True
            )

        # 4. Check Sequential Route command (From A to B):
        # Example: "Di chuyển từ khu vực A đến khu vực B."
        # Example: "Đi từ văn phòng đến phòng thí nghiệm."
        m_from_to = re.search(
            r'(?:đi|di chuyển|chạy|navigate|move)?\s*(?:từ|from)\s+(.+?)\s+(?:đến|tới|sang|to)\s+(.+)',
            norm
        )
        if m_from_to:
            raw_orig, raw_dest = m_from_to.group(1).strip(), m_from_to.group(2).strip()
            z_orig = self.map.lookup_zone(raw_orig)
            z_dest = self.map.lookup_zone(raw_dest)

            invalid = []
            if not z_orig: invalid.append(raw_orig)
            if not z_dest: invalid.append(raw_dest)

            if invalid:
                err = f"Khu vực '{', '.join(invalid)}' không tồn tại trên bản đồ. Các khu vực hợp lệ: {self.map.get_zone_summary_text()}"
                return VoiceIntent(
                    intent_type=IntentType.INVALID_ZONE,
                    raw_text=raw,
                    unrecognized_names=invalid,
                    error_message=err,
                    is_valid=False
                )

            return VoiceIntent(
                intent_type=IntentType.NAVIGATE_FROM_TO,
                raw_text=raw,
                origin_zone=z_orig,
                target_zone=z_dest,
                waypoint_zones=[z_orig, z_dest],
                is_valid=True
            )

        # 5. Check Direct Navigation command (Single Goal):
        # Example: "Đi đến khu vực A."
        # Example: "Đi đến phòng thí nghiệm."
        m_direct = re.search(
            r'(?:đi đến|đi tới|di chuyển đến|di chuyển tới|tới|hãy đến|hãy đi tới|chạy đến|chạy tới|go to|navigate to|move to)\s+(.+)',
            norm
        )
        candidate_dest = m_direct.group(1).strip() if m_direct else norm

        # Try to resolve destination zone
        z_dest = self.map.lookup_zone(candidate_dest)
        if z_dest:
            return VoiceIntent(
                intent_type=IntentType.NAVIGATE_TO,
                raw_text=raw,
                target_zone=z_dest,
                waypoint_zones=[z_dest],
                is_valid=True
            )

        # Check if the sentence has any known zone mentioned inside
        for zid, z in self.map.get_all_zones().items():
            for alias in [zid.lower(), z.name.lower(), z.english_name.lower()] + [a.lower() for a in z.aliases]:
                if re.search(rf'\b{re.escape(alias)}\b', norm) or re.search(rf'\b{re.escape(strip_accents(alias))}\b', unaccented):
                    return VoiceIntent(
                        intent_type=IntentType.NAVIGATE_TO,
                        raw_text=raw,
                        target_zone=z,
                        waypoint_zones=[z],
                        is_valid=True
                    )

        # If user intended navigation ("đi đến ...") but room was not recognized
        if m_direct:
            err = f"Khu vực '{candidate_dest}' không tồn tại trên bản đồ. Các khu vực hợp lệ: {self.map.get_zone_summary_text()}"
            return VoiceIntent(
                intent_type=IntentType.INVALID_ZONE,
                raw_text=raw,
                unrecognized_names=[candidate_dest],
                error_message=err,
                is_valid=False
            )

        # Unrecognized intent
        err = f"Không nhận diện được ý định trong câu: '{raw}'. Vui lòng nói ví dụ: 'Đi đến phòng học' hoặc 'Đi từ khu vực A đến khu vực B'."
        return VoiceIntent(
            intent_type=IntentType.UNKNOWN,
            raw_text=raw,
            error_message=err,
            is_valid=False
        )
