import json
from typing import Any, Dict, Iterable, List, Optional

ROLE_USES_FIELDS = {"", "total_power", "spectrum_decimated"}


def normalize_fields(fields: Optional[Iterable[str]]) -> List[str]:
    if not fields:
        return []
    ordered = []
    for field in fields:
        if field and field not in ordered:
            ordered.append(str(field))
    return ordered



def normalize_role_fields(role: str = "", fields: Optional[Iterable[str]] = None) -> List[str]:
    normalized = normalize_fields(fields)
    return normalized if (role or "") in ROLE_USES_FIELDS else []


def normalize_options(options: Optional[Dict[str, Any]]) -> Dict[str, Any]:
    if not options:
        return {}
    normalized: Dict[str, Any] = {}
    for key, value in sorted(options.items(), key=lambda item: str(item[0])):
        normalized[str(key)] = value
    return normalized



def make_stream_key(
    topic: str,
    *,
    role: str = "",
    fields: Optional[Iterable[str]] = None,
    options: Optional[Dict[str, Any]] = None,
) -> str:
    payload = {
        "topic": topic,
        "role": role or "",
        "fields": normalize_role_fields(role, fields),
        "options": normalize_options(options),
    }
    return json.dumps(payload, sort_keys=True, separators=(",", ":"))
