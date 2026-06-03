import array
import math
from typing import Any, Dict, Iterable, List, Optional, Tuple


NUMERIC_TYPES = (int, float)
ARRAY_TYPES = (list, tuple, array.array)
DEFAULT_MAX_POINTS = 1024
MIN_MAX_POINTS = 8
MAX_MAX_POINTS = 16384


def _is_finite_number(value: Any) -> bool:
    return isinstance(value, NUMERIC_TYPES) and not isinstance(value, bool) and math.isfinite(value)



def _iter_numeric_values(values: Iterable[Any]) -> List[float]:
    numeric: List[float] = []
    for value in values:
        if _is_finite_number(value):
            numeric.append(float(value))
    return numeric



def _is_numeric_array(value: Any) -> bool:
    if not isinstance(value, ARRAY_TYPES):
        return False
    if len(value) == 0:
        return False
    return any(_is_finite_number(v) for v in value)



def sanitize_max_points(value: Any, default: int = DEFAULT_MAX_POINTS) -> int:
    try:
        parsed = int(value)
    except (TypeError, ValueError):
        parsed = default
    return max(MIN_MAX_POINTS, min(MAX_MAX_POINTS, parsed))



def sanitize_channel_index(value: Any) -> Optional[int]:
    if value in (None, ""):
        return None
    try:
        return int(value)
    except (TypeError, ValueError):
        return None



def resolve_channel_range(
    values: Any,
    channel_start: Any = None,
    channel_stop: Any = None,
) -> Tuple[int, int]:
    if not isinstance(values, ARRAY_TYPES) or len(values) == 0:
        return (0, -1)
    n = len(values)
    start = sanitize_channel_index(channel_start)
    stop = sanitize_channel_index(channel_stop)
    if start is None:
        start = 0
    if stop is None:
        stop = n - 1
    start = max(0, min(n - 1, start))
    stop = max(0, min(n - 1, stop))
    if stop < start:
        start, stop = stop, start
    return (start, stop)



def resolve_time_value(data: Dict[str, Any], fallback_time: Optional[float] = None) -> Optional[float]:
    time_value = data.get("time")
    if _is_finite_number(time_value):
        return float(time_value)
    return fallback_time


def select_total_power_fields(data: Dict[str, Any], requested_fields: Optional[Iterable[str]] = None) -> List[str]:
    if requested_fields is not None:
        return [field for field in requested_fields if _is_numeric_array(data.get(field))]
    return [field for field, value in data.items() if _is_numeric_array(value)]



def build_total_power_payload(
    data: Dict[str, Any],
    requested_fields: Optional[Iterable[str]] = None,
    *,
    channel_start: Any = None,
    channel_stop: Any = None,
    fallback_time: Optional[float] = None,
) -> Dict[str, Any]:
    payload: Dict[str, Any] = {}
    fields = select_total_power_fields(data, requested_fields)
    resolved_range: Optional[Tuple[int, int]] = None
    for field in fields:
        raw_values = data.get(field, [])
        if not isinstance(raw_values, ARRAY_TYPES) or len(raw_values) == 0:
            continue
        start, stop = resolve_channel_range(raw_values, channel_start, channel_stop)
        if stop < start:
            continue
        resolved_range = (start, stop)
        numeric_values = _iter_numeric_values(raw_values[start : stop + 1])
        if numeric_values:
            payload[field] = sum(numeric_values)
    has_data_field = any(field in payload for field in fields)
    if not has_data_field:
        return {}
    resolved_time = resolve_time_value(data, fallback_time)
    if resolved_time is not None:
        payload["time"] = resolved_time
    payload["observer_profile"] = "total_power"
    if resolved_range is not None:
        payload["observer_channel_start"] = resolved_range[0]
        payload["observer_channel_stop"] = resolved_range[1]
    return payload



def _collect_indexed_numeric_values(values: Iterable[Any], start_index: int = 0) -> List[Tuple[int, float]]:
    indexed: List[Tuple[int, float]] = []
    for offset, value in enumerate(values):
        if _is_finite_number(value):
            indexed.append((start_index + offset, float(value)))
    return indexed



def _downsample_indexed_pairs(indexed_values: List[Tuple[int, float]], max_points: int = DEFAULT_MAX_POINTS) -> List[Tuple[int, float]]:
    max_points = sanitize_max_points(max_points)
    if len(indexed_values) <= max_points:
        return indexed_values
    if max_points == 1:
        return [indexed_values[-1]]
    last_index = len(indexed_values) - 1
    sampled_indices = []
    for i in range(max_points):
        idx = round(i * last_index / (max_points - 1))
        if (not sampled_indices) or (idx != sampled_indices[-1]):
            sampled_indices.append(idx)
    if sampled_indices[-1] != last_index:
        sampled_indices[-1] = last_index
    return [indexed_values[idx] for idx in sampled_indices]



def downsample_array(values: Iterable[Any], max_points: int = DEFAULT_MAX_POINTS) -> List[float]:
    indexed = _collect_indexed_numeric_values(values)
    return [value for _, value in _downsample_indexed_pairs(indexed, max_points=max_points)]



def build_decimated_payload(
    data: Dict[str, Any],
    requested_fields: Optional[Iterable[str]] = None,
    *,
    max_points: int = DEFAULT_MAX_POINTS,
    channel_start: Any = None,
    channel_stop: Any = None,
    fallback_time: Optional[float] = None,
) -> Dict[str, Any]:
    payload: Dict[str, Any] = {}
    fields = select_total_power_fields(data, requested_fields)
    max_points = sanitize_max_points(max_points)
    resolved_range: Optional[Tuple[int, int]] = None
    x_values: Optional[List[int]] = None
    for field in fields:
        raw_values = data.get(field, [])
        if not isinstance(raw_values, ARRAY_TYPES) or len(raw_values) == 0:
            continue
        start, stop = resolve_channel_range(raw_values, channel_start, channel_stop)
        if stop < start:
            continue
        resolved_range = (start, stop)
        sliced = raw_values[start : stop + 1]
        indexed_numeric = _collect_indexed_numeric_values(sliced, start_index=start)
        decimated_pairs = _downsample_indexed_pairs(indexed_numeric, max_points=max_points)
        if not decimated_pairs:
            continue
        payload[field] = [value for _, value in decimated_pairs]
        if x_values is None:
            x_values = [idx for idx, _ in decimated_pairs]
    has_data_field = any(field in payload for field in fields)
    if not has_data_field:
        return {}
    resolved_time = resolve_time_value(data, fallback_time)
    if resolved_time is not None:
        payload["time"] = resolved_time
    payload["observer_profile"] = "spectrum_decimated"
    payload["observer_max_points"] = max_points
    if resolved_range is not None:
        payload["observer_channel_start"] = resolved_range[0]
        payload["observer_channel_stop"] = resolved_range[1]
    if x_values is not None:
        payload["observer_x_values"] = x_values
    return payload



def transform_payload(
    data: Dict[str, Any],
    *,
    role: str = "",
    fields: Optional[Iterable[str]] = None,
    options: Optional[Dict[str, Any]] = None,
    fallback_time: Optional[float] = None,
) -> Dict[str, Any]:
    options = options or {}
    if role == "total_power":
        return build_total_power_payload(
            data,
            fields,
            channel_start=options.get("channel_start"),
            channel_stop=options.get("channel_stop"),
            fallback_time=fallback_time,
        )
    if role == "spectrum_decimated":
        max_points = sanitize_max_points(options.get("max_points", DEFAULT_MAX_POINTS))
        return build_decimated_payload(
            data,
            fields,
            max_points=max_points,
            channel_start=options.get("channel_start"),
            channel_stop=options.get("channel_stop"),
            fallback_time=fallback_time,
        )
    return data
