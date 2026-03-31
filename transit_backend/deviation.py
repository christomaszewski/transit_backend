"""Path deviation computation between reference and target paths."""

from __future__ import annotations

import math
from datetime import datetime, timezone

from .state import WaypointState


def compute_deviations(
    reference_waypoints: list[dict],
    target_waypoints: list[dict],
    reference_name: str,
    target_name: str,
) -> dict:
    """Compute spatial and temporal deviations between two paths.

    Each waypoint dict has keys: lat, lon, alt, desired_time, speed_to_next.
    Returns a comparison dict matching the deviation_update protocol.
    """
    if not reference_waypoints or not target_waypoints:
        return {
            "reference": reference_name,
            "target": target_name,
            "max_spatial_deviation_m": 0.0,
            "mean_spatial_deviation_m": 0.0,
            "path_length_diff_m": 0.0,
            "temporal_deviations": [],
        }

    # Spatial deviation: for each target point, find min distance to reference polyline
    spatial_devs = []
    for wp in target_waypoints:
        d = _point_to_polyline_distance(wp["lat"], wp["lon"], reference_waypoints)
        spatial_devs.append(d)

    max_dev = max(spatial_devs) if spatial_devs else 0.0
    mean_dev = sum(spatial_devs) / len(spatial_devs) if spatial_devs else 0.0

    # Path length difference
    ref_len = _path_length(reference_waypoints)
    tgt_len = _path_length(target_waypoints)
    length_diff = tgt_len - ref_len

    # Temporal deviation: for each reference waypoint, find nearest target point and compare times
    temporal_devs = []
    for i, ref_wp in enumerate(reference_waypoints):
        nearest_target = _find_nearest_waypoint(ref_wp, target_waypoints)
        diff_s = _time_diff_seconds(ref_wp.get("desired_time"), nearest_target.get("desired_time"))
        temporal_devs.append({
            "waypoint_index": i,
            "ahead_behind_s": round(diff_s, 1),
        })

    return {
        "reference": reference_name,
        "target": target_name,
        "max_spatial_deviation_m": round(max_dev, 1),
        "mean_spatial_deviation_m": round(mean_dev, 1),
        "path_length_diff_m": round(length_diff, 1),
        "temporal_deviations": temporal_devs,
    }


def _point_to_polyline_distance(lat: float, lon: float, polyline: list[dict]) -> float:
    """Minimum distance from point to polyline (meters)."""
    if len(polyline) < 2:
        if polyline:
            return _haversine_dict(lat, lon, polyline[0]["lat"], polyline[0]["lon"])
        return 0.0

    min_dist = float("inf")
    for i in range(len(polyline) - 1):
        d = _point_to_segment_distance(
            lat, lon,
            polyline[i]["lat"], polyline[i]["lon"],
            polyline[i + 1]["lat"], polyline[i + 1]["lon"],
        )
        min_dist = min(min_dist, d)
    return min_dist


def _point_to_segment_distance(
    plat: float, plon: float,
    alat: float, alon: float,
    blat: float, blon: float,
) -> float:
    """Distance from point P to line segment AB using equirectangular projection."""
    # Project to local meters using equirectangular approximation
    mid_lat = math.radians((alat + blat) / 2)
    cos_mid = math.cos(mid_lat)
    R = 6371000.0

    # Convert to local x,y (meters)
    px = math.radians(plon) * cos_mid * R
    py = math.radians(plat) * R
    ax = math.radians(alon) * cos_mid * R
    ay = math.radians(alat) * R
    bx = math.radians(blon) * cos_mid * R
    by = math.radians(blat) * R

    # Project P onto segment AB
    dx, dy = bx - ax, by - ay
    seg_len_sq = dx * dx + dy * dy
    if seg_len_sq < 1e-12:
        return math.sqrt((px - ax) ** 2 + (py - ay) ** 2)

    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / seg_len_sq))
    proj_x = ax + t * dx
    proj_y = ay + t * dy
    return math.sqrt((px - proj_x) ** 2 + (py - proj_y) ** 2)


def _haversine_dict(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    R = 6371000.0
    rlat1, rlat2 = math.radians(lat1), math.radians(lat2)
    dlat = rlat2 - rlat1
    dlon = math.radians(lon2 - lon1)
    h = math.sin(dlat / 2) ** 2 + math.cos(rlat1) * math.cos(rlat2) * math.sin(dlon / 2) ** 2
    return 2 * R * math.asin(math.sqrt(h))


def _path_length(waypoints: list[dict]) -> float:
    total = 0.0
    for i in range(len(waypoints) - 1):
        total += _haversine_dict(
            waypoints[i]["lat"], waypoints[i]["lon"],
            waypoints[i + 1]["lat"], waypoints[i + 1]["lon"],
        )
    return total


def _find_nearest_waypoint(ref_wp: dict, targets: list[dict]) -> dict:
    """Find the target waypoint nearest to the reference waypoint."""
    best = targets[0]
    best_dist = _haversine_dict(ref_wp["lat"], ref_wp["lon"], best["lat"], best["lon"])
    for t in targets[1:]:
        d = _haversine_dict(ref_wp["lat"], ref_wp["lon"], t["lat"], t["lon"])
        if d < best_dist:
            best = t
            best_dist = d
    return best


def _time_diff_seconds(ref_time: str | None, target_time: str | None) -> float:
    """Compute target_time - ref_time in seconds. Positive = behind schedule."""
    if not ref_time or not target_time:
        return 0.0
    try:
        ref_dt = datetime.fromisoformat(ref_time.replace("Z", "+00:00"))
        tgt_dt = datetime.fromisoformat(target_time.replace("Z", "+00:00"))
        return (tgt_dt - ref_dt).total_seconds()
    except (ValueError, AttributeError):
        return 0.0
