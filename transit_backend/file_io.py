"""JSON save/load for path and environment data."""

from __future__ import annotations

import json
import os
from datetime import datetime, timezone
from pathlib import Path

from .state import (
    EnvironmentState,
    KeepoutZoneState,
    ObstacleState,
    TimedPathState,
    TransitState,
    WaypointState,
)


def save_path(state: TransitState, filename: str, save_dir: str) -> dict:
    """Save current path + environment to JSON. Returns the full data dict."""
    now = datetime.now(timezone.utc).isoformat()
    data = {
        "version": 1,
        "tool": "TRANSIT",
        "created": now,
        "modified": now,
        "path": state.path.to_dict(),
        "environment": state.environment.to_dict(),
    }
    # Remove computed fields from path for storage
    data["path"].pop("total_distance_m", None)
    data["path"].pop("estimated_time_s", None)

    os.makedirs(save_dir, exist_ok=True)
    filepath = Path(save_dir) / filename
    # Preserve original created timestamp if file exists
    if filepath.exists():
        try:
            existing = json.loads(filepath.read_text())
            data["created"] = existing.get("created", now)
        except (json.JSONDecodeError, KeyError):
            pass

    filepath.write_text(json.dumps(data, indent=2))
    return data


def load_path(filename: str, save_dir: str) -> tuple[TimedPathState, EnvironmentState]:
    """Load path + environment from JSON file."""
    filepath = Path(save_dir) / filename
    data = json.loads(filepath.read_text())
    return _parse_save_data(data)


def parse_upload(data: dict) -> tuple[TimedPathState, EnvironmentState]:
    """Parse uploaded JSON data."""
    return _parse_save_data(data)


def _parse_save_data(data: dict) -> tuple[TimedPathState, EnvironmentState]:
    path_data = data.get("path", {})
    env_data = data.get("environment", {})

    waypoints = [
        WaypointState(
            lat=w["lat"],
            lon=w["lon"],
            alt=w.get("alt", 0.0),
            desired_time=w.get("desired_time"),
            speed_to_next=w.get("speed_to_next", 0.0),
        )
        for w in path_data.get("waypoints", [])
    ]
    path = TimedPathState(
        path_id=path_data.get("path_id", "loaded_path"),
        waypoints=waypoints,
    )

    obstacles = [
        ObstacleState(
            id=o["id"],
            lat=o["lat"],
            lon=o["lon"],
            radius=o["radius"],
        )
        for o in env_data.get("obstacles", [])
    ]
    keepout_zones = [
        KeepoutZoneState(
            id=k["id"],
            vertices=k["vertices"],
            min_altitude=k.get("min_altitude"),
            max_altitude=k.get("max_altitude"),
        )
        for k in env_data.get("keepout_zones", [])
    ]
    environment = EnvironmentState(obstacles=obstacles, keepout_zones=keepout_zones)
    return path, environment


def list_saved_paths(save_dir: str) -> list[dict]:
    """List saved path files with metadata."""
    files = []
    save_path = Path(save_dir)
    if not save_path.exists():
        return files
    for f in sorted(save_path.glob("*.json")):
        try:
            data = json.loads(f.read_text())
            modified = data.get("modified", "")
        except (json.JSONDecodeError, KeyError):
            modified = ""
        files.append({"filename": f.name, "modified": modified})
    return files
