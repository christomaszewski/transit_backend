"""Core state management and undo/redo for TRANSIT."""

from __future__ import annotations

import math
import uuid
from copy import deepcopy
from dataclasses import dataclass, field
from datetime import datetime, timezone


@dataclass
class WaypointState:
    lat: float
    lon: float
    alt: float = 0.0
    desired_time: str | None = None  # ISO 8601 or None
    speed_to_next: float = 0.0  # m/s, 0 = unconstrained

    def to_dict(self) -> dict:
        return {
            "lat": self.lat,
            "lon": self.lon,
            "alt": self.alt,
            "desired_time": self.desired_time,
            "speed_to_next": self.speed_to_next,
        }


@dataclass
class TimedPathState:
    path_id: str = field(default_factory=lambda: f"user_path_{uuid.uuid4().hex[:8]}")
    waypoints: list[WaypointState] = field(default_factory=list)

    def total_distance_m(self) -> float:
        total = 0.0
        for i in range(len(self.waypoints) - 1):
            total += _haversine(self.waypoints[i], self.waypoints[i + 1])
        return total

    def estimated_time_s(self) -> float:
        total = 0.0
        for i in range(len(self.waypoints) - 1):
            dist = _haversine(self.waypoints[i], self.waypoints[i + 1])
            speed = self.waypoints[i].speed_to_next
            if speed > 0:
                total += dist / speed
        return total

    def to_dict(self) -> dict:
        return {
            "path_id": self.path_id,
            "waypoints": [w.to_dict() for w in self.waypoints],
            "total_distance_m": round(self.total_distance_m(), 1),
            "estimated_time_s": round(self.estimated_time_s(), 1),
        }


@dataclass
class ObstacleState:
    id: str
    lat: float
    lon: float
    radius: float  # meters

    def to_dict(self) -> dict:
        return {"id": self.id, "lat": self.lat, "lon": self.lon, "radius": self.radius}


@dataclass
class KeepoutZoneState:
    id: str
    vertices: list[dict]  # [{"lat": float, "lon": float}, ...]
    min_altitude: float | None = None
    max_altitude: float | None = None

    def to_dict(self) -> dict:
        return {
            "id": self.id,
            "vertices": self.vertices,
            "min_altitude": self.min_altitude,
            "max_altitude": self.max_altitude,
        }


@dataclass
class EnvironmentState:
    obstacles: list[ObstacleState] = field(default_factory=list)
    keepout_zones: list[KeepoutZoneState] = field(default_factory=list)

    def to_dict(self) -> dict:
        return {
            "obstacles": [o.to_dict() for o in self.obstacles],
            "keepout_zones": [k.to_dict() for k in self.keepout_zones],
        }


@dataclass
class PublishConfig:
    path_topic: str = "/transit/planned_path"
    path_rate_hz: float = 1.0
    path_enabled: bool = True
    nav_path_enabled: bool = False
    nav_path_topic: str = "/transit/nav_path"
    env_topic: str = "/transit/environment"
    env_enabled: bool = True

    def to_dict(self) -> dict:
        return {
            "path_topic": self.path_topic,
            "path_rate_hz": self.path_rate_hz,
            "path_enabled": self.path_enabled,
            "nav_path_enabled": self.nav_path_enabled,
            "nav_path_topic": self.nav_path_topic,
            "env_topic": self.env_topic,
            "env_enabled": self.env_enabled,
        }


@dataclass
class SubscriptionConfig:
    name: str
    topic: str
    msg_type: str  # "TimedPath" | "NavPath" | "ScenarioEnvironment"
    color: str = "#FF6B35"
    visible: bool = True

    def to_dict(self) -> dict:
        return {
            "name": self.name,
            "topic": self.topic,
            "msg_type": self.msg_type,
            "color": self.color,
            "visible": self.visible,
        }


@dataclass
class DeviationConfig:
    mode: str = "one_to_many"  # "one_to_many" | "pairwise"
    reference: str = "user_path"
    targets: list[str] = field(default_factory=list)
    show_map_visualization: bool = True

    def to_dict(self) -> dict:
        return {
            "mode": self.mode,
            "reference": self.reference,
            "targets": list(self.targets),
            "show_map_visualization": self.show_map_visualization,
        }


@dataclass
class TransitState:
    path: TimedPathState = field(default_factory=TimedPathState)
    environment: EnvironmentState = field(default_factory=EnvironmentState)
    publish_config: PublishConfig = field(default_factory=PublishConfig)
    subscriptions: dict[str, SubscriptionConfig] = field(default_factory=dict)
    deviation_config: DeviationConfig = field(default_factory=DeviationConfig)


class UndoManager:
    def __init__(self, max_depth: int = 50):
        self.max_depth = max_depth
        self.undo_stack: list[tuple[TimedPathState, EnvironmentState]] = []
        self.redo_stack: list[tuple[TimedPathState, EnvironmentState]] = []

    def push(self, state: TransitState) -> None:
        """Save current path+env state before mutation."""
        snapshot = (deepcopy(state.path), deepcopy(state.environment))
        self.undo_stack.append(snapshot)
        if len(self.undo_stack) > self.max_depth:
            self.undo_stack.pop(0)
        self.redo_stack.clear()

    def undo(self, state: TransitState) -> bool:
        if not self.undo_stack:
            return False
        self.redo_stack.append((deepcopy(state.path), deepcopy(state.environment)))
        path, env = self.undo_stack.pop()
        state.path = path
        state.environment = env
        return True

    def redo(self, state: TransitState) -> bool:
        if not self.redo_stack:
            return False
        self.undo_stack.append((deepcopy(state.path), deepcopy(state.environment)))
        path, env = self.redo_stack.pop()
        state.path = path
        state.environment = env
        return True

    @property
    def can_undo(self) -> bool:
        return len(self.undo_stack) > 0

    @property
    def can_redo(self) -> bool:
        return len(self.redo_stack) > 0


def _haversine(a: WaypointState, b: WaypointState) -> float:
    """Haversine distance in meters between two waypoints."""
    R = 6371000.0
    lat1, lat2 = math.radians(a.lat), math.radians(b.lat)
    dlat = lat2 - lat1
    dlon = math.radians(b.lon - a.lon)
    h = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    return 2 * R * math.asin(math.sqrt(h))
