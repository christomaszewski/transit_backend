"""Dynamic ROS2 subscription management for TRANSIT."""

from __future__ import annotations

import time
from typing import TYPE_CHECKING

from geographic_msgs.msg import GeoPoint
from nav_msgs.msg import Path as NavPath
from transit_msgs.msg import ScenarioEnvironment, TimedPath

from .nav_path_bridge import nav_path_to_waypoints
from .state import SubscriptionConfig

if TYPE_CHECKING:
    from rclpy.node import Node


class SubscriptionManager:
    """Creates and destroys ROS2 subscriptions at runtime."""

    def __init__(self, node: Node):
        self._node = node
        self._subs: dict[str, object] = {}  # name -> subscription handle
        self._last_received: dict[str, float] = {}  # name -> timestamp
        self._latest_data: dict[str, dict] = {}  # name -> latest message data
        # Callback set by transit_node
        self.on_data_received: callable = lambda name, data: None

    def add(self, config: SubscriptionConfig) -> None:
        """Create a new ROS2 subscription."""
        if config.name in self._subs:
            self.remove(config.name)

        msg_type_map = {
            "TimedPath": TimedPath,
            "NavPath": NavPath,
            "ScenarioEnvironment": ScenarioEnvironment,
        }
        ros_type = msg_type_map.get(config.msg_type)
        if ros_type is None:
            return

        name = config.name

        def callback(msg, _name=name, _msg_type=config.msg_type):
            self._last_received[_name] = time.time()
            data = self._convert_message(msg, _msg_type)
            self._latest_data[_name] = data
            self.on_data_received(_name, data)

        sub = self._node.create_subscription(ros_type, config.topic, callback, 10)
        self._subs[name] = sub

    def remove(self, name: str) -> None:
        """Destroy a subscription."""
        if name in self._subs:
            self._node.destroy_subscription(self._subs[name])
            del self._subs[name]
            self._last_received.pop(name, None)
            self._latest_data.pop(name, None)

    def update(self, name: str, config: SubscriptionConfig, updates: dict) -> None:
        """Handle subscription updates. Recreate if topic or msg_type changed."""
        if "topic" in updates or "msg_type" in updates:
            self.remove(name)
            self.add(config)

    def get_latest_data(self, name: str) -> dict | None:
        return self._latest_data.get(name)

    def get_status(self, configs: dict[str, SubscriptionConfig], timeout: float = 5.0) -> list[dict]:
        """Get subscription status for frontend."""
        now = time.time()
        statuses = []
        for name, cfg in configs.items():
            last = self._last_received.get(name, 0)
            receiving = (now - last) < timeout if last > 0 else False
            statuses.append({
                "name": name,
                "topic": cfg.topic,
                "active": name in self._subs,
                "receiving": receiving,
            })
        return statuses

    def _convert_message(self, msg: object, msg_type: str) -> dict:
        """Convert a ROS2 message to a dict for WS transmission."""
        if msg_type == "TimedPath":
            return self._convert_timed_path(msg)
        elif msg_type == "NavPath":
            return self._convert_nav_path(msg)
        elif msg_type == "ScenarioEnvironment":
            return self._convert_env(msg)
        return {}

    def _convert_timed_path(self, msg: TimedPath) -> dict:
        waypoints = []
        for tw in msg.waypoints:
            wp = {
                "lat": tw.position.latitude,
                "lon": tw.position.longitude,
                "alt": tw.position.altitude,
                "desired_time": _time_to_iso(tw.desired_time),
                "speed_to_next": tw.speed_to_next,
            }
            waypoints.append(wp)
        return {
            "type": "path",
            "path_id": msg.path_id,
            "waypoints": waypoints,
        }

    def _convert_nav_path(self, msg: NavPath) -> dict:
        waypoints = nav_path_to_waypoints(msg)
        return {
            "type": "path",
            "path_id": "",
            "waypoints": waypoints,
        }

    def _convert_env(self, msg: ScenarioEnvironment) -> dict:
        obstacles = []
        for o in msg.obstacles:
            obstacles.append({
                "id": o.id,
                "lat": o.position.latitude,
                "lon": o.position.longitude,
                "radius": o.radius,
            })
        keepout_zones = []
        for k in msg.keepout_zones:
            vertices = [{"lat": v.latitude, "lon": v.longitude} for v in k.vertices]
            keepout_zones.append({
                "id": k.id,
                "vertices": vertices,
                "min_altitude": k.min_altitude if k.min_altitude != float("-inf") else None,
                "max_altitude": k.max_altitude if k.max_altitude != float("inf") else None,
            })
        return {
            "type": "environment",
            "obstacles": obstacles,
            "keepout_zones": keepout_zones,
        }


def _time_to_iso(t) -> str | None:
    if t.sec == 0 and t.nanosec == 0:
        return None
    from datetime import datetime, timezone
    total_seconds = t.sec + t.nanosec / 1e9
    dt = datetime.fromtimestamp(total_seconds, tz=timezone.utc)
    return dt.isoformat().replace("+00:00", "Z")
