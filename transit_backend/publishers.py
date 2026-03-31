"""ROS2 publisher management for TRANSIT."""

from __future__ import annotations

from typing import TYPE_CHECKING

from builtin_interfaces.msg import Time
from geographic_msgs.msg import GeoPoint
from nav_msgs.msg import Path as NavPath
from std_msgs.msg import Header
from transit_msgs.msg import (
    KeepoutZone,
    Obstacle,
    ScenarioEnvironment,
    TimedPath,
    TimedWaypoint,
)

from .nav_path_bridge import timed_path_to_nav_path

if TYPE_CHECKING:
    from rclpy.node import Node
    from .state import TransitState


class PublisherManager:
    """Manages ROS2 publishers for path, nav_path, and environment."""

    def __init__(self, node: Node, state: TransitState):
        self._node = node
        self._state = state

        self._path_pub = None
        self._path_topic = ""
        self._nav_path_pub = None
        self._nav_path_topic = ""
        self._env_pub = None
        self._env_topic = ""

        self._setup_publishers()

    def _setup_publishers(self) -> None:
        cfg = self._state.publish_config
        self._create_path_publisher(cfg.path_topic)
        if cfg.nav_path_enabled:
            self._create_nav_path_publisher(cfg.nav_path_topic)
        self._create_env_publisher(cfg.env_topic)

    def _create_path_publisher(self, topic: str) -> None:
        if self._path_pub is not None and self._path_topic == topic:
            return
        if self._path_pub is not None:
            self._node.destroy_publisher(self._path_pub)
        self._path_pub = self._node.create_publisher(TimedPath, topic, 10)
        self._path_topic = topic

    def _create_nav_path_publisher(self, topic: str) -> None:
        if self._nav_path_pub is not None and self._nav_path_topic == topic:
            return
        if self._nav_path_pub is not None:
            self._node.destroy_publisher(self._nav_path_pub)
        self._nav_path_pub = self._node.create_publisher(NavPath, topic, 10)
        self._nav_path_topic = topic

    def _create_env_publisher(self, topic: str) -> None:
        if self._env_pub is not None and self._env_topic == topic:
            return
        if self._env_pub is not None:
            self._node.destroy_publisher(self._env_pub)
        self._env_pub = self._node.create_publisher(ScenarioEnvironment, topic, 10)
        self._env_topic = topic

    def reconfigure(self) -> None:
        """Reconfigure publishers based on current publish_config."""
        cfg = self._state.publish_config
        self._create_path_publisher(cfg.path_topic)
        self._create_env_publisher(cfg.env_topic)
        if cfg.nav_path_enabled:
            self._create_nav_path_publisher(cfg.nav_path_topic)
        elif self._nav_path_pub is not None:
            self._node.destroy_publisher(self._nav_path_pub)
            self._nav_path_pub = None
            self._nav_path_topic = ""

    def publish_path(self) -> None:
        """Publish current path as TimedPath (and optionally nav_msgs/Path)."""
        cfg = self._state.publish_config
        if not cfg.path_enabled or self._path_pub is None:
            return

        msg = self._build_timed_path_msg()
        self._path_pub.publish(msg)

        if cfg.nav_path_enabled and self._nav_path_pub is not None:
            nav_msg = timed_path_to_nav_path(self._state.path)
            nav_msg.header.stamp = self._node.get_clock().now().to_msg()
            self._nav_path_pub.publish(nav_msg)

    def publish_environment(self) -> None:
        """Publish current environment as ScenarioEnvironment."""
        cfg = self._state.publish_config
        if not cfg.env_enabled or self._env_pub is None:
            return

        msg = self._build_env_msg()
        self._env_pub.publish(msg)

    def get_status(self) -> dict:
        """Get publisher status for frontend."""
        cfg = self._state.publish_config
        status = {}
        if self._path_pub is not None:
            status["path"] = {
                "topic": self._path_topic,
                "rate_hz": cfg.path_rate_hz,
                "active": cfg.path_enabled,
                "listeners": self._path_pub.get_subscription_count(),
            }
        if self._nav_path_pub is not None:
            status["nav_path"] = {
                "topic": self._nav_path_topic,
                "active": cfg.nav_path_enabled,
                "listeners": self._nav_path_pub.get_subscription_count(),
            }
        if self._env_pub is not None:
            status["environment"] = {
                "topic": self._env_topic,
                "active": cfg.env_enabled,
                "listeners": self._env_pub.get_subscription_count(),
            }
        return status

    def _build_timed_path_msg(self) -> TimedPath:
        msg = TimedPath()
        msg.header = Header()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = "wgs84"
        msg.path_id = self._state.path.path_id

        for wp in self._state.path.waypoints:
            tw = TimedWaypoint()
            tw.header = Header()
            tw.header.frame_id = "wgs84"
            tw.position = GeoPoint()
            tw.position.latitude = wp.lat
            tw.position.longitude = wp.lon
            tw.position.altitude = wp.alt
            tw.desired_time = _iso_to_time(wp.desired_time)
            tw.speed_to_next = wp.speed_to_next
            msg.waypoints.append(tw)

        return msg

    def _build_env_msg(self) -> ScenarioEnvironment:
        msg = ScenarioEnvironment()
        msg.header = Header()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = "wgs84"

        for obs in self._state.environment.obstacles:
            o = Obstacle()
            o.id = obs.id
            o.position = GeoPoint()
            o.position.latitude = obs.lat
            o.position.longitude = obs.lon
            o.position.altitude = 0.0
            o.radius = obs.radius
            msg.obstacles.append(o)

        for koz in self._state.environment.keepout_zones:
            k = KeepoutZone()
            k.id = koz.id
            for v in koz.vertices:
                gp = GeoPoint()
                gp.latitude = v["lat"]
                gp.longitude = v["lon"]
                gp.altitude = 0.0
                k.vertices.append(gp)
            k.min_altitude = koz.min_altitude if koz.min_altitude is not None else float("-inf")
            k.max_altitude = koz.max_altitude if koz.max_altitude is not None else float("inf")
            msg.keepout_zones.append(k)

        return msg


def _iso_to_time(iso_str: str | None) -> Time:
    t = Time()
    if not iso_str:
        return t
    try:
        from datetime import datetime, timezone
        dt = datetime.fromisoformat(iso_str.replace("Z", "+00:00"))
        epoch = datetime(1970, 1, 1, tzinfo=timezone.utc)
        delta = dt - epoch
        t.sec = int(delta.total_seconds())
        t.nanosec = int((delta.total_seconds() % 1) * 1e9)
    except (ValueError, AttributeError):
        pass
    return t
