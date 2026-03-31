"""Conversion between TimedPath and nav_msgs/Path using WGS84 convention.

Convention:
  - header.frame_id = "wgs84"
  - pose.position.x = latitude (degrees)
  - pose.position.y = longitude (degrees)
  - pose.position.z = altitude (meters, WGS84)
  - header.stamp = desired arrival time
  - pose.orientation = identity quaternion (unused)
"""

from __future__ import annotations

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from nav_msgs.msg import Path as NavPath
from std_msgs.msg import Header

from .state import TimedPathState, WaypointState


def timed_path_to_nav_path(path_state: TimedPathState) -> NavPath:
    """Convert internal TimedPathState to nav_msgs/Path."""
    nav = NavPath()
    nav.header = Header()
    nav.header.frame_id = "wgs84"

    for wp in path_state.waypoints:
        ps = PoseStamped()
        ps.header.frame_id = "wgs84"
        ps.header.stamp = _iso_to_time(wp.desired_time)
        ps.pose = Pose()
        ps.pose.position.x = wp.lat
        ps.pose.position.y = wp.lon
        ps.pose.position.z = wp.alt
        ps.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        nav.poses.append(ps)

    return nav


def nav_path_to_waypoints(msg: NavPath) -> list[dict]:
    """Convert nav_msgs/Path to list of waypoint dicts (for subscribed paths)."""
    waypoints = []
    for ps in msg.poses:
        wp = {
            "lat": ps.pose.position.x,
            "lon": ps.pose.position.y,
            "alt": ps.pose.position.z,
            "desired_time": _time_to_iso(ps.header.stamp),
            "speed_to_next": 0.0,
        }
        waypoints.append(wp)
    return waypoints


def _iso_to_time(iso_str: str | None) -> Time:
    """Convert ISO 8601 string to ROS2 Time. Returns zero time if None."""
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


def _time_to_iso(t: Time) -> str | None:
    """Convert ROS2 Time to ISO 8601 string. Returns None for zero time."""
    if t.sec == 0 and t.nanosec == 0:
        return None
    from datetime import datetime, timezone
    total_seconds = t.sec + t.nanosec / 1e9
    dt = datetime.fromtimestamp(total_seconds, tz=timezone.utc)
    return dt.isoformat().replace("+00:00", "Z")
