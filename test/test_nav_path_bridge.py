"""Tests for transit_backend.nav_path_bridge — TimedPath <-> nav_msgs/Path conversion.

Requires ROS2 message packages (builtin_interfaces, nav_msgs, geometry_msgs).
These tests run in Docker CI or a sourced ROS2 environment.
"""

import pytest

ros2_available = pytest.importorskip("builtin_interfaces", reason="ROS2 not available")

from transit_backend.nav_path_bridge import (
    _iso_to_time,
    _time_to_iso,
    nav_path_to_waypoints,
    timed_path_to_nav_path,
)
from transit_backend.state import TimedPathState, WaypointState


class TestIsoToTime:
    def test_none(self):
        t = _iso_to_time(None)
        assert t.sec == 0
        assert t.nanosec == 0

    def test_empty_string(self):
        t = _iso_to_time("")
        assert t.sec == 0

    def test_valid_iso(self):
        t = _iso_to_time("2025-04-01T12:00:00Z")
        assert t.sec > 0

    def test_round_trip(self):
        """Convert to Time and back, should get same ISO string."""
        original = "2025-04-01T12:00:00Z"
        t = _iso_to_time(original)
        result = _time_to_iso(t)
        assert result is not None
        # Parse both and compare (may differ in format but same instant)
        from datetime import datetime, timezone
        orig_dt = datetime.fromisoformat(original.replace("Z", "+00:00"))
        result_dt = datetime.fromisoformat(result.replace("Z", "+00:00"))
        assert abs((orig_dt - result_dt).total_seconds()) < 1.0


class TestTimeToIso:
    def test_zero_time(self):
        from builtin_interfaces.msg import Time
        t = Time()
        t.sec = 0
        t.nanosec = 0
        assert _time_to_iso(t) is None

    def test_nonzero_time(self):
        from builtin_interfaces.msg import Time
        t = Time()
        t.sec = 1711972800  # 2024-04-01T12:00:00Z
        t.nanosec = 0
        result = _time_to_iso(t)
        assert result is not None
        assert "2024" in result


class TestTimedPathToNavPath:
    def test_empty_path(self):
        path = TimedPathState(waypoints=[])
        nav = timed_path_to_nav_path(path)
        assert len(nav.poses) == 0
        assert nav.header.frame_id == "wgs84"

    def test_single_waypoint(self):
        path = TimedPathState(waypoints=[
            WaypointState(lat=33.95, lon=-84.55, alt=100.0),
        ])
        nav = timed_path_to_nav_path(path)
        assert len(nav.poses) == 1
        ps = nav.poses[0]
        assert ps.pose.position.x == pytest.approx(33.95)
        assert ps.pose.position.y == pytest.approx(-84.55)
        assert ps.pose.position.z == pytest.approx(100.0)
        assert ps.header.frame_id == "wgs84"

    def test_identity_quaternion(self):
        path = TimedPathState(waypoints=[WaypointState(lat=0, lon=0)])
        nav = timed_path_to_nav_path(path)
        q = nav.poses[0].pose.orientation
        assert q.x == 0.0
        assert q.y == 0.0
        assert q.z == 0.0
        assert q.w == 1.0

    def test_waypoint_count_preserved(self):
        path = TimedPathState(waypoints=[
            WaypointState(lat=i, lon=i) for i in range(5)
        ])
        nav = timed_path_to_nav_path(path)
        assert len(nav.poses) == 5


class TestNavPathToWaypoints:
    def test_round_trip(self):
        """Convert to nav_msgs/Path and back, waypoint data should be preserved."""
        original = TimedPathState(waypoints=[
            WaypointState(lat=33.95, lon=-84.55, alt=100.0, desired_time="2025-04-01T12:00:00Z"),
            WaypointState(lat=34.05, lon=-84.45, alt=50.0),
        ])
        nav = timed_path_to_nav_path(original)
        waypoints = nav_path_to_waypoints(nav)

        assert len(waypoints) == 2
        assert waypoints[0]["lat"] == pytest.approx(33.95)
        assert waypoints[0]["lon"] == pytest.approx(-84.55)
        assert waypoints[0]["alt"] == pytest.approx(100.0)
        # Second waypoint had no time
        assert waypoints[1]["desired_time"] is None

    def test_empty_nav_path(self):
        from nav_msgs.msg import Path
        nav = Path()
        waypoints = nav_path_to_waypoints(nav)
        assert waypoints == []
