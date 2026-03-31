"""Tests for transit_backend.publishers — ROS2 publisher management.

Requires ROS2 message packages (transit_msgs, geographic_msgs, nav_msgs) and rclpy.
These tests run in Docker CI or a sourced ROS2 environment.
"""

import pytest

rclpy = pytest.importorskip("rclpy", reason="ROS2 not available")

import rclpy as _rclpy
from rclpy.node import Node

from transit_backend.publishers import PublisherManager, _iso_to_time
from transit_backend.state import (
    EnvironmentState,
    KeepoutZoneState,
    ObstacleState,
    TimedPathState,
    TransitState,
    WaypointState,
)


@pytest.fixture(scope="module")
def ros_context():
    _rclpy.init()
    yield
    _rclpy.shutdown()


@pytest.fixture
def test_node(ros_context):
    node = Node("test_pub_node")
    yield node
    node.destroy_node()


@pytest.fixture
def empty_state():
    return TransitState()


@pytest.fixture
def populated_state():
    path = TimedPathState(
        path_id="test_path",
        waypoints=[
            WaypointState(lat=33.749, lon=-84.388, alt=0.0),
            WaypointState(lat=33.952, lon=-84.549, alt=50.0, speed_to_next=5.0),
            WaypointState(
                lat=33.774, lon=-84.296, alt=100.0,
                desired_time="2025-04-01T14:30:00Z", speed_to_next=10.0,
            ),
        ],
    )
    env = EnvironmentState(
        obstacles=[ObstacleState(id="obs_001", lat=33.80, lon=-84.40, radius=200.0)],
        keepout_zones=[
            KeepoutZoneState(
                id="koz_001",
                vertices=[
                    {"lat": 33.95, "lon": -84.55},
                    {"lat": 33.96, "lon": -84.55},
                    {"lat": 33.96, "lon": -84.54},
                ],
                min_altitude=None,
                max_altitude=500.0,
            ),
        ],
    )
    return TransitState(path=path, environment=env)


# -- Initialization --


class TestPublisherManagerInit:
    def test_creates_path_publisher(self, test_node, empty_state):
        mgr = PublisherManager(test_node, empty_state)
        assert mgr._path_pub is not None
        assert mgr._path_topic == "/transit/planned_path"

    def test_creates_env_publisher(self, test_node, empty_state):
        mgr = PublisherManager(test_node, empty_state)
        assert mgr._env_pub is not None
        assert mgr._env_topic == "/transit/environment"

    def test_nav_path_disabled_by_default(self, test_node, empty_state):
        mgr = PublisherManager(test_node, empty_state)
        assert mgr._nav_path_pub is None

    def test_nav_path_enabled(self, test_node, empty_state):
        empty_state.publish_config.nav_path_enabled = True
        mgr = PublisherManager(test_node, empty_state)
        assert mgr._nav_path_pub is not None
        assert mgr._nav_path_topic == "/transit/nav_path"


# -- Reconfigure --


class TestReconfigure:
    def test_path_topic_change(self, test_node, empty_state):
        mgr = PublisherManager(test_node, empty_state)
        assert mgr._path_topic == "/transit/planned_path"
        empty_state.publish_config.path_topic = "/custom/path"
        mgr.reconfigure()
        assert mgr._path_topic == "/custom/path"

    def test_same_topic_keeps_publisher(self, test_node, empty_state):
        mgr = PublisherManager(test_node, empty_state)
        old_pub = mgr._path_pub
        mgr.reconfigure()
        assert mgr._path_pub is old_pub

    def test_enable_nav_path(self, test_node, empty_state):
        mgr = PublisherManager(test_node, empty_state)
        assert mgr._nav_path_pub is None
        empty_state.publish_config.nav_path_enabled = True
        mgr.reconfigure()
        assert mgr._nav_path_pub is not None

    def test_disable_nav_path(self, test_node, empty_state):
        empty_state.publish_config.nav_path_enabled = True
        mgr = PublisherManager(test_node, empty_state)
        assert mgr._nav_path_pub is not None
        empty_state.publish_config.nav_path_enabled = False
        mgr.reconfigure()
        assert mgr._nav_path_pub is None
        assert mgr._nav_path_topic == ""

    def test_env_topic_change(self, test_node, empty_state):
        mgr = PublisherManager(test_node, empty_state)
        empty_state.publish_config.env_topic = "/custom/env"
        mgr.reconfigure()
        assert mgr._env_topic == "/custom/env"


# -- Build TimedPath message --


class TestBuildTimedPathMsg:
    def test_empty_path(self, test_node, empty_state):
        mgr = PublisherManager(test_node, empty_state)
        msg = mgr._build_timed_path_msg()
        assert len(msg.waypoints) == 0
        assert msg.header.frame_id == "wgs84"

    def test_waypoint_count(self, test_node, populated_state):
        mgr = PublisherManager(test_node, populated_state)
        msg = mgr._build_timed_path_msg()
        assert len(msg.waypoints) == 3

    def test_waypoint_position_mapping(self, test_node, populated_state):
        mgr = PublisherManager(test_node, populated_state)
        msg = mgr._build_timed_path_msg()
        wp0 = msg.waypoints[0]
        assert wp0.position.latitude == pytest.approx(33.749)
        assert wp0.position.longitude == pytest.approx(-84.388)
        assert wp0.position.altitude == pytest.approx(0.0)

    def test_path_id(self, test_node, populated_state):
        mgr = PublisherManager(test_node, populated_state)
        msg = mgr._build_timed_path_msg()
        assert msg.path_id == "test_path"

    def test_speed_mapping(self, test_node, populated_state):
        mgr = PublisherManager(test_node, populated_state)
        msg = mgr._build_timed_path_msg()
        assert msg.waypoints[0].speed_to_next == pytest.approx(0.0)
        assert msg.waypoints[1].speed_to_next == pytest.approx(5.0)
        assert msg.waypoints[2].speed_to_next == pytest.approx(10.0)

    def test_time_mapping(self, test_node, populated_state):
        mgr = PublisherManager(test_node, populated_state)
        msg = mgr._build_timed_path_msg()
        # Waypoint 0 has no time
        assert msg.waypoints[0].desired_time.sec == 0
        # Waypoint 2 has desired_time set
        assert msg.waypoints[2].desired_time.sec > 0

    def test_header_stamp_set(self, test_node, populated_state):
        mgr = PublisherManager(test_node, populated_state)
        msg = mgr._build_timed_path_msg()
        assert msg.header.stamp.sec > 0 or msg.header.stamp.nanosec > 0

    def test_waypoint_frame_id(self, test_node, populated_state):
        mgr = PublisherManager(test_node, populated_state)
        msg = mgr._build_timed_path_msg()
        assert msg.waypoints[0].header.frame_id == "wgs84"


# -- Build Environment message --


class TestBuildEnvMsg:
    def test_empty_env(self, test_node, empty_state):
        mgr = PublisherManager(test_node, empty_state)
        msg = mgr._build_env_msg()
        assert len(msg.obstacles) == 0
        assert len(msg.keepout_zones) == 0
        assert msg.header.frame_id == "wgs84"

    def test_obstacle_mapping(self, test_node, populated_state):
        mgr = PublisherManager(test_node, populated_state)
        msg = mgr._build_env_msg()
        assert len(msg.obstacles) == 1
        obs = msg.obstacles[0]
        assert obs.id == "obs_001"
        assert obs.position.latitude == pytest.approx(33.80)
        assert obs.position.longitude == pytest.approx(-84.40)
        assert obs.position.altitude == pytest.approx(0.0)
        assert obs.radius == pytest.approx(200.0)

    def test_keepout_zone_mapping(self, test_node, populated_state):
        mgr = PublisherManager(test_node, populated_state)
        msg = mgr._build_env_msg()
        assert len(msg.keepout_zones) == 1
        koz = msg.keepout_zones[0]
        assert koz.id == "koz_001"
        assert len(koz.vertices) == 3
        assert koz.vertices[0].latitude == pytest.approx(33.95)
        assert koz.vertices[0].longitude == pytest.approx(-84.55)

    def test_keepout_altitude_none_becomes_inf(self, test_node, populated_state):
        mgr = PublisherManager(test_node, populated_state)
        msg = mgr._build_env_msg()
        koz = msg.keepout_zones[0]
        assert koz.min_altitude == float("-inf")  # was None
        assert koz.max_altitude == pytest.approx(500.0)  # was 500.0

    def test_keepout_both_none_altitudes(self, test_node, empty_state):
        empty_state.environment.keepout_zones.append(
            KeepoutZoneState(
                id="koz_test",
                vertices=[{"lat": 1.0, "lon": 2.0}],
                min_altitude=None,
                max_altitude=None,
            )
        )
        mgr = PublisherManager(test_node, empty_state)
        msg = mgr._build_env_msg()
        koz = msg.keepout_zones[0]
        assert koz.min_altitude == float("-inf")
        assert koz.max_altitude == float("inf")

    def test_header_stamp_set(self, test_node, populated_state):
        mgr = PublisherManager(test_node, populated_state)
        msg = mgr._build_env_msg()
        assert msg.header.stamp.sec > 0 or msg.header.stamp.nanosec > 0


# -- Publish --


class TestPublishPath:
    def test_publish_when_enabled(self, test_node, populated_state):
        mgr = PublisherManager(test_node, populated_state)
        mgr.publish_path()  # Should not raise

    def test_no_publish_when_disabled(self, test_node, populated_state):
        populated_state.publish_config.path_enabled = False
        mgr = PublisherManager(test_node, populated_state)
        mgr.publish_path()  # Should not raise

    def test_publish_with_nav_path(self, test_node, populated_state):
        populated_state.publish_config.nav_path_enabled = True
        mgr = PublisherManager(test_node, populated_state)
        mgr.publish_path()  # Should publish both

    def test_publish_empty_path(self, test_node, empty_state):
        mgr = PublisherManager(test_node, empty_state)
        mgr.publish_path()  # Should not raise with 0 waypoints


class TestPublishEnvironment:
    def test_publish_when_enabled(self, test_node, populated_state):
        mgr = PublisherManager(test_node, populated_state)
        mgr.publish_environment()  # Should not raise

    def test_no_publish_when_disabled(self, test_node, populated_state):
        populated_state.publish_config.env_enabled = False
        mgr = PublisherManager(test_node, populated_state)
        mgr.publish_environment()  # Should not raise

    def test_publish_empty_env(self, test_node, empty_state):
        mgr = PublisherManager(test_node, empty_state)
        mgr.publish_environment()  # Should not raise


# -- Get Status --


class TestGetStatus:
    def test_has_path_status(self, test_node, empty_state):
        mgr = PublisherManager(test_node, empty_state)
        status = mgr.get_status()
        assert "path" in status
        assert status["path"]["topic"] == "/transit/planned_path"
        assert status["path"]["active"] is True
        assert status["path"]["rate_hz"] == pytest.approx(1.0)
        assert "listeners" in status["path"]

    def test_has_env_status(self, test_node, empty_state):
        mgr = PublisherManager(test_node, empty_state)
        status = mgr.get_status()
        assert "environment" in status
        assert status["environment"]["topic"] == "/transit/environment"
        assert status["environment"]["active"] is True

    def test_no_nav_path_when_disabled(self, test_node, empty_state):
        mgr = PublisherManager(test_node, empty_state)
        status = mgr.get_status()
        assert "nav_path" not in status

    def test_nav_path_when_enabled(self, test_node, empty_state):
        empty_state.publish_config.nav_path_enabled = True
        mgr = PublisherManager(test_node, empty_state)
        status = mgr.get_status()
        assert "nav_path" in status
        assert status["nav_path"]["topic"] == "/transit/nav_path"


# -- _iso_to_time helper --


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

    def test_invalid_iso(self):
        t = _iso_to_time("not-a-date")
        assert t.sec == 0
        assert t.nanosec == 0
