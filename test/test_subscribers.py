"""Tests for transit_backend.subscribers — ROS2 subscription management.

Requires ROS2 message packages (transit_msgs, geographic_msgs, nav_msgs) and rclpy.
These tests run in Docker CI or a sourced ROS2 environment.
"""

import time as _time

import pytest

rclpy = pytest.importorskip("rclpy", reason="ROS2 not available")

import rclpy as _rclpy
from rclpy.node import Node

from transit_backend.subscribers import SubscriptionManager, _time_to_iso
from transit_backend.state import SubscriptionConfig


@pytest.fixture(scope="module")
def ros_context():
    _rclpy.init()
    yield
    _rclpy.shutdown()


@pytest.fixture
def test_node(ros_context):
    node = Node("test_sub_node")
    yield node
    node.destroy_node()


@pytest.fixture
def sub_mgr(test_node):
    return SubscriptionManager(test_node)


# -- Add subscriptions --


class TestAdd:
    def test_add_timed_path(self, sub_mgr):
        cfg = SubscriptionConfig(name="s1", topic="/test/tp", msg_type="TimedPath")
        sub_mgr.add(cfg)
        assert "s1" in sub_mgr._subs

    def test_add_nav_path(self, sub_mgr):
        cfg = SubscriptionConfig(name="s2", topic="/test/np", msg_type="NavPath")
        sub_mgr.add(cfg)
        assert "s2" in sub_mgr._subs

    def test_add_scenario_env(self, sub_mgr):
        cfg = SubscriptionConfig(name="s3", topic="/test/se", msg_type="ScenarioEnvironment")
        sub_mgr.add(cfg)
        assert "s3" in sub_mgr._subs

    def test_add_unknown_type_ignored(self, sub_mgr):
        cfg = SubscriptionConfig(name="s4", topic="/test/bad", msg_type="UnknownType")
        sub_mgr.add(cfg)
        assert "s4" not in sub_mgr._subs

    def test_add_duplicate_removes_old(self, sub_mgr):
        cfg1 = SubscriptionConfig(name="dup", topic="/test/a", msg_type="TimedPath")
        sub_mgr.add(cfg1)
        old_sub = sub_mgr._subs["dup"]
        cfg2 = SubscriptionConfig(name="dup", topic="/test/b", msg_type="TimedPath")
        sub_mgr.add(cfg2)
        assert sub_mgr._subs["dup"] is not old_sub

    def test_callback_set(self, sub_mgr):
        received = []
        sub_mgr.on_data_received = lambda name, data: received.append((name, data))
        cfg = SubscriptionConfig(name="cb", topic="/test/cb", msg_type="TimedPath")
        sub_mgr.add(cfg)
        assert "cb" in sub_mgr._subs


# -- Remove subscriptions --


class TestRemove:
    def test_remove_existing(self, sub_mgr):
        cfg = SubscriptionConfig(name="rem", topic="/test/rem", msg_type="TimedPath")
        sub_mgr.add(cfg)
        sub_mgr.remove("rem")
        assert "rem" not in sub_mgr._subs

    def test_remove_nonexistent(self, sub_mgr):
        sub_mgr.remove("nonexistent")  # Should not raise

    def test_remove_clears_cached_data(self, sub_mgr):
        cfg = SubscriptionConfig(name="clr", topic="/test/clr", msg_type="TimedPath")
        sub_mgr.add(cfg)
        sub_mgr._last_received["clr"] = _time.time()
        sub_mgr._latest_data["clr"] = {"test": True}
        sub_mgr.remove("clr")
        assert "clr" not in sub_mgr._last_received
        assert "clr" not in sub_mgr._latest_data


# -- Update subscriptions --


class TestUpdate:
    def test_topic_change_recreates(self, sub_mgr):
        cfg = SubscriptionConfig(name="upd", topic="/test/old", msg_type="TimedPath")
        sub_mgr.add(cfg)
        old_sub = sub_mgr._subs["upd"]
        cfg.topic = "/test/new"
        sub_mgr.update("upd", cfg, {"topic": "/test/new"})
        assert sub_mgr._subs["upd"] is not old_sub

    def test_msg_type_change_recreates(self, sub_mgr):
        cfg = SubscriptionConfig(name="mt", topic="/test/mt", msg_type="TimedPath")
        sub_mgr.add(cfg)
        old_sub = sub_mgr._subs["mt"]
        cfg.msg_type = "NavPath"
        sub_mgr.update("mt", cfg, {"msg_type": "NavPath"})
        assert sub_mgr._subs["mt"] is not old_sub

    def test_color_change_no_recreate(self, sub_mgr):
        cfg = SubscriptionConfig(name="col", topic="/test/col", msg_type="TimedPath")
        sub_mgr.add(cfg)
        old_sub = sub_mgr._subs["col"]
        sub_mgr.update("col", cfg, {"color": "#FF0000"})
        assert sub_mgr._subs["col"] is old_sub


# -- Latest data --


class TestGetLatestData:
    def test_no_data(self, sub_mgr):
        assert sub_mgr.get_latest_data("none") is None

    def test_with_data(self, sub_mgr):
        sub_mgr._latest_data["test"] = {"lat": 33.0}
        assert sub_mgr.get_latest_data("test") == {"lat": 33.0}


# -- Status --


class TestGetStatus:
    def test_active_receiving(self, sub_mgr):
        cfg = SubscriptionConfig(name="st", topic="/test/st", msg_type="TimedPath")
        sub_mgr.add(cfg)
        sub_mgr._last_received["st"] = _time.time()
        statuses = sub_mgr.get_status({"st": cfg})
        assert len(statuses) == 1
        assert statuses[0]["name"] == "st"
        assert statuses[0]["topic"] == "/test/st"
        assert statuses[0]["active"] is True
        assert statuses[0]["receiving"] is True

    def test_not_receiving_after_timeout(self, sub_mgr):
        cfg = SubscriptionConfig(name="to", topic="/test/to", msg_type="TimedPath")
        sub_mgr.add(cfg)
        sub_mgr._last_received["to"] = _time.time() - 10
        statuses = sub_mgr.get_status({"to": cfg}, timeout=5.0)
        assert statuses[0]["receiving"] is False

    def test_no_subscriptions(self, sub_mgr):
        statuses = sub_mgr.get_status({})
        assert statuses == []

    def test_never_received(self, sub_mgr):
        cfg = SubscriptionConfig(name="nr", topic="/test/nr", msg_type="TimedPath")
        sub_mgr.add(cfg)
        statuses = sub_mgr.get_status({"nr": cfg})
        assert statuses[0]["receiving"] is False
        assert statuses[0]["active"] is True

    def test_not_active_if_not_added(self, sub_mgr):
        cfg = SubscriptionConfig(name="na", topic="/test/na", msg_type="TimedPath")
        statuses = sub_mgr.get_status({"na": cfg})
        assert statuses[0]["active"] is False


# -- Message conversion: TimedPath --


class TestConvertTimedPath:
    def test_basic_conversion(self, sub_mgr):
        from transit_msgs.msg import TimedPath, TimedWaypoint
        from geographic_msgs.msg import GeoPoint
        from builtin_interfaces.msg import Time

        msg = TimedPath()
        msg.path_id = "test_path"
        tw = TimedWaypoint()
        tw.position = GeoPoint(latitude=33.95, longitude=-84.55, altitude=100.0)
        tw.speed_to_next = 5.0
        tw.desired_time = Time()
        msg.waypoints.append(tw)

        result = sub_mgr._convert_timed_path(msg)
        assert result["type"] == "path"
        assert result["path_id"] == "test_path"
        assert len(result["waypoints"]) == 1
        wp = result["waypoints"][0]
        assert wp["lat"] == pytest.approx(33.95)
        assert wp["lon"] == pytest.approx(-84.55)
        assert wp["alt"] == pytest.approx(100.0)
        assert wp["speed_to_next"] == pytest.approx(5.0)

    def test_empty_waypoints(self, sub_mgr):
        from transit_msgs.msg import TimedPath
        msg = TimedPath()
        result = sub_mgr._convert_timed_path(msg)
        assert result["waypoints"] == []
        assert result["path_id"] == ""

    def test_time_conversion(self, sub_mgr):
        from transit_msgs.msg import TimedPath, TimedWaypoint
        from geographic_msgs.msg import GeoPoint
        from builtin_interfaces.msg import Time

        msg = TimedPath()
        tw = TimedWaypoint()
        tw.position = GeoPoint()
        tw.desired_time = Time(sec=1711972800, nanosec=0)
        msg.waypoints.append(tw)

        result = sub_mgr._convert_timed_path(msg)
        assert result["waypoints"][0]["desired_time"] is not None
        assert "2024" in result["waypoints"][0]["desired_time"]

    def test_zero_time_becomes_none(self, sub_mgr):
        from transit_msgs.msg import TimedPath, TimedWaypoint
        from geographic_msgs.msg import GeoPoint
        from builtin_interfaces.msg import Time

        msg = TimedPath()
        tw = TimedWaypoint()
        tw.position = GeoPoint()
        tw.desired_time = Time(sec=0, nanosec=0)
        msg.waypoints.append(tw)

        result = sub_mgr._convert_timed_path(msg)
        assert result["waypoints"][0]["desired_time"] is None

    def test_multiple_waypoints(self, sub_mgr):
        from transit_msgs.msg import TimedPath, TimedWaypoint
        from geographic_msgs.msg import GeoPoint
        from builtin_interfaces.msg import Time

        msg = TimedPath()
        for i in range(5):
            tw = TimedWaypoint()
            tw.position = GeoPoint(latitude=float(i), longitude=float(i))
            tw.desired_time = Time()
            msg.waypoints.append(tw)

        result = sub_mgr._convert_timed_path(msg)
        assert len(result["waypoints"]) == 5


# -- Message conversion: NavPath --


class TestConvertNavPath:
    def test_basic_conversion(self, sub_mgr):
        from nav_msgs.msg import Path as NavPath
        from geometry_msgs.msg import PoseStamped

        msg = NavPath()
        msg.header.frame_id = "wgs84"
        ps = PoseStamped()
        ps.header.frame_id = "wgs84"
        ps.pose.position.x = 33.95  # lat
        ps.pose.position.y = -84.55  # lon
        ps.pose.position.z = 100.0  # alt
        msg.poses.append(ps)

        result = sub_mgr._convert_nav_path(msg)
        assert result["type"] == "path"
        assert result["path_id"] == ""
        assert len(result["waypoints"]) == 1
        assert result["waypoints"][0]["lat"] == pytest.approx(33.95)
        assert result["waypoints"][0]["lon"] == pytest.approx(-84.55)

    def test_empty_path(self, sub_mgr):
        from nav_msgs.msg import Path as NavPath
        msg = NavPath()
        result = sub_mgr._convert_nav_path(msg)
        assert result["waypoints"] == []


# -- Message conversion: ScenarioEnvironment --


class TestConvertEnv:
    def test_obstacle_conversion(self, sub_mgr):
        from transit_msgs.msg import ScenarioEnvironment, Obstacle
        from geographic_msgs.msg import GeoPoint

        msg = ScenarioEnvironment()
        obs = Obstacle()
        obs.id = "obs_1"
        obs.position = GeoPoint(latitude=33.80, longitude=-84.40, altitude=0.0)
        obs.radius = 200.0
        msg.obstacles.append(obs)

        result = sub_mgr._convert_env(msg)
        assert result["type"] == "environment"
        assert len(result["obstacles"]) == 1
        assert result["obstacles"][0]["id"] == "obs_1"
        assert result["obstacles"][0]["lat"] == pytest.approx(33.80)
        assert result["obstacles"][0]["radius"] == pytest.approx(200.0)

    def test_keepout_zone_conversion(self, sub_mgr):
        from transit_msgs.msg import ScenarioEnvironment, KeepoutZone
        from geographic_msgs.msg import GeoPoint

        msg = ScenarioEnvironment()
        koz = KeepoutZone()
        koz.id = "koz_1"
        koz.vertices.append(GeoPoint(latitude=33.95, longitude=-84.55))
        koz.vertices.append(GeoPoint(latitude=33.96, longitude=-84.55))
        koz.vertices.append(GeoPoint(latitude=33.96, longitude=-84.54))
        koz.min_altitude = 100.0
        koz.max_altitude = 500.0
        msg.keepout_zones.append(koz)

        result = sub_mgr._convert_env(msg)
        assert len(result["keepout_zones"]) == 1
        kz = result["keepout_zones"][0]
        assert kz["id"] == "koz_1"
        assert len(kz["vertices"]) == 3
        assert kz["vertices"][0]["lat"] == pytest.approx(33.95)
        assert kz["min_altitude"] == pytest.approx(100.0)
        assert kz["max_altitude"] == pytest.approx(500.0)

    def test_inf_altitude_becomes_none(self, sub_mgr):
        from transit_msgs.msg import ScenarioEnvironment, KeepoutZone

        msg = ScenarioEnvironment()
        koz = KeepoutZone()
        koz.id = "koz_inf"
        koz.min_altitude = float("-inf")
        koz.max_altitude = float("inf")
        msg.keepout_zones.append(koz)

        result = sub_mgr._convert_env(msg)
        kz = result["keepout_zones"][0]
        assert kz["min_altitude"] is None
        assert kz["max_altitude"] is None

    def test_empty_env(self, sub_mgr):
        from transit_msgs.msg import ScenarioEnvironment
        msg = ScenarioEnvironment()
        result = sub_mgr._convert_env(msg)
        assert result["obstacles"] == []
        assert result["keepout_zones"] == []

    def test_multiple_obstacles(self, sub_mgr):
        from transit_msgs.msg import ScenarioEnvironment, Obstacle
        from geographic_msgs.msg import GeoPoint

        msg = ScenarioEnvironment()
        for i in range(3):
            obs = Obstacle()
            obs.id = f"obs_{i}"
            obs.position = GeoPoint(latitude=float(i), longitude=float(i))
            obs.radius = 100.0 * (i + 1)
            msg.obstacles.append(obs)

        result = sub_mgr._convert_env(msg)
        assert len(result["obstacles"]) == 3
        assert result["obstacles"][2]["radius"] == pytest.approx(300.0)


# -- Convert dispatch --


class TestConvertMessage:
    def test_dispatches_timed_path(self, sub_mgr):
        from transit_msgs.msg import TimedPath
        msg = TimedPath()
        msg.path_id = "dispatch_test"
        result = sub_mgr._convert_message(msg, "TimedPath")
        assert result["type"] == "path"
        assert result["path_id"] == "dispatch_test"

    def test_dispatches_scenario_env(self, sub_mgr):
        from transit_msgs.msg import ScenarioEnvironment
        msg = ScenarioEnvironment()
        result = sub_mgr._convert_message(msg, "ScenarioEnvironment")
        assert result["type"] == "environment"

    def test_unknown_type_returns_empty(self, sub_mgr):
        result = sub_mgr._convert_message(None, "Unknown")
        assert result == {}


# -- _time_to_iso helper --


class TestTimeToIso:
    def test_zero_time(self):
        from builtin_interfaces.msg import Time
        t = Time(sec=0, nanosec=0)
        assert _time_to_iso(t) is None

    def test_nonzero_time(self):
        from builtin_interfaces.msg import Time
        t = Time(sec=1711972800, nanosec=0)
        result = _time_to_iso(t)
        assert result is not None
        assert "2024" in result

    def test_with_nanoseconds(self):
        from builtin_interfaces.msg import Time
        t = Time(sec=1711972800, nanosec=500000000)
        result = _time_to_iso(t)
        assert result is not None
