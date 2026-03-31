"""Tests for transit_backend.transit_node — main async ROS2 node orchestration.

Requires full ROS2 environment (rclpy, transit_msgs, etc.).
These tests run in Docker CI or a sourced ROS2 environment.
"""

import asyncio
import json
from unittest.mock import AsyncMock, patch

import pytest

rclpy = pytest.importorskip("rclpy", reason="ROS2 not available")

import rclpy as _rclpy

from transit_backend.transit_node import TransitNode
from transit_backend.state import SubscriptionConfig, WaypointState


def _noop_ensure_future(coro):
    """Close a coroutine without scheduling it, avoiding 'never awaited' warnings."""
    coro.close()
    return None


@pytest.fixture(scope="module")
def ros_context():
    _rclpy.init()
    yield
    _rclpy.shutdown()


@pytest.fixture
def node(ros_context):
    n = TransitNode()
    yield n
    n.destroy_node()


# -- Build state update --


class TestBuildStateUpdate:
    def test_structure(self, node):
        msg = node._build_state_update()
        assert msg["type"] == "state_update"
        assert "state" in msg
        assert "path" in msg["state"]
        assert "environment" in msg["state"]
        assert "can_undo" in msg["state"]
        assert "can_redo" in msg["state"]

    def test_empty_state(self, node):
        msg = node._build_state_update()
        assert msg["state"]["path"]["waypoints"] == []
        assert msg["state"]["environment"]["obstacles"] == []
        assert msg["state"]["environment"]["keepout_zones"] == []
        assert msg["state"]["can_undo"] is False
        assert msg["state"]["can_redo"] is False

    def test_with_waypoints(self, node):
        node.state.path.waypoints.append(WaypointState(lat=33.0, lon=-84.0))
        msg = node._build_state_update()
        assert len(msg["state"]["path"]["waypoints"]) == 1
        assert msg["state"]["path"]["waypoints"][0]["lat"] == 33.0
        node.state.path.waypoints.clear()

    def test_undo_redo_flags(self, node):
        node.undo_mgr.push(node.state)
        node.state.path.waypoints.append(WaypointState(lat=1.0, lon=1.0))
        msg = node._build_state_update()
        assert msg["state"]["can_undo"] is True
        # Undo to check redo
        node.undo_mgr.undo(node.state)
        msg = node._build_state_update()
        assert msg["state"]["can_redo"] is True
        node.state.path.waypoints.clear()

    def test_serializable(self, node):
        """State update must be JSON-serializable."""
        msg = node._build_state_update()
        raw = json.dumps(msg)
        assert isinstance(raw, str)


# -- Broadcast --


class TestBroadcast:
    @pytest.mark.asyncio
    async def test_sends_to_all_clients(self, node):
        ws1 = AsyncMock()
        ws2 = AsyncMock()
        node.ws_clients = {ws1, ws2}
        await node._broadcast({"type": "test", "data": 42})
        ws1.send.assert_called_once()
        ws2.send.assert_called_once()
        sent = json.loads(ws1.send.call_args[0][0])
        assert sent["type"] == "test"
        assert sent["data"] == 42
        node.ws_clients.clear()

    @pytest.mark.asyncio
    async def test_removes_dead_clients(self, node):
        ws_alive = AsyncMock()
        ws_dead = AsyncMock()
        ws_dead.send.side_effect = Exception("disconnected")
        node.ws_clients = {ws_alive, ws_dead}
        await node._broadcast({"type": "test"})
        assert ws_dead not in node.ws_clients
        assert ws_alive in node.ws_clients
        node.ws_clients.clear()

    @pytest.mark.asyncio
    async def test_empty_clients(self, node):
        node.ws_clients = set()
        await node._broadcast({"type": "test"})  # Should not raise

    @pytest.mark.asyncio
    async def test_all_dead_clients(self, node):
        ws1 = AsyncMock()
        ws1.send.side_effect = Exception("gone")
        ws2 = AsyncMock()
        ws2.send.side_effect = Exception("gone")
        node.ws_clients = {ws1, ws2}
        await node._broadcast({"type": "test"})
        assert len(node.ws_clients) == 0


class TestBroadcastState:
    @pytest.mark.asyncio
    async def test_broadcasts_state_update(self, node):
        ws = AsyncMock()
        node.ws_clients = {ws}
        await node._broadcast_state()
        ws.send.assert_called_once()
        sent = json.loads(ws.send.call_args[0][0])
        assert sent["type"] == "state_update"
        node.ws_clients.clear()


# -- Schedule deviation --


class TestScheduleDeviation:
    def test_sets_pending_flag(self, node):
        node._deviation_pending = False
        node._schedule_deviation()
        assert node._deviation_pending is True


# -- Compute deviations --


class TestComputeAndSendDeviations:
    @pytest.mark.asyncio
    async def test_no_targets_no_broadcast(self, node):
        node.state.deviation_config.targets = []
        ws = AsyncMock()
        node.ws_clients = {ws}
        await node._compute_and_send_deviations()
        ws.send.assert_not_called()
        node.ws_clients.clear()

    @pytest.mark.asyncio
    async def test_empty_user_path_no_broadcast(self, node):
        node.state.deviation_config.targets = ["sub1"]
        node.state.deviation_config.reference = "user_path"
        node.state.path.waypoints.clear()
        ws = AsyncMock()
        node.ws_clients = {ws}
        await node._compute_and_send_deviations()
        ws.send.assert_not_called()
        node.ws_clients.clear()

    @pytest.mark.asyncio
    async def test_missing_target_data_skipped(self, node):
        node.state.path.waypoints = [
            WaypointState(lat=33.0, lon=-84.0),
            WaypointState(lat=34.0, lon=-84.0),
        ]
        node.state.deviation_config.reference = "user_path"
        node.state.deviation_config.targets = ["missing_sub"]
        node._subscribed_path_data.clear()
        ws = AsyncMock()
        node.ws_clients = {ws}
        await node._compute_and_send_deviations()
        # No target data → no comparisons → no broadcast
        ws.send.assert_not_called()
        node.state.path.waypoints.clear()
        node.ws_clients.clear()

    @pytest.mark.asyncio
    async def test_with_valid_target(self, node):
        node.state.path.waypoints = [
            WaypointState(lat=33.0, lon=-84.0),
            WaypointState(lat=34.0, lon=-84.0),
        ]
        node.state.deviation_config.reference = "user_path"
        node.state.deviation_config.targets = ["sub1"]
        node._subscribed_path_data["sub1"] = [
            {"lat": 33.01, "lon": -84.0},
            {"lat": 34.01, "lon": -84.0},
        ]
        ws = AsyncMock()
        node.ws_clients = {ws}
        await node._compute_and_send_deviations()
        ws.send.assert_called_once()
        sent = json.loads(ws.send.call_args[0][0])
        assert sent["type"] == "deviation_update"
        assert len(sent["comparisons"]) == 1
        assert sent["comparisons"][0]["reference"] == "user_path"
        assert sent["comparisons"][0]["target"] == "sub1"
        node.state.path.waypoints.clear()
        node._subscribed_path_data.clear()
        node.ws_clients.clear()

    @pytest.mark.asyncio
    async def test_subscribed_reference(self, node):
        """Use a subscribed path as reference instead of user_path."""
        node.state.deviation_config.reference = "ref_sub"
        node.state.deviation_config.targets = ["target_sub"]
        node._subscribed_path_data["ref_sub"] = [
            {"lat": 33.0, "lon": -84.0},
            {"lat": 34.0, "lon": -84.0},
        ]
        node._subscribed_path_data["target_sub"] = [
            {"lat": 33.01, "lon": -84.0},
            {"lat": 34.01, "lon": -84.0},
        ]
        ws = AsyncMock()
        node.ws_clients = {ws}
        await node._compute_and_send_deviations()
        ws.send.assert_called_once()
        sent = json.loads(ws.send.call_args[0][0])
        assert sent["comparisons"][0]["reference"] == "ref_sub"
        node._subscribed_path_data.clear()
        node.ws_clients.clear()


# -- On subscribed data --


class TestOnSubscribedData:
    def test_caches_path_data(self, node):
        node.state.subscriptions["sub1"] = SubscriptionConfig(
            name="sub1", topic="/test", msg_type="TimedPath",
        )
        data = {"type": "path", "path_id": "p1", "waypoints": [{"lat": 1.0, "lon": 2.0}]}
        with patch("asyncio.ensure_future", _noop_ensure_future):
            node._on_subscribed_data("sub1", data)
        assert "sub1" in node._subscribed_path_data
        assert node._subscribed_path_data["sub1"] == [{"lat": 1.0, "lon": 2.0}]
        del node.state.subscriptions["sub1"]
        node._subscribed_path_data.clear()

    def test_env_data_not_cached_as_path(self, node):
        node.state.subscriptions["sub1"] = SubscriptionConfig(
            name="sub1", topic="/test", msg_type="ScenarioEnvironment",
        )
        data = {"type": "environment", "obstacles": [], "keepout_zones": []}
        with patch("asyncio.ensure_future", _noop_ensure_future):
            node._on_subscribed_data("sub1", data)
        assert "sub1" not in node._subscribed_path_data
        del node.state.subscriptions["sub1"]

    def test_unknown_subscription_ignored(self, node):
        data = {"type": "path", "waypoints": []}
        # No subscription config for "nonexistent"
        with patch("asyncio.ensure_future", _noop_ensure_future):
            node._on_subscribed_data("nonexistent", data)
        assert "nonexistent" not in node._subscribed_path_data

    def test_schedules_deviation_on_path(self, node):
        node.state.subscriptions["sub1"] = SubscriptionConfig(
            name="sub1", topic="/test", msg_type="TimedPath",
        )
        node._deviation_pending = False
        data = {"type": "path", "path_id": "p1", "waypoints": []}
        with patch("asyncio.ensure_future", _noop_ensure_future):
            node._on_subscribed_data("sub1", data)
        assert node._deviation_pending is True
        del node.state.subscriptions["sub1"]
        node._subscribed_path_data.clear()

    def test_path_broadcast_message_structure(self, node):
        node.state.subscriptions["sub1"] = SubscriptionConfig(
            name="sub1", topic="/test/topic", msg_type="TimedPath",
        )
        data = {"type": "path", "path_id": "p1", "waypoints": [{"lat": 1.0, "lon": 2.0}]}
        with patch("asyncio.ensure_future", _noop_ensure_future):
            node._on_subscribed_data("sub1", data)
        assert node._subscribed_path_data["sub1"] == [{"lat": 1.0, "lon": 2.0}]
        del node.state.subscriptions["sub1"]
        node._subscribed_path_data.clear()

    def test_env_broadcast_message(self, node):
        node.state.subscriptions["env1"] = SubscriptionConfig(
            name="env1", topic="/test/env", msg_type="ScenarioEnvironment",
        )
        data = {
            "type": "environment",
            "obstacles": [{"id": "o1", "lat": 33.0, "lon": -84.0, "radius": 100.0}],
            "keepout_zones": [],
        }
        with patch("asyncio.ensure_future", _noop_ensure_future):
            node._on_subscribed_data("env1", data)
        # Environment data should NOT be cached in path data
        assert "env1" not in node._subscribed_path_data
        del node.state.subscriptions["env1"]


# -- Callbacks --


class TestCallbacks:
    def test_on_state_changed(self, node):
        with patch("asyncio.ensure_future", _noop_ensure_future):
            node._on_state_changed()
        assert node._deviation_pending is True

    def test_on_config_changed(self, node):
        with patch("asyncio.ensure_future", _noop_ensure_future):
            node._on_config_changed()

    def test_on_subscription_added(self, node):
        cfg = SubscriptionConfig(name="cb_test", topic="/cb/test", msg_type="TimedPath")
        node._on_subscription_added(cfg)
        assert "cb_test" in node.sub_mgr._subs
        node.sub_mgr.remove("cb_test")

    def test_on_subscription_removed(self, node):
        cfg = SubscriptionConfig(name="rm_test", topic="/rm/test", msg_type="TimedPath")
        node._on_subscription_added(cfg)
        node._subscribed_path_data["rm_test"] = [{"lat": 1.0}]
        node._on_subscription_removed("rm_test")
        assert "rm_test" not in node.sub_mgr._subs
        assert "rm_test" not in node._subscribed_path_data

    def test_on_subscription_updated(self, node):
        cfg = SubscriptionConfig(name="upd_test", topic="/upd/test", msg_type="TimedPath")
        node.state.subscriptions["upd_test"] = cfg
        node._on_subscription_added(cfg)
        old_sub = node.sub_mgr._subs["upd_test"]
        cfg.topic = "/upd/new"
        node._on_subscription_updated("upd_test", {"topic": "/upd/new"})
        assert node.sub_mgr._subs["upd_test"] is not old_sub
        node.sub_mgr.remove("upd_test")
        del node.state.subscriptions["upd_test"]

    def test_on_deviation_config_changed(self, node):
        node._deviation_pending = False
        node._on_deviation_config_changed()
        assert node._deviation_pending is True


# -- Node initialization --


class TestNodeInit:
    def test_default_ports(self, node):
        assert node.ws_port == 8765
        assert node.http_port == 8080

    def test_default_save_dir(self, node):
        assert node.save_dir == "/data/paths"

    def test_state_initialized(self, node):
        assert node.state is not None
        assert node.state.path is not None
        assert node.state.environment is not None

    def test_managers_initialized(self, node):
        assert node.pub_mgr is not None
        assert node.sub_mgr is not None
        assert node.cmd_handler is not None
        assert node.undo_mgr is not None

    def test_ws_clients_empty(self, node):
        assert len(node.ws_clients) == 0

    def test_publish_config_from_params(self, node):
        cfg = node.state.publish_config
        assert cfg.path_topic == "/transit/planned_path"
        assert cfg.path_rate_hz == pytest.approx(1.0)
        assert cfg.path_enabled is True
        assert cfg.nav_path_enabled is False
        assert cfg.env_topic == "/transit/environment"
        assert cfg.env_enabled is True

    def test_cmd_handler_callbacks_wired(self, node):
        assert node.cmd_handler.on_state_changed is not None
        assert node.cmd_handler.on_config_changed is not None
        assert node.cmd_handler.on_subscription_added is not None

    def test_sub_mgr_callback_wired(self, node):
        assert node.sub_mgr.on_data_received == node._on_subscribed_data
