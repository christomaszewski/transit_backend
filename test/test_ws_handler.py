"""Tests for transit_backend.ws_handler — command dispatch and state mutations."""

import pytest

from transit_backend.state import TransitState, UndoManager, WaypointState


class TestPathCommands:
    def test_add_waypoint_append(self, cmd_handler):
        state = cmd_handler.state
        cmd_handler.handle({"type": "add_waypoint", "lat": 33.95, "lon": -84.55, "alt": 0.0, "index": None})
        assert len(state.path.waypoints) == 1
        assert state.path.waypoints[0].lat == 33.95

    def test_add_waypoint_insert(self, cmd_handler):
        state = cmd_handler.state
        cmd_handler.handle({"type": "add_waypoint", "lat": 1.0, "lon": 1.0, "alt": 0.0, "index": None})
        cmd_handler.handle({"type": "add_waypoint", "lat": 3.0, "lon": 3.0, "alt": 0.0, "index": None})
        cmd_handler.handle({"type": "add_waypoint", "lat": 2.0, "lon": 2.0, "alt": 0.0, "index": 1})
        assert len(state.path.waypoints) == 3
        assert state.path.waypoints[1].lat == 2.0

    def test_move_waypoint(self, populated_cmd_handler):
        state = populated_cmd_handler.state
        old_lon = state.path.waypoints[0].lon
        populated_cmd_handler.handle({"type": "move_waypoint", "index": 0, "lat": 99.0, "lon": 99.0})
        assert state.path.waypoints[0].lat == 99.0
        assert state.path.waypoints[0].lon == 99.0

    def test_move_waypoint_invalid_index(self, populated_cmd_handler):
        """Moving an out-of-range index should not crash."""
        populated_cmd_handler.handle({"type": "move_waypoint", "index": 999, "lat": 0.0, "lon": 0.0})

    def test_delete_waypoint(self, populated_cmd_handler):
        state = populated_cmd_handler.state
        original_len = len(state.path.waypoints)
        populated_cmd_handler.handle({"type": "delete_waypoint", "index": 1})
        assert len(state.path.waypoints) == original_len - 1

    def test_delete_waypoint_invalid_index(self, populated_cmd_handler):
        state = populated_cmd_handler.state
        original_len = len(state.path.waypoints)
        populated_cmd_handler.handle({"type": "delete_waypoint", "index": 999})
        assert len(state.path.waypoints) == original_len

    def test_set_waypoint_time(self, populated_cmd_handler):
        populated_cmd_handler.handle(
            {"type": "set_waypoint_time", "index": 0, "desired_time": "2025-06-01T12:00:00Z"}
        )
        assert populated_cmd_handler.state.path.waypoints[0].desired_time == "2025-06-01T12:00:00Z"

    def test_set_waypoint_alt(self, populated_cmd_handler):
        populated_cmd_handler.handle({"type": "set_waypoint_alt", "index": 0, "alt": 250.0})
        assert populated_cmd_handler.state.path.waypoints[0].alt == 250.0

    def test_set_waypoint_speed(self, populated_cmd_handler):
        populated_cmd_handler.handle({"type": "set_waypoint_speed", "index": 0, "speed_to_next": 15.0})
        assert populated_cmd_handler.state.path.waypoints[0].speed_to_next == 15.0

    def test_reverse_path(self, populated_cmd_handler):
        state = populated_cmd_handler.state
        first_lat = state.path.waypoints[0].lat
        last_lat = state.path.waypoints[-1].lat
        populated_cmd_handler.handle({"type": "reverse_path"})
        assert state.path.waypoints[0].lat == last_lat
        assert state.path.waypoints[-1].lat == first_lat

    def test_clear_path(self, populated_cmd_handler):
        populated_cmd_handler.handle({"type": "clear_path"})
        assert len(populated_cmd_handler.state.path.waypoints) == 0


class TestEnvironmentCommands:
    def test_add_obstacle(self, cmd_handler):
        cmd_handler.handle({"type": "add_obstacle", "lat": 33.95, "lon": -84.55, "radius": 150.0})
        assert len(cmd_handler.state.environment.obstacles) == 1
        obs = cmd_handler.state.environment.obstacles[0]
        assert obs.lat == 33.95
        assert obs.radius == 150.0
        assert obs.id.startswith("obs_")

    def test_add_obstacle_default_radius(self, cmd_handler):
        cmd_handler.handle({"type": "add_obstacle", "lat": 0.0, "lon": 0.0})
        assert cmd_handler.state.environment.obstacles[0].radius == 100.0

    def test_move_obstacle(self, populated_cmd_handler):
        state = populated_cmd_handler.state
        obs_id = state.environment.obstacles[0].id
        populated_cmd_handler.handle({"type": "move_obstacle", "id": obs_id, "lat": 99.0, "lon": 99.0})
        assert state.environment.obstacles[0].lat == 99.0

    def test_resize_obstacle(self, populated_cmd_handler):
        state = populated_cmd_handler.state
        obs_id = state.environment.obstacles[0].id
        populated_cmd_handler.handle({"type": "resize_obstacle", "id": obs_id, "radius": 500.0})
        assert state.environment.obstacles[0].radius == 500.0

    def test_delete_obstacle(self, populated_cmd_handler):
        state = populated_cmd_handler.state
        obs_id = state.environment.obstacles[0].id
        populated_cmd_handler.handle({"type": "delete_obstacle", "id": obs_id})
        assert len(state.environment.obstacles) == 0

    def test_add_keepout_zone(self, cmd_handler):
        vertices = [{"lat": 1.0, "lon": 1.0}, {"lat": 2.0, "lon": 1.0}, {"lat": 2.0, "lon": 2.0}]
        cmd_handler.handle({"type": "add_keepout_zone", "vertices": vertices})
        assert len(cmd_handler.state.environment.keepout_zones) == 1
        koz = cmd_handler.state.environment.keepout_zones[0]
        assert len(koz.vertices) == 3
        assert koz.id.startswith("koz_")

    def test_move_keepout_vertex(self, populated_cmd_handler):
        state = populated_cmd_handler.state
        koz_id = state.environment.keepout_zones[0].id
        populated_cmd_handler.handle({
            "type": "move_keepout_vertex",
            "zone_id": koz_id,
            "vertex_index": 1,
            "lat": 99.0,
            "lon": 99.0,
        })
        assert state.environment.keepout_zones[0].vertices[1]["lat"] == 99.0

    def test_delete_keepout_zone(self, populated_cmd_handler):
        state = populated_cmd_handler.state
        koz_id = state.environment.keepout_zones[0].id
        populated_cmd_handler.handle({"type": "delete_keepout_zone", "zone_id": koz_id})
        assert len(state.environment.keepout_zones) == 0


class TestUndoRedoCommands:
    def test_undo_after_add(self, cmd_handler):
        cmd_handler.handle({"type": "add_waypoint", "lat": 1.0, "lon": 1.0, "alt": 0.0, "index": None})
        assert len(cmd_handler.state.path.waypoints) == 1
        cmd_handler.handle({"type": "undo"})
        assert len(cmd_handler.state.path.waypoints) == 0

    def test_redo_after_undo(self, cmd_handler):
        cmd_handler.handle({"type": "add_waypoint", "lat": 1.0, "lon": 1.0, "alt": 0.0, "index": None})
        cmd_handler.handle({"type": "undo"})
        cmd_handler.handle({"type": "redo"})
        assert len(cmd_handler.state.path.waypoints) == 1

    def test_undo_multiple_steps(self, cmd_handler):
        for i in range(5):
            cmd_handler.handle({"type": "add_waypoint", "lat": float(i), "lon": 0.0, "alt": 0.0, "index": None})
        assert len(cmd_handler.state.path.waypoints) == 5
        for _ in range(3):
            cmd_handler.handle({"type": "undo"})
        assert len(cmd_handler.state.path.waypoints) == 2

    def test_undo_environment(self, cmd_handler):
        cmd_handler.handle({"type": "add_obstacle", "lat": 0.0, "lon": 0.0, "radius": 50.0})
        assert len(cmd_handler.state.environment.obstacles) == 1
        cmd_handler.handle({"type": "undo"})
        assert len(cmd_handler.state.environment.obstacles) == 0


class TestConfigCommands:
    def test_set_publish_config(self, cmd_handler):
        cmd_handler.handle({
            "type": "set_publish_config",
            "config": {
                "path_topic": "/custom/path",
                "path_rate_hz": 5.0,
                "path_enabled": False,
            },
        })
        cfg = cmd_handler.state.publish_config
        assert cfg.path_topic == "/custom/path"
        assert cfg.path_rate_hz == 5.0
        assert cfg.path_enabled is False
        # Unset fields should keep defaults
        assert cfg.env_enabled is True

    def test_add_subscription(self, cmd_handler):
        cmd_handler.handle({
            "type": "add_subscription",
            "config": {
                "name": "Test Sub",
                "topic": "/test/path",
                "msg_type": "TimedPath",
                "color": "#00FF00",
            },
        })
        assert "Test Sub" in cmd_handler.state.subscriptions
        sub = cmd_handler.state.subscriptions["Test Sub"]
        assert sub.topic == "/test/path"
        assert sub.color == "#00FF00"

    def test_update_subscription(self, cmd_handler):
        cmd_handler.handle({
            "type": "add_subscription",
            "config": {"name": "S1", "topic": "/t", "msg_type": "TimedPath"},
        })
        cmd_handler.handle({
            "type": "update_subscription",
            "name": "S1",
            "updates": {"color": "#FF0000", "visible": False},
        })
        sub = cmd_handler.state.subscriptions["S1"]
        assert sub.color == "#FF0000"
        assert sub.visible is False

    def test_remove_subscription(self, cmd_handler):
        cmd_handler.handle({
            "type": "add_subscription",
            "config": {"name": "S1", "topic": "/t", "msg_type": "TimedPath"},
        })
        cmd_handler.handle({"type": "remove_subscription", "name": "S1"})
        assert "S1" not in cmd_handler.state.subscriptions


class TestDeviationConfig:
    def test_set_deviation_config(self, cmd_handler):
        cmd_handler.handle({
            "type": "set_deviation_config",
            "config": {
                "mode": "pairwise",
                "reference": "user_path",
                "targets": ["Sub A"],
                "show_map_visualization": False,
            },
        })
        dc = cmd_handler.state.deviation_config
        assert dc.mode == "pairwise"
        assert dc.targets == ["Sub A"]
        assert dc.show_map_visualization is False


class TestUnknownCommand:
    def test_unknown_type_returns_none(self, cmd_handler):
        result = cmd_handler.handle({"type": "nonexistent_command"})
        assert result is None
