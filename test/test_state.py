"""Tests for transit_backend.state — dataclasses, haversine, undo/redo."""

import math

import pytest

from transit_backend.state import (
    EnvironmentState,
    KeepoutZoneState,
    ObstacleState,
    TimedPathState,
    TransitState,
    UndoManager,
    WaypointState,
    _haversine,
)


class TestWaypointState:
    def test_defaults(self):
        wp = WaypointState(lat=0.0, lon=0.0)
        assert wp.alt == 0.0
        assert wp.desired_time is None
        assert wp.speed_to_next == 0.0

    def test_to_dict(self):
        wp = WaypointState(lat=33.95, lon=-84.55, alt=100.0, desired_time="2025-04-01T14:30:00Z", speed_to_next=5.0)
        d = wp.to_dict()
        assert d["lat"] == 33.95
        assert d["lon"] == -84.55
        assert d["alt"] == 100.0
        assert d["desired_time"] == "2025-04-01T14:30:00Z"
        assert d["speed_to_next"] == 5.0

    def test_to_dict_none_time(self):
        wp = WaypointState(lat=0.0, lon=0.0)
        assert wp.to_dict()["desired_time"] is None


class TestHaversine:
    def test_same_point(self):
        wp = WaypointState(lat=33.95, lon=-84.55)
        assert _haversine(wp, wp) == 0.0

    def test_one_degree_latitude(self):
        """One degree of latitude ~ 111 km."""
        a = WaypointState(lat=0.0, lon=0.0)
        b = WaypointState(lat=1.0, lon=0.0)
        dist = _haversine(a, b)
        assert 110_000 < dist < 112_000

    def test_one_degree_longitude_equator(self):
        """One degree of longitude at equator ~ 111 km."""
        a = WaypointState(lat=0.0, lon=0.0)
        b = WaypointState(lat=0.0, lon=1.0)
        dist = _haversine(a, b)
        assert 110_000 < dist < 112_000

    def test_longitude_shorter_at_higher_lat(self):
        """One degree of longitude at 60N should be ~half of equator."""
        a = WaypointState(lat=60.0, lon=0.0)
        b = WaypointState(lat=60.0, lon=1.0)
        dist = _haversine(a, b)
        assert 50_000 < dist < 60_000

    def test_symmetric(self):
        a = WaypointState(lat=33.95, lon=-84.55)
        b = WaypointState(lat=34.05, lon=-84.45)
        assert abs(_haversine(a, b) - _haversine(b, a)) < 0.001

    def test_small_offset(self):
        """~0.001 deg lat ~ 111 meters."""
        a = WaypointState(lat=33.9500, lon=-84.55)
        b = WaypointState(lat=33.9510, lon=-84.55)
        dist = _haversine(a, b)
        assert 100 < dist < 120


class TestTimedPathState:
    def test_empty_path(self):
        path = TimedPathState()
        assert path.total_distance_m() == 0.0
        assert path.estimated_time_s() == 0.0
        assert path.waypoints == []

    def test_single_waypoint(self):
        path = TimedPathState(waypoints=[WaypointState(lat=0.0, lon=0.0)])
        assert path.total_distance_m() == 0.0

    def test_distance_two_points(self):
        path = TimedPathState(waypoints=[
            WaypointState(lat=0.0, lon=0.0),
            WaypointState(lat=1.0, lon=0.0),
        ])
        assert 110_000 < path.total_distance_m() < 112_000

    def test_distance_three_points(self, waypoint_atlanta, waypoint_marietta, waypoint_decatur):
        path = TimedPathState(waypoints=[waypoint_atlanta, waypoint_marietta, waypoint_decatur])
        # Should be sum of two segments, each > 0
        assert path.total_distance_m() > 0

    def test_estimated_time_no_speed(self):
        """Zero speed means unconstrained — contributes 0 to time estimate."""
        path = TimedPathState(waypoints=[
            WaypointState(lat=0.0, lon=0.0, speed_to_next=0.0),
            WaypointState(lat=1.0, lon=0.0),
        ])
        assert path.estimated_time_s() == 0.0

    def test_estimated_time_with_speed(self):
        path = TimedPathState(waypoints=[
            WaypointState(lat=0.0, lon=0.0, speed_to_next=10.0),
            WaypointState(lat=0.0, lon=0.001),  # ~111 meters
        ])
        time_s = path.estimated_time_s()
        assert time_s > 0
        # At 10 m/s over ~111 m, should be ~11 seconds
        assert 10 < time_s < 13

    def test_to_dict_includes_computed_fields(self, three_waypoint_path):
        d = three_waypoint_path.to_dict()
        assert "total_distance_m" in d
        assert "estimated_time_s" in d
        assert "waypoints" in d
        assert len(d["waypoints"]) == 3

    def test_path_id_auto_generated(self):
        p1 = TimedPathState()
        p2 = TimedPathState()
        assert p1.path_id.startswith("user_path_")
        assert p1.path_id != p2.path_id


class TestObstacleState:
    def test_to_dict(self, sample_obstacle):
        d = sample_obstacle.to_dict()
        assert d["id"] == "obs_001"
        assert d["lat"] == 33.80
        assert d["radius"] == 200.0


class TestKeepoutZoneState:
    def test_to_dict(self, sample_keepout):
        d = sample_keepout.to_dict()
        assert d["id"] == "koz_001"
        assert len(d["vertices"]) == 3
        assert d["min_altitude"] is None
        assert d["max_altitude"] is None

    def test_with_altitude_bounds(self):
        koz = KeepoutZoneState(
            id="koz_alt",
            vertices=[{"lat": 0, "lon": 0}],
            min_altitude=10.0,
            max_altitude=500.0,
        )
        d = koz.to_dict()
        assert d["min_altitude"] == 10.0
        assert d["max_altitude"] == 500.0


class TestEnvironmentState:
    def test_empty(self):
        env = EnvironmentState()
        d = env.to_dict()
        assert d["obstacles"] == []
        assert d["keepout_zones"] == []

    def test_to_dict(self, sample_environment):
        d = sample_environment.to_dict()
        assert len(d["obstacles"]) == 1
        assert len(d["keepout_zones"]) == 1


class TestUndoManager:
    def test_initial_state(self, undo_mgr):
        assert not undo_mgr.can_undo
        assert not undo_mgr.can_redo

    def test_push_enables_undo(self, undo_mgr, state):
        undo_mgr.push(state)
        assert undo_mgr.can_undo
        assert not undo_mgr.can_redo

    def test_undo_restores_state(self, undo_mgr, state):
        # Save initial empty state
        undo_mgr.push(state)
        # Mutate
        state.path.waypoints.append(WaypointState(lat=1.0, lon=2.0))
        assert len(state.path.waypoints) == 1

        # Undo should restore empty state
        assert undo_mgr.undo(state)
        assert len(state.path.waypoints) == 0

    def test_redo_after_undo(self, undo_mgr, state):
        undo_mgr.push(state)
        state.path.waypoints.append(WaypointState(lat=1.0, lon=2.0))
        undo_mgr.undo(state)
        assert len(state.path.waypoints) == 0

        assert undo_mgr.can_redo
        assert undo_mgr.redo(state)
        assert len(state.path.waypoints) == 1

    def test_undo_empty_stack_returns_false(self, undo_mgr, state):
        assert not undo_mgr.undo(state)

    def test_redo_empty_stack_returns_false(self, undo_mgr, state):
        assert not undo_mgr.redo(state)

    def test_mutation_clears_redo(self, undo_mgr, state):
        undo_mgr.push(state)
        state.path.waypoints.append(WaypointState(lat=1.0, lon=2.0))
        undo_mgr.undo(state)
        assert undo_mgr.can_redo

        # New mutation should clear redo stack
        undo_mgr.push(state)
        assert not undo_mgr.can_redo

    def test_max_depth(self):
        mgr = UndoManager(max_depth=3)
        state = TransitState()
        for i in range(5):
            mgr.push(state)
            state.path.waypoints.append(WaypointState(lat=float(i), lon=0.0))
        # Only 3 undos should be possible
        assert mgr.undo(state)
        assert mgr.undo(state)
        assert mgr.undo(state)
        assert not mgr.undo(state)

    def test_undo_is_deep_copy(self, undo_mgr, state):
        """Undo snapshots should be independent of current state."""
        state.path.waypoints.append(WaypointState(lat=1.0, lon=2.0))
        undo_mgr.push(state)
        state.path.waypoints.append(WaypointState(lat=3.0, lon=4.0))
        assert len(state.path.waypoints) == 2

        undo_mgr.undo(state)
        assert len(state.path.waypoints) == 1
        assert state.path.waypoints[0].lat == 1.0

    def test_environment_also_restored(self, undo_mgr, state):
        undo_mgr.push(state)
        state.environment.obstacles.append(ObstacleState(id="obs", lat=0, lon=0, radius=50))
        assert len(state.environment.obstacles) == 1

        undo_mgr.undo(state)
        assert len(state.environment.obstacles) == 0
