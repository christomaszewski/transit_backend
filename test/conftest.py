"""Shared fixtures for transit_backend tests."""

import os
import tempfile

import pytest

from transit_backend.state import (
    DeviationConfig,
    EnvironmentState,
    KeepoutZoneState,
    ObstacleState,
    PublishConfig,
    SubscriptionConfig,
    TimedPathState,
    TransitState,
    UndoManager,
    WaypointState,
)
from transit_backend.ws_handler import CommandHandler


@pytest.fixture
def waypoint_atlanta():
    return WaypointState(lat=33.7490, lon=-84.3880, alt=0.0)


@pytest.fixture
def waypoint_marietta():
    return WaypointState(lat=33.9526, lon=-84.5499, alt=50.0, speed_to_next=5.0)


@pytest.fixture
def waypoint_decatur():
    return WaypointState(
        lat=33.7748,
        lon=-84.2963,
        alt=100.0,
        desired_time="2025-04-01T14:30:00Z",
        speed_to_next=10.0,
    )


@pytest.fixture
def three_waypoint_path(waypoint_atlanta, waypoint_marietta, waypoint_decatur):
    return TimedPathState(
        path_id="test_path",
        waypoints=[waypoint_atlanta, waypoint_marietta, waypoint_decatur],
    )


@pytest.fixture
def sample_obstacle():
    return ObstacleState(id="obs_001", lat=33.80, lon=-84.40, radius=200.0)


@pytest.fixture
def sample_keepout():
    return KeepoutZoneState(
        id="koz_001",
        vertices=[
            {"lat": 33.95, "lon": -84.55},
            {"lat": 33.96, "lon": -84.55},
            {"lat": 33.96, "lon": -84.54},
        ],
        min_altitude=None,
        max_altitude=None,
    )


@pytest.fixture
def sample_environment(sample_obstacle, sample_keepout):
    return EnvironmentState(
        obstacles=[sample_obstacle],
        keepout_zones=[sample_keepout],
    )


@pytest.fixture
def state():
    return TransitState()


@pytest.fixture
def populated_state(three_waypoint_path, sample_environment):
    return TransitState(
        path=three_waypoint_path,
        environment=sample_environment,
    )


@pytest.fixture
def undo_mgr():
    return UndoManager()


@pytest.fixture
def save_dir():
    with tempfile.TemporaryDirectory() as tmpdir:
        yield tmpdir


@pytest.fixture
def cmd_handler(state, undo_mgr, save_dir):
    """CommandHandler wired up with fresh state, undo manager, and temp save dir."""
    handler = CommandHandler(state, undo_mgr, save_dir)
    # Wire no-op callbacks
    handler.on_state_changed = lambda: None
    handler.on_config_changed = lambda: None
    handler.on_subscription_added = lambda cfg: None
    handler.on_subscription_updated = lambda name, updates: None
    handler.on_subscription_removed = lambda name: None
    handler.on_deviation_config_changed = lambda: None
    return handler


@pytest.fixture
def populated_cmd_handler(populated_state, undo_mgr, save_dir):
    """CommandHandler with pre-populated path and environment."""
    handler = CommandHandler(populated_state, undo_mgr, save_dir)
    handler.on_state_changed = lambda: None
    handler.on_config_changed = lambda: None
    handler.on_subscription_added = lambda cfg: None
    handler.on_subscription_updated = lambda name, updates: None
    handler.on_subscription_removed = lambda name: None
    handler.on_deviation_config_changed = lambda: None
    return handler
