"""Tests for transit_backend.file_io — JSON save/load/list/upload."""

import json
import os

import pytest

from transit_backend.file_io import (
    list_saved_paths,
    load_path,
    parse_upload,
    save_path,
)
from transit_backend.state import (
    EnvironmentState,
    KeepoutZoneState,
    ObstacleState,
    TimedPathState,
    TransitState,
    WaypointState,
)


class TestSavePath:
    def test_creates_file(self, populated_state, save_dir):
        save_path(populated_state, "test.json", save_dir)
        assert os.path.exists(os.path.join(save_dir, "test.json"))

    def test_json_structure(self, populated_state, save_dir):
        data = save_path(populated_state, "test.json", save_dir)
        assert data["version"] == 1
        assert data["tool"] == "TRANSIT"
        assert "created" in data
        assert "modified" in data
        assert "path" in data
        assert "environment" in data

    def test_path_data(self, populated_state, save_dir):
        data = save_path(populated_state, "test.json", save_dir)
        assert data["path"]["path_id"] == "test_path"
        assert len(data["path"]["waypoints"]) == 3
        wp = data["path"]["waypoints"][0]
        assert "lat" in wp
        assert "lon" in wp
        assert "alt" in wp

    def test_environment_data(self, populated_state, save_dir):
        data = save_path(populated_state, "test.json", save_dir)
        assert len(data["environment"]["obstacles"]) == 1
        assert len(data["environment"]["keepout_zones"]) == 1

    def test_no_computed_fields_in_save(self, populated_state, save_dir):
        """Saved JSON should not include total_distance_m or estimated_time_s."""
        data = save_path(populated_state, "test.json", save_dir)
        assert "total_distance_m" not in data["path"]
        assert "estimated_time_s" not in data["path"]

    def test_preserves_created_on_overwrite(self, populated_state, save_dir):
        data1 = save_path(populated_state, "test.json", save_dir)
        created1 = data1["created"]
        data2 = save_path(populated_state, "test.json", save_dir)
        assert data2["created"] == created1

    def test_creates_directory_if_missing(self, populated_state, save_dir):
        nested = os.path.join(save_dir, "nested", "deep")
        save_path(populated_state, "test.json", nested)
        assert os.path.exists(os.path.join(nested, "test.json"))


class TestLoadPath:
    def test_round_trip(self, populated_state, save_dir):
        save_path(populated_state, "test.json", save_dir)
        path, env = load_path("test.json", save_dir)
        assert path.path_id == "test_path"
        assert len(path.waypoints) == 3
        assert path.waypoints[0].lat == populated_state.path.waypoints[0].lat
        assert len(env.obstacles) == 1
        assert len(env.keepout_zones) == 1

    def test_waypoint_fields(self, populated_state, save_dir):
        save_path(populated_state, "test.json", save_dir)
        path, _ = load_path("test.json", save_dir)
        # Check the waypoint with all fields set
        wp = path.waypoints[2]  # waypoint_decatur
        assert wp.alt == 100.0
        assert wp.desired_time == "2025-04-01T14:30:00Z"
        assert wp.speed_to_next == 10.0

    def test_obstacle_fields(self, populated_state, save_dir):
        save_path(populated_state, "test.json", save_dir)
        _, env = load_path("test.json", save_dir)
        obs = env.obstacles[0]
        assert obs.id == "obs_001"
        assert obs.radius == 200.0

    def test_keepout_zone_fields(self, populated_state, save_dir):
        save_path(populated_state, "test.json", save_dir)
        _, env = load_path("test.json", save_dir)
        koz = env.keepout_zones[0]
        assert koz.id == "koz_001"
        assert len(koz.vertices) == 3
        assert koz.min_altitude is None

    def test_file_not_found(self, save_dir):
        with pytest.raises(FileNotFoundError):
            load_path("nonexistent.json", save_dir)


class TestParseUpload:
    def test_parse_minimal(self):
        data = {
            "path": {"path_id": "upload", "waypoints": [{"lat": 1.0, "lon": 2.0}]},
            "environment": {"obstacles": [], "keepout_zones": []},
        }
        path, env = parse_upload(data)
        assert path.path_id == "upload"
        assert len(path.waypoints) == 1
        assert path.waypoints[0].lat == 1.0

    def test_parse_empty(self):
        path, env = parse_upload({})
        assert len(path.waypoints) == 0
        assert len(env.obstacles) == 0

    def test_parse_with_defaults(self):
        data = {
            "path": {"waypoints": [{"lat": 1.0, "lon": 2.0}]},
        }
        path, env = parse_upload(data)
        assert path.waypoints[0].alt == 0.0
        assert path.waypoints[0].speed_to_next == 0.0


class TestListSavedPaths:
    def test_empty_dir(self, save_dir):
        files = list_saved_paths(save_dir)
        assert files == []

    def test_lists_files(self, populated_state, save_dir):
        save_path(populated_state, "alpha.json", save_dir)
        save_path(populated_state, "bravo.json", save_dir)
        files = list_saved_paths(save_dir)
        names = [f["filename"] for f in files]
        assert "alpha.json" in names
        assert "bravo.json" in names

    def test_includes_modified(self, populated_state, save_dir):
        save_path(populated_state, "test.json", save_dir)
        files = list_saved_paths(save_dir)
        assert files[0]["modified"] != ""

    def test_nonexistent_dir(self):
        files = list_saved_paths("/nonexistent/path")
        assert files == []
