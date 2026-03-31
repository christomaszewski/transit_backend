"""Tests for transit_backend.deviation — spatial and temporal deviation computation."""

import pytest

from transit_backend.deviation import (
    _haversine_dict,
    _path_length,
    _point_to_polyline_distance,
    _point_to_segment_distance,
    _time_diff_seconds,
    compute_deviations,
)


class TestHaversineDict:
    def test_same_point(self):
        assert _haversine_dict(33.95, -84.55, 33.95, -84.55) == 0.0

    def test_one_degree(self):
        dist = _haversine_dict(0.0, 0.0, 1.0, 0.0)
        assert 110_000 < dist < 112_000

    def test_symmetric(self):
        d1 = _haversine_dict(33.0, -84.0, 34.0, -83.0)
        d2 = _haversine_dict(34.0, -83.0, 33.0, -84.0)
        assert abs(d1 - d2) < 0.001


class TestPathLength:
    def test_empty(self):
        assert _path_length([]) == 0.0

    def test_single_point(self):
        assert _path_length([{"lat": 0, "lon": 0}]) == 0.0

    def test_two_points(self):
        wps = [{"lat": 0.0, "lon": 0.0}, {"lat": 1.0, "lon": 0.0}]
        length = _path_length(wps)
        assert 110_000 < length < 112_000

    def test_three_points(self):
        wps = [
            {"lat": 0.0, "lon": 0.0},
            {"lat": 1.0, "lon": 0.0},
            {"lat": 1.0, "lon": 1.0},
        ]
        length = _path_length(wps)
        # Two segments, each ~111 km
        assert 200_000 < length < 230_000


class TestPointToSegmentDistance:
    def test_point_on_segment(self):
        """Point exactly on the segment should have ~0 distance."""
        # Segment from (0,0) to (0,1), point at (0, 0.5)
        dist = _point_to_segment_distance(0.0, 0.5, 0.0, 0.0, 0.0, 1.0)
        assert dist < 1.0  # within 1 meter

    def test_point_perpendicular(self):
        """Point 1 degree north of a horizontal segment."""
        dist = _point_to_segment_distance(1.0, 0.5, 0.0, 0.0, 0.0, 1.0)
        assert 110_000 < dist < 112_000

    def test_point_nearest_endpoint(self):
        """Point beyond segment end — nearest is endpoint."""
        dist = _point_to_segment_distance(0.0, 2.0, 0.0, 0.0, 0.0, 1.0)
        # Distance from (0,2) to (0,1) = 1 degree of longitude at equator
        assert 110_000 < dist < 112_000

    def test_degenerate_segment(self):
        """Segment of zero length — distance to the point."""
        dist = _point_to_segment_distance(1.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        assert dist > 0


class TestPointToPolylineDistance:
    def test_single_point_polyline(self):
        poly = [{"lat": 0.0, "lon": 0.0}]
        dist = _point_to_polyline_distance(1.0, 0.0, poly)
        assert 110_000 < dist < 112_000

    def test_empty_polyline(self):
        assert _point_to_polyline_distance(0.0, 0.0, []) == 0.0

    def test_point_on_polyline(self):
        poly = [{"lat": 0.0, "lon": 0.0}, {"lat": 0.0, "lon": 1.0}]
        dist = _point_to_polyline_distance(0.0, 0.5, poly)
        assert dist < 1.0

    def test_multi_segment_polyline(self):
        """Point should find the closest segment."""
        poly = [
            {"lat": 0.0, "lon": 0.0},
            {"lat": 0.0, "lon": 1.0},
            {"lat": 1.0, "lon": 1.0},
        ]
        # Point near second segment
        dist = _point_to_polyline_distance(0.5, 1.0, poly)
        assert dist < 1.0


class TestTimeDiffSeconds:
    def test_same_time(self):
        assert _time_diff_seconds("2025-04-01T12:00:00Z", "2025-04-01T12:00:00Z") == 0.0

    def test_positive_diff(self):
        """Target is 60 seconds behind reference."""
        diff = _time_diff_seconds("2025-04-01T12:00:00Z", "2025-04-01T12:01:00Z")
        assert diff == pytest.approx(60.0)

    def test_negative_diff(self):
        """Target is ahead of reference."""
        diff = _time_diff_seconds("2025-04-01T12:01:00Z", "2025-04-01T12:00:00Z")
        assert diff == pytest.approx(-60.0)

    def test_none_ref(self):
        assert _time_diff_seconds(None, "2025-04-01T12:00:00Z") == 0.0

    def test_none_target(self):
        assert _time_diff_seconds("2025-04-01T12:00:00Z", None) == 0.0

    def test_both_none(self):
        assert _time_diff_seconds(None, None) == 0.0

    def test_invalid_format(self):
        assert _time_diff_seconds("not-a-date", "2025-04-01T12:00:00Z") == 0.0


class TestComputeDeviations:
    def test_empty_reference(self):
        result = compute_deviations([], [{"lat": 0, "lon": 0}], "ref", "tgt")
        assert result["max_spatial_deviation_m"] == 0.0
        assert result["temporal_deviations"] == []

    def test_empty_target(self):
        result = compute_deviations([{"lat": 0, "lon": 0}], [], "ref", "tgt")
        assert result["max_spatial_deviation_m"] == 0.0

    def test_identical_paths(self):
        wps = [
            {"lat": 0.0, "lon": 0.0, "desired_time": "2025-04-01T12:00:00Z", "speed_to_next": 0},
            {"lat": 1.0, "lon": 0.0, "desired_time": "2025-04-01T13:00:00Z", "speed_to_next": 0},
        ]
        result = compute_deviations(wps, wps, "ref", "tgt")
        assert result["max_spatial_deviation_m"] < 1.0
        assert result["mean_spatial_deviation_m"] < 1.0
        assert result["path_length_diff_m"] == pytest.approx(0.0, abs=0.1)

    def test_offset_paths(self):
        ref = [
            {"lat": 0.0, "lon": 0.0, "desired_time": None, "speed_to_next": 0},
            {"lat": 1.0, "lon": 0.0, "desired_time": None, "speed_to_next": 0},
        ]
        tgt = [
            {"lat": 0.0, "lon": 0.01, "desired_time": None, "speed_to_next": 0},
            {"lat": 1.0, "lon": 0.01, "desired_time": None, "speed_to_next": 0},
        ]
        result = compute_deviations(ref, tgt, "ref", "tgt")
        # 0.01 degree longitude at equator ~ 1.1 km
        assert result["max_spatial_deviation_m"] > 100
        assert result["mean_spatial_deviation_m"] > 100

    def test_temporal_deviations(self):
        ref = [
            {"lat": 0.0, "lon": 0.0, "desired_time": "2025-04-01T12:00:00Z", "speed_to_next": 0},
            {"lat": 1.0, "lon": 0.0, "desired_time": "2025-04-01T13:00:00Z", "speed_to_next": 0},
        ]
        tgt = [
            {"lat": 0.0, "lon": 0.0, "desired_time": "2025-04-01T12:01:00Z", "speed_to_next": 0},
            {"lat": 1.0, "lon": 0.0, "desired_time": "2025-04-01T13:02:00Z", "speed_to_next": 0},
        ]
        result = compute_deviations(ref, tgt, "ref", "tgt")
        assert len(result["temporal_deviations"]) == 2
        # First waypoint: target is 60s behind
        assert result["temporal_deviations"][0]["ahead_behind_s"] == pytest.approx(60.0)
        # Second waypoint: target is 120s behind
        assert result["temporal_deviations"][1]["ahead_behind_s"] == pytest.approx(120.0)

    def test_names_in_result(self):
        result = compute_deviations(
            [{"lat": 0, "lon": 0, "desired_time": None, "speed_to_next": 0}],
            [{"lat": 0, "lon": 0, "desired_time": None, "speed_to_next": 0}],
            "User Path",
            "BIT* Reference",
        )
        assert result["reference"] == "User Path"
        assert result["target"] == "BIT* Reference"

    def test_path_length_diff(self):
        """Target path is longer than reference."""
        ref = [
            {"lat": 0.0, "lon": 0.0, "desired_time": None, "speed_to_next": 0},
            {"lat": 1.0, "lon": 0.0, "desired_time": None, "speed_to_next": 0},
        ]
        tgt = [
            {"lat": 0.0, "lon": 0.0, "desired_time": None, "speed_to_next": 0},
            {"lat": 0.5, "lon": 0.5, "desired_time": None, "speed_to_next": 0},
            {"lat": 1.0, "lon": 0.0, "desired_time": None, "speed_to_next": 0},
        ]
        result = compute_deviations(ref, tgt, "ref", "tgt")
        # Detour path should be longer
        assert result["path_length_diff_m"] > 0
