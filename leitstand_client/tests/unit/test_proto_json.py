"""Unit tests for the on-robot proto ACL helpers."""

from __future__ import annotations

import uuid
from datetime import datetime, timezone

import pytest
from google.protobuf import json_format
from leitstand.robot.v1 import mission_pb2, mission_state_pb2

from leitstand_client import proto_json


def _nav_stage(stage_id: str = "s1") -> mission_pb2.Stage:
    return mission_pb2.Stage(
        stage_id=stage_id,
        kind=mission_pb2.STAGE_KIND_NAVIGATION,
        navigation=mission_pb2.NavigationStage(
            waypoints=[mission_pb2.Waypoint(wgs84=mission_pb2.WGS84Waypoint(lat=1.0, lon=2.0))]
        ),
    )


def test_to_json_parse_roundtrip() -> None:
    mission = mission_pb2.Mission(run_id="m1", stages=[_nav_stage()])
    parsed = proto_json.parse(proto_json.to_json(mission), mission_pb2.Mission)
    assert parsed.run_id == "m1"
    assert parsed.stages[0].stage_id == "s1"


def test_parse_rejects_unknown_field() -> None:
    with pytest.raises(json_format.ParseError):
        proto_json.parse(b'{"run_id":"m1","bogus":1}', mission_pb2.Mission)


def test_stage_nav_waypoints_returns_waypoints() -> None:
    waypoints = proto_json.stage_nav_waypoints(_nav_stage())
    assert [wp.WhichOneof("kind") for wp in waypoints] == ["wgs84"]


def test_stage_nav_waypoints_rejects_wrong_kind() -> None:
    stage = mission_pb2.Stage(stage_id="s1", kind=mission_pb2.STAGE_KIND_UNSPECIFIED)
    with pytest.raises(ValueError):
        proto_json.stage_nav_waypoints(stage)


def test_stage_nav_waypoints_rejects_missing_payload() -> None:
    stage = mission_pb2.Stage(stage_id="s1", kind=mission_pb2.STAGE_KIND_NAVIGATION)
    with pytest.raises(ValueError):
        proto_json.stage_nav_waypoints(stage)


def test_make_stage_state_omits_unset_timestamps() -> None:
    ss = proto_json.make_stage_state("s1", mission_state_pb2.STAGE_STATUS_WAITING)
    assert not ss.HasField("started_at")
    assert not ss.HasField("ended_at")


def test_make_stage_state_sets_provided_timestamp() -> None:
    ss = proto_json.make_stage_state(
        "s1",
        mission_state_pb2.STAGE_STATUS_RUNNING,
        started_at=datetime(2026, 1, 1, tzinfo=timezone.utc),
    )
    assert ss.HasField("started_at")
    assert not ss.HasField("ended_at")


def _wp(lat: float, lon: float) -> mission_pb2.Waypoint:
    return mission_pb2.Waypoint(wgs84=mission_pb2.WGS84Waypoint(lat=lat, lon=lon))


def test_stage_waypoints_keeps_the_swaths_and_drops_the_turns():
    """Only the swaths are worked, so a caller asking what gets treated must not see the turns."""
    stage = mission_pb2.Stage(
        stage_id="11111111-1111-1111-1111-111111111111",
        kind=mission_pb2.STAGE_KIND_COVERAGE,
        coverage=mission_pb2.CoverageStage(
            segments=[
                mission_pb2.Segment(
                    kind=mission_pb2.SEGMENT_KIND_SWATH,
                    geometry=[_wp(52.0, 8.0), _wp(52.1, 8.0)],
                ),
                mission_pb2.Segment(
                    kind=mission_pb2.SEGMENT_KIND_TURN,
                    geometry=[_wp(52.1, 8.0), _wp(52.1, 8.05), _wp(52.1, 8.1)],
                ),
                mission_pb2.Segment(
                    kind=mission_pb2.SEGMENT_KIND_SWATH,
                    geometry=[_wp(52.1, 8.1), _wp(52.0, 8.1)],
                ),
            ],
        ),
    )

    points = proto_json.stage_waypoints(stage)

    assert [round(p.wgs84.lat, 3) for p in points] == [52.0, 52.1, 52.1, 52.0]
    assert [round(p.wgs84.lon, 3) for p in points] == [8.0, 8.0, 8.1, 8.1]
    assert len(proto_json.stage_segments(stage)) == 3


def test_a_coverage_stage_is_not_readable_as_navigation():
    """The shapes must not be silently interchangeable: a swath is a line, not a point list."""
    stage = mission_pb2.Stage(
        stage_id="11111111-1111-1111-1111-111111111111",
        kind=mission_pb2.STAGE_KIND_COVERAGE,
        coverage=mission_pb2.CoverageStage(
            segments=[
                mission_pb2.Segment(
                    kind=mission_pb2.SEGMENT_KIND_SWATH,
                    geometry=[_wp(52.0, 8.0), _wp(52.1, 8.0)],
                )
            ],
        ),
    )

    with pytest.raises(ValueError):
        proto_json.stage_nav_waypoints(stage)


def _coverage_mission(
    points_per_segment: int, bad_lat_at: int | None = None
) -> mission_pb2.Mission:
    """A coverage mission with three segments of the given length; optionally one bad point."""
    segments = []
    for seg_index in range(3):
        geometry = []
        for i in range(points_per_segment):
            lat = 52.0 + seg_index * 0.01 + i * 0.0001
            if bad_lat_at is not None and seg_index == 1 and i == bad_lat_at:
                lat = 95.0
            geometry.append(mission_pb2.Waypoint(wgs84=mission_pb2.WGS84Waypoint(lat=lat, lon=8.0)))
        segments.append(
            mission_pb2.Segment(
                kind=mission_pb2.SEGMENT_KIND_SWATH
                if seg_index % 2 == 0
                else mission_pb2.SEGMENT_KIND_TURN,
                geometry=geometry,
            )
        )
    return mission_pb2.Mission(
        run_id="6f1a1b2c-3d4e-4f60-8a7b-9c0d1e2f3a4b",
        stages=[
            mission_pb2.Stage(
                stage_id="0b1c2d3e-4f50-4617-8293-a4b5c6d7e8f9",
                kind=mission_pb2.STAGE_KIND_COVERAGE,
                coverage=mission_pb2.CoverageStage(segments=segments),
            )
        ],
    )


def test_a_field_sized_coverage_mission_passes_validation():
    """The engine sees only each segment's endpoints; the contract's two-point minimum must hold.

    Emptying the segments to save time failed every coverage plan with 'invalid Mission'.
    """
    proto_json.validate_mission(_coverage_mission(points_per_segment=400))


def test_a_two_point_segment_passes_untrimmed():
    proto_json.validate_mission(_coverage_mission(points_per_segment=2))


def test_a_bad_point_deep_inside_a_segment_is_still_caught():
    import pytest

    with pytest.raises(ValueError, match="latitude 95.0 out of range"):
        proto_json.validate_mission(_coverage_mission(points_per_segment=50, bad_lat_at=25))


def _site_local(x: float, y: float) -> mission_pb2.Waypoint:
    return mission_pb2.Waypoint(site_local=mission_pb2.SiteLocalWaypoint(site_id="site", x=x, y=y))


@pytest.mark.parametrize("kind", [mission_pb2.STAGE_KIND_UNSPECIFIED, 99])
def test_an_unknown_stage_kind_is_rejected(kind: int) -> None:
    stage = _nav_stage()
    stage.kind = kind
    with pytest.raises(ValueError, match="unknown stage kind"):
        proto_json.validate_mission(mission_pb2.Mission(run_id="m1", stages=[stage]))


def test_a_kind_without_its_payload_is_rejected() -> None:
    stage = _nav_stage()
    stage.kind = mission_pb2.STAGE_KIND_COVERAGE
    with pytest.raises(ValueError, match="without a coverage payload"):
        proto_json.validate_mission(mission_pb2.Mission(run_id="m1", stages=[stage]))


def test_mixed_frames_in_one_stage_are_rejected() -> None:
    stage = _nav_stage()
    stage.navigation.waypoints.append(_site_local(1.0, 2.0))
    with pytest.raises(ValueError, match="mix frames"):
        proto_json.validate_mission(mission_pb2.Mission(run_id="m1", stages=[stage]))


def test_cleanup_stages_are_checked_too() -> None:
    stage = _nav_stage()
    cleanup = _nav_stage("c1")
    cleanup.kind = 99
    stage.on_cancel.append(cleanup)
    with pytest.raises(ValueError, match="stage c1: unknown stage kind"):
        proto_json.validate_mission(mission_pb2.Mission(run_id="m1", stages=[stage]))


def test_a_well_formed_mission_still_passes() -> None:
    stage = _nav_stage(str(uuid.uuid4()))
    proto_json.validate_mission(mission_pb2.Mission(run_id=str(uuid.uuid4()), stages=[stage]))
