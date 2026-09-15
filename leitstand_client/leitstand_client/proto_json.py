"""On-robot ACL helpers for the leitstand.robot.v1 proto contract.

Centralizes proto-canonical JSON (de)serialization, Timestamp construction, and
the WhichOneof/HasField dispatch the rest of the client would otherwise repeat.
The wire is proto-canonical JSON (snake_case, default-omitting), matching the
backend; binary is a later encoding swap.
"""

from __future__ import annotations

from collections.abc import Iterator
from datetime import datetime, timezone
from typing import Any

import protovalidate
from google.protobuf import json_format, timestamp_pb2
from leitstand.robot.v1 import mission_pb2, mission_state_pb2


def to_json(msg: Any) -> bytes:
    """Serialize a proto message to proto-canonical JSON bytes (snake_case keys)."""
    return json_format.MessageToJson(msg, preserving_proto_field_name=True).encode("utf-8")


def parse(payload: bytes, msg_type: type) -> Any:
    """Strict-parse proto-JSON into ``msg_type``; reject unknown fields/enum names."""
    return json_format.Parse(payload, msg_type(), ignore_unknown_fields=False)


def now_ts() -> timestamp_pb2.Timestamp:
    """Return a Timestamp set to the current UTC time."""
    ts = timestamp_pb2.Timestamp()
    ts.FromDatetime(datetime.now(timezone.utc))
    return ts


_PAYLOAD_FOR_KIND = {
    mission_pb2.STAGE_KIND_NAVIGATION: "navigation",
    mission_pb2.STAGE_KIND_COVERAGE: "coverage",
}


def validate_mission(mission: mission_pb2.Mission) -> None:
    """Check the mission's shape and its buf.validate constraints; raise on the first violation.

    json_format.Parse enforces structure and enum names but not the bound constraints
    (lat/lon/heading ranges, min_items, uuid), and the enum is open, so a kind this client does
    not know must be refused here rather than fail while driving.

    A coverage route is checked numerically because the constraint engine costs about six
    milliseconds per waypoint, which for a field-sized route is seconds out of the 10 s dispatch
    budget. The rules applied are those on WGS84Waypoint and must move with them.
    """
    _validate_stage_shapes(mission.stages)

    # Each segment keeps its two endpoints in the copy the engine sees: the contract requires at
    # least two points per segment. The points between are checked numerically below.
    trimmed = mission_pb2.Mission()
    trimmed.CopyFrom(mission)
    for stage in _walk_stages(trimmed):
        for segment in stage.coverage.segments:
            if len(segment.geometry) > 2:
                first, last = segment.geometry[0], segment.geometry[-1]
                del segment.geometry[:]
                segment.geometry.append(first)
                segment.geometry.append(last)
    protovalidate.validate(trimmed)

    for stage in _walk_stages(mission):
        index = 0
        for segment in stage.coverage.segments:
            for waypoint in segment.geometry:
                _validate_path_waypoint(stage.stage_id, index, waypoint)
                index += 1


def _validate_stage_shapes(stages: Any) -> None:
    """Reject an unknown kind, a payload that does not match it, or mixed frames, at every depth."""
    for stage in stages:
        payload = _PAYLOAD_FOR_KIND.get(stage.kind)
        if payload is None:
            raise ValueError(f"stage {stage.stage_id}: unknown stage kind {stage.kind}")
        if stage.WhichOneof("payload") != payload:
            raise ValueError(f"stage {stage.stage_id}: kind {payload} without a {payload} payload")
        if payload == "navigation":
            frames = {wp.WhichOneof("kind") for wp in stage.navigation.waypoints}
            if len(frames) > 1:
                raise ValueError(f"stage {stage.stage_id}: waypoints mix frames {sorted(frames)}")
        _validate_stage_shapes(stage.on_cancel)


def _walk_stages(mission: mission_pb2.Mission) -> Iterator[mission_pb2.Stage]:
    """Yield every coverage stage carrying segments, cleanup stages included."""

    def walk(stages: Any) -> Iterator[mission_pb2.Stage]:
        for stage in stages:
            if stage.kind == mission_pb2.STAGE_KIND_COVERAGE and stage.coverage.segments:
                yield stage
            yield from walk(stage.on_cancel)

    return walk(mission.stages)


def _validate_path_waypoint(stage_id: str, index: int, waypoint: mission_pb2.Waypoint) -> None:
    if waypoint.WhichOneof("kind") != "wgs84":
        raise ValueError(f"stage {stage_id}: path[{index}] is not a geographic waypoint")
    point = waypoint.wgs84
    if not -90.0 <= point.lat <= 90.0:
        raise ValueError(f"stage {stage_id}: path[{index}] latitude {point.lat} out of range")
    if not -180.0 <= point.lon <= 180.0:
        raise ValueError(f"stage {stage_id}: path[{index}] longitude {point.lon} out of range")
    if point.HasField("heading_deg") and not 0.0 <= point.heading_deg < 360.0:
        raise ValueError(
            f"stage {stage_id}: path[{index}] heading {point.heading_deg} out of range"
        )


def stage_nav_waypoints(stage: mission_pb2.Stage) -> list[mission_pb2.Waypoint]:
    """Return a NAVIGATION stage's waypoints; raise if the kind/payload arm is unsupported."""
    if (
        stage.kind != mission_pb2.STAGE_KIND_NAVIGATION
        or stage.WhichOneof("payload") != "navigation"
    ):
        raise ValueError(f"stage {stage.stage_id}: unsupported kind/payload arm")
    return list(stage.navigation.waypoints)


def stage_segments(stage: mission_pb2.Stage) -> list[mission_pb2.Segment]:
    """Return a COVERAGE stage's segments in order; raise if the kind/payload arm is unsupported."""
    if stage.kind != mission_pb2.STAGE_KIND_COVERAGE or stage.WhichOneof("payload") != "coverage":
        raise ValueError(f"stage {stage.stage_id}: unsupported kind/payload arm")
    return list(stage.coverage.segments)


def stage_waypoints(stage: mission_pb2.Stage) -> list[mission_pb2.Waypoint]:
    """Return every waypoint a stage will drive, whatever shape it carries them in.

    For callers that only need the points. Anything that must respect a swath as a line to hold
    rather than a set of points to reach has to read the swaths themselves.
    """
    if stage.kind == mission_pb2.STAGE_KIND_COVERAGE:
        return [
            wp
            for seg in stage_segments(stage)
            if seg.kind == mission_pb2.SEGMENT_KIND_SWATH
            for wp in seg.geometry
        ]
    return stage_nav_waypoints(stage)


def make_stage_state(
    stage_id: str,
    status: int,
    *,
    started_at: datetime | None = None,
    ended_at: datetime | None = None,
    progress: float = 0.0,
    result: dict[str, str] | None = None,
) -> mission_state_pb2.StageState:
    """Build a StageState, setting the optional Timestamps only when provided.

    Proto optional Timestamps read as a present default when unset, so they are
    populated via FromDatetime only for non-None inputs rather than assigned.
    """
    ss = mission_state_pb2.StageState(stage_id=stage_id, status=status, progress=progress)
    if started_at is not None:
        ss.started_at.FromDatetime(started_at)
    if ended_at is not None:
        ss.ended_at.FromDatetime(ended_at)
    if result:
        ss.result.update(result)
    return ss
