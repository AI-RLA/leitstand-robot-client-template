"""On-robot ACL helpers for the leitstand.robot.v1 proto contract.

Centralizes proto-canonical JSON (de)serialization, Timestamp construction, and
the WhichOneof/HasField dispatch the rest of the client would otherwise repeat.
The wire is proto-canonical JSON (snake_case, default-omitting), matching the
backend; binary is a later encoding swap.
"""

from __future__ import annotations

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
    """
    _validate_stage_shapes(mission.stages, "stages")
    try:
        # The refusal names the first violation only, so the rest need not be collected.
        protovalidate.validate(mission, fail_fast=True)
    except protovalidate.ValidationError as exc:
        raise ValueError(_describe(exc.violations[0])) from exc


def _describe(violation: protovalidate.Violation) -> str:
    """Return the violated field's path, the rule's message and the value it refused."""
    parts = []
    for element in violation.proto.field.elements:
        subscript = element.subscript
        index = (
            f"[{subscript.value}]" if subscript is not None and subscript.field == "index" else ""
        )
        parts.append(f"{element.field_name}{index}")
    where = ".".join(parts) or "mission"
    value = repr(violation.field_value)
    if len(value) > 40:
        value = value[:37] + "..."
    return f"{where}: {violation.proto.message} (got {value})"


def _validate_stage_shapes(stages: Any, prefix: str) -> None:
    """Reject an unknown kind, a mismatched payload, mixed frames or a non-geographic path point.

    Errors name the stage by its position, such as ``stages[0].on_cancel[1]``, the same way
    ``_describe`` names a field.
    """
    for position, stage in enumerate(stages):
        where = f"{prefix}[{position}]"
        payload = _PAYLOAD_FOR_KIND.get(stage.kind)
        if payload is None:
            raise ValueError(f"{where}: unknown stage kind {stage.kind}")
        if stage.WhichOneof("payload") != payload:
            raise ValueError(f"{where}: kind {payload} without a {payload} payload")
        if payload == "navigation":
            frames = {wp.WhichOneof("kind") for wp in stage.navigation.waypoints}
            if len(frames) > 1:
                raise ValueError(f"{where}.navigation: waypoints mix frames {sorted(frames)}")
        else:
            for s_index, segment in enumerate(stage.coverage.segments):
                for g_index, waypoint in enumerate(segment.geometry):
                    if waypoint.WhichOneof("kind") != "wgs84":
                        raise ValueError(
                            f"{where}.coverage.segments[{s_index}].geometry[{g_index}]: "
                            "must be a wgs84 waypoint"
                        )
        _validate_stage_shapes(stage.on_cancel, f"{where}.on_cancel")


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
