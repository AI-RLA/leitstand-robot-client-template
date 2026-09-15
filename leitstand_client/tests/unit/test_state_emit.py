"""Round-trip the executor's emitted MissionState through the strict parser.

This is the regression guard for the original broken mock: it asserts the wire
carries proto MissionState (exec_status present) and none of the dropped fields
(mission_status / update_id / frame_id).
"""

from __future__ import annotations

import asyncio

from google.protobuf import json_format
from leitstand.robot.v1 import mission_pb2, mission_state_pb2

from leitstand_client.mission_executor import MissionExecutor, _ActiveContext
from leitstand_client.navigation import FakeNavigation


class _FakePublisher:
    def __init__(self) -> None:
        self.payloads: list[bytes] = []

    def put(self, payload: bytes) -> None:
        self.payloads.append(payload)


def _mission() -> mission_pb2.Mission:
    return mission_pb2.Mission(
        run_id="m1",
        stages=[
            mission_pb2.Stage(
                stage_id="s1",
                kind=mission_pb2.STAGE_KIND_NAVIGATION,
                navigation=mission_pb2.NavigationStage(
                    waypoints=[
                        mission_pb2.Waypoint(wgs84=mission_pb2.WGS84Waypoint(lat=1.0, lon=2.0))
                    ]
                ),
            )
        ],
    )


def test_emit_is_strict_parseable_proto_state() -> None:
    publisher = _FakePublisher()
    executor = MissionExecutor(
        session=object(), robot_id="r1", navigation=FakeNavigation(), state_publisher=publisher
    )

    asyncio.run(executor._publish_state(_ActiveContext(mission=_mission())))

    assert len(publisher.payloads) == 1
    payload = publisher.payloads[0]

    parsed = json_format.Parse(
        payload, mission_state_pb2.MissionState(), ignore_unknown_fields=False
    )
    assert parsed.run_id == "m1"
    assert parsed.exec_status == mission_state_pb2.MISSION_EXEC_STATUS_RUNNING

    wire = payload.decode()
    assert "mission_status" not in wire
    assert "update_id" not in wire
    assert "frame_id" not in wire
