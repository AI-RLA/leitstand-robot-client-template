"""A re-delivered dispatch that lands during the readiness probe is accepted, not refused."""

from __future__ import annotations

import threading
import time
from typing import Any

from leitstand.robot.v1 import mission_pb2

from leitstand_client import proto_json
from leitstand_client.mission_executor import MissionExecutor
from leitstand_client.navigation import FakeNavigation


class _SlowReadyNavigation(FakeNavigation):
    def check_ready(self, mission: mission_pb2.Mission, timeout_s: float = 1.0) -> str | None:
        time.sleep(0.2)
        return None


class _Query:
    def __init__(self, payload: bytes) -> None:
        self.payload = payload
        self.replies: list[bytes] = []

    def reply(self, key: str, payload: bytes) -> None:
        self.replies.append(payload)


class _Session:
    def declare_publisher(self, key: str) -> Any:
        return _Sink()

    def declare_queryable(self, key: str, handler: Any) -> Any:
        return _Sink()

    def declare_subscriber(self, key: str, handler: Any) -> Any:
        return _Sink()


class _Sink:
    def put(self, payload: bytes) -> None:
        """Discard."""

    def undeclare(self) -> None:
        """Nothing declared."""


_RUN = "11111111-1111-4111-8111-111111111111"
_STAGE = "22222222-2222-4222-8222-222222222222"


def _dispatch(run_id: str) -> bytes:
    stage = mission_pb2.Stage(stage_id=_STAGE, kind=mission_pb2.STAGE_KIND_NAVIGATION)
    stage.navigation.waypoints.add().wgs84.lat = 52.0
    request = mission_pb2.MissionDispatchRequest(
        dispatch_id="d", mission=mission_pb2.Mission(run_id=run_id, stages=[stage])
    )
    return proto_json.to_json(request)


def test_same_run_during_probe_window_is_accepted() -> None:
    executor = MissionExecutor(_Session(), "r1", _SlowReadyNavigation(speed_mps=1000.0))
    executor.start()
    try:
        queries = [_Query(_dispatch(_RUN)), _Query(_dispatch(_RUN))]
        threads = [threading.Thread(target=executor._handle_send_goal, args=(q,)) for q in queries]
        for t in threads:
            t.start()
        for t in threads:
            t.join(timeout=5.0)
        replies = [
            proto_json.parse(q.replies[0], mission_pb2.MissionDispatchResponse) for q in queries
        ]
        assert all(r.accepted for r in replies), [r.reason for r in replies]
    finally:
        executor.close()
