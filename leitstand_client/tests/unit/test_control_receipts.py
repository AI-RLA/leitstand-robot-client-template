"""Pause, resume and cancel are answered first, applied second, and confirmed by a frame."""

from __future__ import annotations

import asyncio
from typing import Any

from google.protobuf import json_format
from leitstand.robot.v1 import mission_pb2, mission_state_pb2, robot_control_pb2

from leitstand_client import proto_json
from leitstand_client.mission_executor import MissionExecutor, _ActiveContext
from leitstand_client.navigation import FakeNavigation

RUN = "11111111-1111-4111-8111-111111111111"
OTHER = "33333333-3333-4333-8333-333333333333"


class _Query:
    def __init__(self, payload: bytes | None) -> None:
        self.payload = payload
        self.replies: list[robot_control_pb2.ControlResponse] = []

    def reply(self, key: str, payload: bytes) -> None:
        self.replies.append(proto_json.parse(payload, robot_control_pb2.ControlResponse))


class _Publisher:
    def __init__(self) -> None:
        self.frames: list[mission_state_pb2.MissionState] = []

    def put(self, payload: bytes) -> None:
        self.frames.append(
            json_format.Parse(
                payload, mission_state_pb2.MissionState(), ignore_unknown_fields=False
            )
        )


class _RecordingNavigation(FakeNavigation):
    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.calls: list[str] = []

    def request_pause(self) -> None:
        self.calls.append("pause")
        super().request_pause()

    def request_resume(self) -> None:
        self.calls.append("resume")
        super().request_resume()


def _stage(stage_id: str, points: int = 3) -> mission_pb2.Stage:
    stage = mission_pb2.Stage(stage_id=stage_id, kind=mission_pb2.STAGE_KIND_NAVIGATION)
    for i in range(points):
        wp = stage.navigation.waypoints.add()
        wp.wgs84.lat = 52.0 + 0.001 * i
        wp.wgs84.lon = 8.0
    return stage


def _control(run_id: str) -> bytes:
    return proto_json.to_json(robot_control_pb2.ControlRequest(run_id=run_id))


def _executor(loop: asyncio.AbstractEventLoop, nav: FakeNavigation, publisher: _Publisher):
    return MissionExecutor(
        session=object(), robot_id="r1", navigation=nav, state_publisher=publisher, loop=loop
    )


def _activate(executor: MissionExecutor, mission: mission_pb2.Mission) -> _ActiveContext:
    ctx = _ActiveContext(mission=mission)
    executor._active = ctx
    return ctx


def test_pause_is_answered_then_applied_then_published() -> None:
    async def run() -> tuple[_Query, list[str], int]:
        nav = _RecordingNavigation(tick_s=0.01)
        publisher = _Publisher()
        executor = _executor(asyncio.get_running_loop(), nav, publisher)
        _activate(executor, mission_pb2.Mission(run_id=RUN, stages=[_stage("s1")]))
        query = _Query(_control(RUN))
        executor._handle_pause(query)
        await asyncio.sleep(0.02)
        return query, nav.calls, len(publisher.frames)

    query, calls, frames = asyncio.run(run())
    assert query.replies[0].applied is True
    assert calls == ["pause"]
    assert frames == 1


def test_a_command_for_another_run_is_refused_and_not_applied() -> None:
    async def run() -> tuple[_Query, list[str]]:
        nav = _RecordingNavigation(tick_s=0.01)
        executor = _executor(asyncio.get_running_loop(), nav, _Publisher())
        _activate(executor, mission_pb2.Mission(run_id=RUN, stages=[_stage("s1")]))
        query = _Query(_control(OTHER))
        executor._handle_resume(query)
        await asyncio.sleep(0.01)
        return query, nav.calls

    query, calls = asyncio.run(run())
    reply = query.replies[0]
    assert reply.applied is False
    assert reply.refusal == robot_control_pb2.CONTROL_REFUSAL_NOT_EXECUTING_RUN
    assert calls == []


def test_an_unreadable_command_is_refused_with_other() -> None:
    async def run() -> _Query:
        executor = _executor(asyncio.get_running_loop(), FakeNavigation(), _Publisher())
        query = _Query(b"not json")
        executor._handle_pause(query)
        return query

    reply = asyncio.run(run()).replies[0]
    assert reply.applied is False
    assert reply.refusal == robot_control_pb2.CONTROL_REFUSAL_OTHER


def test_cancel_is_answered_with_a_receipt_and_refused_for_an_idle_robot() -> None:
    async def run() -> tuple[_Query, _Query]:
        nav = FakeNavigation(speed_mps=1000.0, min_leg_s=0.05, tick_s=0.01)
        publisher = _Publisher()
        executor = _executor(asyncio.get_running_loop(), nav, publisher)
        idle = _Query(proto_json.to_json(mission_pb2.CancelRequest(run_id=RUN)))
        executor._handle_cancel_goal(idle)
        ctx = _activate(executor, mission_pb2.Mission(run_id=RUN, stages=[_stage("s1")]))
        task = asyncio.create_task(executor._execute_mission(ctx))
        await asyncio.sleep(0.02)
        active = _Query(proto_json.to_json(mission_pb2.CancelRequest(run_id=RUN)))
        executor._handle_cancel_goal(active)
        await task
        return idle, active

    idle, active = asyncio.run(run())
    assert idle.replies[0].applied is False
    assert idle.replies[0].refusal == robot_control_pb2.CONTROL_REFUSAL_NOT_EXECUTING_RUN
    assert active.replies[0].applied is True


def test_active_run_id_follows_the_run() -> None:
    async def run() -> tuple[str | None, str | None, str | None]:
        nav = FakeNavigation(speed_mps=1000.0, min_leg_s=0.02, tick_s=0.01)
        executor = _executor(asyncio.get_running_loop(), nav, _Publisher())
        before = executor.active_run_id()
        ctx = _activate(executor, mission_pb2.Mission(run_id=RUN, stages=[_stage("s1")]))
        task = asyncio.create_task(executor._execute_mission(ctx))
        await asyncio.sleep(0.01)
        during = executor.active_run_id()
        await task
        return before, during, executor.active_run_id()

    assert asyncio.run(run()) == (None, RUN, None)


def test_paused_is_reported_once_the_machine_stands_still_and_running_after_resume() -> None:
    async def run() -> tuple[bool, list[int]]:
        nav = _RecordingNavigation(speed_mps=1000.0, min_leg_s=0.2, tick_s=0.005)
        publisher = _Publisher()
        executor = _executor(asyncio.get_running_loop(), nav, publisher)
        ctx = _activate(executor, mission_pb2.Mission(run_id=RUN, stages=[_stage("s1", 2)]))
        task = asyncio.create_task(executor._execute_mission(ctx))
        await asyncio.sleep(0.02)
        nav.request_pause()
        paused_at_once = nav.is_paused()
        await asyncio.sleep(0.03)
        paused_after_tick = nav.is_paused()
        nav.request_resume()
        await asyncio.sleep(0.03)
        statuses = [f.exec_status for f in publisher.frames]
        ctx.cancel_mode = mission_pb2.CANCEL_MODE_IMMEDIATE
        ctx.cancel_stage_index = 0
        await task
        return paused_at_once, statuses if paused_after_tick else []

    paused_at_once, statuses = asyncio.run(run())
    assert paused_at_once is False
    assert mission_state_pb2.MISSION_EXEC_STATUS_PAUSED in statuses
    assert statuses[-1] == mission_state_pb2.MISSION_EXEC_STATUS_RUNNING
