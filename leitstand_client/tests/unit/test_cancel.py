"""Cancel modes against the fake navigation: when each returns, and what the executor publishes."""

from __future__ import annotations

import asyncio
import time

from google.protobuf import json_format
from leitstand.robot.v1 import mission_pb2, mission_state_pb2

from leitstand_client.mission_executor import MissionExecutor, _ActiveContext
from leitstand_client.navigation import FakeNavigation, is_immediate


def _two_leg_stage(stage_id: str = "s1") -> mission_pb2.Stage:
    stage = mission_pb2.Stage(stage_id=stage_id, kind=mission_pb2.STAGE_KIND_NAVIGATION)
    for lat in (52.0, 52.001, 52.002):
        stage.navigation.waypoints.add().wgs84.lat = lat
        stage.navigation.waypoints[-1].wgs84.lon = 8.0
    return stage


async def _cancel_after(nav: FakeNavigation, delay_s: float, mode: int) -> tuple[int, float]:
    holder: dict[str, int | None] = {"mode": None}
    task = asyncio.create_task(nav.execute_stage(_two_leg_stage(), "r1", lambda: holder["mode"]))
    await asyncio.sleep(delay_s)
    holder["mode"] = mode
    t0 = time.monotonic()
    result = await task
    return result.status, time.monotonic() - t0


def test_immediate_returns_within_a_tick() -> None:
    nav = FakeNavigation(speed_mps=1.0, tick_s=0.02)
    status, dt = asyncio.run(_cancel_after(nav, 0.1, mission_pb2.CANCEL_MODE_IMMEDIATE))
    assert status == mission_state_pb2.STAGE_STATUS_FAILED
    assert dt < 0.05


def test_graceful_returns_within_a_tick_too() -> None:
    nav = FakeNavigation(speed_mps=1.0, tick_s=0.02)
    status, dt = asyncio.run(_cancel_after(nav, 0.1, mission_pb2.CANCEL_MODE_GRACEFUL))
    assert status == mission_state_pb2.STAGE_STATUS_FAILED
    assert dt < 0.05


def test_unspecified_counts_as_graceful() -> None:
    assert not is_immediate(mission_pb2.CANCEL_MODE_UNSPECIFIED)
    assert not is_immediate(mission_pb2.CANCEL_MODE_GRACEFUL)
    assert is_immediate(mission_pb2.CANCEL_MODE_IMMEDIATE)
    assert not is_immediate(None)


class _Publisher:
    def __init__(self) -> None:
        self.frames: list[mission_state_pb2.MissionState] = []

    def put(self, payload: bytes) -> None:
        self.frames.append(
            json_format.Parse(
                payload, mission_state_pb2.MissionState(), ignore_unknown_fields=False
            )
        )


def _mission_with_cleanup() -> mission_pb2.Mission:
    stage = _two_leg_stage("main")
    cleanup = stage.on_cancel.add()
    cleanup.CopyFrom(_two_leg_stage("cleanup"))
    return mission_pb2.Mission(run_id="run-1", stages=[stage])


def test_executor_runs_cleanup_and_publishes_cancelled() -> None:
    async def run() -> _Publisher:
        publisher = _Publisher()
        nav = FakeNavigation(speed_mps=1000.0, min_leg_s=0.02, tick_s=0.01)
        executor = MissionExecutor(
            session=object(),
            robot_id="r1",
            navigation=nav,
            state_publisher=publisher,
            loop=asyncio.get_running_loop(),
        )
        ctx = _ActiveContext(mission=_mission_with_cleanup())
        task = asyncio.create_task(executor._execute_mission(ctx))
        await asyncio.sleep(0.015)
        ctx.cancel_mode = mission_pb2.CANCEL_MODE_GRACEFUL
        await task
        return publisher

    publisher = asyncio.run(run())
    last = publisher.frames[-1]
    assert last.exec_status == mission_state_pb2.MISSION_EXEC_STATUS_CANCELLED
    reported = {s.stage_id for s in last.stage_states}
    assert "cleanup" in reported, reported


def test_paused_single_waypoint_stage_waits_for_resume() -> None:
    """A stage dispatched while paused does not finish until resumed, even with nothing to drive."""

    async def run() -> tuple[bool, float]:
        nav = FakeNavigation(tick_s=0.01)
        nav.request_pause()
        stage = mission_pb2.Stage(stage_id="s1", kind=mission_pb2.STAGE_KIND_NAVIGATION)
        stage.navigation.waypoints.add().wgs84.lat = 52.0
        task = asyncio.create_task(nav.execute_stage(stage, "r1", lambda: None))
        await asyncio.sleep(0.05)
        still_running = not task.done()
        nav.request_resume()
        result = await task
        return still_running, result.status

    still_running, status = asyncio.run(run())
    assert still_running
    assert status == mission_state_pb2.STAGE_STATUS_FINISHED
