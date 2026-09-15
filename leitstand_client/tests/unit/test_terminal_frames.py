"""What the terminal frame says after a cancel, a late tick, a shutdown or a plain finish."""

from __future__ import annotations

import asyncio

from google.protobuf import json_format
from leitstand.robot.v1 import mission_pb2, mission_state_pb2

from leitstand_client.mission_executor import MissionExecutor, _ActiveContext
from leitstand_client.navigation import FakeNavigation

RUN = "11111111-1111-4111-8111-111111111111"
CANCELLED = mission_state_pb2.STAGE_STATUS_CANCELLED
SKIPPED = mission_state_pb2.STAGE_STATUS_SKIPPED
FINISHED = mission_state_pb2.STAGE_STATUS_FINISHED
RUNNING = mission_state_pb2.STAGE_STATUS_RUNNING


class _Publisher:
    def __init__(self) -> None:
        self.frames: list[mission_state_pb2.MissionState] = []

    def put(self, payload: bytes) -> None:
        self.frames.append(
            json_format.Parse(
                payload, mission_state_pb2.MissionState(), ignore_unknown_fields=False
            )
        )


def _stage(stage_id: str, points: int = 3, cleanup: bool = False) -> mission_pb2.Stage:
    stage = mission_pb2.Stage(stage_id=stage_id, kind=mission_pb2.STAGE_KIND_NAVIGATION)
    for i in range(points):
        wp = stage.navigation.waypoints.add()
        wp.wgs84.lat = 52.0 + 0.001 * i
        wp.wgs84.lon = 8.0
    if cleanup:
        stage.on_cancel.add().CopyFrom(_stage(f"{stage_id}-cleanup", 2))
    return stage


def _executor(nav: FakeNavigation, publisher: _Publisher) -> MissionExecutor:
    return MissionExecutor(
        session=object(),
        robot_id="r1",
        navigation=nav,
        state_publisher=publisher,
        loop=asyncio.get_running_loop(),
    )


def _statuses(frame: mission_state_pb2.MissionState) -> dict[str, int]:
    return {s.stage_id: s.status for s in frame.stage_states}


def test_a_cancel_mid_stage_reports_cancelled_and_skipped() -> None:
    async def run() -> mission_state_pb2.MissionState:
        publisher = _Publisher()
        nav = FakeNavigation(speed_mps=1000.0, min_leg_s=0.05, tick_s=0.01)
        executor = _executor(nav, publisher)
        ctx = _ActiveContext(
            mission=mission_pb2.Mission(run_id=RUN, stages=[_stage("a"), _stage("b")])
        )
        task = asyncio.create_task(executor._execute_mission(ctx))
        await asyncio.sleep(0.02)
        ctx.cancel_mode = mission_pb2.CANCEL_MODE_IMMEDIATE
        ctx.cancel_stage_index = ctx.stage_index
        await task
        return publisher.frames[-1]

    last = asyncio.run(run())
    assert last.exec_status == mission_state_pb2.MISSION_EXEC_STATUS_CANCELLED
    assert _statuses(last) == {"a": CANCELLED, "b": SKIPPED}


def test_a_graceful_cancel_during_the_last_leg_still_ends_cancelled_with_cleanup() -> None:
    async def run() -> mission_state_pb2.MissionState:
        publisher = _Publisher()
        nav = FakeNavigation(speed_mps=1000.0, min_leg_s=0.03, tick_s=0.005)
        executor = _executor(nav, publisher)
        mission = mission_pb2.Mission(run_id=RUN, stages=[_stage("a", 2, cleanup=True)])
        ctx = _ActiveContext(mission=mission)
        task = asyncio.create_task(executor._execute_mission(ctx))
        await asyncio.sleep(0.02)
        ctx.cancel_mode = mission_pb2.CANCEL_MODE_GRACEFUL
        ctx.cancel_stage_index = ctx.stage_index
        await task
        return publisher.frames[-1]

    last = asyncio.run(run())
    assert last.exec_status == mission_state_pb2.MISSION_EXEC_STATUS_CANCELLED
    statuses = _statuses(last)
    # Whether the fake noticed the cancel before its last tick is timing; the run-level rule is
    # what this test pins, the finished-stage case is pinned by the between-stages test.
    assert statuses["a"] in (FINISHED, CANCELLED)
    assert statuses["a-cleanup"] == FINISHED


def test_a_cancel_between_stages_skips_the_unstarted_stage_without_its_cleanup() -> None:
    async def run() -> mission_state_pb2.MissionState:
        publisher = _Publisher()
        nav = FakeNavigation(speed_mps=1000.0, min_leg_s=0.01, tick_s=0.005)
        executor = _executor(nav, publisher)
        mission = mission_pb2.Mission(
            run_id=RUN, stages=[_stage("a", 2), _stage("b", 2, cleanup=True)]
        )
        ctx = _ActiveContext(mission=mission)
        # Cancel before the loop starts: stage "a" never begins, so no cleanup of anything.
        ctx.cancel_mode = mission_pb2.CANCEL_MODE_GRACEFUL
        ctx.cancel_stage_index = 0
        await executor._execute_mission(ctx)
        return publisher.frames[-1]

    last = asyncio.run(run())
    assert last.exec_status == mission_state_pb2.MISSION_EXEC_STATUS_CANCELLED
    assert _statuses(last) == {"a": SKIPPED, "b": SKIPPED}


def test_a_late_progress_tick_does_not_reopen_a_finished_stage() -> None:
    async def run() -> mission_state_pb2.MissionState:
        publisher = _Publisher()
        nav = FakeNavigation(speed_mps=1000.0, min_leg_s=0.01, tick_s=0.005)
        executor = _executor(nav, publisher)
        ctx = _ActiveContext(mission=mission_pb2.Mission(run_id=RUN, stages=[_stage("a", 2)]))
        await executor._execute_mission(ctx)
        executor._apply_progress(ctx, 0, RUNNING, 0.5)
        await asyncio.sleep(0.01)
        return publisher.frames[-1]

    last = asyncio.run(run())
    assert last.exec_status == mission_state_pb2.MISSION_EXEC_STATUS_SUCCEEDED
    assert _statuses(last) == {"a": FINISHED}
    assert last.stage_states[0].progress == 1.0


def test_progress_is_clamped() -> None:
    async def run() -> float:
        executor = _executor(FakeNavigation(), _Publisher())
        ctx = _ActiveContext(mission=mission_pb2.Mission(run_id=RUN, stages=[_stage("a")]))
        ctx.stage_states[0].status = RUNNING
        executor._apply_progress(ctx, 0, RUNNING, 1.7)
        return ctx.stage_states[0].progress

    assert asyncio.run(run()) == 1.0


def test_shutdown_mid_run_stops_the_stage_and_reports_failed_with_a_reason() -> None:
    async def run() -> tuple[mission_state_pb2.MissionState, int | None]:
        publisher = _Publisher()
        nav = FakeNavigation(speed_mps=1000.0, min_leg_s=0.05, tick_s=0.005)
        executor = _executor(nav, publisher)
        ctx = _ActiveContext(
            mission=mission_pb2.Mission(run_id=RUN, stages=[_stage("a", cleanup=True)])
        )
        executor._active = ctx
        task = asyncio.create_task(executor._execute_mission(ctx))
        await asyncio.sleep(0.02)
        ctx.shutting_down = True
        ctx.cancel_mode = mission_pb2.CANCEL_MODE_IMMEDIATE
        await task
        return publisher.frames[-1], ctx.cancel_mode

    last, seen = asyncio.run(run())
    assert last.exec_status == mission_state_pb2.MISSION_EXEC_STATUS_FAILED
    assert [e.type for e in last.errors] == ["client_shutdown"]
    assert "a-cleanup" not in _statuses(last)
    assert seen == mission_pb2.CANCEL_MODE_IMMEDIATE


def test_the_heartbeat_keeps_reporting_during_cleanup() -> None:
    async def run() -> int:
        publisher = _Publisher()
        nav = FakeNavigation(speed_mps=1000.0, min_leg_s=0.05, tick_s=0.005)
        executor = _executor(nav, publisher)
        import leitstand_client.mission_executor as me

        me._HEARTBEAT_INTERVAL_S, saved = 0.01, me._HEARTBEAT_INTERVAL_S
        try:
            mission = mission_pb2.Mission(run_id=RUN, stages=[_stage("a", 2, cleanup=True)])
            ctx = _ActiveContext(mission=mission)
            task = asyncio.create_task(executor._execute_mission(ctx))
            await asyncio.sleep(0.02)
            ctx.cancel_mode = mission_pb2.CANCEL_MODE_IMMEDIATE
            ctx.cancel_stage_index = 0
            n_at_cancel = len(publisher.frames)
            await task
        finally:
            me._HEARTBEAT_INTERVAL_S = saved
        cleanup_frames = [
            f
            for f in publisher.frames[n_at_cancel:]
            if f.exec_status == mission_state_pb2.MISSION_EXEC_STATUS_RUNNING
            and any(s.stage_id == "a-cleanup" for s in f.stage_states)
        ]
        return len(cleanup_frames)

    assert asyncio.run(run()) >= 2


def test_a_cancel_while_paused_still_runs_the_cleanup_and_ends() -> None:
    async def run() -> mission_state_pb2.MissionState:
        publisher = _Publisher()
        nav = FakeNavigation(speed_mps=1000.0, min_leg_s=0.05, tick_s=0.005)
        executor = _executor(nav, publisher)
        ctx = _ActiveContext(
            mission=mission_pb2.Mission(run_id=RUN, stages=[_stage("a", cleanup=True)])
        )
        task = asyncio.create_task(executor._execute_mission(ctx))
        await asyncio.sleep(0.02)
        nav.request_pause()
        await asyncio.sleep(0.02)
        ctx.cancel_mode = mission_pb2.CANCEL_MODE_GRACEFUL
        ctx.cancel_stage_index = 0
        await asyncio.wait_for(task, timeout=2.0)
        return publisher.frames[-1]

    last = asyncio.run(run())
    assert last.exec_status == mission_state_pb2.MISSION_EXEC_STATUS_CANCELLED
    assert _statuses(last)["a-cleanup"] == FINISHED


def test_the_next_run_after_a_cancel_while_paused_starts_running() -> None:
    async def run() -> list[int]:
        publisher = _Publisher()
        nav = FakeNavigation(speed_mps=1000.0, min_leg_s=0.02, tick_s=0.005)
        executor = _executor(nav, publisher)
        first = _ActiveContext(mission=mission_pb2.Mission(run_id=RUN, stages=[_stage("a")]))
        task = asyncio.create_task(executor._execute_mission(first))
        await asyncio.sleep(0.01)
        nav.request_pause()
        await asyncio.sleep(0.02)
        first.cancel_mode = mission_pb2.CANCEL_MODE_IMMEDIATE
        first.cancel_stage_index = 0
        await task
        publisher.frames.clear()
        nav.request_resume()
        second = _ActiveContext(mission=mission_pb2.Mission(run_id=RUN, stages=[_stage("b", 2)]))
        await executor._execute_mission(second)
        return [f.exec_status for f in publisher.frames]

    statuses = asyncio.run(run())
    assert mission_state_pb2.MISSION_EXEC_STATUS_PAUSED not in statuses
    assert statuses[-1] == mission_state_pb2.MISSION_EXEC_STATUS_SUCCEEDED


def test_a_cancel_between_stages_runs_the_finished_stages_cleanup() -> None:
    """The stage that was current when the cancel arrived gets its cleanup, wherever the loop is."""

    async def run() -> mission_state_pb2.MissionState:
        publisher = _Publisher()
        nav = FakeNavigation(speed_mps=1000.0, min_leg_s=0.01, tick_s=0.005)
        executor = _executor(nav, publisher)
        mission = mission_pb2.Mission(
            run_id=RUN, stages=[_stage("a", 2, cleanup=True), _stage("b", 2, cleanup=True)]
        )
        ctx = _ActiveContext(mission=mission)
        # The cancel arrived while "a" was current and "a" then finished before the loop saw it.
        ctx.cancel_mode = mission_pb2.CANCEL_MODE_GRACEFUL
        ctx.cancel_stage_index = 0
        ctx.stage_states[0].status = FINISHED
        await executor._execute_mission(ctx)
        return publisher.frames[-1]

    last = asyncio.run(run())
    statuses = _statuses(last)
    assert statuses["a-cleanup"] == FINISHED
    assert "b-cleanup" not in statuses
