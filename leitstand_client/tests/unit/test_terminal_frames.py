"""What the terminal frame says after a cancel, a late tick, a shutdown or a plain finish."""

from __future__ import annotations

import asyncio

import pytest
from google.protobuf import json_format
from leitstand.robot.v1 import mission_pb2, mission_state_pb2

from leitstand_client.mission_executor import MissionExecutor, _ActiveContext
from leitstand_client.navigation import FakeNavigation, StageResult

RUN = "11111111-1111-4111-8111-111111111111"
CANCELLED = mission_state_pb2.STAGE_STATUS_CANCELLED
SKIPPED = mission_state_pb2.STAGE_STATUS_SKIPPED
FINISHED = mission_state_pb2.STAGE_STATUS_FINISHED
RUNNING = mission_state_pb2.STAGE_STATUS_RUNNING
FAILED = mission_state_pb2.STAGE_STATUS_FAILED
UNSPECIFIED = mission_state_pb2.STAGE_STATUS_UNSPECIFIED


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


def test_a_cancel_while_paused_ends_cancelled_without_the_cleanup() -> None:
    """Someone may have paused because a person stands by the machine, so it must not drive off."""

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
    assert "a-cleanup" not in _statuses(last)


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


class _FixedResultNavigation(FakeNavigation):
    """Answers every stage with one fixed result."""

    def __init__(self, status: int, error: mission_state_pb2.Error | None = None) -> None:
        super().__init__()
        self._result = StageResult(status=status, error=error)

    async def execute_stage(self, stage, robot_id, cancel_requested, on_progress=None):
        return self._result


@pytest.mark.parametrize(
    ("status", "given", "error_type", "text"),
    [
        (CANCELLED, None, "navigation_cancelled", "no cancel was requested"),
        (UNSPECIFIED, None, "unexpected_stage_result", "STAGE_STATUS_UNSPECIFIED"),
        (99, None, "unexpected_stage_result", "navigation returned 99"),
        (FAILED, None, "navigation_failed", "without a reason"),
        (CANCELLED, mission_state_pb2.Error(type="estop", description="e-stop"), "estop", "e-stop"),
    ],
)
def test_a_stage_that_stops_on_its_own_fails_the_run(
    status: int, given: mission_state_pb2.Error | None, error_type: str, text: str
) -> None:
    async def run() -> mission_state_pb2.MissionState:
        publisher = _Publisher()
        mission = mission_pb2.Mission(run_id=RUN, stages=[_stage("a", cleanup=True)])
        await _executor(_FixedResultNavigation(status, given), publisher)._execute_mission(
            _ActiveContext(mission=mission)
        )
        return publisher.frames[-1]

    last = asyncio.run(run())
    assert last.exec_status == mission_state_pb2.MISSION_EXEC_STATUS_FAILED
    # FAILED, and no cleanup: that belongs to an operator's cancel.
    assert _statuses(last) == {"a": FAILED}
    (error,) = last.errors
    assert error.type == error_type
    assert text in error.description
    assert ("stage_id", "a") in [(r.key, r.value) for r in error.references]


class _FailsDuringTheCancel(FakeNavigation):
    """Runs until the test cancels, then reports a failure with its own error."""

    def __init__(self) -> None:
        super().__init__()
        self.started = asyncio.Event()

    async def execute_stage(self, stage, robot_id, cancel_requested, on_progress=None):
        self.started.set()
        while cancel_requested() is None:
            await asyncio.sleep(0.005)
        return StageResult(
            status=FAILED,
            error=mission_state_pb2.Error(type="stop_unconfirmed", description="not stopped"),
        )


def test_a_cancel_that_ends_with_an_error_fails_the_run_without_the_cleanup() -> None:
    """Cleanup would send new goals to a machine whose stop nobody confirmed."""

    async def run() -> mission_state_pb2.MissionState:
        publisher = _Publisher()
        nav = _FailsDuringTheCancel()
        executor = _executor(nav, publisher)
        ctx = _ActiveContext(
            mission=mission_pb2.Mission(run_id=RUN, stages=[_stage("a", cleanup=True)])
        )
        task = asyncio.create_task(executor._execute_mission(ctx))
        await nav.started.wait()
        ctx.cancel_mode = mission_pb2.CANCEL_MODE_GRACEFUL
        ctx.cancel_stage_index = 0
        await asyncio.wait_for(task, timeout=2.0)
        return publisher.frames[-1]

    last = asyncio.run(run())
    assert last.exec_status == mission_state_pb2.MISSION_EXEC_STATUS_FAILED
    assert [e.type for e in last.errors] == ["stop_unconfirmed"]
    assert "a-cleanup" not in _statuses(last)
