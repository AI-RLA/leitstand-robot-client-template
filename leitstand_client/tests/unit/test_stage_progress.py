"""Unit tests for RUNNING/PAUSED stage progress and stage_id error attribution.

Drives a fake execute_stage that calls on_progress and returns controlled results;
asserts the correct MissionState frames are published and the throttle suppresses
sub-2% progress updates.
"""

from __future__ import annotations

import asyncio
from typing import Callable

from google.protobuf import json_format
from leitstand.robot.v1 import mission_pb2, mission_state_pb2

from leitstand_client.mission_executor import MissionExecutor, _ActiveContext
from leitstand_client.navigation import StageResult


class _FakePublisher:
    def __init__(self) -> None:
        self.payloads: list[bytes] = []

    def put(self, payload: bytes) -> None:
        self.payloads.append(payload)

    def parsed(self) -> list[mission_state_pb2.MissionState]:
        return [
            json_format.Parse(p, mission_state_pb2.MissionState(), ignore_unknown_fields=False)
            for p in self.payloads
        ]


def _mission(stage_id: str = "s1") -> mission_pb2.Mission:
    return mission_pb2.Mission(
        run_id="m1",
        stages=[
            mission_pb2.Stage(
                stage_id=stage_id,
                kind=mission_pb2.STAGE_KIND_NAVIGATION,
                navigation=mission_pb2.NavigationStage(
                    waypoints=[
                        mission_pb2.Waypoint(wgs84=mission_pb2.WGS84Waypoint(lat=1.0, lon=2.0))
                    ]
                ),
            )
        ],
    )


class _ScriptedNavigation:
    """Navigation whose execute_stage behaviour is injected per test."""

    def __init__(self, paused: bool = False) -> None:
        self._paused = paused
        # Injected by tests; signature: async (on_progress) -> StageResult
        self.execute_fn: Callable | None = None

    def is_paused(self) -> bool:
        return self._paused

    def request_pause(self) -> None:
        self._paused = True

    def request_resume(self) -> None:
        self._paused = False

    async def execute_stage(
        self,
        stage: mission_pb2.Stage,
        robot_id: str,
        cancel_requested: Callable[[], int | None],
        on_progress: Callable[[int, float], None] | None = None,
    ) -> StageResult:
        if self.execute_fn is not None:
            return await self.execute_fn(on_progress)
        return StageResult(status=mission_state_pb2.STAGE_STATUS_FINISHED)


async def _run_mission(
    fm: _ScriptedNavigation,
    publisher: _FakePublisher,
    stage_id: str = "s1",
) -> None:
    """Run one mission on the already-running loop, publishing to ``publisher``."""
    executor = MissionExecutor(
        session=object(),
        robot_id="r1",
        navigation=fm,
        state_publisher=publisher,
        loop=asyncio.get_running_loop(),
    )

    ctx = _ActiveContext(mission=_mission(stage_id=stage_id))
    await executor._execute_mission(ctx)


def test_running_progress_published() -> None:
    """A single on_progress(RUNNING, 0.5) call must produce a RUNNING frame with progress≈0.5."""
    fm = _ScriptedNavigation()

    async def _execute(on_progress: Callable | None) -> StageResult:
        if on_progress is not None:
            on_progress(mission_state_pb2.STAGE_STATUS_RUNNING, 0.5)
        # Yield so the scheduled _apply_progress and its ensure_future fire.
        await asyncio.sleep(0)
        await asyncio.sleep(0)
        return StageResult(status=mission_state_pb2.STAGE_STATUS_FINISHED)

    fm.execute_fn = _execute
    publisher = _FakePublisher()
    asyncio.run(_run_mission(fm, publisher))

    frames = publisher.parsed()
    running_frames = [
        f
        for f in frames
        if f.stage_states
        and f.stage_states[0].status == mission_state_pb2.STAGE_STATUS_RUNNING
        and abs(f.stage_states[0].progress - 0.5) < 0.01
    ]
    assert running_frames, "Expected a RUNNING frame with progress≈0.5; got statuses " + str(
        [(f.stage_states[0].status, f.stage_states[0].progress) for f in frames if f.stage_states]
    )


def test_throttle_suppresses_sub_2pct_update() -> None:
    """Two on_progress calls 1% apart must not both trigger a publish (throttle ≥ 2%)."""
    fm = _ScriptedNavigation()

    async def _execute(on_progress: Callable | None) -> StageResult:
        if on_progress is not None:
            on_progress(mission_state_pb2.STAGE_STATUS_RUNNING, 0.50)
        await asyncio.sleep(0)
        await asyncio.sleep(0)
        if on_progress is not None:
            # 1% delta, below the 2% threshold; must not trigger another publish.
            on_progress(mission_state_pb2.STAGE_STATUS_RUNNING, 0.51)
        await asyncio.sleep(0)
        await asyncio.sleep(0)
        return StageResult(status=mission_state_pb2.STAGE_STATUS_FINISHED)

    fm.execute_fn = _execute
    publisher = _FakePublisher()
    asyncio.run(_run_mission(fm, publisher))

    frames = publisher.parsed()
    # Count RUNNING frames at progress in the 0.50-0.51 band.
    running_50_ish = [
        f
        for f in frames
        if f.stage_states
        and f.stage_states[0].status == mission_state_pb2.STAGE_STATUS_RUNNING
        and 0.49 < f.stage_states[0].progress < 0.53
    ]
    # The 1% follow-up must be suppressed: exactly ONE such frame.
    assert len(running_50_ish) == 1, (
        f"Expected exactly 1 RUNNING frame in the 0.50-0.51 band; got {len(running_50_ish)}: "
        + str(
            [
                (f.stage_states[0].status, f.stage_states[0].progress)
                for f in frames
                if f.stage_states
            ]
        )
    )


def test_stage_failure_attaches_stage_id_reference() -> None:
    """A failed stage must have an ErrorReference(key='stage_id', value=stage.stage_id)."""
    fm = _ScriptedNavigation()

    async def _execute(on_progress: Callable | None) -> StageResult:
        return StageResult(
            status=mission_state_pb2.STAGE_STATUS_FAILED,
            error=mission_state_pb2.Error(
                severity=mission_state_pb2.ERROR_SEVERITY_FATAL,
                type="nav2_terminal_failure",
                description="Nav2 aborted",
            ),
        )

    fm.execute_fn = _execute
    publisher = _FakePublisher()
    asyncio.run(_run_mission(fm, publisher, stage_id="stage-abc"))

    frames = publisher.parsed()
    failed_frames = [f for f in frames if f.errors]
    assert failed_frames, "Expected at least one frame with errors on stage failure"

    terminal = failed_frames[-1]
    stage_id_refs = [r for e in terminal.errors for r in e.references if r.key == "stage_id"]
    assert stage_id_refs, "Expected stage_id ErrorReference; references found: " + str(
        [(r.key, r.value) for e in terminal.errors for r in e.references]
    )
    assert (
        stage_id_refs[0].value == "stage-abc"
    ), f"Expected stage_id='stage-abc', got '{stage_id_refs[0].value}'"
