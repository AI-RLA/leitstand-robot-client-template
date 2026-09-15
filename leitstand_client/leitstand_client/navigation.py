"""What a robot must implement to drive a stage, and a fake that drives nothing."""

from __future__ import annotations

import asyncio
import logging
import threading
from dataclasses import dataclass
from typing import Callable, Protocol

from leitstand.robot.v1 import mission_pb2, mission_state_pb2

from leitstand_client import geo, proto_json

logger = logging.getLogger(__name__)

CancelRequested = Callable[[], int | None]
"""Return None while the run continues, else the CancelMode the operator sent."""


def is_immediate(mode: int | None) -> bool:
    """CANCEL_MODE_UNSPECIFIED counts as graceful, as the contract says."""
    return mode == mission_pb2.CANCEL_MODE_IMMEDIATE


@dataclass
class StageResult:
    status: int  # mission_state_pb2.STAGE_STATUS_* (FINISHED or FAILED)
    error: mission_state_pb2.Error | None = None


class Navigation(Protocol):
    """The seam between the Leitstand protocol and the machine. One instance per robot process.

    Three threads call it. ``execute_stage`` and ``is_paused`` run on the client's asyncio loop,
    which also publishes the heartbeat every 5 s and every state frame, so a call that blocks
    there silences the robot on the Leitstand: wrap a blocking SDK call in
    ``await asyncio.to_thread(...)``. ``check_ready``, ``request_pause`` and ``request_resume``
    run on the Zenoh thread that receives commands. ``close`` runs on the main thread.
    """

    async def execute_stage(
        self,
        stage: mission_pb2.Stage,
        robot_id: str,
        cancel_requested: CancelRequested,
        on_progress: Callable[[int, float], None] | None = None,
    ) -> StageResult:
        """Drive one stage; return when it finishes, fails, or stops for a cancel.

        ``robot_id`` is the id this process registered as, for logs and for a process that
        serves several robots. Poll ``cancel_requested()`` between steps and stop when it
        returns a mode: immediate means stop now, graceful means finish the current pass first.
        Call ``on_progress(status, fraction)`` with ``STAGE_STATUS_RUNNING`` or
        ``STAGE_STATUS_PAUSED`` and the fraction of the stage done (0 to 1); each call becomes
        a state frame the operator sees. Return ``StageResult(STAGE_STATUS_FINISHED)`` or
        ``StageResult(STAGE_STATUS_FAILED, error=Error(type=..., description=..., severity=...))``
        with a stable ``type`` code and a description for the operator. An exception raised
        here ends the whole run as FAILED with the traceback as the description.
        """

    def check_ready(self, mission: mission_pb2.Mission, timeout_s: float) -> str | None:
        """Return None if the mission can start, else the reason the Leitstand shows.

        Zenoh thread, so pose and cancel wait while this runs; return within ``timeout_s``
        (about 1 s), which means checking what is already known, not waiting for the stack.
        """

    def request_pause(self) -> None:
        """Ask the machine to stop where it is. Zenoh thread; must not block."""

    def request_resume(self) -> None:
        """Ask the machine to continue. Zenoh thread; must not block."""

    def is_paused(self) -> bool:
        """True while the machine stands still for a pause, not while it is still stopping.

        Report the change through ``on_progress`` so the backend sees PAUSED as soon as it holds.
        Asyncio loop thread.
        """

    def close(self) -> None:
        """Release the machine at shutdown; the stop is done by the executor. Main thread."""


class FakeNavigation:
    """Walks a stage's waypoints in simulated time and drives nothing.

    A stage that finished in a millisecond would exercise none of the progress reporting,
    pausing or cancelling a real navigation has to get right, so each leg takes the time the
    machine would. Pause and resume use a threading.Event because they arrive on a Zenoh thread.
    """

    def __init__(
        self,
        speed_mps: float = 1.0,
        min_leg_s: float = 0.2,
        max_leg_s: float = 5.0,
        tick_s: float = 0.05,
    ) -> None:
        self._speed_mps = speed_mps
        self._min_leg_s = min_leg_s
        self._max_leg_s = max_leg_s
        self._tick_s = tick_s
        self._running = threading.Event()
        self._running.set()
        # Set once the drive loop has noticed a pause: that is when the machine stands still.
        self._stopped = threading.Event()

    async def execute_stage(
        self,
        stage: mission_pb2.Stage,
        robot_id: str,
        cancel_requested: CancelRequested,
        on_progress: Callable[[int, float], None] | None = None,
    ) -> StageResult:
        """Execute one stage; return when it finishes or is cancelled."""
        if await self._wait_while_paused(cancel_requested, on_progress, 0.0):
            return StageResult(status=mission_state_pb2.STAGE_STATUS_FAILED)

        waypoints = proto_json.stage_waypoints(stage)
        total = max(len(waypoints) - 1, 1)
        for index in range(1, len(waypoints)):
            progress = (index - 1) / total
            if await self._drive_leg(
                waypoints[index - 1], waypoints[index], cancel_requested, on_progress, progress
            ):
                return StageResult(status=mission_state_pb2.STAGE_STATUS_FAILED)
            if on_progress is not None:
                on_progress(mission_state_pb2.STAGE_STATUS_RUNNING, index / total)

        logger.debug("[fake_navigation] stage %s finished", stage.stage_id)
        return StageResult(status=mission_state_pb2.STAGE_STATUS_FINISHED)

    async def _wait_while_paused(
        self,
        cancel_requested: CancelRequested,
        on_progress: Callable[[int, float], None] | None,
        progress: float,
    ) -> bool:
        """Stand still while paused; return True if a cancel arrived before or during the pause."""
        stopped_here = False
        if not self._running.is_set() and not self._stopped.is_set():
            self._stopped.set()
            stopped_here = True
            if on_progress is not None:
                on_progress(mission_state_pb2.STAGE_STATUS_PAUSED, progress)
        while not self._running.is_set():
            if cancel_requested() is not None:
                self._stopped.clear()
                return True
            await asyncio.sleep(self._tick_s)
        if stopped_here or self._stopped.is_set():
            self._stopped.clear()
            if on_progress is not None:
                on_progress(mission_state_pb2.STAGE_STATUS_RUNNING, progress)
        return cancel_requested() is not None

    async def _drive_leg(
        self,
        start: mission_pb2.Waypoint,
        end: mission_pb2.Waypoint,
        cancel_requested: CancelRequested,
        on_progress: Callable[[int, float], None] | None,
        progress: float,
    ) -> bool:
        """Sleep for the leg's duration in ticks; return True if a cancel arrived meanwhile."""
        remaining = self._leg_seconds(start, end)
        while remaining > 0.0:
            # Pause is checked per tick so the machine holds where it is, mid-leg included.
            if await self._wait_while_paused(cancel_requested, on_progress, progress):
                return True
            step = min(self._tick_s, remaining)
            await asyncio.sleep(step)
            remaining -= step
        return False

    def _leg_seconds(self, start: mission_pb2.Waypoint, end: mission_pb2.Waypoint) -> float:
        """How long to pretend a leg takes, from its length at the simulated speed."""
        if start.WhichOneof("kind") != "wgs84" or end.WhichOneof("kind") != "wgs84":
            return self._min_leg_s
        east, north = geo.enu_offset(start.wgs84.lat, start.wgs84.lon, end.wgs84.lat, end.wgs84.lon)
        metres = (east * east + north * north) ** 0.5
        return min(max(metres / self._speed_mps, self._min_leg_s), self._max_leg_s)

    def check_ready(self, mission: mission_pb2.Mission, timeout_s: float = 1.0) -> str | None:
        """Always ready: there is no navigation stack to be down."""
        return None

    def request_pause(self) -> None:
        self._running.clear()
        logger.info("[fake_navigation] pause requested")

    def request_resume(self) -> None:
        self._running.set()
        # The fake moves again on its next tick, so the stop is over as soon as it is asked.
        self._stopped.clear()
        logger.info("[fake_navigation] resume requested")

    def is_paused(self) -> bool:
        return self._stopped.is_set()

    def close(self) -> None:
        """Nothing to release."""
