"""A Navigation for a robot with no ROS: replace the body of drive_leg with your own API calls."""

from __future__ import annotations

import asyncio
import logging
import threading
from typing import Callable

from leitstand.robot.v1 import mission_pb2, mission_state_pb2

from leitstand_client import StageResult, proto_json
from leitstand_client.navigation import CancelRequested

logger = logging.getLogger(__name__)


class MyNavigation:
    """Drives a stage waypoint by waypoint. This one only logs; yours talks to the machine.

    A pause is requested on one thread and takes effect on another: request_pause only asks,
    and the drive loop reports PAUSED once the machine actually stands still. is_paused must
    say what the machine does, not what was asked, because the Leitstand shows it as fact.
    This example checks for a pause between legs; a real drive_leg also has to stop the machine
    mid-leg when request_pause is called (the Nav2 reference cancels the goal for that).
    """

    def __init__(self) -> None:
        self._running = threading.Event()
        self._running.set()
        # Set by the drive loop once it has stopped for a pause, cleared when it moves again.
        self._stopped = threading.Event()

    async def execute_stage(
        self,
        stage: mission_pb2.Stage,
        robot_id: str,
        cancel_requested: CancelRequested,
        on_progress: Callable[[int, float], None] | None = None,
    ) -> StageResult:
        waypoints = proto_json.stage_waypoints(stage)
        for index, waypoint in enumerate(waypoints, start=1):
            progress = (index - 1) / len(waypoints)
            if await self._hold_while_paused(cancel_requested, on_progress, progress):
                return StageResult(status=mission_state_pb2.STAGE_STATUS_FAILED)
            # A blocking SDK call belongs in a thread, or the heartbeat stops while it runs:
            #     await asyncio.to_thread(self._sdk.go_to, waypoint)
            await self.drive_leg(waypoint)
            if on_progress is not None:
                on_progress(mission_state_pb2.STAGE_STATUS_RUNNING, index / len(waypoints))
        return StageResult(status=mission_state_pb2.STAGE_STATUS_FINISHED)

    async def drive_leg(self, waypoint: mission_pb2.Waypoint) -> None:
        """Send the machine to one waypoint and return when it arrives."""
        logger.info("driving to %.6f, %.6f", waypoint.wgs84.lat, waypoint.wgs84.lon)
        await asyncio.sleep(1.0)

    async def _hold_while_paused(
        self,
        cancel_requested: CancelRequested,
        on_progress: Callable[[int, float], None] | None,
        progress: float,
    ) -> bool:
        """Stand still while paused; return True if a cancel arrived before or during it."""
        if self._running.is_set():
            return cancel_requested() is not None
        # Here a real robot is told to stop and this waits until it has.
        self._stopped.set()
        if on_progress is not None:
            on_progress(mission_state_pb2.STAGE_STATUS_PAUSED, progress)
        while not self._running.is_set():
            if cancel_requested() is not None:
                self._stopped.clear()
                return True
            await asyncio.sleep(0.1)
        self._stopped.clear()
        if on_progress is not None:
            on_progress(mission_state_pb2.STAGE_STATUS_RUNNING, progress)
        return False

    def check_ready(self, mission: mission_pb2.Mission, timeout_s: float) -> str | None:
        return None

    def request_pause(self) -> None:
        self._running.clear()

    def request_resume(self) -> None:
        self._running.set()

    def is_paused(self) -> bool:
        return self._stopped.is_set()

    def close(self) -> None:
        """Nothing to release."""
