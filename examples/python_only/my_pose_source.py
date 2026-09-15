"""A PoseSource for a robot with no ROS: replace read_position with your GNSS receiver."""

from __future__ import annotations

import logging
import threading
import time
from typing import Any

import zenoh
from leitstand.robot.v1 import telemetry_pb2

from leitstand_client import keys, proto_json

logger = logging.getLogger(__name__)


class MyPoseSource:
    """Publishes the machine's position once a second on the robot's pose key."""

    def __init__(self, session: Any, robot_id: str, lat: float, lon: float) -> None:
        self._session = session
        self._key = keys.POSE.format(robot_id=robot_id)
        self._lat, self._lon = lat, lon
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True, name="pose-source")

    def read_position(self) -> tuple[float, float]:
        """Return the current WGS84 position; this example never moves."""
        return self._lat, self._lon

    def start(self) -> None:
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)

    def _run(self) -> None:
        while not self._stop.wait(1.0):
            lat, lon = self.read_position()
            pose = telemetry_pb2.Pose(lat=lat, lon=lon)
            pose.timestamp.FromNanoseconds(time.time_ns())
            try:
                self._session.put(
                    self._key, proto_json.to_json(pose), encoding=zenoh.Encoding.APPLICATION_JSON
                )
            except Exception as exc:  # noqa: BLE001
                logger.warning("pose publish failed: %s", exc)
