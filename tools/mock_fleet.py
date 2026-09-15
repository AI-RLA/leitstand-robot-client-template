"""Run N robots that drive nothing, for testing the Leitstand without hardware or Gazebo.

    python tools/mock_fleet.py --fleet 3 --endpoint tcp/127.0.0.1:7447

Each robot registers as mock_<n>, accepts navigation missions, walks them in simulated time
with FakeNavigation, and publishes a fixed pose and a slowly draining battery.
"""

from __future__ import annotations

import argparse
import logging
import signal
import threading
import time
from typing import Any

import zenoh
from leitstand.robot.v1 import telemetry_pb2

from leitstand_client import (
    FakeNavigation,
    LeitstandClient,
    RobotSpec,
    build_zenoh_config,
    keys,
    open_session,
    proto_json,
)

logger = logging.getLogger("mock_fleet")


class FixedTelemetry:
    """Publishes one position and a battery that drains one percent a minute."""

    def __init__(self, session: Any, robot_id: str, lat: float, lon: float) -> None:
        self._session = session
        self._pose_key = keys.POSE.format(robot_id=robot_id)
        self._battery_key = keys.BATTERY.format(robot_id=robot_id)
        self._lat, self._lon = lat, lon
        self._started = time.monotonic()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True, name=f"telemetry-{robot_id}")

    def start(self) -> None:
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)

    def _run(self) -> None:
        while not self._stop.wait(1.0):
            pose = telemetry_pb2.Pose(lat=self._lat, lon=self._lon)
            pose.timestamp.FromNanoseconds(time.time_ns())
            pct = max(0, 100 - int((time.monotonic() - self._started) / 60))
            battery = telemetry_pb2.Battery(battery_pct=pct, charging=False)
            battery.timestamp.FromNanoseconds(time.time_ns())
            for key, message in ((self._pose_key, pose), (self._battery_key, battery)):
                try:
                    self._session.put(
                        key, proto_json.to_json(message), encoding=zenoh.Encoding.APPLICATION_JSON
                    )
                except Exception as exc:  # noqa: BLE001
                    logger.warning("%s publish failed: %s", key, exc)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--fleet", type=int, default=1, help="number of robots (default 1)")
    parser.add_argument("--endpoint", default="tcp/127.0.0.1:7447", help="the Leitstand router")
    parser.add_argument("--speed", type=float, default=2.0, help="simulated speed in m/s")
    args = parser.parse_args()
    logging.basicConfig(level="INFO", format="%(asctime)s [%(levelname)s] %(name)s: %(message)s")

    clients = []
    sessions = []
    for n in range(1, args.fleet + 1):
        spec = RobotSpec.model_validate(
            {
                "id": f"mock_{n}",
                "leitstand": {"endpoint": args.endpoint},
                "factsheet": {"navigation": {"supported_waypoint_kinds": ["wgs84"]}},
                "navigation": "fake",
            }
        )
        session = open_session(build_zenoh_config(spec.leitstand))
        # A row of robots 20 m apart, so they are told apart on the map.
        telemetry = FixedTelemetry(session, spec.id, lat=52.2843, lon=8.0242 + 0.0003 * n)
        client = LeitstandClient(spec, session, FakeNavigation(speed_mps=args.speed), telemetry)
        client.start()
        clients.append(client)
        sessions.append(session)
    logger.info("%d mock robots online at %s", args.fleet, args.endpoint)

    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    stop.wait()

    for client, session in zip(reversed(clients), reversed(sessions)):
        client.stop()
        session.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
