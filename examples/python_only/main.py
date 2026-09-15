"""Bring a robot with no ROS onto the Leitstand: load config, open Zenoh, plug in two adapters."""

from __future__ import annotations

import logging
import signal
import sys
import threading

from my_navigation import MyNavigation
from my_pose_source import MyPoseSource

from leitstand_client import LeitstandClient, build_zenoh_config, load_spec, open_session


def main() -> int:
    logging.basicConfig(level="INFO", format="%(asctime)s [%(levelname)s] %(name)s: %(message)s")
    spec = load_spec(sys.argv[1] if len(sys.argv) > 1 else "robot.yaml")
    session = open_session(build_zenoh_config(spec.leitstand))
    client = LeitstandClient(
        spec,
        session,
        MyNavigation(),
        MyPoseSource(session, spec.id, lat=52.0, lon=8.0),
    )
    client.start()

    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    stop.wait()

    client.stop()
    session.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
