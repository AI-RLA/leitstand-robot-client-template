"""The ROS 2 entry point: one node, one executor, the adapters wired into the client."""

from __future__ import annotations

import argparse
import logging
import os
import sys

import rclpy
import zenoh
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter

from leitstand_client import (
    FakeNavigation,
    LeitstandClient,
    Navigation,
    build_zenoh_config,
    load_spec,
    open_session,
)
from leitstand_client.config import RobotSpec
from leitstand_client_ros2.config import Nav2Config
from leitstand_client_ros2.nav2_navigation import Nav2Navigation
from leitstand_client_ros2.pose_relay import PoseRelay

logger = logging.getLogger(__name__)


def _parse_args(argv: list[str]) -> tuple[str, list[str]]:
    parser = argparse.ArgumentParser(prog="leitstand_client_ros2")
    parser.add_argument("--config", required=True, help="path to robot.yaml")
    known, rest = parser.parse_known_args(argv[1:])
    return known.config, [argv[0], *rest]


def _apply_environment(spec: RobotSpec) -> None:
    """Set what rclpy reads at init: the domain from the file, a Cyclone config only if named.

    A robot's bringup usually sets the right Cyclone config system-wide, so an empty value
    inherits it rather than clearing it.
    """
    os.environ["ROS_DOMAIN_ID"] = str(spec.ros.domain)
    if spec.ros.cyclonedds_uri:
        os.environ["CYCLONEDDS_URI"] = spec.ros.cyclonedds_uri


def _nav2_config(spec: RobotSpec) -> Nav2Config | None:
    """Validate the nav2: block up front, so a misspelt key fails before ROS or Zenoh start."""
    if spec.navigation != "nav2":
        return None
    return Nav2Config.model_validate(spec.nav2 or {})


def _navigation(spec: RobotSpec, nav2_config: Nav2Config | None, node: Node) -> Navigation:
    """Construct the Navigation named in the file: add your robot's own class here."""
    if spec.navigation == "fake":
        logger.info("[node] navigation: fake (drives nothing)")
        return FakeNavigation(speed_mps=spec.fake.speed_mps)
    if spec.navigation == "nav2" and nav2_config is not None:
        logger.info("[node] navigation: nav2 (%s)", nav2_config.actions.navigate_through_poses)
        return Nav2Navigation(nav2_config, node)
    raise ValueError(
        f"navigation {spec.navigation!r} is not one this package ships (nav2, fake); "
        "construct it in _navigation() in node.py of your own copy of the package"
    )


def _pose_source(spec: RobotSpec, session: zenoh.Session, node: Node) -> PoseRelay | None:
    if spec.telemetry is None or spec.telemetry.pose is None:
        return None
    return PoseRelay(session, spec.id, spec.telemetry.pose, node)


def main(argv: list[str] | None = None) -> int:
    logging.basicConfig(
        level=os.getenv("LEITSTAND_LOG_LEVEL", "INFO").upper(),
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )
    config_path, ros_argv = _parse_args(argv if argv is not None else sys.argv)
    try:
        spec = load_spec(config_path)
        nav2_config = _nav2_config(spec)
    except ValueError as exc:
        logger.error("[node] %s", exc)
        return 1
    _apply_environment(spec)
    try:
        session = open_session(build_zenoh_config(spec.leitstand))
    except RuntimeError as exc:
        logger.error("[node] %s", exc)
        return 1

    rclpy.init(args=ros_argv)
    node = Node(
        spec.ros.node_name,
        parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, spec.ros.use_sim_time)],
    )
    client = LeitstandClient(
        spec, session, _navigation(spec, nav2_config, node), _pose_source(spec, session, node)
    )

    # rclpy installs handlers for SIGINT and SIGTERM itself and wakes the executor from inside
    # rcl_wait; a Python-level handler would never run while spin() blocks in C.
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        client.start()
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        logger.info("[node] shutting down")
    finally:
        client.stop()
        try:
            session.close()
        except Exception as exc:  # noqa: BLE001
            logger.warning("[node] session close failed: %s", exc)
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
