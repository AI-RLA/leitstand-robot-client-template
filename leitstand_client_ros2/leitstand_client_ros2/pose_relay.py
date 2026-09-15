"""GNSS pose relay: a ROS GNSS topic (DDS) -> leitstand/robot/<id>/pose (Zenoh, proto-JSON).

Subscribes on DDS rather than to the bridge's Zenoh re-publication because two Zenoh clients on
one router cannot peer-link, so a Zenoh-side subscriber would round-trip every sample through
the Leitstand router.
"""

from __future__ import annotations

import logging
from typing import Any, Callable

import zenoh
from leitstand.robot.v1 import telemetry_pb2
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix

from leitstand_client import keys, proto_json
from leitstand_client.config import PoseSpec
from leitstand_client_ros2.gnss import pose_from_gpsfix, pose_from_navsatfix

try:
    from gps_msgs.msg import GPSFix
except ImportError:  # optional: a robot that publishes NavSatFix need not install gps_msgs
    GPSFix = None

logger = logging.getLogger(__name__)

_RELAYS: dict[str, tuple[type | None, Callable[[Any, bool], telemetry_pb2.Pose | None]]] = {
    "gps_msgs/msg/GPSFix": (GPSFix, pose_from_gpsfix),
    "sensor_msgs/msg/NavSatFix": (NavSatFix, pose_from_navsatfix),
}


class PoseRelay:
    """Relays one GNSS topic to the robot's pose key; a PoseSource on a node the caller owns."""

    def __init__(self, session: zenoh.Session, robot_id: str, spec: PoseSpec, node: Node) -> None:
        self._session = session
        self._robot_id = robot_id
        self._node = node
        self._topic = spec.topic if spec.topic.startswith("/") else f"/{spec.topic}"
        self._message_class, self._convert = _RELAYS[spec.msg_type]
        if self._message_class is None:
            raise ValueError(f"{spec.msg_type} needs the gps_msgs package, which is not installed")
        self._heading_from_dip = spec.heading_from_dip
        self._sink_key = keys.POSE.format(robot_id=robot_id)
        self._subscription: Any = None

    @property
    def sink_key(self) -> str:
        return self._sink_key

    def start(self) -> None:
        self._subscription = self._node.create_subscription(
            self._message_class, self._topic, self._on_msg, qos_profile_sensor_data
        )
        logger.info("[pose-relay] %s -> %s", self._topic, self._sink_key)

    def close(self) -> None:
        if self._subscription is not None:
            try:
                self._node.destroy_subscription(self._subscription)
            except Exception as exc:  # noqa: BLE001
                logger.warning("[pose-relay] unsubscribe failed: %s", exc)
            self._subscription = None

    def _on_msg(self, msg: Any) -> None:
        try:
            pose = self._convert(msg, self._heading_from_dip)
            if pose is None:
                return
            self._session.put(
                self._sink_key,
                proto_json.to_json(pose),
                encoding=zenoh.Encoding.APPLICATION_JSON,
            )
        except Exception as e:  # noqa: BLE001 - never let the DDS callback raise
            logger.warning("[pose-relay] sample handling failed for %s: %s", self._robot_id, e)
