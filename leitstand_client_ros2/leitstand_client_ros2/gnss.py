"""Convert a ROS GNSS fix to the contract's Pose; no ROS needed, the messages are duck-typed."""

from __future__ import annotations

import math
from typing import Any

from leitstand.robot.v1 import telemetry_pb2


def _horizontal_accuracy_m(msg: Any) -> float | None:
    # position_covariance_type 0 means UNKNOWN; diagonals aren't trustworthy.
    if msg.position_covariance_type == 0:
        return None
    c0 = msg.position_covariance[0]
    c4 = msg.position_covariance[4]
    if not (math.isfinite(c0) and math.isfinite(c4)):
        return None
    worst = max(c0, c4)
    if worst < 0:
        return None
    return math.sqrt(worst)


def pose_from_gpsfix(msg: Any, heading_from_dip: bool) -> telemetry_pb2.Pose | None:
    lat, lon = msg.latitude, msg.longitude
    if not (math.isfinite(lat) and math.isfinite(lon)):
        return None
    if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
        return None

    pose = telemetry_pb2.Pose(lat=lat, lon=lon)
    pose.timestamp.seconds = msg.header.stamp.sec
    pose.timestamp.nanos = msg.header.stamp.nanosec
    # Septentrio publishes the heading in dip, in ROS convention (0 = East, counter-clockwise);
    # the backend wants a compass bearing, so (90 - dip) flips it and mod 360 folds the sign.
    if heading_from_dip and math.isfinite(msg.dip):
        pose.heading_deg = (90.0 - msg.dip) % 360.0
    accuracy = _horizontal_accuracy_m(msg)
    if accuracy is not None:
        pose.horizontal_accuracy_m = accuracy
    return pose


def pose_from_navsatfix(msg: Any, heading_from_dip: bool) -> telemetry_pb2.Pose | None:
    """Convert sensor_msgs/msg/NavSatFix to a proto Pose. Heading is always absent."""
    lat, lon = msg.latitude, msg.longitude
    if not (math.isfinite(lat) and math.isfinite(lon)):
        return None
    if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
        return None

    pose = telemetry_pb2.Pose(lat=lat, lon=lon)
    pose.timestamp.seconds = msg.header.stamp.sec
    pose.timestamp.nanos = msg.header.stamp.nanosec
    accuracy = _horizontal_accuracy_m(msg)
    if accuracy is not None:
        pose.horizontal_accuracy_m = accuracy
    return pose
