"""The GNSS conversions: what reaches the pose key, and what a producer's defaults must not become."""

from __future__ import annotations

from types import SimpleNamespace

import pytest

from leitstand_client.config import PoseSpec
from leitstand_client_ros2.gnss import pose_from_gpsfix, pose_from_navsatfix


def _gpsfix(dip: float = 0.0) -> SimpleNamespace:
    return SimpleNamespace(
        latitude=52.3,
        longitude=8.05,
        dip=dip,
        header=SimpleNamespace(stamp=SimpleNamespace(sec=10, nanosec=5)),
        position_covariance_type=0,
        position_covariance=[0.0] * 9,
    )


def test_a_gpsfix_without_the_switch_carries_no_heading():
    """GPSFix.dip is 0.0 on every receiver but Septentrio, which would read as heading 90."""
    pose = pose_from_gpsfix(_gpsfix(dip=0.0), heading_from_dip=False)

    assert pose is not None
    assert not pose.HasField("heading_deg")


def test_a_septentrio_dip_becomes_a_compass_bearing_when_asked():
    pose = pose_from_gpsfix(_gpsfix(dip=0.0), heading_from_dip=True)

    assert pose is not None
    assert pose.heading_deg == 90.0


def test_a_navsatfix_never_carries_a_heading():
    msg = SimpleNamespace(
        latitude=52.3,
        longitude=8.05,
        header=SimpleNamespace(stamp=SimpleNamespace(sec=10, nanosec=5)),
        position_covariance_type=0,
        position_covariance=[0.0] * 9,
    )

    pose = pose_from_navsatfix(msg, heading_from_dip=True)

    assert pose is not None
    assert not pose.HasField("heading_deg")


def test_the_dip_switch_is_refused_for_navsatfix():
    with pytest.raises(ValueError):
        PoseSpec(msg_type="sensor_msgs/msg/NavSatFix", topic="/fix", heading_from_dip=True)
