"""The key expressions render, and the id rule matches the backend's."""

from __future__ import annotations

import pytest

from leitstand_client import keys


def test_every_key_renders_with_the_robot_id() -> None:
    for name in (
        "ONLINE",
        "METADATA",
        "FACTSHEET",
        "POSE",
        "SEND_GOAL",
        "CANCEL_GOAL",
        "PAUSE",
        "RESUME",
        "STATE",
    ):
        key = getattr(keys, name).format(robot_id="scout_2")
        assert key.startswith("leitstand/robot/scout_2/"), key


@pytest.mark.parametrize("robot_id", ["scout_2", "a", "0", "robot-x_1"])
def test_validate_robot_id_accepts(robot_id: str) -> None:
    assert keys.validate_robot_id(robot_id) == robot_id


@pytest.mark.parametrize("robot_id", ["Scout", "2 robots", "", "-lead", "robot/x", None, 42])
def test_validate_robot_id_rejects(robot_id: object) -> None:
    with pytest.raises(ValueError):
        keys.validate_robot_id(robot_id)
