"""The Zenoh key expressions a robot owns, and the shape of the id inside them."""

from __future__ import annotations

import re

ONLINE = "leitstand/robot/{robot_id}/online"
METADATA = "leitstand/robot/{robot_id}/metadata"
FACTSHEET = "leitstand/robot/{robot_id}/factsheet"
POSE = "leitstand/robot/{robot_id}/pose"
BATTERY = "leitstand/robot/{robot_id}/battery"
SEND_GOAL = "leitstand/robot/{robot_id}/mission/_action/send_goal"
CANCEL_GOAL = "leitstand/robot/{robot_id}/mission/_action/cancel_goal"
PAUSE = "leitstand/robot/{robot_id}/mission/_action/pause"
RESUME = "leitstand/robot/{robot_id}/mission/_action/resume"
STATE = "leitstand/robot/{robot_id}/mission/state"

# The backend rejects any other shape before it looks at the payload.
ROBOT_ID_RE = re.compile(r"^[a-z0-9][a-z0-9_-]*$")


def validate_robot_id(value: object) -> str:
    """Return ``value`` if it is a valid robot id, else raise ``ValueError``."""
    if not isinstance(value, str) or not value:
        raise ValueError("robot id must be a non-empty string")
    if not ROBOT_ID_RE.fullmatch(value):
        raise ValueError(
            f"robot id {value!r} is not valid; allowed [a-z0-9_-], must start with [a-z0-9]"
        )
    return value
