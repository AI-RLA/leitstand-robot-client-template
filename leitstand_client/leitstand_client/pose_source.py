"""What a robot must implement to publish its position."""

from __future__ import annotations

from typing import Protocol


class PoseSource(Protocol):
    """Publishes ``leitstand.robot.v1.Pose`` on the Zenoh session given at construction."""

    def start(self) -> None:
        """Begin publishing. Main thread."""

    def close(self) -> None:
        """Stop publishing. Main thread."""
