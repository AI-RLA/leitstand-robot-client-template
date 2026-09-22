"""The nav2: block of robot.yaml: every name, frame and tuning value the Nav2 adapter needs."""

from __future__ import annotations

from pydantic import BaseModel, ConfigDict, Field


class Actions(BaseModel):
    model_config = ConfigDict(extra="forbid")

    navigate_through_poses: str = "navigate_through_poses"
    navigate_to_pose: str = "navigate_to_pose"
    follow_path: str = "follow_path"


class Frames(BaseModel):
    model_config = ConfigDict(extra="forbid")

    map: str = "map"
    base: str = "base_footprint"


class Nav2Config(BaseModel):
    """Defaults match a stock Nav2 bringup; a standard robot omits the block entirely."""

    model_config = ConfigDict(extra="forbid")

    actions: Actions = Field(default_factory=Actions)
    frames: Frames = Field(default_factory=Frames)
    # The controller_server plugin name in the robot's own nav2 params.
    controller_id: str = "FollowPath"
    projection_service: str = "/fromLL"
    projection_timeout_s: float = Field(default=2.0, gt=0)
    send_goal_timeout_s: float = Field(default=5.0, gt=0)
    follow_path_attempts: int = Field(default=3, ge=1)
    follow_path_retry_s: float = Field(default=5.0, ge=0)
    # After an abort, resume this far behind the machine so the controller has path to lock onto.
    resume_behind_m: float = Field(default=3.0, ge=0)
    # Farther than this from the path, drive to it first: the controller only follows a path
    # where it crosses the local costmap.
    approach_tolerance_m: float = Field(default=2.0, gt=0)
    max_pose_spacing_m: float = Field(default=0.5, gt=0)
    progress_tick_s: float = Field(default=0.5, gt=0)
    # The path handed to FollowPath, published here so RViz can show it (Nav2 does not).
    commanded_path_topic: str = "/leitstand/coverage_path"
    # A twist_mux input with top priority. Empty means an IMMEDIATE cancel is the same as a
    # GRACEFUL one: cancel the goal and let the controller stop.
    stop_topic: str = ""
