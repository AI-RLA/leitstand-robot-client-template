"""robot.yaml validation: what passes, what fails, and what passes through."""

from __future__ import annotations

from pathlib import Path

import pytest
from pydantic import ValidationError

from leitstand_client.config import RobotSpec
from leitstand_client.registration import load_spec

_MINIMAL = {"id": "scout_2", "leitstand": {"endpoint": "tcp/localhost:7447"}}


def test_minimal_spec_validates_with_defaults() -> None:
    spec = RobotSpec.model_validate(_MINIMAL)
    assert spec.navigation == "nav2"
    assert spec.leitstand.zenoh_mode == "client"
    assert spec.ros.node_name == "leitstand_client"
    assert spec.nav2 is None


def test_nav2_block_passes_through_untouched() -> None:
    spec = RobotSpec.model_validate(
        {**_MINIMAL, "nav2": {"controller_id": "MPPI", "frames": {"map": "map"}}}
    )
    assert spec.nav2 == {"controller_id": "MPPI", "frames": {"map": "map"}}


@pytest.mark.parametrize(
    "bad",
    [
        {**_MINIMAL, "navigaton": "nav2"},
        {**_MINIMAL, "ros": {"domain": 0, "use_sim": True}},
        {**_MINIMAL, "factsheet": {"navigation": {"supported_waypoint_kinds": ["wgs84"], "x": 1}}},
    ],
    ids=["top-level typo", "nested typo", "capability typo"],
)
def test_unknown_key_at_any_level_fails(bad: dict) -> None:
    with pytest.raises(ValidationError):
        RobotSpec.model_validate(bad)


def test_unknown_msg_type_fails_at_load() -> None:
    with pytest.raises(ValidationError):
        RobotSpec.model_validate(
            {
                **_MINIMAL,
                "telemetry": {"pose": {"msg_type": "gps_msgs/msg/GpsFix", "topic": "/gps"}},
            }
        )


def test_invalid_id_fails_at_load() -> None:
    with pytest.raises(ValidationError):
        RobotSpec.model_validate({**_MINIMAL, "id": "Scout"})


def test_site_local_without_sites_fails_for_coverage_too() -> None:
    with pytest.raises(ValidationError):
        RobotSpec.model_validate(
            {**_MINIMAL, "factsheet": {"coverage": {"supported_waypoint_kinds": ["site_local"]}}}
        )


def test_load_spec_returns_a_spec(tmp_path: Path) -> None:
    p = tmp_path / "robot.yaml"
    p.write_text("id: scout_2\nleitstand:\n  endpoint: tcp/localhost:7447\n")
    assert load_spec(p).id == "scout_2"


@pytest.mark.parametrize(
    "text",
    ["", "- not\n- a mapping\n", "id: [unclosed\n", "id: scout_2\n"],
    ids=["empty", "list", "bad yaml", "missing endpoint"],
)
def test_load_spec_rejects(tmp_path: Path, text: str) -> None:
    p = tmp_path / "robot.yaml"
    p.write_text(text)
    with pytest.raises(ValueError):
        load_spec(p)


def test_load_spec_missing_file(tmp_path: Path) -> None:
    with pytest.raises(ValueError):
        load_spec(tmp_path / "nope.yaml")
