"""Unit tests for the FactsheetSpec -> proto Factsheet builder + spec validation."""

from __future__ import annotations

import pytest
from leitstand.robot.v1 import factsheet_pb2
from pydantic import ValidationError

from leitstand_client.config import (
    CoverageCapabilitySpec,
    FactsheetSpec,
    NavigationCapabilitySpec,
    PhysicalParametersSpec,
)
from leitstand_client.factsheet import build_factsheet


def test_navigation_default_is_wgs84() -> None:
    factsheet = build_factsheet(FactsheetSpec())
    caps = factsheet.stage_capabilities
    assert [c.WhichOneof("detail") for c in caps] == ["navigation"]
    assert list(caps[0].navigation.supported_waypoint_kinds) == [factsheet_pb2.WAYPOINT_KIND_WGS84]


def test_site_local_maps_to_proto_kind() -> None:
    spec = FactsheetSpec.model_validate(
        {
            "navigation": {"supported_waypoint_kinds": ["wgs84", "site_local"]},
            "sites": [
                {
                    "site_id": "00000000-0000-0000-0000-000000000001",
                    "nav2_map_file": "/etc/leitstand/maps/a.yaml",
                }
            ],
        }
    )
    kinds = list(build_factsheet(spec).stage_capabilities[0].navigation.supported_waypoint_kinds)
    assert kinds == [factsheet_pb2.WAYPOINT_KIND_WGS84, factsheet_pb2.WAYPOINT_KIND_SITE_LOCAL]


def test_no_capabilities_raises() -> None:
    with pytest.raises(ValueError):
        build_factsheet(FactsheetSpec(navigation=None))


def test_site_local_without_sites_rejected_at_spec_load() -> None:
    with pytest.raises(ValidationError):
        FactsheetSpec.model_validate({"navigation": {"supported_waypoint_kinds": ["site_local"]}})


def test_unknown_waypoint_kind_rejected_at_spec_load() -> None:
    with pytest.raises(ValidationError):
        FactsheetSpec.model_validate({"navigation": {"supported_waypoint_kinds": ["lunar"]}})


def test_physical_parameters_are_declared_when_configured():
    spec = FactsheetSpec(
        navigation=NavigationCapabilitySpec(supported_waypoint_kinds=["wgs84"]),
        physical_parameters=PhysicalParametersSpec(track_width_m=0.58, min_turning_radius_m=0.0),
    )

    factsheet = build_factsheet(spec)

    assert factsheet.HasField("physical_parameters")
    assert factsheet.physical_parameters.track_width_m == 0.58
    assert factsheet.physical_parameters.min_turning_radius_m == 0.0


def test_an_unconfigured_machine_declares_nothing_rather_than_zero():
    """A zero radius means turns on the spot, so it must not stand in for "not declared"."""
    spec = FactsheetSpec(navigation=NavigationCapabilitySpec(supported_waypoint_kinds=["wgs84"]))

    factsheet = build_factsheet(spec)

    assert not factsheet.HasField("physical_parameters")


def test_coverage_capability_is_declared_when_configured():
    """Without it the backend must not send this robot a coverage plan."""
    spec = FactsheetSpec(coverage=CoverageCapabilitySpec(supported_waypoint_kinds=["wgs84"]))

    factsheet = build_factsheet(spec)

    kinds = [c.WhichOneof("detail") for c in factsheet.stage_capabilities]
    assert "coverage" in kinds
    coverage = next(
        c.coverage for c in factsheet.stage_capabilities if c.WhichOneof("detail") == "coverage"
    )
    assert list(coverage.supported_waypoint_kinds) == [factsheet_pb2.WAYPOINT_KIND_WGS84]


def test_a_robot_that_only_drives_points_declares_no_coverage():
    factsheet = build_factsheet(FactsheetSpec())

    assert [c.WhichOneof("detail") for c in factsheet.stage_capabilities] == ["navigation"]
