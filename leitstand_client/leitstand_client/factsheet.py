"""The robot's capability declaration: built from config, served on a Zenoh queryable."""

from __future__ import annotations

import logging
from typing import Any

from leitstand.robot.v1 import factsheet_pb2

from leitstand_client import keys, proto_json
from leitstand_client.config import FactsheetSpec

logger = logging.getLogger(__name__)

_WAYPOINT_KIND = {
    "wgs84": factsheet_pb2.WAYPOINT_KIND_WGS84,
    "site_local": factsheet_pb2.WAYPOINT_KIND_SITE_LOCAL,
}


def build_factsheet(spec: FactsheetSpec) -> factsheet_pb2.Factsheet:
    """Map a FactsheetSpec to a proto Factsheet; raise if it declares no capabilities."""
    capabilities = []
    if spec.navigation is not None:
        capabilities.append(
            factsheet_pb2.StageCapability(
                navigation=factsheet_pb2.NavigationCapability(
                    supported_waypoint_kinds=[
                        _WAYPOINT_KIND[kind] for kind in spec.navigation.supported_waypoint_kinds
                    ]
                )
            )
        )
    if spec.coverage is not None:
        capabilities.append(
            factsheet_pb2.StageCapability(
                coverage=factsheet_pb2.CoverageCapability(
                    supported_waypoint_kinds=[
                        _WAYPOINT_KIND[kind] for kind in spec.coverage.supported_waypoint_kinds
                    ]
                )
            )
        )
    if not capabilities:
        raise ValueError("factsheet declares no capabilities")
    factsheet = factsheet_pb2.Factsheet(stage_capabilities=capabilities)
    if spec.physical_parameters is not None:
        # Set through the submessage rather than the constructor so an unconfigured robot leaves
        # the field absent: zero is a real turning radius, and must not read as "not declared".
        factsheet.physical_parameters.track_width_m = spec.physical_parameters.track_width_m
        factsheet.physical_parameters.min_turning_radius_m = (
            spec.physical_parameters.min_turning_radius_m
        )
    return factsheet


class FactsheetPublisher:
    """Replies to backend factsheet queries with the robot's capability declaration."""

    def __init__(self, session: Any, robot_id: str, spec: FactsheetSpec) -> None:
        self._key = keys.FACTSHEET.format(robot_id=robot_id)
        # build_factsheet raises if the spec declares no capabilities, so a misconfigured robot
        # fails here rather than serving an empty factsheet the backend would reject opaquely.
        self._payload = proto_json.to_json(build_factsheet(spec))
        self._queryable = session.declare_queryable(self._key, self._handle)
        logger.info("[factsheet] queryable declared: %s", self._key)

    def _handle(self, query: Any) -> None:
        try:
            query.reply(self._key, self._payload)
        except Exception as exc:  # noqa: BLE001
            logger.exception("[factsheet] reply failed: %s", exc)

    def close(self) -> None:
        try:
            self._queryable.undeclare()
        except Exception as exc:  # noqa: BLE001
            logger.warning("[factsheet] undeclare failed: %s", exc)
