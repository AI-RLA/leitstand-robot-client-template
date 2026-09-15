"""Local tangent-plane geometry, so converting a mission costs one service call rather than one
per waypoint."""

from __future__ import annotations

import math

# WGS84 ellipsoid.
_SEMI_MAJOR_AXIS_M = 6378137.0
_ECCENTRICITY_SQUARED = 6.69437999014e-3


def enu_offset(
    origin_lat_deg: float,
    origin_lon_deg: float,
    lat_deg: float,
    lon_deg: float,
) -> tuple[float, float]:
    """Return a point's (east, north) offset in metres from an origin.

    Flattens the ellipsoid onto the plane that touches it at the origin, using the two radii of
    curvature there. Error grows with the square of the distance from that origin and stays under a
    centimetre across a field, which is far below what the machine can steer to.

    Callers must pick an origin inside the area they are converting, not a fixed datum, or the
    approximation is being asked to hold over a distance it was not chosen for.
    """
    origin_lat_rad = math.radians(origin_lat_deg)
    sin_origin_lat = math.sin(origin_lat_rad)
    denominator = math.sqrt(1.0 - _ECCENTRICITY_SQUARED * sin_origin_lat * sin_origin_lat)

    meridional_radius = _SEMI_MAJOR_AXIS_M * (1.0 - _ECCENTRICITY_SQUARED) / denominator**3
    normal_radius = _SEMI_MAJOR_AXIS_M / denominator

    north = math.radians(lat_deg - origin_lat_deg) * meridional_radius
    east = math.radians(lon_deg - origin_lon_deg) * normal_radius * math.cos(origin_lat_rad)
    return east, north
