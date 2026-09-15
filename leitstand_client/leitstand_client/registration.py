"""Going online: the config file, the Zenoh session, and the identity the backend asks for."""

from __future__ import annotations

import json
import logging
import time
from pathlib import Path
from typing import Callable

import yaml
import zenoh

from leitstand_client.config import LeitstandSpec, RobotSpec

logger = logging.getLogger(__name__)


def load_spec(path: str | Path) -> RobotSpec:
    """Read and validate robot.yaml; raise ``ValueError`` on anything wrong with it."""
    p = Path(path)
    if not p.is_file():
        raise ValueError(f"spec file not found: {path}")
    try:
        with p.open("r", encoding="utf-8") as fh:
            data = yaml.safe_load(fh)
    except yaml.YAMLError as e:
        raise ValueError(f"spec file {path} is not valid YAML: {e}") from e
    if not isinstance(data, dict):
        raise ValueError(f"spec file {path} must contain a YAML mapping at top level")
    try:
        return RobotSpec.model_validate(data)
    except Exception as e:
        raise ValueError(f"spec file {path} is invalid: {e}") from e


def build_zenoh_config(leitstand: LeitstandSpec) -> zenoh.Config:
    """Build the Zenoh config that reaches the Leitstand router.

    Multicast scouting is off because the robot's LAN is usually a different L2 segment from
    the router's; only an explicit endpoint reaches across.
    """
    cfg = zenoh.Config()
    cfg.insert_json5("mode", json.dumps(leitstand.zenoh_mode))
    cfg.insert_json5("connect/endpoints", json.dumps([leitstand.endpoint]))
    cfg.insert_json5("scouting/multicast/enabled", "false")
    logger.info(
        "[zenoh] connect endpoint: %s (mode=%s, multicast=off)",
        leitstand.endpoint,
        leitstand.zenoh_mode,
    )
    return cfg


def open_session(cfg: zenoh.Config, attempts: int = 5, base_delay_s: float = 1.0) -> zenoh.Session:
    """Open a Zenoh session, retrying with exponential backoff; raise after ``attempts``."""
    delay = base_delay_s
    for attempt in range(1, attempts + 1):
        try:
            session = zenoh.open(cfg)
            logger.info("[zenoh] session opened")
            return session
        except (OSError, zenoh.ZError) as e:
            logger.warning("[zenoh] connection attempt %d failed: %s", attempt, e)
            if attempt == attempts:
                raise RuntimeError("failed to establish Zenoh connection after retries") from e
            logger.info("[zenoh] retrying in %.1f seconds", delay)
            time.sleep(delay)
            delay *= 2.0


def metadata_payload(robot_id: str, active_run_id: str | None) -> bytes:
    """The identity JSON the backend queries on liveliness, with the run being executed."""
    return json.dumps(
        {"id": robot_id, "active_run_id": active_run_id}, separators=(",", ":")
    ).encode("utf-8")


def metadata_handler(
    robot_id: str, key: str, active_run_id: Callable[[], str | None]
) -> Callable[[zenoh.Query], None]:
    """Return a queryable callback that answers the identity JSON built at query time."""

    def _handler(query: zenoh.Query) -> None:
        try:
            query.reply(key, metadata_payload(robot_id, active_run_id()))
        except Exception as e:  # noqa: BLE001
            logger.exception("[zenoh] metadata reply failed: %s", e)

    return _handler
