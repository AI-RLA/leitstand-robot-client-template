"""Unit tests for the run_id-keyed dispatch decision.

Idempotency is anchored on the run_id (not the per-attempt
dispatch_id): a re-delivered dispatch of the running mission is a no-op accept,
a different mission while busy is rejected, an idle robot accepts.
"""

from __future__ import annotations

from leitstand_client.mission_executor import _dispatch_decision, _DispatchDecision


def test_idle_accepts() -> None:
    assert _dispatch_decision(None, "m1") is _DispatchDecision.ACCEPT


def test_same_mission_is_idempotent_duplicate() -> None:
    assert _dispatch_decision("m1", "m1") is _DispatchDecision.DUPLICATE


def test_different_mission_is_busy() -> None:
    assert _dispatch_decision("m1", "m2") is _DispatchDecision.BUSY
