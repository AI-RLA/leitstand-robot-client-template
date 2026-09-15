"""open_session retries the Zenoh connection and gives up with one error."""

import zenoh

from leitstand_client.registration import open_session


def test_open_session_retries_then_returns(monkeypatch):
    calls = []

    def flaky_open(cfg):
        calls.append(cfg)
        if len(calls) < 3:
            raise zenoh.ZError("unable to connect")
        return "session"

    monkeypatch.setattr(zenoh, "open", flaky_open)
    assert open_session("cfg", attempts=3, base_delay_s=0.0) == "session"
    assert len(calls) == 3


def test_open_session_gives_up_after_attempts(monkeypatch):
    def never_open(cfg):
        raise zenoh.ZError("unable to connect")

    monkeypatch.setattr(zenoh, "open", never_open)
    try:
        open_session("cfg", attempts=2, base_delay_s=0.0)
    except RuntimeError as exc:
        assert isinstance(exc.__cause__, zenoh.ZError)
    else:
        raise AssertionError("open_session returned without a session")


def test_metadata_names_the_active_run_at_query_time() -> None:
    import json

    from leitstand_client.registration import metadata_handler

    class _Query:
        replies: list[bytes] = []

        def reply(self, key: str, payload: bytes) -> None:
            self.replies.append(payload)

    holder = {"run": None}
    handler = metadata_handler("r1", "k", lambda: holder["run"])
    query = _Query()
    handler(query)
    holder["run"] = "11111111-1111-4111-8111-111111111111"
    handler(query)
    assert [json.loads(r) for r in query.replies] == [
        {"id": "r1", "active_run_id": None},
        {"id": "r1", "active_run_id": holder["run"]},
    ]
