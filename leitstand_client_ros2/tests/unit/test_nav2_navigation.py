"""Planned-path execution in the Nav2 frame manager, driven against recorders.

The module imports without ROS; only construction needs it, so the manager is built bare and
its Nav2 legs are replaced with fakes that record what was asked of them.
"""

from __future__ import annotations

import asyncio
import threading
import time
from types import SimpleNamespace

import pytest
from leitstand.robot.v1 import mission_pb2, mission_state_pb2

from leitstand_client.navigation import StageResult
from leitstand_client_ros2 import nav2_navigation as nav
from leitstand_client_ros2.config import Nav2Config

# action_msgs/GoalStatus.STATUS_SUCCEEDED, without importing ROS.
_SUCCEEDED = 4

FINISHED = StageResult(status=mission_state_pb2.STAGE_STATUS_FINISHED)


def _pose(x: float, y: float) -> SimpleNamespace:
    return SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=x, y=y)))


def _line(spacing_m: float, count: int) -> list[SimpleNamespace]:
    return [_pose(i * spacing_m, 0.0) for i in range(count)]


class _Recorder:
    """A bare manager whose follow and approach legs answer from scripts.

    A follow outcome is ``(result, aborted, position_afterwards)``; an approach outcome is
    ``(result, aborted)``, and an arrival moves the machine onto the pose it was sent to.
    """

    def __init__(self, monkeypatch, poses, *, position, follow=(), approach=()):
        self.poses = poses
        self.position = position
        self.follow_starts: list[int] = []
        self.approach_targets: list[tuple[float, float]] = []
        follow_script = iter(follow)
        approach_script = iter(approach)

        manager = object.__new__(nav.Nav2Navigation)
        manager._cfg = Nav2Config(follow_path_retry_s=0.0)
        manager._stop_pub = None
        manager._running = threading.Event()
        manager._running.set()
        manager._paused = threading.Event()
        manager._send_goal_timeout_s = 1.0
        manager._follow_path_client = SimpleNamespace(wait_for_server=lambda timeout_sec: True)
        manager._path_pub = SimpleNamespace(publish=lambda path: None)

        async def _build_path_poses(stage):
            return poses

        async def _run_follow_path(poses, start, cancel_requested, on_progress):
            self.follow_starts.append(start)
            result, aborted, self.position = next(follow_script)
            return result, aborted

        async def _drive_to(pose, cancel_requested):
            self.approach_targets.append(nav._xy(pose))
            result, aborted = next(approach_script)
            if result is None and not aborted:
                self.position = nav._xy(pose)
            return result, aborted

        monkeypatch.setattr(manager, "_build_path_poses", _build_path_poses)
        monkeypatch.setattr(manager, "_run_follow_path", _run_follow_path)
        monkeypatch.setattr(manager, "_drive_to", _drive_to)
        monkeypatch.setattr(manager, "_robot_xy", lambda: self.position)
        self.manager = manager

    def run(self) -> StageResult:
        return asyncio.run(self.manager._follow_planned_path(None, lambda: None, on_progress=None))


def test_backed_off_counts_metres_not_poses():
    dense = _line(0.25, 200)
    sparse = _line(5.0, 12)
    assert nav._backed_off(dense, 100, 3.0) == 88
    assert nav._backed_off(sparse, 10, 3.0) == 9
    assert nav._backed_off(sparse, 0, 3.0) == 0
    assert nav._backed_off(dense, 4, 3.0) == 0


def test_filled_fills_straights_and_leaves_dense_runs_alone():
    straight = [(0.0, 0.0, 90.0), (12.0, 0.0, 90.0)]
    assert nav._filled(straight, 5.0) == [
        (0.0, 0.0, 90.0),
        (5.0, 0.0, 90.0),
        (10.0, 0.0, 90.0),
        (12.0, 0.0, 90.0),
    ]
    dense = [(0.0, 0.0, None), (0.25, 0.0, None), (0.5, 0.0, None)]
    assert nav._filled(dense, 5.0) == dense


def test_straights_are_filled_well_inside_the_controller_lookahead():
    """A silent revert to sparse poses would bring back the carrot flipping behind the machine."""
    assert Nav2Config().max_pose_spacing_m <= 2.0


def test_entry_drives_to_the_head_of_the_path_when_the_machine_stands_elsewhere(monkeypatch):
    poses = _line(5.0, 12)
    # Parked six metres beyond the far end: the nearest pose is the last one, and a nearest-first
    # entry would declare the field done.
    rec = _Recorder(
        monkeypatch,
        poses,
        position=(61.0, 0.0),
        follow=[(FINISHED, False, (55.0, 0.0))],
        approach=[(None, False)],
    )
    assert rec.run().status == mission_state_pb2.STAGE_STATUS_FINISHED
    assert rec.approach_targets == [(0.0, 0.0)]
    assert rec.follow_starts == [0]


def test_entry_follows_directly_when_the_machine_is_at_the_head(monkeypatch):
    poses = _line(5.0, 12)
    rec = _Recorder(
        monkeypatch, poses, position=(0.5, 0.3), follow=[(FINISHED, False, (55.0, 0.0))]
    )
    assert rec.run().status == mission_state_pb2.STAGE_STATUS_FINISHED
    assert rec.approach_targets == []
    assert rec.follow_starts == [0]


def test_resume_backs_off_three_metres_from_where_the_machine_stands(monkeypatch):
    poses = _line(0.25, 200)
    rec = _Recorder(
        monkeypatch,
        poses,
        position=(0.0, 0.0),
        follow=[(None, True, (30.0, 0.1)), (FINISHED, False, (49.75, 0.0))],
    )
    assert rec.run().status == mission_state_pb2.STAGE_STATUS_FINISHED
    assert rec.approach_targets == []
    assert rec.follow_starts == [0, 108]


def test_resume_drives_back_onto_the_path_when_the_machine_left_it(monkeypatch):
    poses = _line(5.0, 12)
    rec = _Recorder(
        monkeypatch,
        poses,
        position=(0.0, 0.0),
        follow=[(None, True, (20.0, 15.0)), (FINISHED, False, (55.0, 0.0))],
        approach=[(None, False)],
    )
    assert rec.run().status == mission_state_pb2.STAGE_STATUS_FINISHED
    assert rec.approach_targets == [(20.0, 0.0)]
    assert rec.follow_starts == [0, 3]


def test_follow_path_is_exhausted_after_three_aborts(monkeypatch):
    poses = _line(5.0, 12)
    rec = _Recorder(
        monkeypatch,
        poses,
        position=(0.0, 0.0),
        follow=[(None, True, (0.0, 0.0))] * 3,
    )
    result = rec.run()
    assert result.status == mission_state_pb2.STAGE_STATUS_FAILED
    assert result.error.type == "follow_path_exhausted"
    assert rec.follow_starts == [0, 0, 0]


def test_approach_aborts_spend_the_same_budget(monkeypatch):
    poses = _line(5.0, 12)
    rec = _Recorder(
        monkeypatch,
        poses,
        position=(61.0, 0.0),
        approach=[(None, True)] * 3,
    )
    result = rec.run()
    assert result.status == mission_state_pb2.STAGE_STATUS_FAILED
    assert result.error.type == "follow_path_exhausted"
    assert rec.approach_targets == [(0.0, 0.0)] * 3
    assert rec.follow_starts == []


def test_resume_without_a_position_fails_rather_than_guessing(monkeypatch):
    poses = _line(5.0, 12)
    rec = _Recorder(
        monkeypatch,
        poses,
        position=(0.0, 0.0),
        follow=[(None, True, None)],
    )
    result = rec.run()
    assert result.status == mission_state_pb2.STAGE_STATUS_FAILED
    assert result.error.type == "resume_position_unknown"


class _FakeHandle:
    """One accepted Nav2 goal whose result the test resolves when it chooses."""

    def __init__(self, result: asyncio.Future) -> None:
        self.accepted = True
        self.cancelled = False
        self._result = result

    def get_result_async(self) -> asyncio.Future:
        return self._result

    def cancel_goal_async(self) -> asyncio.Future:
        self.cancelled = True
        done: asyncio.Future = asyncio.get_running_loop().create_future()
        done.set_result(SimpleNamespace(return_code=0))
        return done


class _Approach:
    """A bare manager whose navigate action is a fake, for driving _drive_to directly.

    ``positions`` is walked one entry per tick, the last one repeating, so a test can put the
    machine somewhere and then move it.
    """

    def __init__(self, monkeypatch, *, positions, goal_status=None):
        self.positions = list(positions)
        self.goals: list[object] = []
        self.handle: _FakeHandle | None = None

        manager = object.__new__(nav.Nav2Navigation)
        manager._cfg = Nav2Config(progress_tick_s=0.001)
        manager._stop_pub = None
        manager._GoalStatus = SimpleNamespace(STATUS_SUCCEEDED=_SUCCEEDED)
        manager._running = threading.Event()
        manager._running.set()
        manager._paused = threading.Event()
        manager._send_goal_timeout_s = 1.0
        manager._UUID = lambda uuid: SimpleNamespace(uuid=uuid)
        manager._requested_goals = set()
        manager._goal_statuses_lock = threading.Lock()
        manager._cancel_accepted = _CANCEL_ACCEPTED
        manager._NavigateToPose = SimpleNamespace(Goal=lambda: SimpleNamespace(pose=None))

        def _robot_xy():
            position = self.positions[0]
            if len(self.positions) > 1:
                self.positions.pop(0)
            return position

        def _send_goal_async(goal, feedback_callback=None, goal_uuid=None):
            self.goals.append(goal)
            loop = asyncio.get_running_loop()
            result: asyncio.Future = loop.create_future()
            if goal_status is not None:
                result.set_result(SimpleNamespace(status=goal_status))
            self.handle = _FakeHandle(result)
            accepted: asyncio.Future = loop.create_future()
            accepted.set_result(self.handle)
            return accepted

        manager._approach_client = SimpleNamespace(
            wait_for_server=lambda timeout_sec: True,
            send_goal_async=_send_goal_async,
        )

        async def _await(future):
            return await future

        monkeypatch.setattr(nav, "_await_rclpy", _await)
        monkeypatch.setattr(manager, "_robot_xy", _robot_xy)
        self.manager = manager

    def run(self, target=(0.0, 0.0)):
        return asyncio.run(self.manager._drive_to(_pose(*target), lambda: None))


def test_approach_waits_for_nav2_even_when_the_machine_is_near(monkeypatch):
    """Nav2's goal checker settles the heading the path starts from, so proximity is not arrival."""
    approach = _Approach(monkeypatch, positions=[(40.0, 0.0), (1.5, 0.0)])
    ran: list[bool] = []

    async def run_and_finish():
        task = asyncio.ensure_future(approach.manager._drive_to(_pose(0.0, 0.0), lambda: None))
        for _ in range(20):
            await asyncio.sleep(0)
        assert not task.done()
        ran.append(True)
        approach.handle.get_result_async().set_result(SimpleNamespace(status=_SUCCEEDED))
        return await task

    assert asyncio.run(run_and_finish()) == (None, False)
    assert ran and approach.handle is not None and not approach.handle.cancelled
    assert approach.goals and approach.goals[0].pose is not None


def test_approach_accepts_a_navigator_abort_that_still_left_the_machine_close(monkeypatch):
    approach = _Approach(monkeypatch, positions=[(1.9, 0.0)], goal_status=5)
    assert approach.run() == (None, False)


def test_approach_reports_an_abort_that_left_the_machine_far_away(monkeypatch):
    approach = _Approach(monkeypatch, positions=[(40.0, 0.0)], goal_status=5)
    assert approach.run() == (None, True)


def test_approach_accepts_a_succeeded_goal(monkeypatch):
    approach = _Approach(monkeypatch, positions=[(40.0, 0.0)], goal_status=_SUCCEEDED)
    assert approach.run() == (None, False)


class _RclpyFuture:
    """The subset of rclpy.task.Future that _await_rclpy uses."""

    def __init__(self) -> None:
        self._callbacks: list = []
        self._result = None
        self._done = False

    def add_done_callback(self, cb) -> None:
        if self._done:
            cb(self)
        else:
            self._callbacks.append(cb)

    def set_result(self, value) -> None:
        self._result, self._done = value, True
        for cb in self._callbacks:
            cb(self)

    def result(self):
        return self._result

    def done(self) -> bool:
        return self._done


class _Handle:
    """A goal handle whose result arrives when the test says so; cancel resolves at once."""

    def __init__(self) -> None:
        self.result_future = _RclpyFuture()
        self.cancelled = 0
        self.accepted = True

    def get_result_async(self) -> _RclpyFuture:
        return self.result_future

    def cancel_goal_async(self) -> _RclpyFuture:
        self.cancelled += 1
        done = _RclpyFuture()
        done.set_result(SimpleNamespace(return_code=0))
        return done


# The action_msgs values the manager reads from GoalStatus and CancelGoal at construction.
_ACTIVE = frozenset({1, 2, 3})
_ENDED = frozenset({4, 5, 6})
_CANCEL_ACCEPTED = frozenset({0, 3})


def _bare_manager() -> nav.Nav2Navigation:
    manager = object.__new__(nav.Nav2Navigation)
    manager._active_statuses = _ACTIVE
    manager._ended_statuses = _ENDED
    manager._cancel_accepted = _CANCEL_ACCEPTED
    manager._requested_goals = set()
    manager._goal_statuses = {}
    manager._goal_statuses_lock = threading.Lock()
    manager._stop_pub = None
    manager._running = threading.Event()
    manager._running.set()
    manager._paused = threading.Event()
    manager._send_goal_timeout_s = 0.2
    return manager


def test_pause_cancels_the_goal_and_reports_paused_only_once_it_has_returned():
    async def run():
        manager = _bare_manager()
        handle = _Handle()
        reports: list[tuple[int, float]] = []
        task = asyncio.create_task(
            manager._wait_goal(
                handle,
                lambda: None,
                lambda st, pr: reports.append((st, pr)),
                lambda: 0.4,
                tick_s=0.01,
            )
        )
        await asyncio.sleep(0.03)
        manager.request_pause()
        before = manager.is_paused()
        await asyncio.sleep(0.05)
        paused = manager.is_paused()
        manager.request_resume()
        outcome, _ = await task
        return before, paused, handle.cancelled, reports, outcome, manager.is_paused()

    before, paused, cancelled, reports, outcome, after = asyncio.run(run())
    assert before is False
    assert paused is True
    assert cancelled == 1
    assert reports == [
        (mission_state_pb2.STAGE_STATUS_PAUSED, 0.4),
        (mission_state_pb2.STAGE_STATUS_RUNNING, 0.4),
    ]
    assert outcome == "paused"
    assert after is False


def test_a_cancel_while_paused_abandons_the_wait():
    async def run():
        manager = _bare_manager()
        handle = _Handle()
        holder = {"mode": None}
        task = asyncio.create_task(
            manager._wait_goal(handle, lambda: holder["mode"], None, lambda: 0.0, tick_s=0.01)
        )
        await asyncio.sleep(0.02)
        manager.request_pause()
        await asyncio.sleep(0.03)
        holder["mode"] = 1
        outcome, _ = await task
        return outcome, manager.is_paused()

    assert asyncio.run(run()) == ("abandoned", False)


_FAILED = mission_state_pb2.STAGE_STATUS_FAILED
_ACTIONS = ("navigate_through_poses", "navigate_to_pose", "follow_path")


class _CancelAllClient:
    """A cancel_goal service client that answers at once and records each request."""

    def __init__(self, action: str) -> None:
        self.srv_name = f"{action}/_action/cancel_goal"
        self.requests: list[object] = []

    def service_is_ready(self) -> bool:
        return True

    def call_async(self, request: object) -> asyncio.Future:
        self.requests.append(request)
        future: asyncio.Future = asyncio.get_running_loop().create_future()
        future.set_result(SimpleNamespace(return_code=0))
        return future


class _CountingPublisher:
    def __init__(self) -> None:
        self.times: list[float] = []

    def publish(self, msg: object) -> None:
        self.times.append(time.monotonic())


def _stopping_manager(
    monkeypatch, *, statuses=None, stop_pub=None, ok=True, stage_result=None
) -> nav.Nav2Navigation:
    manager = _bare_manager()
    manager._cfg = Nav2Config()
    manager._rclpy = SimpleNamespace(ok=lambda: ok)
    manager._send_goal_timeout_s = 0.3
    manager._stop_pub = stop_pub
    manager._Twist = SimpleNamespace
    manager._CancelGoal = SimpleNamespace(Request=lambda: SimpleNamespace(kind="cancel-all"))
    manager._cancel_all_clients = [_CancelAllClient(a) for a in _ACTIONS]
    manager._goal_statuses = dict(statuses or {})
    manager._goal_statuses_lock = threading.Lock()
    manager._requested_goals = set()
    manager._UUID = lambda uuid: SimpleNamespace(uuid=uuid)

    async def _await(future):
        return await future

    monkeypatch.setattr(nav, "_await_rclpy", _await)
    if stage_result is not None:

        async def _execute_stage(stage, robot_id, cancel_requested, on_progress=None):
            return stage_result

        manager._execute_stage = _execute_stage
    return manager


def _status_array(goal_id: bytes, status: int) -> SimpleNamespace:
    """A GoalStatusArray with one goal, as the rclpy subscription delivers it."""
    info = SimpleNamespace(goal_id=SimpleNamespace(uuid=goal_id))
    return SimpleNamespace(status_list=[SimpleNamespace(goal_info=info, status=status)])


def _requests(manager: nav.Nav2Navigation) -> list[int]:
    return [len(c.requests) for c in manager._cancel_all_clients]


@pytest.mark.parametrize("cancel_mode", [None, mission_pb2.CANCEL_MODE_GRACEFUL])
def test_an_unfinished_stage_cancels_every_goal_before_returning(monkeypatch, cancel_mode):
    """With a cancel too, since a navigation failure can arrive in the same moment."""
    manager = _stopping_manager(monkeypatch, stage_result=StageResult(status=_FAILED))
    result = asyncio.run(manager.execute_stage(None, "r1", lambda: cancel_mode))
    assert _requests(manager) == [1, 1, 1]
    assert result.status == _FAILED
    assert result.error is None


def test_a_finished_stage_sends_nothing(monkeypatch):
    manager = _stopping_manager(monkeypatch, stage_result=FINISHED)
    assert asyncio.run(manager.execute_stage(None, "r1", lambda: None)) == FINISHED
    assert _requests(manager) == [0, 0, 0]


def test_an_unconfirmed_stop_is_reported(monkeypatch):
    failed = StageResult(
        status=_FAILED,
        error=mission_state_pb2.Error(
            type="x", description="x", severity=mission_state_pb2.ERROR_SEVERITY_WARNING
        ),
    )
    manager = _stopping_manager(
        monkeypatch, statuses={"follow_path": {b"theirs": 2}}, stage_result=failed
    )
    started = time.monotonic()
    result = asyncio.run(manager.execute_stage(None, "r1", lambda: None))
    assert result.error.description == "x The machine did not confirm that it stopped."
    assert result.error.severity == mission_state_pb2.ERROR_SEVERITY_FATAL
    assert [(r.key, r.value) for r in result.error.references] == [("stop_confirmed", "false")]
    assert time.monotonic() - started < 2.0


def test_a_requested_goal_without_a_status_is_not_taken_as_stopped(monkeypatch):
    manager = _stopping_manager(monkeypatch)
    manager._requested_goals = {b"ours"}
    assert asyncio.run(manager._ensure_stopped()) is False


def test_nothing_is_sent_at_shutdown(monkeypatch):
    manager = _stopping_manager(monkeypatch, ok=False, stage_result=StageResult(status=_FAILED))
    started = time.monotonic()
    result = asyncio.run(manager.execute_stage(None, "r1", lambda: None))
    assert _requests(manager) == [0, 0, 0]
    assert result.error.type == "stop_unconfirmed"
    assert time.monotonic() - started < 0.5


def test_a_pause_whose_cancel_is_not_acknowledged_ends_the_stage(monkeypatch):
    manager = _stopping_manager(monkeypatch)

    async def run():
        loop = asyncio.get_running_loop()
        handle = SimpleNamespace(
            get_result_async=loop.create_future, cancel_goal_async=loop.create_future
        )
        manager.request_pause()
        return await manager._wait_goal(handle, lambda: None, None, lambda: 0.0, tick_s=0.01)

    outcome, error = asyncio.run(run())
    assert outcome == "abandoned"
    assert error.type == "pause_failed"


def test_paused_is_reported_only_once_the_goal_has_ended(monkeypatch):
    """An accepted cancel only means the goal is stopping, the machine may still be braking."""
    manager = _stopping_manager(monkeypatch)
    manager._requested_goals = {b"ours"}
    manager._store_goal_statuses("follow_path", _status_array(b"ours", 3))
    reports: list[tuple[float, int]] = []

    async def run():
        loop = asyncio.get_running_loop()
        started = time.monotonic()
        handle = _FakeHandle(loop.create_future())
        loop.call_later(0.3, manager._store_goal_statuses, "follow_path", _status_array(b"ours", 5))
        loop.call_later(0.6, manager.request_resume)
        manager.request_pause()
        outcome = await manager._wait_goal(
            handle,
            lambda: None,
            lambda st, pr: reports.append((time.monotonic() - started, st)),
            lambda: 0.0,
            tick_s=0.01,
        )
        return outcome

    assert asyncio.run(run()) == ("paused", None)
    paused_at = next(at for at, st in reports if st == mission_state_pb2.STAGE_STATUS_PAUSED)
    assert paused_at >= 0.25


def test_the_zero_hold_lasts_until_the_goal_ends(monkeypatch):
    publisher = _CountingPublisher()
    manager = _stopping_manager(monkeypatch, stop_pub=publisher)
    manager._send_goal_timeout_s = 2.0

    async def run():
        loop = asyncio.get_running_loop()
        result: asyncio.Future = loop.create_future()
        loop.call_later(1.0, result.set_result, None)
        handle = _FakeHandle(result)
        started = time.monotonic()
        await manager._abandon(handle, result, mission_pb2.CANCEL_MODE_IMMEDIATE)
        return started

    started = asyncio.run(run())
    # The goal ends after 1.0 s: the zero command runs until then, and stops soon after.
    assert any(t >= started + 0.9 for t in publisher.times)
    assert not any(t > started + 1.4 for t in publisher.times)


def test_a_goal_accepted_after_the_wait_is_cancelled():
    async def run():
        loop = asyncio.get_running_loop()
        accepted = _FakeHandle(loop.create_future())
        done: asyncio.Future = loop.create_future()
        done.set_result(accepted)
        nav._cancel_if_accepted_late(done)
        refused_handle = _FakeHandle(loop.create_future())
        refused_handle.accepted = False
        refused = loop.create_future()
        refused.set_result(refused_handle)
        nav._cancel_if_accepted_late(refused)
        return accepted.cancelled, refused_handle.cancelled

    assert asyncio.run(run()) == (True, False)


def test_a_send_goal_timeout_cancels_the_goal_when_it_is_accepted_late(monkeypatch):
    manager = _stopping_manager(monkeypatch)
    # The real bridge, because an rclpy future outlives the wait that timed out.
    monkeypatch.undo()

    async def run():
        loop = asyncio.get_running_loop()
        acceptance = _RclpyFuture()
        client = SimpleNamespace(
            send_goal_async=lambda goal, feedback_callback=None, goal_uuid=None: acceptance
        )
        try:
            await manager._send_goal(client, object())
        except asyncio.TimeoutError:
            pass
        handle = _FakeHandle(loop.create_future())
        acceptance.set_result(handle)
        await asyncio.sleep(0)
        return handle.cancelled, len(manager._requested_goals)

    cancelled, requested = asyncio.run(run())
    assert cancelled is True
    # Still tracked: only a status can show that the late goal has ended.
    assert requested == 1


def test_a_stop_is_confirmed_once_the_active_goal_ends(monkeypatch):
    manager = _stopping_manager(monkeypatch, statuses={"follow_path": {b"theirs": 2}})

    async def run():
        loop = asyncio.get_running_loop()
        loop.call_later(
            0.15,
            manager._store_goal_statuses,
            "follow_path",
            _status_array(b"theirs", 5),
        )
        return await manager._ensure_stopped()

    assert asyncio.run(run()) is True


def test_zero_velocity_is_held_only_while_a_goal_is_active(monkeypatch):
    idle = _stopping_manager(monkeypatch, stop_pub=_CountingPublisher())
    assert asyncio.run(idle._ensure_stopped()) is True
    assert idle._stop_pub.times == []

    busy = _stopping_manager(
        monkeypatch, stop_pub=_CountingPublisher(), statuses={"follow_path": {b"theirs": 2}}
    )
    assert asyncio.run(busy._ensure_stopped()) is False
    assert busy._stop_pub.times


def test_a_cancel_service_that_is_not_ready_is_skipped(monkeypatch):
    manager = _stopping_manager(monkeypatch)
    manager._cancel_all_clients[0].service_is_ready = lambda: False
    assert asyncio.run(manager._ensure_stopped()) is True
    assert _requests(manager) == [0, 1, 1]


def test_a_rejected_goal_does_not_wait_for_a_status(monkeypatch):
    manager = _stopping_manager(monkeypatch)

    async def run():
        loop = asyncio.get_running_loop()
        handle = _FakeHandle(loop.create_future())
        handle.accepted = False
        acceptance: asyncio.Future = loop.create_future()
        acceptance.set_result(handle)
        client = SimpleNamespace(
            send_goal_async=lambda goal, feedback_callback=None, goal_uuid=None: acceptance
        )
        await manager._send_goal(client, object())
        return manager._requested_goals, await manager._ensure_stopped()

    assert asyncio.run(run()) == (set(), True)


def test_our_goal_counts_until_a_status_shows_it_ended(monkeypatch):
    manager = _stopping_manager(monkeypatch)
    manager._requested_goals = {b"ours"}
    manager._store_goal_statuses("navigate_to_pose", _status_array(b"ours", 2))
    assert asyncio.run(manager._ensure_stopped()) is False
    manager._store_goal_statuses("navigate_to_pose", _status_array(b"ours", 6))
    assert asyncio.run(manager._ensure_stopped()) is True


def test_resume_continues_with_the_waypoints_not_yet_passed(monkeypatch):
    """Re-sending every waypoint would drive the machine back to the first one."""
    manager = _stopping_manager(monkeypatch)
    poses = [_pose(float(i), 0.0) for i in range(5)]
    sent: list[list[object]] = []
    progress: list[float] = []

    async def _build_poses(stage):
        return list(poses)

    def _send_goal_async(goal, feedback_callback=None, goal_uuid=None):
        sent.append(list(goal.poses))
        accepted: asyncio.Future = asyncio.get_running_loop().create_future()
        accepted.set_result(SimpleNamespace(accepted=True))
        manager._feedback = feedback_callback
        return accepted

    outcomes = iter([("paused", None), ("done", SimpleNamespace(status=_SUCCEEDED))])

    async def _wait_goal(handle, cancel_requested, on_progress, current, **kwargs):
        remaining = 3 if len(sent) == 1 else 1
        manager._feedback(
            SimpleNamespace(feedback=SimpleNamespace(number_of_poses_remaining=remaining))
        )
        return next(outcomes)

    manager._build_poses = _build_poses
    manager._wait_goal = _wait_goal
    manager._action_client = SimpleNamespace(
        wait_for_server=lambda timeout_sec: True, send_goal_async=_send_goal_async
    )
    manager._NavigateThroughPoses = SimpleNamespace(Goal=lambda: SimpleNamespace(poses=None))
    stage = mission_pb2.Stage(kind=mission_pb2.STAGE_KIND_NAVIGATION)

    result = asyncio.run(
        manager._execute_stage(stage, "r1", lambda: None, lambda st, pr: progress.append(pr))
    )

    assert result == FINISHED
    assert sent == [poses, poses[2:]]
    assert progress == sorted(progress)


def test_a_pause_whose_stop_is_not_confirmed_is_never_reported_as_paused(monkeypatch):
    manager = _stopping_manager(monkeypatch, statuses={"follow_path": {b"theirs": 2}})
    reports: list[int] = []

    async def run():
        loop = asyncio.get_running_loop()
        handle = SimpleNamespace(
            get_result_async=loop.create_future, cancel_goal_async=loop.create_future
        )
        manager.request_pause()
        return await manager._wait_goal(
            handle, lambda: None, lambda st, pr: reports.append(st), lambda: 0.0, tick_s=0.01
        )

    outcome, error = asyncio.run(run())
    assert outcome == "abandoned"
    assert error.type == "pause_failed"
    assert mission_state_pb2.STAGE_STATUS_PAUSED not in reports
    assert manager.is_paused() is False


def test_a_stage_that_raises_still_leaves_the_machine_standing(monkeypatch):
    manager = _stopping_manager(monkeypatch)

    async def _execute_stage(stage, robot_id, cancel_requested, on_progress=None):
        raise RuntimeError("boom")

    manager._execute_stage = _execute_stage
    with pytest.raises(RuntimeError, match="boom"):
        asyncio.run(manager.execute_stage(None, "r1", lambda: None))
    assert _requests(manager) == [1, 1, 1]


def test_a_refused_cancel_is_not_taken_as_acknowledged(monkeypatch):
    manager = _stopping_manager(monkeypatch)

    async def run():
        loop = asyncio.get_running_loop()
        refused: asyncio.Future = loop.create_future()
        refused.set_result(SimpleNamespace(return_code=1))
        handle = SimpleNamespace(cancel_goal_async=lambda: refused)
        return await manager._abandon(handle, asyncio.ensure_future(loop.create_future()))

    assert asyncio.run(run()) is False


def test_a_status_of_unknown_does_not_end_our_goal(monkeypatch):
    manager = _stopping_manager(monkeypatch)
    manager._requested_goals = {b"ours"}
    manager._store_goal_statuses("follow_path", _status_array(b"ours", 0))
    assert manager._requested_goals == {b"ours"}


def test_a_cancel_during_the_approach_fails_the_leg_without_an_error(monkeypatch):
    approach = _Approach(monkeypatch, positions=[(40.0, 0.0)])
    result, aborted = asyncio.run(
        approach.manager._drive_to(_pose(0.0, 0.0), lambda: mission_pb2.CANCEL_MODE_GRACEFUL)
    )
    assert result.status == mission_state_pb2.STAGE_STATUS_FAILED
    assert result.error is None
    assert aborted is False
    assert approach.handle.cancelled
