"""Planned-path execution in the Nav2 frame manager, driven against recorders.

The module imports without ROS; only construction needs it, so the manager is built bare and
its Nav2 legs are replaced with fakes that record what was asked of them.
"""

from __future__ import annotations

import asyncio
import threading
from types import SimpleNamespace

from leitstand.robot.v1 import mission_state_pb2

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
        done.set_result(None)
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
        manager._NavigateToPose = SimpleNamespace(Goal=lambda: SimpleNamespace(pose=None))

        def _robot_xy():
            position = self.positions[0]
            if len(self.positions) > 1:
                self.positions.pop(0)
            return position

        def _send_goal_async(goal, feedback_callback=None):
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
        done.set_result(None)
        return done


def _bare_manager() -> nav.Nav2Navigation:
    manager = object.__new__(nav.Nav2Navigation)
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

    assert asyncio.run(run()) == ("cancelled", False)
