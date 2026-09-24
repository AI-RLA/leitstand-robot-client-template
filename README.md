# leitstand-robot-client-template

The robot-side client for the Leitstand, and the reference for adding a new robot.

It registers the robot, serves its capability declaration, receives missions, runs them
stage by stage, reports execution state, and publishes GNSS pose. That part is complete and
robot-independent. What varies per robot is how a stage is driven and where the position is
read; those are the two interfaces to implement.

```
┌──────────────────────────────────────────────────────────┐
│ leitstand_client_ros2              ROS 2 package         │
│   node.py            single rclpy node; composition root │
│   nav2_navigation.py drives stages via Nav2              │  ← replace for a robot
│   pose_relay.py      GNSS topic -> pose, via gnss.py     │    that does not run Nav2
└──────────────┬───────────────────────────────────────────┘
               │ implements Navigation + PoseSource
┌──────────────┴───────────────────────────────────────────┐
│ leitstand_client                  Python library, no ROS │
│   registration · factsheet · mission_executor · keys     │  ← robot-independent,
│   Navigation and PoseSource interfaces · FakeNavigation  │    no changes required
└──────────────┬───────────────────────────────────────────┘
               │ Zenoh + protobuf (leitstand-robot-contract)
            Leitstand
```

## Integration paths

`docs/integration-guide.md` walks these in order, with the code for each.

- **Your robot runs a standard Nav2 stack** (`navigate_through_poses`, `navigate_to_pose`,
  `follow_path`, `robot_localization`'s `/fromLL`): install both packages, write one
  `robot.yaml`, launch. No code.
- **ROS 2, but your own navigation**: copy `leitstand_client_ros2/` to
  `leitstand_client_<yourrobot>/`, replace `nav2_navigation.py` with a class implementing
  `Navigation` (six methods, below), construct it in `_navigation()` in `node.py` and name it
  in `navigation:` of your `robot.yaml` (any name but `nav2` and `fake` is yours).
  `pose_relay.py` is unchanged.
- **Anything else with Python**: `pip install ./leitstand_client`, implement `Navigation`
  and `PoseSource`, write a `main.py` of about forty lines. Worked example in `examples/python_only/`.
- **No Python on the robot**: write your own client against `docs/contract.md`, which
  specifies all ten Zenoh keys and their payloads. That is the whole protocol, and this
  repository then serves only as a worked reference.

The first two paths run the client as its own ROS 2 node, which reaches the robot through the
Nav2 actions and a GNSS topic. The language of everything around it does not matter.

## Prerequisites

- ROS 2 with Nav2 for the ROS package (the library requires no ROS). Target whichever your robot
  already runs.
- A Nav2 stack exposing the standard actions, for the Nav2 path.
- Git access to `leitstand-robot-contract`. The client speaks contract version **0.4.0**, which
  Leitstand backend 0.6.0 and later serve.
- A reachable Leitstand Zenoh router.

The Leitstand is a research prototype. The connection between the backend and the robots is
neither authenticated nor encrypted, and the operator interface has no user accounts. Run all
components on a private network or behind a VPN.

## Installation on the host

Both this repo and the contract go into a colcon workspace, side by side. With the robot's ROS
sourced and `python3-venv` installed (apt):

```
mkdir -p ws/src && cd ws
git clone <this repo> src/leitstand-robot-client-template
vcs import src < src/leitstand-robot-client-template/leitstand.repos   # fetches the contract at its tag
rosdep install --from-paths src --ignore-src -y
python3 -m venv --system-site-packages venv && touch venv/COLCON_IGNORE
venv/bin/pip install --upgrade pip   # 22.04's pip cannot build the contract's package metadata
venv/bin/pip install ./src/leitstand-robot-contract -r src/leitstand-robot-client-template/leitstand_client/requirements.txt
venv/bin/python -m colcon build --symlink-install
source install/setup.bash
```

The same block works on Ubuntu 22.04, 24.04 and 26.04. The virtual environment exists because pip
may not replace the Python packages Ubuntu installed, and `--system-site-packages` keeps the ROS
packages visible.
`python -m colcon build` runs colcon under the environment's interpreter so the installed `client`
entry point uses it. A plain `colcon build` binds it to `/usr/bin/python3`, which cannot import the
packages above (`ModuleNotFoundError: zenoh` at start). `vcs` comes with `python3-vcstool`. Without
it, `pip install "git+https://github.com/AI-RLA/leitstand-robot-contract.git@v0.4.0"` installs the
contract directly.

Upgrading from v0.1.0, which installed with `pip install --user`: remove those packages first
(`python3 -m pip uninstall leitstand-robot-contract eclipse-zenoh protobuf protovalidate pydantic`).
The environment would otherwise use the copies in `~/.local` and break when they are removed.

## Installation with Docker

Put your `robot.yaml` in a directory of its own, for example `/etc/leitstand`, and mount that
directory at `/config`. From the workspace `src/` directory that holds both checkouts:

```
docker build --build-arg ROS_DISTRO=jazzy -f leitstand-robot-client-template/docker/Dockerfile -t leitstand-client:jazzy .
docker run --rm --network host -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -v /etc/leitstand:/config leitstand-client:jazzy
```

The image runs `/config/robot.yaml`. For another file name, append the launch command with your
own, as the compose file does:
`ros2 launch leitstand_client_ros2 client.launch.py config:=/config/<file>`.

As a service that survives reboots, the same in one file: copy `docker/.env.example` to
`docker/.env`, set the ROS distribution, the DDS vendor, the directory that holds your
configuration and the file in it to load, then

```
docker compose -f leitstand-robot-client-template/docker/docker-compose.yaml up -d --build
```

The compose file carries the flags below (`network_mode: host`, `ipc: host`, the RMW choice,
the config mount) and `restart: unless-stopped`. `docker/.env` names the robot's ROS distribution
(`ROBOT_ROS_DISTRO`, a name of its own so a ROS sourced in the shell cannot override it), its DDS
vendor (`RMW_IMPLEMENTATION`, taken from the shell when the robot exports it) and the configuration
directory with the file in it to run. It names only what has to be known before that file can be
read, so everything else, a Cyclone config included, is set inside the file. The build context
is the parent of the two checkouts, and `docker/Dockerfile.dockerignore` limits what is sent to the
two of them.

`--network host` is required so DDS inside the container discovers your Nav2, and the container
must use the robot's DDS vendor: the image holds Fast DDS and Cyclone, so pass
`-e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` when the robot runs Cyclone. A robot that needs its own
Cyclone config puts it beside `robot.yaml` as `cyclone.xml` and names it there
(`ros.cyclonedds_uri: file:///config/cyclone.xml`). Without one, Cyclone runs on its defaults.
With Fast DDS, the image default, add `--ipc host` as well: on one host Fast DDS moves data over
shared memory, and without the host's `/dev/shm` discovery succeeds while no message ever
arrives. The image's ROS distribution is a build argument (`--build-arg ROS_DISTRO=jazzy`) and
must match the robot's. Any distribution with a `ros:<distro>` image and Python 3.10 or newer
builds the same way. Inside it the Python dependencies live in a virtual environment as well
(`/opt/venv`), for the reason given above.

## Configuration

Copy `leitstand_client_ros2/config/robot.yaml` and edit it. The file is annotated; the blocks are:

| Block | What it is |
|---|---|
| `id` | the robot's name on the Leitstand: lowercase, digits, `_`, `-` |
| `leitstand.endpoint` | the router, e.g. `tcp/192.168.126.10:7447` |
| `ros` | domain, node name, `use_sim_time`, an optional Cyclone config path (overrides the environment only when set) |
| `telemetry.pose` | which GNSS topic and message type to relay; omit for no pose |
| `factsheet` | what the robot can do. **Required to receive missions.** |
| `navigation` | `nav2`, or `fake` to verify the connection before Nav2 is running |
| `nav2` | action names, frames, controller id, tuning. **Omit if your Nav2 is standard.** |

Configuration is not read from the environment, apart from `LEITSTAND_LOG_LEVEL` and what ROS
itself reads (`RMW_IMPLEMENTATION`, and `CYCLONEDDS_URI` unless `ros.cyclonedds_uri` names one).
A misspelled key fails at startup and is named in the error.

## Running the client

```
ros2 launch leitstand_client_ros2 client.launch.py config:=/path/to/robot.yaml
```

The robot appears in the fleet view as online within a few seconds. If it does not, see
Troubleshooting.

## Testing without hardware

`tools/mock_fleet.py --fleet 3` registers three robots on the Leitstand from a single
process. Each accepts navigation missions, executes them in simulated time, and publishes a
pose and a battery level. Requires the contract and then the library on the Python path
(`pip install ../leitstand-robot-contract ./leitstand_client`, the contract first because the
library pins it and it is not on PyPI).

## Implementing the Navigation interface

Implement these six methods. Each docstring states which thread calls it.

```python
class Navigation(Protocol):
    async def execute_stage(self, stage, robot_id, cancel_requested, on_progress=None) -> StageResult
    def check_ready(self, mission, timeout_s) -> str | None
    def request_pause(self) -> None
    def request_resume(self) -> None
    def is_paused(self) -> bool
    def close(self) -> None
```

`FakeNavigation` in the library is the minimal implementation, about 100 lines.
`Nav2Navigation` in the ROS package is the complete reference: readiness probing, coordinate
projection, a pause that stops the machine, resume (a coverage stage re-enters the path a few
metres behind the machine, a navigation stage continues from the waypoints Nav2 reports as still
ahead), and structured error reporting.

`cancel_requested()` returns `None` while the run continues, otherwise the cancel mode. Return
`StageResult(FAILED)` once the machine has stopped. Without a cancel, any status other than
FINISHED or FAILED ends the run as FAILED.
`is_paused()` must be true only while the machine stands still. The change is reported through
`on_progress` (`PAUSED`, then `RUNNING`), which is what the Leitstand shows the operator.

## Cancellation

- **GRACEFUL** cancels the Nav2 goal; the controller publishes zero velocity and the velocity
  smoother ramps down at its configured deceleration.
- **IMMEDIATE** does the same, and additionally holds zero velocity on `nav2.stop_topic` until the
  goal has ended (at most twice `nav2.send_goal_timeout_s`), if you configure one. That must be a
  `twist_mux` input with top priority: a plain `cmd_vel` publish would be overwritten by the next
  controller command. Without a mux the two modes are identical. A guaranteed hard stop requires
  the emergency stop.

After either, the stage's `on_cancel` cleanup runs and the run reports CANCELLED. A stage that was
paused when the cancel arrived ends CANCELLED without its cleanup, so a machine someone paused, for
example because a person stands next to it, does not drive off. A stage that fails with its own
error during a cancel ends the run FAILED with that error, also without cleanup.

When a stage ends without finishing, the client cancels every goal on the three Nav2 action
servers, including goals other programs sent, and waits until they have ended. If a goal is still
active a second later, it holds zero velocity on `nav2.stop_topic`, if configured, which overrides
manual driving on that mux input. If it cannot confirm the stop, the stage's error says "The
machine did not confirm that it stopped."

## Troubleshooting

| Symptom | Cause |
|---|---|
| Robot is online but every dispatch is refused | no `factsheet:` block. A robot without one can never be given a mission. |
| Robot never appears | the metadata queryable was declared after the liveliness token, or the router is unreachable. The backend queries metadata the moment liveliness appears, waits three seconds, and never retries. |
| Timestamps in 1970 | the robot's clock is not set. The backend trusts robot clocks. |
| `failed to establish Zenoh connection after retries` | the router at `leitstand.endpoint` is unreachable. The client tries five times over about fifteen seconds, then exits 1. |
| Start fails naming a key | a misspelled key in `robot.yaml`. Fix the spelling; nothing is ignored silently. |
| A Cyclone setting has no effect and nothing is logged | `ros.cyclonedds_uri` points at a file the client cannot read. Cyclone ignores a missing or unreadable config without a word. In Docker the path is the container's: `file:///config/cyclone.xml` for a `cyclone.xml` beside your `robot.yaml`. |
| `ModuleNotFoundError: rclpy` when the node starts | ROS was not sourced before `ros2 launch`. |
| `ModuleNotFoundError: numpy` on the first message import | the virtual environment was created without `--system-site-packages`. |
| `ModuleNotFoundError: zenoh` when the node starts | the workspace was built with a plain `colcon build` instead of `venv/bin/python -m colcon build`. |
| A run fails a second or two after the robot starts driving | Nav2's behavior tree gave up waiting for the controller to acknowledge its `follow_path` goal (`bt_navigator` logs "Timed out while waiting for action server to acknowledge goal request") and aborted our goal, while the controller kept the goal it had accepted. Raise `bt_navigator: default_server_timeout` (milliseconds) well above the acknowledgement latency on that machine. |
| A coverage stage never moves: `controller_server` logs "Failed to make progress" every 10 s | the controller sees too little path: `nav2.max_pose_spacing_m` must stay well inside the local costmap's half-size (Nav2 prunes the path beyond it) and inside the controller's lookahead. Enlarge the local costmap or lower the spacing. |
| The robot spins and backs up right after a dispatch, then the stage fails | Nav2 could not plan and ran its recoveries, and `planner_server` says why. Most often "Global costmap is not current": a static layer without a map server, or an observation source that publishes nothing. A goal from RViz fails the same way. |

## Known limitations

- Halt/release, a robot-level interlock separate from cancel, is not in the contract yet and
  therefore not implemented here.
- A run does not survive a client restart: the executor holds it in memory only, so the
  backend closes it when the robot comes back without it. Resuming a run across a restart
  would need the run and its `header_id` counter persisted; not done.

## Development

In the workspace's virtual environment (`source venv/bin/activate` in `ws/`):

```
pip install -r requirements-dev.txt   # ruff and pytest, the versions CI runs
make contract-check   # leitstand.repos, setup.py and the installed contract agree on the version
make test             # both packages; needs no ROS on the path
make lint
```

## Contact

Jannik Jose, jannik.jose@hs-osnabrueck.de

## License

Copyright 2026 Osnabrück University of Applied Sciences.
Apache License 2.0, see [LICENSE](LICENSE).
