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

- ROS 2 (humble, iron or jazzy) with Nav2 for the ROS package; the library requires no ROS.
  Target whichever your robot already runs.
- A Nav2 stack exposing the standard actions, for the Nav2 path.
- Git access to `leitstand-robot-contract`. The client speaks contract version **0.4.0**, which
  Leitstand backend 0.6.0 and later serve.
- A reachable Leitstand Zenoh router. The link carries no authentication: anyone who can
  reach the router can act as a robot. Operate it only on a trusted network.

## Installation on the host

Both this repo and the contract go into a colcon workspace, side by side:

```
mkdir -p ws/src && cd ws
git clone <this repo> src/leitstand-robot-client-template
vcs import src < src/leitstand-robot-client-template/leitstand.repos   # fetches the contract at its tag
pip install --user ./src/leitstand-robot-contract
pip install --user -r src/leitstand-robot-client-template/leitstand_client/requirements.txt
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

`vcs` is provided by `python3-vcstool`. Use `--user` rather than a virtualenv: `rclpy` is
installed only with ROS, and a virtualenv cannot see it unless ROS is sourced first. Without
`vcs`, `pip install "git+https://github.com/AI-RLA/leitstand-robot-contract.git@v0.4.0"` installs the
contract directly.

## Installation with Docker

From the workspace `src/` directory that holds both checkouts:

```
docker build -f leitstand-robot-client-template/docker/Dockerfile -t leitstand-client .
docker run --rm --network host -v $PWD/robot.yaml:/config/robot.yaml leitstand-client
```

As a service that survives reboots, the same in one file: copy `docker/.env.example` to
`docker/.env`, set the ROS distribution, the DDS vendor and the path to your `robot.yaml`, then

```
docker compose -f leitstand-robot-client-template/docker/docker-compose.yaml up -d --build
```

The compose file carries the flags below (`network_mode: host`, `ipc: host`, the RMW choice,
the config mounts) and `restart: unless-stopped`. `ROS_DISTRO` left unset takes the sourced
shell's distribution, else humble. The build context is the parent of the two checkouts;
`docker/Dockerfile.dockerignore` limits what is sent to the two of them.

`--network host` is required so DDS inside the container discovers your Nav2, and the container
must use the robot's DDS vendor: the image holds Fast DDS and Cyclone, so pass
`-e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` (with `-e CYCLONEDDS_URI=file:///config/cyclone.xml`
and the config mounted there) when the robot runs Cyclone. With Fast DDS, the image default, add
`--ipc host` as well: on one host Fast DDS moves data over shared memory, and without the host's
`/dev/shm` discovery succeeds while no message ever arrives. The image's
ROS distribution is a build argument (`--build-arg ROS_DISTRO=iron`) and must match the
robot's; humble, iron and jazzy are built and checked from it. Python dependencies go into
a virtual environment inside the image (`/opt/venv`, sharing the ROS packages), so the base
image's own Python is not touched.

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
itself reads (`RMW_IMPLEMENTATION`, and `CYCLONEDDS_URI` unless `ros.cyclonedds_uri` is set).
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
metres behind the machine; a navigation stage re-issues its original waypoint list), and
structured error reporting.

`cancel_requested()` returns `None` while the run continues, otherwise the cancel mode. Return
`StageResult(FAILED)` once the machine has stopped. `is_paused()` must be true only while the
machine stands still; the change is reported through `on_progress` (`PAUSED`, then `RUNNING`),
which is what the Leitstand shows the operator.

## Cancellation

- **GRACEFUL** cancels the Nav2 goal; the controller publishes zero velocity and the velocity
  smoother ramps down at its configured deceleration.
- **IMMEDIATE** does the same, and additionally holds zero velocity on `nav2.stop_topic`
  if you configure one. That must be a `twist_mux` input with top priority: a plain
  `cmd_vel` publish would be overwritten by the next controller command. Without a mux the
  two modes are identical; a guaranteed hard stop requires the emergency stop.

After either, the stage's `on_cancel` cleanup runs and the run reports CANCELLED.

## Troubleshooting

| Symptom | Cause |
|---|---|
| Robot is online but every dispatch is refused | no `factsheet:` block. A robot without one can never be given a mission. |
| Robot never appears | the metadata queryable was declared after the liveliness token, or the router is unreachable. The backend queries metadata the moment liveliness appears, waits three seconds, and never retries. |
| Timestamps in 1970 | the robot's clock is not set. The backend trusts robot clocks. |
| `failed to establish Zenoh connection after retries` | the router at `leitstand.endpoint` is unreachable. The client tries five times over about fifteen seconds, then exits 1. |
| Start fails naming a key | a misspelled key in `robot.yaml`. Fix the spelling; nothing is ignored silently. |
| `ModuleNotFoundError: rclpy` in a virtualenv | source ROS before Python, or install with `--user` instead of a virtualenv. |

## Known limitations

- Halt/release, a robot-level interlock separate from cancel, is not in the contract yet and
  therefore not implemented here.
- A run does not survive a client restart: the executor holds it in memory only, so the
  backend closes it when the robot comes back without it. Resuming a run across a restart
  would need the run and its `header_id` counter persisted; not done.

## Development

```
make contract-check   # the installed contract is the version this client speaks
make test             # both packages; needs no ROS on the path
make lint
```

## Contact

Jannik Jose, jannik.jose@hs-osnabrueck.de

## License

Copyright 2026 Osnabrück University of Applied Sciences.
Apache License 2.0, see [LICENSE](LICENSE).
