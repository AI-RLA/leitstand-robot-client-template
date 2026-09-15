# What the Leitstand requires of a robot

The wire contract is `leitstand-robot-contract`, five Protobuf files under
`leitstand.robot.v1`, carried as proto-canonical JSON over Zenoh. This page is what the
backend actually enforces, checked against its source. Version **0.4.0**; the receipts, the
3 s pause/resume wait and the metadata-driven close arrive with backend 0.6.0.

| File | Carries |
|---|---|
| `mission.proto` | Mission, stages, waypoints, dispatch, cancel |
| `robot_control.proto` | pause and resume requests, and the receipt every command is answered with |
| `mission_state.proto` | what the robot reports back |
| `telemetry.proto` | pose and battery |
| `factsheet.proto` | what the robot can do |

## How a robot joins

Three declarations on the Zenoh session, in this order:

1. A queryable on `leitstand/robot/<id>/metadata` replying
   `{"id": "<id>", "active_run_id": "<uuid>" | null}` as plain JSON, built at query time: the
   backend asks on every reconnect and closes any run the robot does not name.
2. A queryable on `leitstand/robot/<id>/factsheet` replying a proto `Factsheet`.
3. A liveliness token on `leitstand/robot/<id>/online`. **Last.** It is the readiness signal.

On shutdown, drop the liveliness token **first**, then the rest.

Why the order matters: when the backend sees the token appear it queries metadata with a
**3 s** timeout and **never retries**. A robot that declares the token before the queryable
is invisible until it re-declares. The factsheet is queried once per online transition, also
3 s, also never re-fetched.

The id must match `^[a-z0-9][a-z0-9_-]*$`. Anything else is dropped before the payload is read.
Nothing detects two robots claiming the same id.

## The factsheet

Strict parse: one unknown field discards the whole thing. **A robot with no factsheet is online
and can never be given a mission**; every dispatch to it fails with `RobotFactsheetMissing`.
That is the most common integration mistake.

- `stage_capabilities[]`: one entry per stage kind the robot executes, `navigation` and/or
  `coverage`, each listing `supported_waypoint_kinds` (`WGS84`, `SITE_LOCAL`). A repeated
  kind is rejected.
- Declaring `coverage` is **a claim about steering**: that the machine holds a swath as a
  line. A robot that can only drive point to point must leave it out; the backend then sends
  it navigation stages instead of a plan it would drive wrong.
- `physical_parameters`: `track_width_m` (> 0, wheel centre to centre) and
  `min_turning_radius_m` (>= 0; zero turns on the spot). Optional as a whole; if present,
  both fields.

## Pose

`leitstand/robot/<id>/pose`, a proto `Pose` as JSON, encoding `application/json`.

- `timestamp` is required. The robot's own clock, and the backend trusts it.
- `lat`, `lon` in WGS84. There is no frame field; WGS84 is the only frame in v1.
- `heading_deg`, optional: compass bearing, **clockwise from true north**, in [0, 360).
  Declination correction is the robot's job.
- `horizontal_accuracy_m`, optional, one sigma.
- `NaN` and `Infinity` drop the frame. So does any unknown field.

There is no staleness detection. A robot that goes silent looks healthy and frozen. On
degraded GPS, keep publishing the last good fix.

Suggested rates: 1 Hz idle, 5 Hz while driving.

## Receiving a mission

`leitstand/robot/<id>/mission/_action/send_goal` is a queryable. The backend sends a
`MissionDispatchRequest{dispatch_id, mission}` and waits **10 s** for a
`MissionDispatchResponse{accepted, reason}`. An explicit `accepted: false` is recorded REJECTED;
no reply, or an unparseable one, is recorded FAILED.

A `Mission` is a `run_id` and ordered `stages[]`. Each `Stage` has a `stage_id`, a `kind`,
the matching payload, and `on_cancel[]` cleanup stages that are themselves not cancellable.

```
Stage
  stage_id
  kind = NAVIGATION   navigation { waypoints[] }
  kind = COVERAGE     coverage   { segments[] }
                                     Segment { kind = SWATH | TURN, geometry[] }
  on_cancel[]
```

A `Waypoint` is one of `wgs84 { lat, lon, heading_deg? }` or
`site_local { site_id, x, y, theta? }`. Note the two heading conventions: `heading_deg` is
degrees clockwise from north; `theta` is **radians counterclockwise from the site's +x**.

A coverage stage is a route of segments. A `SWATH` segment works the ground under it and
must be driven as the line given, endpoints unmoved; a `TURN` only crosses ground.
Consecutive segments share their joint waypoint.

Obligations on receipt:

- Reject the whole mission on an unknown stage kind. The enum is open; values are added
  over time, and a robot must refuse what it does not recognise rather than guess.
- Reject a `kind` that does not match the payload set, and empty waypoint lists.
- Validate bounds (`buf.validate`): lat/lon ranges, UUIDs, `min_items`. JSON parsing does
  not do this for you.
- **Be idempotent on `run_id`.** A re-delivered dispatch of the run already executing is
  accepted without a second execution. A different run while one is executing is rejected.
- Reply within 10 s. Bound any readiness probe well under that, so a dead navigation stack
  yields a clean rejection rather than a timeout.

## Cancel, pause, resume

All three are queryables answered with a `ControlResponse{applied, reason, refusal}`. The
reply is a receipt: send it **before** acting, then act, then publish a state frame as soon as
the command has taken effect. The backend believes the frame, not the reply; a reply that is
`applied: false` with `refusal: NOT_EXECUTING_RUN` tells it the robot does not hold that run.

- `…/mission/_action/cancel_goal`: `CancelRequest{run_id, mode}`, 5 s. Cancel **only** the
  named run. `mode` is `GRACEFUL` (stop at the next safe point) or `IMMEDIATE` (stop now);
  `UNSPECIFIED` counts as GRACEFUL. After either, run `on_cancel` while still publishing
  frames, then the terminal frame with `exec_status: CANCELLED`.
- `…/mission/_action/pause` and `…/_action/resume`: `ControlRequest{run_id}`, 3 s. Report
  `PAUSED` only once the machine stands still, `RUNNING` once it moves again.

## Reporting state

`leitstand/robot/<id>/mission/state`, a proto `MissionState` published on change and at
least every 5 s while a run executes.

```
MissionState
  run_id
  header_id            monotone per robot, not per run; the backend orders on it, never on time
  timestamp            required
  exec_status          RUNNING | PAUSED | SUCCEEDED | FAILED | CANCELLED   (UNSPECIFIED is rejected)
  current_stage_index  top-level cursor; cleanup stages do not appear here
  stage_states[]       { stage_id, status, progress 0..1, started_at?, ended_at?, result{} }
  errors[]             { severity, type, description, references[] }
```

Rejected outright: a missing timestamp; `exec_status` UNSPECIFIED; a duplicate `stage_id`
within one frame; `progress` outside [0, 1] or NaN.

`header_id` orders the frames of one run: the backend drops a stage update whose counter is not
above the last one it stored for that run, so the counter must only increase while a run is
being reported. A client that restarts has no run to report and may start again at zero.

Stage `status` is one of WAITING, INITIALIZING, RUNNING, PAUSED, FINISHED, FAILED, CANCELLED,
SKIPPED. The stage a cancel interrupted is CANCELLED; stages never started are SKIPPED in the
terminal frame, which carries every stage's final status.

There is no latching on the state channel. A terminal frame published while the backend is
down is lost; the 5 s heartbeat is the only cover, and only while the run is executing. A
robot may go straight to a terminal frame without a RUNNING one and the run still resolves.

## The keys, all under `leitstand/robot/<id>/`

| Key | Kind | Payload |
|---|---|---|
| `online` | liveliness token | none |
| `metadata` | queryable | `{"id": …, "active_run_id": …}` JSON |
| `factsheet` | queryable | proto `Factsheet` |
| `pose` | publisher | proto `Pose` |
| `battery` | publisher | proto `Battery` (optional; the mock fleet publishes it, the ROS client does not yet) |
| `mission/_action/send_goal` | queryable | `MissionDispatchRequest` → `MissionDispatchResponse` |
| `mission/_action/cancel_goal` | queryable | `CancelRequest` → `ControlResponse` |
| `mission/_action/pause`, `mission/_action/resume` | queryable | `ControlRequest` → `ControlResponse` |
| `mission/state` | publisher | proto `MissionState` |

## Not in v1

Halt/release (a robot-level interlock), any authentication on the link, staleness detection,
frames other than WGS84.
