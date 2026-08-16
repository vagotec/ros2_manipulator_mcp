# Phase 11 - OpenMANIPULATOR-X Full MCP Simulation E2E

## Outcome and scope

Phase 11 proves the complete public v0.1.0 path through a real stdio MCP
session and the official OpenMANIPULATOR-X mock-hardware MoveIt runtime:

```text
MCP ClientSession
 -> stdio
 -> ros2-manipulator-mcp console entry point
 -> MCP Tool / Resource
 -> ManipulatorService
 -> SafetyEvaluator where applicable
 -> JazzyManipulatorAdapter
 -> MoveIt 2.12.4 services
 -> OpenMANIPULATOR-X 4.1.3 mock hardware
```

No fake server or injected service was used by the Phase 11 live tests. The
accepted domain, application, registry, safety, adapter, profile, 19 Tools,
7 static Resources, 3 Resource Templates, 6 Prompts, and MCP 2026 discovery
behavior were preserved.

## Exact mock runtime

The official binary package launch files were run without RViz on isolated
ROS domain `66`:

```bash
export ROS_DOMAIN_ID=66
export ROS_LOG_DIR=/tmp/ros2_manipulator_mcp_phase11_logs

ros2 launch open_manipulator_bringup open_manipulator_x.launch.py \
  use_mock_hardware:=true init_position:=false start_rviz:=false

ros2 launch open_manipulator_moveit_config open_manipulator_x_moveit.launch.py \
  start_rviz:=false \
  warehouse_sqlite_path:=/tmp/ros2_manipulator_mcp_phase11_runtime/warehouse.sqlite
```

The bringup selected `mock_components/GenericSystem`. Disabling
`init_position` prevented the vendor initialization-motion process from
starting. The vendor launch itself starts controllers as infrastructure; the
project did not call, switch, or expose them.

Before testing, the live graph contained `move_group`, `/joint_states`, `/tf`,
`/tf_static`, `/planning_scene`, and `/monitored_planning_scene`. The following
adapter endpoints were present with their exact installed types:

| Endpoint | Interface |
|---|---|
| `/compute_ik` | `moveit_msgs/srv/GetPositionIK` |
| `/compute_fk` | `moveit_msgs/srv/GetPositionFK` |
| `/check_state_validity` | `moveit_msgs/srv/GetStateValidity` |
| `/compute_cartesian_path` | `moveit_msgs/srv/GetCartesianPath` |
| `/plan_kinematic_path` | `moveit_msgs/srv/GetMotionPlan` |
| `/get_planning_scene` | `moveit_msgs/srv/GetPlanningScene` |
| `/apply_planning_scene` | `moveit_msgs/srv/ApplyPlanningScene` |

The graph also advertised `/execute_trajectory`, `/move_action`, and
`/arm_controller/follow_joint_trajectory`. They were inspected only and were
never called by the project or tests.

## Live MCP workflows

### Discovery, reads, and kinematics

One real `ClientSession.discover()` workflow negotiated MCP `2026-07-28`,
confirmed the `19/7/3/6` surface, and exercised these Tools:

- `describe_manipulator`
- `list_planning_groups`
- `get_current_robot_state`
- `get_end_effector_pose`
- `compute_forward_kinematics`
- `compute_inverse_kinematics`
- `check_robot_state_validity`

It confirmed model `open_manipulator_x`, planning group `arm`, active joints
`joint1` through `joint4`, frame `world`, and a valid IK result. It also read
live `manipulator://overview`, `manipulator://state/current`,
`manipulator://scene`, and `manipulator://health`; health reported ready.

### Planning and stored-plan lifecycle

The joint target was the conservative Phase 6 target `(0.1, -0.4, 0.4,
0.1)` for `joint1` through `joint4`, using the current state as the explicit
start and velocity/acceleration scaling `0.2`. The MCP response contained a
fresh opaque application-owned `plan_id` and a bounded trajectory summary
with the four arm joints, a positive point count, and positive duration.

The same MCP session retrieved the plan with `get_motion_plan`, validated it
with `validate_motion_plan`, read `manipulator://plans/{plan_id}`, discarded
it, and confirmed later retrieval returned stable `not_found`. Validation was
true under the policy and stored planning-scene revision. A second
deterministic pose-goal plan moved the live FK pose by `-0.01 m` in `world` X;
it produced a nonempty trajectory and was also discarded. No plan was
executed.

### Planning scene and rejection

The deterministic object `phase11-mcp-box` was a `0.02 m` cube at
`(0.5, 0.0, 0.5)` in `world`. The workflow read the initial scene, removed any
stale object from an interrupted prior run, added the cube through
`apply_collision_object`, verified it through `get_collision_object` and
`manipulator://scene`, removed it, and verified final absence. A `finally`
block performs cleanup even after an intermediate assertion failure.

The same session submitted `phase11-policy-rejected-box` with a `5.01 m`
dimension against the configured `5.0 m` maximum. MCP returned
`isError=true` and stable `policy_rejected`; a lookup returned `not_found`.
This proves the SafetyEvaluator rejected the request before backend mutation.

## Interoperability correction

The first live test launch found one test-harness issue: SDK 2.0.0's stdio
client deliberately inherits only an allow-list of environment variables, so
the sourced Jazzy `PYTHONPATH` was not passed to the console-entry-point child.
The child consequently could not import installed `moveit_msgs`. The test
parameters now explicitly compose the project `src` directory with the
verified sourced `PYTHONPATH` (`/opt/ros/jazzy/lib/python3.12/site-packages`).
No production, application, safety, adapter, or MCP implementation changed.

## Tests and verification

`tests/integration/test_open_manipulator_x_mcp_live.py` contains three opt-in
tests because each covers one coherent high-value workflow:

1. real discovery, surface listing, live Resources, robot state, FK, IK, and
   validity;
2. real joint and pose planning plus plan retrieval, validation, resource
   projection, discard, and bounded trajectory summaries;
3. real policy-controlled scene round-trip, cleanup, and one rejected mutation.

Set `ROS2_MANIPULATOR_MCP_RUN_LIVE_TESTS=1` with the documented graph to run
them. Results:

- Python compilation: passed for `src` and `tests`;
- Phase 11 live MCP suite: `3 passed in 4.04s`;
- full Phase 1-11 suite with all live tests: `46 passed in 7.52s`;
- normal graph-independent regression: `40 passed, 6 skipped in 2.81s`;
- architecture/import-boundary scans: passed;
- execution/controller API absence scan: passed.

The final scene response contained no `phase11-mcp-box`. Both official launch
sessions were stopped. Mock hardware deactivated and shut down. As observed
in Phase 6, ros2_control emitted PAL statistics context messages during
shutdown. This MoveIt instance also required the launch system's bounded
SIGTERM escalation after it did not exit within five seconds of SIGINT; this
occurred after all tests and did not leave a running process.

## Authoritative source audit

Sources consulted and exact versions verified:

- installed Ubuntu packages: ROS 2 Jazzy, `rclpy` 7.1.11,
  MoveIt 2.12.4, and `moveit_msgs` 2.6.0;
- installed official ROBOTIS packages
  `open_manipulator_moveit_config`, `open_manipulator_bringup`, and
  `open_manipulator_description` 4.1.3;
- installed 4.1.3 launch files, nested OpenMANIPULATOR-X SRDF,
  `kinematics.yaml`, URDF/Xacro, controller configuration, and joint limits;
- the official ROBOTIS OpenMANIPULATOR 4.1.3 source/configuration already
  matched to installed files in Phase 6;
- the live ROS graph and exact service, topic, and action type discovery;
- installed MCP Python SDK 2.0.0 `stdio_client`,
  `StdioServerParameters`, `ClientSession.discover`, listing, tool-call,
  resource-read, and subprocess-shutdown implementations;
- installed `mcp-types` 2.0.0 modern `2026-07-28` discovery models and the
  MCP 2026 behavior audited in Phase 10.

Runtime observations and discrepancies remained the documented vendor
configuration warnings: root URDF/SRDF inference warnings despite explicit
nested SRDF configuration and topic-provided URDF, no SRDF root virtual joint,
an end-effector parent-group warning, missing collision geometry on
`end_effector_link`, no 3D octomap sensor plugin, a default planning volume,
and an empty planner ID filled by MoveIt. None prevented the verified
operations. The SDK environment allow-list required explicit ROS
`PYTHONPATH` propagation, as documented above. Nothing needed for Phase 11
remained unverifiable.

## Files and boundaries

Created:

- `tests/integration/test_open_manipulator_x_mcp_live.py`
- `docs/README_PHASE_11.md`

Updated:

- `docs/README_PHASES.md`

No dependency or production source changed. Physical execution,
`ExecuteTrajectory`, `FollowJointTrajectory`, direct joint commands,
controller management, Servo, `plan_and_execute`, and physical robot access
remain absent. Phase 12 was not started.
