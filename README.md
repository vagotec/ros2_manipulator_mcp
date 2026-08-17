# ros2_manipulator_mcp

`ros2_manipulator_mcp` is a backend-neutral Model Context Protocol server for
inspecting ROS 2 manipulators, computing kinematics, planning and validating
motion, managing primitive MoveIt planning-scene objects, and executing
application-owned plans when execution has been explicitly enabled. It exposes
a small typed manipulation API rather than arbitrary ROS, controller, or shell
access.

Version `0.2.0` adds bounded execution, status, and cancellation workflows. The
shipped default remains fail-closed: `execution.enabled = false`.

## Architecture

```text
MCP client
 -> stdio MCP server
 -> application service
 -> backend-neutral domain and safety policy
 -> application ports
 -> ROS 2 Jazzy / MoveIt adapter
 -> MoveIt 2 -> ros2_control -> manipulator hardware or mock hardware
```

MoveIt is replaceable backend infrastructure, not the Manipulator domain.
Domain, application, and safety modules contain no ROS, MoveIt, or MCP types.
ROS messages and version-specific Jazzy behavior, including execution stopping,
remain behind the adapter boundary. Controller and hardware lifecycle management
belong to `ros2_control_mcp`, not this project's public MCP surface.

## Supported baseline

The verified v0.2.0 baseline is:

- Ubuntu 24.04.4 LTS and Python 3.12+
- ROS 2 Jazzy, `rclpy` 7.1.11
- MoveIt 2.12.4 and `moveit_msgs` 2.6.0
- `ros2_control` 4.45.2 and `joint_trajectory_controller` 4.40.1
- ROBOTIS OpenMANIPULATOR-X packages 4.1.3 and
  `dynamixel_hardware_interface` 1.5.2
- MCP Specification 2026-07-28, MCP Python SDK 2.0.0, and `mcp-types` 2.0.0

The MCP server exposes stdio only.

## MCP surface

The public v0.2.0 surface contains 22 Tools, 7 static Resources, 4 Resource
Templates, and 7 Prompts.

The Tools cover discovery; state, pose, FK, IK, and validity; joint, pose, and
Cartesian planning; application-owned plan inspection, validation, and discard;
primitive planning-scene operations; and these three execution operations:

- `execute_motion_plan(plan_id)`
- `get_execution_status(execution_id)`
- `cancel_execution(execution_id)`

Execution context is also available at
`manipulator://executions/{execution_id}`, and
`diagnose_execution_failure(execution_id)` provides a non-mutating diagnostic
prompt. No raw trajectory, controller name, direct `FollowJointTrajectory`,
arbitrary ROS service/action/topic, controller-management, or shell interface is
exposed.

## Safety and execution contract

Planning, scene mutation, and execution pass through deterministic application
policy and validation. Stored plans are process-local, immutable, and
application-owned. Before execution the service reserves the plan, requires a
fresh matching start state, enforces start tolerance and exact scene revision,
and permits only one active execution per manipulator. Backend acceptance
consumes the plan permanently; a failure before acceptance releases its
reservation. Ambiguous cancellation or unsafe timeout outcomes quarantine the
backend until process/adapter restart.

These controls are application safeguards, **not certified physical safety**.
In particular:

- physical execution is opt-in and disabled in the shipped configuration;
- `cancel_execution` is not an emergency stop or machinery-safety function;
- project `CANCELLED` requires causal MoveIt `PREEMPTED` and measured-state
  stabilization evidence, but does not certify physical standstill;
- the physical power cutoff remains the emergency mechanism;
- the `/trajectory_execution_event` stop mechanism is specific to the audited
  MoveIt 2.12.4 baseline and must be reverified for every newly supported
  MoveIt version.

## OpenMANIPULATOR-X reference and verification

The generic API is manipulator-independent. The shipped reference profile uses
the official ROBOTIS 4.1.3 model: `world` planning frame, four-joint `arm`
group, `gripper` group, and `end_effector_link`. Only the independently
commandable `gripper_left_joint` is exposed in the gripper group;
`gripper_right_joint` remains its URDF mimic joint.

The complete MCP-to-MoveIt path is verified on official mock hardware. Phase 20
also verified real startup, Dynamixel IDs 11-15, controller and torque lifecycle,
real execution and separately operator-visible movement for joints 1-4 and ID
15, orderly Torque OFF, and serial-device release. See
[Phase 20](docs/README_PHASE_20.md) for the evidence matrix.

Accepted hardware limitations are: real-hardware cancellation was deliberately
not physically release-verified; the tested ID 15 mechanism has a mechanical or
encoder-reference mismatch relative to the official model; one historical ID
12 shutdown has an unknown initiating cause; and FastSyncRead can return `-3001`
before the official driver successfully falls back to normal SyncRead. Do not
change URDF/SRDF limits or offsets to mask the ID 15 mechanical-reference issue.

## Installation

Install the ROS dependencies from the ROS 2 Jazzy apt repository:

```bash
sudo apt-get install \
  ros-jazzy-moveit \
  ros-jazzy-open-manipulator-bringup \
  ros-jazzy-open-manipulator-moveit-config
```

Then create the isolated Python environment from the lock file:

```bash
cd /path/to/ros2_manipulator_mcp
uv sync --frozen
```

## Configuration and startup

The packaged configuration is
`src/ros2_manipulator_mcp/config/default.toml`. Select a deployment-specific
copy with `ROS2_MANIPULATOR_MCP_CONFIG`. The default selects the
`ros2_jazzy_moveit` backend and `open_manipulator_x` profile and has:

```toml
[execution]
enabled = false
```

For planning-only use, start the desired ROS 2/MoveIt graph and then start the
stdio server from a ROS-sourced shell. The verified mock-hardware example is:

```bash
export ROS_DOMAIN_ID=66
ros2 launch open_manipulator_bringup open_manipulator_x.launch.py \
  use_mock_hardware:=true init_position:=false start_rviz:=false
```

```bash
export ROS_DOMAIN_ID=66
ros2 launch open_manipulator_moveit_config open_manipulator_x_moveit.launch.py \
  start_rviz:=false
```

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=66
uv run ros2-manipulator-mcp
```

The server writes MCP protocol messages to stdout. A client that does not
inherit a ROS-sourced environment should use a wrapper that sources Jazzy and
then `exec`s `.venv/bin/ros2-manipulator-mcp`.

To enable execution intentionally, make a reviewed deployment-specific copy of
the configuration, set `[execution] enabled = true`, point
`ROS2_MANIPULATOR_MCP_CONFIG` at that copy, and restart the server. This switch
does not authorize a particular motion or make the system safe: operators must
separately establish hardware readiness, clearance, observation, and immediate
physical cutoff access. Never enable execution merely to inspect or plan.

## Planning and execution workflow

A bounded planning workflow is:

1. Inspect overview, health, safety, current-state, group, and scene Resources.
2. Validate the fresh current state and select a returned planning group.
3. Plan a joint, pose, or Cartesian goal with conservative scaling.
4. Inspect the opaque `plan_id` and call `validate_motion_plan`.
5. Discard the plan if it will not be executed.

When execution is explicitly enabled and separately authorized:

1. Reconfirm current physical and backend readiness.
2. Call `execute_motion_plan(plan_id)` once; never submit a raw trajectory.
3. Observe `get_execution_status` or the execution Resource to a terminal state.
4. Treat telemetry, controller result, and physical observation as separate
   evidence.
5. Use `cancel_execution` only as an application stop request, never as an
   emergency-stop substitute.

## Testing

Run the normal graph-independent regression suite with:

```bash
uv run pytest -q
```

Live integration tests are opt-in and require the documented official
OpenMANIPULATOR-X mock graph. Each integration module documents its required
`ROS2_MANIPULATOR_MCP_RUN_*` environment gate. Phase 20 hardware results are
documented evidence and are not part of routine regression.

## Known scope limits

v0.2.0 provides one production composition profile, one stdio transport,
primitive collision objects, and process-local plans and executions. It does
not provide attach/detach, meshes, Servo/jogging, Hybrid Planning, MoveIt Task
Constructor, perception, navigation, task orchestration, controller or hardware
management, persistent plan storage, authentication, or certified safety.

The development history and exact verification evidence are indexed in
[docs/README_PHASES.md](docs/README_PHASES.md).

## License

Licensed under the [Apache License 2.0](LICENSE). See [NOTICE](NOTICE) for
copyright and attribution information.
