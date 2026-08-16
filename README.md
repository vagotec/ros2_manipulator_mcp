# ros2_manipulator_mcp

`ros2_manipulator_mcp` is a backend-neutral Model Context Protocol server for
inspecting ROS 2 manipulators, computing kinematics, planning motion, and
managing primitive MoveIt planning-scene objects. It gives MCP clients a
small typed manipulation API instead of arbitrary access to ROS services,
topics, actions, nodes, or shell commands.

Version `0.1.0` is planning-only. It can generate and validate trajectories,
but it cannot execute them or command physical robot motion.

## Architecture

```text
MCP client
 -> stdio MCP server
 -> application service
 -> backend-neutral domain and safety policy
 -> application ports
 -> ROS 2 Jazzy / MoveIt adapter
 -> rclpy and MoveIt 2
```

MoveIt is a replaceable backend, not the Manipulator domain. Domain,
application, and safety modules contain no ROS, MoveIt, or MCP types. ROS
messages and Jazzy-specific behavior remain behind the adapter boundary.

Controller and hardware management belong to `ros2_control_mcp`, not this
project.

## Supported baseline

The verified v0.1.0 baseline is:

- Ubuntu 24.04.4 LTS
- Python 3.12.3 or compatible Python 3.12+
- ROS 2 Jazzy
- `rclpy` 7.1.11
- MoveIt 2.12.4 and `moveit_msgs` 2.6.0
- MCP Specification 2026-07-28
- MCP Python SDK 2.0.0 and `mcp-types` 2.0.0
- ROBOTIS OpenMANIPULATOR-X packages/configuration 4.1.3

The MCP server exposes stdio only.

## Capabilities

The public v0.1.0 surface contains 19 Tools, 7 static Resources, 3 Resource
Templates, and 6 Prompts.

Tools cover:

- manipulator and planning-group discovery;
- current robot state and end-effector pose;
- forward kinematics, collision-aware inverse kinematics, and state validity;
- joint-goal, pose-goal, and Cartesian-path planning;
- opaque application-owned plan retrieval, validation, and discard;
- planning-scene reads and primitive collision-object add/remove.

Resources provide bounded overview, group, current-state, planning-scene,
safety-policy, health, collision-object, and stored-plan context. Prompts guide
inspection, diagnosis, plan review, scene-change review, and workflows that
stop before execution.

See [Phase 13](docs/README_PHASE_13.md) for the exact API inventory.

## Safety model

Planning and planning-scene mutations pass through a deterministic
`SafetyEvaluator`. Policy limits include planning groups, planning time and
attempts, scaling factors, Cartesian path requirements, workspace bounds,
collision-object types/dimensions/count, scene frames, replacement, and scene
provenance.

This is application policy enforcement, **not certified physical safety**.
The project does not provide or claim:

- emergency-stop or machinery-safety capability;
- real-time safe torque or speed enforcement;
- guaranteed collision avoidance;
- human detection;
- physical execution authorization.

There is no execution Tool, `ExecuteTrajectory` wrapper,
`FollowJointTrajectory` wrapper, controller-management Tool, direct joint
command, arbitrary ROS operation, or shell Tool.

## OpenMANIPULATOR-X reference

The generic API is manipulator-independent. The v0.1.0 composition root ships
one explicit reference profile verified against ROBOTIS 4.1.3:

- model: `open_manipulator_x`
- planning frame: `world`
- arm group: `arm`
- active arm joints: `joint1`, `joint2`, `joint3`, `joint4`
- gripper group: `gripper`
- tool frame: `end_effector_link`
- KDL position-only IK
- OMPL default planning pipeline

The full MCP-to-MoveIt path is verified with the official mock-hardware
runtime. Physical OpenMANIPULATOR-X verification is deferred because hardware
was not connected.

For future physical verification, `init_position:=false` prevents the vendor
launch's explicit initialization trajectory. It does not make startup
motion-free: real bringup still enables Dynamixel torque and activates
position controllers. See [Phase 12](docs/README_PHASE_12.md) before any
physical startup.

## Installation

Install the exact ROS packages through the ROS 2 Jazzy apt repository:

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

The project declares NumPy and PyYAML because installed Jazzy Python modules
import them when used from the isolated uv environment.

## Configuration

The packaged configuration is
`src/ros2_manipulator_mcp/config/default.toml`. Select another file with:

```bash
export ROS2_MANIPULATOR_MCP_CONFIG=/path/to/config.toml
```

The default configuration selects the `ros2_jazzy_moveit` backend and
`open_manipulator_x` profile, uses a five-second service timeout, rejects
physical execution, and defines the effective planning/Cartesian/scene
policy. Empty group, object-prefix, and scene-frame allow-lists mean those
properties are unrestricted by that specific allow-list; other validation
still applies. Review defaults before deployment.

The adapter has typed endpoint defaults for the root-namespace MoveIt graph:
`/joint_states`, `/compute_ik`, `/compute_fk`, `/check_state_validity`,
`/compute_cartesian_path`, `/plan_kinematic_path`, `/get_planning_scene`, and
`/apply_planning_scene`. Alternative endpoint names require constructing the
adapter with `JazzyMoveItSettings`; they are not TOML options in v0.1.0.

Stored plans are process-local and immutable. The default `PlanRegistry`
holds at most 32 plans for 300 seconds. These limits are code-level defaults,
not configuration-file settings in v0.1.0, and plans disappear when the
server exits.

## Starting the server

Start the desired ROS 2 / MoveIt planning graph first. For the verified
official mock-hardware baseline, use separate sourced terminals and an
isolated domain:

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

Then run the stdio server from a ROS-sourced shell:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=66
uv run ros2-manipulator-mcp
```

The server writes MCP protocol messages to stdout. Do not use that terminal
for interactive input outside an MCP client.

## Connecting an MCP client

Configure an MCP 2026-07-28 client to spawn the stdio command. If the client
does not inherit a ROS-sourced environment, a shell wrapper can source Jazzy
before replacing itself with the server:

```json
{
  "mcpServers": {
    "ros2-manipulator": {
      "command": "/bin/bash",
      "args": [
        "-lc",
        "source /opt/ros/jazzy/setup.bash && export ROS_DOMAIN_ID=66 && exec /path/to/ros2_manipulator_mcp/.venv/bin/ros2-manipulator-mcp"
      ]
    }
  }
}
```

Replace the project path and domain for the deployment. The verified modern
protocol flow uses `server/discover`; SDK 2.0.0's legacy `initialize()` path
negotiates an older protocol revision.

## Representative workflow

A conservative planning workflow is:

1. Read `manipulator://overview`, `manipulator://health`,
   `manipulator://safety`, `manipulator://state/current`, and
   `manipulator://scene`.
2. Select a returned planning group and validate the current state.
3. Call `plan_to_joint_goal` or `plan_to_pose_goal` with conservative scaling.
4. Inspect the returned opaque `plan_id` with `get_motion_plan` or
   `manipulator://plans/{plan_id}`.
5. Call `validate_motion_plan`.
6. Discard the plan when finished.
7. Stop. There is no execution operation.

For scene changes, inspect the scene and safety Resource first, add one
policy-compatible primitive, verify it, and remove it. Policy rejection is a
structured Tool error and does not call the backend mutation.

## Testing

Run the normal graph-independent suite:

```bash
uv run pytest -q
```

Six ROS/MoveIt integration tests are opt-in. They require the documented
OpenMANIPULATOR-X mock graph and never execute trajectories:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=66
export ROS2_MANIPULATOR_MCP_RUN_LIVE_TESTS=1
uv run pytest -q tests/integration
```

Phase 12 includes no fabricated physical-hardware test result.

## Limitations and v0.2.0 direction

v0.1.0 supports one production composition profile, one stdio transport,
primitive collision objects, process-local plans, and service/topic-based
Jazzy MoveIt integration. It does not include physical execution,
attach/detach, mesh objects, Servo/jogging, Hybrid Planning, MoveIt Task
Constructor, controller or hardware management, perception, navigation,
task orchestration, resource subscriptions, or persistent plan storage.

Possible v0.2.0 work requires separate approval and may include additional
profiles/configurability and carefully scoped capabilities. Physical
execution is not implied by this direction.

Development and verification history is indexed in
[docs/README_PHASES.md](docs/README_PHASES.md).

## License

Licensed under the [Apache License 2.0](LICENSE). See [NOTICE](NOTICE) for
copyright and attribution information.
