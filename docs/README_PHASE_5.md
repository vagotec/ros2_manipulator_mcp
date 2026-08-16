# Phase 5 - ROS 2 Jazzy / MoveIt 2 Adapter Foundation

## Scope

Phase 5 implements the replaceable ROS adapter contract, managed `rclpy` runtime, domain/message conversions, and the approved non-execution MoveIt service paths. It registers no MCP capabilities and implements no trajectory execution.

```text
Application capability ports
 -> ManipulatorAdapter protocol
 -> JazzyManipulatorAdapter
 -> JazzyRosRuntime
 -> rclpy 7.1.11
 -> MoveIt 2.12.4 / moveit_msgs 2.6.0
```

ROS imports exist only below `ros/jazzy`. The existing application, domain, safety, and MCP packages remain ROS-independent.

## Files

Created:

- `src/ros2_manipulator_mcp/ros/adapter.py`
- `src/ros2_manipulator_mcp/ros/jazzy/adapter.py`
- `src/ros2_manipulator_mcp/ros/jazzy/conversions.py`
- `src/ros2_manipulator_mcp/ros/jazzy/runtime.py`
- `src/ros2_manipulator_mcp/ros/jazzy/settings.py`
- package initializers under `ros/` and `ros/jazzy/`
- `tests/ros/jazzy/test_jazzy_adapter.py`

Updated the former `adapters/ros2_jazzy_moveit` placeholder to re-export the canonical `ros/jazzy` implementation. `pyproject.toml` and `uv.lock` now include NumPy and PyYAML because the installed Jazzy Python modules import them.

## Implemented interfaces

The default absolute names below correspond to a root-namespace `move_group`. Every endpoint is configurable through `JazzyMoveItSettings` because the upstream capability constants are relative names and ROS namespace resolution is deployment-specific.

| Capability | Installed interface | Default endpoint |
|---|---|---|
| Inverse kinematics | `moveit_msgs/srv/GetPositionIK` | `/compute_ik` |
| Forward kinematics | `moveit_msgs/srv/GetPositionFK` | `/compute_fk` |
| State validity | `moveit_msgs/srv/GetStateValidity` | `/check_state_validity` |
| Cartesian path | `moveit_msgs/srv/GetCartesianPath` | `/compute_cartesian_path` |
| Plan-only motion planning | `moveit_msgs/srv/GetMotionPlan` | `/plan_kinematic_path` |
| Planning-scene read | `moveit_msgs/srv/GetPlanningScene` | `/get_planning_scene` |
| Planning-scene diff | `moveit_msgs/srv/ApplyPlanningScene` | `/apply_planning_scene` |
| Current joint state | `sensor_msgs/msg/JointState` | `/joint_states` |

The installed `moveit_msgs/action/ExecuteTrajectory` and upstream relative endpoint `execute_trajectory` were verified but deliberately not implemented. Public physical execution remains outside v0.1.0.

## Conversion behavior

- FK and IK exchange `moveit_msgs/RobotState`, framed poses, group names, and link names.
- IK enables collision avoidance and uses a bounded `builtin_interfaces/Duration`.
- State validity returns only the backend-neutral Boolean result.
- Motion planning creates verified `JointConstraint`, `PositionConstraint`, and `OrientationConstraint` messages. Joint tolerance defaults to the upstream MoveGroupInterface 2.12.4 value `1e-4`; pose tolerances come from the domain goal.
- Cartesian requests require one common waypoint frame and preserve explicit fraction, step, jump threshold, scaling, and collision behavior.
- Joint trajectories preserve names, positions, optional velocity/acceleration arrays, and normalized durations.
- Scene reads request robot state plus world object names and geometry. A SHA-256 digest of the serialized scene world is the application scene revision.
- Scene writes use `PlanningScene.is_diff=true` and verified `CollisionObject.ADD` or `REMOVE` operations. Phase 4 authorizes replacement before the adapter is called.
- MoveIt integer codes are translated to stable domain categories inside the adapter and never cross the boundary.

The public scene domain supports exactly one box, sphere, cylinder, or cone per object. Meshes, planes, unknown primitives, and multi-primitive objects return a backend error rather than being silently omitted.

## Timeout and lifecycle behavior

`JazzyRosRuntime` owns a private `rclpy.Context`, node, two-thread executor, cached typed service clients, and daemon spin thread. Service discovery, service response, topic sampling, IK computation, and joint-state sampling are bounded independently.

Calls that wait on ROS run through `asyncio.to_thread`, keeping application methods async-compatible. Joint-state reads use a temporary subscription with the installed `qos_profile_sensor_data`; the owned executor services the callback. Service-unavailable and response-timeout outcomes are distinguished.

`close()` is idempotent. It stops the executor, joins the thread, removes and destroys the node, and shuts down only the adapter-owned context. Partial construction also shuts down its initialized context.

## Tests

Seven Phase 5 tests cover:

- structural conformance to the composite adapter protocol and idempotent cleanup
- installed JointState and CollisionObject conversion
- verified IK fields and `NO_IK_SOLUTION` translation
- verified plan service fields, empty planner selection, and joint tolerance
- bounded timeout translation
- real installed Jazzy context/node/executor lifecycle without a robot
- automated ROS-import boundary enforcement

The full Phase 1-5 suite reports `25 passed`. Python compilation and explicit import-boundary searches also pass.

The real lifecycle test needs normal local network-interface access for the installed CycloneDDS RMW. It does not require a robot or MoveIt graph.

## Authoritative source audit

Sources consulted:

- installed interface definitions under `/opt/ros/jazzy/share/moveit_msgs`, `/opt/ros/jazzy/share/shape_msgs`, `/opt/ros/jazzy/share/sensor_msgs`, and `/opt/ros/jazzy/share/trajectory_msgs`
- installed `rclpy` 7.1.11 source and signatures under `/opt/ros/jazzy/lib/python3.12/site-packages/rclpy`
- [MoveIt 2.12.4 capability names](https://github.com/moveit/moveit2/blob/2.12.4/moveit_ros/move_group/include/moveit/move_group/capability_names.hpp)
- [MoveIt 2.12.4 default capabilities](https://github.com/moveit/moveit2/tree/2.12.4/moveit_ros/move_group/src/default_capabilities)
- [MoveIt 2.12.4 constraint utilities](https://github.com/moveit/moveit2/blob/2.12.4/moveit_core/kinematic_constraints/src/utils.cpp)
- [MoveGroupInterface 2.12.4 defaults](https://github.com/moveit/moveit2/blob/2.12.4/moveit_ros/planning_interface/move_group_interface/src/move_group_interface.cpp)
- [moveit_msgs 2.6.0 source](https://github.com/moveit/moveit_msgs/tree/2.6.0)
- [ROS 2 Jazzy `rclpy` documentation](https://docs.ros.org/en/jazzy/p/rclpy/)

Exact installed versions verified:

- `rclpy` 7.1.11
- MoveIt 2 packages 2.12.4
- `moveit_msgs` 2.6.0
- Ubuntu `python3-yaml` 6.0.1 and project PyYAML 6.0.3
- Ubuntu `python3-numpy` 1.26.4; the project lock resolved NumPy 2.5.2 within the declared compatible range

The service request/response fields, `MoveItErrorCodes`, planning-scene component bits, collision-object operations, solid-primitive dimension ordering, trajectory fields, duration fields, and endpoint constants were checked against those exact installed/upstream definitions.

Discrepancies found:

- The isolated venv initially lacked PyYAML and NumPy even though installed `rclpy` and generated Jazzy messages import them. Both are now explicit bounded runtime dependencies.
- MoveIt capability names are relative upstream. The adapter's leading-slash defaults are valid only for the documented root-namespace deployment and therefore remain configurable.

Unverified assumptions and intentionally deferred items:

- OpenMANIPULATOR-X MoveIt configuration packages are available from the Jazzy repository but are not installed locally. No planning-group, joint, link, or end-effector names were hardcoded; a verified deployment descriptor must be supplied.
- No running `move_group` graph was available, so service interoperability was tested with installed message classes and typed fake responses, not a live planner.
- Robot-model discovery, SRDF parsing, multi-DOF joints, attached objects, meshes, multiple primitives per object, planner selection, QoS overrides, and namespaced launch integration remain deferred.
- No execution action, controller access, physical robot operation, Servo, Hybrid Planning, perception, navigation, or task orchestration was implemented.

## Phase boundary

Phase 6 was not started. The MCP server composition root still constructs no ROS resources, and no MCP Tool, Resource, or Prompt uses this adapter yet.
