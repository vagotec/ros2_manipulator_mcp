# Phase 7 - MCP Tools for v0.1.0

## Scope and architecture

Phase 7 registers MCP Tools for every approved capability already exposed by `ManipulatorService`. It adds no backend operation, Resource, Prompt, or execution path.

```text
MCP Tool
 -> ManipulatorService
 -> SafetyEvaluator where applicable
 -> application port
 -> JazzyManipulatorAdapter
 -> ROS 2 Jazzy / MoveIt 2
```

Tool modules import application and domain types only. They contain no ROS, MoveIt, adapter, service-name, action-name, or node references. Generic tools contain no OpenMANIPULATOR-X identifiers.

## Implemented tools

| Classification | Tool | Application method |
|---|---|---|
| Read-only | `describe_manipulator` | `describe_manipulator` |
| Read-only | `list_planning_groups` | `list_planning_groups` |
| Read-only | `get_planning_group` | `list_planning_groups`, then typed lookup |
| Read-only | `get_current_robot_state` | `get_current_state` |
| Read-only | `get_end_effector_pose` | `get_end_effector_pose` |
| Read-only | `compute_forward_kinematics` | `compute_forward_kinematics` |
| Read-only | `compute_inverse_kinematics` | `compute_inverse_kinematics` |
| Read-only | `check_robot_state_validity` | `check_state_validity` |
| Planning | `plan_to_joint_goal` | `plan_to_joint_goal` |
| Planning | `plan_to_pose_goal` | `plan_to_pose_goal` |
| Planning | `compute_cartesian_path` | `compute_cartesian_path` |
| Read-only | `get_motion_plan` | `get_motion_plan` |
| State-changing registry | `discard_motion_plan` | `discard_motion_plan` |
| Validation | `validate_motion_plan` | `prepare_plan_validation` |
| Read-only | `get_planning_scene` | `get_planning_scene` |
| Read-only | `list_collision_objects` | `get_planning_scene`, then typed projection |
| Read-only | `get_collision_object` | `get_planning_scene`, then typed lookup |
| State-changing scene | `apply_collision_object` | `apply_collision_object` |
| State-changing scene | `remove_collision_object` | `remove_collision_object` |

The lookup tools use already-returned application domain values. They do not duplicate backend calls or introduce new ports.

## Schemas and results

The installed SDK derives JSON Schema 2020-12 inputs from typed function signatures and nested Pydantic models. Inputs represent poses, quaternions, robot state, joint/pose goals, scaling values, Cartesian waypoints, collision primitives, and application-owned plan IDs. Extra nested input fields are rejected.

All tools advertise one stable structured output envelope:

```json
{
  "success": true,
  "status": "success",
  "error_code": null,
  "message": "Operation succeeded.",
  "data": {}
}
```

Domain/application failures set `success=false`, `status="error"`, a stable domain error code, and MCP `isError=true`. Tool-originated errors therefore remain visible to an LLM for correction instead of becoming protocol lookup errors. ROS messages, MoveIt codes, and Python exception objects never enter results.

Planning results contain `plan_id`, planning group, planning duration, scene revision, request scaling metadata, and a bounded trajectory summary: joint names, point count, and total duration. Raw trajectory points are not returned. A client cannot submit a trajectory or manufacture a trusted plan.

## Safety path

Joint planning, pose planning, and Cartesian planning call `ManipulatorService`, which evaluates the active Phase 4 policy before the planning port. Scene add/remove tools call only the corresponding service methods. Collision-object additions therefore perform scene read, policy evaluation, and only then port mutation. Policy rejection returns `policy_rejected` without a backend write.

Tool annotations are hints as specified by MCP. Read-only operations set `readOnlyHint`; planning and mutations do not. Scene removal/replacement and plan discard use destructive hints. These hints do not replace server-side policy enforcement.

## Composition and lifecycle

`create_server()` registers schemas without constructing ROS resources, preserving the Phase 1 construction guarantee. Tests inject an existing `ManipulatorService`. The stdio `main()` composition root loads configuration, constructs the verified OpenMANIPULATOR-X/Jazzy adapter and application service, runs MCP, and closes the adapter in `finally`. Importing the package or constructing the server does not start ROS.

## Tests

Five focused tests verify:

1. the exact 19-tool set and absence of execution/move tools;
2. discovery returns structured data obtained through the service;
3. planning returns an application-owned plan ID and bounded trajectory summary;
4. policy rejection is a stable MCP tool error and prevents backend mutation;
5. an allowed scene mutation reaches the fake backend through the real application/safety path.

No rclpy mocks or MCP SDK internals are tested. Phase 6 already covers live MoveIt interoperability.

## Authoritative source audit

Consulted MCP Specification 2026-07-28 sections:

- [Server Tools](https://modelcontextprotocol.io/specification/2026-07-28/server/tools): `tools/list`, `tools/call`, input/output schemas, structured content, tool-originated errors, annotations, and security guidance
- the 2026-07-28 generated `Tool`, `ToolAnnotations`, `OutputSchema`, and `CallToolResult` definitions installed with `mcp-types` 2.0.0

Verified installed MCP Python SDK 2.0.0 APIs and source:

- `mcp.server.MCPServer`
- `MCPServer.tool(...)` and `add_tool(...)`
- `MCPServer.list_tools()` and `call_tool(...)`
- `structured_output=True` and Pydantic return-schema derivation
- `Annotated[CallToolResult, OutputModel]` validation
- `CallToolResult.content`, `structuredContent`, and `isError`
- `mcp_types.ToolAnnotations` hint fields
- `MCPServer.run(transport="stdio")`

The installed SDK and `mcp-types` distributions both report exactly `2.0.0`; installed bindings contain the `2026_07_28` protocol model. No ROS/MoveIt API was changed or newly assumed in this phase, so the verified Phase 5 and Phase 6 sources remain applicable.

Discrepancies and unverifiable assumptions:

- The SDK 2.0.0 package exposes `MCPServer` directly and has no `mcp.server.fastmcp` module. Implementation follows the installed API rather than older FastMCP examples.
- No required MCP behavior remained unverifiable. Network retrieval of the dated specification page was unavailable during implementation, so exact wire requirements were additionally checked against the official installed 2026-07-28 generated bindings and SDK source.

## Files

Created:

- `src/ros2_manipulator_mcp/mcp/tools/__init__.py`
- `src/ros2_manipulator_mcp/mcp/tools/models.py`
- `src/ros2_manipulator_mcp/mcp/tools/registration.py`
- `tests/mcp/test_tools.py`
- `docs/README_PHASE_7.md`

Updated:

- `src/ros2_manipulator_mcp/server.py`
- `src/ros2_manipulator_mcp/mcp/instructions.py`
- `docs/README_PHASES.md`

No dependency changed.

## Explicit exclusions and deferrals

There is no `execute_motion_plan`, `move_to_pose`, `move_to_joint_goal`, `plan_and_execute`, `send_trajectory`, ExecuteTrajectory wrapper, FollowJointTrajectory wrapper, controller-management tool, arbitrary ROS service/action/topic tool, or shell tool.

Resources and Prompts remain deferred. Physical execution, attach/detach, meshes, Servo, Hybrid Planning, MoveIt Task Constructor, perception, navigation, and task orchestration remain out of scope.

Phase 8 was not started.
