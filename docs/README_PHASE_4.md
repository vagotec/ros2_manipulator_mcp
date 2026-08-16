# Phase 4 - Safety and Policy Layer

## Goal

Introduce deterministic backend-neutral policy checks for planning, Cartesian paths, primitive planning-scene mutations, and stored-plan validation preparation. This boundary is not a certified physical safety system.

## Safety models

- `SafetyPolicy`: immutable configuration-driven limits and allowlists
- `WorkspaceBounds`: optional inclusive Cartesian bounds
- `SafetyFinding`: stable textual code, message, and severity
- `SafetyDecision`: policy identity, structured findings, and allowed result
- `SafetyEvaluator`: deterministic evaluation with no backend access

The packaged TOML contains the usable conservative defaults. A deployment may create another `SafetyPolicy` from equivalent configured values.

## Implemented policies

Planning checks allowed groups, planning time, attempts, velocity/acceleration scaling, joint-target count, and optional pose workspace bounds. Cartesian checks add waypoint count, step bounds, collision avoidance, waypoint workspace bounds, and minimum completion fraction.

Primitive scene checks cover object-ID prefixes, object count, primitive allowlist, maximum individual dimension, frame allowlist, duplicate IDs, and explicit replacement permission. Removal checks naming policy only; an allowed unknown ID still reaches the backend, which defines the not-found result.

Stored plans retain the policy identity active at creation. Validation preparation checks that identity, the current group allowlist, and required scene-revision metadata. Registry expiry is evaluated before safety validation, so expired plans return the existing stable not-found result.

## Application integration

```text
ManipulatorService
 -> SafetyEvaluator
 -> capability port, only when allowed
```

Planning and scene mutations are rejected before their mutating or planning backend call. Cartesian results are also checked after computation so a partial path below policy cannot silently be treated as accepted. Rejections use the domain `policy_rejected` category and include stable finding codes.

## Stable finding codes

Implemented codes include:

- `PLANNING_GROUP_NOT_ALLOWED`
- `PLANNING_TIME_EXCEEDED`
- `PLANNING_ATTEMPTS_EXCEEDED`
- `VELOCITY_SCALING_EXCEEDED`
- `ACCELERATION_SCALING_EXCEEDED`
- `JOINT_TARGET_LIMIT_EXCEEDED`
- `POSE_OUTSIDE_WORKSPACE`
- `CARTESIAN_WAYPOINT_LIMIT_EXCEEDED`
- `CARTESIAN_STEP_OUT_OF_POLICY`
- `COLLISION_AVOIDANCE_REQUIRED`
- `WAYPOINT_OUTSIDE_WORKSPACE`
- `CARTESIAN_PATH_INCOMPLETE`
- `OBJECT_ID_NOT_ALLOWED`
- `OBJECT_ALREADY_EXISTS`
- `OBJECT_REPLACE_NOT_ALLOWED`
- `COLLISION_OBJECT_LIMIT_EXCEEDED`
- `PRIMITIVE_TYPE_NOT_ALLOWED`
- `PRIMITIVE_DIMENSION_EXCEEDED`
- `SCENE_FRAME_NOT_ALLOWED`
- `PLAN_POLICY_MISMATCH`
- `PLAN_SCENE_REVISION_REQUIRED`

## Non-guarantees

Policy acceptance does not certify safe motion or a safe robot. The layer does not inspect live ROS state, perform collision checking, compare current joints with a planned start state, authorize execution, provide emergency stopping, enforce real-time limits, or validate hardware and controllers.

## Validation

```bash
uv run pytest -q tests/test_foundation.py tests/domain tests/application tests/safety
uv run python -m compileall -q src tests
```

Five Phase 4 tests prove pre-backend planning rejection, allowed planning and policy association, oversized geometry rejection, replace enforcement, and policy-invalid stored-plan rejection.

## Deferred

- ROS state and transform freshness
- MoveIt collision and trajectory revalidation
- start-state comparison
- backend availability and current-scene comparison
- physical execution authorization and monitoring
- emergency-stop and certified hardware safety
- meshes, attachment, perception, Servo, Hybrid Planning, and task workflows

## Next phase

Phase 5 must be separately approved before ROS 2 Jazzy / MoveIt 2 integration is implemented.

## Authoritative source audit

Sources consulted:

- [Python 3.12 `tomllib` documentation](https://docs.python.org/3.12/library/tomllib.html)
- [MCP specification 2026-07-28](https://modelcontextprotocol.io/specification/2026-07-28)
- [MCP Python SDK v2.0.0 source](https://github.com/modelcontextprotocol/python-sdk/tree/v2.0.0)
- [ROS 2 Jazzy Jalisco documentation](https://docs.ros.org/en/jazzy/Releases/Release-Jazzy-Jalisco.html)
- [MoveIt messages 2.6.0 source](https://github.com/moveit/moveit_msgs/tree/2.6.0)
- locally installed package metadata, Python signatures, and Jazzy interface files under `/opt/ros/jazzy`

Exact installed baseline verified on 2026-08-16:

- Ubuntu 24.04.4 LTS
- Python 3.12.3 and standard-library `tomllib`
- MCP Python SDK 2.0.0 and `mcp-types` 2.0.0
- `MCPServer.run(transport=...)` accepts `stdio`, `sse`, and `streamable-http`; this phase retains the existing `stdio` choice
- ROS distribution `jazzy`
- `rclpy` 7.1.11
- `moveit_msgs` 2.6.0, including the Jazzy planning, kinematics, Cartesian-path, state-validity, and planning-scene service definitions

Discrepancies found: none affecting Phase 4. The live Python documentation is at maintenance release 3.12.13 while Ubuntu provides Python 3.12.3; the `tomllib` API used here is present in the installed interpreter. No ROS, MoveIt, or MCP protocol interface is called by the Phase 4 safety layer.

Unverified assumptions: none in the implemented Phase 4 policy logic. Concrete ROS service semantics, topic names, QoS, MoveIt error translation, and node behavior remain deliberately unimplemented and must be verified during the later adapter phase.
