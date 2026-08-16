# Phase 18 - MCP Execution Tools, Resource, Prompt, and Mock E2E

## Scope

Phase 18 exposes only the approved application-owned execution workflow through
MCP. The default configuration remains `execution.enabled=false`. An explicit
test-only configuration enables execution for the OpenMANIPULATOR-X
`mock_components/GenericSystem` integration.

No direct trajectory, controller, FollowJointTrajectory, ROS publisher, Servo,
Hybrid Planning, or task-construction surface was added.

## Tools

`execute_motion_plan(plan_id: string)` starts the existing Phase 15–17 workflow
in an application-owned task and returns its observable execution record. It
accepts no trajectory, planning request, ROS value, or controller selection.
The returned data contains execution and plan IDs, state, backend acceptance and
start flags, cancellation evidence, terminal evidence, stabilization,
`physical_stop_confirmed`, and a bounded failure object. Disabled execution is a
structured `execution_disabled` MCP error.

`get_execution_status(execution_id: string)` returns the same bounded projection
without backend status integers, MoveIt codes, goal handles, or exceptions.

`cancel_execution(execution_id: string)` invokes only the narrow application
cancellation path. Repeated requests are idempotent. The MoveIt 2.12.4 global
stop topic and `std_msgs/String` stay private to the Jazzy adapter. Acceptance of
the cancel request does not establish standstill; only causal `PREEMPTED` plus
bounded state stabilization produces project `CANCELLED`.

## Resource and prompt

`manipulator://executions/{execution_id}` is a read-only resource template for
one retained execution. It does not enumerate or mutate executions.

`diagnose_execution_failure(execution_id)` is advisory. It directs review of the
execution, plan provenance, validation, state/scene mismatch, timeout,
quarantine, cancellation ambiguity, and MoveIt compatibility behavior. It does
not invoke tools, retry, publish stop, or claim physical safety.

The final MCP inventory is 22 Tools, 7 static Resources, 4 Resource Templates,
and 7 Prompts.

## Server guidance and safety

Server instructions now state that execution exists but is disabled by default,
requires an application-owned validated plan ID, and offers no arbitrary
trajectory, ROS, or controller access. They also identify cancellation as a
MoveIt 2.12.4 compatibility mechanism and state that `CANCELLED` is not certified
physical standstill. Quarantine remains process/adapter-restart only.

## Verification

Focused MCP tests cover exact registration, the disabled structured error,
execution ID response, bounded status, idempotent cancellation, resource
resolution, and advisory prompt behavior. Compilation, full regression,
inventory, import-boundary, tag, and default-configuration checks passed. The
focused affected MCP suites passed 18 tests. The complete non-live suite passed
with `62 passed, 8 skipped`. Direct inventory returned `22 / 7 / 4 / 7`, and the
v0.1.0 tag remained at `54caa8e387a5954d3d85d6f4c45157ea294b09ff`.

The opt-in MCP stdio E2E uses an isolated `ROS_DOMAIN_ID`, official
OpenMANIPULATOR-X 4.1.3 bringup with `use_mock_hardware:=true` and
`init_position:=false`, and the explicit `phase18_mock.toml`. It verifies normal
execution, consumed-plan rejection, MCP cancellation through `CANCELLING` to
confirmed `CANCELLED`, the execution resource, and successful post-cancel reuse.
The final opt-in result was `1 passed`. MoveIt/controller logs showed one stop
event and one controller cancellation for the cancellation execution.
No `/dev/ttyUSB0`, Dynamixel hardware plugin, physical torque, or physical robot
motion is used.

## Authoritative source audit

MCP decorators and protocol behavior were verified directly against installed
MCP Python SDK 2.0.0 and mcp-types 2.0.0. Verified signatures were
`MCPServer.tool(..., structured_output=True)`, `MCPServer.resource(uri, ...)`, and
`MCPServer.prompt(...)`; protocol discovery, structured tool results, resource
templates, resource reads, and prompt retrieval follow installed official SDK
client APIs.

The execution backend remains the Phase 17 interface verified against ROS 2
Jazzy, rclpy 7.1.11, MoveIt 2.12.4, and moveit_msgs 2.6.0. Phase 18 introduced no
new ROS or MoveIt interface. No installed-versus-upstream discrepancy or
unverified external assumption was found.

Physical hardware execution remains excluded, and Phase 19 was not started.
