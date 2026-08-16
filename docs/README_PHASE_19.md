# Phase 19 - v0.2.0 Mock Execution Audit and RC Preparation

## Scope and findings

Phase 19 audits the existing v0.2.0 execution implementation under bounded
edge conditions. It adds no public feature. All live work used official
OpenMANIPULATOR-X 4.1.3 `mock_components/GenericSystem` in isolated ROS domain
219 with `use_mock_hardware:=true` and `init_position:=false`.

The audit found and corrected two existing robustness defects. An accepted
execution whose terminal result timed out previously entered quarantine without
first requesting the approved stop. It now transitions through `CANCELLING`,
dispatches at most one stop, then records `TIMED_OUT` and quarantines. Adapter
shutdown previously closed ROS resources while an accepted execution could
remain active. It now attempts the same exactly-once private stop before bounded
runtime cleanup. Neither path claims stop acknowledgement, stabilization, or
physical standstill.

## Mock audit results

Three sequential real-MCP executions produced distinct plan and execution IDs,
reached their measured endpoints, ended `SUCCEEDED`, and permanently rejected
each consumed plan. No stale record or quarantine affected later work.

While a long execution was `RUNNING`, a second stored plan returned stable
`execution_conflict`. MoveIt logs showed no second execution goal, no queue was
created, and the first execution remained independently cancellable.

A plan made before moving the mock robot was rejected twice with
`execution_validation_failed`, `accepted=false`, proving start-state mismatch
released its reservation and did not consume it. MoveIt received no execution
request for either attempt.

A plan made before a policy-compliant primitive scene change was likewise
rejected twice with `accepted=false`. This proved exact scene revision mismatch,
reservation release, and non-consumption. The audit object was removed before
completion, and no execution request was sent for the stale plan.

Repeated cancellation returned `CANCELLING` without duplicate stop publication.
MoveIt logged exactly one `stop`, causal controller cancellation, and terminal
`PREEMPTED`; the project then recorded `CANCELLED` after state stabilization.
Cancellation after terminal returned stable `invalid_execution_transition` and
published no stop. The deterministic gated unit path separately proved the
`STARTING` latch, one post-acceptance stop, and confirmed cancellation.

A post-cancel execution succeeded. A final long execution was left active while
the real MCP stdio server exited; adapter shutdown returned boundedly, MoveIt
logged one stop and controller cancellation, and no client/task/process remained.

## Timeout and quarantine

The deterministic fake timeout test accepted one goal, exhausted the configured
terminal-result bound, requested stop exactly once, transitioned to `TIMED_OUT`,
and quarantined the backend. A later execution returned `backend_quarantined`.
The existing ambiguous-success test preserves its failed execution record,
quarantines, and rejects later execution. There is no auto-clear or public reset;
process/adapter restart remains the only recovery.

## Default and MCP contract audits

The shipped default remains `execution.enabled=false`, and a normal default MCP
server returns structured `execution_disabled`. The test-only mock configuration
is isolated under `tests/integration`.

The MCP inventory remains exactly 22 Tools, 7 static Resources, 4 Resource
Templates, and 7 Prompts. The only execution surface is:

- `execute_motion_plan(plan_id)`;
- `get_execution_status(execution_id)`;
- `cancel_execution(execution_id)`;
- `manipulator://executions/{execution_id}`;
- `diagnose_execution_failure(execution_id)`.

No raw trajectory execution, controller name, generic ROS publication,
`trajectory_execution_event`, `std_msgs/String`, FollowJointTrajectory, or
controller-management operation is exposed above `ros/jazzy`.

## Verification results

- focused execution/adapter audit: `16 passed`;
- complete permitted non-live suite: `65 passed, 9 skipped`;
- isolated real-MCP mock audit: `1 passed`;
- Python compilation and whitespace checks: passed;
- MCP inventory, architecture/import boundaries, default configuration, branch,
  and v0.1.0 tag checks: passed;
- final process and `/dev/ttyUSB0` checks: clean.

## Authoritative-source audit

Phase 19 introduced no new ROS, MoveIt, or MCP interface. Timeout and shutdown
reuse the already verified MoveIt 2.12.4 implementation mechanism:
`/trajectory_execution_event`, `std_msgs/msg/String`, command `stop`. Its behavior
remains supported by the exact installed/upstream MoveIt 2.12.4 trajectory
execution manager, ExecuteTrajectory capability, action-based controller handle,
and joint_trajectory_controller 4.40.1 sources. Installed ROS 2 Jazzy, rclpy
7.1.11, moveit_msgs 2.6.0, ros2_control 4.45.2, MCP SDK 2.0.0, and mcp-types 2.0.0
remain the verified baseline. No discrepancy was found.

This global, unacknowledged topic is an implementation compatibility mechanism,
not a stable MoveIt cancellation API. Every future supported MoveIt version must
re-verify it.

## Remaining blockers for physical execution

Real OpenMANIPULATOR-X execution still requires a new explicit user-approved
checkpoint. Before motion, that checkpoint must address the audited vendor
startup behavior that enables Dynamixel torque and activates position
controllers even with `init_position:=false`; establish an operator-controlled
workspace and physical emergency-stop/recovery procedure; verify current device,
IDs, baud, firmware, controller state, robot pose, collision scene, and
conservative motion limits; and define observation and abort authority.

The MCP layer is not certified machinery safety, does not prove physical
standstill, and does not replace hardware emergency stopping. No physical
hardware, torque, Dynamixel command, or physical execution occurred. Phase 20 was
not started.
