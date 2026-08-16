# Phase 17 - Production Execution Backend and Workflow

## Scope and architecture

Phase 17 adds a backend-neutral `ExecutionPort`, application-owned execution
workflow, and the ROS 2 Jazzy implementation behind `ros/jazzy`. Execution
accepts only an immutable stored `plan_id`; raw trajectories, controllers, ROS
goals, and generic publishing are not public operations. No MCP execution Tool,
Resource, or Prompt was added. Execution remains disabled by default.

One `ManipulatorService` owns at most one execution in `PENDING`, `VALIDATING`,
`STARTING`, `RUNNING`, or `CANCELLING`. Scene mutation remains blocked during
that interval. Planning remains allowed as approved in Phase 14 because it does
not change the world or execution slot; its resulting plans remain independently
owned and cannot bypass reservation. The Jazzy adapter additionally assumes
exclusive ownership of MoveIt's execution facility.

## Submission, consumption, and result mapping

After the complete Phase 16 gate reaches `STARTING`, the adapter submits
`moveit_msgs/action/ExecuteTrajectory` to `/execute_trajectory`. Its goal contains
only the stored domain trajectory converted to `RobotTrajectory`; public
controller selection is absent. Server readiness and goal acceptance are each
bounded at 2 seconds. A rejected or failed pre-acceptance submission transitions
to `FAILED` and releases the reservation. Only acceptance consumes the plan,
permanently preventing reuse regardless of the later result.

The exact installed mapping is:

- ROS `SUCCEEDED` plus MoveIt `SUCCESS` -> project `SUCCEEDED`;
- ROS `ABORTED` plus MoveIt `PREEMPTED`, after a project stop request and
  successful stabilization -> project `CANCELLED`;
- every other combination -> `FAILED`, or quarantine when cancellation makes
  the outcome ambiguous.

ROS `ABORTED` alone and stop publication alone never mean cancellation.

## MoveIt 2.12.4 cancellation compatibility

Phase 17B proved `ExecuteTrajectory.cancel_goal_async()` cannot reliably stop
active execution on this baseline, so it is not used. The Jazzy adapter exposes
only `request_execution_stop(execution_id)` and internally publishes exactly one
reliable depth-1 `std_msgs/msg/String(data="stop")` message to the global
`/trajectory_execution_event` topic. This is a MoveIt 2.12.4 implementation
compatibility mechanism, not a stable generic MoveIt API.

Cancellation in `STARTING` is latched: rejection releases the plan without a
stop; acceptance consumes it and immediately dispatches one stop. Repeated calls
are idempotent, and stale/wrong IDs cannot publish. The execution slot remains
owned until the causal terminal result and required post-stop observation.

After the expected `PREEMPTED` result, backend-neutral measured joint state must
remain quiet for 0.5 seconds within a 2-second observation window. Position
change must be at most 0.001 rad between samples; reported velocity must be at
most 0.01 rad/s when present. These are conservative project defaults, not
certified safety limits. `state_stabilized=true` records only this observation.
`physical_stop_confirmed` remains false. **CANCELLED does not mean certified
physical standstill.**

## Quarantine and bounds

Success after stop, result timeout, unexpected post-stop result, failed stop
dispatch, or failed stabilization quarantines execution until adapter/process
restart. New execution is rejected with stable `backend_quarantined`; there is
no public reset. Normal terminal timeout is trajectory duration times 2 plus 2
seconds, capped at 30 seconds. Cancellation terminal timeout is 2 seconds.
Runtime shutdown is configured for 5 seconds; client/publisher cleanup and
adapter close are idempotent.

## Tests and verification

Focused tests cover port-compatible fakes, acceptance/consumption, rejection and
reservation release, normal success, exactly-once stop, confirmed cancellation,
ambiguous-success quarantine, active-slot conflict, installed message conversion,
exact endpoint/result mapping, and idempotent cleanup. The complete non-live
suite passed with `59 passed, 7 skipped`; Python compilation and import-boundary
audits passed. The opt-in OpenMANIPULATOR-X execution test passed (`1 passed`)
using `use_mock_hardware:=true` and `init_position:=false`. Through the production
service it confirmed normal `SUCCEEDED` execution and measured endpoint, one-stop
`PREEMPTED` plus stabilization to `CANCELLED`, successful execution after that
cancellation, and rejection of consumed-plan reuse. MoveIt logs independently
showed the controller cancel request and later successful execution. Cleanup left
no MoveIt, ros2_control, or Dynamixel hardware process. No physical hardware path
was authorized.

## Authoritative source audit

Verified against installed ROS 2 Jazzy `rclpy` 7.1.11, `moveit_msgs` 2.6.0,
MoveIt 2.12.4, ros2_control 4.45.2, and joint trajectory controller 4.40.1.
Primary upstream definitions/source consulted:

- `moveit_msgs/action/ExecuteTrajectory.action` and
  `moveit_msgs/msg/MoveItErrorCodes.msg`;
- MoveIt 2.12.4 `execute_trajectory_action_capability.cpp` and
  `trajectory_execution_manager.cpp`;
- MoveIt 2.12.4 action-based controller handle and
  `follow_joint_trajectory_controller_handle.cpp`;
- ros2_controllers 4.40.1 `joint_trajectory_controller.cpp`.

Installed constants and fields matched upstream: goal `trajectory` and
`controllers`, result `error_code`, `SUCCESS=1`, `PREEMPTED=-7`, ROS terminal
statuses `SUCCEEDED=4` and `ABORTED=6`. The adapter intentionally leaves goal
controllers empty. No discrepancy or invented external interface remains.

## Limitations

The stop topic is global, unacknowledged, and implementation-specific; exclusive
ownership and causal terminal evidence are therefore mandatory. This is not an
emergency stop, real-time safety function, collision guarantee, or physical
standstill proof. Future MoveIt versions must re-verify the compatibility path.
No physical robot execution was performed, and Phase 18 was not started.
