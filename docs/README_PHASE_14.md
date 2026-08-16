# Phase 14 - v0.2.0 Execution Architecture and Safety Design

## Scope and outcome

Phase 14 is analysis and architecture design only. It defines the smallest
backend-neutral, trusted-plan execution design suitable for later v0.2.0
implementation. It changes no production code, configuration, version, public
MCP surface, or runtime behavior.

The governing boundary remains:

```text
MCP
 -> application execution coordinator
 -> domain / safety policy
 -> ExecutionPort
 -> ROS 2 Jazzy ExecuteTrajectory adapter
 -> MoveIt TrajectoryExecutionManager
 -> MoveIt controller manager
 -> FollowJointTrajectory controller
 -> ros2_control
 -> OpenMANIPULATOR-X hardware
```

Passing the proposed checks is not certification of physical safety. The MCP
server is neither an emergency stop nor a real-time or machinery-safety system.

## A. Repository and v0.1.0 baseline

The Phase 14 audit began from this verified state:

- repository: `/home/sarvg/projects/robotics/ros2_manipulator_mcp`;
- branch: `main`, tracking `origin/main`;
- working tree: clean;
- `HEAD` and `origin/main`:
  `ffb05289b718e2e488c25a4b1354e6e69fa38515`;
- immutable `v0.1.0` tag target:
  `54caa8e387a5954d3d85d6f4c45157ea294b09ff`.

The current code still advertises version `0.1.0`. Phase 12 has since verified
real hardware state, kinematics, and planning, but no trajectory was executed.
The tag and its historical Phase 13 release assessment remain unchanged.

## B. v0.2.0 branch strategy

Use a `dev` branch for Phase 15 onward, then merge reviewed release-ready work
to `main`. This matches the post-release convention visible in the local
`ros2_mcp` and `ros2_control_mcp` repositories and keeps the released mainline
stable while execution behavior is developed. Phase 14 did not create it.

Change package/server version from `0.1.0` to `0.2.0` in the first Phase 15
implementation change on `dev`, and describe it as **unreleased v0.2.0** in
documentation until release. A PEP 440 development suffix is unnecessary
unless an actual prerelease distribution will be published.

## C. Authoritative source audit

The following exact-version primary sources were consulted:

- installed Ubuntu 24.04.4, Python 3.12.3, ROS 2 Jazzy, `rclpy` 7.1.11,
  MoveIt 2.12.4, `moveit_msgs` 2.6.0, `control_msgs` 5.9.0, and
  `action_msgs` 2.0.4 package metadata and generated interfaces under
  `/opt/ros/jazzy`;
- installed `moveit_msgs/action/ExecuteTrajectory`,
  `moveit_msgs/msg/MoveItErrorCodes`,
  `control_msgs/action/FollowJointTrajectory`, and
  `action_msgs/srv/CancelGoal` definitions;
- MoveIt 2.12.4 upstream
  [`execute_trajectory_action_capability.cpp`](https://github.com/moveit/moveit2/blob/2.12.4/moveit_ros/move_group/src/default_capabilities/execute_trajectory_action_capability.cpp),
  [`trajectory_execution_manager.cpp`](https://github.com/moveit/moveit2/blob/2.12.4/moveit_ros/planning/trajectory_execution_manager/src/trajectory_execution_manager.cpp),
  and the simple-controller-manager FollowJointTrajectory handle;
- installed MoveIt capability constants and plugin descriptions;
- installed ROBOTIS OpenMANIPULATOR 4.1.3 MoveIt controller and ros2_control
  configuration;
- installed `rclpy.action.client` 7.1.11 source and the official Jazzy
  [`CancelGoal`](https://docs.ros.org/en/jazzy/p/action_msgs/srv/CancelGoal.html)
  definition;
- MCP Specification 2026-07-28
  [Tools](https://modelcontextprotocol.io/specification/2026-07-28/server/tools)
  and exact schema, plus installed MCP Python SDK and `mcp-types` 2.0.0;
- local and GitHub `vagotec/ros2_mcp`, `vagotec/ros2_control_mcp`, and this
  repository as internal architecture/release references only.

No installed-interface/documentation field discrepancy was found. One
important cancellation-source ambiguity is recorded in section P: the exact
MoveIt capability accepts action cancellation and defines a stop helper, but
the inspected server construction does not visibly invoke that helper from its
cancel callback. This must be resolved with exact-version mock-runtime evidence
before physical execution. No safe cancellation outcome is assumed.

## D. Exact installed ExecuteTrajectory interface

The installed `moveit_msgs` 2.6.0 action is:

```text
Goal
  moveit_msgs/RobotTrajectory trajectory
    trajectory_msgs/JointTrajectory joint_trajectory
    trajectory_msgs/MultiDOFJointTrajectory multi_dof_joint_trajectory
  string[] controller_names

Result
  moveit_msgs/MoveItErrorCodes error_code

Feedback
  string state
```

The action itself declares no additional constants. Its result uses
`MoveItErrorCodes`, whose execution-relevant installed values include
`SUCCESS=1`, `FAILURE=99999`, `MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE=-3`,
`CONTROL_FAILED=-4`, `TIMED_OUT=-6`, `PREEMPTED=-7`,
`START_STATE_IN_COLLISION=-10`, `START_STATE_VIOLATES_PATH_CONSTRAINTS=-11`,
`INVALID_MOTION_PLAN=-2`, `ROBOT_STATE_STALE=-23`,
`COMMUNICATION_FAILURE=-25`, `START_STATE_INVALID=-26`, `CRASH=-29`, and
`ABORT=-30`. The message also has `message` and `source` strings. These values
remain adapter-private.

The installed MoveIt constant is `EXECUTE_ACTION_NAME = "execute_trajectory"`.
The audited default/root MoveIt graph therefore exposes `/execute_trajectory`,
but the adapter must take an explicit resolved action name from configuration;
it must not assume a root namespace.

## E. Exact MoveIt execution path

The 2.12.4 `MoveGroupExecuteTrajectoryAction` plugin creates the
`ExecuteTrajectory` action server under `EXECUTE_ACTION_NAME`. It accepts a
goal, passes its trajectory and optional controller names to
`TrajectoryExecutionManager::push`, starts execution, waits for completion,
and maps controller-manager execution status to a MoveIt result.

`TrajectoryExecutionManager` selects suitable controllers, can manage their
activation according to MoveIt configuration, partitions trajectories where
needed, validates the first trajectory state against current state, monitors
execution duration, sends trajectories through controller handles, waits, and
stops controller execution on its stop path. It allows only one unfinished
execution context.

The installed default start tolerance in MoveIt 2.12.4 is `0.01`, configurable
through `trajectory_execution.allowed_start_tolerance`; zero disables that
MoveIt check. Its source waits for current state and rejects unknown joints,
length mismatches, or a start outside tolerance. This is backend defense in
depth, not a substitute for the application's explicit policy check.

## F. Controller and hardware relationship

ROBOTIS 4.1.3 configures
`moveit_simple_controller_manager/MoveItSimpleControllerManager`. The arm
mapping uses controller `arm_controller`, action namespace
`follow_joint_trajectory`, type `FollowJointTrajectory`, and joints
`joint1`-`joint4`. The resulting controller action is
`/arm_controller/follow_joint_trajectory` in the audited root namespace. The
ros2_control controller is a position-interface
`joint_trajectory_controller/JointTrajectoryController`, which ultimately
drives the Dynamixel hardware interface.

The project must call MoveIt's `ExecuteTrajectory`, not
`FollowJointTrajectory` directly. That preserves MoveIt's trajectory execution
manager, controller selection, start validation, duration monitoring, and
controller abstraction. Direct controller calls would duplicate
`ros2_control_mcp` ownership, couple the application to controller names and
types, and bypass MoveIt execution protections. The adapter must send an empty
`controller_names` list so MoveIt selects configured controllers; arbitrary
client controller selection is not exposed.

## G. Minimum execution model

The minimum backend-neutral public/domain concepts are:

- `ExecutionId`: an opaque, high-entropy, non-empty string; a type alias or
  validated field is enough, not a wrapper hierarchy;
- `ExecutionState`: `preparing`, `starting`, `running`, `cancelling`,
  `succeeded`, `failed`, `cancelled`, or `timed_out`;
- immutable `ExecutionStatus`: execution ID, plan ID, state, timestamps,
  optional stable failure, cancellation-requested flag, and
  `physical_stop_confirmed` which remains false/unknown unless a future
  authoritative mechanism can establish it;
- application-private `_ExecutionRecord`: current immutable status plus the
  PlanRegistry reservation and opaque adapter operation token.

No separate `ExecutionRequest` is justified while the only accepted input is
`plan_id`. No distinct `ExecutionResult` is needed because terminal
`ExecutionStatus` carries the outcome. `DomainFailure` remains the failure
envelope. ROS goal handles, futures, status integers, MoveIt messages, and
controller data remain inside the Jazzy adapter.

The application port should be narrow and asynchronous: check availability,
start one domain `Trajectory`, cancel by its adapter-private operation token,
and deliver backend-neutral status/result updates. It must not accept raw ROS
objects or controller names.

## H. Execution state machine

`preparing` covers reservation and preflight checks; `starting` begins only
when the adapter is sending/waiting for goal acceptance; `running` begins on
accepted goal. This avoids separate client-visible `pending` and `validating`
states that add no useful behavior.

Legal transitions are:

```text
preparing -> starting | failed | timed_out | cancelled
starting  -> running | failed | timed_out | cancelling
running   -> succeeded | failed | timed_out | cancelling
cancelling -> cancelled | succeeded | failed | timed_out
```

Terminal states are `succeeded`, `failed`, `cancelled`, and `timed_out`.
Cancellation racing with natural completion may truthfully end `succeeded`.
Backend rejection or a mapped execution error ends `failed`. A bounded wait
expiry ends `timed_out`, after an attempted cancellation/cleanup; timeout does
not imply that motion stopped.

On shutdown, reject new work, transition an active operation to `cancelling`,
request cancellation, wait only for the cleanup timeout, retain the final
known state, destroy the action client, then close the existing ROS runtime.
An unconfirmed stop must be logged and exposed as unconfirmed, never rewritten
as `cancelled` merely because shutdown continued.

## I. Trusted-plan rule

Physical execution accepts exactly one application-issued `plan_id`. The
server obtains the immutable `MotionPlan` and trajectory from `PlanRegistry`.
Clients cannot submit, replace, edit, or select the trajectory, ROS goal,
controller, or MoveIt message.

Plans are single-use for execution. Once a MoveIt execution goal has been
accepted, the plan is consumed regardless of success, failure, cancellation,
or timeout. This prevents accidental retries from an obsolete physical start
state. A new plan is required for another attempt. If preparation or goal
acceptance fails before MoveIt accepts the goal, the reservation may be
released without consumption, but every later attempt repeats all validation.

Successful planning does not authorize execution. Policy identity, planning
group, expiry, start state, scene revision, validity, concurrency, feature
enablement, and backend readiness are checked again at execution time.

## J. Required PlanRegistry changes

Add one in-memory reservation per executing plan and an atomic
`acquire_for_execution(plan_id, execution_id)` operation. Acquisition purges
ordinary expired entries first, validates ownership/expiry, and marks the plan
reserved. A reserved entry is pinned: lazy TTL cleanup, capacity eviction, and
discard must not remove it.

Add an atomic release operation with explicit `consume` behavior. Goal
acceptance causes eventual `consume=True`; pre-acceptance failure permits
`consume=False`. Capacity eviction skips reserved entries. If capacity is full
of reserved entries, storing a new plan fails with a stable capacity/conflict
error instead of evicting active evidence. No database, queue, persistence, or
distributed lock is needed. One application lock protects registry acquisition
and active-execution selection as one critical section.

## K. Pre-execution validation pipeline

The deterministic order is:

1. **Application configuration:** execution is explicitly enabled and all
   execution timeout/policy values are finite and valid.
2. **Application/registry:** atomically establish that no execution conflicts,
   then acquire an existing, unexpired, application-owned plan.
3. **Domain/application:** recheck immutable trajectory structure and that its
   joints/group agree with the stored planning request.
4. **SafetyEvaluator:** require the recorded policy ID to equal the active
   policy and the group to remain permitted.
5. **State adapter:** obtain a new complete measured state within a bounded
   deadline and reject stale data.
6. **SafetyEvaluator:** compare every planned trajectory joint in the first
   point with measured state under explicit project policy; reject missing or
   mismatched joints.
7. **Scene adapter/SafetyEvaluator:** read the current scene and require the
   approved revision rule.
8. **Runtime MoveIt validation:** call existing state-validity checks for the
   first state and, for the baseline, every trajectory point before execution.
   A backend error or invalid point rejects execution.
9. **Execution adapter:** verify configured `ExecuteTrajectory` action-server
   availability within its discovery timeout.
10. Recheck the active-execution ownership and scene-mutation guard immediately
    before changing `preparing` to `starting`.
11. Convert the already-owned domain trajectory inside the adapter, send it
    with no client-selected controllers, and bound goal acceptance.

Checks 1-4 are application/domain invariants and policy. Checks 5-8 are live
application safety revalidation backed by adapter reads. MoveIt's trajectory
execution manager performs its own start-state and controller checks again at
runtime. Checks 9-11 belong to adapter availability/lifecycle. None certifies
machinery safety or proves the workspace is free of people or unmodeled
obstacles.

## L. Start-state revalidation

Use the first trajectory point and its exact trajectory joint-name order.
Every such joint must appear exactly once in the measured state; missing,
duplicate, non-finite, or unmatched values reject execution. Extra measured
joints, such as the gripper during an arm plan, are ignored. Compare angular
distance by joint semantics; for the OpenMANIPULATOR-X arm's bounded revolute
joints, absolute radian difference is sufficient.

The tolerance belongs in `SafetyPolicy`, with a possible future per-joint
override. It must be positive, finite, explicit, documented as project policy,
and never silently clamped. MoveIt's installed `0.01` rad default is evidence,
not automatic project policy; the actual project value requires approval.

State acquisition must wait for a newly received complete state and also check
its ROS header age against a configured maximum using the node's ROS clock.
The adapter maps missing/zero/uncomparable or over-age stamps to
`stale_state`; the application never receives ROS time objects. If the robot
was manually moved after planning beyond tolerance, execution is rejected and
the client must replan from the measured state.

## M. Planning-scene revalidation

For the v0.2.0 baseline require exact equality:

```text
current_scene_revision == motion_plan.scene_revision
```

Deterministic rejection is preferable to deciding that an unclassified scene
change is harmless. The current revision represents the scene components read
by this project, presently primitive world collision objects; it is not a
universal MoveIt scene certificate. Before Phase 15, confirm that the current
serialization/hash is canonical across repeated unchanged reads and extend its
documented component coverage if necessary. A mismatch consumes no plan before
goal acceptance and requires revalidation/replanning.

## N. Execution enablement

Preserve the existing safety switch, but rename/migrate it deliberately in
Phase 15 to the clearer `[execution] enabled = false`, defaulting false. The
typed composition root owns it. Hardware detection, MoveIt availability,
active controllers, or a client request can never change it.

The safest stable MCP behavior is to register the v0.2 tools consistently but
have `execute_motion_plan` reject with `policy_rejected` while disabled; health
and safety Resources disclose the disabled state. Startup may construct the
action client for discovery but must not send a goal. Enabling requires an
explicit configuration edit and server restart. Authentication is not added;
deployment access control remains outside this local stdio server.

## O. Concurrency strategy

Allow one active physical execution per configured service/manipulator. A
second execute request fails immediately with `conflict`; there is no queue.
Reads and execution status/cancel remain available.

While an execution is `preparing`, `starting`, `running`, or `cancelling`:

- reject new planning requests, because the measured state may be changing;
- reject all planning-scene mutations;
- reject discard of the reserved plan, while allowing discard of unrelated
  plans;
- allow read-only state, scene, plan, health, and status operations;
- serialize repeated cancel requests and make them return the current status.

The execution coordinator and PlanRegistry reservation share one application
lock for state transitions, but the lock is never held across ROS waits.
Shutdown follows section H.

## P. Cancellation semantics

`cancel_execution(execution_id)` looks up only the application-owned execution,
marks cancellation requested, and asks `ExecutionPort.cancel`. The adapter uses
the private rclpy goal handle's `cancel_goal_async()` with a bounded wait.

Installed `CancelGoal` semantics are exact: return code `ERROR_NONE=0` means at
least one goal transitioned to `CANCELING`; `ERROR_REJECTED=1`,
`ERROR_UNKNOWN_GOAL_ID=2`, and `ERROR_GOAL_TERMINATED=3` do not. Even
`ERROR_NONE` is only cancel acceptance. Public state becomes `cancelling`, not
`cancelled`, until a terminal action result/status is received. A terminal ROS
`CANCELED` status or MoveIt `PREEMPTED` result maps to application `cancelled`;
a completion race may map to `succeeded`.

Critically, the inspected MoveIt 2.12.4 capability accepts cancel requests and
defines `preemptExecuteTrajectoryCallback()` calling
`TrajectoryExecutionManager::stopExecution(true)`, but its visible action-server
cancel lambda only returns `ACCEPT` and does not visibly call that helper. The
simple-controller layer has a cancellation path, but Phase 14 cannot prove that
this exact action capability invokes it from an external cancel. Phase 15 must
verify cancellation propagation and terminal behavior using official mock
hardware. Until then, and even after ROS cancellation completes, the public API
must report `physical_stop_confirmed=false`: controller/action completion is
not independently certified physical standstill.

## Q. Timeout strategy

Use separate positive finite configuration values:

| Timeout | Owner | Outcome |
|---|---|---|
| action discovery | Jazzy adapter | `unavailable` after bounded `wait_for_server` |
| goal acceptance | Jazzy adapter | cancel if a late accepted handle appears; `timeout` |
| execution completion | application coordinator | request cancel, enter `cancelling`, then bounded cleanup |
| cancel acknowledgement | Jazzy adapter | retain stop-unconfirmed status; stable timeout/failure |
| shutdown cleanup | composition/runtime | stop accepting work, cancel, bounded wait, deterministic destroy |

The adapter owns ROS future and action-server waits. The application owns the
maximum operation lifetime and state transition. Completion timeout should be
configurable and may later be validated against planned trajectory duration,
but no numeric defaults are approved in Phase 14. MoveIt's own duration
monitoring remains enabled as independent backend defense.

## R. Backend-neutral error strategy

Retain existing errors and add only:

- `conflict`: another execution is active or a plan is reserved;
- `plan_invalidated`: policy, start-state, scene, or trajectory revalidation
  invalidated an otherwise known plan;
- `execution_failed`: an accepted execution ended unsuccessfully.

Continue using `not_found`, `policy_rejected`, `unavailable`, `timeout`,
`stale_state`, `invalid_request`, and `backend_error`. Cancel rejection can use
`execution_failed` while returning the still-current execution status; it does
not need its own permanent code.

The Jazzy adapter maps MoveIt error values and ROS goal terminal statuses to
these categories. Raw integers, backend messages, exceptions, goal handles,
and controller result strings are logged privately with bounded/sanitized
context and never exposed. Unknown values map to `backend_error` or
`execution_failed`, never guessed success.

## S. Proposed MCP v0.2.0 surface

Only these additions are justified:

| Surface | Input / result | MCP annotations |
|---|---|---|
| `execute_motion_plan` Tool | `plan_id`; returns opaque `execution_id` and status | `readOnlyHint=false`, `destructiveHint=true`, `idempotentHint=false`, `openWorldHint=false` |
| `get_execution_status` Tool | `execution_id`; returns immutable status | read-only, non-destructive, idempotent, closed-world |
| `cancel_execution` Tool | `execution_id`; returns cancellation acknowledgement/current status | mutating, non-destructive, idempotent for repeated calls, closed-world |
| `manipulator://executions/{execution_id}` Resource template | same bounded status snapshot | read-only resource |
| `diagnose_execution_failure` Prompt | execution ID/context | analysis instructions only; never executes or cancels |

MCP annotations are advisory and untrusted under the 2026-07-28 specification;
they are not authorization. Explicit opaque handles follow the specification's
stateful-tool guidance. A list resource is unnecessary for a one-active-
execution baseline and could expose retained history without a recovery need.
Do not add combined plan-and-execute or raw trajectory/controller tools.

## T. Mock-hardware-first progression

Phase 15 and later implementation should progress only through these gates:

1. domain status and registry reservation;
2. application coordinator, policy, revalidation, and mock port;
3. exact Jazzy `ExecuteTrajectory` adapter;
4. official OpenMANIPULATOR-X 4.1.3 mock-hardware MoveIt runtime;
5. real MCP stdio plan, execute, status, and cancellation E2E;
6. repeated verification of timeout, mismatch, scene mutation, concurrency,
   shutdown, and cancellation behavior;
7. only then propose a new explicit real-hardware checkpoint.

Mock execution must demonstrate measured joint-state progress/final state and
MoveIt/controller terminal results, not merely action goal acceptance.

## U. Later real OpenMANIPULATOR-X checkpoint

Preserve all Phase 12 evidence and warnings. A later checkpoint requires fresh
explicit user approval immediately before any physical goal. Before approval,
repeat device identity/permissions, official launch/config audit, joint/TF
state, scene, validity, execution-disabled default, cancellation mock evidence,
and shutdown/torque-off plan.

After approval, use a clear workspace, human observation, accessible power
cutoff, conservative scaling, a fresh measured state, and one very small
single-joint plan well inside limits. Revalidate it immediately, execute once
through MCP -> application -> MoveIt only, observe action and measured state,
then shut down and verify controller deactivation and torque off. Cancellation
on real hardware must be a separate specifically approved test and must not be
attempted until mock cancellation propagation is understood. Do not claim an
E-stop or certified stop.

Existing observations remain relevant: FastSyncRead `-3001` fallback,
vendor `InitItem` register writes, unavailable FIFO scheduling, disabled URDF
command-limit enforcement warning, MoveIt/SRDF/end-effector/octomap warnings,
and shutdown warnings. Official bringup enables torque even with
`init_position:=false`.

## V. Minimal later test strategy

Prefer a compact suite centered on failure boundaries:

- two or three state-machine tests covering success, cancellation race, and
  timeout/cleanup;
- three application tests covering disabled execution, stale/start-mismatched
  or scene-mismatched plan, and concurrent execution;
- two PlanRegistry tests proving reserved plans survive TTL/capacity and are
  consumed/released correctly;
- one adapter goal-timeout test and one cancellation/result-mapping cleanup
  test;
- architecture scan proving ROS/MoveIt imports remain under `ros/jazzy`;
- strong opt-in mock-hardware tests for successful movement, rejection paths,
  cancellation propagation, and MCP stdio status;
- later one narrowly approved, opt-in real-hardware execution checkpoint.

Combine related invariant cases parametrically where clearer. Do not test
trivial getters, dataclass mechanics, every enum edge independently, or create
broad mocks for coverage count.

## W. Version and documentation strategy

At Phase 15 start, create `dev`, bump package/server metadata together to
`0.2.0`, and mark the development line unreleased. Update README only when its
instructions and public inventory describe working behavior. Add a `CHANGELOG`
in the first v0.2 implementation phase or release-preparation phase: execution
is a material public and safety change, so a durable release history is now
justified.

Keep this phases index factual: Phase 14 is complete design-only; later rows are
added only when explicitly started/completed. Do not rewrite the released
v0.1.0 history. Release notes must state physical-execution verification status
precisely.

## X. Decisions requiring approval before Phase 15

1. The numeric project start-state tolerance, and whether v0.2 starts with one
   scalar or explicit per-joint values.
2. Maximum measured-state age and the treatment of zero/uncomparable ROS
   timestamps.
3. Numeric discovery, goal-acceptance, completion, cancellation, and shutdown
   timeouts; whether completion limit is fixed or trajectory-derived with a cap.
4. Confirmation of single-use consumption after goal acceptance and reusable
   behavior for failures before acceptance.
5. Whether execution tools stay visible while disabled (recommended) or are
   absent from the advertised surface.
6. Verification that scene revision calculation is stable and covers every
   scene component intended as an execution gate.
7. Execution-status retention count/TTL after terminal completion.
8. Exact MoveIt 2.12.4 cancellation propagation, to be settled by source
   clarification or deterministic mock-runtime evidence before real hardware.
9. Whether planning is rejected for the entire active-execution window
   (recommended) or only during physical movement.

No unresolved choice may be silently filled from model memory.

## Y. Explicitly out of scope

- any Phase 14 production implementation or public MCP change;
- arbitrary/client-supplied trajectories or ROS messages;
- direct `FollowJointTrajectory`, controller selection/switching, hardware
  lifecycle, torque, Dynamixel register, firmware, or serial commands;
- execution queues, persistence, databases, brokers, distributed locks, or
  workflow engines;
- combined plan-and-execute, pose/joint convenience execution, replay, batch
  execution, jogging, Servo, Hybrid Planning, or MoveIt Task Constructor;
- attach/detach, mesh scene objects, advanced constraints, perception,
  navigation, camera, LiDAR, or task orchestration;
- gripper execution until separately designed and approved;
- authentication/authorization framework work;
- claims of certified collision avoidance, safe speed/torque, protective stop,
  emergency stop, human detection, or guaranteed physical standstill;
- any real or mock ROS runtime, hardware startup, torque enablement, controller
  activation, action goal, physical movement, Phase 15, commit, tag, push, or
  release during Phase 14.

## Phase 14 change audit

Changed only `docs/README_PHASES.md` and this document. No production code,
configuration, dependency, test, package version, branch, commit, tag, or
remote state was changed. No ROS, MoveIt, ros2_control, MCP server, mock
hardware, or physical-hardware runtime was started. No trajectory or controller
command was sent, torque was not changed, and Phase 15 was not started.
