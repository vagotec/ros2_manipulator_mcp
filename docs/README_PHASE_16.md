# Phase 16 - Pre-execution Revalidation and Safety Gate

## Scope and outcome

Phase 16 implements the deterministic application safety gate between a
reserved stored plan in `VALIDATING` and later adapter submission. Passing the
gate transitions the execution to `STARTING`; it does not submit a goal, consume
the plan, start MoveIt execution, or imply physical safety certification.

No MCP execution surface, ROS action client, controller command, timer, worker,
or physical motion path was added.

## Validation pipeline

`ManipulatorService.validate_execution(execution_id)` performs these checks in
order:

1. the execution exists and is exactly `VALIDATING`;
2. execution remains enabled and the ID owns the one active execution slot;
3. the stored plan exists and is reserved by that execution, which also proves
   it is neither available to another execution nor consumed;
4. stored policy identity, allowed planning group, planning limits, velocity
   scaling, and acceleration scaling are re-evaluated through the existing
   `SafetyEvaluator`;
5. the backend description is readable and still contains the planning group;
6. a current robot state is readable and carries a backend-computed sample age;
7. sample age is no greater than `0.25` seconds;
8. every planning-group joint exists in the stored trajectory's first point and
   current measured state, and differs by no more than `0.02` rad;
9. the planning scene is readable and its revision equals the stored plan
   revision exactly.

The stored trajectory remains an immutable domain `Trajectory`, so its finite
values, unique names, matching position lengths, and monotonic timestamps have
already been enforced. The Phase 16 structural gate additionally requires its
first point to contain every current planning-group joint.

Descriptor, state, and scene failures are converted into stable validation
findings. Phase 16 does not check `ExecuteTrajectory` availability; that is a
Phase 17 responsibility.

## State freshness

The existing `RobotState.timestamp_seconds` is a source timestamp but did not
identify a clock domain to the application. Subtracting it from Python wall or
monotonic time would therefore be unsafe, especially with ROS simulated time.

`RobotState` now optionally carries backend-neutral `sample_age_seconds`. The
Jazzy adapter computes it immediately after receiving `/joint_states` by
subtracting the message header stamp from its own node clock. Both values are
therefore in the same ROS clock domain. A timestamp ahead of that clock is
rejected by the adapter rather than clamped. Missing age becomes
`STATE_FRESHNESS_UNKNOWN`; age above the configured project limit becomes
`STATE_STALE`.

The approved `0.25` seconds is project policy, not a ROS or MoveIt standard.

## Start-state comparison

The expected state is the stored trajectory's first point. Only the planning
group's required joints are compared; unrelated measured joints are ignored.
Domain `JointState` already rejects duplicates and non-finite values.

Missing trajectory joints produce `TRAJECTORY_START_JOINT_MISSING`. Missing
measured joints produce `MEASURED_JOINT_MISSING`. Deviations greater than
`0.02` rad produce `START_STATE_MISMATCH` with:

- joint name;
- expected position;
- measured position;
- absolute deviation;
- configured tolerance.

Equality at the limit passes. No per-joint tolerance or silent clamping exists.

## Scene and policy revalidation

The current and planned scene revisions must be exactly equal when
`require_exact_scene_revision=true`, which remains the default. A difference
produces `SCENE_REVISION_MISMATCH`; no automatic replan or semantic-change
guess is attempted. Domain scene/plan construction already forbids absent or
empty revisions.

Stored policy identity and group permission are checked again. The complete
stored `PlanningRequest` is also re-evaluated, so currently available planning,
attempt, velocity-scaling, and acceleration-scaling limits cannot bypass a
changed policy.

## Result and state transition

`ExecutionValidationResult` is immutable and includes:

- `valid` derived from whether findings are empty;
- execution ID and plan ID;
- active policy ID;
- stable findings;
- state age;
- start-state comparison outcome;
- scene-revision comparison outcome.

`ExecutionValidationFinding` is a small stable model with optional joint
comparison fields. It contains no ROS or MoveIt object.

Success performs `VALIDATING -> STARTING`. In Phase 16, `STARTING` means only
that application validation passed and the plan is ready for a later adapter
submission. The plan remains reserved and unconsumed.

Rejection performs `VALIDATING -> FAILED`, records
`execution_validation_failed`, and releases the reservation. The plan becomes
available with its suspended TTL restored. No backend execution attempt occurs.
Unknown execution IDs or calls in the wrong execution state return existing
structured application errors instead of fabricating a validation result.

## Scene mutation and discard boundaries

Both primitive scene mutation methods reject with `execution_conflict` whenever
the registry has an active execution. This includes `PENDING`, `VALIDATING`,
`STARTING`, `RUNNING`, and `CANCELLING`; `PENDING` counts because it already owns
the single active slot. Collision-object application rechecks the guard after
its scene read and policy evaluation, before requesting the backend mutation.

Reserved-plan discard remains rejected by `PlanRegistry` throughout the active
states. Terminal validation failure releases the plan, after which ordinary
discard is permitted.

## Stable findings and error changes

Phase 16 adds the result-level findings:

- `EXECUTION_DISABLED`
- `EXECUTION_OWNERSHIP_CONFLICT`
- `PLAN_RESERVATION_INVALID`
- existing safety-policy finding codes
- `BACKEND_DESCRIPTION_UNAVAILABLE`
- `PLANNING_GROUP_UNAVAILABLE`
- `CURRENT_STATE_UNAVAILABLE`
- `STATE_FRESHNESS_UNKNOWN`
- `STATE_STALE`
- `TRAJECTORY_START_JOINT_MISSING`
- `MEASURED_JOINT_MISSING`
- `START_STATE_MISMATCH`
- `PLANNING_SCENE_UNAVAILABLE`
- `SCENE_REVISION_MISMATCH`

The single new terminal error category is `execution_validation_failed`.
Backend exceptions, raw timestamps, ROS messages, and MoveIt errors remain
contained.

## Tests and verification

Seven focused tests prove:

1. a fresh matching state and exact scene reach `STARTING` while the plan stays
   reserved;
2. a stale sample rejects and releases the plan;
3. start mismatch returns complete joint-specific evidence;
4. a scene revision mismatch rejects without replanning;
5. a missing required measured joint rejects;
6. backend state-read failure becomes a bounded finding and releases the plan;
7. active validation blocks scene mutation before the backend write and keeps
   discard blocked.

Results:

- Python compilation: passed;
- focused Phase 16 tests: `7 passed`;
- Phase 15 plus Phase 16 execution tests: `13 passed`;
- complete permitted non-live suite: `52 passed, 7 deselected`;
- deselected: six opt-in live integrations and the existing test that
  deliberately constructs an rclpy runtime.

Static audits confirm domain/application/safety contain no ROS, MoveIt, control,
or MCP imports. No `ExecuteTrajectory`, `FollowJointTrajectory`, or MCP execution
Tool/Resource/Prompt exists. Execution still defaults disabled.

## Authoritative source audit

No new MoveIt, ros2_control, action, or MCP interface was introduced. The only
new external runtime assumption is adapter-side ROS clock access. It was
verified against installed ROS 2 Jazzy `rclpy` 7.1.11 source:

- `Node.get_clock()` returns the node's configured `Clock`;
- `Clock.now()` returns a `Time` in that clock domain;
- `Time.nanoseconds` is the total nanoseconds since that clock's epoch;
- the already verified Jazzy `sensor_msgs/JointState.header.stamp` supplies the
  sample time.

The installed exact runtime matched the official Jazzy API semantics. No
discrepancy or unverified external assumption remains for Phase 16.

## Deferred work and limitations

The gate is application policy and consistency validation, not certified
machinery safety, collision guarantees, human detection, emergency stop,
real-time enforcement, or confirmed physical standstill.

Phase 17 retains the execution adapter port, exact MoveIt
`ExecuteTrajectory` action client, backend readiness, goal acceptance,
plan consumption, result/status mapping, cancellation, timeouts, and cleanup.
Later phases retain MCP execution capabilities, mock-hardware E2E, and any
separately approved real-hardware execution checkpoint.

No ROS or hardware runtime was started, no controller or torque state changed,
no trajectory was sent, no physical execution occurred, and Phase 17 was not
started.
