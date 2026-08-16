# Phase 15 - Execution Domain, State Machine, and Plan Reservation

## Scope

Phase 15 begins unreleased v0.2.0 implementation on branch `dev`. It adds only
backend-neutral execution state, in-memory ownership, trusted-plan reservation,
configuration, and a preparation-only application skeleton.

It adds no ROS action client, MoveIt execution adapter, controller call, MCP
execution capability, timer, worker, or physical execution path.

## Branch and version

The `dev` branch was created from the unchanged post-release `main` commit
`ffb05289b718e2e488c25a4b1354e6e69fa38515`. The accepted uncommitted Phase 14
documentation was carried onto `dev`. Project, package, lockfile, and MCP server
version declarations now agree on unreleased `0.2.0`. The immutable `v0.1.0`
tag remains at `54caa8e387a5954d3d85d6f4c45157ea294b09ff`.

## Execution domain model

`ExecutionState` exposes these stable values:

- `pending`
- `validating`
- `starting`
- `running`
- `cancelling`
- `succeeded`
- `failed`
- `cancelled`
- `timed_out`

`ExecutionRecord` is frozen and contains only an opaque execution ID, trusted
plan ID, current state, optional stable `DomainFailure`, cancellation-requested
flag, and physical-stop-confirmation flag. Phase 15 requires that last flag to
remain false. No ROS goal handle, Future, action integer, MoveIt type, or
controller value exists in the model.

Legal transitions are:

```text
pending    -> validating | failed
validating -> starting | failed
starting   -> running | failed | timed_out
running    -> succeeded | failed | cancelling | timed_out
cancelling -> cancelled | failed | timed_out
```

The four terminal states cannot transition. Invalid transitions return
`invalid_execution_transition` from the registry. The broader state machine is
ready for later phases, but Phase 15 application preparation stops at
`validating`.

## ExecutionRegistry

`ExecutionRegistry` creates opaque high-entropy application IDs, holds one
active record, and retains a configurable bounded number of recent terminal
records. A second active request returns `execution_conflict`; it is neither
queued nor allowed to cancel the first operation. All record updates replace
immutable snapshots under a small in-process lock. Terminal transition releases
the single-active slot and trims only old terminal history.

No persistence, database, queue, background thread, or workflow framework was
introduced.

## Trusted-plan reservation

`PlanRegistry` now has private `available`, `reserved`, and `consumed` states.
The state is an application detail and is not exposed through MCP or domain
models.

`reserve_for_execution(plan_id, execution_id)` performs lookup, expiry check,
and ownership assignment as one synchronous, non-awaiting registry operation.
The application coordinator lock prevents competing preparation flows. A
reserved entry:

- cannot expire while reserved;
- cannot be selected by capacity eviction;
- cannot be discarded;
- cannot be reserved by another execution.

Release restores the unused TTL remaining when reservation began. This is the
future pre-goal-acceptance failure path. Consumption permanently blocks another
execution reservation. This is the future post-goal-acceptance path. Consumed
plans may remain temporarily inspectable until normal TTL/capacity removal, but
can never be executed again.

If every capacity slot is reserved, storing another plan fails instead of
evicting active execution evidence.

## Application preparation

`ManipulatorService.prepare_execution(plan_id)` is intentionally not an
execution method. It:

1. rejects while execution is disabled;
2. creates the one active execution ownership record;
3. atomically reserves the application-owned plan;
4. enters `validating`;
5. checks existing stored policy identity, group permission, and scene-revision
   presence through `SafetyEvaluator`;
6. releases the reservation and marks the record failed if stored policy
   validation rejects it.

It does not perform live state, start-state, exact current-scene, trajectory,
or MoveIt availability validation; those belong to later phases. It cannot
enter `starting` or `running`. `get_execution_status` and
`has_active_execution` support later orchestration and scene-mutation guards,
but are not registered with MCP.

## Configuration and timeout policy

The former v0.1 planning-only safety switch is replaced by typed `[execution]`
configuration. Defaults are project policy, not MoveIt standards:

| Setting | Default |
|---|---:|
| `enabled` | `false` |
| `start_state_tolerance_rad` | `0.02` |
| `state_freshness_sec` | `0.25` |
| `require_exact_scene_revision` | `true` |
| `action_server_discovery_timeout_sec` | `2.0` |
| `goal_acceptance_timeout_sec` | `2.0` |
| `cancel_timeout_sec` | `2.0` |
| `shutdown_timeout_sec` | `5.0` |
| `execution_timeout_multiplier` | `2.0` |
| `execution_timeout_margin_sec` | `2.0` |
| `execution_timeout_max_sec` | `30.0` |

Invalid numeric values are rejected, never clamped. The pure future outer-wait
calculation is:

```text
min(
  trajectory_duration * execution_timeout_multiplier
    + execution_timeout_margin_sec,
  execution_timeout_max_sec,
)
```

No timer or ROS wait uses these values in Phase 15. Production composition
still rejects `enabled=true` because no physical execution adapter exists yet.

## Stable errors

Five focused categories supplement the existing taxonomy:

- `execution_disabled`
- `execution_conflict`
- `plan_reserved`
- `plan_consumed`
- `invalid_execution_transition`

Expected ownership and state errors therefore remain structured. No backend
exception, ROS status, or MoveIt error code is introduced.

## Cancellation limitation

Phase 15 represents `running -> cancelling` and the later terminal outcomes
only. It sends no cancellation request. `cancellation_requested` records intent;
`cancelled` must not be described as independently confirmed physical
standstill. The unresolved exact MoveIt 2.12.4 cancellation-propagation audit
from Phase 14 remains a later mock-runtime gate.

## Tests

Six focused tests cover:

1. the complete cancellation-oriented legal state path and rejection after a
   terminal state;
2. disabled preparation leaving no plan reservation or active execution;
3. reserved-plan protection from TTL expiry, discard, and capacity eviction;
4. reversible release before acceptance and irreversible consumption after
   acceptance;
5. deterministic rejection of a second active preparation;
6. timeout formula, maximum bound, default-disabled setting, and invalid-input
   rejection.

These tests use no ROS or MoveIt mocks. Existing foundation and regression
tests provide the broader compatibility checks described below.

Verification results:

- Python compilation: passed;
- focused Phase 15 tests: `6 passed`;
- MCP regression suite outside the restricted sandbox: `15 passed`;
- complete permitted non-live regression selection: `45 passed, 7 deselected`;
  the six opt-in live integrations and the one test that deliberately constructs
  an rclpy runtime were excluded;
- initial sandbox-only diagnostics found that AnyIO's thread helper hung and a
  rclpy node could not enumerate sandboxed UDP interfaces. Neither issue
  reproduced in the permitted MCP rerun or caused a project workaround.

No ROS graph or hardware process was started. The attempted sandboxed lifecycle
check could not create its rclpy node and cleaned up through its failure path.

## Authoritative source audit

Phase 15 introduces no external protocol or ROS/MoveIt API. Its implementation
uses only Python 3.12 standard-library dataclasses, enums, locks, monotonic
in-memory ownership concepts, and existing project types. The approved values
come directly from the accepted Phase 15 instruction and are documented as
project policy.

The permanent baseline remains Ubuntu 24.04.4, Python 3.12.3, ROS 2 Jazzy,
`rclpy` 7.1.11, MoveIt 2.12.4, `moveit_msgs` 2.6.0, MCP 2026-07-28, MCP Python
SDK 2.0.0, and `mcp-types` 2.0.0. No new external assumption or discrepancy was
encountered.

## Deferred work

Phase 16 and later retain all live pre-execution validation, exact scene
comparison, start-state freshness/proximity checks, execution adapter port,
`ExecuteTrajectory` client, goal acceptance/consumption wiring, cancellation,
shutdown cleanup, ROS lifecycle, MCP Tools/Resources/Prompts, mock-hardware E2E,
and explicitly approved real-hardware execution.

No `ExecuteTrajectory` or `FollowJointTrajectory` code exists. No ROS or
hardware runtime was started, no torque/controller state changed, no trajectory
was sent, and no physical execution occurred.
