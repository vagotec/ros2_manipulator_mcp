# Phase 2 - Backend-neutral Manipulator Domain

## Goal

Define only the immutable, backend-neutral models needed by the approved public v0.1.0 planning scope. The domain uses the Python standard library and has no MCP, ROS, or MoveIt dependency.

## Models

- `geometry`: finite vectors, explicitly normalized quaternions, and framed poses
- `description`: manipulator descriptors and planning groups
- `state`: ordered joint state and timestamped robot state
- `goals`: joint and framed pose goals
- `trajectory`: timestamped points and immutable joint trajectories
- `planning`: joint/pose planning requests, motion plans, and Cartesian path requests/results
- `scene`: primitive collision objects and revisioned scene snapshots
- `results`: stable domain failure categories and typed success/failure results

## Invariants

Models reject empty identifiers, duplicate joint names, mismatched vectors, non-finite values, negative durations, invalid scaling factors, non-monotonic trajectories, and invalid primitive dimensions. Quaternions must already have a squared norm within `1e-6` of one; they are never silently normalized. Inputs are converted to immutable tuples where needed, but numeric values are never clamped.

Primitive dimension order is part of the domain contract:

- box: `(x, y, z)`
- sphere: `(radius,)`
- cylinder: `(height, radius)`
- cone: `(height, radius)`

## Architecture decisions

- Frozen dataclasses make plans, trajectories, state, and scene snapshots value objects.
- A `MotionPlan` carries an opaque ID, original request, trajectory, planning duration, and scene revision. Storage and expiry remain application concerns.
- Cartesian results expose completion fraction explicitly; a positive fraction requires a trajectory.
- Domain errors use stable string categories and never expose backend integer codes.
- Planning groups describe kinematic membership without representing controllers or hardware.

## Validation

```bash
uv run pytest -q tests/test_foundation.py tests/domain
uv run python -m compileall -q src tests
```

Six focused domain tests cover joint-data integrity, quaternion policy, planning scaling, monotonic timing, deep trajectory immutability, and collision primitive dimensions.

## Deferred

Phase 2 includes no application service, adapter protocol, safety evaluator, ROS/MoveIt conversion, MCP capability, execution model, attached object, mesh, advanced constraint, or planner selection.

## Next phase

Phase 3 must be separately approved before application logic or backend ports are implemented.
