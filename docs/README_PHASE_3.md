# Phase 3 - Application Layer and Backend Ports

## Goal

Implement the backend-neutral application use cases and adapter contracts for the approved v0.1.0 planning scope. This phase has no ROS, MoveIt, MCP registration, or physical execution behavior.

## Capability ports

Five async protocols divide backend responsibilities:

- `ManipulatorDescriptionPort`: manipulator metadata and planning groups
- `ManipulatorStatePort`: current state and end-effector pose
- `KinematicsPort`: FK, IK, and state validity
- `MotionPlanningPort`: motion planning and Cartesian paths
- `PlanningScenePort`: scene reads and primitive object add/replace/remove

Ports exchange domain models and Python identifiers only. They expose no backend clients, futures, nodes, messages, classes, or error codes.

## Application facade

`ManipulatorService` orchestrates discovery, state, kinematics, joint/pose planning, Cartesian paths, plan retrieval/discard, validation preparation, scene reads, and primitive scene mutations. Awaited backend calls translate timeouts and unexpected failures into stable domain failures.

Successful motion planning follows this path:

```text
read scene revision
 -> request backend trajectory
 -> generate application-owned opaque ID
 -> create immutable MotionPlan
 -> store bounded in-memory entry
```

No application method accepts a trajectory for storage. Consequently, an arbitrary caller-supplied trajectory cannot be promoted into a public stored plan through the service.

## Plan registry

`PlanRegistry` provides retrieval and discard by opaque ID. It has a configurable positive maximum count and optional positive TTL. Expired entries are removed on registry access, and inserting beyond the bound evicts the oldest entry. Storage is process-local and intentionally has no persistence layer.

Expiry timestamps are application-only metadata; they do not mutate the frozen `MotionPlan` domain model. The default is 32 plans with a five-minute lifetime.

## Safety boundary

Scene mutation remains in explicit service methods, providing one application boundary where the dedicated safety phase can insert policy checks. Phase 3 implements no safety evaluator and no execution path.

## Validation

```bash
uv run pytest -q tests/test_foundation.py tests/domain tests/application
uv run python -m compileall -q src tests
```

Five application tests cover read delegation and exception translation, application-owned plan creation, missing/discarded IDs, bounded registry eviction, and typed scene-mutation delegation.

## Deferred

- safety evaluator and scene mutation policy
- ROS 2 Jazzy and MoveIt 2 adapter
- conversions to ROS messages
- MCP Tools, Resources, and Prompts
- execution models and physical trajectory execution
- persistence, background tasks, queues, and databases

## Next phase

Phase 4 must be separately approved before safety and validation behavior is implemented.
