# Phase 8 - MCP Resources and Health/Safety Context

## Scope and architecture

Phase 8 adds bounded, read-only MCP Resources that project existing application, domain, configuration, and safety-policy data. It introduces no backend call type, mutation, planning behavior, execution behavior, background task, Resource subscription, Prompt, or history store.

```text
MCP Resource
 -> ManipulatorService / effective SafetyPolicy
 -> existing application ports for live reads
 -> existing Jazzy adapter
```

Resource handlers never import or call ROS, MoveIt, or the adapter. Generic handlers contain no OpenMANIPULATOR-X group, joint, frame, or end-effector constants.

## Static Resources

| URI | Purpose | Important fields |
|---|---|---|
| `manipulator://overview` | Reusable manipulator context | identity, configured profile, model frame, groups, capability names, execution unavailable |
| `manipulator://groups` | Bounded planning-group context | group name, joints, links, base and end-effector link |
| `manipulator://state/current` | Current state summary | joints, positions, optional velocity/effort, source timestamp, timestamp availability |
| `manipulator://scene` | Current scene context | revision, object count, bounded primitive summaries |
| `manipulator://scene/objects` | Collision-object context | revision and primitive object summaries |
| `manipulator://safety` | Effective policy and non-guarantees | policy ID, planning/Cartesian/scene limits, execution unavailable |
| `manipulator://health` | On-demand readiness summary | descriptor/state checks, status, profile, timeout, no-polling flags |

All content uses `application/json` and a stable availability envelope containing `success`, `status`, and `data`; unavailable application reads also include stable `error_code` and `message` fields.

## Resource Templates

| RFC 6570 template | Parameter | Purpose |
|---|---|---|
| `manipulator://groups/{group}` | `group` | One group selected from `list_planning_groups()` |
| `manipulator://scene/objects/{object_id}` | `object_id` | One object selected from the current scene |
| `manipulator://plans/{plan_id}` | `plan_id` | One application-owned plan and current policy validation summary |

The installed SDK requires the template variable set to match handler parameters exactly. It percent-decodes matched variables and applies default protection against path traversal, absolute paths, and NUL bytes. Missing group, object, or plan identifiers produce bounded `not_found` content.

## Bounded-data decisions

- State contains only the current sample; there is no history.
- Scene data is bounded by the effective `SafetyPolicy.max_collision_objects` write limit and contains only supported primitive summaries.
- Plan content contains goal metadata, provenance, scaling, scene revision, planning duration, validation status, and trajectory joint names/count/duration. It does not contain trajectory points.
- `manipulator://plans` was deliberately deferred because `PlanRegistry` exposes safe lookup but no enumeration API. Phase 8 did not expand application architecture merely to support a Resource.
- Freshness does not infer wall-clock age from an arbitrary ROS clock. It reports whether the source supplied a timestamp.

## Health semantics

Health is evaluated only when read. It checks whether the application service is composed, whether the descriptor is obtainable, and whether a current state can be obtained through existing application methods. It reports `ready` only when both reads succeed; otherwise it reports `degraded` with stable domain errors.

The Resource reports the configured profile and service timeout. It explicitly reports `polling=false` and `service_level_readiness_probed=false`. It does not claim that every MoveIt service is available, retain the last error, launch monitoring threads, or provide metrics.

## Safety semantics and non-guarantees

`manipulator://safety` exposes the effective policy identity and configured planning, scaling, Cartesian, workspace, collision-object, primitive, and scene-revision limits. It reports physical execution as unavailable.

It explicitly states that this MCP policy layer is **not certified physical safety**. It makes no emergency-stop, machinery-compliance, guaranteed collision-avoidance, human-detection, or real-time torque/speed-enforcement claim. Resource metadata is context only and does not replace the actual `SafetyEvaluator` path used by Tools.

## Read-only guarantee

Resource registration receives only a `ManipulatorService` provider plus immutable settings/policy values. Handlers call these application reads:

- `describe_manipulator`
- `list_planning_groups`
- `get_current_state`
- `get_planning_scene`
- `get_motion_plan`
- `prepare_plan_validation`

No handler calls planning, plan discard, collision-object apply/remove, an adapter, ROS, MoveIt, or execution. Health has no background polling.

## Tests

Four focused tests verify:

1. the exact seven static Resources and three Resource Templates;
2. overview/state return bounded application projections without planning or mutation;
3. parameterized object and plan URIs resolve, with no raw trajectory points;
4. safety and health expose policy/non-guarantee/readiness semantics without planning or mutation.

No live MoveIt test was added because Phase 8 introduced no runtime dependency or backend behavior.

## Authoritative source audit

Consulted MCP Specification 2026-07-28 Resource semantics for `resources/list`, `resources/templates/list`, `resources/read`, URI templates, resource contents, MIME types, and security considerations. The intended dated section is [Server Resources](https://modelcontextprotocol.io/specification/2026-07-28/server/resources).

Verified directly in installed MCP Python SDK 2.0.0 source:

- `MCPServer.resource(uri, ...)`
- static-versus-template selection from parsed URI variables
- exact handler/template-variable matching
- `MCPServer.list_resources()`
- `MCPServer.list_resource_templates()`
- `MCPServer.read_resource()`
- `FunctionResource` JSON conversion and MIME propagation
- `ResourceTemplate` creation, matching, parameter validation, and secure defaults
- RFC 6570 parsing through the installed `UriTemplate`

Verified in installed `mcp-types` 2.0.0 2026-07-28 models:

- `Resource`
- `ResourceTemplate`
- `TextResourceContents` / `BlobResourceContents`
- `ReadResourceResult`

SDK/spec discrepancies and environment observations:

- No semantic mismatch was found between the installed 2026-07-28 bindings and the implemented Resource behavior.
- After a template handler resolves, SDK 2.0.0 captures its result in a synchronous lambda and `FunctionResource.read()` uses AnyIO's worker-thread path. The restricted filesystem/network sandbox did not schedule that path, while normal host execution passed all template tests. This is an execution-environment limitation, not application or template resolution behavior.
- As in Phase 7, direct network retrieval of the dated specification page was unavailable; exact behavior was cross-checked against the official installed generated protocol bindings and SDK source.

No new ROS, MoveIt, or ROBOTIS assumption was introduced, so no new ROS-side source was required. Nothing needed by the implemented Resources remained unverifiable.

## Files

Created:

- `src/ros2_manipulator_mcp/mcp/resources/__init__.py`
- `src/ros2_manipulator_mcp/mcp/resources/registration.py`
- `tests/mcp/test_resources.py`
- `docs/README_PHASE_8.md`

Updated:

- `src/ros2_manipulator_mcp/server.py`
- `docs/README_PHASES.md`

No dependency changed, and Phase 7 Tools were not modified.

## Explicit exclusions and deferrals

Deferred: bounded plan enumeration at `manipulator://plans`, live per-service readiness probing, last-failure history, and subscriptions. These would require application/runtime capabilities not present in the accepted architecture.

No physical execution, ExecuteTrajectory, controller management, arbitrary ROS capability, planning behavior, scene mutation, attach/detach, mesh, Servo, Hybrid Planning, Task Constructor, perception, navigation, orchestration, Resource subscription, or Prompt was added.

Phase 9 was not started.
