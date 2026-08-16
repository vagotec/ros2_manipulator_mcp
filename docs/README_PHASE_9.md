# Phase 9 - MCP Prompts for v0.1.0

## Scope and behavior

Phase 9 adds six concise advisory workflow templates. Prompt generation is static apart from substituting caller-supplied text. It does not access `ManipulatorService`, SafetyEvaluator, adapters, ROS, MoveIt, Tools, or Resources.

Prompts tell the client which existing Resources to read and which Tools it may choose to call. They never call those capabilities themselves.

## Prompt catalog

| Prompt | Arguments | Purpose |
|---|---|---|
| `inspect_manipulator` | optional `focus` | Inspect overview, groups, current state, health, and scene without changing state |
| `diagnose_kinematics_failure` | required `failure`; optional `planning_group`, `target_link`, `frame` | Check group/link/frame/state/quaternion/tolerance/health assumptions before targeted FK or IK diagnostics |
| `diagnose_planning_failure` | required `failure`; optional `goal_summary`, `plan_id` | Review state, policy, group, scene, goal, and stored-plan context without automatic replanning |
| `review_motion_plan` | required `plan_id` | Review plan provenance, scene revision, policy, bounded trajectory metadata, and validation |
| `review_planning_scene_change` | required `intended_change`; optional `object_id` | Review primitive naming, dimensions, frame, replacement intent, and policy without applying it |
| `safe_manipulation_workflow` | required `goal_summary`; optional `planning_group` | Guide inspect → plan → review → validate → stop |

All arguments are semantic strings. Generic prompt text does not hard-code an OpenMANIPULATOR-X group, joint, frame, or link.

## Workflow and capability references

The prompts reference existing contextual URIs where relevant:

- `manipulator://overview`
- `manipulator://groups` and parameterized group context
- `manipulator://state/current`
- `manipulator://scene` and object context
- `manipulator://plans/{plan_id}`
- `manipulator://safety`
- `manipulator://health`

They may recommend the existing FK, IK, planning, stored-plan, validation, and scene Tools. The safe workflow inspects first, confirms group/state/scene/policy, requests plan-only planning, reviews the application-owned `plan_id`, validates it, and then explicitly stops.

No prompt recommends an execution Tool because none exists. Review and validation are explicitly described as not establishing certified physical safety.

## Prompt/Tool separation

Each registered function is an async pure text renderer. It accepts only prompt arguments and returns one user-role text message through the SDK conversion path. There is no injected MCP Context, service provider, application call, hidden Tool call, Resource read, mutation, planning request, thread, or background sequence.

The client or LLM decides whether to follow any recommendation and when to invoke a named Tool or read a Resource. Existing safety enforcement remains in `ManipulatorService`; prompts are not enforcement.

## Tests

Three focused tests verify:

1. the exact six-prompt registry and required/optional argument contracts;
2. the safe workflow names existing safety/validation context and explicitly stops before execution;
3. prompt generation works with `create_server()` and no application/ROS runtime.

Tests intentionally avoid wording-by-wording assertions and do not mock ROS or SDK internals.

## Authoritative source audit

Consulted MCP Specification 2026-07-28 Prompt semantics for `prompts/list`, `prompts/get`, prompt arguments, messages, roles, text content, and error behavior. The intended dated section is [Server Prompts](https://modelcontextprotocol.io/specification/2026-07-28/server/prompts).

Verified directly in installed MCP Python SDK 2.0.0 source:

- `MCPServer.prompt(name, title, description, icons)`
- `MCPServer.add_prompt(...)`
- `MCPServer.list_prompts()`
- `MCPServer.get_prompt(name, arguments, context)`
- `Prompt.from_function(...)` argument derivation
- required arguments from signature parameters without defaults
- `Prompt.render(...)` validation and async invocation
- string-to-user-role `TextContent` conversion
- `UserMessage`, `AssistantMessage`, and permitted roles

Verified in installed `mcp-types` 2.0.0 2026-07-28 models:

- `Prompt`
- `PromptArgument`
- `PromptMessage`
- `GetPromptResult`
- `TextContent`

No SDK/spec semantic discrepancy affected this phase. As in Phases 7–8, direct network retrieval of the dated specification page was unavailable; the official installed 2026-07-28 generated bindings and SDK source supplied the exact runtime authority. Nothing required by the implemented prompts remained unverifiable.

No ROS, MoveIt, or ROBOTIS interface was used or newly assumed.

## Files

Created:

- `src/ros2_manipulator_mcp/mcp/prompts/__init__.py`
- `src/ros2_manipulator_mcp/mcp/prompts/registration.py`
- `tests/mcp/test_prompts.py`
- `docs/README_PHASE_9.md`

Updated:

- `src/ros2_manipulator_mcp/server.py`
- `docs/README_PHASES.md`

No dependency changed. Phase 7 Tools and Phase 8 Resources were not modified.

## Limitations and exclusions

Prompts are advisory text, not orchestration, transactions, monitoring, or safety controls. They do not inspect live state during generation and cannot confirm that recommended context remains current.

There is no execution workflow or `diagnose_execution_failure` prompt. No physical execution, ExecuteTrajectory, controller management, arbitrary ROS access, new planning/backend operation, scene mutation, attach/detach, mesh, Servo, Hybrid Planning, Task Constructor, perception, navigation, or task orchestration was added.

Phase 10 was not started.
