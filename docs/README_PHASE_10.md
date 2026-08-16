# Phase 10 - Server Composition and MCP End-to-End

## Outcome

Phase 10 finalizes the v0.1.0 composition root and proves the real MCP stdio protocol path with a deterministic fake application backend. It adds no robotics capability.

```text
MCP ClientSession
 -> stdio JSON-RPC subprocess
 -> MCPServer 0.1.0
 -> Tool / Resource / Prompt
 -> ManipulatorService
 -> SafetyEvaluator where applicable
 -> fake port backend for deterministic E2E tests
```

The already accepted Phase 6 suite remains responsible for the real application-to-Jazzy/MoveIt path.

## Final composition

Importing the package and calling `create_server()` do not initialize rclpy or start ROS. `create_server()` deterministically loads immutable configuration, constructs `MCPServer`, advertises version `0.1.0`, and registers the accepted surface.

The stdio `main()` exclusively owns live composition:

1. load and validate configuration;
2. select the approved `ros2_jazzy_moveit` backend and OpenMANIPULATOR-X profile;
3. reject physical-execution configuration;
4. construct `JazzyMoveItSettings` and `JazzyManipulatorAdapter`;
5. explicitly construct `PlanRegistry`, `SafetyEvaluator`, and `ManipulatorService`;
6. create and run the MCP server using `transport="stdio"`;
7. close the adapter in `finally`.

No Streamable HTTP or SSE entry point is configured. The SDK supports those transports generically, but this project invokes only stdio.

## Server identity, instructions, and capabilities

Modern MCP discovery advertises:

- name: `ros2-manipulator-mcp`
- version: `0.1.0`
- Tools capability: present
- Resources capability: present
- Prompts capability: present
- server instructions: present

The instructions direct clients to use semantic Manipulator context, inspect before mutation, treat planning as separate from execution, and respect policy-controlled scene changes. They explicitly state that physical execution and arbitrary ROS access are unavailable and that validation is not certified physical safety.

Final approved surface:

| Capability | Count |
|---|---:|
| Tools | 19 |
| Static Resources | 7 |
| Resource Templates | 3 |
| Prompts | 6 |

## MCP 2026 lifecycle behavior

The MCP Specification 2026-07-28 removes the legacy `initialize` handshake. Its modern negotiation path is `server/discover` plus required per-request metadata. Installed `ClientSession.initialize()` deliberately offers only the latest legacy handshake version, `2025-11-25`; installed `ClientSession.discover()` negotiates `2026-07-28` and adopts the returned server metadata/capabilities.

Therefore Phase 10 uses `discover()` as the authoritative 2026 initialization/discovery path. Calling legacy `initialize()` would test a different protocol revision and would not satisfy the approved baseline.

The official stdio client spawns a subprocess, exchanges newline-delimited JSON-RPC over stdin/stdout, and performs bounded process shutdown. The server owns application/adapter cleanup around `MCPServer.run(transport="stdio")`.

## E2E tests

Three protocol tests use installed `StdioServerParameters`, `stdio_client`, and `ClientSession` against a subprocess running the real server with a deterministic fake application backend:

1. `server/discover` verifies protocol support, server identity/version, instructions, capabilities, and exact 19/7/3/6 listings.
2. `tools/call` verifies structured discovery success, distinguishable policy rejection with `isError=true`, and sanitized unexpected backend failure without its private exception detail.
3. `resources/read` and `prompts/get` verify JSON overview content, MIME type, and the execution-stopping safe workflow.

The tests combine representative paths instead of duplicating every Phase 7–9 registration test. No live MoveIt E2E test was added because it would duplicate Phase 6.

## Structured error behavior

Domain and policy failures retain their stable codes in `structuredContent`, including `policy_rejected`, and set MCP `isError=true`. A verified defect was corrected in `ManipulatorService._call`: unexpected backend exceptions previously copied `str(error)` into the public domain failure. They now return the stable generic message `The backend operation failed unexpectedly.` The original exception object and backend-private text do not cross the application or MCP boundary.

Input/schema errors, unknown MCP capability names, and transport/protocol failures remain SDK-level errors as required by MCP. Expected application outcomes remain visible tool results so a client can reason and self-correct.

## Authoritative source audit

Consulted MCP Specification 2026-07-28 sections for:

- lifecycle and `server/discover`
- per-request protocol/client metadata
- server identity, instructions, and capabilities
- stdio transport and shutdown
- Tools listing/calling and structured errors
- Resources listing/templates/reading
- Prompts listing/getting

Verified installed MCP Python SDK 2.0.0 APIs and source:

- `MCPServer(name, instructions, version)`
- `MCPServer.run(transport="stdio")`
- `StdioServerParameters`
- `stdio_client(...)`
- `ClientSession` async lifecycle
- `ClientSession.discover()` and `adopt(...)`
- `ClientSession.initialize()` legacy-only behavior
- `list_tools`, `call_tool`
- `list_resources`, `list_resource_templates`, `read_resource`
- `list_prompts`, `get_prompt`
- structured output-schema caching/validation and `isError`
- bounded stdio subprocess shutdown

Verified installed `mcp-types` 2.0.0 version registry and 2026-07-28 `DiscoverResult`, `ServerCapabilities`, listing, call, resource, and prompt models. The registry identifies `2026-07-28` as the sole modern version and `2025-11-25` as the latest handshake version.

SDK/spec discrepancies and observations:

- The user-facing phrase “initialize a session” maps to `server/discover` for the exact 2026-07-28 baseline. The installed `initialize()` method exists for compatibility but cannot negotiate 2026.
- No other semantic discrepancy affected Phase 10.
- Direct retrieval of the dated specification pages remained unavailable, so exact behavior was verified against the official installed generated bindings and SDK source.

No new ROS or MoveIt behavior was introduced. Existing rclpy 7.1.11 lifecycle ownership and Phase 5 adapter cleanup remain unchanged; no new ROS-side source was needed. Nothing required by Phase 10 remained unverifiable.

## Files

Created:

- `tests/mcp/stdio_test_server.py`
- `tests/mcp/test_stdio_e2e.py`
- `docs/README_PHASE_10.md`

Updated:

- `src/ros2_manipulator_mcp/server.py`
- `src/ros2_manipulator_mcp/application/service.py`
- `src/ros2_manipulator_mcp/mcp/instructions.py`
- `docs/README_PHASES.md`

The application change is the smallest correction for a verified public error-detail leak. Tools, Resources, Prompts, ports, domain models, safety evaluation, adapter behavior, and OpenMANIPULATOR-X integration were otherwise unchanged. No dependency changed.

## Limitations and exclusions

The deterministic MCP E2E subprocess uses a fake application backend; live MoveIt interoperability remains covered separately by the opt-in Phase 6 tests. `create_server()` without an injected service supports construction, discovery, listing, and static Prompt generation but application-backed reads/calls require the composed stdio runtime or explicit test injection.

No physical execution, ExecuteTrajectory, FollowJointTrajectory, controller management, arbitrary ROS access, new planning behavior, Servo, Hybrid Planning, Task Constructor, attach/detach, mesh, perception, navigation, or orchestration was added.

Phase 11 was not started.
