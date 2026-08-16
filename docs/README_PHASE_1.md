# Phase 1 - Project Foundation

## Goal

Establish the Python package, MCP stdio composition root, configuration loader, architecture boundaries, minimal documentation, and focused foundation tests. This phase implements no manipulator behavior and makes no ROS or MoveIt calls.

## Result

The project uses a `src/` layout, `uv`, `uv_build`, Python 3.12+, MCP Python SDK 2.x, and the `ros2-manipulator-mcp` console entry point. `create_server()` loads packaged configuration and constructs an empty `MCPServer` without starting ROS.

```text
create_server()
 -> load_settings()
 -> MCPServer

Future capability path:
MCP -> Application -> Domain -> Safety -> Port -> Jazzy/MoveIt adapter
```

## Architecture boundaries

- `domain`: backend-neutral manipulator concepts; currently empty.
- `application`: future use cases and plan management; currently empty.
- `application/ports`: future backend contracts; currently empty.
- `safety`: future deterministic policies and validation; currently empty.
- `mcp`: protocol registration and server instructions.
- `adapters/ros2_jazzy_moveit`: future explicit Jazzy action/service/topic integration; currently empty.
- `config`: packaged defaults and typed loading.

The domain and application layers will not expose MoveIt classes, ROS messages, ROS futures, or ROS node objects.

## Configuration

The packaged default selects the future `ros2_jazzy_moveit` backend, identifies OpenMANIPULATOR-X as the reference manipulator, provides a bounded service timeout, and explicitly disables physical execution. An alternate TOML file may be selected with `ROS2_MANIPULATOR_MCP_CONFIG`.

## Validation

```bash
uv run pytest -q tests/test_foundation.py
```

The two tests prove that the package imports with version `0.1.0` and that the composition root constructs the named MCP server without ROS.

## Deliberate limitations

- no registered MCP Tools, Resources, or Prompts
- no domain models or application use cases
- no ROS node or executor
- no MoveIt actions, services, or topics
- no public physical execution
- no HTTP transport
- no integration or E2E tests

## Next phase

Phase 2 must be separately approved before domain models, application ports, or manipulator functionality are implemented.
