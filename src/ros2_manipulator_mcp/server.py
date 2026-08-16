"""Composition root and stdio entry point."""

from mcp.server import MCPServer

from ros2_manipulator_mcp import __version__
from ros2_manipulator_mcp.application import ManipulatorService, PlanRegistry
from ros2_manipulator_mcp.config.settings import load_settings
from ros2_manipulator_mcp.mcp.instructions import SERVER_INSTRUCTIONS
from ros2_manipulator_mcp.mcp.prompts import register_prompts
from ros2_manipulator_mcp.mcp.resources import register_resources
from ros2_manipulator_mcp.mcp.tools import register_tools


SERVER_NAME = "ros2-manipulator-mcp"


def create_server(service: ManipulatorService | None = None) -> MCPServer:
    """Create the MCP server without constructing ROS resources."""
    settings = load_settings()

    server = MCPServer(
        SERVER_NAME,
        instructions=SERVER_INSTRUCTIONS,
        version=__version__,
    )
    register_tools(server, lambda: _require_service(service))
    register_resources(
        server,
        lambda: _require_service(service),
        policy=settings.policy,
        configured_profile=settings.runtime.reference_manipulator,
        service_timeout_seconds=settings.runtime.service_timeout_seconds,
    )
    register_prompts(server)
    return server


def main() -> None:
    """Run the MCP server over the v0.1.0 stdio transport."""
    service, adapter = _build_default_service()
    try:
        create_server(service).run(transport="stdio")
    finally:
        adapter.close()


def _require_service(service: ManipulatorService | None) -> ManipulatorService:
    if service is None:
        raise RuntimeError("Manipulator application runtime is not active.")
    return service


def _build_default_service():
    """Construct the configured ROS application only when stdio starts."""
    from ros2_manipulator_mcp.profiles import OPEN_MANIPULATOR_X_DESCRIPTOR
    from ros2_manipulator_mcp.ros.jazzy import (
        JazzyManipulatorAdapter,
        JazzyMoveItSettings,
    )
    from ros2_manipulator_mcp.safety import SafetyEvaluator

    settings = load_settings()
    if settings.runtime.backend != "ros2_jazzy_moveit":
        raise ValueError("Unsupported manipulator backend configuration.")
    if settings.runtime.reference_manipulator != "open_manipulator_x":
        raise ValueError("Unsupported reference manipulator configuration.")
    if settings.safety.physical_execution_enabled:
        raise ValueError("Physical execution is not available in v0.1.0.")

    adapter = JazzyManipulatorAdapter(
        OPEN_MANIPULATOR_X_DESCRIPTOR,
        settings=JazzyMoveItSettings(
            service_timeout_seconds=settings.runtime.service_timeout_seconds,
        ),
    )
    service = ManipulatorService(
        description=adapter,
        state=adapter,
        kinematics=adapter,
        planning=adapter,
        scene=adapter,
        plans=PlanRegistry(),
        safety=SafetyEvaluator(settings.policy),
    )
    return service, adapter


if __name__ == "__main__":
    main()
