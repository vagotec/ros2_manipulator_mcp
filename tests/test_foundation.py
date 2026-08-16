"""Focused checks for the Phase 1 project foundation."""

import ros2_manipulator_mcp
from mcp.server import MCPServer

from ros2_manipulator_mcp.server import SERVER_NAME, create_server


def test_package_import_exposes_version() -> None:
    """The installed src-layout package imports with its release version."""
    assert ros2_manipulator_mcp.__version__ == "0.2.0"


def test_composition_root_constructs_server_without_ros() -> None:
    """Server construction loads configuration but starts no ROS runtime."""
    server = create_server()

    assert isinstance(server, MCPServer)
    assert server.name == SERVER_NAME
