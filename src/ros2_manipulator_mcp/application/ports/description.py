"""Manipulator description backend capability."""

from typing import Protocol

from ros2_manipulator_mcp.domain import DomainResult, ManipulatorDescriptor


class ManipulatorDescriptionPort(Protocol):
    """Provide backend-neutral manipulator metadata."""

    async def describe_manipulator(
        self,
    ) -> DomainResult[ManipulatorDescriptor]:
        """Return the available manipulator description."""
        ...
