"""MCP-only input and structured result models."""

from typing import Any, Literal

from pydantic import BaseModel, ConfigDict, Field


class ToolModel(BaseModel):
    """Reject fields not declared by the public tool schema."""

    model_config = ConfigDict(extra="forbid")


class Vector3Input(ToolModel):
    x: float
    y: float
    z: float


class QuaternionInput(ToolModel):
    x: float
    y: float
    z: float
    w: float


class PoseInput(ToolModel):
    frame_id: str
    position: Vector3Input
    orientation: QuaternionInput


class JointStateInput(ToolModel):
    joint_names: list[str]
    positions: list[float]
    velocities: list[float] | None = None
    efforts: list[float] | None = None
    timestamp_seconds: float | None = None


class JointGoalInput(ToolModel):
    planning_group: str
    joint_names: list[str]
    positions: list[float]


class PoseGoalInput(ToolModel):
    planning_group: str
    target_link: str
    pose: PoseInput
    position_tolerance: float = 0.001
    orientation_tolerance: float = 0.01


class CollisionObjectInput(ToolModel):
    object_id: str
    primitive_type: Literal["box", "sphere", "cylinder", "cone"]
    dimensions: list[float]
    pose: PoseInput


class ToolResponse(BaseModel):
    """Stable public envelope for successful and failed tool calls."""

    success: bool
    status: Literal["success", "error"]
    error_code: str | None = None
    message: str
    data: dict[str, Any] | None = None
