"""Focused tests for the Phase 7 MCP tool boundary."""

import asyncio

from ros2_manipulator_mcp.application import ManipulatorService, PlanRegistry
from ros2_manipulator_mcp.domain import (
    CartesianPathResult,
    DomainResult,
    JointState,
    ManipulatorDescriptor,
    PlanningGroup,
    PlanningSceneSnapshot,
    Pose,
    Quaternion,
    RobotState,
    Trajectory,
    TrajectoryPoint,
    Vector3,
)
from ros2_manipulator_mcp.safety import SafetyEvaluator, SafetyPolicy
from ros2_manipulator_mcp.server import create_server


class FakeBackend:
    def __init__(self) -> None:
        group = PlanningGroup("arm", ("joint1",), ("base", "tool"), "base", "tool")
        self.descriptor = ManipulatorDescriptor("test-arm", "base", (group,))
        self.state = RobotState(JointState(("joint1",), (0.0,)))
        self.scene = PlanningSceneSnapshot("scene-1", ())
        self.apply_calls = 0

    async def describe_manipulator(self):
        return DomainResult(value=self.descriptor)

    async def get_current_state(self):
        return DomainResult(value=self.state)

    async def get_end_effector_pose(self, group_name, end_effector_link):
        return DomainResult(value=Pose("base", Vector3(0, 0, 0), Quaternion(0, 0, 0, 1)))

    async def compute_forward_kinematics(self, group_name, target_link, state):
        return await self.get_end_effector_pose(group_name, target_link)

    async def compute_inverse_kinematics(self, group_name, target_link, target_pose, seed_state=None):
        return DomainResult(value=seed_state or self.state)

    async def check_state_validity(self, group_name, state):
        return DomainResult(value=True)

    async def plan_motion(self, request):
        return DomainResult(value=Trajectory(("joint1",), (TrajectoryPoint((0.2,), 0.0),)))

    async def compute_cartesian_path(self, request):
        trajectory = Trajectory(("joint1",), (TrajectoryPoint((0.2,), 0.0),))
        return DomainResult(value=CartesianPathResult(1.0, trajectory))

    async def get_planning_scene(self):
        return DomainResult(value=self.scene)

    async def apply_collision_object(self, collision_object, *, replace):
        self.apply_calls += 1
        return DomainResult(value=self.scene)

    async def remove_collision_object(self, object_id):
        return DomainResult(value=self.scene)


def _server(*, max_dimension: float = 5.0):
    backend = FakeBackend()
    service = ManipulatorService(
        description=backend,
        state=backend,
        kinematics=backend,
        planning=backend,
        scene=backend,
        plans=PlanRegistry(id_factory=lambda: "server-plan-id"),
        safety=SafetyEvaluator(SafetyPolicy(max_primitive_dimension=max_dimension)),
    )
    return create_server(service), backend


def test_registration_exposes_only_approved_tools() -> None:
    server, _ = _server()
    names = {tool.name for tool in asyncio.run(server.list_tools())}

    assert names == {
        "describe_manipulator", "list_planning_groups", "get_planning_group",
        "get_current_robot_state", "get_end_effector_pose",
        "compute_forward_kinematics", "compute_inverse_kinematics",
        "check_robot_state_validity", "plan_to_joint_goal",
        "plan_to_pose_goal", "compute_cartesian_path", "get_motion_plan",
        "discard_motion_plan", "validate_motion_plan", "get_planning_scene",
        "list_collision_objects", "get_collision_object",
        "apply_collision_object", "remove_collision_object",
        "execute_motion_plan", "get_execution_status", "cancel_execution",
    }
    assert "execute_trajectory" not in names
    assert not any(name.startswith("move_") for name in names)


def test_discovery_tool_returns_structured_application_data() -> None:
    server, _ = _server()
    result = asyncio.run(server.call_tool("describe_manipulator", {}))

    assert result.is_error is not True
    assert result.structured_content["success"] is True
    assert result.structured_content["data"]["manipulator_id"] == "test-arm"


def test_planning_tool_returns_server_owned_plan_id_and_summary() -> None:
    server, _ = _server()
    result = asyncio.run(server.call_tool("plan_to_joint_goal", {
        "goal": {"planning_group": "arm", "joint_names": ["joint1"], "positions": [0.2]}
    }))

    data = result.structured_content["data"]
    assert data["plan_id"] == "server-plan-id"
    assert data["trajectory"] == {
        "joint_names": ["joint1"], "point_count": 1, "duration_seconds": 0.0
    }


def test_policy_rejection_is_a_stable_structured_tool_error() -> None:
    server, backend = _server(max_dimension=0.1)
    result = asyncio.run(server.call_tool("apply_collision_object", {
        "collision_object": {
            "object_id": "box", "primitive_type": "box",
            "dimensions": [0.2, 0.2, 0.2],
            "pose": {
                "frame_id": "base", "position": {"x": 0, "y": 0, "z": 0},
                "orientation": {"x": 0, "y": 0, "z": 0, "w": 1},
            },
        }
    }))

    assert result.is_error is True
    assert result.structured_content["error_code"] == "policy_rejected"
    assert backend.apply_calls == 0


def test_allowed_scene_mutation_reaches_backend_only_through_service() -> None:
    server, backend = _server()
    result = asyncio.run(server.call_tool("apply_collision_object", {
        "collision_object": {
            "object_id": "box", "primitive_type": "box",
            "dimensions": [0.05, 0.05, 0.05],
            "pose": {
                "frame_id": "base", "position": {"x": 0, "y": 0, "z": 0},
                "orientation": {"x": 0, "y": 0, "z": 0, "w": 1},
            },
        }
    }))

    assert result.structured_content["success"] is True
    assert backend.apply_calls == 1
