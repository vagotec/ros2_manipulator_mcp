"""Focused tests for bounded read-only MCP Resources."""

import asyncio
import json

from ros2_manipulator_mcp.application import ManipulatorService, PlanRegistry
from ros2_manipulator_mcp.domain import (
    CollisionObject,
    CollisionPrimitive,
    DomainResult,
    JointGoal,
    JointState,
    ManipulatorDescriptor,
    PlanningGroup,
    PlanningSceneSnapshot,
    Pose,
    PrimitiveType,
    Quaternion,
    RobotState,
    Trajectory,
    TrajectoryPoint,
    Vector3,
)
from ros2_manipulator_mcp.server import create_server


class ReadOnlyBackend:
    def __init__(self) -> None:
        group = PlanningGroup("arm", ("joint1",), ("base", "tool"), "base", "tool")
        self.descriptor = ManipulatorDescriptor("test-arm", "base", (group,))
        self.state = RobotState(JointState(("joint1",), (0.0,)), timestamp_seconds=12.5)
        collision = CollisionObject(
            "fixture",
            CollisionPrimitive(PrimitiveType.BOX, (0.1, 0.2, 0.3)),
            Pose("base", Vector3(0, 0, 0), Quaternion(0, 0, 0, 1)),
        )
        self.scene = PlanningSceneSnapshot("scene-1", (collision,))
        self.planning_calls = 0
        self.mutation_calls = 0

    async def describe_manipulator(self):
        return DomainResult(value=self.descriptor)

    async def get_current_state(self):
        return DomainResult(value=self.state)

    async def get_planning_scene(self):
        return DomainResult(value=self.scene)

    async def plan_motion(self, request):
        self.planning_calls += 1
        trajectory = Trajectory(("joint1",), (TrajectoryPoint((0.2,), 0.0),))
        return DomainResult(value=trajectory)

    async def apply_collision_object(self, collision_object, *, replace):
        self.mutation_calls += 1
        return DomainResult(value=self.scene)

    async def remove_collision_object(self, object_id):
        self.mutation_calls += 1
        return DomainResult(value=self.scene)


def _fixture():
    backend = ReadOnlyBackend()
    registry = PlanRegistry(id_factory=lambda: "resource-plan")
    service = ManipulatorService(
        description=backend,
        state=backend,
        kinematics=backend,
        planning=backend,
        scene=backend,
        plans=registry,
    )
    return create_server(service), service, backend


def _read(server, uri: str):
    contents = asyncio.run(server.read_resource(uri))
    return json.loads(list(contents)[0].content)


def test_resource_and_template_registration_is_exact_and_bounded() -> None:
    server, _, _ = _fixture()
    resources = {str(value.uri) for value in asyncio.run(server.list_resources())}
    templates = {
        value.uri_template for value in asyncio.run(server.list_resource_templates())
    }

    assert resources == {
        "manipulator://overview", "manipulator://groups",
        "manipulator://state/current", "manipulator://scene",
        "manipulator://scene/objects", "manipulator://safety",
        "manipulator://health",
    }
    assert templates == {
        "manipulator://groups/{group}",
        "manipulator://scene/objects/{object_id}",
        "manipulator://plans/{plan_id}",
    }


def test_overview_and_state_are_bounded_read_only_application_projections() -> None:
    server, _, backend = _fixture()
    overview = _read(server, "manipulator://overview")
    state = _read(server, "manipulator://state/current")

    assert overview["data"]["manipulator_id"] == "test-arm"
    assert overview["data"]["physical_execution_available"] is False
    assert state["data"]["robot_state"]["freshness"] == "timestamp_available"
    assert backend.planning_calls == 0
    assert backend.mutation_calls == 0


def test_parameterized_object_and_plan_resources_resolve_summaries() -> None:
    server, service, backend = _fixture()
    asyncio.run(service.plan_to_joint_goal(JointGoal("arm", ("joint1",), (0.2,))))

    collision = _read(server, "manipulator://scene/objects/fixture")
    plan = _read(server, "manipulator://plans/resource-plan")

    assert collision["data"]["collision_object"]["object_id"] == "fixture"
    assert plan["data"]["motion_plan"]["trajectory"]["point_count"] == 1
    assert "points" not in plan["data"]["motion_plan"]["trajectory"]
    assert backend.planning_calls == 1
    assert backend.mutation_calls == 0


def test_safety_and_health_expose_limits_readiness_and_non_guarantees() -> None:
    server, _, backend = _fixture()
    safety = _read(server, "manipulator://safety")
    health = _read(server, "manipulator://health")

    assert safety["data"]["certified_physical_safety"] is False
    assert safety["data"]["physical_execution_available"] is False
    assert "NOT certified physical safety" in safety["data"]["non_guarantees"][0]
    assert health["data"]["status"] == "ready"
    assert health["data"]["polling"] is False
    assert health["data"]["service_level_readiness_probed"] is False
    assert backend.planning_calls == 0
    assert backend.mutation_calls == 0
