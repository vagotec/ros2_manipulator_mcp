"""Deterministic fake-backed stdio server used by Phase 10 E2E tests."""

from ros2_manipulator_mcp.application import ManipulatorService, PlanRegistry
from ros2_manipulator_mcp.domain import (
    CartesianPathResult,
    DomainResult,
    JointState,
    ManipulatorDescriptor,
    PlanningGroup,
    PlanningSceneSnapshot,
    RobotState,
    Trajectory,
    TrajectoryPoint,
)
from ros2_manipulator_mcp.safety import SafetyEvaluator, SafetyPolicy
from ros2_manipulator_mcp.server import create_server


class StdioFakeBackend:
    def __init__(self) -> None:
        group = PlanningGroup("test-group", ("joint-a",), ("base", "tool"), "base", "tool")
        self.descriptor = ManipulatorDescriptor("stdio-test-arm", "base", (group,))
        self.state = RobotState(JointState(("joint-a",), (0.0,)))
        self.scene = PlanningSceneSnapshot("stdio-scene", ())

    async def describe_manipulator(self):
        return DomainResult(value=self.descriptor)

    async def get_current_state(self):
        return DomainResult(value=self.state)

    async def get_end_effector_pose(self, group_name, end_effector_link):
        raise RuntimeError("private backend detail")

    async def compute_forward_kinematics(self, group_name, target_link, state):
        raise RuntimeError("unused")

    async def compute_inverse_kinematics(self, group_name, target_link, target_pose, seed_state=None):
        raise RuntimeError("unused")

    async def check_state_validity(self, group_name, state):
        return DomainResult(value=True)

    async def plan_motion(self, request):
        trajectory = Trajectory(("joint-a",), (TrajectoryPoint((0.1,), 0.0),))
        return DomainResult(value=trajectory)

    async def compute_cartesian_path(self, request):
        trajectory = Trajectory(("joint-a",), (TrajectoryPoint((0.1,), 0.0),))
        return DomainResult(value=CartesianPathResult(1.0, trajectory))

    async def get_planning_scene(self):
        return DomainResult(value=self.scene)

    async def apply_collision_object(self, collision_object, *, replace):
        return DomainResult(value=self.scene)

    async def remove_collision_object(self, object_id):
        return DomainResult(value=self.scene)


backend = StdioFakeBackend()
policy = SafetyPolicy(max_primitive_dimension=0.1)
service = ManipulatorService(
    description=backend,
    state=backend,
    kinematics=backend,
    planning=backend,
    scene=backend,
    plans=PlanRegistry(id_factory=lambda: "stdio-plan"),
    safety=SafetyEvaluator(policy),
)

create_server(service).run(transport="stdio")
