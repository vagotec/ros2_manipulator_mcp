"""Focused application orchestration tests with one fake backend."""

import asyncio

from ros2_manipulator_mcp.application import ManipulatorService, PlanRegistry
from ros2_manipulator_mcp.domain import (
    CartesianPathRequest,
    CartesianPathResult,
    CollisionObject,
    CollisionPrimitive,
    DomainErrorCode,
    DomainResult,
    JointGoal,
    JointState,
    ManipulatorDescriptor,
    PlanningGroup,
    PlanningRequest,
    PlanningSceneSnapshot,
    Pose,
    PrimitiveType,
    Quaternion,
    RobotState,
    Trajectory,
    TrajectoryPoint,
    Vector3,
)


def _state() -> RobotState:
    return RobotState(JointState(("joint1",), (0.0,)))


def _trajectory(position: float = 0.5) -> Trajectory:
    return Trajectory(
        ("joint1",),
        (TrajectoryPoint((position,), 0.0),),
    )


def _request() -> PlanningRequest:
    return PlanningRequest(JointGoal("arm", ("joint1",), (0.5,)))


class FakeBackend:
    """Small fake implementing all five capability protocols."""

    def __init__(self) -> None:
        group = PlanningGroup(
            "arm",
            ("joint1",),
            ("base", "tool"),
            "base",
            "tool",
        )
        self.descriptor = ManipulatorDescriptor("test-arm", "base", (group,))
        self.scene = PlanningSceneSnapshot("scene-1", ())
        self.description_calls = 0
        self.planning_requests: list[PlanningRequest] = []
        self.applied_objects: list[tuple[CollisionObject, bool]] = []
        self.state_times_out = False

    async def describe_manipulator(self) -> DomainResult[ManipulatorDescriptor]:
        self.description_calls += 1
        return DomainResult(value=self.descriptor)

    async def get_current_state(self) -> DomainResult[RobotState]:
        if self.state_times_out:
            raise TimeoutError
        return DomainResult(value=_state())

    async def get_end_effector_pose(
        self, group_name: str, end_effector_link: str
    ) -> DomainResult[Pose]:
        return DomainResult(
            value=Pose("base", Vector3(0.0, 0.0, 0.0), Quaternion(0, 0, 0, 1))
        )

    async def compute_forward_kinematics(
        self, group_name: str, target_link: str, state: RobotState
    ) -> DomainResult[Pose]:
        return await self.get_end_effector_pose(group_name, target_link)

    async def compute_inverse_kinematics(
        self,
        group_name: str,
        target_link: str,
        target_pose: Pose,
        seed_state: RobotState | None = None,
    ) -> DomainResult[RobotState]:
        return DomainResult(value=seed_state or _state())

    async def check_state_validity(
        self, group_name: str, state: RobotState
    ) -> DomainResult[bool]:
        return DomainResult(value=True)

    async def plan_motion(
        self, request: PlanningRequest
    ) -> DomainResult[Trajectory]:
        self.planning_requests.append(request)
        return DomainResult(value=_trajectory())

    async def compute_cartesian_path(
        self, request: CartesianPathRequest
    ) -> DomainResult[CartesianPathResult]:
        return DomainResult(value=CartesianPathResult(1.0, _trajectory()))

    async def get_planning_scene(self) -> DomainResult[PlanningSceneSnapshot]:
        return DomainResult(value=self.scene)

    async def apply_collision_object(
        self, collision_object: CollisionObject, *, replace: bool
    ) -> DomainResult[PlanningSceneSnapshot]:
        self.applied_objects.append((collision_object, replace))
        return DomainResult(value=self.scene)

    async def remove_collision_object(
        self, object_id: str
    ) -> DomainResult[PlanningSceneSnapshot]:
        return DomainResult(value=self.scene)


def _service(
    backend: FakeBackend,
    plans: PlanRegistry | None = None,
) -> ManipulatorService:
    return ManipulatorService(
        description=backend,
        state=backend,
        kinematics=backend,
        planning=backend,
        scene=backend,
        plans=plans,
    )


def test_read_use_cases_delegate_and_translate_backend_timeout() -> None:
    """Read orchestration delegates once and contains backend exceptions."""
    backend = FakeBackend()
    service = _service(backend)

    description = asyncio.run(service.describe_manipulator())
    backend.state_times_out = True
    state = asyncio.run(service.get_current_state())

    assert description.value == backend.descriptor
    assert backend.description_calls == 1
    assert state.error.code is DomainErrorCode.TIMEOUT


def test_successful_planning_creates_and_stores_application_plan() -> None:
    """Only a backend-produced trajectory receives a public opaque plan ID."""
    backend = FakeBackend()
    registry = PlanRegistry(id_factory=lambda: "application-plan-id")
    service = _service(backend, registry)

    result = asyncio.run(service.plan_motion(_request()))
    retrieved = service.get_motion_plan("application-plan-id")

    assert result.value.plan_id == "application-plan-id"
    assert result.value.scene_revision == "scene-1"
    assert retrieved.value is result.value
    assert backend.planning_requests == [_request()]


def test_unknown_and_discarded_plan_ids_return_not_found() -> None:
    """Plan lifecycle failures have one stable application-visible category."""
    backend = FakeBackend()
    service = _service(
        backend,
        PlanRegistry(id_factory=lambda: "temporary-plan"),
    )
    asyncio.run(service.plan_motion(_request()))

    discarded = service.discard_motion_plan("temporary-plan")
    missing = service.get_motion_plan("temporary-plan")
    unknown = service.get_motion_plan("never-created")

    assert discarded.value is True
    assert missing.error.code is DomainErrorCode.NOT_FOUND
    assert unknown.error.code is DomainErrorCode.NOT_FOUND


def test_plan_registry_evicts_oldest_plan_at_configured_bound() -> None:
    """Unbounded planning cannot produce unbounded in-memory state."""
    ids = iter(("first", "second"))
    registry = PlanRegistry(max_plans=1, id_factory=lambda: next(ids))

    registry.store(
        request=_request(),
        trajectory=_trajectory(0.5),
        planning_duration_seconds=0.1,
        scene_revision="scene-1",
    )
    registry.store(
        request=_request(),
        trajectory=_trajectory(0.6),
        planning_duration_seconds=0.1,
        scene_revision="scene-1",
    )

    assert registry.get("first").error.code is DomainErrorCode.NOT_FOUND
    assert registry.get("second").value.plan_id == "second"


def test_scene_mutation_delegates_domain_object_and_replace_intent() -> None:
    """Scene writes remain a distinct typed port operation."""
    backend = FakeBackend()
    service = _service(backend)
    collision_object = CollisionObject(
        "workbench",
        CollisionPrimitive(PrimitiveType.BOX, (1.0, 0.5, 0.1)),
        Pose("base", Vector3(0.5, 0.0, 0.0), Quaternion(0, 0, 0, 1)),
    )

    result = asyncio.run(
        service.apply_collision_object(collision_object, replace=False)
    )

    assert result.value == backend.scene
    assert backend.applied_objects == [(collision_object, False)]
