"""Backend-neutral application facade for manipulator use cases."""

from collections.abc import Awaitable, Callable
from time import perf_counter
from typing import TypeVar

from ros2_manipulator_mcp.application.plans import PlanRegistry
from ros2_manipulator_mcp.application.ports import (
    KinematicsPort,
    ManipulatorDescriptionPort,
    ManipulatorStatePort,
    MotionPlanningPort,
    PlanningScenePort,
)
from ros2_manipulator_mcp.domain import (
    CartesianPathRequest,
    CartesianPathResult,
    CollisionObject,
    DomainErrorCode,
    DomainFailure,
    DomainResult,
    JointGoal,
    ManipulatorDescriptor,
    MotionPlan,
    PlanningGroup,
    PlanningRequest,
    PlanningSceneSnapshot,
    Pose,
    PoseGoal,
    RobotState,
)
from ros2_manipulator_mcp.safety import SafetyDecision, SafetyEvaluator


T = TypeVar("T")


class ManipulatorService:
    """Orchestrate v0.1.0 manipulator operations through narrow ports."""

    def __init__(
        self,
        *,
        description: ManipulatorDescriptionPort,
        state: ManipulatorStatePort,
        kinematics: KinematicsPort,
        planning: MotionPlanningPort,
        scene: PlanningScenePort,
        plans: PlanRegistry | None = None,
        safety: SafetyEvaluator | None = None,
    ) -> None:
        self._description = description
        self._state = state
        self._kinematics = kinematics
        self._planning = planning
        self._scene = scene
        self._plans = plans or PlanRegistry()
        self._safety = safety or SafetyEvaluator()

    async def describe_manipulator(
        self,
    ) -> DomainResult[ManipulatorDescriptor]:
        """Return backend-neutral manipulator metadata."""
        return await self._call(self._description.describe_manipulator)

    async def list_planning_groups(
        self,
    ) -> DomainResult[tuple[PlanningGroup, ...]]:
        """Return planning groups from the manipulator description."""
        result = await self.describe_manipulator()
        if result.error is not None:
            return DomainResult(error=result.error)
        return DomainResult(value=result.value.groups)

    async def get_current_state(self) -> DomainResult[RobotState]:
        """Return the current robot state."""
        return await self._call(self._state.get_current_state)

    async def get_end_effector_pose(
        self,
        group_name: str,
        end_effector_link: str,
    ) -> DomainResult[Pose]:
        """Return the current end-effector pose."""
        return await self._call(
            lambda: self._state.get_end_effector_pose(
                group_name,
                end_effector_link,
            )
        )

    async def compute_forward_kinematics(
        self,
        group_name: str,
        target_link: str,
        state: RobotState,
    ) -> DomainResult[Pose]:
        """Compute forward kinematics."""
        return await self._call(
            lambda: self._kinematics.compute_forward_kinematics(
                group_name,
                target_link,
                state,
            )
        )

    async def compute_inverse_kinematics(
        self,
        group_name: str,
        target_link: str,
        target_pose: Pose,
        seed_state: RobotState | None = None,
    ) -> DomainResult[RobotState]:
        """Compute inverse kinematics."""
        return await self._call(
            lambda: self._kinematics.compute_inverse_kinematics(
                group_name,
                target_link,
                target_pose,
                seed_state,
            )
        )

    async def check_state_validity(
        self,
        group_name: str,
        state: RobotState,
    ) -> DomainResult[bool]:
        """Check whether a robot state is valid."""
        return await self._call(
            lambda: self._kinematics.check_state_validity(group_name, state)
        )

    async def plan_to_joint_goal(
        self,
        goal: JointGoal,
        *,
        start_state: RobotState | None = None,
        max_velocity_scaling_factor: float = 1.0,
        max_acceleration_scaling_factor: float = 1.0,
        allowed_planning_time_seconds: float = 5.0,
        planning_attempts: int = 1,
    ) -> DomainResult[MotionPlan]:
        """Plan to a joint goal and store the successful trajectory."""
        return await self.plan_motion(
            PlanningRequest(
                goal=goal,
                start_state=start_state,
                max_velocity_scaling_factor=max_velocity_scaling_factor,
                max_acceleration_scaling_factor=max_acceleration_scaling_factor,
                allowed_planning_time_seconds=allowed_planning_time_seconds,
                planning_attempts=planning_attempts,
            )
        )

    async def plan_to_pose_goal(
        self,
        goal: PoseGoal,
        *,
        start_state: RobotState | None = None,
        max_velocity_scaling_factor: float = 1.0,
        max_acceleration_scaling_factor: float = 1.0,
        allowed_planning_time_seconds: float = 5.0,
        planning_attempts: int = 1,
    ) -> DomainResult[MotionPlan]:
        """Plan to a pose goal and store the successful trajectory."""
        return await self.plan_motion(
            PlanningRequest(
                goal=goal,
                start_state=start_state,
                max_velocity_scaling_factor=max_velocity_scaling_factor,
                max_acceleration_scaling_factor=max_acceleration_scaling_factor,
                allowed_planning_time_seconds=allowed_planning_time_seconds,
                planning_attempts=planning_attempts,
            )
        )

    async def plan_motion(
        self,
        request: PlanningRequest,
    ) -> DomainResult[MotionPlan]:
        """Bind a backend trajectory to an application-owned plan ID."""
        decision = self._safety.evaluate_planning_request(request)
        if not decision.allowed:
            return self._policy_rejection(decision)

        scene_result = await self.get_planning_scene()
        if scene_result.error is not None:
            return DomainResult(error=scene_result.error)

        started = perf_counter()
        trajectory_result = await self._call(
            lambda: self._planning.plan_motion(request)
        )
        duration = perf_counter() - started
        if trajectory_result.error is not None:
            return DomainResult(error=trajectory_result.error)

        plan = self._plans.store(
            request=request,
            trajectory=trajectory_result.value,
            planning_duration_seconds=duration,
            scene_revision=scene_result.value.revision,
            policy_id=self._safety.policy.policy_id,
        )
        return DomainResult(value=plan)

    async def compute_cartesian_path(
        self,
        request: CartesianPathRequest,
    ) -> DomainResult[CartesianPathResult]:
        """Compute a Cartesian path without executing or storing it."""
        request_decision = self._safety.evaluate_cartesian_request(request)
        if not request_decision.allowed:
            return self._policy_rejection(request_decision)
        result = await self._call(
            lambda: self._planning.compute_cartesian_path(request)
        )
        if result.error is not None:
            return result
        result_decision = self._safety.evaluate_cartesian_result(result.value)
        if not result_decision.allowed:
            return self._policy_rejection(result_decision)
        return result

    def get_motion_plan(self, plan_id: str) -> DomainResult[MotionPlan]:
        """Retrieve an application-owned stored plan."""
        return self._plans.get(plan_id)

    def discard_motion_plan(self, plan_id: str) -> DomainResult[bool]:
        """Discard an application-owned stored plan."""
        return self._plans.discard(plan_id)

    def prepare_plan_validation(
        self,
        plan_id: str,
    ) -> DomainResult[SafetyDecision]:
        """Evaluate stored metadata available without a live backend."""
        context = self._plans.validation_context(plan_id)
        if context.error is not None:
            return DomainResult(error=context.error)
        plan, policy_id = context.value
        decision = self._safety.evaluate_stored_plan(
            plan,
            associated_policy_id=policy_id,
        )
        if not decision.allowed:
            return self._policy_rejection(decision)
        return DomainResult(value=decision)

    async def get_planning_scene(
        self,
    ) -> DomainResult[PlanningSceneSnapshot]:
        """Return the current planning scene."""
        return await self._call(self._scene.get_planning_scene)

    async def apply_collision_object(
        self,
        collision_object: CollisionObject,
        *,
        replace: bool = False,
    ) -> DomainResult[PlanningSceneSnapshot]:
        """Add or explicitly replace one primitive collision object."""
        scene_result = await self.get_planning_scene()
        if scene_result.error is not None:
            return DomainResult(error=scene_result.error)
        decision = self._safety.evaluate_collision_object(
            collision_object,
            scene_result.value,
            replace=replace,
        )
        if not decision.allowed:
            return self._policy_rejection(decision)
        return await self._call(
            lambda: self._scene.apply_collision_object(
                collision_object,
                replace=replace,
            )
        )

    async def remove_collision_object(
        self,
        object_id: str,
    ) -> DomainResult[PlanningSceneSnapshot]:
        """Remove one primitive collision object."""
        if not isinstance(object_id, str) or not object_id.strip():
            return DomainResult(
                error=DomainFailure(
                    DomainErrorCode.INVALID_REQUEST,
                    "object_id must be a non-empty string.",
                )
            )
        decision = self._safety.evaluate_collision_object_removal(object_id)
        if not decision.allowed:
            return self._policy_rejection(decision)
        return await self._call(
            lambda: self._scene.remove_collision_object(object_id)
        )

    @staticmethod
    def _policy_rejection(decision: SafetyDecision) -> DomainResult:
        """Translate a structured policy decision into an application error."""
        codes = ", ".join(finding.code for finding in decision.findings)
        return DomainResult(
            error=DomainFailure(
                DomainErrorCode.POLICY_REJECTED,
                f"Safety policy '{decision.policy_id}' rejected the operation: "
                f"{codes}.",
            )
        )

    @staticmethod
    async def _call(
        operation: Callable[[], Awaitable[DomainResult[T]]],
    ) -> DomainResult[T]:
        """Translate unexpected backend failures into stable domain results."""
        try:
            return await operation()
        except TimeoutError:
            return DomainResult(
                error=DomainFailure(
                    DomainErrorCode.TIMEOUT,
                    "The backend operation timed out.",
                )
            )
        except Exception as error:
            del error
            return DomainResult(
                error=DomainFailure(
                    DomainErrorCode.BACKEND_ERROR,
                    "The backend operation failed unexpectedly.",
                )
            )
