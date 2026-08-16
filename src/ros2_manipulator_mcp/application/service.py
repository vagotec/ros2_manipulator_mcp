"""Backend-neutral application facade for manipulator use cases."""

import asyncio
from collections.abc import Awaitable, Callable
from math import isfinite
from threading import RLock
from time import monotonic, perf_counter
from typing import TypeVar

from ros2_manipulator_mcp.application.executions import (
    ExecutionRegistry,
    calculate_execution_timeout,
)
from ros2_manipulator_mcp.application.plans import PlanRegistry
from ros2_manipulator_mcp.application.ports import (
    KinematicsPort,
    BackendExecutionOutcome,
    BackendExecutionResult,
    ExecutionPort,
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
    ExecutionRecord,
    ExecutionState,
    ExecutionValidationFinding,
    ExecutionValidationResult,
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
    """Orchestrate manipulator operations through narrow application ports."""

    def __init__(
        self,
        *,
        description: ManipulatorDescriptionPort,
        state: ManipulatorStatePort,
        kinematics: KinematicsPort,
        planning: MotionPlanningPort,
        scene: PlanningScenePort,
        execution: ExecutionPort | None = None,
        plans: PlanRegistry | None = None,
        executions: ExecutionRegistry | None = None,
        execution_enabled: bool = False,
        start_state_tolerance_rad: float = 0.02,
        state_freshness_seconds: float = 0.25,
        require_exact_scene_revision: bool = True,
        action_server_discovery_timeout_seconds: float = 2.0,
        goal_acceptance_timeout_seconds: float = 2.0,
        execution_timeout_multiplier: float = 2.0,
        execution_timeout_margin_seconds: float = 2.0,
        execution_timeout_max_seconds: float = 30.0,
        cancellation_timeout_seconds: float = 5.0,
        stabilization_timeout_seconds: float = 2.0,
        stabilization_quiet_window_seconds: float = 0.5,
        stabilization_position_delta_rad: float = 0.001,
        stabilization_velocity_rad_per_sec: float = 0.01,
        safety: SafetyEvaluator | None = None,
    ) -> None:
        self._description = description
        self._state = state
        self._kinematics = kinematics
        self._planning = planning
        self._scene = scene
        self._execution = execution
        self._plans = plans or PlanRegistry()
        self._executions = executions or ExecutionRegistry()
        self._execution_enabled = execution_enabled
        if not isfinite(start_state_tolerance_rad) or start_state_tolerance_rad <= 0:
            raise ValueError(
                "start_state_tolerance_rad must be finite and greater than zero"
            )
        if not isfinite(state_freshness_seconds) or state_freshness_seconds <= 0:
            raise ValueError(
                "state_freshness_seconds must be finite and greater than zero"
            )
        self._start_state_tolerance_rad = float(start_state_tolerance_rad)
        self._state_freshness_seconds = float(state_freshness_seconds)
        self._require_exact_scene_revision = require_exact_scene_revision
        self._action_server_discovery_timeout_seconds = action_server_discovery_timeout_seconds
        self._goal_acceptance_timeout_seconds = goal_acceptance_timeout_seconds
        self._execution_timeout_multiplier = execution_timeout_multiplier
        self._execution_timeout_margin_seconds = execution_timeout_margin_seconds
        self._execution_timeout_max_seconds = execution_timeout_max_seconds
        self._cancellation_timeout_seconds = cancellation_timeout_seconds
        self._stabilization_timeout_seconds = stabilization_timeout_seconds
        self._stabilization_quiet_window_seconds = stabilization_quiet_window_seconds
        self._stabilization_position_delta_rad = stabilization_position_delta_rad
        self._stabilization_velocity_rad_per_sec = stabilization_velocity_rad_per_sec
        execution_limits = (
            action_server_discovery_timeout_seconds,
            goal_acceptance_timeout_seconds,
            execution_timeout_multiplier,
            execution_timeout_max_seconds,
            cancellation_timeout_seconds,
            stabilization_timeout_seconds,
            stabilization_quiet_window_seconds,
            stabilization_position_delta_rad,
            stabilization_velocity_rad_per_sec,
        )
        if any(not isfinite(value) or value <= 0 for value in execution_limits):
            raise ValueError("execution limits must be finite and positive")
        if not isfinite(execution_timeout_margin_seconds) or execution_timeout_margin_seconds < 0:
            raise ValueError("execution timeout margin must be finite and non-negative")
        self._execution_quarantined = False
        self._execution_lock = RLock()
        self._execution_tasks: dict[str, asyncio.Task[DomainResult[ExecutionRecord]]] = {}
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

    def prepare_execution(
        self,
        plan_id: str,
    ) -> DomainResult[ExecutionRecord]:
        """Reserve a trusted plan and enter validation without executing it."""
        if not self._execution_enabled:
            return DomainResult(
                error=DomainFailure(
                    DomainErrorCode.EXECUTION_DISABLED,
                    "Physical execution is disabled by configuration.",
                )
            )

        with self._execution_lock:
            created = self._executions.create(plan_id)
            if created.error is not None:
                return DomainResult(error=created.error)
            execution = created.value
            reserved = self._plans.reserve_for_execution(
                plan_id,
                execution.execution_id,
            )
            if reserved.error is not None:
                self._executions.transition(
                    execution.execution_id,
                    ExecutionState.FAILED,
                    failure=reserved.error,
                )
                return DomainResult(error=reserved.error)

            validating = self._executions.transition(
                execution.execution_id,
                ExecutionState.VALIDATING,
            )
            return validating

    async def validate_execution(
        self,
        execution_id: str,
    ) -> DomainResult[ExecutionValidationResult]:
        """Revalidate one reserved plan and stop before adapter submission."""
        execution_result = self._executions.get(execution_id)
        if execution_result.error is not None:
            return DomainResult(error=execution_result.error)
        execution = execution_result.value
        if execution.state is not ExecutionState.VALIDATING:
            return DomainResult(
                error=DomainFailure(
                    DomainErrorCode.INVALID_EXECUTION_TRANSITION,
                    "Execution must be validating before pre-execution checks.",
                )
            )

        findings: list[ExecutionValidationFinding] = []
        if not self._execution_enabled:
            findings.append(self._finding(
                "EXECUTION_DISABLED",
                "Physical execution is disabled by configuration.",
            ))
        if not self._executions.owns_active_execution(execution_id):
            findings.append(self._finding(
                "EXECUTION_OWNERSHIP_CONFLICT",
                "The execution does not own the active execution slot.",
            ))

        context = self._plans.execution_context(execution.plan_id, execution_id)
        if context.error is not None:
            findings.append(self._finding(
                "PLAN_RESERVATION_INVALID",
                "The plan is not reserved by this execution.",
            ))
            return self._reject_execution(execution, findings)
        plan, associated_policy_id = context.value

        stored_decision = self._safety.evaluate_stored_plan(
            plan,
            associated_policy_id=associated_policy_id,
        )
        request_decision = self._safety.evaluate_planning_request(plan.request)
        for decision in (stored_decision, request_decision):
            findings.extend(
                self._finding(item.code, item.message)
                for item in decision.findings
                if item.severity.value == "error"
            )
        if findings:
            return self._reject_execution(execution, findings)

        descriptor_result = await self.describe_manipulator()
        if descriptor_result.error is not None:
            findings.append(self._finding(
                "BACKEND_DESCRIPTION_UNAVAILABLE",
                "The manipulator description could not be read.",
            ))
            return self._reject_execution(execution, findings)
        group_name = plan.request.goal.group_name
        group = next(
            (item for item in descriptor_result.value.groups if item.name == group_name),
            None,
        )
        if group is None:
            findings.append(self._finding(
                "PLANNING_GROUP_UNAVAILABLE",
                "The stored planning group is not available from the backend.",
            ))
            return self._reject_execution(execution, findings)

        state_result = await self.get_current_state()
        if state_result.error is not None:
            findings.append(self._finding(
                "CURRENT_STATE_UNAVAILABLE",
                "The current robot state could not be read.",
            ))
            return self._reject_execution(execution, findings)
        state = state_result.value
        age = state.sample_age_seconds
        if age is None:
            findings.append(self._finding(
                "STATE_FRESHNESS_UNKNOWN",
                "The current robot-state age is unavailable.",
            ))
        elif age > self._state_freshness_seconds:
            findings.append(self._finding(
                "STATE_STALE",
                "The current robot state exceeds the configured maximum age.",
            ))

        trajectory_positions = dict(
            zip(plan.trajectory.joint_names, plan.trajectory.points[0].positions)
        )
        measured_positions = dict(
            zip(state.joints.joint_names, state.joints.positions)
        )
        start_state_matches = True
        for joint_name in group.joint_names:
            expected = trajectory_positions.get(joint_name)
            measured = measured_positions.get(joint_name)
            if expected is None:
                start_state_matches = False
                findings.append(self._finding(
                    "TRAJECTORY_START_JOINT_MISSING",
                    "A required planning-group joint is absent from the trajectory start.",
                    joint_name=joint_name,
                ))
                continue
            if measured is None:
                start_state_matches = False
                findings.append(self._finding(
                    "MEASURED_JOINT_MISSING",
                    "A required planning-group joint is absent from current state.",
                    joint_name=joint_name,
                    expected_position=expected,
                    tolerance=self._start_state_tolerance_rad,
                ))
                continue
            deviation = abs(measured - expected)
            if deviation > self._start_state_tolerance_rad:
                start_state_matches = False
                findings.append(self._finding(
                    "START_STATE_MISMATCH",
                    "A measured joint differs from the planned trajectory start.",
                    joint_name=joint_name,
                    expected_position=expected,
                    measured_position=measured,
                    absolute_deviation=deviation,
                    tolerance=self._start_state_tolerance_rad,
                ))

        scene_result = await self.get_planning_scene()
        if scene_result.error is not None:
            findings.append(self._finding(
                "PLANNING_SCENE_UNAVAILABLE",
                "The current planning scene could not be read.",
            ))
            return self._reject_execution(
                execution,
                findings,
                state_age_seconds=age,
                start_state_matches=start_state_matches,
            )
        scene_matches = scene_result.value.revision == plan.scene_revision
        if self._require_exact_scene_revision and not scene_matches:
            findings.append(self._finding(
                "SCENE_REVISION_MISMATCH",
                "The current planning-scene revision differs from the planned revision.",
            ))

        result = ExecutionValidationResult(
            execution_id=execution.execution_id,
            plan_id=execution.plan_id,
            policy_id=self._safety.policy.policy_id,
            findings=tuple(findings),
            state_age_seconds=age,
            start_state_matches=start_state_matches,
            scene_revision_matches=scene_matches,
        )
        if findings:
            return self._reject_execution(
                execution,
                findings,
                state_age_seconds=age,
                start_state_matches=start_state_matches,
                scene_revision_matches=scene_matches,
            )
        transitioned = self._executions.transition(
            execution_id,
            ExecutionState.STARTING,
        )
        if transitioned.error is not None:
            self._plans.release_reservation(execution.plan_id, execution_id)
            return DomainResult(error=transitioned.error)
        return DomainResult(value=result)

    def get_execution_status(
        self,
        execution_id: str,
    ) -> DomainResult[ExecutionRecord]:
        """Return application-owned preparation/execution state."""
        return self._executions.get(execution_id)

    async def execute_motion_plan(self, plan_id: str) -> DomainResult[ExecutionRecord]:
        """Validate and execute one immutable application-owned plan."""
        started = await self.start_motion_plan_execution(plan_id)
        if started.error is not None:
            return started
        task = self._execution_tasks[started.value.execution_id]
        return await task

    async def start_motion_plan_execution(
        self,
        plan_id: str,
    ) -> DomainResult[ExecutionRecord]:
        """Start the trusted execution workflow and return its observable record."""
        if self._execution is None:
            return DomainResult(error=DomainFailure(
                DomainErrorCode.EXECUTION_DISABLED,
                "The execution backend is not configured.",
            ))
        if self._execution_quarantined:
            return self._quarantine_failure()
        prepared = self.prepare_execution(plan_id)
        if prepared.error is not None:
            return prepared
        execution = prepared.value
        self._execution_tasks = {
            key: task for key, task in self._execution_tasks.items() if not task.done()
        }
        self._execution_tasks[execution.execution_id] = asyncio.create_task(
            self._run_execution(plan_id, execution.execution_id)
        )
        await asyncio.sleep(0)
        return self._executions.get(execution.execution_id)

    async def _run_execution(
        self,
        plan_id: str,
        execution_id: str,
    ) -> DomainResult[ExecutionRecord]:
        """Run validation, submission, observation, and terminal classification."""
        validation = await self.validate_execution(execution_id)
        status = self._executions.get(execution_id).value
        if validation.error is not None or not validation.value.valid:
            return DomainResult(value=status)
        context = self._plans.execution_context(plan_id, execution_id)
        if context.error is not None:
            return self._fail_before_acceptance(status, context.error)
        plan, _ = context.value
        submission = await self._call(lambda: self._execution.submit_execution(
            execution_id,
            plan.trajectory,
            server_timeout_seconds=self._action_server_discovery_timeout_seconds,
            acceptance_timeout_seconds=self._goal_acceptance_timeout_seconds,
        ))
        if submission.error is not None or not submission.value:
            failure = submission.error or DomainFailure(
                DomainErrorCode.EXECUTION_FAILED,
                "MoveIt rejected the execution goal.",
            )
            return self._fail_before_acceptance(status, failure)

        self._executions.update_cancellation_status(
            execution_id, backend_goal_accepted=True
        )

        consumed = self._plans.consume_reservation(plan_id, execution_id)
        if consumed.error is not None:
            self._enter_quarantine(execution_id)
            return self._terminal_failure(execution_id, consumed.error)

        with self._execution_lock:
            current = self._executions.get(execution_id).value
            if current.state is ExecutionState.STARTING:
                current = self._executions.transition(
                    execution_id, ExecutionState.RUNNING
                ).value
            cancellation_latched = current.state is ExecutionState.CANCELLING
        if cancellation_latched:
            stop = await self._dispatch_stop_once(execution_id)
            if stop.error is not None:
                failure = DomainFailure(
                    DomainErrorCode.BACKEND_QUARANTINED,
                    "The backend stop request could not be dispatched.",
                )
                return self._transition_terminal(
                    execution_id, ExecutionState.FAILED, failure
                )

        normal_timeout = calculate_execution_timeout(
            plan.trajectory.points[-1].time_from_start_seconds,
            multiplier=self._execution_timeout_multiplier,
            margin_seconds=self._execution_timeout_margin_seconds,
            maximum_seconds=self._execution_timeout_max_seconds,
        )
        terminal = await self._wait_for_execution_terminal(
            execution_id,
            normal_timeout_seconds=normal_timeout,
            cancellation_latched=cancellation_latched,
        )
        if terminal.error is not None:
            if terminal.error.code is DomainErrorCode.TIMEOUT:
                current = self._executions.get(execution_id)
                if (
                    current.error is None
                    and current.value.state in {
                        ExecutionState.STARTING,
                        ExecutionState.RUNNING,
                    }
                ):
                    self._executions.transition(
                        execution_id, ExecutionState.CANCELLING
                    )
                await self._dispatch_stop_once(execution_id)
                self._enter_quarantine(execution_id)
                return self._transition_terminal(
                    execution_id, ExecutionState.TIMED_OUT, terminal.error
                )
            return self._transition_terminal(
                execution_id, ExecutionState.FAILED, terminal.error
            )

        self._executions.update_cancellation_status(
            execution_id, execution_terminal=True
        )
        current = self._executions.get(execution_id).value
        outcome = terminal.value.outcome
        if outcome is BackendExecutionOutcome.SUCCEEDED:
            if current.state is ExecutionState.CANCELLING:
                failure = DomainFailure(
                    DomainErrorCode.BACKEND_QUARANTINED,
                    "Execution succeeded after a stop request; cancellation is ambiguous.",
                )
                self._enter_quarantine(execution_id)
                return self._transition_terminal(execution_id, ExecutionState.FAILED, failure)
            return self._executions.transition(execution_id, ExecutionState.SUCCEEDED)
        if outcome is BackendExecutionOutcome.PREEMPTED and current.state is ExecutionState.CANCELLING:
            self._executions.update_cancellation_status(
                execution_id, backend_stop_acknowledged=True
            )
            stabilized = await self._wait_for_state_stabilization(plan.trajectory.joint_names)
            if stabilized:
                self._executions.update_cancellation_status(
                    execution_id, state_stabilized=True
                )
                return self._executions.transition(execution_id, ExecutionState.CANCELLED)
            failure = DomainFailure(
                DomainErrorCode.BACKEND_QUARANTINED,
                "Joint state did not stabilize after backend preemption.",
            )
            self._enter_quarantine(execution_id)
            return self._transition_terminal(execution_id, ExecutionState.FAILED, failure)
        failure = DomainFailure(
            DomainErrorCode.EXECUTION_FAILED,
            terminal.value.detail or "The execution backend reported failure.",
        )
        if current.state is ExecutionState.CANCELLING:
            self._enter_quarantine(execution_id)
            failure = DomainFailure(
                DomainErrorCode.BACKEND_QUARANTINED,
                "Unexpected terminal result after a stop request.",
            )
        return self._transition_terminal(execution_id, ExecutionState.FAILED, failure)

    async def _wait_for_execution_terminal(
        self,
        execution_id: str,
        *,
        normal_timeout_seconds: float,
        cancellation_latched: bool,
    ) -> DomainResult[BackendExecutionResult]:
        """Observe cancellation while retaining bounded action-result waits."""
        deadline = monotonic() + normal_timeout_seconds
        cancellation_deadline = (
            monotonic() + self._cancellation_timeout_seconds
            if cancellation_latched
            else None
        )
        while True:
            current = self._executions.get(execution_id)
            if (
                current.error is None
                and current.value.state is ExecutionState.CANCELLING
                and cancellation_deadline is None
            ):
                cancellation_deadline = monotonic() + self._cancellation_timeout_seconds
            active_deadline = min(
                deadline,
                cancellation_deadline if cancellation_deadline is not None else deadline,
            )
            remaining = active_deadline - monotonic()
            if remaining <= 0:
                return DomainResult(error=DomainFailure(
                    DomainErrorCode.TIMEOUT,
                    "Timed out waiting for the terminal execution result.",
                ))
            result = await self._call(
                lambda: self._execution.wait_execution_result(
                    execution_id,
                    timeout_seconds=min(0.1, remaining),
                )
            )
            if result.error is None or result.error.code is not DomainErrorCode.TIMEOUT:
                return result

    async def cancel_execution(self, execution_id: str) -> DomainResult[ExecutionRecord]:
        """Latch or dispatch one bounded backend stop request."""
        if self._execution is None:
            return DomainResult(error=DomainFailure(
                DomainErrorCode.EXECUTION_DISABLED,
                "The execution backend is not configured.",
            ))
        with self._execution_lock:
            found = self._executions.get(execution_id)
            if found.error is not None:
                return found
            current = found.value
            if not self._executions.owns_active_execution(execution_id):
                return DomainResult(error=DomainFailure(
                    DomainErrorCode.INVALID_EXECUTION_TRANSITION,
                    "Only the active execution can be cancelled.",
                ))
            if current.state is ExecutionState.CANCELLING:
                return DomainResult(value=current)
            if current.state not in {ExecutionState.STARTING, ExecutionState.RUNNING}:
                return DomainResult(error=DomainFailure(
                    DomainErrorCode.INVALID_EXECUTION_TRANSITION,
                    "Execution is not in a cancellable state.",
                ))
            current = self._executions.transition(
                execution_id, ExecutionState.CANCELLING
            ).value
            starting = found.value.state is ExecutionState.STARTING
        if starting:
            return DomainResult(value=current)
        stop = await self._dispatch_stop_once(execution_id)
        if stop.error is not None:
            return DomainResult(error=stop.error)
        return self._executions.get(execution_id)

    def has_active_execution(self) -> bool:
        """Expose the later scene/planning concurrency guard internally."""
        return self._executions.has_active_execution()

    async def _dispatch_stop_once(
        self,
        execution_id: str,
    ) -> DomainResult[bool]:
        current = self._executions.get(execution_id)
        if current.error is not None:
            return DomainResult(error=current.error)
        if current.value.backend_stop_requested:
            return DomainResult(value=False)
        self._executions.update_cancellation_status(
            execution_id, backend_stop_requested=True
        )
        stop = await self._call(
            lambda: self._execution.request_execution_stop(execution_id)
        )
        if stop.error is not None:
            self._enter_quarantine(execution_id)
        return stop

    async def _wait_for_state_stabilization(
        self,
        joint_names: tuple[str, ...],
    ) -> bool:
        deadline = monotonic() + self._stabilization_timeout_seconds
        previous: dict[str, float] | None = None
        quiet_since: float | None = None
        while monotonic() < deadline:
            result = await self.get_current_state()
            if result.error is not None:
                return False
            state = result.value.joints
            positions = dict(zip(state.joint_names, state.positions))
            if any(name not in positions for name in joint_names):
                return False
            velocities = (
                None
                if state.velocities is None
                else dict(zip(state.joint_names, state.velocities))
            )
            velocity_quiet = velocities is None or all(
                abs(velocities[name]) <= self._stabilization_velocity_rad_per_sec
                for name in joint_names
            )
            position_quiet = previous is not None and all(
                abs(positions[name] - previous[name])
                <= self._stabilization_position_delta_rad
                for name in joint_names
            )
            now = monotonic()
            if velocity_quiet and position_quiet:
                quiet_since = quiet_since or now
                if now - quiet_since >= self._stabilization_quiet_window_seconds:
                    return True
            else:
                quiet_since = None
            previous = positions
            await asyncio.sleep(0.05)
        return False

    def _fail_before_acceptance(
        self,
        execution: ExecutionRecord,
        failure: DomainFailure,
    ) -> DomainResult[ExecutionRecord]:
        self._plans.release_reservation(execution.plan_id, execution.execution_id)
        return self._transition_terminal(
            execution.execution_id, ExecutionState.FAILED, failure
        )

    def _terminal_failure(
        self,
        execution_id: str,
        failure: DomainFailure,
    ) -> DomainResult[ExecutionRecord]:
        return self._transition_terminal(execution_id, ExecutionState.FAILED, failure)

    def _transition_terminal(
        self,
        execution_id: str,
        state: ExecutionState,
        failure: DomainFailure,
    ) -> DomainResult[ExecutionRecord]:
        return self._executions.transition(execution_id, state, failure=failure)

    def _enter_quarantine(self, execution_id: str) -> None:
        self._execution_quarantined = True
        if self._execution is not None:
            self._execution.quarantine_execution_backend(execution_id)

    @staticmethod
    def _quarantine_failure() -> DomainResult:
        return DomainResult(error=DomainFailure(
            DomainErrorCode.BACKEND_QUARANTINED,
            "Execution backend is quarantined until process/adapter restart.",
        ))

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
        if self.has_active_execution():
            return self._active_execution_rejection()
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
        if self.has_active_execution():
            return self._active_execution_rejection()
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
        if self.has_active_execution():
            return self._active_execution_rejection()
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

    def _reject_execution(
        self,
        execution: ExecutionRecord,
        findings: list[ExecutionValidationFinding],
        *,
        state_age_seconds: float | None = None,
        start_state_matches: bool | None = None,
        scene_revision_matches: bool | None = None,
    ) -> DomainResult[ExecutionValidationResult]:
        """Fail validation, release ownership, and return bounded findings."""
        self._plans.release_reservation(
            execution.plan_id,
            execution.execution_id,
        )
        failure = DomainFailure(
            DomainErrorCode.EXECUTION_VALIDATION_FAILED,
            "Pre-execution validation was rejected.",
        )
        self._executions.transition(
            execution.execution_id,
            ExecutionState.FAILED,
            failure=failure,
        )
        return DomainResult(value=ExecutionValidationResult(
            execution_id=execution.execution_id,
            plan_id=execution.plan_id,
            policy_id=self._safety.policy.policy_id,
            findings=tuple(findings),
            state_age_seconds=state_age_seconds,
            start_state_matches=start_state_matches,
            scene_revision_matches=scene_revision_matches,
        ))

    @staticmethod
    def _finding(
        code: str,
        message: str,
        **details: object,
    ) -> ExecutionValidationFinding:
        return ExecutionValidationFinding(code, message, **details)

    @staticmethod
    def _active_execution_rejection() -> DomainResult:
        return DomainResult(error=DomainFailure(
            DomainErrorCode.EXECUTION_CONFLICT,
            "Planning-scene mutation is unavailable during active execution.",
        ))

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
