"""ROS 2 Jazzy adapter using verified MoveIt 2 services and messages."""

from hashlib import sha256
from threading import RLock
from typing import Any

from action_msgs.msg import GoalStatus
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import (
    CollisionObject as RosCollisionObject,
    MoveItErrorCodes,
    PlanningScene,
    PlanningSceneComponents,
)
from moveit_msgs.srv import (
    ApplyPlanningScene,
    GetCartesianPath,
    GetMotionPlan,
    GetPlanningScene,
    GetPositionFK,
    GetPositionIK,
    GetStateValidity,
)
from rclpy.serialization import serialize_message
from sensor_msgs.msg import JointState as RosJointState
from std_msgs.msg import String

from ros2_manipulator_mcp.application.ports import (
    BackendExecutionOutcome,
    BackendExecutionResult,
)

from ros2_manipulator_mcp.domain import (
    CartesianPathRequest,
    CartesianPathResult,
    CollisionObject,
    DomainErrorCode,
    DomainFailure,
    DomainResult,
    ManipulatorDescriptor,
    PlanningRequest,
    PlanningSceneSnapshot,
    Pose,
    RobotState,
    Trajectory,
)
from ros2_manipulator_mcp.ros.jazzy.conversions import (
    from_joint_state_message,
    from_planning_scene_message,
    from_pose_message,
    from_robot_state_message,
    from_trajectory_message,
    removal_collision_object_message,
    to_collision_object_message,
    to_duration,
    to_goal_constraints,
    to_pose_message,
    to_pose_stamped_message,
    to_robot_state_message,
    to_trajectory_message,
)
from ros2_manipulator_mcp.ros.jazzy.runtime import (
    JazzyRosRuntime,
    RosServiceRuntime,
    ServiceUnavailableError,
)
from ros2_manipulator_mcp.ros.jazzy.settings import JazzyMoveItSettings


class JazzyManipulatorAdapter:
    """Implement all v0.1.0 backend ports without exposing ROS values."""

    def __init__(
        self,
        descriptor: ManipulatorDescriptor,
        *,
        settings: JazzyMoveItSettings | None = None,
        runtime: RosServiceRuntime | None = None,
    ) -> None:
        self._descriptor = descriptor
        self._settings = settings or JazzyMoveItSettings()
        self._runtime = runtime or JazzyRosRuntime(self._settings.node_name)
        self._closed = False
        self._execution_lock = RLock()
        self._active_execution_id: str | None = None
        self._active_goal_handle: Any | None = None
        self._stop_requested: set[str] = set()
        self._execution_quarantined = False

    async def submit_execution(
        self,
        execution_id: str,
        trajectory: Trajectory,
        *,
        server_timeout_seconds: float,
        acceptance_timeout_seconds: float,
    ) -> DomainResult[bool]:
        """Submit one trajectory to the verified ExecuteTrajectory action."""
        with self._execution_lock:
            if self._execution_quarantined:
                return DomainResult(error=DomainFailure(
                    DomainErrorCode.BACKEND_QUARANTINED,
                    "The execution backend is quarantined until adapter restart.",
                ))
            if self._active_execution_id is not None:
                return DomainResult(error=DomainFailure(
                    DomainErrorCode.EXECUTION_CONFLICT,
                    "The execution backend already owns an active execution.",
                ))
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = to_trajectory_message(trajectory)
        try:
            handle = await self._runtime.send_action_goal(
                ExecuteTrajectory,
                self._settings.execute_trajectory_action,
                goal,
                server_timeout_seconds,
                acceptance_timeout_seconds,
            )
        except Exception as error:
            return self._runtime_failure(error)
        if handle is None or not handle.accepted:
            return DomainResult(value=False)
        with self._execution_lock:
            self._active_execution_id = execution_id
            self._active_goal_handle = handle
        return DomainResult(value=True)

    async def wait_execution_result(
        self,
        execution_id: str,
        *,
        timeout_seconds: float,
    ) -> DomainResult[BackendExecutionResult]:
        """Wait boundedly and map exact Jazzy MoveIt terminal semantics."""
        with self._execution_lock:
            if execution_id != self._active_execution_id:
                return DomainResult(error=DomainFailure(
                    DomainErrorCode.NOT_FOUND,
                    "The execution is not active in this backend.",
                ))
            handle = self._active_goal_handle
        try:
            wrapped = await self._runtime.wait_action_result(handle, timeout_seconds)
        except Exception as error:
            return self._runtime_failure(error)
        status = int(wrapped.status)
        code = int(wrapped.result.error_code.val)
        if status == GoalStatus.STATUS_SUCCEEDED and code == MoveItErrorCodes.SUCCESS:
            outcome = BackendExecutionOutcome.SUCCEEDED
        elif status == GoalStatus.STATUS_ABORTED and code == MoveItErrorCodes.PREEMPTED:
            outcome = BackendExecutionOutcome.PREEMPTED
        else:
            outcome = BackendExecutionOutcome.FAILED
        detail = wrapped.result.error_code.message.strip()
        with self._execution_lock:
            self._active_execution_id = None
            self._active_goal_handle = None
            self._stop_requested.discard(execution_id)
        return DomainResult(value=BackendExecutionResult(outcome, detail))

    async def request_execution_stop(
        self,
        execution_id: str,
    ) -> DomainResult[bool]:
        """Publish the MoveIt 2.12.4 global stop event exactly once."""
        with self._execution_lock:
            if execution_id != self._active_execution_id:
                return DomainResult(error=DomainFailure(
                    DomainErrorCode.NOT_FOUND,
                    "The execution is not active in this backend.",
                ))
            if execution_id in self._stop_requested:
                return DomainResult(value=False)
            self._stop_requested.add(execution_id)
        try:
            self._runtime.publish_message(
                String,
                self._settings.trajectory_execution_event_topic,
                String(data="stop"),
            )
        except Exception as error:
            self.quarantine_execution_backend(execution_id)
            return self._runtime_failure(error)
        return DomainResult(value=True)

    def quarantine_execution_backend(self, execution_id: str) -> None:
        """Prevent later execution after an ambiguous backend outcome."""
        del execution_id
        with self._execution_lock:
            self._execution_quarantined = True

    async def describe_manipulator(
        self,
    ) -> DomainResult[ManipulatorDescriptor]:
        """Return deployment-supplied backend-neutral metadata."""
        return DomainResult(value=self._descriptor)

    async def get_current_state(self) -> DomainResult[RobotState]:
        """Read one bounded JointState sample from the configured topic."""
        try:
            message = await self._runtime.read_message(
                RosJointState,
                self._settings.joint_states_topic,
                self._settings.state_timeout_seconds,
            )
            stamp = message.header.stamp
            timestamp = float(stamp.sec) + float(stamp.nanosec) / 1_000_000_000
            age = self._runtime.now_seconds() - timestamp
            if age < 0:
                return self._backend_error(
                    "Joint-state timestamp is ahead of the adapter clock."
                )
            return DomainResult(
                value=RobotState(
                    from_joint_state_message(message),
                    timestamp_seconds=timestamp,
                    sample_age_seconds=age,
                )
            )
        except Exception as error:
            return self._runtime_failure(error)

    async def get_end_effector_pose(
        self,
        group_name: str,
        end_effector_link: str,
    ) -> DomainResult[Pose]:
        """Resolve the current state and compute its end-effector FK."""
        state = await self.get_current_state()
        if state.error is not None:
            return DomainResult(error=state.error)
        return await self.compute_forward_kinematics(
            group_name,
            end_effector_link,
            state.value,
        )

    async def compute_forward_kinematics(
        self,
        group_name: str,
        target_link: str,
        state: RobotState,
    ) -> DomainResult[Pose]:
        """Call `moveit_msgs/srv/GetPositionFK`."""
        request = GetPositionFK.Request()
        request.header.frame_id = self._descriptor.model_frame
        request.fk_link_names = [target_link]
        request.robot_state = to_robot_state_message(state, is_diff=True)
        response = await self._call(
            GetPositionFK,
            self._settings.fk_service,
            request,
        )
        if isinstance(response, DomainResult):
            return response
        failure = self._moveit_failure(response.error_code, "Forward kinematics")
        if failure is not None:
            return DomainResult(error=failure)
        if target_link not in response.fk_link_names:
            return self._backend_error("FK response did not contain the target link.")
        index = list(response.fk_link_names).index(target_link)
        if index >= len(response.pose_stamped):
            return self._backend_error("FK response pose list was inconsistent.")
        pose = response.pose_stamped[index]
        return DomainResult(
            value=from_pose_message(pose.pose, pose.header.frame_id)
        )

    async def compute_inverse_kinematics(
        self,
        group_name: str,
        target_link: str,
        target_pose: Pose,
        seed_state: RobotState | None = None,
    ) -> DomainResult[RobotState]:
        """Call `moveit_msgs/srv/GetPositionIK` collision-aware."""
        request = GetPositionIK.Request()
        request.ik_request.group_name = group_name
        request.ik_request.robot_state = to_robot_state_message(
            seed_state,
            is_diff=True,
        )
        request.ik_request.avoid_collisions = True
        request.ik_request.ik_link_name = target_link
        request.ik_request.pose_stamped = to_pose_stamped_message(target_pose)
        request.ik_request.timeout = to_duration(
            self._settings.ik_timeout_seconds
        )
        response = await self._call(
            GetPositionIK,
            self._settings.ik_service,
            request,
        )
        if isinstance(response, DomainResult):
            return response
        failure = self._moveit_failure(response.error_code, "Inverse kinematics")
        if failure is not None:
            return DomainResult(error=failure)
        try:
            return DomainResult(value=from_robot_state_message(response.solution))
        except ValueError as error:
            return self._backend_error(f"Invalid IK response: {error}")

    async def check_state_validity(
        self,
        group_name: str,
        state: RobotState,
    ) -> DomainResult[bool]:
        """Call `moveit_msgs/srv/GetStateValidity`."""
        request = GetStateValidity.Request()
        request.group_name = group_name
        request.robot_state = to_robot_state_message(state, is_diff=True)
        response = await self._call(
            GetStateValidity,
            self._settings.state_validity_service,
            request,
        )
        if isinstance(response, DomainResult):
            return response
        return DomainResult(value=bool(response.valid))

    async def plan_motion(
        self,
        request: PlanningRequest,
    ) -> DomainResult[Trajectory]:
        """Call the plan-only `moveit_msgs/srv/GetMotionPlan` service."""
        service_request = GetMotionPlan.Request()
        motion_request = service_request.motion_plan_request
        motion_request.group_name = request.goal.group_name
        motion_request.start_state = to_robot_state_message(
            request.start_state,
            is_diff=True,
        )
        motion_request.goal_constraints = [
            to_goal_constraints(
                request,
                joint_goal_tolerance=self._settings.joint_goal_tolerance,
            )
        ]
        motion_request.num_planning_attempts = request.planning_attempts
        motion_request.allowed_planning_time = (
            request.allowed_planning_time_seconds
        )
        motion_request.max_velocity_scaling_factor = (
            request.max_velocity_scaling_factor
        )
        motion_request.max_acceleration_scaling_factor = (
            request.max_acceleration_scaling_factor
        )
        response = await self._call(
            GetMotionPlan,
            self._settings.motion_plan_service,
            service_request,
        )
        if isinstance(response, DomainResult):
            return response
        plan_response = response.motion_plan_response
        failure = self._moveit_failure(
            plan_response.error_code,
            "Motion planning",
        )
        if failure is not None:
            return DomainResult(error=failure)
        try:
            return DomainResult(
                value=from_trajectory_message(plan_response.trajectory)
            )
        except ValueError as error:
            return self._backend_error(f"Invalid planning response: {error}")

    async def compute_cartesian_path(
        self,
        request: CartesianPathRequest,
    ) -> DomainResult[CartesianPathResult]:
        """Call `moveit_msgs/srv/GetCartesianPath`."""
        frames = {waypoint.frame_id for waypoint in request.waypoints}
        if len(frames) != 1:
            return DomainResult(
                error=DomainFailure(
                    DomainErrorCode.INVALID_REQUEST,
                    "Cartesian waypoints must use one common frame.",
                )
            )
        service_request = GetCartesianPath.Request()
        service_request.header.frame_id = next(iter(frames))
        service_request.start_state = to_robot_state_message(
            request.start_state,
            is_diff=True,
        )
        service_request.group_name = request.group_name
        service_request.link_name = request.target_link
        service_request.waypoints = [
            to_pose_message(waypoint) for waypoint in request.waypoints
        ]
        service_request.max_step = request.max_step
        service_request.jump_threshold = request.jump_threshold
        service_request.avoid_collisions = request.avoid_collisions
        service_request.max_velocity_scaling_factor = (
            request.max_velocity_scaling_factor
        )
        service_request.max_acceleration_scaling_factor = (
            request.max_acceleration_scaling_factor
        )
        response = await self._call(
            GetCartesianPath,
            self._settings.cartesian_path_service,
            service_request,
        )
        if isinstance(response, DomainResult):
            return response
        failure = self._moveit_failure(response.error_code, "Cartesian path")
        if failure is not None:
            return DomainResult(error=failure)
        try:
            trajectory = (
                from_trajectory_message(response.solution)
                if response.solution.joint_trajectory.points
                else None
            )
            return DomainResult(
                value=CartesianPathResult(float(response.fraction), trajectory)
            )
        except ValueError as error:
            return self._backend_error(f"Invalid Cartesian response: {error}")

    async def get_planning_scene(
        self,
    ) -> DomainResult[PlanningSceneSnapshot]:
        """Read robot state and primitive world geometry."""
        request = GetPlanningScene.Request()
        request.components.components = (
            PlanningSceneComponents.ROBOT_STATE
            | PlanningSceneComponents.WORLD_OBJECT_NAMES
            | PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        )
        response = await self._call(
            GetPlanningScene,
            self._settings.get_planning_scene_service,
            request,
        )
        if isinstance(response, DomainResult):
            return response
        try:
            revision = sha256(
                serialize_message(response.scene.world)
            ).hexdigest()
            return DomainResult(
                value=from_planning_scene_message(
                    response.scene,
                    revision=revision,
                )
            )
        except ValueError as error:
            return self._backend_error(f"Unsupported planning scene: {error}")

    async def apply_collision_object(
        self,
        collision_object: CollisionObject,
        *,
        replace: bool,
    ) -> DomainResult[PlanningSceneSnapshot]:
        """Apply one safety-approved ADD operation and return the new scene."""
        del replace  # Phase 4 has already authorized explicit replacement.
        return await self._apply_scene_object(
            to_collision_object_message(
                collision_object,
                operation=RosCollisionObject.ADD,
            )
        )

    async def remove_collision_object(
        self,
        object_id: str,
    ) -> DomainResult[PlanningSceneSnapshot]:
        """Apply one safety-approved REMOVE operation and return the new scene."""
        return await self._apply_scene_object(
            removal_collision_object_message(object_id)
        )

    async def _apply_scene_object(
        self,
        collision_object: RosCollisionObject,
    ) -> DomainResult[PlanningSceneSnapshot]:
        request = ApplyPlanningScene.Request()
        request.scene = PlanningScene()
        request.scene.is_diff = True
        request.scene.world.collision_objects = [collision_object]
        response = await self._call(
            ApplyPlanningScene,
            self._settings.apply_planning_scene_service,
            request,
        )
        if isinstance(response, DomainResult):
            return response
        if not response.success:
            return self._backend_error("MoveIt rejected the planning-scene diff.")
        return await self.get_planning_scene()

    async def _call(
        self,
        service_type: type,
        service_name: str,
        request: Any,
    ) -> Any | DomainResult:
        try:
            self._ensure_open()
            return await self._runtime.call_service(
                service_type,
                service_name,
                request,
                self._settings.service_timeout_seconds,
            )
        except Exception as error:
            return self._runtime_failure(error)

    @staticmethod
    def _moveit_failure(
        error_code: MoveItErrorCodes,
        operation: str,
    ) -> DomainFailure | None:
        if error_code.val == MoveItErrorCodes.SUCCESS:
            return None
        if error_code.val == MoveItErrorCodes.TIMED_OUT:
            code = DomainErrorCode.TIMEOUT
        elif error_code.val == MoveItErrorCodes.NO_IK_SOLUTION:
            code = DomainErrorCode.NO_SOLUTION
        elif error_code.val in {
            MoveItErrorCodes.START_STATE_IN_COLLISION,
            MoveItErrorCodes.GOAL_IN_COLLISION,
        }:
            code = DomainErrorCode.IN_COLLISION
        elif error_code.val in {
            MoveItErrorCodes.INVALID_GROUP_NAME,
            MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS,
            MoveItErrorCodes.INVALID_ROBOT_STATE,
            MoveItErrorCodes.INVALID_LINK_NAME,
            MoveItErrorCodes.INVALID_OBJECT_NAME,
            MoveItErrorCodes.START_STATE_INVALID,
            MoveItErrorCodes.GOAL_STATE_INVALID,
            MoveItErrorCodes.UNRECOGNIZED_GOAL_TYPE,
        }:
            code = DomainErrorCode.INVALID_REQUEST
        elif error_code.val == MoveItErrorCodes.ROBOT_STATE_STALE:
            code = DomainErrorCode.STALE_STATE
        elif error_code.val in {
            MoveItErrorCodes.COMMUNICATION_FAILURE,
            MoveItErrorCodes.COLLISION_CHECKING_UNAVAILABLE,
        }:
            code = DomainErrorCode.UNAVAILABLE
        elif error_code.val in {
            MoveItErrorCodes.PLANNING_FAILED,
            MoveItErrorCodes.INVALID_MOTION_PLAN,
            MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE,
        }:
            code = DomainErrorCode.PLANNING_FAILED
        else:
            code = DomainErrorCode.BACKEND_ERROR
        details = error_code.message.strip() or f"MoveIt error {error_code.val}"
        return DomainFailure(code, f"{operation} failed: {details}")

    @staticmethod
    def _runtime_failure(error: Exception) -> DomainResult:
        if isinstance(error, ServiceUnavailableError):
            return DomainResult(
                error=DomainFailure(DomainErrorCode.UNAVAILABLE, str(error))
            )
        if isinstance(error, TimeoutError):
            return DomainResult(
                error=DomainFailure(DomainErrorCode.TIMEOUT, str(error))
            )
        return JazzyManipulatorAdapter._backend_error(str(error))

    @staticmethod
    def _backend_error(message: str) -> DomainResult:
        return DomainResult(
            error=DomainFailure(DomainErrorCode.BACKEND_ERROR, message)
        )

    def close(self) -> None:
        """Attempt one stop for active execution, then release runtime once."""
        with self._execution_lock:
            if self._closed:
                return
            self._closed = True
            execution_id = self._active_execution_id
            should_stop = (
                execution_id is not None
                and execution_id not in self._stop_requested
            )
            if should_stop:
                self._stop_requested.add(execution_id)
        if should_stop:
            try:
                self._runtime.publish_message(
                    String,
                    self._settings.trajectory_execution_event_topic,
                    String(data="stop"),
                )
            except Exception:
                pass
        self._runtime.close()

    def _ensure_open(self) -> None:
        if self._closed:
            raise RuntimeError("The Jazzy manipulator adapter is closed.")

    def __enter__(self) -> "JazzyManipulatorAdapter":
        return self

    def __exit__(
        self,
        exception_type: type[BaseException] | None,
        exception: BaseException | None,
        traceback: object | None,
    ) -> None:
        self.close()
