"""Register structured tools that call only ManipulatorService."""

import json
from collections.abc import Callable
from typing import Annotated, Any

from mcp.server import MCPServer
from mcp_types import CallToolResult, TextContent, ToolAnnotations

from ros2_manipulator_mcp.application import ManipulatorService
from ros2_manipulator_mcp.domain import (
    CartesianPathRequest,
    CollisionObject,
    CollisionPrimitive,
    DomainErrorCode,
    DomainFailure,
    DomainResult,
    JointGoal,
    JointState,
    Pose,
    PoseGoal,
    PrimitiveType,
    Quaternion,
    RobotState,
    Vector3,
)
from ros2_manipulator_mcp.mcp.tools.models import (
    CollisionObjectInput,
    JointGoalInput,
    JointStateInput,
    PoseGoalInput,
    PoseInput,
    ToolResponse,
)


StructuredResult = Annotated[CallToolResult, ToolResponse]
ServiceProvider = Callable[[], ManipulatorService]

READ_ONLY = ToolAnnotations(
    readOnlyHint=True, destructiveHint=False, idempotentHint=True,
    openWorldHint=False,
)
PLANNING = ToolAnnotations(
    readOnlyHint=False, destructiveHint=False, idempotentHint=False,
    openWorldHint=False,
)
MUTATION = ToolAnnotations(
    readOnlyHint=False, destructiveHint=True, idempotentHint=False,
    openWorldHint=False,
)


def register_tools(server: MCPServer, service_provider: ServiceProvider) -> None:
    """Register the complete Phase 7 tool set against an application provider."""

    def tool(name: str, description: str, annotations: ToolAnnotations):
        return server.tool(
            name=name,
            description=description,
            annotations=annotations,
            structured_output=True,
        )

    @tool("describe_manipulator", "Describe the configured manipulator.", READ_ONLY)
    async def describe_manipulator() -> StructuredResult:
        result = await service_provider().describe_manipulator()
        return _result(result, _descriptor_data)

    @tool("list_planning_groups", "List available planning groups.", READ_ONLY)
    async def list_planning_groups() -> StructuredResult:
        result = await service_provider().list_planning_groups()
        return _result(result, lambda groups: {"planning_groups": [_group_data(g) for g in groups]})

    @tool("get_planning_group", "Get one planning group by name.", READ_ONLY)
    async def get_planning_group(planning_group: str) -> StructuredResult:
        result = await service_provider().list_planning_groups()
        if result.error is not None:
            return _result(result)
        match = next((g for g in result.value if g.name == planning_group), None)
        if match is None:
            return _failure(DomainErrorCode.NOT_FOUND, "Planning group was not found.")
        return _success({"planning_group": _group_data(match)})

    @tool("get_current_robot_state", "Read the current robot joint state.", READ_ONLY)
    async def get_current_robot_state() -> StructuredResult:
        return _result(await service_provider().get_current_state(), _state_data)

    @tool("get_end_effector_pose", "Read a group's current end-effector pose.", READ_ONLY)
    async def get_end_effector_pose(planning_group: str, target_link: str) -> StructuredResult:
        result = await service_provider().get_end_effector_pose(planning_group, target_link)
        return _result(result, lambda pose: {"pose": _pose_data(pose)})

    @tool("compute_forward_kinematics", "Compute a link pose for a supplied robot state.", READ_ONLY)
    async def compute_forward_kinematics(
        planning_group: str, target_link: str, robot_state: JointStateInput
    ) -> StructuredResult:
        try:
            state = _robot_state(robot_state)
        except ValueError as error:
            return _invalid(error)
        result = await service_provider().compute_forward_kinematics(planning_group, target_link, state)
        return _result(result, lambda pose: {"pose": _pose_data(pose)})

    @tool("compute_inverse_kinematics", "Compute a collision-aware joint solution for a pose.", READ_ONLY)
    async def compute_inverse_kinematics(
        planning_group: str,
        target_link: str,
        target_pose: PoseInput,
        seed_state: JointStateInput | None = None,
    ) -> StructuredResult:
        try:
            pose = _pose(target_pose)
            seed = _robot_state(seed_state) if seed_state is not None else None
        except ValueError as error:
            return _invalid(error)
        result = await service_provider().compute_inverse_kinematics(
            planning_group, target_link, pose, seed
        )
        return _result(result, _state_data)

    @tool("check_robot_state_validity", "Check collision and model validity for a robot state.", READ_ONLY)
    async def check_robot_state_validity(
        planning_group: str, robot_state: JointStateInput
    ) -> StructuredResult:
        try:
            state = _robot_state(robot_state)
        except ValueError as error:
            return _invalid(error)
        result = await service_provider().check_state_validity(planning_group, state)
        return _result(result, lambda valid: {"valid": valid})

    @tool("plan_to_joint_goal", "Create and store a policy-approved joint-goal plan.", PLANNING)
    async def plan_to_joint_goal(
        goal: JointGoalInput,
        start_state: JointStateInput | None = None,
        max_velocity_scaling_factor: float = 1.0,
        max_acceleration_scaling_factor: float = 1.0,
        allowed_planning_time_seconds: float = 5.0,
        planning_attempts: int = 1,
    ) -> StructuredResult:
        try:
            domain_goal = JointGoal(goal.planning_group, tuple(goal.joint_names), tuple(goal.positions))
            state = _robot_state(start_state) if start_state is not None else None
        except ValueError as error:
            return _invalid(error)
        result = await service_provider().plan_to_joint_goal(
            domain_goal,
            start_state=state,
            max_velocity_scaling_factor=max_velocity_scaling_factor,
            max_acceleration_scaling_factor=max_acceleration_scaling_factor,
            allowed_planning_time_seconds=allowed_planning_time_seconds,
            planning_attempts=planning_attempts,
        )
        return _result(result, _plan_data)

    @tool("plan_to_pose_goal", "Create and store a policy-approved pose-goal plan.", PLANNING)
    async def plan_to_pose_goal(
        goal: PoseGoalInput,
        start_state: JointStateInput | None = None,
        max_velocity_scaling_factor: float = 1.0,
        max_acceleration_scaling_factor: float = 1.0,
        allowed_planning_time_seconds: float = 5.0,
        planning_attempts: int = 1,
    ) -> StructuredResult:
        try:
            domain_goal = PoseGoal(
                goal.planning_group, goal.target_link, _pose(goal.pose),
                goal.position_tolerance, goal.orientation_tolerance,
            )
            state = _robot_state(start_state) if start_state is not None else None
        except ValueError as error:
            return _invalid(error)
        result = await service_provider().plan_to_pose_goal(
            domain_goal,
            start_state=state,
            max_velocity_scaling_factor=max_velocity_scaling_factor,
            max_acceleration_scaling_factor=max_acceleration_scaling_factor,
            allowed_planning_time_seconds=allowed_planning_time_seconds,
            planning_attempts=planning_attempts,
        )
        return _result(result, _plan_data)

    @tool("compute_cartesian_path", "Compute a policy-approved Cartesian path without execution.", PLANNING)
    async def compute_cartesian_path(
        planning_group: str,
        target_link: str,
        waypoints: list[PoseInput],
        max_step: float,
        jump_threshold: float = 0.0,
        avoid_collisions: bool = True,
        start_state: JointStateInput | None = None,
        max_velocity_scaling_factor: float = 1.0,
        max_acceleration_scaling_factor: float = 1.0,
    ) -> StructuredResult:
        try:
            request = CartesianPathRequest(
                planning_group, target_link, tuple(_pose(p) for p in waypoints),
                max_step, jump_threshold, avoid_collisions,
                _robot_state(start_state) if start_state is not None else None,
                max_velocity_scaling_factor, max_acceleration_scaling_factor,
            )
        except ValueError as error:
            return _invalid(error)
        result = await service_provider().compute_cartesian_path(request)
        return _result(result, lambda value: {
            "fraction": value.fraction,
            "trajectory": _trajectory_summary(value.trajectory) if value.trajectory else None,
        })

    @tool("get_motion_plan", "Inspect a server-owned stored motion plan.", READ_ONLY)
    async def get_motion_plan(plan_id: str) -> StructuredResult:
        return _result(service_provider().get_motion_plan(plan_id), _plan_data)

    @tool("discard_motion_plan", "Discard a server-owned stored motion plan.", MUTATION)
    async def discard_motion_plan(plan_id: str) -> StructuredResult:
        return _result(service_provider().discard_motion_plan(plan_id), lambda discarded: {"discarded": discarded})

    @tool("validate_motion_plan", "Validate stored plan provenance and policy metadata.", READ_ONLY)
    async def validate_motion_plan(plan_id: str) -> StructuredResult:
        result = service_provider().prepare_plan_validation(plan_id)
        return _result(result, lambda decision: {
            "valid": decision.allowed, "policy_id": decision.policy_id,
            "findings": [
                {"code": f.code, "message": f.message, "severity": f.severity.value}
                for f in decision.findings
            ],
        })

    @tool("get_planning_scene", "Read the primitive planning scene.", READ_ONLY)
    async def get_planning_scene() -> StructuredResult:
        return _result(await service_provider().get_planning_scene(), _scene_data)

    @tool("list_collision_objects", "List primitive collision objects.", READ_ONLY)
    async def list_collision_objects() -> StructuredResult:
        result = await service_provider().get_planning_scene()
        return _result(result, lambda scene: {
            "scene_revision": scene.revision,
            "collision_objects": [_collision_data(o) for o in scene.collision_objects],
        })

    @tool("get_collision_object", "Get one primitive collision object by ID.", READ_ONLY)
    async def get_collision_object(object_id: str) -> StructuredResult:
        result = await service_provider().get_planning_scene()
        if result.error is not None:
            return _result(result)
        match = next((o for o in result.value.collision_objects if o.object_id == object_id), None)
        if match is None:
            return _failure(DomainErrorCode.NOT_FOUND, "Collision object was not found.")
        return _success({"scene_revision": result.value.revision, "collision_object": _collision_data(match)})

    @tool("apply_collision_object", "Add or explicitly replace a policy-approved primitive.", MUTATION)
    async def apply_collision_object(
        collision_object: CollisionObjectInput, replace: bool = False
    ) -> StructuredResult:
        try:
            value = CollisionObject(
                collision_object.object_id,
                CollisionPrimitive(
                    PrimitiveType(collision_object.primitive_type),
                    tuple(collision_object.dimensions),
                ),
                _pose(collision_object.pose),
            )
        except ValueError as error:
            return _invalid(error)
        result = await service_provider().apply_collision_object(value, replace=replace)
        return _result(result, _scene_data)

    @tool("remove_collision_object", "Remove a policy-approved primitive by ID.", MUTATION)
    async def remove_collision_object(object_id: str) -> StructuredResult:
        return _result(await service_provider().remove_collision_object(object_id), _scene_data)

    @tool(
        "execute_motion_plan",
        "Start execution of one application-owned validated plan ID.",
        MUTATION,
    )
    async def execute_motion_plan(plan_id: str) -> StructuredResult:
        result = await service_provider().start_motion_plan_execution(plan_id)
        return _result(result, _execution_data)

    @tool(
        "get_execution_status",
        "Read bounded application-owned execution status.",
        READ_ONLY,
    )
    async def get_execution_status(execution_id: str) -> StructuredResult:
        return _result(
            service_provider().get_execution_status(execution_id),
            _execution_data,
        )

    @tool(
        "cancel_execution",
        "Request bounded cancellation of the active execution.",
        MUTATION,
    )
    async def cancel_execution(execution_id: str) -> StructuredResult:
        result = await service_provider().cancel_execution(execution_id)
        return _result(result, _execution_data)


def _pose(value: PoseInput) -> Pose:
    return Pose(
        value.frame_id,
        Vector3(value.position.x, value.position.y, value.position.z),
        Quaternion(
            value.orientation.x, value.orientation.y,
            value.orientation.z, value.orientation.w,
        ),
    )


def _execution_data(execution: Any) -> dict[str, Any]:
    failure = (
        None
        if execution.failure is None
        else {
            "code": execution.failure.code.value,
            "message": execution.failure.message,
        }
    )
    return {
        "execution_id": execution.execution_id,
        "plan_id": execution.plan_id,
        "state": execution.state.value,
        "accepted": execution.backend_goal_accepted,
        "started": execution.backend_goal_accepted,
        "cancellation_requested": execution.cancellation_requested,
        "backend_stop_requested": execution.backend_stop_requested,
        "backend_stop_acknowledged": execution.backend_stop_acknowledged,
        "execution_terminal": execution.execution_terminal,
        "state_stabilized": execution.state_stabilized,
        "physical_stop_confirmed": execution.physical_stop_confirmed,
        "failure": failure,
    }


def _robot_state(value: JointStateInput) -> RobotState:
    return RobotState(
        JointState(
            tuple(value.joint_names), tuple(value.positions),
            tuple(value.velocities) if value.velocities is not None else None,
            tuple(value.efforts) if value.efforts is not None else None,
        ),
        timestamp_seconds=value.timestamp_seconds,
    )


def _result(result: DomainResult, serializer: Callable[[Any], dict[str, Any]] | None = None) -> StructuredResult:
    if result.error is not None:
        return _failure(result.error.code, result.error.message)
    return _success(serializer(result.value) if serializer else {"value": result.value})


def _success(data: dict[str, Any]) -> StructuredResult:
    return _mcp_result(ToolResponse(success=True, status="success", message="Operation succeeded.", data=data))


def _failure(code: DomainErrorCode, message: str) -> StructuredResult:
    return _mcp_result(
        ToolResponse(success=False, status="error", error_code=code.value, message=message),
        is_error=True,
    )


def _invalid(error: ValueError) -> StructuredResult:
    return _failure(DomainErrorCode.INVALID_REQUEST, str(error))


def _mcp_result(response: ToolResponse, *, is_error: bool = False) -> StructuredResult:
    structured = response.model_dump(mode="json")
    return CallToolResult(
        content=[TextContent(type="text", text=json.dumps(structured, separators=(",", ":")))],
        structuredContent=structured,
        isError=is_error,
    )


def _pose_data(pose: Pose) -> dict[str, Any]:
    return {
        "frame_id": pose.frame_id,
        "position": {"x": pose.position.x, "y": pose.position.y, "z": pose.position.z},
        "orientation": {
            "x": pose.orientation.x, "y": pose.orientation.y,
            "z": pose.orientation.z, "w": pose.orientation.w,
        },
    }


def _state_data(state: RobotState) -> dict[str, Any]:
    return {
        "joint_names": list(state.joints.joint_names),
        "positions": list(state.joints.positions),
        "velocities": list(state.joints.velocities) if state.joints.velocities is not None else None,
        "efforts": list(state.joints.efforts) if state.joints.efforts is not None else None,
        "timestamp_seconds": state.timestamp_seconds,
    }


def _group_data(group: Any) -> dict[str, Any]:
    return {
        "name": group.name, "joint_names": list(group.joint_names),
        "link_names": list(group.link_names), "base_link": group.base_link,
        "end_effector_link": group.end_effector_link,
    }


def _descriptor_data(descriptor: Any) -> dict[str, Any]:
    return {
        "manipulator_id": descriptor.manipulator_id,
        "model_frame": descriptor.model_frame,
        "planning_groups": [_group_data(group) for group in descriptor.groups],
    }


def _trajectory_summary(trajectory: Any) -> dict[str, Any]:
    return {
        "joint_names": list(trajectory.joint_names),
        "point_count": len(trajectory.points),
        "duration_seconds": trajectory.points[-1].time_from_start_seconds,
    }


def _plan_data(plan: Any) -> dict[str, Any]:
    return {
        "plan_id": plan.plan_id,
        "planning_group": plan.request.goal.group_name,
        "planning_duration_seconds": plan.planning_duration_seconds,
        "scene_revision": plan.scene_revision,
        "trajectory": _trajectory_summary(plan.trajectory),
        "max_velocity_scaling_factor": plan.request.max_velocity_scaling_factor,
        "max_acceleration_scaling_factor": plan.request.max_acceleration_scaling_factor,
    }


def _collision_data(value: CollisionObject) -> dict[str, Any]:
    return {
        "object_id": value.object_id,
        "primitive_type": value.primitive.primitive_type.value,
        "dimensions": list(value.primitive.dimensions),
        "pose": _pose_data(value.pose),
    }


def _scene_data(scene: Any) -> dict[str, Any]:
    return {
        "scene_revision": scene.revision,
        "collision_objects": [_collision_data(value) for value in scene.collision_objects],
        "robot_state": _state_data(scene.robot_state) if scene.robot_state is not None else None,
    }
