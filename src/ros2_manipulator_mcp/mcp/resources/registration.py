"""Register bounded read-only projections of application state."""

from collections.abc import Callable
from typing import Any

from mcp.server import MCPServer

from ros2_manipulator_mcp.application import ManipulatorService
from ros2_manipulator_mcp.domain import CollisionObject, MotionPlan, Pose, RobotState
from ros2_manipulator_mcp.safety import SafetyPolicy


ServiceProvider = Callable[[], ManipulatorService]
JSON_MIME = "application/json"


def register_resources(
    server: MCPServer,
    service_provider: ServiceProvider,
    *,
    policy: SafetyPolicy,
    configured_profile: str,
    service_timeout_seconds: float,
) -> None:
    """Register Phase 8 resources without adding backend behavior."""

    def resource(uri: str, name: str, description: str):
        return server.resource(
            uri,
            name=name,
            description=description,
            mime_type=JSON_MIME,
        )

    @resource(
        "manipulator://overview",
        "manipulator-overview",
        "Manipulator identity, groups, frames, and supported capabilities.",
    )
    async def overview() -> dict[str, Any]:
        result = await service_provider().describe_manipulator()
        if result.error is not None:
            return _error(result.error.code.value, result.error.message)
        return _ok({
            **_descriptor_data(result.value),
            "configured_profile": configured_profile,
            "capabilities": _capabilities(),
            "physical_execution_available": False,
        })

    @resource(
        "manipulator://groups",
        "planning-groups",
        "Bounded planning-group descriptors.",
    )
    async def groups() -> dict[str, Any]:
        result = await service_provider().list_planning_groups()
        if result.error is not None:
            return _error(result.error.code.value, result.error.message)
        return _ok({"planning_groups": [_group_data(group) for group in result.value]})

    @resource(
        "manipulator://groups/{group}",
        "planning-group",
        "One planning-group descriptor selected by URI.",
    )
    async def group(group: str) -> dict[str, Any]:
        result = await service_provider().list_planning_groups()
        if result.error is not None:
            return _error(result.error.code.value, result.error.message)
        match = next((value for value in result.value if value.name == group), None)
        if match is None:
            return _error("not_found", "Planning group was not found.")
        return _ok({"planning_group": _group_data(match)})

    @resource(
        "manipulator://state/current",
        "current-robot-state",
        "Current joint-state summary and source timestamp status.",
    )
    async def current_state() -> dict[str, Any]:
        result = await service_provider().get_current_state()
        if result.error is not None:
            return _error(result.error.code.value, result.error.message)
        return _ok({"robot_state": _state_data(result.value)})

    @resource(
        "manipulator://scene",
        "planning-scene",
        "Planning-scene revision and bounded primitive object summaries.",
    )
    async def scene() -> dict[str, Any]:
        result = await service_provider().get_planning_scene()
        if result.error is not None:
            return _error(result.error.code.value, result.error.message)
        return _ok(_scene_data(result.value))

    @resource(
        "manipulator://scene/objects",
        "collision-objects",
        "Bounded primitive collision-object summaries.",
    )
    async def objects() -> dict[str, Any]:
        result = await service_provider().get_planning_scene()
        if result.error is not None:
            return _error(result.error.code.value, result.error.message)
        return _ok({
            "scene_revision": result.value.revision,
            "collision_objects": [
                _collision_data(value) for value in result.value.collision_objects
            ],
        })

    @resource(
        "manipulator://scene/objects/{object_id}",
        "collision-object",
        "One primitive collision object selected by URI.",
    )
    async def object_by_id(object_id: str) -> dict[str, Any]:
        result = await service_provider().get_planning_scene()
        if result.error is not None:
            return _error(result.error.code.value, result.error.message)
        match = next(
            (value for value in result.value.collision_objects if value.object_id == object_id),
            None,
        )
        if match is None:
            return _error("not_found", "Collision object was not found.")
        return _ok({
            "scene_revision": result.value.revision,
            "collision_object": _collision_data(match),
        })

    @resource(
        "manipulator://plans/{plan_id}",
        "stored-motion-plan",
        "One server-owned plan's bounded metadata and current validation status.",
    )
    async def plan_by_id(plan_id: str) -> dict[str, Any]:
        service = service_provider()
        result = service.get_motion_plan(plan_id)
        if result.error is not None:
            return _error(result.error.code.value, result.error.message)
        validation = service.prepare_plan_validation(plan_id)
        validation_data = (
            {
                "valid": True,
                "policy_id": validation.value.policy_id,
                "findings": [],
            }
            if validation.error is None
            else {
                "valid": False,
                "error_code": validation.error.code.value,
                "message": validation.error.message,
            }
        )
        return _ok({
            "motion_plan": _plan_data(result.value),
            "validation": validation_data,
        })

    @resource(
        "manipulator://safety",
        "safety-policy",
        "Effective MCP policy limits and explicit physical-safety non-guarantees.",
    )
    def safety() -> dict[str, Any]:
        return _ok({
            "policy": _policy_data(policy),
            "physical_execution_available": False,
            "certified_physical_safety": False,
            "non_guarantees": [
                "This MCP policy layer is NOT certified physical safety.",
                "No emergency-stop capability is exposed.",
                "No machinery-safety compliance is claimed.",
                "Collision avoidance is not guaranteed.",
                "No human detection or real-time torque/speed enforcement is provided.",
            ],
        })

    @resource(
        "manipulator://health",
        "manipulator-health",
        "On-demand application/backend readiness without background polling.",
    )
    async def health() -> dict[str, Any]:
        service = service_provider()
        descriptor = await service.describe_manipulator()
        state = await service.get_current_state()
        checks = {
            "application": {"ready": True},
            "descriptor": _health_check(descriptor),
            "current_state": _health_check(state),
        }
        ready = all(check["ready"] for check in checks.values())
        return _ok({
            "status": "ready" if ready else "degraded",
            "ready": ready,
            "checks": checks,
            "configured_profile": configured_profile,
            "service_timeout_seconds": service_timeout_seconds,
            "polling": False,
            "service_level_readiness_probed": False,
        })


def _ok(data: dict[str, Any]) -> dict[str, Any]:
    return {"success": True, "status": "available", "data": data}


def _error(code: str, message: str) -> dict[str, Any]:
    return {
        "success": False,
        "status": "unavailable",
        "error_code": code,
        "message": message,
        "data": None,
    }


def _health_check(result: Any) -> dict[str, Any]:
    if result.error is None:
        return {"ready": True}
    return {
        "ready": False,
        "error_code": result.error.code.value,
        "message": result.error.message,
    }


def _capabilities() -> list[str]:
    return [
        "discovery", "current_state", "forward_kinematics",
        "inverse_kinematics", "state_validity", "joint_goal_planning",
        "pose_goal_planning", "cartesian_path", "stored_plans",
        "primitive_planning_scene",
    ]


def _group_data(group: Any) -> dict[str, Any]:
    return {
        "name": group.name,
        "joint_names": list(group.joint_names),
        "link_names": list(group.link_names),
        "base_link": group.base_link,
        "end_effector_link": group.end_effector_link,
    }


def _descriptor_data(descriptor: Any) -> dict[str, Any]:
    return {
        "manipulator_id": descriptor.manipulator_id,
        "model_frame": descriptor.model_frame,
        "planning_groups": [_group_data(group) for group in descriptor.groups],
    }


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
        "freshness": "timestamp_available" if state.timestamp_seconds is not None else "unknown",
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
        "collision_object_count": len(scene.collision_objects),
        "collision_objects": [_collision_data(value) for value in scene.collision_objects],
    }


def _trajectory_summary(trajectory: Any) -> dict[str, Any]:
    return {
        "joint_names": list(trajectory.joint_names),
        "point_count": len(trajectory.points),
        "duration_seconds": trajectory.points[-1].time_from_start_seconds,
    }


def _plan_data(plan: MotionPlan) -> dict[str, Any]:
    goal = plan.request.goal
    goal_data = {
        "type": "joint" if hasattr(goal, "joint_names") else "pose",
        "planning_group": goal.group_name,
    }
    if hasattr(goal, "joint_names"):
        goal_data["joint_names"] = list(goal.joint_names)
    else:
        goal_data["target_link"] = goal.target_link
        goal_data["pose"] = _pose_data(goal.pose)
    return {
        "plan_id": plan.plan_id,
        "goal": goal_data,
        "scene_revision": plan.scene_revision,
        "planning_duration_seconds": plan.planning_duration_seconds,
        "max_velocity_scaling_factor": plan.request.max_velocity_scaling_factor,
        "max_acceleration_scaling_factor": plan.request.max_acceleration_scaling_factor,
        "trajectory": _trajectory_summary(plan.trajectory),
    }


def _policy_data(policy: SafetyPolicy) -> dict[str, Any]:
    workspace = None
    if policy.workspace is not None:
        workspace = {
            "minimum": vars(policy.workspace.minimum),
            "maximum": vars(policy.workspace.maximum),
        }
    return {
        "policy_id": policy.policy_id,
        "allowed_planning_groups": list(policy.allowed_planning_groups),
        "max_planning_time_seconds": policy.max_planning_time_seconds,
        "max_planning_attempts": policy.max_planning_attempts,
        "max_velocity_scaling_factor": policy.max_velocity_scaling_factor,
        "max_acceleration_scaling_factor": policy.max_acceleration_scaling_factor,
        "max_joint_targets": policy.max_joint_targets,
        "workspace": workspace,
        "cartesian": {
            "max_waypoints": policy.max_cartesian_waypoints,
            "min_step": policy.min_cartesian_step,
            "max_step": policy.max_cartesian_step,
            "min_completion_fraction": policy.min_cartesian_completion_fraction,
            "require_collision_avoidance": policy.require_collision_avoidance,
        },
        "collision_objects": {
            "allowed_id_prefixes": list(policy.allowed_object_id_prefixes),
            "max_objects": policy.max_collision_objects,
            "max_primitive_dimension": policy.max_primitive_dimension,
            "allowed_primitive_types": [value.value for value in policy.allowed_primitive_types],
            "allowed_scene_frames": list(policy.allowed_scene_frames),
            "allow_replace": policy.allow_object_replace,
        },
        "require_scene_revision": policy.require_scene_revision,
    }
