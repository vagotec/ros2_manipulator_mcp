"""Focused tests against installed ROS 2 Jazzy interface classes."""

import ast
import asyncio
from pathlib import Path
from typing import Any

from moveit_msgs.msg import MoveItErrorCodes, RobotTrajectory
from moveit_msgs.srv import GetMotionPlan, GetPositionIK
from sensor_msgs.msg import JointState as RosJointState
from trajectory_msgs.msg import JointTrajectoryPoint

from ros2_manipulator_mcp.domain import (
    CollisionObject,
    CollisionPrimitive,
    DomainErrorCode,
    DomainResult,
    JointGoal,
    JointState,
    ManipulatorDescriptor,
    PlanningGroup,
    PlanningRequest,
    Pose,
    PrimitiveType,
    Quaternion,
    RobotState,
    Vector3,
)
from ros2_manipulator_mcp.ros.adapter import ManipulatorAdapter
from ros2_manipulator_mcp.ros.jazzy.adapter import JazzyManipulatorAdapter
from ros2_manipulator_mcp.ros.jazzy.conversions import (
    from_collision_object_message,
    from_joint_state_message,
    to_collision_object_message,
)
from ros2_manipulator_mcp.ros.jazzy.runtime import JazzyRosRuntime
from ros2_manipulator_mcp.ros.jazzy.settings import JazzyMoveItSettings


def _descriptor() -> ManipulatorDescriptor:
    group = PlanningGroup(
        "arm",
        ("joint1",),
        ("base", "tool"),
        "base",
        "tool",
    )
    return ManipulatorDescriptor("test-arm", "base", (group,))


class FakeRuntime:
    """Capture typed requests and return configured Jazzy responses."""

    def __init__(self) -> None:
        self.responses: dict[type, Any] = {}
        self.calls: list[tuple[type, str, Any, float]] = []
        self.close_count = 0
        self.error: Exception | None = None

    async def call_service(
        self,
        service_type: type,
        service_name: str,
        request: Any,
        timeout_seconds: float,
    ) -> Any:
        self.calls.append(
            (service_type, service_name, request, timeout_seconds)
        )
        if self.error is not None:
            raise self.error
        return self.responses[service_type]

    async def read_message(
        self,
        message_type: type,
        topic_name: str,
        timeout_seconds: float,
    ) -> Any:
        if self.error is not None:
            raise self.error
        message = RosJointState()
        message.name = ["joint1"]
        message.position = [0.0]
        return message

    def close(self) -> None:
        self.close_count += 1


def test_adapter_satisfies_composite_port_and_closes_once() -> None:
    """One replaceable adapter implements all ports and owns cleanup."""
    runtime = FakeRuntime()
    adapter = JazzyManipulatorAdapter(_descriptor(), runtime=runtime)

    assert isinstance(adapter, ManipulatorAdapter)
    adapter.close()
    adapter.close()

    assert runtime.close_count == 1


def test_installed_joint_and_collision_messages_round_trip_domain_values() -> None:
    """Core Jazzy message fields preserve typed state and primitive geometry."""
    joint_message = RosJointState()
    joint_message.name = ["joint1"]
    joint_message.position = [0.25]
    joint_message.velocity = [0.5]
    collision_object = CollisionObject(
        "fixture",
        CollisionPrimitive(PrimitiveType.CYLINDER, (0.4, 0.1)),
        Pose("base", Vector3(0.5, 0.0, 0.2), Quaternion(0, 0, 0, 1)),
    )

    state = from_joint_state_message(joint_message)
    converted_object = from_collision_object_message(
        to_collision_object_message(collision_object)
    )

    assert state == JointState(("joint1",), (0.25,), (0.5,))
    assert converted_object == collision_object


def test_ik_uses_verified_service_fields_and_maps_no_solution() -> None:
    """IK is collision-aware, bounded, and does not leak MoveIt codes."""
    runtime = FakeRuntime()
    response = GetPositionIK.Response()
    response.error_code.val = MoveItErrorCodes.NO_IK_SOLUTION
    response.error_code.message = "No solution"
    runtime.responses[GetPositionIK] = response
    adapter = JazzyManipulatorAdapter(_descriptor(), runtime=runtime)
    target = Pose("base", Vector3(0.2, 0.0, 0.3), Quaternion(0, 0, 0, 1))

    result = asyncio.run(
        adapter.compute_inverse_kinematics("arm", "tool", target)
    )
    service_type, name, request, timeout = runtime.calls[0]

    assert service_type is GetPositionIK
    assert name == "/compute_ik"
    assert request.ik_request.avoid_collisions is True
    assert request.ik_request.ik_link_name == "tool"
    assert request.ik_request.timeout.sec == 1
    assert timeout == 5.0
    assert result.error.code is DomainErrorCode.NO_SOLUTION


def test_motion_plan_uses_verified_plan_only_service_and_constraints() -> None:
    """Planning maps domain limits and the verified joint tolerance exactly."""
    runtime = FakeRuntime()
    response = GetMotionPlan.Response()
    response.motion_plan_response.error_code.val = MoveItErrorCodes.SUCCESS
    response.motion_plan_response.trajectory = RobotTrajectory()
    trajectory = response.motion_plan_response.trajectory.joint_trajectory
    trajectory.joint_names = ["joint1"]
    point = JointTrajectoryPoint()
    point.positions = [0.5]
    trajectory.points = [point]
    runtime.responses[GetMotionPlan] = response
    adapter = JazzyManipulatorAdapter(_descriptor(), runtime=runtime)
    request = PlanningRequest(
        JointGoal("arm", ("joint1",), (0.5,)),
        max_velocity_scaling_factor=0.5,
        max_acceleration_scaling_factor=0.4,
        allowed_planning_time_seconds=2.0,
        planning_attempts=2,
    )

    result = asyncio.run(adapter.plan_motion(request))
    service_type, name, ros_request, _ = runtime.calls[0]
    motion_request = ros_request.motion_plan_request
    constraint = motion_request.goal_constraints[0].joint_constraints[0]

    assert service_type is GetMotionPlan
    assert name == "/plan_kinematic_path"
    assert motion_request.group_name == "arm"
    assert motion_request.pipeline_id == ""
    assert motion_request.planner_id == ""
    assert motion_request.allowed_planning_time == 2.0
    assert motion_request.num_planning_attempts == 2
    assert constraint.tolerance_above == 0.0001
    assert constraint.tolerance_below == 0.0001
    assert result.value.joint_names == ("joint1",)


def test_adapter_translates_bounded_runtime_timeout() -> None:
    """A runtime timeout becomes a stable backend-neutral error."""
    runtime = FakeRuntime()
    runtime.error = TimeoutError("bounded timeout")
    adapter = JazzyManipulatorAdapter(_descriptor(), runtime=runtime)

    result = asyncio.run(
        adapter.check_state_validity(
            "arm",
            RobotState(JointState(("joint1",), (0.0,))),
        )
    )

    assert result.error.code is DomainErrorCode.TIMEOUT
    assert "bounded timeout" in result.error.message


def test_real_jazzy_runtime_constructs_and_cleans_up_without_robot(
    monkeypatch: Any,
    tmp_path: Path,
) -> None:
    """The installed rclpy lifecycle works without a running MoveIt graph."""
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path))
    runtime = JazzyRosRuntime("phase5_lifecycle_test")

    runtime.close()
    runtime.close()


def test_ros_imports_remain_inside_ros_adapter_boundary() -> None:
    """MCP, application, domain, and safety modules stay ROS-independent."""
    package_root = Path(__file__).parents[3] / "src" / "ros2_manipulator_mcp"
    forbidden_roots = {
        "rclpy",
        "moveit_msgs",
        "geometry_msgs",
        "sensor_msgs",
        "shape_msgs",
        "trajectory_msgs",
        "builtin_interfaces",
    }
    violations: list[str] = []

    for boundary in ("application", "domain", "safety", "mcp"):
        for path in (package_root / boundary).rglob("*.py"):
            tree = ast.parse(path.read_text(encoding="utf-8"))
            for node in ast.walk(tree):
                if isinstance(node, ast.Import):
                    roots = {name.name.split(".", 1)[0] for name in node.names}
                elif isinstance(node, ast.ImportFrom) and node.module:
                    roots = {node.module.split(".", 1)[0]}
                else:
                    continue
                if roots & forbidden_roots:
                    violations.append(str(path.relative_to(package_root)))

    assert violations == []
