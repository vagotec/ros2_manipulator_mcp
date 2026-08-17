"""OpenMANIPULATOR-X profile verified against ROBOTIS release 4.1.3."""

from ros2_manipulator_mcp.domain import ManipulatorDescriptor, PlanningGroup


OPEN_MANIPULATOR_X_DESCRIPTOR = ManipulatorDescriptor(
    manipulator_id="open_manipulator_x",
    model_frame="world",
    groups=(
        PlanningGroup(
            name="arm",
            joint_names=("joint1", "joint2", "joint3", "joint4"),
            link_names=(
                "link1",
                "link2",
                "link3",
                "link4",
                "link5",
                "end_effector_link",
            ),
            base_link="link1",
            end_effector_link="end_effector_link",
        ),
        PlanningGroup(
            name="gripper",
            # The right finger is a URDF mimic joint. MoveIt plans and the
            # gripper controller command only the independent left joint.
            joint_names=("gripper_left_joint",),
            link_names=(
                "link5",
                "gripper_left_link",
                "gripper_right_link",
            ),
            base_link="link5",
        ),
    ),
)

__all__ = ["OPEN_MANIPULATOR_X_DESCRIPTOR"]
