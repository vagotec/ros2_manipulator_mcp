"""OpenMANIPULATOR-X profile invariants."""

from ros2_manipulator_mcp.profiles import OPEN_MANIPULATOR_X_DESCRIPTOR


def test_gripper_exposes_only_its_independently_commanded_joint() -> None:
    """The right finger mimics the left and must not be execution-required."""
    gripper = next(
        group
        for group in OPEN_MANIPULATOR_X_DESCRIPTOR.groups
        if group.name == "gripper"
    )

    assert gripper.joint_names == ("gripper_left_joint",)
    assert "gripper_right_link" in gripper.link_names
