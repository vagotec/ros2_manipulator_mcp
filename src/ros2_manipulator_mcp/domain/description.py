"""Manipulator and planning-group descriptions."""

from dataclasses import dataclass

from ros2_manipulator_mcp.domain._validation import identifier, identifiers


@dataclass(frozen=True)
class PlanningGroup:
    """A named kinematic group exposed by a manipulator backend."""

    name: str
    joint_names: tuple[str, ...]
    link_names: tuple[str, ...]
    base_link: str
    end_effector_link: str | None = None

    def __post_init__(self) -> None:
        identifier(self.name, "planning group name")
        joints = identifiers(self.joint_names, "joint_names")
        links = identifiers(self.link_names, "link_names")
        identifier(self.base_link, "base_link")
        if self.base_link not in links:
            raise ValueError("base_link must belong to link_names")
        if self.end_effector_link is not None:
            identifier(self.end_effector_link, "end_effector_link")
            if self.end_effector_link not in links:
                raise ValueError("end_effector_link must belong to link_names")
        object.__setattr__(self, "joint_names", joints)
        object.__setattr__(self, "link_names", links)


@dataclass(frozen=True)
class ManipulatorDescriptor:
    """Backend-neutral description of one manipulator."""

    manipulator_id: str
    model_frame: str
    groups: tuple[PlanningGroup, ...]

    def __post_init__(self) -> None:
        identifier(self.manipulator_id, "manipulator_id")
        identifier(self.model_frame, "model_frame")
        groups = tuple(self.groups)
        if not groups:
            raise ValueError("groups must not be empty")
        names = tuple(group.name for group in groups)
        if len(set(names)) != len(names):
            raise ValueError("planning group names must be unique")
        object.__setattr__(self, "groups", groups)
