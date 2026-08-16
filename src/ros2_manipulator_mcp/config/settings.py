"""Typed configuration loading for the composition root."""

from dataclasses import dataclass
import os
from pathlib import Path
import tomllib

from ros2_manipulator_mcp.domain import PrimitiveType
from ros2_manipulator_mcp.safety import SafetyPolicy


CONFIG_ENV_VAR = "ROS2_MANIPULATOR_MCP_CONFIG"
PACKAGED_DEFAULT_CONFIG = Path(__file__).with_name("default.toml")


@dataclass(frozen=True)
class RuntimeSettings:
    """Backend selection and bounded runtime defaults."""

    backend: str
    reference_manipulator: str
    service_timeout_seconds: float


@dataclass(frozen=True)
class SafetySettings:
    """Top-level safety policy switches."""

    physical_execution_enabled: bool


@dataclass(frozen=True)
class Settings:
    """Application configuration."""

    runtime: RuntimeSettings
    safety: SafetySettings
    policy: SafetyPolicy


def resolve_config_path(config_path: Path | None = None) -> Path:
    """Resolve an explicit, environment, or packaged configuration path."""
    candidate = config_path

    if candidate is None and (environment_path := os.environ.get(CONFIG_ENV_VAR)):
        candidate = Path(environment_path)

    resolved = (candidate or PACKAGED_DEFAULT_CONFIG).expanduser().resolve()

    if not resolved.is_file():
        raise FileNotFoundError(f"Configuration file was not found: {resolved}")

    return resolved


def load_settings(config_path: Path | None = None) -> Settings:
    """Load and minimally validate TOML settings."""
    with resolve_config_path(config_path).open("rb") as config_file:
        data = tomllib.load(config_file)

    runtime = data["runtime"]
    safety = data["safety"]
    policy = data["policy"]
    timeout = float(runtime["service_timeout_seconds"])

    if timeout <= 0:
        raise ValueError("service_timeout_seconds must be greater than zero")

    return Settings(
        runtime=RuntimeSettings(
            backend=str(runtime["backend"]),
            reference_manipulator=str(runtime["reference_manipulator"]),
            service_timeout_seconds=timeout,
        ),
        safety=SafetySettings(
            physical_execution_enabled=bool(
                safety["physical_execution_enabled"]
            ),
        ),
        policy=SafetyPolicy(
            policy_id=str(policy["policy_id"]),
            allowed_planning_groups=tuple(policy["allowed_planning_groups"]),
            max_planning_time_seconds=float(
                policy["max_planning_time_seconds"]
            ),
            max_planning_attempts=int(policy["max_planning_attempts"]),
            max_velocity_scaling_factor=float(
                policy["max_velocity_scaling_factor"]
            ),
            max_acceleration_scaling_factor=float(
                policy["max_acceleration_scaling_factor"]
            ),
            max_joint_targets=int(policy["max_joint_targets"]),
            max_cartesian_waypoints=int(policy["max_cartesian_waypoints"]),
            min_cartesian_step=float(policy["min_cartesian_step"]),
            max_cartesian_step=float(policy["max_cartesian_step"]),
            min_cartesian_completion_fraction=float(
                policy["min_cartesian_completion_fraction"]
            ),
            require_collision_avoidance=bool(
                policy["require_collision_avoidance"]
            ),
            allowed_object_id_prefixes=tuple(
                policy["allowed_object_id_prefixes"]
            ),
            max_collision_objects=int(policy["max_collision_objects"]),
            max_primitive_dimension=float(policy["max_primitive_dimension"]),
            allowed_primitive_types=tuple(
                PrimitiveType(value)
                for value in policy["allowed_primitive_types"]
            ),
            allowed_scene_frames=tuple(policy["allowed_scene_frames"]),
            allow_object_replace=bool(policy["allow_object_replace"]),
            require_scene_revision=bool(policy["require_scene_revision"]),
        ),
    )
