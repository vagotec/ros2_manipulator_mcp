"""Typed configuration loading for the composition root."""

from dataclasses import dataclass
from math import isfinite
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
class ExecutionSettings:
    """Backend-neutral execution policy and bounded timeout values."""

    enabled: bool
    start_state_tolerance_rad: float
    state_freshness_sec: float
    require_exact_scene_revision: bool
    action_server_discovery_timeout_sec: float
    goal_acceptance_timeout_sec: float
    cancel_timeout_sec: float
    shutdown_timeout_sec: float
    timeout_multiplier: float
    timeout_margin_sec: float
    timeout_max_sec: float
    stabilization_timeout_sec: float
    stabilization_quiet_window_sec: float
    stabilization_position_delta_rad: float
    stabilization_velocity_rad_per_sec: float

    def __post_init__(self) -> None:
        positive = {
            "start_state_tolerance_rad": self.start_state_tolerance_rad,
            "state_freshness_sec": self.state_freshness_sec,
            "action_server_discovery_timeout_sec": (
                self.action_server_discovery_timeout_sec
            ),
            "goal_acceptance_timeout_sec": self.goal_acceptance_timeout_sec,
            "cancel_timeout_sec": self.cancel_timeout_sec,
            "shutdown_timeout_sec": self.shutdown_timeout_sec,
            "timeout_multiplier": self.timeout_multiplier,
            "timeout_max_sec": self.timeout_max_sec,
            "stabilization_timeout_sec": self.stabilization_timeout_sec,
            "stabilization_quiet_window_sec": self.stabilization_quiet_window_sec,
            "stabilization_position_delta_rad": self.stabilization_position_delta_rad,
            "stabilization_velocity_rad_per_sec": self.stabilization_velocity_rad_per_sec,
        }
        if any(not isfinite(value) or value <= 0 for value in positive.values()):
            raise ValueError("execution numeric limits must be finite and positive")
        if not isfinite(self.timeout_margin_sec) or self.timeout_margin_sec < 0:
            raise ValueError("execution timeout margin must be finite and non-negative")


@dataclass(frozen=True)
class Settings:
    """Application configuration."""

    runtime: RuntimeSettings
    execution: ExecutionSettings
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
    execution = data["execution"]
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
        execution=ExecutionSettings(
            enabled=bool(execution["enabled"]),
            start_state_tolerance_rad=float(
                execution["start_state_tolerance_rad"]
            ),
            state_freshness_sec=float(execution["state_freshness_sec"]),
            require_exact_scene_revision=bool(
                execution["require_exact_scene_revision"]
            ),
            action_server_discovery_timeout_sec=float(
                execution["action_server_discovery_timeout_sec"]
            ),
            goal_acceptance_timeout_sec=float(
                execution["goal_acceptance_timeout_sec"]
            ),
            cancel_timeout_sec=float(execution["cancel_timeout_sec"]),
            shutdown_timeout_sec=float(execution["shutdown_timeout_sec"]),
            timeout_multiplier=float(execution["execution_timeout_multiplier"]),
            timeout_margin_sec=float(execution["execution_timeout_margin_sec"]),
            timeout_max_sec=float(execution["execution_timeout_max_sec"]),
            stabilization_timeout_sec=float(execution["stabilization_timeout_sec"]),
            stabilization_quiet_window_sec=float(
                execution["stabilization_quiet_window_sec"]
            ),
            stabilization_position_delta_rad=float(
                execution["stabilization_position_delta_rad"]
            ),
            stabilization_velocity_rad_per_sec=float(
                execution["stabilization_velocity_rad_per_sec"]
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
