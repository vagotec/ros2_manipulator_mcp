"""Shared domain invariant helpers."""

from collections.abc import Iterable
from math import isfinite


def identifier(value: str, field_name: str) -> str:
    """Return a non-empty identifier without changing it."""
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{field_name} must be a non-empty string")
    return value


def identifiers(values: Iterable[str], field_name: str) -> tuple[str, ...]:
    """Return a non-empty tuple of unique identifiers."""
    result = tuple(identifier(value, field_name) for value in values)
    if not result:
        raise ValueError(f"{field_name} must not be empty")
    if len(set(result)) != len(result):
        raise ValueError(f"{field_name} must contain unique values")
    return result


def finite(value: float, field_name: str) -> float:
    """Return a finite floating-point value."""
    result = float(value)
    if not isfinite(result):
        raise ValueError(f"{field_name} must be finite")
    return result


def finite_values(values: Iterable[float], field_name: str) -> tuple[float, ...]:
    """Return finite floating-point values."""
    return tuple(finite(value, field_name) for value in values)


def positive(value: float, field_name: str) -> float:
    """Return a finite value greater than zero."""
    result = finite(value, field_name)
    if result <= 0:
        raise ValueError(f"{field_name} must be greater than zero")
    return result


def non_negative(value: float, field_name: str) -> float:
    """Return a finite value greater than or equal to zero."""
    result = finite(value, field_name)
    if result < 0:
        raise ValueError(f"{field_name} must be non-negative")
    return result


def scaling_factor(value: float, field_name: str) -> float:
    """Return a scaling factor in the closed interval above zero through one."""
    result = finite(value, field_name)
    if not 0 < result <= 1:
        raise ValueError(f"{field_name} must be within (0, 1]")
    return result
