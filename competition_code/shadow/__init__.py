"""Observational controllers that never receive vehicle-control authority."""

from .longitudinal import ShadowLongitudinalPlanner

__all__ = ["ShadowLongitudinalPlanner"]
