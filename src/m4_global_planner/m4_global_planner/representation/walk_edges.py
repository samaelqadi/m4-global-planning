from dataclasses import dataclass
from math import atan2, cos, hypot, isfinite, pi, sin

from m4_global_planner.representation.geometry import (
    bounds_xy,
    disk_overlaps_box,
    overlaps_box,
    segment_intersects_region,
    segment_region_interval,
    straight_sweep_corners,
)
from m4_global_planner.representation.states import PoseState
from m4_global_planner.representation.walk_costs import StraightWalkCost, WalkMotionCost


@dataclass(frozen=True, slots=True)
class StraightWalkEdge:
    source: PoseState
    target: PoseState
    distance: float
    cost: float
    motion: str = 'forward'


class StraightWalkEdgeValidator:
    """Forward, fixed-heading, flat-ground translations only."""

    def __init__(self, parameters, state_validator):
        settings = parameters['straight_walk']
        self.max_distance = settings['max_distance']
        self.heading_tolerance = settings['heading_tolerance_rad']
        self.lateral_tolerance = settings['lateral_tolerance']
        values = (
            self.max_distance, self.heading_tolerance, self.lateral_tolerance,
        )
        if not all(isfinite(value) and value > 0 for value in values):
            raise ValueError('Straight WALK limits must be finite and positive')
        if self.heading_tolerance >= pi / 2:
            raise ValueError('Heading tolerance must be less than pi/2')
        self.state_validator = state_validator

    def is_valid(self, start, end, planning_map):
        if not planning_map.supports_flat_walk_edges():
            return False

        angle = end.heading - start.heading
        if abs(atan2(sin(angle), cos(angle))) > self.heading_tolerance:
            return False
        dx, dy = end.x - start.x, end.y - start.y
        distance = hypot(dx, dy)
        forward = dx * cos(start.heading) + dy * sin(start.heading)
        lateral = -dx * sin(start.heading) + dy * cos(start.heading)
        if (
            distance == 0 or distance > self.max_distance or forward <= 0
            or abs(lateral) > self.lateral_tolerance
        ):
            return False
        if not all(
            self.state_validator.is_valid(state, planning_map)
            for state in (start, end)
        ):
            return False

        if any(
            segment_intersects_region(start, end, region)
            for region in planning_map.query_walk_exclusions(*bounds_xy(
                [(start.x, start.y), (end.x, end.y)]))
        ):
            return False

        validator = self.state_validator
        # Inflate by numerical acceptance tolerances to avoid underchecking
        # slightly misaligned endpoints or slightly differing orientations.
        margin = (
            abs(lateral)
            + hypot(validator.length, validator.width) * self.heading_tolerance
        )
        sweep = straight_sweep_corners(
            start, end, validator.length + 2 * margin,
            validator.width + 2 * margin,
        )
        xmin, xmax, ymin, ymax, _, _ = planning_map.get_3d_bounds()
        if not all(xmin <= x <= xmax and ymin <= y <= ymax for x, y in sweep):
            return False
        for obstacle in planning_map.query_occupied_volumes(*bounds_xy(sweep)):
            if (
                max(start.z, end.z) + validator.height < obstacle.lower[2]
                or min(start.z, end.z) > obstacle.upper[2]
            ):
                continue
            if overlaps_box(sweep, start.heading, obstacle):
                return False
        return True


class StraightWalkEdgeGenerator:
    """Validate explicit candidate pairs; no all-pairs graph construction."""

    def __init__(self, parameters, validator, cost=None):
        self.validator = validator
        self.cost = cost if cost is not None else StraightWalkCost(parameters)

    def generate_edge(self, start, end, planning_map):
        if not self.validator.is_valid(start, end, planning_map):
            return None
        if any(
            segment_intersects_region(start, end, region)
            for region in planning_map.query_walk_costs(*bounds_xy(
                [(start.x, start.y), (end.x, end.y)]))
        ):
            raise NotImplementedError('WALK terrain edge costs are pending')
        distance = hypot(end.x - start.x, end.y - start.y)
        return StraightWalkEdge(
            start, end, distance, self.cost.calculate(distance),
        )


class WalkMotionValidator:
    """Reuse straight sweeps; pivots use a conservative full-rotation disk."""

    def __init__(self, parameters, state_validator):
        self.straight = StraightWalkEdgeValidator(parameters, state_validator)
        self.state_validator = state_validator
        self.reverse_allowed = parameters['reverse_allowed']
        self.pivot_allowed = parameters['pivot_allowed']
        if any(type(value) is not bool for value in (
            self.reverse_allowed, self.pivot_allowed,
        )):
            raise ValueError('Motion permissions must be booleans')
        angle = parameters['pivot_walk']['max_angle_deg']
        if not isfinite(angle) or not 0 < angle <= 180:
            raise ValueError('Pivot angle must be finite and in (0, 180]')
        self.max_pivot_angle = angle * pi / 180

    def motion(self, start, end, planning_map):
        if self.straight.is_valid(start, end, planning_map):
            return 'forward'
        if self.reverse_allowed and self.straight.is_valid(end, start, planning_map):
            return 'reverse'
        if not self.pivot_allowed or not planning_map.supports_flat_walk_edges():
            return None
        if (start.x, start.y, start.z) != (end.x, end.y, end.z):
            return None
        angle = abs(atan2(sin(end.heading - start.heading), cos(end.heading - start.heading)))
        if not self.straight.heading_tolerance < angle <= (
            self.max_pivot_angle + self.straight.heading_tolerance
        ):
            return None
        if not all(self.state_validator.is_valid(state, planning_map) for state in (start, end)):
            return None
        validator = self.state_validator
        radius = hypot(validator.length, validator.width) / 2
        xmin, xmax, ymin, ymax, _, _ = planning_map.get_3d_bounds()
        if not (
            xmin <= start.x - radius and start.x + radius <= xmax
            and ymin <= start.y - radius and start.y + radius <= ymax
        ):
            return None
        for obstacle in planning_map.query_occupied_volumes(
                (start.x - radius, start.y - radius),
                (start.x + radius, start.y + radius)):
            if start.z + validator.height < obstacle.lower[2] or start.z > obstacle.upper[2]:
                continue
            if disk_overlaps_box(start.x, start.y, radius, obstacle):
                return None
        return 'pivot'

    def is_valid(self, start, end, planning_map):
        return self.motion(start, end, planning_map) is not None


class WalkMotionEdgeGenerator(StraightWalkEdgeGenerator):
    def __init__(self, parameters, validator):
        super().__init__(parameters, validator, WalkMotionCost(parameters))
        self.terrain_mode = parameters.get('walk_terrain_cost_mode', 'unsupported')
        if self.terrain_mode not in ('unsupported', 'distance_density'):
            raise ValueError('Unknown WALK terrain cost mode')

    def generate_edge(self, start, end, planning_map):
        motion = self.validator.motion(start, end, planning_map)
        if motion is None:
            return None
        distance = hypot(end.x - start.x, end.y - start.y)
        cost = self.cost.calculate(distance, motion)
        for region in planning_map.query_walk_costs(*bounds_xy(
                [(start.x, start.y), (end.x, end.y)])):
            interval = segment_region_interval(start, end, region)
            if interval is None:
                continue
            if self.terrain_mode == 'unsupported':
                raise NotImplementedError('WALK terrain edge costs are pending')
            density = region.cost
            if not isfinite(density) or density < 0:
                raise ValueError('Terrain density must be finite and nonnegative')
            cost += distance * (interval[1] - interval[0]) * density
        return StraightWalkEdge(start, end, distance, cost, motion)
