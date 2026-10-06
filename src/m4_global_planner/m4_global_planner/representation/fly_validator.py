from math import dist, isfinite

from m4_global_planner.graph import MotionMode
from m4_global_planner.representation.geometry import footprint_corners, segment_box_interval
from m4_global_planner.representation.interfaces import StateValidator


def require_policy(settings, key, supported):
    if settings[key] != supported:
        raise NotImplementedError(f'Unsupported {key}: {settings[key]}')


def position(state):
    return state.x, state.y, state.z


class BoxBody:
    """Bottom-center box shared by flight and morphing geometry."""

    def __init__(self, dimensions, settings):
        length, width, height = (dimensions[key] for key in ('length', 'width', 'height'))
        if not all(isfinite(value) and value > 0 for value in (length, width, height)):
            raise ValueError('Body dimensions must be finite and positive')
        margin = settings['safety_margin']
        self.tolerance = settings['collision_tolerance']
        if not all(isfinite(value) and value >= 0 for value in (margin, self.tolerance)):
            raise ValueError('Margins and tolerance must be finite and nonnegative')
        self.lower = (-length / 2 - margin, -width / 2 - margin, -margin)
        self.upper = (length / 2 + margin, width / 2 + margin, height + margin)

    def is_clear(self, start, end, planning_map, floor):
        xmin, xmax, ymin, ymax, zmin, zmax = planning_map.get_3d_bounds()
        lower, upper = (xmin, ymin, max(zmin, floor)), (xmax, ymax, zmax)
        if not all(isfinite(value) for value in lower + upper):
            raise ValueError('Map bounds and floor must be finite')
        if any(a > b for a, b in zip(lower, upper)):
            raise ValueError('Map bounds are inverted')
        for anchor in (start, end):
            if not all(a <= value + lo and value + hi <= b
                       for value, lo, hi, a, b in zip(
                           anchor, self.lower, self.upper, lower, upper
                       )):
                return False
        for obstacle in planning_map.get_obstacles_3d():
            lo, hi = obstacle['min'], obstacle['max']
            if len(lo) != 3 or len(hi) != 3 or not all(isfinite(value) for value in (*lo, *hi)):
                raise ValueError('Obstacle bounds must be finite 3D coordinates')
            if any(a > b for a, b in zip(lo, hi)):
                raise ValueError('Obstacle bounds are inverted')
            expanded_lower = tuple(a - body - self.tolerance for a, body in zip(lo, self.upper))
            expanded_upper = tuple(b - body + self.tolerance for b, body in zip(hi, self.lower))
            if segment_box_interval(start, end, expanded_lower, expanded_upper) is not None:
                return False
        return True


class FlyStateValidator(StateValidator):
    """Geometry only: fixed level box, independent altitude, no flight dynamics."""

    def __init__(self, parameters):
        settings = parameters['fly_validation']
        policies = {
            'pose_anchor': 'bottom_center', 'body_policy': 'fixed_axis_aligned_box',
            'yaw_policy': 'absent', 'map_policy': 'complete_test_boxes',
            'ground_policy': 'flat_floor', 'ground_contact_policy': 'allow',
            'unknown_policy': 'reject_map', 'obstacle_contact_policy': 'collision',
            'motion_policy': 'straight_translation', 'takeoff_landing_policy': 'unsupported',
        }
        for key, supported in policies.items():
            require_policy(settings, key, supported)
        self.body = BoxBody(parameters['fly_footprint'], settings)
        self.altitude_min, self.altitude_max = settings['altitude_limits']
        self.minimum = settings['min_edge_distance']
        self.maximum = settings['max_edge_distance']
        if not all(isfinite(value) for value in (
            self.altitude_min, self.altitude_max, self.minimum, self.maximum,
        )) or not (self.altitude_min <= self.altitude_max and 0 <= self.minimum < self.maximum):
            raise ValueError('Altitude and edge limits must be finite and ordered')

    def floor(self, planning_map):
        if planning_map.get_unknown_regions():
            raise NotImplementedError('Unknown occupancy maps are unsupported')
        floor = planning_map.get_flat_ground_height()
        if not isfinite(floor):
            raise ValueError('Floor height must be finite')
        return floor

    def is_valid(self, state, planning_map):
        if getattr(state, 'mode', None) != MotionMode.FLY:
            return False
        if state.yaw is not None:
            raise NotImplementedError('Flight yaw and attitude are unsupported')
        xyz = position(state)
        if not all(isfinite(value) for value in xyz):
            return False
        floor = self.floor(planning_map)
        if not self.altitude_min <= state.z <= self.altitude_max:
            return False
        return self.body.is_clear(xyz, xyz, planning_map, floor)

    def is_edge_valid(self, start, end, planning_map, motion='straight_translation'):
        if motion != 'straight_translation':
            raise NotImplementedError(f'Unsupported flight motion: {motion}')
        if not self.is_valid(start, planning_map) or not self.is_valid(end, planning_map):
            return False
        a, b = position(start), position(end)
        if not self.minimum < dist(a, b) <= self.maximum:
            return False
        return self.body.is_clear(a, b, planning_map, self.floor(planning_map))


class TransitionValidator:
    """Stationary envelope check; takeoff and landing are separate operations."""

    def __init__(self, parameters, walk_validator, fly_validator):
        settings = parameters['transition_validation']
        require_policy(settings, 'motion_policy', 'stationary_morphing')
        require_policy(settings, 'envelope_policy', 'fixed_axis_aligned_swept_box')
        require_policy(settings, 'anchor_match_policy', 'exact')
        require_policy(settings, 'takeoff_landing_policy', 'unsupported')
        self.directions = settings['supported_directions']
        if not self.directions or any(
            direction not in ('walk_to_fly', 'fly_to_walk') for direction in self.directions
        ):
            raise NotImplementedError('Unsupported transition direction policy')
        self.walk = walk_validator
        self.fly = fly_validator
        self.body = BoxBody(parameters['morphing_envelope'], parameters['fly_validation'])
        if not all(a <= b and c <= d for a, b, c, d in zip(
            self.body.lower, self.fly.body.lower, self.fly.body.upper, self.body.upper,
        )):
            raise ValueError('Morphing envelope must contain the FLY body')

    def is_valid(self, walk_pose, fly_node, planning_map, direction='walk_to_fly'):
        if direction not in self.directions:
            raise NotImplementedError(f'Unsupported transition direction: {direction}')
        if position(walk_pose) != position(fly_node):
            return False
        if (not self.walk.is_valid(walk_pose, planning_map)
                or not self.fly.is_valid(fly_node, planning_map)):
            return False
        corners = footprint_corners(0, 0, walk_pose.heading, self.walk.length, self.walk.width)
        if not all(self.body.lower[0] <= x <= self.body.upper[0]
                   and self.body.lower[1] <= y <= self.body.upper[1] for x, y in corners
                   ) or self.walk.height > self.body.upper[2]:
            raise NotImplementedError('Morphing envelope does not cover the WALK body')
        xyz = position(walk_pose)
        return self.body.is_clear(xyz, xyz, planning_map, self.fly.floor(planning_map))
