from math import isclose, isfinite

from m4_global_planner.representation.geometry import (
    footprint_corners,
    overlaps_box,
)
from m4_global_planner.representation.interfaces import StateValidator


class WalkStateValidator(StateValidator):
    """Test convention: x/y is body center; z is ground-contact height."""

    def __init__(self, parameters):
        footprint = parameters['walk_footprint']
        self.length = footprint['length']
        self.width = footprint['width']
        self.height = footprint['height']

        if not all(
            isfinite(value) and value > 0
            for value in (self.length, self.width, self.height)
        ):
            raise ValueError('Footprint dimensions must be finite and positive')

    def is_valid(self, state, planning_map):
        if not all(
            isfinite(value)
            for value in (state.x, state.y, state.z, state.heading)
        ):
            return False

        ground = planning_map.get_ground_height(state.x, state.y)
        if ground is None or not isclose(state.z, ground):
            return False

        if planning_map.is_walk_forbidden(state.x, state.y):
            return False

        corners = footprint_corners(
            state.x,
            state.y,
            state.heading,
            self.length,
            self.width,
        )

        xmin, xmax, ymin, ymax, zmin, zmax = planning_map.get_3d_bounds()

        if not all(
            xmin <= x <= xmax and ymin <= y <= ymax
            for x, y in corners
        ):
            return False

        if state.z < zmin or state.z + self.height > zmax:
            return False

        for obstacle in planning_map.get_obstacles_3d():
            bottom = obstacle['min'][2]
            top = obstacle['max'][2]

            if state.z + self.height < bottom or state.z > top:
                continue

            if overlaps_box(corners, state.heading, obstacle):
                return False

        return True
