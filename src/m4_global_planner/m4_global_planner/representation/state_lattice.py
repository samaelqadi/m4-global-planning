from math import radians

from m4_global_planner.representation.interfaces import StateGenerator
from m4_global_planner.representation.states import PoseState


class StateLatticeGenerator(StateGenerator):
    def __init__(self, spatial_resolution, heading_resolution_deg):
        if spatial_resolution <= 0:
            raise ValueError('Spatial resolution must be positive')

        if heading_resolution_deg <= 0 or heading_resolution_deg > 360:
            raise ValueError(
                'Heading resolution must be between 0 and 360 degrees'
            )

        self.spatial_resolution = spatial_resolution
        self.heading_resolution_deg = heading_resolution_deg

    def generate_headings(self):
        headings = []
        angle = 0.0

        while angle < 360.0:
            headings.append(radians(angle))
            angle += self.heading_resolution_deg

        return headings

    def generate_states(self, planning_map, validator):
        xmin, xmax, ymin, ymax = planning_map.get_bounds()
        headings = self.generate_headings()
        states = []

        x = xmin

        while x <= xmax:
            y = ymin

            while y <= ymax:
                z = planning_map.get_ground_height(x, y)

                if z is not None:
                    for heading in headings:
                        state = PoseState(
                            x=x,
                            y=y,
                            z=z,
                            heading=heading,
                        )

                        if validator.is_valid(state, planning_map):
                            states.append(state)

                y += self.spatial_resolution

            x += self.spatial_resolution

        return states
