class DummyPlanningMap:
    def __init__(self, scenario):
        self.world = scenario['world']

    def get_bounds(self):
        xmin, xmax, ymin, ymax, _, _ = self.get_3d_bounds()
        return xmin, xmax, ymin, ymax

    def get_3d_bounds(self):
        return tuple(self.world['bounds'])

    def get_obstacles_3d(self):
        return self.world.get('obstacles_3d', [])

    def get_ground_height(self, x, y):
        ground = self.world['ground']

        for region in ground.get('missing_regions', []):
            xmin, ymin = region['min']
            xmax, ymax = region['max']

            if xmin <= x <= xmax and ymin <= y <= ymax:
                return None

        if ground['type'] == 'flat':
            return ground['z']

        if ground['type'] == 'plane':
            return (
                ground['z_at_origin']
                + ground['slope_x'] * x
                + ground['slope_y'] * y
            )

        raise ValueError('Unknown ground type')

    def is_walk_forbidden(self, x, y):
        for region in self.world.get('walk_forbidden', []):
            xmin, ymin = region['min']
            xmax, ymax = region['max']

            if xmin <= x <= xmax and ymin <= y <= ymax:
                return True

        return False

    def get_walk_exclusion_regions(self):
        """Center-path exclusions, preserving the node validator policy."""
        return (
            self.world['ground'].get('missing_regions', [])
            + self.world.get('walk_forbidden', [])
        )

    def supports_flat_walk_edges(self):
        """Require flat terrain to certify swept edges."""
        return self.world['ground']['type'] == 'flat'

    def get_walk_cost_regions(self):
        return self.world.get('walk_cost_regions', [])

    def get_flat_ground_height(self):
        if self.world['ground']['type'] != 'flat':
            raise NotImplementedError('FLY ground sweeps require a flat test floor')
        return self.world['ground']['z']

    def get_unknown_regions(self):
        return self.world.get('unknown_regions', [])
