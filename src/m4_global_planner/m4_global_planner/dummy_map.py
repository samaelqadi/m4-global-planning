from copy import deepcopy
from math import isfinite

from m4_global_planner.representation.interfaces import MapRegion, PlanningMap


class DummyPlanningMap(PlanningMap):
    def __init__(self, scenario):
        self.world = (scenario['world'] if 'world' in scenario
                      else deepcopy(scenario['initial_world']))
        self.unknown_policy = self.world.get('unknown_policy', 'reject_map')
        if self.unknown_policy not in ('reject_map', 'block'):
            raise NotImplementedError('Unknown policy must be reject_map or block')
        self.revision = 0

    def get_bounds(self):
        xmin, xmax, ymin, ymax, _, _ = self.get_3d_bounds()
        return xmin, xmax, ymin, ymax

    def get_3d_bounds(self):
        return tuple(self.world['bounds'])

    def get_obstacles_3d(self):
        return self.world.get('obstacles_3d', [])

    def get_occupied_volumes(self):
        regions = [MapRegion(tuple(r['min']), tuple(r['max']))
                   for r in self.get_obstacles_3d()]
        if self.unknown_policy == 'block':
            zmin, zmax = self.get_3d_bounds()[4:]
            regions += [MapRegion((*r.lower[:2], zmin), (*r.upper[:2], zmax))
                        for r in self.get_unknown_regions()]
        return regions

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

        if self.unknown_policy == 'block':
            return any(r.lower[0] <= x <= r.upper[0] and r.lower[1] <= y <= r.upper[1]
                       for r in self.get_unknown_regions())
        return False

    def get_walk_exclusion_regions(self):
        """Center-path exclusions, preserving the node validator policy."""
        regions = (self.world['ground'].get('missing_regions', [])
                   + self.world.get('walk_forbidden', []))
        values = [MapRegion(tuple(r['min']), tuple(r['max'])) for r in regions]
        if self.unknown_policy == 'block':
            values += [MapRegion(r.lower[:2], r.upper[:2]) for r in self.get_unknown_regions()]
        return values

    def supports_flat_walk_edges(self):
        """Require flat terrain to certify swept edges."""
        return self.world['ground']['type'] == 'flat'

    def get_walk_cost_regions(self):
        return [MapRegion(tuple(r['min']), tuple(r['max']), r['cost'])
                for r in self.world.get('walk_cost_regions', [])]

    def get_flat_ground_height(self):
        if self.world['ground']['type'] != 'flat':
            raise NotImplementedError('FLY ground sweeps require a flat test floor')
        return self.world['ground']['z']

    def get_unknown_regions(self):
        return [MapRegion(tuple(r['min']), tuple(r['max']))
                for r in self.world.get('unknown_regions', [])]

    def apply_update(self, update):
        """Return a new snapshot; reject unknown or malformed update operations."""
        supported = {'add_obstacles', 'remove_walk_forbidden', 'set_walk_cost_regions'}
        if not isinstance(update, dict) or not set(update) <= supported:
            raise NotImplementedError('Unsupported map update operation')
        world = deepcopy(self.world)
        for operation, regions in update.items():
            if not isinstance(regions, list):
                raise ValueError('Update regions must be a list')
            dimensions = 3 if operation == 'add_obstacles' else 2
            for region in regions:
                if not isinstance(region, dict):
                    raise ValueError('Update region must be a mapping')
                lower, upper = region.get('min', ()), region.get('max', ())
                if (not isinstance(lower, (list, tuple))
                        or not isinstance(upper, (list, tuple))
                        or len(lower) != dimensions or len(upper) != dimensions
                        or not all(isinstance(v, (int, float)) and isfinite(v)
                                   for v in (*lower, *upper))
                        or any(a > b for a, b in zip(lower, upper))):
                    raise ValueError('Update region bounds must be finite and ordered')
                if operation == 'set_walk_cost_regions':
                    cost = region.get('cost')
                    if not isinstance(cost, (int, float)) or not isfinite(cost) or cost < 0:
                        raise ValueError('Terrain cost must be finite and nonnegative')
            if operation == 'add_obstacles':
                world.setdefault('obstacles_3d', []).extend(deepcopy(regions))
            elif operation == 'remove_walk_forbidden':
                for region in regions:
                    if region not in world.get('walk_forbidden', []):
                        raise ValueError('Removed WALK exclusion is absent')
                    world['walk_forbidden'].remove(region)
            else:
                world['walk_cost_regions'] = deepcopy(regions)
        result = DummyPlanningMap({'world': world})
        result.revision = self.revision + 1
        return result
