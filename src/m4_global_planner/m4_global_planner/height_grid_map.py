"""Mock height-grid adapter; not a claim about the future upstream costmap format."""

from copy import deepcopy
from math import floor, isfinite

from m4_global_planner.representation.interfaces import MapRegion, PlanningMap


class HeightGridMap(PlanningMap):
    """Rows indexed y/x; heights are absolute map z, occupancy is explicit."""

    def __init__(self, settings):
        self._settings = deepcopy(settings)
        self.unknown_policy = settings['unknown_policy']
        if self.unknown_policy not in ('reject_map', 'block'):
            raise NotImplementedError('Unknown policy must be reject_map or block')
        if settings['inflation_policy'] != 'raw_geometry':
            raise NotImplementedError('Preinflated grids are unsupported')
        if settings['height_reference'] != 'map_z':
            raise NotImplementedError('Only absolute map_z heights are supported')
        self.resolution = settings['resolution']
        self.origin = tuple(settings['origin'])
        self.vertical = tuple(settings['vertical_bounds'])
        self.default_cost = settings['default_cost']
        if not isfinite(self.resolution) or self.resolution <= 0:
            raise ValueError('Grid resolution must be finite and positive')
        if len(self.origin) != 2 or not all(isfinite(v) for v in self.origin):
            raise ValueError('Grid origin must be finite x/y')
        if (len(self.vertical) != 2 or not all(isfinite(v) for v in self.vertical)
                or self.vertical[0] >= self.vertical[1]):
            raise ValueError('Vertical bounds must be finite and ordered')
        if not isfinite(self.default_cost) or self.default_cost < 0:
            raise ValueError('Default cost must be finite and nonnegative')
        self._cells = self._settings['cells']
        if (not self._cells or not self._cells[0]
                or any(len(row) != len(self._cells[0]) for row in self._cells)):
            raise ValueError('Grid must have rectangular nonempty rows')
        self.revision = 0
        # Validate all supplied cell data, including data irrelevant to a query.
        for row in self._cells:
            for cell in row:
                if not isinstance(cell, dict):
                    raise ValueError('Grid cell must be a mapping')
                for field in ('ground_height', 'obstacle_top_height'):
                    value = cell.get(field)
                    if value is not None and (
                            not isfinite(value)
                            or not self.vertical[0] <= value <= self.vertical[1]):
                        raise ValueError('Height must be finite and within vertical bounds')
                cost = cell.get('cost', self.default_cost)
                if not isfinite(cost) or cost < 0:
                    raise ValueError('Cell cost must be finite and nonnegative')
                intervals = cell.get('occupied_intervals')
                if intervals is not None:
                    for interval in intervals:
                        if (len(interval) != 2 or not all(isfinite(v) for v in interval)
                                or not self.vertical[0] <= interval[0] <= interval[1]
                                <= self.vertical[1]):
                            raise ValueError('Occupied interval must be finite and ordered')
                    top = cell.get('obstacle_top_height')
                    if top is not None and top != max((b for a, b in intervals), default=None):
                        raise ValueError('Obstacle top conflicts with occupied intervals')

        # Immutable geometry caches and cell bins belong to this snapshot/revision.
        self._cache = {name: [] for name in ('occupied', 'unknown', 'exclusions', 'costs')}
        self._bins = {name: {} for name in self._cache}
        for iy, row in enumerate(self._cells):
            for ix, cell in enumerate(row):
                lower = (self.origin[0] + ix * self.resolution,
                         self.origin[1] + iy * self.resolution)
                upper = tuple(v + self.resolution for v in lower)
                unknown = (cell.get('ground_height') is None
                           or cell.get('occupied_intervals') is None or cell.get('unknown', False))
                regions = {name: [] for name in self._cache}
                if unknown:
                    regions['unknown'].append(MapRegion(lower, upper))
                intervals = ([(self.vertical[0], self.vertical[1])]
                             if unknown and self.unknown_policy == 'block'
                             else cell.get('occupied_intervals') or ())
                regions['occupied'] = [MapRegion((*lower, a), (*upper, b)) for a, b in intervals]
                if (cell.get('walk_forbidden', False) or cell.get('ground_height') is None
                        or unknown and self.unknown_policy == 'block'):
                    regions['exclusions'].append(MapRegion(lower, upper))
                cost = cell.get('cost', self.default_cost)
                if cost:
                    regions['costs'].append(MapRegion(lower, upper, cost, upper_open=True))
                for name, values in regions.items():
                    if values:
                        self._bins[name][ix, iy] = tuple(values)
                    self._cache[name].extend(values)
        self._cache = {name: tuple(values) for name, values in self._cache.items()}
        heights = {cell.get('ground_height') for row in self._cells for cell in row}
        self._known_heights = heights - {None}
        self._flat = (len(self._known_heights) == 1
                      and (None not in heights or self.unknown_policy == 'block'))

    @property
    def settings(self):
        return deepcopy(self._settings)

    @property
    def cells(self):
        return deepcopy(self._cells)

    def get_3d_bounds(self):
        x, y = self.origin
        return (x, x + len(self._cells[0]) * self.resolution,
                y, y + len(self._cells) * self.resolution, *self.vertical)

    def _cell(self, x, y):
        if not isfinite(x) or not isfinite(y):
            return None
        ix = floor((x - self.origin[0]) / self.resolution)
        iy = floor((y - self.origin[1]) / self.resolution)
        if 0 <= iy < len(self._cells) and 0 <= ix < len(self._cells[0]):
            return self._cells[iy][ix]
        return None

    def get_ground_height(self, x, y):
        cell = self._cell(x, y)
        return cell.get('ground_height') if cell is not None else None

    def get_occupied_height_intervals(self, x, y) -> tuple | None:
        """Half-open cell ownership; None means outside or unknown, () known clear."""
        cell = self._cell(x, y)
        if (cell is None or cell.get('ground_height') is None
                or cell.get('occupied_intervals') is None or cell.get('unknown', False)):
            return None
        return tuple(tuple(interval) for interval in cell['occupied_intervals'])

    def get_obstacle_top_height(self, x, y):
        intervals = self.get_occupied_height_intervals(x, y)
        return max((b for a, b in intervals), default=None) if intervals is not None else None

    def get_occupied_volumes(self):
        return self._cache['occupied']

    def get_unknown_regions(self):
        return self._cache['unknown']

    def get_walk_exclusion_regions(self):
        return self._cache['exclusions']

    def get_walk_cost_regions(self):
        return self._cache['costs']

    def _query(self, name, lower, upper):
        # Include the previous cell at a lower seam for closed collision geometry.
        lo = [max(0, floor((v - o) / self.resolution) - 1)
              for v, o in zip(lower[:2], self.origin)]
        dimensions = (len(self._cells[0]), len(self._cells))
        hi = [min(n - 1, floor((v - o) / self.resolution))
              for v, o, n in zip(upper[:2], self.origin, dimensions)]
        regions = (r for iy in range(lo[1], hi[1] + 1) for ix in range(lo[0], hi[0] + 1)
                   for r in self._bins[name].get((ix, iy), ()))
        return self._overlapping(regions, lower, upper)

    def query_occupied_volumes(self, lower, upper):
        return self._query('occupied', lower, upper)

    def query_walk_exclusions(self, lower, upper):
        return self._query('exclusions', lower, upper)

    def query_walk_costs(self, lower, upper):
        return self._query('costs', lower, upper)

    def is_walk_forbidden(self, x, y):
        return bool(self.query_walk_exclusions((x, y), (x, y)))

    def supports_flat_walk_edges(self):
        return self._flat

    def get_flat_ground_height(self):
        if not self._flat:
            raise NotImplementedError('FLY ground sweeps require a flat test floor')
        return next(iter(self._known_heights))

    def apply_update(self, update):
        if not isinstance(update, dict) or set(update) != {'set_cells'}:
            raise NotImplementedError('Grid updates support set_cells only')
        settings = deepcopy(self._settings)
        if not isinstance(update['set_cells'], list):
            raise ValueError('Cell replacements must be a list')
        for replacement in update['set_cells']:
            if (not isinstance(replacement, dict)
                    or not isinstance(replacement.get('index'), (list, tuple))
                    or len(replacement['index']) != 2 or 'cell' not in replacement):
                raise ValueError('Cell replacement requires index and cell')
            ix, iy = replacement['index']
            if (type(ix) is not int or type(iy) is not int
                    or not 0 <= iy < len(self._cells) or not 0 <= ix < len(self._cells[0])):
                raise ValueError('Updated cell index is outside the grid')
            settings['cells'][iy][ix] = deepcopy(replacement['cell'])
        result = HeightGridMap(settings)
        result.revision = self.revision + 1
        return result
