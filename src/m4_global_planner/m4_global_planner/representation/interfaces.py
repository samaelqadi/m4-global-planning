from abc import ABC, abstractmethod
from dataclasses import dataclass
from math import isfinite


@dataclass(frozen=True, slots=True)
class MapRegion:
    """Adapter-neutral closed geometry; costs may use half-open upper bounds."""

    lower: tuple
    upper: tuple
    cost: float = 0.0
    upper_open: bool = False

    def __post_init__(self):
        if (len(self.lower) not in (2, 3) or len(self.lower) != len(self.upper)
                or not all(isfinite(v) for v in (*self.lower, *self.upper))
                or any(a > b for a, b in zip(self.lower, self.upper))):
            raise ValueError('Region bounds must be finite and ordered')
        if not isfinite(self.cost) or self.cost < 0:
            raise ValueError('Terrain density must be finite and nonnegative')


class PlanningMap(ABC):
    """Geometry queries, independent of storage; revisioned native adapter updates."""

    revision: int
    unknown_policy = 'reject_map'

    def get_bounds(self):
        return self.get_3d_bounds()[:4]

    @abstractmethod
    def get_3d_bounds(self):
        pass

    @abstractmethod
    def get_ground_height(self, x, y):
        pass

    @abstractmethod
    def get_occupied_volumes(self):
        """Return conservative, uninflated 3D MapRegions, not storage records."""
        pass

    @abstractmethod
    def get_walk_exclusion_regions(self):
        pass

    @abstractmethod
    def get_walk_cost_regions(self):
        pass

    @abstractmethod
    def get_unknown_regions(self):
        """Include missing vertical coverage; no heights means unknown, not free."""
        pass

    @abstractmethod
    def is_walk_forbidden(self, x, y):
        pass

    @abstractmethod
    def supports_flat_walk_edges(self):
        pass

    @abstractmethod
    def get_flat_ground_height(self):
        pass

    @abstractmethod
    def apply_update(self, update):
        """Return an independent new revision; payload belongs to the adapter."""
        pass

    def require_known_occupancy(self):
        if self.unknown_policy == 'reject_map' and self.get_unknown_regions():
            raise NotImplementedError('Unknown occupancy maps are unsupported')

    @staticmethod
    def _overlapping(regions, lower, upper):
        return tuple(r for r in regions if all(
            a <= v and b >= u for a, b, u, v in zip(r.lower, r.upper, lower, upper)
        ))

    def query_occupied_volumes(self, lower, upper):
        return self._overlapping(self.get_occupied_volumes(), lower, upper)

    def query_walk_exclusions(self, lower, upper):
        return self._overlapping(self.get_walk_exclusion_regions(), lower, upper)

    def query_walk_costs(self, lower, upper):
        return self._overlapping(self.get_walk_cost_regions(), lower, upper)

    def get_occupied_height_intervals(self, x, y) -> tuple | None:
        xmin, xmax, ymin, ymax = self.get_bounds()
        if not xmin <= x < xmax or not ymin <= y < ymax:
            return None
        if any(all(a <= v <= b for a, v, b in zip(r.lower, (x, y), r.upper))
               for r in self.get_unknown_regions()):
            return None
        return tuple((r.lower[2], r.upper[2]) for r in self.get_occupied_volumes()
                     if r.lower[0] <= x <= r.upper[0] and r.lower[1] <= y <= r.upper[1])

    def get_obstacle_top_height(self, x, y):
        intervals = self.get_occupied_height_intervals(x, y)
        if intervals is None:
            return None
        return max((upper for _, upper in intervals), default=None)


class StateValidator(ABC):
    @abstractmethod
    def is_valid(self, state, planning_map):
        # check supported geometric feasibility
        pass


class StateGenerator(ABC):
    @abstractmethod
    def generate_states(self, planning_map, validator):
        # generate valid planning states
        pass
