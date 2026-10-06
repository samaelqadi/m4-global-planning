"""Focused local query diagnostic; construction and broad searches are excluded."""

from copy import deepcopy
import json
from pathlib import Path
from statistics import median
from time import perf_counter_ns

from m4_global_planner.graph import MotionMode, Node
from m4_global_planner.height_grid_map import HeightGridMap
from m4_global_planner.representation.fly_validator import FlyStateValidator
from m4_global_planner.representation.states import PoseState
from m4_global_planner.representation.walk_validator import WalkStateValidator
from m4_global_planner.scenario_loader import default_scenario_path, load_scenario_data


def measure():
    data = load_scenario_data(default_scenario_path())
    rows = []
    for size in (50, 200):
        settings = deepcopy(data['height_grid_mock'])
        settings['unknown_policy'] = 'reject_map'
        settings['cells'] = [[{'ground_height': 0, 'occupied_intervals': (
            [[0, .5]] if x % 20 == 10 and y % 20 == 10 else [])}
            for x in range(size)] for y in range(size)]
        planning_map = HeightGridMap(settings)
        walk, fly = WalkStateValidator(data['test_parameters']), FlyStateValidator(data['test_parameters'])
        queries = {
            'known_check': planning_map.require_known_occupancy,
            'point_intervals': lambda: planning_map.get_occupied_height_intervals(2.5, 2.5),
            'walk_state': lambda: walk.is_valid(PoseState(2.5, 2.5, 0, 0), planning_map),
            'fly_edge': lambda: fly.is_edge_valid(Node(0, 2.5, 2.5, 1, MotionMode.FLY),
                                                Node(1, 3.5, 2.5, 1, MotionMode.FLY), planning_map),
        }
        for name, query in queries.items():
            query()
            samples = []
            for _ in range(7):
                start = perf_counter_ns()
                query()
                samples.append((perf_counter_ns() - start) / 1e6)
            rows.append({'size': size, 'query': name, 'samples_ms': samples,
                         'median_ms': median(samples)})
    return rows


if __name__ == '__main__':
    import sys
    Path(sys.argv[1]).write_text(json.dumps(measure(), indent=2) + '\n')
