"""Independent shortest-path audits over the explicit graphs actually searched."""

from copy import deepcopy
import heapq
import math

from m4_global_planner.astar import astar
from m4_global_planner.fly_benchmark import SCENARIOS, setup as fly_setup
from m4_global_planner.graph import MotionMode, Node
from m4_global_planner.representation.fly_graph import build_graph, validate_route
from m4_global_planner.scenario_loader import load_scenario_data
from m4_global_planner.walk_comparison import (
    explicit_lattice, Motions, plan as walk_plan, pose,
)
import pytest
from test_straight_walk_edges import DATA as WALK_DATA, setup_scenario


def dijkstra_path(graph, start, goal):
    """Compute shortest cost independently of the planner's A* implementation."""
    costs, parents = {start: 0.0}, {}
    queue = [(0.0, start)]
    while queue:
        cost, node_id = heapq.heappop(queue)
        if cost != costs[node_id]:
            continue
        if node_id == goal:
            path = [goal]
            while path[-1] != start:
                path.append(parents[path[-1]])
            return cost, list(reversed(path))
        for edge in graph.get_neighbors(node_id):
            candidate = cost + edge.cost
            if candidate < costs.get(edge.target_id, math.inf):
                costs[edge.target_id] = candidate
                parents[edge.target_id] = node_id
                heapq.heappush(queue, (candidate, edge.target_id))
    return math.inf, []


def dijkstra_cost(graph, start, goal):
    return dijkstra_path(graph, start, goal)[0]


@pytest.mark.parametrize('approach', ['lattice', 'grid', 'roadmap'])
def test_walk_astar_matches_dijkstra_on_captured_successors(approach):
    scenario, parameters, planning_map, _, _ = setup_scenario('G02_equal_cost_routes')
    settings = deepcopy(WALK_DATA['walk_benchmark'])
    parameters['walk_terrain_cost_mode'] = settings['terrain_cost_mode']
    start, goal = pose(scenario['start']), pose(scenario['goal'])
    result = walk_plan(
        approach, parameters, settings, planning_map, start, goal,
        seed=11, capture_search_graph=True,
    )
    assert result['_search_graph_complete']
    graph, states = result['_search_graph'], result['_search_states']
    start_id, goal_id = result['_search_start_id'], result['_search_goal_id']
    independent = dijkstra_cost(graph, start_id, goal_id)
    assert result['success'] and math.isfinite(independent)
    assert result['cost'] == pytest.approx(independent)
    assert states[start_id] == start and states[goal_id] == goal

    if approach == 'lattice':
        complete, _, endpoint_ids, _ = explicit_lattice(
            parameters, settings, Motions(parameters, planning_map), start, goal,
        )
        full_cost = dijkstra_cost(complete, endpoint_ids[start], endpoint_ids[goal])
        assert astar(complete, endpoint_ids[start],
                     endpoint_ids[goal])['total_cost'] == pytest.approx(full_cost)
        assert result['cost'] == pytest.approx(full_cost)

    # Rebuild the independent graph path and check every primitive motion.
    _, path = dijkstra_path(graph, start_id, goal_id)
    segments = result['_search_segments']
    route = [states[path[0]]]
    for source, target in zip(path, path[1:]):
        segment = segments[source, target]
        assert segment[0] == states[source] and segment[-1] == states[target]
        route.extend(segment[1:])
    motions = Motions(parameters, planning_map)
    edge_cost = 0.0
    for first, second in zip(route, route[1:]):
        edge = motions.edge(first, second)
        assert edge is not None
        edge_cost += edge.cost
    assert route[0] == start and route[-1] == goal
    assert edge_cost == pytest.approx(independent)


@pytest.mark.parametrize('approach', ['adaptive_lattice', 'sparse_roadmap'])
@pytest.mark.parametrize('case_index', [4, 5])
def test_fly_astar_matches_dijkstra_per_leg(approach, case_index):
    data = load_scenario_data(SCENARIOS)
    case = data['fly_benchmark']['cases'][case_index]
    planning_map, builder = fly_setup(data, case)
    # Build the identical configured mandatory-anchor snapshot used by planning.
    settings = data['fly_benchmark']
    anchors = builder.build(
        planning_map,
        {str(i): Node(i, *xyz, MotionMode.FLY) for i, xyz in enumerate(case['positions'])},
    )
    graph, mapping, _ = build_graph(
        approach, planning_map, builder, anchors, settings['graph'],
        settings['sweep']['coarse'], seed=47,
    )
    endpoints = dict(anchors.endpoints)
    ids = [mapping[endpoints[str(i)].node_id] for i in range(len(case['positions']))]
    scale = settings['graph']['distance_cost_per_meter']
    for start, goal in zip(ids, ids[1:]):
        result = astar(graph, start, goal, heuristic_scale=scale)
        expected = dijkstra_cost(graph, start, goal)
        assert result['success'] == math.isfinite(expected)
        if result['success']:
            assert result['total_cost'] == pytest.approx(expected)
            validate_route(
                graph, result['path'], [start, goal], builder.fly, planning_map,
                scale, result['total_cost'],
            )
