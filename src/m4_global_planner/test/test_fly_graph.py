from copy import deepcopy
from math import dist

from m4_global_planner.astar import astar
from m4_global_planner.fly_benchmark import SCENARIOS, setup as fly_setup
from m4_global_planner.graph import MotionMode, Node
from m4_global_planner.representation.fly_graph import (
    BudgetExceeded, build_graph, plan, validate_route,
)
from m4_global_planner.representation.fly_validator import position
from m4_global_planner.scenario_loader import load_scenario_data
import pytest


@pytest.fixture
def data():
    return load_scenario_data(SCENARIOS)


def prepared(data, index):
    case = data['fly_benchmark']['cases'][index]
    planning_map, builder = fly_setup(data, case)
    anchors = builder.build(planning_map, {str(i): Node(i, *xyz, MotionMode.FLY)
                                           for i, xyz in enumerate(case['positions'])})
    return case, planning_map, builder, anchors


@pytest.mark.parametrize('approach', ['adaptive_lattice', 'sparse_roadmap'])
@pytest.mark.parametrize('index', [0, 1, 2, 3, 4, 5, 6, 7])
def test_shared_fly_graph(data, approach, index):
    case, planning_map, builder, anchors = prepared(data, index)
    settings = data['fly_benchmark']
    graph, mapping, attempts = build_graph(approach, planning_map, builder, anchors,
                                           settings['graph'], settings['sweep']['coarse'], 11)
    assert {position(graph.nodes[mapping[n.node_id]]) for n in anchors.fly_nodes} == {
        position(n) for n in anchors.fly_nodes}
    for node in graph.nodes.values():
        assert builder.fly.is_valid(node, planning_map)
        for edge in graph.get_neighbors(node.node_id):
            target = graph.nodes[edge.target_id]
            assert builder.fly.is_edge_valid(node, target, planning_map)
            assert edge.cost == pytest.approx(dist(position(node), position(target)))
    endpoints = dict(anchors.endpoints)
    start_index = str(len(case['positions']) - 1)
    start = mapping[endpoints['0'].node_id]
    goal = mapping[endpoints[start_index].node_id]
    if index == 1:
        assert goal not in graph.edges[start]
        assert not builder.fly.is_edge_valid(graph.nodes[start], graph.nodes[goal], planning_map)
    if index == 6:
        assert not astar(graph, start, goal)['success']
    else:
        result = astar(graph, start, goal)
        # A coverage failure is recorded; successful paths must always be valid.
        if result['success']:
            validate_route(graph, result['path'], [start, goal], builder.fly, planning_map, 1,
                           result['total_cost'])


def test_shared_snapshot_deterministic_and_budgets(data):
    case, planning_map, builder, anchors = prepared(data, 0)
    settings = data['fly_benchmark']
    args = ('sparse_roadmap', planning_map, builder, anchors, settings['graph'],
            settings['sweep']['coarse'], 29)
    first, mapping, _ = build_graph(*args)
    second, other, _ = build_graph(*args)
    assert first.nodes == second.nodes and first.edges == second.edges and mapping == other
    small = deepcopy(settings['graph'])
    small['max_nodes'] = len(anchors.fly_nodes)
    with pytest.raises(BudgetExceeded):
        build_graph('adaptive_lattice', planning_map, builder, anchors, small,
                    settings['sweep']['coarse'], 11)
    small = deepcopy(settings['graph'])
    small['connection_radius'] = 7
    with pytest.raises(ValueError):
        build_graph('adaptive_lattice', planning_map, builder, anchors, small,
                    settings['sweep']['coarse'], 11)


def test_cost_scale_and_route_rejection(data):
    case, planning_map, builder, anchors = prepared(data, 0)
    settings = data['fly_benchmark']
    graph, mapping, _ = build_graph('adaptive_lattice', planning_map, builder, anchors,
                                    settings['graph'], settings['sweep']['coarse'], 11)
    endpoints = dict(anchors.endpoints)
    ids = [mapping[endpoints[str(i)].node_id] for i in range(2)]
    result = astar(graph, *ids)
    assert result['success']
    with pytest.raises(ValueError):
        validate_route(graph, result['path'], ids, builder.fly, planning_map, 1,
                       result['total_cost'] + 1)
    scaled = deepcopy(settings['graph'])
    scaled['distance_cost_per_meter'] = .2
    output = plan('adaptive_lattice', planning_map, builder, anchors, scaled,
                  settings['sweep']['coarse'], 11, ['0', '1'])
    assert output['cost'] == pytest.approx(result['total_cost'] * .2)
