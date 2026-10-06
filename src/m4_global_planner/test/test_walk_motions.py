from math import pi, radians

from m4_global_planner.representation.states import PoseState
from m4_global_planner.representation.walk_edges import (
    WalkMotionEdgeGenerator, WalkMotionValidator,
)
from m4_global_planner.representation.walk_graph import WalkMotionNeighbors
import pytest
from test_straight_walk_edges import (
    graph_generator, node_id, pose, setup_scenario,
)


def build(parameters, planning_map, validator):
    edges = WalkMotionEdgeGenerator(parameters, WalkMotionValidator(parameters, validator))
    builder = graph_generator(parameters, validator, edges)
    builder.neighbors = WalkMotionNeighbors(parameters)
    return builder.generate_graph(planning_map), edges


def reachable(graph, source, target):
    """Boolean graph traversal only; no costs, planner, or ranking."""
    pending, visited = [source], {source}
    while pending:
        current = pending.pop()
        if current == target:
            return True
        for edge in graph.get_neighbors(current):
            if edge.target_id not in visited:
                visited.add(edge.target_id)
                pending.append(edge.target_id)
    return False


@pytest.mark.parametrize('enabled', [True, False])
def test_w04_reverse_escape_and_original_invalid_start(enabled):
    scenario, parameters, planning_map, validator, _ = setup_scenario('W04_dead_end_reverse')
    parameters['reverse_allowed'] = enabled
    graph, edges = build(parameters, planning_map, validator)
    original_start = PoseState(5, 1, 0, 0)
    original_goal = PoseState(1, 1, 0, pi)
    assert not validator.is_valid(original_start, planning_map)
    assert validator.is_valid(original_goal, planning_map)
    assert edges.generate_edge(original_start, original_goal, planning_map) is None
    assert not any((n.x, n.y, n.yaw) == (5, 1, 0) for n in graph.nodes.values())
    # Corrected scenario endpoints are used directly, with no snapping.
    start, end = pose(scenario['start']), pose(scenario['goal'])
    assert validator.is_valid(start, planning_map)
    assert validator.is_valid(end, planning_map)
    source, target = node_id(graph, start), node_id(graph, end)
    assert reachable(graph, source, target) == enabled
    assert not reachable(graph, source, node_id(graph, original_goal))
    near = PoseState(4, 1, 0, 0)
    reverse = edges.generate_edge(start, near, planning_map)
    assert (reverse is not None) == enabled
    if reverse:
        assert reverse.motion == 'reverse' and reverse.distance == 0.5
        assert reverse.cost == 0.5 + parameters['reverse_extra_cost']
        assert graph.edges[source][node_id(graph, near)].cost == reverse.cost
    route = [start, PoseState(1.5, 1, 0, 0), end]
    for a, b in zip(route, route[1:]):
        result = edges.generate_edge(a, b, planning_map)
        assert (result is not None) == enabled
        if result:
            assert result.motion == 'reverse'
            assert result.cost == (
                result.distance * parameters['straight_walk']['distance_cost_per_meter']
                + parameters['reverse_extra_cost']
            )
            assert graph.edges[node_id(graph, a)][node_id(graph, b)].cost == result.cost
    assert edges.generate_edge(start, PoseState(4.5, 1, 0, pi), planning_map) is None


def test_w03_reachability_and_explicit_route_edges():
    scenario, parameters, planning_map, validator, _ = setup_scenario('W03_ninety_degree_turn')
    graph, edges = build(parameters, planning_map, validator)
    start, end = pose(scenario['start']), pose(scenario['goal'])
    assert reachable(graph, node_id(graph, start), node_id(graph, end))
    # Witness route, not an optimized planner result: forward, pivot, forward.
    route = [start, PoseState(4, 2, 0, 0), PoseState(4, 2, 0, pi / 2), end]
    expected_motions = ('forward', 'pivot', 'forward')
    for state in route:
        assert validator.is_valid(state, planning_map)
    for a, b, motion in zip(route, route[1:], expected_motions):
        result = edges.generate_edge(a, b, planning_map)
        assert result is not None and result.motion == motion
        source, target = node_id(graph, a), node_id(graph, b)
        assert graph.edges[source][target].cost == result.cost
        assert result.distance == (0 if motion == 'pivot' else 2.5)
    # Also revalidate every generated edge in this obstacle-filled scenario.
    for outgoing in graph.edges.values():
        for edge in outgoing.values():
            a, b = graph.nodes[edge.source_id], graph.nodes[edge.target_id]
            result = edges.generate_edge(
                PoseState(a.x, a.y, a.z, a.yaw),
                PoseState(b.x, b.y, b.z, b.yaw), planning_map,
            )
            assert result is not None and result.cost == edge.cost


@pytest.mark.parametrize('enabled', [True, False])
def test_w05_pivot_permissions_cost_and_reachability(enabled):
    scenario, parameters, planning_map, validator, _ = setup_scenario('W05_pivot')
    parameters['pivot_allowed'] = enabled
    graph, edges = build(parameters, planning_map, validator)
    start, end = pose(scenario['from']), pose(scenario['to'])
    source, target = node_id(graph, start), node_id(graph, end)
    assert (target in graph.edges[source]) == enabled
    assert reachable(graph, source, target) == enabled
    edge = edges.generate_edge(start, end, planning_map)
    assert (edge is not None) == enabled
    if edge:
        assert edge.motion == 'pivot' and edge.distance == 0
        assert edge.cost == parameters['pivot_extra_cost']
        assert graph.edges[source][target].cost == edge.cost


def test_pivot_interior_sweep_and_vertical_clearance():
    scenario, parameters, planning_map, validator, _ = setup_scenario('W05_pivot')
    start, end = pose(scenario['from']), pose(scenario['to'])
    scenario['world']['obstacles_3d'] = [
        {'min': [2.845, 2.845, 0], 'max': [2.855, 2.855, 0.5]},
    ]
    assert validator.is_valid(start, planning_map)
    assert validator.is_valid(end, planning_map)
    assert not validator.is_valid(PoseState(2.5, 2.5, 0, radians(8)), planning_map)
    graph, edges = build(parameters, planning_map, validator)
    assert edges.generate_edge(start, end, planning_map) is None
    assert node_id(graph, end) not in graph.edges[node_id(graph, start)]
    scenario['world']['obstacles_3d'][0]['min'][2] = 0.6
    scenario['world']['obstacles_3d'][0]['max'][2] = 1
    assert edges.generate_edge(start, end, planning_map) is not None
    assert edges.generate_edge(PoseState(0.4, 2.5, 0, 0), PoseState(0.4, 2.5, 0, pi / 2),
                               planning_map) is None


def test_configured_motion_penalties_and_pivot_neighbors():
    scenario, parameters, planning_map, validator, _ = setup_scenario('W05_pivot')
    parameters['reverse_extra_cost'] = 7
    parameters['pivot_extra_cost'] = 4
    parameters['pivot_walk']['neighbor_heading_steps'] = [1]
    graph, edges = build(parameters, planning_map, validator)
    start, end = pose(scenario['from']), pose(scenario['to'])
    source, target = node_id(graph, start), node_id(graph, end)
    assert target not in graph.edges[source]
    assert reachable(graph, source, target)  # two validated 45-degree pivots
    mid = PoseState(2.5, 2.5, 0, pi / 4)
    assert graph.edges[source][node_id(graph, mid)].cost == 4
    reverse = edges.generate_edge(start, PoseState(2, 2.5, 0, 0), planning_map)
    assert reverse.cost == 7.5
    parameters['pivot_walk']['max_angle_deg'] = 45
    limited = WalkMotionEdgeGenerator(parameters, WalkMotionValidator(parameters, validator))
    assert limited.generate_edge(start, end, planning_map) is None


@pytest.mark.parametrize('name,exists', [
    ('W01_forward', True), ('W02_sideways_motion', False),
    ('W06_blocked_between_free_endpoints', False),
])
def test_enabled_motions_retain_connection_regressions(name, exists):
    scenario, parameters, planning_map, validator, _ = setup_scenario(name)
    graph, edges = build(parameters, planning_map, validator)
    start, end = pose(scenario['from']), pose(scenario['to'])
    assert (node_id(graph, end) in graph.edges[node_id(graph, start)]) == exists
    if name in ('W01_forward', 'W06_blocked_between_free_endpoints'):
        assert reachable(graph, node_id(graph, start), node_id(graph, end)) == exists
    if name == 'W06_blocked_between_free_endpoints':
        assert edges.generate_edge(end, start, planning_map) is None
    for outgoing in graph.edges.values():
        for edge in outgoing.values():
            a, b = graph.nodes[edge.source_id], graph.nodes[edge.target_id]
            result = edges.generate_edge(
                PoseState(a.x, a.y, a.z, a.yaw),
                PoseState(b.x, b.y, b.z, b.yaw), planning_map,
            )
            assert result is not None and edge.cost == result.cost


@pytest.mark.parametrize('key,value', [
    ('reverse_allowed', 'false'), ('pivot_allowed', 1),
    ('reverse_extra_cost', -1), ('pivot_extra_cost', float('nan')),
])
def test_invalid_motion_configuration(key, value):
    _, parameters, _, validator, _ = setup_scenario('W05_pivot')
    parameters[key] = value
    with pytest.raises(ValueError):
        WalkMotionEdgeGenerator(parameters, WalkMotionValidator(parameters, validator))


def test_wrapped_pivot_and_unsupported_terrain():
    scenario, parameters, planning_map, validator, _ = setup_scenario('W05_pivot')
    edges = WalkMotionEdgeGenerator(parameters, WalkMotionValidator(parameters, validator))
    start = PoseState(2.5, 2.5, 0, 0)
    end = PoseState(2.5, 2.5, 0, 3 * pi / 2)
    assert edges.generate_edge(start, end, planning_map).motion == 'pivot'
    reverse_target = PoseState(2, 2.5, 0, 0)
    scenario['world']['walk_cost_regions'] = [
        {'min': [2, 2], 'max': [3, 3], 'cost': 5},
    ]
    for target in (end, reverse_target):
        with pytest.raises(NotImplementedError, match='terrain'):
            edges.generate_edge(start, target, planning_map)
    scenario['world']['ground'] = {
        'type': 'plane', 'z_at_origin': 0, 'slope_x': 0, 'slope_y': 0,
    }
    for target in (end, reverse_target):
        assert edges.generate_edge(start, target, planning_map) is None


@pytest.mark.parametrize('key,value', [
    ('max_angle_deg', 0), ('max_angle_deg', float('nan')),
    ('neighbor_heading_steps', [0]), ('neighbor_heading_steps', [1, 1]),
    ('neighbor_heading_steps', [8]),
])
def test_invalid_pivot_configuration(key, value):
    _, parameters, _, validator, _ = setup_scenario('W05_pivot')
    parameters['pivot_walk'][key] = value
    with pytest.raises(ValueError):
        WalkMotionValidator(parameters, validator)
        WalkMotionNeighbors(parameters)
