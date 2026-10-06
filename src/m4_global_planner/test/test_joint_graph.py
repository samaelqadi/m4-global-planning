"""Joint graph audits on existing T scenarios, with independent Dijkstra."""

from copy import deepcopy
from math import isfinite

from m4_global_planner.graph import Edge, MotionMode, Node
from m4_global_planner.representation.anchors import MandatoryAnchorBuilder
from m4_global_planner.representation.fly_validator import FlyStateValidator, TransitionValidator
from m4_global_planner.representation.joint_graph import JointPlanner
from m4_global_planner.representation.state_lattice import StateLatticeGenerator
from m4_global_planner.representation.walk_graph import (
    ForwardWalkGraphGenerator, WalkMotionNeighbors,
)
from m4_global_planner.representation.walk_validator import WalkStateValidator
import pytest
from test_anchors import endpoint
from test_graph_optimality import dijkstra_path
from test_straight_walk_edges import DATA, setup_scenario


def prepared(name, penalty=None, directions=None, endpoints=None, approach='adaptive_lattice',
             rate=None, search_settings=None):
    scenario, parameters, planning_map, _, _ = setup_scenario(name)
    settings = deepcopy(DATA['joint_planner'])
    settings.update(search_settings or {})
    parameters['walk_terrain_cost_mode'] = settings['walk_terrain_cost_mode']
    if penalty is not None:
        parameters['transition_extra_cost'] = penalty
    if directions is not None:
        parameters['transition_validation']['supported_directions'] = directions
    if rate is not None:
        settings['fly_graph']['distance_cost_per_meter'] = rate
    settings['fly_approach'] = approach
    walk, fly = WalkStateValidator(parameters), FlyStateValidator(parameters)
    builder = MandatoryAnchorBuilder(parameters, walk, fly,
                                     TransitionValidator(parameters, walk, fly))
    if endpoints is None:
        if 'start' in scenario:
            endpoints = {key: endpoint(scenario[key], i)
                         for i, key in enumerate(('start', 'goal'))}
        else:
            endpoints = {'start': Node(0, 1, 2, 0, MotionMode.WALK, 0),
                         'goal': Node(1, 4 if name.startswith(('T01', 'T02')) else 9,
                                      2, 0, MotionMode.WALK, 0)}
    return JointPlanner(parameters, settings, planning_map, builder, endpoints, seed=11)


def audit(planner):
    result = planner.plan()
    assert result['route_kind'] == 'geometric' and result['executable'] is False
    start, goal = (planner.endpoints[key] for key in ('start', 'goal'))
    cost, path = dijkstra_path(planner.graph, start, goal)
    assert result['success'] == isfinite(cost)
    if result['success']:
        assert result['total_cost'] == pytest.approx(cost)
        planner.validate_route(path, start, goal, cost)
    return result


def morphs(planner, result):
    return [(planner.graph.nodes[a], planner.graph.nodes[b])
            for a, b in zip(result['path'], result['path'][1:])
            if planner.graph.nodes[a].mode != planner.graph.nodes[b].mode]


@pytest.mark.parametrize('name', ['T01_open_transition', 'T02_blocked_morphing_space',
                                  'T03_only_one_transition_zone',
                                  'T04_multiple_transition_choices', 'T05_flight_bridge',
                                  'T06_walk_only'])
def test_t_scenarios_complete_graph_optimality(name):
    planner = prepared(name)
    result = audit(planner)
    assert result['success']
    if name.startswith('T05'):
        changes = morphs(planner, result)
        assert len(changes) == 2
        assert changes[0][0].x < 4 and changes[1][1].x > 6
    elif name.startswith('T06'):
        assert not planner.anchors.transitions
        assert not morphs(planner, result)
    elif name.startswith('T03'):
        assert all(a.walk_pose.x == 5 for a in planner.anchors.transitions)
    elif name.startswith('T02'):
        assert not any(a.walk_pose.x == a.walk_pose.y == 2.5
                       for a in planner.anchors.transitions)


def test_transition_penalty_changes_whole_route_and_locations():
    cheap = prepared('T04_multiple_transition_choices', penalty=0, rate=.2)
    low = audit(cheap)
    expensive = prepared('T04_multiple_transition_choices', penalty=100, rate=.2)
    high = audit(expensive)
    assert len(morphs(cheap, low)) == 2
    assert not morphs(expensive, high)
    assert low['total_cost'] < high['total_cost']
    assert len({(a.walk_pose.x, a.walk_pose.y) for a in cheap.anchors.transitions}) > 2
    # Forcing the first transition later must increase the whole route cost.
    for source in list(cheap.graph.edges):
        for target in list(cheap.graph.edges[source]):
            a, b = cheap.graph.nodes[source], cheap.graph.nodes[target]
            if a.mode == MotionMode.WALK and b.mode == MotionMode.FLY and a.x < 5:
                del cheap.graph.edges[source][target]
    later = audit(cheap)
    assert later['total_cost'] > low['total_cost']
    assert morphs(cheap, later)[0][0].x >= 5


@pytest.mark.parametrize('approach', ['adaptive_lattice', 'sparse_roadmap'])
@pytest.mark.parametrize('penalty', [0, 3, 17])
def test_required_bridge_penalty(approach, penalty):
    planner = prepared('T05_flight_bridge', penalty=penalty, approach=approach)
    result = audit(planner)
    assert result['success'] and len(morphs(planner, result)) == 2
    baseline = audit(prepared('T05_flight_bridge', penalty=0, approach=approach))
    assert result['total_cost'] == pytest.approx(baseline['total_cost'] + 2 * penalty)


def test_directed_morphs_and_unreachable_bridge():
    planner = prepared('T05_flight_bridge', directions=['walk_to_fly'])
    assert not audit(planner)['success']
    assert all(not (planner.graph.nodes[a].mode == MotionMode.FLY
                    and planner.graph.nodes[b].mode == MotionMode.WALK)
               for a, edges in planner.graph.edges.items() for b in edges)


@pytest.mark.parametrize('modes', [(MotionMode.WALK, MotionMode.FLY),
                                   (MotionMode.FLY, MotionMode.WALK),
                                   (MotionMode.FLY, MotionMode.FLY)])
def test_exact_off_lattice_endpoints_and_modes(modes):
    endpoints = {key: Node(i, x, 2.13, 0 if mode == MotionMode.WALK else 1.07,
                           mode, .123 if mode == MotionMode.WALK else None)
                 for i, (key, x, mode) in enumerate(zip(('start', 'goal'), (1.13, 4.17), modes))}
    planner = prepared('T01_open_transition', endpoints=endpoints, rate=.2)
    assert audit(planner)['success']
    for key, node in endpoints.items():
        actual = planner.graph.nodes[planner.endpoints[key]]
        assert (actual.x, actual.y, actual.z, actual.mode, actual.yaw) == (
            node.x, node.y, node.z, node.mode, node.yaw)


def test_route_audit_rejects_corrupt_cost_and_stale_geometry():
    planner = prepared('T05_flight_bridge')
    result = audit(planner)
    route, cost = result['path'], result['total_cost']
    start, goal = route[0], route[-1]
    with pytest.raises(ValueError, match='total cost'):
        planner.validate_route(route, start, goal, cost + 1)
    source, target = route[:2]
    edge = planner.graph.edges[source][target]
    planner.graph.edges[source][target] = Edge(source, target, edge.cost + 1)
    with pytest.raises(ValueError, match='edge cost'):
        planner.validate_route(route, start, goal, cost)
    planner.graph.edges[source][target] = edge
    a, _ = morphs(planner, result)[0]
    planner.map.world['obstacles_3d'] = [
        {'min': [a.x - .1, a.y - .1, .75], 'max': [a.x + .1, a.y + .1, .9]},
    ]
    with pytest.raises(ValueError, match='morph'):
        planner.validate_route(route, start, goal, cost)


@pytest.mark.parametrize('penalty', [-1, float('nan'), float('inf')])
def test_invalid_penalty(penalty):
    with pytest.raises(ValueError, match='Transition penalty'):
        prepared('T01_open_transition', penalty=penalty)


def test_blocked_morph_requires_travel_to_a_clear_anchor():
    endpoints = {'start': Node(0, 2.5, 2.5, 0, MotionMode.WALK, 0),
                 'goal': Node(1, 2.5, 2.5, 0, MotionMode.FLY)}
    planner = prepared('T02_blocked_morphing_space', endpoints=endpoints)
    result = audit(planner)
    assert result['success']
    changes = morphs(planner, result)
    assert len(changes) == 1
    assert (changes[0][0].x, changes[0][0].y) != (2.5, 2.5)
    assert result['total_cost'] > planner.penalty


def test_identical_endpoint_and_exact_walk_lattice_reference():
    planner = prepared('T05_flight_bridge')
    _, parameters, _, _, _ = setup_scenario('T05_flight_bridge')
    generator = ForwardWalkGraphGenerator(
        WalkMotionNeighbors(parameters),
        StateLatticeGenerator(parameters['spatial_resolution'],
                              parameters['heading_resolution_deg']),
        planner.motions.validator, planner.motions.edges,
    )
    reference = generator.generate_graph(planner.map)
    assert all(planner.graph.nodes[i] == node for i, node in reference.nodes.items())
    for source, edges in reference.edges.items():
        assert {target: edge for target, edge in planner.graph.edges[source].items()
                if planner.graph.nodes[target].mode == MotionMode.WALK} == edges
    planner.endpoints['goal'] = planner.endpoints['start']
    result = audit(planner)
    assert result['total_cost'] == 0 and len(result['path']) == 1


@pytest.mark.parametrize('method', ['dijkstra', 'astar', 'weighted_astar'])
@pytest.mark.parametrize('case', ['walk_only', 'flight_bridge', 'blocked_morph',
                                  'cheap_morph', 'expensive_morph'])
def test_public_search_configuration(method, case, monkeypatch):
    from m4_global_planner.representation import joint_graph

    settings = {'search_method': method, 'weighted_astar_weight': 2.0}
    endpoints = None
    name, penalty, rate = 'T04_multiple_transition_choices', 0, .2
    if case == 'walk_only':
        name = 'T06_walk_only'
    elif case == 'flight_bridge':
        name = 'T05_flight_bridge'
    elif case == 'blocked_morph':
        name = 'T02_blocked_morphing_space'
        endpoints = {'start': Node(0, 2.5, 2.5, 0, MotionMode.WALK, 0),
                     'goal': Node(1, 2.5, 2.5, 0, MotionMode.FLY)}
    elif case == 'expensive_morph':
        penalty = 100
    calls = []
    search = joint_graph.dijkstra if method == 'dijkstra' else joint_graph.astar

    def recorded_search(*args, **kwargs):
        calls.append(kwargs)
        return search(*args, **kwargs)

    monkeypatch.setattr(joint_graph, 'dijkstra' if method == 'dijkstra' else 'astar',
                        recorded_search)
    planner = prepared(name, penalty=penalty, rate=rate, endpoints=endpoints,
                       search_settings=settings)
    result = planner.plan()
    assert result['success'] and len(calls) == 1
    assert result['route_kind'] == 'geometric' and result['executable'] is False
    if method == 'dijkstra':
        assert calls == [{}]
    else:
        assert calls[0]['heuristic_weight'] == (2 if method == 'weighted_astar' else 1)
        assert calls[0]['heuristic_scale'] == planner.scale
    start, goal = (planner.endpoints[key] for key in ('start', 'goal'))
    planner.validate_route(result['path'], start, goal, result['total_cost'])
    minimum, _ = dijkstra_path(planner.graph, start, goal)
    if method != 'weighted_astar':
        assert result['total_cost'] == pytest.approx(minimum)
    else:
        assert result['total_cost'] >= minimum - 1e-9
    changes = morphs(planner, result)
    if case in ('walk_only', 'expensive_morph'):
        assert not changes
    elif case in ('flight_bridge', 'cheap_morph'):
        assert len(changes) == 2
    else:
        assert len(changes) == 1
        assert (changes[0][0].x, changes[0][0].y) != (2.5, 2.5)


def test_default_search_without_configuration():
    planner = prepared('T06_walk_only')
    assert planner.search_method == 'dijkstra'
    assert audit(planner)['total_cost'] == pytest.approx(8)
    # Existing callers omitting the new settings also default to Dijkstra.
    settings = deepcopy(DATA['joint_planner'])
    del settings['search_method']
    del settings['weighted_astar_weight']
    _, parameters, _, _, _ = setup_scenario('T06_walk_only')
    endpoints = {name: planner.graph.nodes[node_id] for name, node_id in planner.endpoints.items()}
    fallback = JointPlanner(parameters, settings, planner.map, planner.builder, endpoints)
    assert fallback.search_method == 'dijkstra'
    assert audit(fallback)['total_cost'] == pytest.approx(8)


@pytest.mark.parametrize('settings', [
    {'search_method': 'unknown'}, {'weighted_astar_weight': .5},
    {'weighted_astar_weight': float('nan')}, {'weighted_astar_weight': float('inf')},
])
def test_invalid_public_search_configuration(settings):
    with pytest.raises(ValueError):
        prepared('T06_walk_only', search_settings=settings)


@pytest.mark.parametrize('tolerance', [0, -1, float('nan'), float('inf')])
def test_invalid_joint_cost_tolerance(tolerance):
    with pytest.raises(ValueError, match='Cost tolerance'):
        prepared('T06_walk_only', search_settings={'cost_tolerance': tolerance})


def test_configured_joint_cost_tolerance():
    planner = prepared('T06_walk_only', search_settings={'cost_tolerance': 1e-6})
    result = planner.plan()
    source, target = result['path'][0], result['path'][-1]
    planner.validate_route(result['path'], source, target, result['total_cost'] + 1e-7)
    with pytest.raises(ValueError, match='total cost'):
        planner.validate_route(result['path'], source, target, result['total_cost'] + 1e-3)


@pytest.mark.parametrize('name', ['T05_flight_bridge', 'T06_walk_only'])
def test_executable_request_is_rejected_before_search(name, monkeypatch):
    from m4_global_planner.representation import joint_graph

    planner = prepared(name)

    def forbidden_search(*args, **kwargs):
        pytest.fail('Execution request must fail before a geometric search')

    monkeypatch.setattr(joint_graph, 'dijkstra', forbidden_search)
    with pytest.raises(NotImplementedError, match='takeoff, landing or controller'):
        planner.plan(require_execution=True)


def test_execution_configuration_cannot_enable_physical_support():
    with pytest.raises(NotImplementedError, match='execution_policy'):
        prepared('T05_flight_bridge', search_settings={'execution_policy': 'executable'})
