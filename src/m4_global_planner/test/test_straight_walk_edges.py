from copy import deepcopy
from math import isclose, pi, radians
from pathlib import Path

from m4_global_planner.dummy_map import DummyPlanningMap
from m4_global_planner.graph import MotionMode
from m4_global_planner.representation.state_lattice import StateLatticeGenerator
from m4_global_planner.representation.states import PoseState
from m4_global_planner.representation.walk_edges import (
    StraightWalkEdgeGenerator, StraightWalkEdgeValidator,
)
from m4_global_planner.representation.walk_graph import (
    ForwardWalkGraphGenerator, ForwardWalkNeighbors,
)
from m4_global_planner.representation.walk_validator import WalkStateValidator
from m4_global_planner.scenario_loader import (
    get_scenario_parameters, load_scenario_data,
)
import pytest


DATA = load_scenario_data(
    Path(__file__).parents[1] / 'test_data' / 'scenarios.yaml'
)


def setup_scenario(name):
    scenario = deepcopy(DATA['scenarios'][name])
    parameters = get_scenario_parameters(DATA, name)
    planning_map = DummyPlanningMap(scenario)
    validator = WalkStateValidator(parameters)
    edge_validator = StraightWalkEdgeValidator(parameters, validator)
    generator = StraightWalkEdgeGenerator(parameters, edge_validator)
    return scenario, parameters, planning_map, validator, generator


def pose(record):
    return PoseState(*record['position'], radians(record['heading_deg']))


@pytest.mark.parametrize('name,exists', [
    ('W01_forward', True),
    ('W02_sideways_motion', False),
    ('W06_blocked_between_free_endpoints', False),
])
def test_documented_edges(name, exists):
    scenario, _, planning_map, validator, generator = setup_scenario(name)
    start, end = pose(scenario['from']), pose(scenario['to'])
    assert validator.is_valid(start, planning_map)
    assert validator.is_valid(end, planning_map)
    edge = generator.generate_edge(start, end, planning_map)
    assert (edge is not None) == exists
    if edge:
        assert edge.source == start and edge.target == end
        assert edge.distance == 1.0 and edge.cost == 1.0


@pytest.mark.parametrize('mutation', [
    'reverse', 'pivot', 'heading', 'distance', 'missing', 'forbidden',
    'thin_obstacle', 'footprint_obstacle', 'contact_obstacle',
])
def test_invalid_connections(mutation):
    scenario, parameters, planning_map, _, generator = setup_scenario('W01_forward')
    start, end = pose(scenario['from']), pose(scenario['to'])
    if mutation == 'reverse':
        start, end = end, start
    elif mutation == 'pivot':
        end = PoseState(start.x, start.y, start.z, pi / 2)
    elif mutation == 'heading':
        end = PoseState(end.x, end.y, end.z, pi / 4)
    elif mutation == 'distance':
        generator.validator.max_distance = parameters['spatial_resolution']
    elif mutation in ('missing', 'forbidden'):
        region = {'min': [1.45, 0.9], 'max': [1.55, 1.1]}
        if mutation == 'missing':
            scenario['world']['ground']['missing_regions'] = [region]
        else:
            scenario['world']['walk_forbidden'] = [region]
    else:
        ymin = {'thin_obstacle': 0.9, 'footprint_obstacle': 1.2,
                'contact_obstacle': 1.3}[mutation]
        scenario['world']['obstacles_3d'] = [
            {'min': [1.49, ymin, 0.0], 'max': [1.51, 1.4, 0.5]},
        ]
    assert generator.generate_edge(start, end, planning_map) is None


def test_configuration_and_unimplemented_terrain_costs():
    scenario, parameters, planning_map, validator, _ = setup_scenario('W01_forward')
    start, end = pose(scenario['from']), pose(scenario['to'])
    parameters['straight_walk']['distance_cost_per_meter'] = 2.5
    generator = StraightWalkEdgeGenerator(
        parameters, StraightWalkEdgeValidator(parameters, validator),
    )
    assert generator.generate_edge(start, end, planning_map).cost == 2.5
    end = PoseState(end.x, end.y, end.z, 2 * pi)
    assert generator.generate_edge(start, end, planning_map) is not None
    scenario['world']['walk_cost_regions'] = [
        {'min': [1.45, 0.9], 'max': [1.55, 1.1], 'cost': 5.0},
    ]
    with pytest.raises(NotImplementedError, match='terrain'):
        generator.generate_edge(start, end, planning_map)
    scenario['world']['ground'] = {
        'type': 'plane', 'z_at_origin': 0.0, 'slope_x': 0.0, 'slope_y': 0.0,
    }
    assert generator.generate_edge(start, end, planning_map) is None


def test_overhead_clearance_and_endpoint_rejection():
    scenario, _, planning_map, _, generator = setup_scenario('W01_forward')
    start, end = pose(scenario['from']), pose(scenario['to'])
    scenario['world']['obstacles_3d'] = [
        {'min': [1.49, 0.9, 0.6], 'max': [1.51, 1.1, 1.0]},
    ]
    assert generator.generate_edge(start, end, planning_map) is not None
    invalid = PoseState(end.x, end.y, 0.2, end.heading)
    assert generator.generate_edge(start, invalid, planning_map) is None


@pytest.mark.parametrize('key,value', [
    ('max_distance', float('nan')),
    ('lateral_tolerance', 0.0),
    ('heading_tolerance_rad', pi),
    ('distance_cost_per_meter', -1.0),
])
def test_invalid_configuration(key, value):
    _, parameters, _, validator, _ = setup_scenario('W01_forward')
    parameters['straight_walk'][key] = value
    with pytest.raises(ValueError):
        StraightWalkEdgeGenerator(
            parameters, StraightWalkEdgeValidator(parameters, validator),
        )


@pytest.mark.parametrize('name', [name for name in DATA['scenarios'] if name.startswith('N0')])
def test_node_regressions(name):
    scenario, parameters, planning_map, validator, _ = setup_scenario(name)
    generator = StateLatticeGenerator(
        parameters['spatial_resolution'], parameters['heading_resolution_deg'],
    )
    states = generator.generate_states(planning_map, validator)
    assert states and len(states) == len(set(states))
    counts = {'N01': 616, 'N02': 444, 'N03': 616,
              'N04': 544, 'N05': 416, 'N06': 616}
    assert len(states) == counts[name[:3]]
    headings = generator.generate_headings()
    for state in states:
        assert validator.is_valid(state, planning_map)
        assert state.heading in headings
        assert isclose(state.z, planning_map.get_ground_height(state.x, state.y))
        assert not planning_map.is_walk_forbidden(state.x, state.y)
    # Independently enumerate configured lattice positions and apply the
    # existing node policy. Footprint clipping supersedes old raw counts.
    xmin, xmax, ymin, ymax = planning_map.get_bounds()
    resolution = parameters['spatial_resolution']
    expected = set()
    for ix in range(int((xmax - xmin) / resolution) + 1):
        for iy in range(int((ymax - ymin) / resolution) + 1):
            x, y = xmin + ix * resolution, ymin + iy * resolution
            z = planning_map.get_ground_height(x, y)
            if z is not None:
                for heading in headings:
                    state = PoseState(x, y, z, heading)
                    if validator.is_valid(state, planning_map):
                        expected.add(state)
    assert set(states) == expected
    if name == 'N02_non_integer_bounds':
        assert max(state.x for state in states) <= 5.0
        assert max(state.y for state in states) <= 3.5
    if name == 'N03_variable_ground_height':
        assert len({state.z for state in states}) > 1
    if name == 'N06_high_cost_ground':
        assert any(2 <= state.x <= 4 and 1 <= state.y <= 3 for state in states)
        scenario['world']['walk_cost_regions'] = []
        assert states == generator.generate_states(planning_map, validator)


def graph_generator(parameters, validator, edges):
    return ForwardWalkGraphGenerator(
        ForwardWalkNeighbors(parameters),
        StateLatticeGenerator(
            parameters['spatial_resolution'], parameters['heading_resolution_deg'],
        ), validator, edges,
    )


def node_id(graph, state):
    return next(
        node.node_id for node in graph.nodes.values()
        if (node.x, node.y, node.z, node.yaw)
        == (state.x, state.y, state.z, state.heading)
    )


@pytest.mark.parametrize('name,exists', [
    ('W01_forward', True),
    ('W02_sideways_motion', False),
    ('W06_blocked_between_free_endpoints', False),
])
def test_generated_graph(name, exists):
    scenario, parameters, planning_map, validator, edges = setup_scenario(name)
    builder = graph_generator(parameters, validator, edges)
    attempted = set()
    original = edges.generate_edge

    def record(start, end, planning_map):
        attempted.add((start, end))
        return original(start, end, planning_map)

    edges.generate_edge = record
    graph = builder.generate_graph(planning_map)
    start, end = pose(scenario['from']), pose(scenario['to'])
    source, target = node_id(graph, start), node_id(graph, end)
    assert (target in graph.edges[source]) == exists
    assert source not in graph.edges[target]  # no reverse at this heading
    assert ((start, end) in attempted) == (name != 'W02_sideways_motion')
    if exists:
        assert graph.edges[source][target].cost == 1.0
    assert builder.candidate_attempts == len(attempted)
    # Each heading has at most one forward neighbor per configured shell.
    assert 0 < builder.candidate_attempts <= len(graph.nodes) * len(
        parameters['straight_walk']['neighbor_grid_steps']
    )
    assert builder.candidate_attempts < len(graph.nodes) * (len(graph.nodes) - 1)
    assert all(node.mode == MotionMode.WALK for node in graph.nodes.values())
    for outgoing in graph.edges.values():
        for edge in outgoing.values():
            a, b = graph.get_node(edge.source_id), graph.get_node(edge.target_id)
            expected = original(
                PoseState(a.x, a.y, a.z, a.yaw),
                PoseState(b.x, b.y, b.z, b.yaw), planning_map,
            )
            assert expected is not None and edge.cost == expected.cost
    again = builder.generate_graph(planning_map)
    assert graph.nodes == again.nodes and graph.edges == again.edges


def test_configured_neighbors_and_graph_costs():
    scenario, parameters, planning_map, validator, _ = setup_scenario('W01_forward')
    parameters['straight_walk']['neighbor_grid_steps'] = [1]
    parameters['straight_walk']['distance_cost_per_meter'] = 2.5
    edges = StraightWalkEdgeGenerator(
        parameters, StraightWalkEdgeValidator(parameters, validator),
    )
    builder = graph_generator(parameters, validator, edges)
    graph = builder.generate_graph(planning_map)
    start, end = pose(scenario['from']), pose(scenario['to'])
    source, target = node_id(graph, start), node_id(graph, end)
    assert target not in graph.edges[source]
    midpoint = PoseState(1.5, 1.0, 0.0, 0.0)
    middle = node_id(graph, midpoint)
    assert graph.edges[source][middle].cost == 1.25
    assert graph.edges[middle][target].cost == 1.25
    # Diagonal lattice neighbors have sqrt(2) times the axial spacing.
    assert set(builder.neighbors.offsets(pi / 4)) == {(1, 1)}


@pytest.mark.parametrize('steps', [[], [0], [-1], [1, 1], [1.5], [True]])
def test_invalid_neighbor_steps(steps):
    _, parameters, _, _, _ = setup_scenario('W01_forward')
    parameters['straight_walk']['neighbor_grid_steps'] = steps
    with pytest.raises(ValueError, match='grid steps'):
        ForwardWalkNeighbors(parameters)


def test_lattice_index_validation():
    _, parameters, _, _, _ = setup_scenario('W01_forward')
    neighbors = ForwardWalkNeighbors(parameters)
    with pytest.raises(ValueError, match='configured lattice'):
        list(neighbors.candidate_pairs([PoseState(1.1, 1, 0, 0)], (0, 0)))
    state = PoseState(1, 1, 0, 0)
    with pytest.raises(ValueError, match='Duplicate'):
        list(neighbors.candidate_pairs([state, state], (0, 0)))
