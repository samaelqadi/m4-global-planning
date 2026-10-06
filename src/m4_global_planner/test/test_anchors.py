from copy import deepcopy

from m4_global_planner.graph import Graph, MotionMode, Node
from m4_global_planner.representation.anchors import MandatoryAnchorBuilder
from m4_global_planner.representation.states import PoseState
import pytest
from test_fly_validation import fly_setup


def anchor_setup(name):
    scenario, parameters, planning_map, fly, transition = fly_setup(name)
    builder = MandatoryAnchorBuilder(parameters, transition.walk, fly, transition)
    return scenario, parameters, planning_map, builder


def endpoint(record, node_id):
    mode = MotionMode(record['mode'])
    heading = record.get('heading_deg')
    if mode == MotionMode.WALK:
        from math import radians
        heading = radians(heading)
    return Node(node_id, *record['position'], mode, heading)


def test_t03_anchors_only_in_full_envelope_clear_center():
    _, _, planning_map, builder = anchor_setup('T03_only_one_transition_zone')
    anchors = builder.build(planning_map)
    assert anchors.transitions and anchors.rejected_transition_states > 0
    assert {node.x for node in anchors.fly_nodes} == {5.0}
    assert all(4.5 < node.x < 5.5 for node in anchors.fly_nodes)
    for anchor in anchors.transitions:
        node = anchors.fly_nodes[anchor.fly_id]
        assert tuple(anchor.directions) == ('walk_to_fly', 'fly_to_walk')
        assert all(builder.transition.is_valid(anchor.walk_pose, node, planning_map, direction)
                   for direction in anchor.directions)


def test_t04_soft_costs_do_not_filter_or_price_anchors():
    scenario, _, planning_map, builder = anchor_setup('T04_multiple_transition_choices')
    anchors = builder.build(planning_map)
    assert len(anchors.fly_nodes) > 1
    positions = {(node.x, node.y) for node in anchors.fly_nodes}
    assert (2.0, 2.0) in positions and (8.0, 4.0) in positions
    scenario['world']['walk_cost_regions'] = []
    assert builder.build(planning_map) == anchors
    assert not hasattr(anchors.transitions[0], 'cost')


def test_t05_shared_exact_anchors_on_both_sides_not_in_forbidden_ground():
    scenario, _, planning_map, builder = anchor_setup('T05_flight_bridge')
    endpoints = {name: endpoint(scenario[name], index) for index, name in enumerate(('start',
                                                                                     'goal'))}
    anchors = builder.build(planning_map, endpoints)
    assert any(node.x < 4 for node in anchors.fly_nodes)
    assert any(node.x > 6 for node in anchors.fly_nodes)
    assert not any(4 <= node.x <= 6 for node in anchors.fly_nodes)
    assert all(any(anchor.walk_pose == PoseState(node.x, node.y, node.z, node.yaw)
                   for anchor in anchors.transitions) for node in endpoints.values())
    # Check only supported segment geometry, not executable takeoff or a route.
    left = next(node for node in anchors.fly_nodes if (node.x, node.y) == (3.5, 2.0))
    right = next(node for node in anchors.fly_nodes if (node.x, node.y) == (6.5, 2.0))
    assert builder.fly.is_edge_valid(left, right, planning_map)
    with pytest.raises(NotImplementedError, match='takeoff'):
        builder.fly.is_edge_valid(left, right, planning_map, 'takeoff')
    graphs = Graph(), Graph()
    graphs[1].add_node(Node(50, left.x, left.y, left.z, MotionMode.FLY))
    for graph in graphs:
        mapping = builder.insert(anchors, graph, planning_map)
        assert len(mapping) == len(anchors.fly_nodes)
        assert {(node.x, node.y, node.z, node.mode,
                 node.yaw) for node in graph.nodes.values()} == {
            (node.x, node.y, node.z, node.mode, node.yaw) for node in anchors.fly_nodes}
        assert builder.insert(anchors, graph, planning_map) == mapping
        assert not any(graph.edges.values())


def test_off_grid_endpoints_preserve_coordinates_and_deduplicate():
    _, _, planning_map, builder = anchor_setup('T04_multiple_transition_choices')
    node = Node(71, 2.13, 2.17, 1.07, MotionMode.FLY)
    walk = Node(72, 2.13, 2.17, 0, MotionMode.WALK, 0.123)
    endpoints = {'start': node, 'same': Node(99, node.x, node.y, node.z, MotionMode.FLY),
                 'walk': walk}
    anchors = builder.build(planning_map, endpoints)
    actual = dict(anchors.endpoints)
    assert actual['start'] == actual['same']
    assert (actual['start'].x, actual['start'].y, actual['start'].z) == (2.13, 2.17, 1.07)
    assert any(anchor.walk_pose == PoseState(2.13, 2.17, 0,
                                             0.123) for anchor in anchors.transitions)
    assert anchors == builder.build(planning_map, dict(reversed(list(endpoints.items()))))


def test_invalid_endpoint_and_stale_morphing_fail_before_insertion():
    scenario, _, planning_map, builder = anchor_setup('T04_multiple_transition_choices')
    with pytest.raises(ValueError, match='Invalid endpoint'):
        builder.build(planning_map, {'start': Node(0, 0, 0, 0, MotionMode.FLY)})
    with pytest.raises(ValueError, match='heading'):
        builder.build(planning_map, {'start': Node(0, 2, 2, 0, MotionMode.WALK)})
    anchors = builder.build(planning_map)
    scenario['world']['obstacles_3d'] = [{'min': [1.5, 1.5, 0.75], 'max': [2.5, 2.5, 1.0]}]
    graph = Graph()
    with pytest.raises(ValueError, match='Transition anchor'):
        builder.insert(anchors, graph, planning_map)
    assert not graph.nodes


def test_blocked_morphing_does_not_invalidate_valid_walk_endpoint():
    scenario, _, planning_map, builder = anchor_setup('T02_blocked_morphing_space')
    node = Node(0, *scenario['position'], MotionMode.WALK, 0)
    anchors = builder.build(planning_map, {'start': node})
    assert dict(anchors.endpoints)['start'] == node
    assert not any(anchor.walk_pose == PoseState(*scenario['position'],
                                                 0) for anchor in anchors.transitions)


@pytest.mark.parametrize('change', ['missing', 'budget', 'sampling', 'resolution',
                                    'endpoint_policy'])
def test_missing_or_unsupported_anchor_configuration(change):
    _, parameters, planning_map, builder = anchor_setup('T04_multiple_transition_choices')
    parameters = deepcopy(parameters)
    if change == 'missing':
        del parameters['mandatory_anchors']['transition_spatial_resolution']
    elif change == 'budget':
        parameters['mandatory_anchors']['max_sampled_walk_states'] = 1
    elif change == 'sampling':
        parameters['mandatory_anchors']['transition_sampling_policy'] = 'random'
    elif change == 'endpoint_policy':
        parameters['mandatory_anchors']['endpoint_policy'] = 'snap'
    else:
        parameters['mandatory_anchors']['transition_spatial_resolution'] = float('nan')
    with pytest.raises((ValueError, NotImplementedError)):
        other = MandatoryAnchorBuilder(parameters, builder.walk, builder.fly, builder.transition)
        other.build(planning_map)
