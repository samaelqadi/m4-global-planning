from copy import deepcopy
from math import dist

from m4_global_planner.graph import MotionMode, Node
from m4_global_planner.representation.fly_validator import FlyStateValidator, TransitionValidator
from m4_global_planner.representation.states import PoseState
import pytest
from test_straight_walk_edges import DATA, setup_scenario


def fly(x, y, z, yaw=None):
    return Node(0, x, y, z, MotionMode.FLY, yaw)


def fly_setup(name):
    scenario, parameters, planning_map, walk, _ = setup_scenario(name)
    validator = FlyStateValidator(parameters)
    return scenario, parameters, planning_map, validator, TransitionValidator(parameters, walk,
                                                                              validator)


def test_f02_free_endpoints_but_blocked_full_sweep():
    scenario, _, planning_map, validator, _ = fly_setup('F02_blocked_flight_segment')
    a, b = fly(*scenario['from']['position']), fly(*scenario['to']['position'])
    assert validator.is_valid(a, planning_map) and validator.is_valid(b, planning_map)
    assert not validator.is_edge_valid(a, b, planning_map)
    scenario['world']['obstacles_3d'] = []
    assert validator.is_edge_valid(a, b, planning_map)
    # A thin box catches the body's side although its anchor line is clear.
    scenario['world']['obstacles_3d'] = [
        {'min': [3.0, 2.3, 2.1], 'max': [3.001, 2.31, 2.2]},
    ]
    assert not validator.is_edge_valid(a, b, planning_map)


def test_f03_full_body_ceiling_and_bounds():
    _, _, planning_map, validator, _ = fly_setup('F03_low_ceiling')
    assert validator.is_valid(fly(1, 1, 0.8), planning_map)
    assert not validator.is_valid(fly(1, 1, 0.9), planning_map)
    assert not validator.is_valid(fly(0.2, 1, 0.5), planning_map)
    assert not validator.is_valid(fly(1, 0.2, 0.5), planning_map)
    assert not validator.is_valid(fly(1, 1, -0.1), planning_map)
    assert validator.is_valid(fly(1, 1, 0), planning_map)  # Geometric dock only.


def test_f04_overhead_and_vertical_sweep():
    _, _, planning_map, validator, _ = fly_setup('F04_overhead_obstacle')
    under, above = fly(3, 2, 0.2), fly(3, 2, 1.3)
    assert validator.is_valid(under, planning_map) and validator.is_valid(above, planning_map)
    assert not validator.is_valid(fly(3, 2, 0.4), planning_map)
    assert not validator.is_edge_valid(under, above, planning_map)
    assert validator.is_edge_valid(fly(1, 2, 0.2), fly(1, 2, 1.3), planning_map)


@pytest.mark.parametrize('name,valid', [('T01_open_transition', True),
                                        ('T02_blocked_morphing_space', False)])
@pytest.mark.parametrize('direction', ['walk_to_fly', 'fly_to_walk'])
def test_stationary_morphing_envelope(name, valid, direction):
    scenario, _, planning_map, validator, transitions = fly_setup(name)
    walk = PoseState(*scenario['position'], 0)
    node = fly(*scenario['position'])
    assert transitions.walk.is_valid(walk, planning_map)
    assert validator.is_valid(node, planning_map)
    assert transitions.is_valid(walk, node, planning_map, direction) == valid
    assert dist((walk.x, walk.y, walk.z), (node.x, node.y, node.z)) == 0
    shifted = fly(node.x + 1.0e-12, node.y, node.z)
    assert not transitions.is_valid(walk, shifted, planning_map, direction)


def test_diagonal_sweep_and_body_top_bound():
    scenario, parameters, planning_map, _, _ = fly_setup('F02_blocked_flight_segment')
    parameters['fly_validation']['max_edge_distance'] = 8
    validator = FlyStateValidator(parameters)
    a, b = fly(1, 0.5, 0.2), fly(7, 3.5, 3.5)
    assert validator.is_valid(a, planning_map) and validator.is_valid(b, planning_map)
    assert not validator.is_edge_valid(a, b, planning_map)
    scenario['world']['obstacles_3d'] = []
    assert validator.is_edge_valid(a, b, planning_map)
    assert not validator.is_valid(fly(1, 1, 3.8), planning_map)


def test_configured_margin_limits_and_collision_tolerance():
    scenario, parameters, planning_map, validator, _ = fly_setup('F04_overhead_obstacle')
    assert validator.is_valid(fly(1, 2, 0.2), planning_map)
    parameters['fly_validation']['altitude_limits'] = [0.5, 1.0]
    limited = FlyStateValidator(parameters)
    assert not limited.is_valid(fly(1, 2, 0.2), planning_map)
    parameters['fly_validation']['altitude_limits'] = [0, 8]
    parameters['fly_validation']['safety_margin'] = 0.1
    padded = FlyStateValidator(parameters)
    assert not padded.is_valid(fly(0.45, 2, 0.2), planning_map)
    assert not padded.is_valid(fly(1, 2, 0), planning_map)
    # At y=0.6 the body touches the obstacle's y=1 face.
    assert not validator.is_valid(fly(3, 0.6, 0.8), planning_map)
    assert validator.is_valid(fly(3, 0.59, 0.8), planning_map)
    parameters['fly_validation']['safety_margin'] = 0
    parameters['fly_validation']['collision_tolerance'] = 0.02
    assert not FlyStateValidator(parameters).is_valid(fly(3, 0.59, 0.8), planning_map)


def test_transition_body_bounds_and_missing_ground():
    scenario, _, planning_map, _, transitions = fly_setup('T01_open_transition')
    walk = PoseState(0.45, 2.5, 0, 0)
    node = fly(0.45, 2.5, 0)
    assert transitions.walk.is_valid(walk, planning_map)
    assert transitions.fly.is_valid(node, planning_map)
    assert not transitions.is_valid(walk, node, planning_map)
    scenario['world']['ground']['missing_regions'] = [
        {'min': [2, 2], 'max': [3, 3]},
    ]
    assert transitions.fly.is_valid(fly(2.5, 2.5, 1), planning_map)
    assert not transitions.is_valid(PoseState(2.5, 2.5, 0, 0), fly(2.5, 2.5, 0), planning_map)


def test_edge_limits_and_morphing_top_bounds():
    scenario, _, planning_map, validator, transitions = fly_setup('T01_open_transition')
    node = fly(2.5, 2.5, 0)
    assert not validator.is_edge_valid(node, node, planning_map)
    # Both bodies fit under this bound, but the morphing envelope does not.
    scenario['world']['bounds'][-1] = 0.8
    assert validator.is_valid(node, planning_map)
    walk = PoseState(2.5, 2.5, 0, 0)
    assert transitions.walk.is_valid(walk, planning_map)
    assert not transitions.is_valid(walk, node, planning_map)
    _, parameters, other_map, _, _ = fly_setup('F02_blocked_flight_segment')
    parameters['fly_validation']['max_edge_distance'] = 1
    limited = FlyStateValidator(parameters)
    assert not limited.is_edge_valid(fly(1, 0.5, 1), fly(3, 0.5, 1), other_map)


def test_morphing_envelope_must_cover_endpoint_bodies():
    _, parameters, _, _, transitions = fly_setup('T01_open_transition')
    parameters['morphing_envelope']['width'] = 0.5
    with pytest.raises(ValueError, match='FLY body'):
        TransitionValidator(parameters, transitions.walk, transitions.fly)
    parameters['morphing_envelope']['width'] = 1.0
    parameters['walk_footprint']['length'] = 2.0
    from m4_global_planner.representation.walk_validator import WalkStateValidator
    transitions = TransitionValidator(parameters, WalkStateValidator(parameters), transitions.fly)
    _, _, planning_map, _, _ = fly_setup('T01_open_transition')
    with pytest.raises(NotImplementedError, match='WALK body'):
        transitions.is_valid(PoseState(2.5, 2.5, 0, 0), fly(2.5, 2.5, 0), planning_map)


@pytest.mark.parametrize('motion', ['takeoff', 'landing', 'curved', 'yaw_rotation',
                                    'dynamic_flight'])
def test_unsupported_motion_is_explicit(motion):
    _, _, planning_map, validator, _ = fly_setup('F04_overhead_obstacle')
    with pytest.raises(NotImplementedError, match='Unsupported flight motion'):
        validator.is_edge_valid(fly(1, 2, 0.2), fly(1, 2, 1.3), planning_map, motion)


@pytest.mark.parametrize('field,value', [
    ('pose_anchor', 'body_center'), ('yaw_policy', 'free'),
    ('ground_policy', 'plane'), ('unknown_policy', 'free'),
    ('obstacle_contact_policy', 'allow'), ('motion_policy', 'curved'),
])
def test_unsupported_policy_is_explicit(field, value):
    parameters = deepcopy(DATA['test_parameters'])
    parameters['fly_validation'][field] = value
    with pytest.raises(NotImplementedError, match='Unsupported'):
        FlyStateValidator(parameters)


def test_unsupported_yaw_map_and_transition_direction():
    scenario, _, planning_map, validator, transitions = fly_setup('T01_open_transition')
    with pytest.raises(NotImplementedError, match='yaw'):
        validator.is_valid(fly(2.5, 2.5, 1, yaw=0), planning_map)
    with pytest.raises(NotImplementedError, match='direction'):
        transitions.is_valid(PoseState(2.5, 2.5, 0, 0), fly(2.5, 2.5, 0), planning_map, 'takeoff')
    scenario['world']['unknown_regions'] = [{'min': [0, 0], 'max': [1, 1]}]
    with pytest.raises(NotImplementedError, match='Unknown'):
        validator.is_valid(fly(2.5, 2.5, 1), planning_map)
    scenario['world']['unknown_regions'] = []
    scenario['world']['ground'] = {'type': 'plane', 'z_at_origin': 0, 'slope_x': 0, 'slope_y': 0}
    with pytest.raises(NotImplementedError, match='flat'):
        validator.is_valid(fly(2.5, 2.5, 1), planning_map)


@pytest.mark.parametrize('field,value', [
    ('safety_margin', -1), ('collision_tolerance', float('nan')),
    ('altitude_limits', [2, 1]), ('max_edge_distance', 0),
])
def test_invalid_numeric_settings(field, value):
    parameters = deepcopy(DATA['test_parameters'])
    parameters['fly_validation'][field] = value
    with pytest.raises(ValueError):
        FlyStateValidator(parameters)


@pytest.mark.parametrize('operation', ['takeoff', 'landing'])
def test_ground_to_air_geometry_does_not_validate_flight_operations(operation):
    _, _, planning_map, validator, transitions = fly_setup('T01_open_transition')
    ground, airborne = fly(2.5, 2.5, 0), fly(2.5, 2.5, 1)
    start, end = (ground, airborne) if operation == 'takeoff' else (airborne, ground)
    assert validator.is_edge_valid(start, end, planning_map)
    with pytest.raises(NotImplementedError, match='Unsupported flight motion'):
        validator.is_edge_valid(start, end, planning_map, operation)
    # Stationary morphing is supported only at the matching ground pose.
    walk = PoseState(2.5, 2.5, 0, 0)
    assert transitions.is_valid(walk, ground, planning_map, 'walk_to_fly')
    assert transitions.is_valid(walk, ground, planning_map, 'fly_to_walk')
    assert not transitions.is_valid(walk, airborne, planning_map, 'walk_to_fly')
    assert not transitions.is_valid(walk, airborne, planning_map, 'fly_to_walk')
    with pytest.raises(NotImplementedError, match='direction'):
        transitions.is_valid(walk, ground, planning_map, operation)


@pytest.mark.parametrize('section', ['fly_validation', 'transition_validation'])
def test_takeoff_landing_configuration_cannot_enable_missing_model(section):
    _, parameters, _, validator, transitions = fly_setup('T01_open_transition')
    parameters[section]['takeoff_landing_policy'] = 'supported'
    with pytest.raises(NotImplementedError, match='takeoff_landing_policy'):
        if section == 'fly_validation':
            FlyStateValidator(parameters)
        else:
            TransitionValidator(parameters, transitions.walk, validator)
