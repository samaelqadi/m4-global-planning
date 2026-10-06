"""Snapshot rebuild replanning on existing R01/R02 geometric worlds."""

from copy import deepcopy
from math import isfinite

from m4_global_planner.dummy_map import DummyPlanningMap
from m4_global_planner.representation.anchors import MandatoryAnchorBuilder
from m4_global_planner.representation.fly_validator import FlyStateValidator, TransitionValidator
from m4_global_planner.representation.joint_graph import JointPlanner
from m4_global_planner.representation.walk_validator import WalkStateValidator
from m4_global_planner.scenario_loader import get_scenario_parameters
import pytest
from test_anchors import endpoint
from test_graph_optimality import dijkstra_path
from test_straight_walk_edges import DATA


@pytest.fixture
def planner():
    def build(name, method='dijkstra'):

        scenario = deepcopy(DATA['scenarios'][name])
        parameters = get_scenario_parameters(DATA, name)
        settings = deepcopy(DATA['joint_planner'])
        settings['search_method'] = method
        parameters['walk_terrain_cost_mode'] = settings['walk_terrain_cost_mode']
        walk, fly = WalkStateValidator(parameters), FlyStateValidator(parameters)
        builder = MandatoryAnchorBuilder(parameters, walk, fly,
                                         TransitionValidator(parameters, walk, fly))
        endpoints = {key: endpoint({'mode': 'walk', **scenario[key]}, i)
                     for i, key in enumerate(('start', 'goal'))}
        return JointPlanner(parameters, settings, DummyPlanningMap(scenario), builder, endpoints)
    return build


def audit(planner, result):
    planner.validate_result(result)
    assert result['route_kind'] == 'geometric' and not result['executable']
    source, target = (planner.endpoints[key] for key in ('start', 'goal'))
    cost, path = dijkstra_path(planner.graph, source, target)
    assert result['success'] == isfinite(cost)
    if result['success']:
        assert result['total_cost'] == pytest.approx(cost)
        planner.validate_route(path, source, target, cost)


def test_r01_blocks_previous_route_and_rebuilds_all_data(planner):
    current = planner('R01_route_becomes_blocked')
    previous = current.plan()
    audit(current, previous)
    old_graph, old_map, old_anchors = current.graph, current.map, current.anchors
    result = current.replan(DATA['scenarios']['R01_route_becomes_blocked']['update'])
    audit(current, result)
    assert result['success'] and result['total_cost'] > previous['total_cost']
    assert current.graph is not old_graph and current.map is not old_map
    assert current.anchors is not old_anchors
    assert old_map.get_obstacles_3d() == []
    assert result['map_revision'] == 1 and previous['map_revision'] == 0
    with pytest.raises(ValueError, match='Stale'):
        current.validate_result(previous)
    # Initial route is WALK-only; at least one old sweep fails on the new map.
    assert all(old_graph.nodes[i].mode.value == 'walk' for i in previous['path'])
    assert any(current.motions.edge(current.pose(old_graph.nodes[source]),
                                    current.pose(old_graph.nodes[target])) is None
               for source, target in zip(previous['path'], previous['path'][1:]))
    assert result['replanning_ms'] >= result['rebuild_ms'] >= 0


def test_r02_removed_exclusion_creates_cheaper_route(planner):
    current = planner('R02_new_route_available')
    previous = current.plan()
    audit(current, previous)
    result = current.replan(DATA['scenarios']['R02_new_route_available']['update'])
    audit(current, result)
    assert result['success'] and result['total_cost'] < previous['total_cost']
    assert current.map.get_walk_exclusion_regions() == []
    assert result['total_cost'] == pytest.approx(8)


def test_cost_replacement_reprices_edges_and_reversal(planner):
    current = planner('R01_route_becomes_blocked')
    before = current.plan()
    graph = current.graph
    region = {'min': [0, 0], 'max': [10, 4], 'cost': 100}
    costly = current.replan({'set_walk_cost_regions': [region]})
    audit(current, costly)
    assert current.graph is not graph
    assert costly['total_cost'] > before['total_cost']
    # Cost-only updates retain poses; the old route's total must fail a fresh audit.
    with pytest.raises(ValueError, match='total cost'):
        current.validate_route(before['path'], current.endpoints['start'],
                               current.endpoints['goal'], before['total_cost'])
    assert any(current.graph.nodes[i].mode.value == 'fly' for i in costly['path'])
    with pytest.raises(ValueError, match='Stale'):
        current.validate_result(before)
    restored = current.replan({'set_walk_cost_regions': []})
    audit(current, restored)
    assert restored['total_cost'] == pytest.approx(before['total_cost'])
    assert restored['map_revision'] == 2


def test_complete_barrier_publishes_no_path_and_stale_route_is_rejected(planner):
    current = planner('R01_route_becomes_blocked')
    before = current.plan()
    result = current.replan({'add_obstacles': [
        {'min': [4.5, 0, 0], 'max': [5.5, 4, 3]},
    ]})
    audit(current, result)
    assert not result['success'] and result['path'] == []
    assert current.plan()['success'] is False
    with pytest.raises(ValueError, match='Stale'):
        current.validate_result(before)


def test_accepted_update_with_invalid_endpoint_fails_closed(planner):
    current = planner('R01_route_becomes_blocked')
    before = current.plan()
    with pytest.raises(ValueError, match='Invalid endpoint'):
        current.replan({'add_obstacles': [
            {'min': [.5, 1.5, 0], 'max': [1.5, 2.5, 3]},
        ]})
    assert current.map.revision == 1 and not current.graph.nodes
    with pytest.raises(RuntimeError, match='No valid graph'):
        current.plan()
    with pytest.raises(ValueError, match='Stale'):
        current.validate_result(before)


@pytest.mark.parametrize('update', [
    {'unknown_operation': []}, {'add_obstacles': 'bad'},
    {'add_obstacles': [{'min': [1, 2], 'max': [3, 4]}]},
    {'add_obstacles': [{'min': None, 'max': [3, 4, 5]}]},
    {'set_walk_cost_regions': [{'min': [0, 0], 'max': [1, 1], 'cost': -1}]},
    {'remove_walk_forbidden': [{'min': [0, 0], 'max': [1, 1]}]},
])
def test_invalid_update_preserves_previous_snapshot(planner, update):
    current = planner('R01_route_becomes_blocked')
    graph, planning_map = current.graph, current.map
    with pytest.raises((ValueError, NotImplementedError)):
        current.replan(update)
    assert current.graph is graph and current.map is planning_map
    audit(current, current.plan())


def test_execution_request_cannot_apply_update(planner):
    current = planner('R01_route_becomes_blocked')
    with pytest.raises(NotImplementedError, match='Executable'):
        current.replan(DATA['scenarios']['R01_route_becomes_blocked']['update'],
                       require_execution=True)
    assert current.map.revision == 0


def test_update_snapshots_do_not_alias_input_data():
    scenario = deepcopy(DATA['scenarios']['R01_route_becomes_blocked'])
    planning_map = DummyPlanningMap(scenario)
    update = deepcopy(scenario['update'])
    updated = planning_map.apply_update(update)
    update['add_obstacles'][0]['min'][0] = 0
    scenario['initial_world']['bounds'][1] = 999
    assert planning_map.get_bounds()[1] == updated.get_bounds()[1] == 10
    assert updated.get_obstacles_3d()[0]['min'][0] == 4.5


def test_replanning_uses_dijkstra_even_after_astar(planner, monkeypatch):
    from m4_global_planner.representation import joint_graph

    current = planner('R01_route_becomes_blocked', method='astar')
    audit(current, current.plan())
    calls = []
    search = joint_graph.dijkstra

    def record(*args, **kwargs):
        calls.append(True)
        return search(*args, **kwargs)

    monkeypatch.setattr(joint_graph, 'dijkstra', record)
    result = current.replan(DATA['scenarios']['R01_route_becomes_blocked']['update'])
    audit(current, result)
    assert calls == [True] and current.search_method == 'dijkstra'


def test_incremental_policy_is_unsupported(planner):
    current = planner('R01_route_becomes_blocked')
    settings = {**current.settings, 'replanning_policy': 'incremental'}
    with pytest.raises(NotImplementedError, match='replanning_policy'):
        JointPlanner(current.parameters, settings, current.map, current.builder,
                     current.request_endpoints)
