from copy import deepcopy

from m4_global_planner.representation.states import PoseState
from m4_global_planner.representation.walk_edges import (
    WalkMotionEdgeGenerator, WalkMotionValidator,
)
from m4_global_planner.walk_comparison import APPROACHES, Motions, plan, pose
import pytest
from test_straight_walk_edges import DATA, setup_scenario


@pytest.mark.parametrize('approach', APPROACHES)
@pytest.mark.parametrize('reverse', [True, False])
def test_comparison_reverse_and_exact_endpoints(approach, reverse):
    scenario, parameters, planning_map, validator, _ = setup_scenario('W04_dead_end_reverse')
    settings = deepcopy(DATA['walk_benchmark'])
    parameters['reverse_allowed'] = reverse
    parameters['walk_terrain_cost_mode'] = settings['terrain_cost_mode']
    start, goal = pose(scenario['start']), pose(scenario['goal'])
    result = plan(approach, parameters, settings, planning_map, start, goal, seed=11)
    assert result['success'] == reverse
    if reverse:
        assert result['route'][0] == start and result['route'][-1] == goal
        edges = WalkMotionEdgeGenerator(parameters, WalkMotionValidator(parameters, validator))
        cost = sum(edges.generate_edge(a, b, planning_map).cost
                   for a, b in zip(result['route'], result['route'][1:]))
        assert cost == pytest.approx(result['cost'])
    invalid = PoseState(5, 1, 0, 0)
    assert plan(approach, parameters, settings, planning_map, invalid,
                goal)['status'] == 'invalid_start'


def test_density_integrates_covered_length_and_retains_legacy_mode():
    scenario, parameters, planning_map, validator, _ = setup_scenario('W01_forward')
    scenario['world']['walk_cost_regions'] = [
        {'min': [1.5, 0.5], 'max': [2.0, 1.5], 'cost': 5.0},
    ]
    a, b = pose(scenario['from']), pose(scenario['to'])
    edges = WalkMotionEdgeGenerator(parameters, WalkMotionValidator(parameters, validator))
    with pytest.raises(NotImplementedError):
        edges.generate_edge(a, b, planning_map)
    parameters['walk_terrain_cost_mode'] = 'distance_density'
    edges = WalkMotionEdgeGenerator(parameters, WalkMotionValidator(parameters, validator))
    assert edges.generate_edge(a, b, planning_map).cost == 3.5


def test_route_audit_rejects_unvalidated_shortcut(monkeypatch):
    scenario, parameters, planning_map, _, _ = setup_scenario('W06_blocked_between_free_endpoints')
    monkeypatch.setattr(Motions, 'connect', lambda self, a, b: ([a, b], 0.0))
    result = plan('hybrid', parameters, DATA['walk_benchmark'], planning_map,
                  pose(scenario['from']), pose(scenario['to']))
    assert result['status'] == 'invalid_route' and result['rejected_route']
    assert not result['success'] and result['cost'] is None


def test_hybrid_roundoff_does_not_change_requested_heading():
    scenario, parameters, planning_map, _, _ = setup_scenario('G02_equal_cost_routes')
    result = plan('hybrid', parameters, DATA['walk_benchmark'], planning_map,
                  pose(scenario['start']), pose(scenario['goal']))
    assert result['status'] == 'success'
    assert result['route'][-1] == pose(scenario['goal'])
