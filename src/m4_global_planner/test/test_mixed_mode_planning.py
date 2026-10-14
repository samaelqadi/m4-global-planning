"""Large ready-graph tests for complete mixed WALK/FLY route search."""

from math import fsum, isclose

import pytest

from m4_global_planner.astar import dijkstra
from m4_global_planner.global_planner import GlobalPlanner
from benchmarks.mixed_mode_fixtures import (
    GOAL_ID,
    START_ID,
    apply_cost_profile,
    build_mixed_mode_fixture,
    selected_route_family,
)


def _assert_route_matches_reference(graph, result, reference, expected_family):
    assert result['success'] is True
    assert result['path'][0] == START_ID
    assert result['path'][-1] == GOAL_ID
    assert len(result['path']) == len(result['nodes'])
    assert len(result['edges']) == len(result['path']) - 1
    assert all(
        target in graph.edges[source]
        for source, target in zip(result['path'], result['path'][1:])
    )
    path_cost = fsum(
        graph.edges[source][target].cost
        for source, target in zip(result['path'], result['path'][1:])
    )
    assert isclose(path_cost, result['total_cost'], rel_tol=1e-12, abs_tol=1e-12)
    assert isclose(
        result['total_cost'], reference['total_cost'],
        rel_tol=1e-12, abs_tol=1e-12,
    )
    assert selected_route_family(result) == expected_family
    assert result['nodes_expanded'] >= 0
    assert result['nodes_generated'] >= len(result['path'])
    assert result['processed_nodes'] >= 0
    assert result['edge_evaluations'] >= 0


@pytest.mark.parametrize(
    ('profile_name', 'expected_family'),
    [
        ('walk_favored', 'walk'),
        ('fly_favored', 'fly'),
        ('mixed_favored', 'mixed'),
    ],
)
def test_large_mixed_ready_graph_cost_profiles_choose_exact_complete_route(
    profile_name, expected_family
):
    fixture = build_mixed_mode_fixture(alternatives_per_family=128)
    apply_cost_profile(fixture, profile_name)
    graph = fixture.graph
    assert sum(node.mode.value == 'walk' for node in graph.nodes.values()) > 0
    assert sum(node.mode.value == 'fly' for node in graph.nodes.values()) > 0
    reference = dijkstra(graph, START_ID, GOAL_ID)

    planner = GlobalPlanner(search_method='astar')
    planner.set_graph(graph, profile_name)
    result = planner.plan(START_ID, GOAL_ID)

    assert len(graph.nodes) == 3_842
    assert sum(map(len, graph.edges.values())) == 4_224
    assert fixture.alternatives_per_family * 3 == 384
    _assert_route_matches_reference(graph, result, reference, expected_family)


def test_updated_ready_graph_costs_replan_to_fly_route_against_dijkstra():
    fixture = build_mixed_mode_fixture(alternatives_per_family=128)
    apply_cost_profile(fixture, 'walk_favored')
    graph = fixture.graph
    planner = GlobalPlanner(search_method='astar')
    planner.set_graph(graph, 'walk-costs')
    previous = planner.plan(START_ID, GOAL_ID)
    assert selected_route_family(previous) == 'walk'
    node_ids = set(graph.nodes)
    edge_pairs = {
        (source, target)
        for source, outgoing in graph.edges.items()
        for target in outgoing
    }

    apply_cost_profile(fixture, 'fly_favored')
    assert set(graph.nodes) == node_ids
    assert {
        (source, target)
        for source, outgoing in graph.edges.items()
        for target in outgoing
    } == edge_pairs
    reference = dijkstra(graph, START_ID, GOAL_ID)
    result = planner.replan(graph, 'fly-costs', START_ID, GOAL_ID)

    assert not planner.is_current(previous)
    assert result['graph_revision'] == 'fly-costs'
    assert result['path'] != previous['path']
    assert result['total_cost'] < previous['total_cost']
    _assert_route_matches_reference(graph, result, reference, 'fly')


def test_larger_mixed_ready_graph_scales_and_matches_dijkstra():
    fixture = build_mixed_mode_fixture(alternatives_per_family=512)
    apply_cost_profile(fixture, 'mixed_favored')
    graph = fixture.graph
    reference = dijkstra(graph, START_ID, GOAL_ID)

    planner = GlobalPlanner(search_method='astar')
    planner.set_graph(graph, 'large-mixed')
    result = planner.plan(START_ID, GOAL_ID)

    assert len(graph.nodes) == 15_362
    assert sum(map(len, graph.edges.values())) == 16_896
    assert fixture.alternatives_per_family * 3 == 1_536
    _assert_route_matches_reference(graph, result, reference, 'mixed')
