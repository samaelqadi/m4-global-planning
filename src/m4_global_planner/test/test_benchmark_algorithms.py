from math import inf

import pytest

from m4_global_planner.astar import dijkstra
from m4_global_planner.benchmark_algorithms import (
    DStarLite,
    LPAStar,
    bidirectional_astar,
    bidirectional_dijkstra,
    safe_heuristic_scale,
    weighted_astar,
)
from m4_global_planner.graph import Edge, Graph, Node


def make_graph():
    graph = Graph()
    for node_id, x, y in ((0, 0, 0), (1, 1, 0), (2, 0, 1), (3, 1, 1)):
        graph.add_node(Node(node_id, x, y, 0))
    for source, target, cost in (
        (0, 1, 1), (1, 0, 1), (0, 2, 1), (2, 0, 1),
        (1, 3, 1), (3, 1, 1), (2, 3, 1), (3, 2, 1),
    ):
        graph.add_edge(Edge(source, target, cost))
    return graph


@pytest.mark.parametrize(
    'search',
    [
        lambda g: weighted_astar(g, 0, 3, 1.0, safe_heuristic_scale(g)),
        lambda g: weighted_astar(g, 0, 3, 1.0, 0.0),
        lambda g: bidirectional_dijkstra(g, 0, 3),
        lambda g: bidirectional_astar(g, 0, 3),
    ],
)
def test_static_variants_match_dijkstra_and_return_valid_paths(search):
    graph = make_graph()
    expected = dijkstra(graph, 0, 3)
    result = search(graph)
    assert result['success']
    assert result['total_cost'] == expected['total_cost']
    assert result['path'][0] == 0 and result['path'][-1] == 3
    assert all(target in graph.edges[source]
               for source, target in zip(result['path'], result['path'][1:]))
    assert result['edge_evaluations'] >= 0


@pytest.mark.parametrize('planner_type', [LPAStar, DStarLite])
def test_incremental_algorithms_repair_routes_after_repeated_changes(planner_type):
    graph = make_graph()
    planner = planner_type(graph, 0, 3)

    def assert_optimal(result):
        reference = dijkstra(graph, 0, 3)
        assert result['success'] == reference['success']
        assert result['total_cost'] == reference['total_cost']
        if result['success']:
            assert result['path'][0] == 0 and result['path'][-1] == 3
            assert all(target in graph.edges[source]
                       for source, target in zip(result['path'], result['path'][1:]))
        return result

    assert assert_optimal(planner.plan())['total_cost'] == 2
    planner.update_edge(0, 1, None)
    assert assert_optimal(planner.replan())['total_cost'] == 2
    planner.update_edge(0, 2, 8)
    assert assert_optimal(planner.replan())['total_cost'] == 9
    planner.update_edge(0, 1, 1)
    assert assert_optimal(planner.replan())['total_cost'] == 2
    planner.update_edge(0, 1, None)
    planner.update_edge(0, 2, None)
    assert not assert_optimal(planner.replan())['success']
    planner.update_edge(0, 2, 1)
    assert assert_optimal(planner.replan())['total_cost'] == 2
    planner.update_edge(0, 2, 0)
    assert planner.scale == 0
    assert assert_optimal(planner.replan())['total_cost'] == 1


def test_weighted_astar_reports_suboptimal_route_without_claiming_optimality():
    graph = Graph()
    length = 100
    goal = length + 3
    graph.add_node(Node(0, 0, 0, 0))
    graph.add_node(Node(1, 9, 0, 0))
    graph.add_node(Node(goal, 10, 0, 0))
    graph.add_edge(Edge(0, 1, 14))
    graph.add_edge(Edge(1, goal, 1))
    graph.add_node(Node(2, 0, 1, 0))
    graph.add_edge(Edge(0, 2, 1))
    previous = 2
    for index in range(1, length + 1):
        node_id = index + 2
        graph.add_node(Node(node_id, index * 10 / length, 1, 0))
        graph.add_edge(Edge(previous, node_id, 10 / length))
        previous = node_id
    graph.add_edge(Edge(previous, goal, 1))
    scale = safe_heuristic_scale(graph)
    exact = weighted_astar(graph, 0, goal, 1.0, scale)
    intermediate = weighted_astar(graph, 0, goal, 1.25, scale)
    fast = weighted_astar(graph, 0, goal, 1.5, scale)
    fastest = weighted_astar(graph, 0, goal, 2.0, scale)
    assert exact['total_cost'] == pytest.approx(12)
    assert intermediate['total_cost'] == pytest.approx(12)
    assert fast['total_cost'] == pytest.approx(15)
    assert fastest['total_cost'] == pytest.approx(15)
    assert len(fast['path']) < len(exact['path'])
    for result in (exact, intermediate, fast, fastest):
        assert result['path'][0] == 0 and result['path'][-1] == goal
        assert all(target in graph.edges[source]
                   for source, target in zip(result['path'], result['path'][1:]))


def test_bidirectional_searches_handle_unreachable_and_same_node():
    graph = make_graph()
    graph.edges[0].clear()
    for search in (bidirectional_dijkstra, bidirectional_astar):
        result = search(graph, 0, 3)
        assert not result['success']
        assert result['total_cost'] == inf
        assert search(graph, 1, 1)['path'] == [1]


@pytest.mark.parametrize('planner_type', [LPAStar, DStarLite])
@pytest.mark.parametrize('removed', [(0, 1), (2, 3)])
def test_zero_cost_cycle_update_resets_stale_labels(planner_type, removed):
    graph = Graph()
    for node_id in range(4):
        graph.add_node(Node(node_id, node_id, 0, 0))
    for a, b, cost in [(0, 1, 1), (1, 2, 0), (2, 1, 0), (2, 3, 1), (0, 3, 5)]:
        graph.add_edge(Edge(a, b, cost))
    planner = planner_type(graph, 0, 3)
    assert planner.plan()['total_cost'] == 2
    planner.update_edge(*removed, None)
    result = planner.replan()
    assert result['repair_strategy'] == 'full_reset_zero_cost'
    assert result['path'] == [0, 3]
    assert result['total_cost'] == dijkstra(graph, 0, 3)['total_cost'] == 5
    planner.update_edge(*removed, 1)
    assert planner.replan()['total_cost'] == 2
