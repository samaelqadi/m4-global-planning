"""Matched graph searches, heuristic proofs and benchmark snapshot integrity."""

from copy import deepcopy
from math import sqrt

from m4_global_planner.astar import astar, heuristic
from m4_global_planner.graph import Edge, Graph, MotionMode, Node
from m4_global_planner.joint_search_benchmark import methods, prepare, run_case, summary
from m4_global_planner.search_comparison import dijkstra, verify_heuristic
import pytest
from test_graph_optimality import dijkstra_path
from test_joint_graph import prepared
from test_straight_walk_edges import DATA


@pytest.mark.parametrize('name', ['T02_blocked_morphing_space',
                                  'T04_multiple_transition_choices', 'T05_flight_bridge',
                                  'T06_walk_only'])
def test_all_methods_on_identical_joint_graph(name):
    planner = prepared(name, rate=.2)
    graph = planner.graph
    original_nodes, original_edges = deepcopy(graph.nodes), deepcopy(graph.edges)
    start, goal = (planner.endpoints[key] for key in ('start', 'goal'))
    proof = verify_heuristic(graph, goal, planner.scale, 1e-9)
    assert proof['edges_checked'] == sum(map(len, graph.edges.values()))
    expected, _ = dijkstra_path(graph, start, goal)
    reference = dijkstra(graph, start, goal)
    assert reference['total_cost'] == pytest.approx(expected)
    for weight in [1] + DATA['joint_search_benchmark']['weighted_astar_weights']:
        result = astar(graph, start, goal, planner.scale, weight)
        planner.validate_route(result['path'], start, goal, result['total_cost'])
        assert result['total_cost'] >= expected - 1e-9
        if weight == 1:
            assert result['total_cost'] == pytest.approx(expected)
        assert result['nodes_explored'] <= result['nodes_expanded']
    assert graph.nodes == original_nodes and graph.edges == original_edges


def test_heuristic_lower_bound_and_measured_weighted_gap():
    graph = Graph()
    for i, xyz in enumerate(((1, 2, 0), (2, 3, 0), (4, 2, 0))):
        graph.add_node(Node(i, *xyz, MotionMode.FLY))
    for source, target, cost in ((0, 1, sqrt(2)), (1, 2, sqrt(5)), (0, 2, 4)):
        graph.add_edge(Edge(source, target, cost))
    verify_heuristic(graph, 2, 1, 1e-9)
    for node in graph.nodes.values():
        expected, _ = dijkstra_path(graph, node.node_id, 2)
        assert heuristic(node, graph.nodes[2]) <= expected
    exact = astar(graph, 0, 2)
    weighted = astar(graph, 0, 2, heuristic_weight=2)
    assert exact['total_cost'] == pytest.approx(dijkstra(graph, 0, 2)['total_cost'])
    assert weighted['total_cost'] > exact['total_cost']
    graph.edges[0][2] = Edge(0, 2, .1)
    with pytest.raises(ValueError, match='lower bound'):
        verify_heuristic(graph, 2, 1, 1e-9)


@pytest.mark.parametrize('scale,weight', [
    (-1, 1), (float('nan'), 1), (1, .5), (1, float('inf')),
])
def test_invalid_search_settings(scale, weight):
    with pytest.raises(ValueError):
        astar(Graph(), 0, 1, scale, weight)


def test_benchmark_snapshot_and_route_audits(tmp_path):
    case = DATA['joint_search_benchmark']['cases'][5]
    snapshot = tmp_path / 'graph.pickle'
    metadata = prepare(DATA, case, snapshot)
    rows = []
    for method, weight in methods(DATA['joint_search_benchmark']):
        job = dict(snapshot=str(snapshot), method=method, weight=weight,
                   cost_tolerance=1e-7, case=case, **metadata)
        row = run_case(job)
        assert row['validated'] and row['success']
        assert row['graph_sha256'] == metadata['graph_sha256']
        assert row['search_python_peak_mib'] > 0
        rows.append(row)
    summary(rows, tmp_path / 'comparison.md')
    assert 'wA* 2' in (tmp_path / 'comparison.md').read_text()
    snapshot.write_bytes(b'corrupt')
    with pytest.raises(ValueError, match='snapshot changed'):
        run_case(job)


def test_no_path_and_trivial_results():
    graph = Graph()
    for i in range(2):
        graph.add_node(Node(i, i, 0, 0, MotionMode.FLY))
    for search in (dijkstra, astar):
        result = search(graph, 0, 1)
        assert not result['success'] and result['path'] == []
        same = search(graph, 0, 0)
        assert same['success'] and same['total_cost'] == 0 and same['path'] == [0]


def test_yaml_weights_reject_invalid_values():
    for weights in ([], [1], [2, 2], [float('nan')]):
        with pytest.raises(ValueError):
            methods({'weighted_astar_weights': weights})
