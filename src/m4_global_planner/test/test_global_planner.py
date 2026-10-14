"""Global-planner contract checks without map or graph-generation imports."""

import hashlib
import json
from math import inf
from types import SimpleNamespace

import pytest

from m4_global_planner.global_planner import GlobalPlanner
from m4_global_planner.graph import Edge, Graph, MotionMode, Node


def supplied_graph():
    graph = Graph()
    for node_id, x, mode in [(0, 0, 'walk'), (1, 1, 'walk'), (2, 0, 'fly'),
                             (3, 10, 'fly'), (4, 10, 'walk')]:
        graph.add_node(Node(node_id, x, 0, 0, MotionMode(mode), 0))
    for a, b, cost in [(0, 1, 3), (1, 4, 3), (0, 2, 1), (2, 3, 1), (3, 4, 1)]:
        graph.add_edge(Edge(a, b, cost))
    return graph


def reference_cost(graph, start, goal):
    # Independent repeated relaxation on the supplied explicit edges.
    costs = {node_id: inf for node_id in graph.nodes}
    costs[start] = 0
    for _ in range(len(graph.nodes) - 1):
        for source, outgoing in graph.edges.items():
            for edge in outgoing.values():
                costs[edge.target_id] = min(costs[edge.target_id], costs[source] + edge.cost)
    return costs[goal]


def graph_fingerprint(graph):
    nodes = [
        (node_id, node.x, node.y, node.z,
         node.mode.value if node.mode is not None else None, node.yaw)
        for node_id, node in sorted(graph.nodes.items())
    ]
    edges = [
        (source, target, edge.cost)
        for source, outgoing in sorted(graph.edges.items())
        for target, edge in sorted(outgoing.items())
    ]
    serialized = json.dumps((nodes, edges), separators=(',', ':'), allow_nan=False)
    return hashlib.sha256(serialized.encode('utf-8')).hexdigest()


@pytest.mark.parametrize('method', ['dijkstra', 'astar'])
def test_selects_whole_multimodal_route_on_supplied_costs(method):
    graph = supplied_graph()
    planner = GlobalPlanner(search_method=method)
    planner.set_graph(graph, 'v1')
    result = planner.plan(0, 4)
    assert result['path'] == [0, 2, 3, 4]
    assert result['total_cost'] == reference_cost(graph, 0, 4) == 3
    assert len(result['transition_edges']) == 2
    assert [n.mode for n in result['nodes']] == ['walk', 'fly', 'fly', 'walk']
    assert planner.is_current(result)
    assert not planner.plan(4, 0)['success']
    assert planner.plan(2, 2)['total_cost'] == 0
    assert planner.plan(2, 2)['path'] == [2]


@pytest.mark.parametrize('method', ['dijkstra', 'astar'])
def test_updates_costs_block_routes_and_allow_changed_start(method):
    graph = supplied_graph()
    planner = GlobalPlanner(search_method=method)
    planner.set_graph(graph, 1)
    old = planner.plan(0, 4)
    graph.edges[2][3] = Edge(2, 3, 10)
    result = planner.replan(graph, 2, 0, 4)
    assert not planner.is_current(old)
    assert result['path'] == [0, 1, 4] and result['total_cost'] == 6
    del graph.edges[1][4]
    del graph.edges[2][3]
    assert not planner.replan(graph, 3, 0, 4)['success']
    graph.add_edge(Edge(2, 3, 1))
    assert planner.replan(graph, 4, 2, 4)['total_cost'] == 2


def test_input_is_copied_and_reused_revision_does_not_revive_old_result():
    graph = supplied_graph()
    planner = GlobalPlanner()
    planner.set_graph(graph, 'v1')
    old = planner.plan(0, 4)
    graph.edges[2].clear()
    assert planner.plan(0, 4)['total_cost'] == 3
    planner.set_graph(graph, 'v1')
    assert not planner.is_current(old)
    assert planner.plan(0, 4)['total_cost'] == 6


@pytest.mark.parametrize('defect', ['negative', 'nan', 'dangling', 'key', 'mode', 'invalid'])
def test_malformed_snapshot_fails_closed(defect):
    graph = supplied_graph()
    planner = GlobalPlanner()
    planner.set_graph(graph, 1)
    old = planner.plan(0, 4)
    if defect in ('negative', 'nan'):
        graph.edges[0][1] = SimpleNamespace(source_id=0, target_id=1,
                                          cost=-1 if defect == 'negative' else float('nan'))
    elif defect == 'dangling':
        graph.edges[0][99] = Edge(0, 99, 1)
    elif defect == 'key':
        graph.edges[0][1] = Edge(0, 2, 1)
    elif defect == 'mode':
        graph.nodes[0] = Node(0, 0, 0, 0, 'invalid')
    else:
        graph.edges[0][1] = SimpleNamespace(source_id=0, target_id=1, cost=1, valid=False)
    with pytest.raises(ValueError):
        planner.set_graph(graph, 2)
    assert not planner.is_current(old)
    with pytest.raises(RuntimeError):
        planner.plan(0, 4)


def test_missing_endpoints_and_lazy_builders_are_not_implicitly_attached_or_run():
    planner = GlobalPlanner()
    planner.set_graph(supplied_graph(), 1)
    with pytest.raises(ValueError):
        planner.plan(100, 4)

    class Builder(Graph):
        def get_neighbors(self, node_id):
            raise AssertionError('Must never generate edges here')

    with pytest.raises(TypeError, match='explicit Graph'):
        planner.set_graph(Builder(), 2)


def test_zero_cost_translation_keeps_astar_exact():
    graph = supplied_graph()
    graph.edges[2][3] = Edge(2, 3, 0)
    planner = GlobalPlanner(search_method='astar')
    planner.set_graph(graph, 1)
    assert planner.plan(0, 4)['total_cost'] == reference_cost(graph, 0, 4) == 2


@pytest.mark.parametrize('method', ['dijkstra', 'astar'])
def test_graph_without_geometry_or_mode_metadata_is_searchable(method):
    graph = Graph()
    for node_id in range(4):
        graph.add_node(Node(node_id))
    for source, target, cost in ((0, 1, 2), (1, 3, 2), (0, 2, 1), (2, 3, 5)):
        graph.add_edge(Edge(source, target, cost))

    planner = GlobalPlanner(search_method=method)
    planner.set_graph(graph, 'metadata-free')
    result = planner.plan(0, 3)

    assert result['path'] == [0, 1, 3]
    assert result['total_cost'] == 4
    assert result['transition_edges'] == []


def test_different_costs_on_same_geometry_choose_the_cheaper_supplied_route():
    graphs = []
    for route_costs in ((1, 1, 10, 10), (10, 10, 1, 1)):
        graph = Graph()
        for node_id, x, y in ((0, 0, 0), (1, 1, 0), (2, 0, 1), (3, 1, 1)):
            graph.add_node(Node(node_id, x, y, 0))
        for source, target, cost in (
            (0, 1, route_costs[0]), (1, 3, route_costs[1]),
            (0, 2, route_costs[2]), (2, 3, route_costs[3]),
        ):
            graph.add_edge(Edge(source, target, cost))
        graphs.append(graph)

    assert [
        GlobalPlanner().replan(graph, revision, 0, 3)['path']
        for revision, graph in enumerate(graphs)
    ] == [[0, 1, 3], [0, 2, 3]]


def test_dijkstra_and_astar_compare_the_same_fingerprinted_graph():
    graph = supplied_graph()
    fingerprint = graph_fingerprint(graph)
    results = []
    for method in ('dijkstra', 'astar'):
        planner = GlobalPlanner(search_method=method)
        planner.set_graph(graph, 'frozen-fixture')
        results.append(planner.plan(0, 4))

    assert graph_fingerprint(graph) == fingerprint
    assert [result['total_cost'] for result in results] == [3, 3]
    assert [result['graph_revision'] for result in results] == [
        'frozen-fixture', 'frozen-fixture',
    ]


@pytest.mark.parametrize('method', ['dijkstra', 'astar'])
def test_equal_cost_routes_and_cycles_preserve_optimal_cost(method):
    graph = Graph()
    for node_id in range(4):
        graph.add_node(Node(node_id))
    for source, target, cost in (
        (0, 1, 1), (1, 3, 1), (0, 2, 1), (2, 3, 1),
        (1, 0, 0), (2, 1, 0), (3, 2, 0),
    ):
        graph.add_edge(Edge(source, target, cost))

    planner = GlobalPlanner(search_method=method)
    planner.set_graph(graph, 1)
    result = planner.plan(0, 3)

    assert result['success']
    assert result['total_cost'] == reference_cost(graph, 0, 3) == 2
    assert result['path'][0] == 0 and result['path'][-1] == 3
    assert all(b in graph.edges[a] for a, b in zip(result['path'], result['path'][1:]))


def test_graph_updates_support_new_cheaper_route_and_changed_goal():
    graph = Graph()
    for node_id in range(4):
        graph.add_node(Node(node_id))
    graph.add_edge(Edge(0, 1, 8))
    graph.add_edge(Edge(1, 3, 8))
    planner = GlobalPlanner()
    planner.set_graph(graph, 1)
    assert planner.plan(0, 3)['total_cost'] == 16

    graph.add_edge(Edge(0, 2, 1))
    graph.add_edge(Edge(2, 3, 1))
    cheaper = planner.replan(graph, 2, 0, 3)
    changed_goal = planner.replan(graph, 3, 0, 1)

    assert cheaper['path'] == [0, 2, 3] and cheaper['total_cost'] == 2
    assert changed_goal['path'] == [0, 1] and changed_goal['total_cost'] == 8


def test_replanning_handles_multiple_cost_changes_in_one_snapshot():
    graph = supplied_graph()
    planner = GlobalPlanner()
    planner.set_graph(graph, 1)
    assert planner.plan(0, 4)['total_cost'] == 3

    graph.edges[0][2] = Edge(0, 2, 20)
    graph.edges[1][4] = Edge(1, 4, 1)
    graph.edges[3][4] = Edge(3, 4, 20)
    result = planner.replan(graph, 2, 0, 4)

    assert result['path'] == [0, 1, 4]
    assert result['total_cost'] == 4


@pytest.mark.parametrize('method', ['dijkstra', 'astar'])
def test_dense_and_large_sparse_graphs_match_reference(method):
    dense = Graph()
    for node_id in range(12):
        dense.add_node(Node(node_id))
    for source in dense.nodes:
        for target in dense.nodes:
            if source != target:
                dense.add_edge(Edge(source, target, 1 + ((source + target) % 5)))
    dense_planner = GlobalPlanner(search_method=method)
    dense_planner.set_graph(dense, 'dense')
    assert dense_planner.plan(0, 11)['total_cost'] == reference_cost(dense, 0, 11)

    sparse = Graph()
    for node_id in range(250):
        sparse.add_node(Node(node_id))
    for node_id in range(249):
        sparse.add_edge(Edge(node_id, node_id + 1, 1))
        if node_id + 7 < 250:
            sparse.add_edge(Edge(node_id, node_id + 7, 10))
    sparse_planner = GlobalPlanner(search_method=method)
    sparse_planner.set_graph(sparse, 'large-sparse')
    result = sparse_planner.plan(0, 249)
    assert result['total_cost'] == reference_cost(sparse, 0, 249) == 249


@pytest.mark.parametrize('method', ['dijkstra', 'astar'])
def test_all_node_pairs_match_independent_relaxation(method):
    graph = supplied_graph()
    graph.add_edge(Edge(3, 2, 0))
    planner = GlobalPlanner(search_method=method)
    planner.set_graph(graph, 1)
    for start in graph.nodes:
        for goal in graph.nodes:
            expected = reference_cost(graph, start, goal)
            result = planner.plan(start, goal)
            assert result['total_cost'] == expected
            assert result['success'] == (expected != inf)
            if result['success']:
                assert sum(e.cost for e in result['edges']) == result['total_cost']

@pytest.mark.parametrize('method', ['dijkstra', 'astar'])
def test_small_directed_graphs_match_reference(method):
    import random
    for seed in (11, 29, 47):
        rng = random.Random(seed)
        graph = Graph()
        for node_id in range(8):
            graph.add_node(Node(node_id, rng.random()*10, rng.random()*10,
                                rng.random()*3, MotionMode.WALK if node_id % 2 else MotionMode.FLY))
        for a in graph.nodes:
            for b in graph.nodes:
                if a != b and rng.random() < 0.25:
                    graph.add_edge(Edge(a, b, rng.choice((0, 0.25, 1, 5))))
        planner = GlobalPlanner(search_method=method)
        planner.set_graph(graph, seed)
        for a in graph.nodes:
            for b in graph.nodes:
                result = planner.plan(a, b)
                expected = reference_cost(graph, a, b)
                assert result['total_cost'] == expected
                assert result['success'] == (expected != inf)
                if result['success']:
                    assert result['path'][0] == a and result['path'][-1] == b
                    assert sum(e.cost for e in result['edges']) == expected


@pytest.mark.parametrize('revision', ['', None, True, 1.5])
def test_invalid_revision_disables_previous_snapshot(revision):
    planner = GlobalPlanner()
    planner.set_graph(supplied_graph(), 1)
    result = planner.plan(0, 4)
    with pytest.raises(ValueError):
        planner.set_graph(supplied_graph(), revision)
    assert not planner.is_current(result)
    with pytest.raises(RuntimeError):
        planner.plan(0, 4)


@pytest.mark.parametrize('method', ['dijkstra', 'astar'])
def test_empty_and_isolated_nodes(method):
    planner = GlobalPlanner(search_method=method)
    planner.set_graph(Graph(), 1)
    with pytest.raises(ValueError):
        planner.plan(0, 0)
    graph = Graph()
    for i in (0, 1):
        graph.add_node(Node(i, i, 0, 0, MotionMode.WALK))
    planner.set_graph(graph, 2)
    assert planner.plan(0, 0)['path'] == [0]
    assert not planner.plan(0, 1)['success']


@pytest.mark.parametrize('method', ['dijkstra', 'astar'])
def test_large_finite_coordinates_do_not_overflow_distance(method):
    graph = Graph()
    graph.add_node(Node(0, 0, 0, 0, MotionMode.WALK))
    graph.add_node(Node(1, 1e200, 0, 0, MotionMode.WALK))
    graph.add_edge(Edge(0, 1, 1))
    planner = GlobalPlanner(search_method=method)
    planner.set_graph(graph, 1)
    assert planner.plan(0, 1)['total_cost'] == 1


def test_graph_rejects_duplicate_nodes_edges_and_self_edges():
    graph = supplied_graph()
    with pytest.raises(ValueError):
        graph.add_node(graph.nodes[0])
    with pytest.raises(ValueError):
        graph.add_edge(graph.edges[0][1])
    with pytest.raises(ValueError):
        Edge(0, 0, 0)
    with pytest.raises(ValueError):
        graph.add_edge(Edge(99, 0, 1))


@pytest.mark.parametrize('value', [float('nan'), float('inf'), -float('inf')])
def test_nonfinite_node_and_edge_values_rejected(value):
    with pytest.raises(ValueError):
        Node(0, value, 0, 0, MotionMode.WALK)
    with pytest.raises(ValueError):
        Node(0, 0, 0, 0, MotionMode.WALK, value)
    with pytest.raises(ValueError):
        Edge(0, 1, value)


def test_partial_node_geometry_is_rejected():
    with pytest.raises(ValueError, match='complete or omitted'):
        Node(0, x=1.0, y=None, z=0.0)
