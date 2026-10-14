"""Deterministic supplied-graph fixtures for planner benchmarking."""

from m4_global_planner.graph import Edge, Graph, MotionMode, Node


def _grid(size, mode=MotionMode.WALK, blocked_crossings=(), weighted_crossing=None):
    graph = Graph()
    for y in range(size):
        for x in range(size):
            node_id = y * size + x
            graph.add_node(Node(node_id, x, y, 0, mode))
    blocked = {frozenset(pair) for pair in blocked_crossings}
    for y in range(size):
        for x in range(size):
            source = y * size + x
            for nx, ny in (
                (x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1),
                (x + 1, y + 1), (x + 1, y - 1),
                (x - 1, y + 1), (x - 1, y - 1),
            ):
                if not (0 <= nx < size and 0 <= ny < size):
                    continue
                target = ny * size + nx
                if frozenset((source, target)) in blocked:
                    continue
                cost = 2 ** 0.5 if nx != x and ny != y else 1.0
                if weighted_crossing is not None:
                    edge_cost = weighted_crossing(source, target, x, y, nx, ny)
                    if edge_cost is not None:
                        cost = edge_cost
                graph.add_edge(Edge(source, target, cost))
    return graph


def _narrow_route(length=300):
    graph = Graph()
    for x in range(length):
        graph.add_node(Node(x, x, 0, 0, MotionMode.WALK))
    for x in range(length - 1):
        graph.add_edge(Edge(x, x + 1, 1.0))
        graph.add_edge(Edge(x + 1, x, 1.0))
    for x in range(length - 1):
        branch = length + x
        graph.add_node(Node(branch, x, 1, 0, MotionMode.WALK))
        graph.add_edge(Edge(x, branch, 1.0))
        graph.add_edge(Edge(branch, x, 1.0))
    return graph, 0, length - 1


def _multimodal_demo():
    graph = Graph()
    for node_id, x, mode in (
        (0, 0, MotionMode.WALK),
        (1, 1, MotionMode.WALK),
        (2, 0, MotionMode.FLY),
        (3, 10, MotionMode.FLY),
        (4, 10, MotionMode.WALK),
    ):
        graph.add_node(Node(node_id, x, 0, 0, mode))
    for source, target, cost in (
        (0, 1, 3), (1, 4, 3), (0, 2, 1), (2, 3, 1), (3, 4, 1),
    ):
        graph.add_edge(Edge(source, target, cost))
    return graph, 0, 4


def _weighted_tradeoff(length=1000):
    graph = Graph()
    start, goal, trap = 0, length + 3, 1
    graph.add_node(Node(start, 0, 0, 0, MotionMode.WALK))
    graph.add_node(Node(trap, length * 0.01 - 1, 0, 0, MotionMode.WALK))
    graph.add_node(Node(goal, length * 0.01, 0, 0, MotionMode.WALK))
    graph.add_edge(Edge(start, trap, 14.0))
    graph.add_edge(Edge(trap, goal, 1.0))
    first_chain = 2
    graph.add_node(Node(first_chain, 0, 1, 0, MotionMode.WALK))
    graph.add_edge(Edge(start, first_chain, 1.0))
    previous = first_chain
    for index in range(1, length + 1):
        node_id = first_chain + index
        graph.add_node(Node(node_id, index * 0.01, 1, 0, MotionMode.WALK))
        graph.add_edge(Edge(previous, node_id, 0.01))
        previous = node_id
    graph.add_edge(Edge(previous, goal, 1.0))
    return graph, start, goal


def dynamic_corridor(length=300):
    """Two disjoint routes: a short corridor and a longer alternate corridor."""
    graph = Graph()
    for x in range(length + 1):
        graph.add_node(Node(x, x, 0, 0, MotionMode.WALK))
    for x in range(length):
        graph.add_edge(Edge(x, x + 1, 1.0))
        graph.add_edge(Edge(x + 1, x, 1.0))

    alternate_start = length + 1
    alternate_length = length * 2
    graph.add_node(Node(alternate_start, 0, 10, 0, MotionMode.WALK))
    graph.add_edge(Edge(0, alternate_start, 1.0))
    graph.add_edge(Edge(alternate_start, 0, 1.0))
    previous = alternate_start
    for index in range(1, alternate_length):
        node_id = alternate_start + index
        x = index * length / alternate_length
        graph.add_node(Node(node_id, x, 10, 0, MotionMode.WALK))
        graph.add_edge(Edge(previous, node_id, 1.0))
        graph.add_edge(Edge(node_id, previous, 1.0))
        previous = node_id
    graph.add_edge(Edge(previous, length, 1.0))
    graph.add_edge(Edge(length, previous, 1.0))
    return graph, 0, length


def scenarios():
    alternative_cost = lambda source, target, x, y, nx, ny: (
        12.0 if (x, y, nx, ny) in (
            (24, 10, 25, 10), (25, 10, 24, 10),
            (24, 11, 25, 11), (25, 11, 24, 11),
            (24, 12, 25, 12), (25, 12, 24, 12),
            (24, 13, 25, 13), (25, 13, 24, 13),
        ) else None
    )
    cases = {
        'small_static_grid': (_grid(12), 0, 143, 'Small static reference grid'),
        'medium_static_grid': (_grid(40), 0, 1599, 'Medium scalability grid'),
        'large_static_grid': (_grid(100), 0, 9999, 'Large scalability grid'),
        'very_large_static_grid': (_grid(200), 0, 39999, 'Very large scalability grid'),
        'walk_only_grid': (_grid(30, MotionMode.WALK), 0, 899, 'WALK-only route'),
        'multimodal_supplied_graph': (*_multimodal_demo(), 'Existing WALK/FLY supplied graph'),
        'weighted_astar_tradeoff': (
            *_weighted_tradeoff(),
            'Long optimal corridor versus short geometrically attractive suboptimal route',
        ),
        'unreachable_goal': (
            _grid(40, blocked_crossings=tuple(
                (y * 40 + 19, (y + delta) * 40 + 20)
                for y in range(40) for delta in (-1, 0, 1)
                if 0 <= y + delta < 40
            )),
            0, 1599, 'Disconnected goal behind a complete vertical cut',
        ),
        'narrow_constrained_route': (*_narrow_route(), 'Long narrow corridor with dead-end branches'),
        'multiple_alternative_costs': (
            _grid(60, weighted_crossing=alternative_cost),
            0, 3599, 'Alternative routes with a localized expensive crossing',
        ),
    }
    return {
        name: {
            'graph': graph,
            'start': start,
            'goal': goal,
            'purpose': purpose,
        }
        for name, (graph, start, goal, purpose) in cases.items()
    }
