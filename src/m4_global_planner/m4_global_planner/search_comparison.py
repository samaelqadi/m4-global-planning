"""Independent Dijkstra reference and finite-graph heuristic verification."""

import heapq
from math import dist, inf, isfinite

from m4_global_planner.astar import build_path, heuristic
from m4_global_planner.representation.fly_validator import position


def dijkstra(graph, start, goal):
    """Search directed nonnegative edges without using A* or its heuristic."""
    graph.get_node(start)
    graph.get_node(goal)
    costs, parents = {start: 0.0}, {}
    queue, explored, expanded = [(0.0, start)], set(), 0
    while queue:
        cost, node_id = heapq.heappop(queue)
        if cost != costs[node_id]:
            continue
        explored.add(node_id)
        expanded += 1
        if node_id == goal:
            break
        for edge in graph.get_neighbors(node_id):
            candidate = cost + edge.cost
            if candidate < costs.get(edge.target_id, inf):
                costs[edge.target_id] = candidate
                parents[edge.target_id] = node_id
                heapq.heappush(queue, (candidate, edge.target_id))
    success = goal in explored
    return {'success': success, 'path': build_path(parents, goal) if success else [],
            'total_cost': costs[goal] if success else inf, 'nodes_expanded': expanded,
            'nodes_explored': len(explored), 'nodes_generated': len(costs)}


def verify_heuristic(graph, goal, scale, tolerance):
    """Check all edges, not just a route, for the Euclidean cost lower bound."""
    if not isfinite(scale) or scale < 0 or not isfinite(tolerance) or tolerance <= 0:
        raise ValueError('Invalid heuristic verification settings')
    target = graph.get_node(goal)
    values = {i: scale * heuristic(node, target) for i, node in graph.nodes.items()}
    if values[goal] != 0 or not all(isfinite(h) and h >= 0 for h in values.values()):
        raise ValueError('Invalid heuristic values')
    maximum_bound_error, maximum_consistency_error, checked = 0.0, 0.0, 0
    for source, edges in graph.edges.items():
        for end, edge in edges.items():
            if not isfinite(edge.cost) or edge.cost < 0:
                raise ValueError('Invalid graph cost')
            bound = scale * dist(position(graph.nodes[source]), position(graph.nodes[end]))
            error = bound - edge.cost
            consistency = values[source] - edge.cost - values[end]
            if error > tolerance or consistency > tolerance:
                raise ValueError('Heuristic exceeds an edge cost lower bound')
            maximum_bound_error = max(maximum_bound_error, error)
            maximum_consistency_error = max(maximum_consistency_error, consistency)
            checked += 1
    return {'edges_checked': checked, 'goal_heuristic': values[goal],
            'maximum_bound_error': maximum_bound_error,
            'maximum_consistency_error': maximum_consistency_error,
            'tolerance': tolerance}
