import heapq
from math import dist, inf, isfinite


def heuristic(node, goal):
    """Straight-line distance in 3D between two node-like objects."""
    return dist((node.x, node.y, node.z), (goal.x, goal.y, goal.z))


def build_path(parent, goal_id):
    """Reconstruct a start-to-goal path from parent links."""
    path = [goal_id]
    while goal_id in parent:
        goal_id = parent[goal_id]
        path.append(goal_id)
    path.reverse()
    return path


def dijkstra(graph, start_id, goal_id):
    """Minimum-cost search over the supplied explicit graph."""
    graph.get_node(start_id)
    graph.get_node(goal_id)

    costs = {start_id: 0.0}
    parents = {}
    queue = [(0.0, start_id)]
    explored = set()
    expanded = 0

    while queue:
        current_cost, current_id = heapq.heappop(queue)
        if current_cost > costs.get(current_id, inf):
            continue
        explored.add(current_id)
        expanded += 1

        if current_id == goal_id:
            break

        for edge in graph.get_neighbors(current_id):
            candidate = current_cost + edge.cost
            if candidate < costs.get(edge.target_id, inf):
                costs[edge.target_id] = candidate
                parents[edge.target_id] = current_id
                heapq.heappush(queue, (candidate, edge.target_id))

    success = goal_id in costs
    if not success:
        return {
            'success': False,
            'path': [],
            'total_cost': inf,
            'nodes_expanded': expanded,
            'nodes_explored': len(explored),
            'nodes_generated': len(costs),
        }

    return {
        'success': True,
        'path': build_path(parents, goal_id),
        'total_cost': costs[goal_id],
        'nodes_expanded': expanded,
        'nodes_explored': len(explored),
        'nodes_generated': len(costs),
    }


def astar(graph, start_id, goal_id, heuristic_scale=0.0):
    if not isfinite(heuristic_scale) or heuristic_scale < 0:
        raise ValueError('Heuristic scale must be finite and nonnegative')
    scale = heuristic_scale

    start = graph.get_node(start_id)
    goal = graph.get_node(goal_id)

    open_list = []
    g_cost = {start_id: 0.0}
    parent = {}
    generated_nodes = {start_id}
    nodes_expanded = 0
    explored = set()
    start_h = heuristic(start, goal) * scale if scale else 0.0
    heapq.heappush(open_list, (start_h, 0.0, start_id))

    while open_list:
        _, current_g, current_id = heapq.heappop(open_list)
        if current_g > g_cost[current_id]:
            continue

        nodes_expanded += 1
        explored.add(current_id)

        if current_id == goal_id:
            return {
                'success': True,
                'path': build_path(parent, goal_id),
                'total_cost': g_cost[goal_id],
                'nodes_expanded': nodes_expanded,
                'nodes_explored': len(explored),
                'nodes_generated': len(generated_nodes),
            }

        for edge in graph.get_neighbors(current_id):
            neighbor_id = edge.target_id
            new_g = g_cost[current_id] + edge.cost

            if neighbor_id not in g_cost or new_g < g_cost[neighbor_id]:
                g_cost[neighbor_id] = new_g
                parent[neighbor_id] = current_id
                generated_nodes.add(neighbor_id)

                neighbor = graph.get_node(neighbor_id)
                h = heuristic(neighbor, goal) * scale if scale else 0.0
                f = new_g + h
                heapq.heappush(open_list, (f, new_g, neighbor_id))

    return {
        'success': False,
        'path': [],
        'total_cost': float('inf'),
        'nodes_expanded': nodes_expanded,
        'nodes_explored': len(explored),
        'nodes_generated': len(generated_nodes),
    }
