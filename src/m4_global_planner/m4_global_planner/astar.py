import heapq
from math import sqrt


def heuristic(node, goal):
    # straight-line distance to the goal
    dx = goal.x - node.x
    dy = goal.y - node.y
    dz = goal.z - node.z

    return sqrt(dx**2 + dy**2 + dz**2)


def build_path(parent, goal_id):
    # start from the goal
    path = [goal_id]

    # move backward until reaching the start
    while goal_id in parent:
        goal_id = parent[goal_id]
        path.append(goal_id)

    # change goal-to-start into start-to-goal
    path.reverse()

    return path


def astar(graph, start_id, goal_id, heuristic_scale=1.0):
    # get start and goal nodes
    start = graph.get_node(start_id)
    goal = graph.get_node(goal_id)

    # priority queue used by A*
    open_list = []

    # best known cost from start to every discovered node
    g_cost = {start_id: 0.0}

    # stores the previous node used to rebuild the path
    parent = {}

    # stores every unique node discovered by A*
    generated_nodes = {start_id}

    # counts how many nodes A* actually checks
    nodes_expanded = 0

    # calculate the first heuristic
    start_h = heuristic(start, goal) * heuristic_scale

    # store f cost, g cost and node id
    heapq.heappush(open_list, (start_h, 0.0, start_id))

    while open_list:
        # get the node with the smallest f cost
        _, current_g, current_id = heapq.heappop(open_list)

        # ignore an old queue entry if a better path was already found
        if current_g > g_cost[current_id]:
            continue

        nodes_expanded += 1

        # stop when the goal is reached
        if current_id == goal_id:
            return {
                'success': True,
                'path': build_path(parent, goal_id),
                'total_cost': g_cost[goal_id],
                'nodes_expanded': nodes_expanded,
                'nodes_generated': len(generated_nodes)
            }

        # check every possible movement from the current node
        for edge in graph.get_neighbors(current_id):
            neighbor_id = edge.target_id

            # calculate the new cost to reach this neighbor
            new_g = g_cost[current_id] + edge.cost

            # continue only if this path is better
            if neighbor_id not in g_cost or new_g < g_cost[neighbor_id]:
                g_cost[neighbor_id] = new_g
                parent[neighbor_id] = current_id

                # count this node only once when first discovered
                generated_nodes.add(neighbor_id)

                neighbor = graph.get_node(neighbor_id)

                # estimate remaining distance to the goal
                h = heuristic(neighbor, goal) * heuristic_scale

                # A* priority
                f = new_g + h

                heapq.heappush(open_list, (f, new_g, neighbor_id))

    # no path was found
    return {
        'success': False,
        'path': [],
        'total_cost': float('inf'),
        'nodes_expanded': nodes_expanded,
        'nodes_generated': len(generated_nodes)
    }
