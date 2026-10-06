from math import sqrt

from m4_global_planner.graph import Edge, Graph, MotionMode, Node


def distance(node1, node2):
    # calculate straight-line distance between two nodes
    dx = node2.x - node1.x
    dy = node2.y - node1.y
    dz = node2.z - node1.z

    return sqrt(dx**2 + dy**2 + dz**2)


def add_edge(graph, source_id, target_id, extra_cost=0.0):
    # get the two connected nodes
    source = graph.get_node(source_id)
    target = graph.get_node(target_id)

    # edge cost is distance plus extra cost
    cost = distance(source, target) + extra_cost

    graph.add_edge(Edge(source_id, target_id, cost))


def create_dummy_graph(scenario='flight_cheaper'):
    graph = Graph()

    # walking nodes
    graph.add_node(Node(1, 0.0, 0.0, 0.0, MotionMode.WALK))
    graph.add_node(Node(2, 2.0, 0.0, 0.0, MotionMode.WALK))
    graph.add_node(Node(3, 2.0, 3.0, 0.0, MotionMode.WALK))
    graph.add_node(Node(4, 6.0, 3.0, 0.0, MotionMode.WALK))
    graph.add_node(Node(5, 6.0, 0.0, 0.0, MotionMode.WALK))

    # flying nodes
    graph.add_node(Node(6, 2.0, 0.0, 1.5, MotionMode.FLY))
    graph.add_node(Node(7, 4.0, 0.0, 1.5, MotionMode.FLY))
    graph.add_node(Node(8, 6.0, 0.0, 1.5, MotionMode.FLY))

    # start connection
    add_edge(graph, 1, 2)

    if scenario == 'flight_cheaper':
        # both routes exist but flight is cheaper
        add_edge(graph, 2, 3)
        add_edge(graph, 3, 4)
        add_edge(graph, 4, 5)

        add_edge(graph, 2, 6, 1.0)
        add_edge(graph, 6, 7)
        add_edge(graph, 7, 8)
        add_edge(graph, 8, 5, 1.0)

    elif scenario == 'walk_cheaper':
        # flight has a larger extra cost
        add_edge(graph, 2, 3)
        add_edge(graph, 3, 4)
        add_edge(graph, 4, 5)

        add_edge(graph, 2, 6, 5.0)
        add_edge(graph, 6, 7)
        add_edge(graph, 7, 8)
        add_edge(graph, 8, 5, 5.0)

    elif scenario == 'walk_blocked':
        # ground route is unavailable
        add_edge(graph, 2, 6, 1.0)
        add_edge(graph, 6, 7)
        add_edge(graph, 7, 8)
        add_edge(graph, 8, 5, 1.0)

    elif scenario == 'fly_blocked':
        # flight route is unavailable
        add_edge(graph, 2, 3)
        add_edge(graph, 3, 4)
        add_edge(graph, 4, 5)

    elif scenario == 'no_path':
        # no route continues after node 2
        pass

    else:
        raise ValueError('Unknown dummy scenario')

    start_id = 1
    goal_id = 5

    return graph, start_id, goal_id
