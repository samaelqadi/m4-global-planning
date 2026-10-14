"""Toy upstream graph used only to demonstrate the graph-input planner."""
from m4_global_planner.global_planner import GlobalPlanner
from m4_global_planner.graph import Edge, Graph, MotionMode, Node


def supplied_fixture():
    graph = Graph()
    for node_id, x, mode in [(0, 0, 'walk'), (1, 1, 'walk'), (2, 0, 'fly'),
                             (3, 10, 'fly'), (4, 10, 'walk')]:
        graph.add_node(Node(node_id, x, 0, 0, MotionMode(mode), 0))
    for source, target, cost in [(0, 1, 3), (1, 4, 3), (0, 2, 1), (2, 3, 1), (3, 4, 1)]:
        graph.add_edge(Edge(source, target, cost))
    return graph


def show(result):
    print('revision:', result['graph_revision'], 'success:', result['success'],
          'path:', result['path'], 'cost:', result['total_cost'])


if __name__ == '__main__':
    graph = supplied_fixture()
    planner = GlobalPlanner()
    planner.set_graph(graph, 1)
    show(planner.plan(0, 4))
    graph.edges[2][3] = Edge(2, 3, 10)
    show(planner.replan(graph, 2, 0, 4))
    del graph.edges[1][4]
    del graph.edges[2][3]
    show(planner.replan(graph, 3, 0, 4))
