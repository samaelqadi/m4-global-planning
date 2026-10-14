"""Global route search over a complete graph supplied by the graph-generation task."""

from math import dist, fsum, isclose, isfinite
from time import perf_counter

from m4_global_planner.astar import astar, dijkstra
from m4_global_planner.graph import Edge, Graph, MotionMode, Node


class GlobalPlanner:
    """Own search and graph updates; never generate or collision-check connections."""

    def __init__(self, *, search_method='dijkstra', cost_tolerance=1e-9):
        if search_method not in ('dijkstra', 'astar'):
            raise ValueError('Search method must be dijkstra or astar')
        if not isfinite(cost_tolerance) or cost_tolerance < 0:
            raise ValueError('Cost tolerance must be finite and nonnegative')
        self.search_method = search_method
        self.cost_tolerance = cost_tolerance
        self._graph = None
        self._revision = None
        self._snapshot_id = 0
        self._heuristic_scale = 0.0

    def set_graph(self, graph, revision):
        """Copy a complete snapshot. A rejected update disables the old snapshot."""
        self._graph = None
        self._revision = None
        self._snapshot_id += 1
        if type(revision) not in (int, str) or revision == '':
            raise ValueError('Graph revision must be an integer or nonempty string')
        if type(graph) is not Graph:
            raise TypeError('Supply an explicit Graph, not a lazy graph builder')
        copied = Graph()
        for node_id, node in graph.nodes.items():
            if type(node_id) is not int or node.node_id != node_id:
                raise ValueError('Node IDs must be integers matching their dictionary keys')
            copied.add_node(Node(node_id, node.x, node.y, node.z,
                                 MotionMode(node.mode) if node.mode is not None else None,
                                 node.yaw))
        if set(graph.edges) != set(copied.nodes):
            raise ValueError('Adjacency entries must match the supplied nodes')
        lower_bound = float('inf')
        heuristic_available = all(
            node.x is not None and node.y is not None and node.z is not None
            for node in copied.nodes.values()
        )
        for source, outgoing in graph.edges.items():
            for target, edge in outgoing.items():
                if edge.source_id != source or edge.target_id != target:
                    raise ValueError('Edge endpoints must match their adjacency keys')
                if getattr(edge, 'valid', True) is not True:
                    raise ValueError('The supplier must exclude invalid edges')
                copied.add_edge(Edge(source, target, edge.cost))
                if heuristic_available:
                    a, b = copied.nodes[source], copied.nodes[target]
                    distance = dist((a.x, a.y, a.z), (b.x, b.y, b.z))
                    if not isfinite(distance):
                        heuristic_available = False
                    elif distance > 0:
                        lower_bound = min(lower_bound, edge.cost / distance)
        # A zero scale disables geometric guidance when the graph lacks usable geometry.
        self._heuristic_scale = (
            lower_bound if heuristic_available and isfinite(lower_bound) else 0.0
        )
        self._graph = copied
        self._revision = revision

    def is_current(self, result):
        """A result belongs to the presently accepted graph snapshot."""
        return (self._graph is not None
                and result.get('snapshot_id') == self._snapshot_id
                and result.get('graph_revision') == self._revision)

    def plan(self, start_id, goal_id):
        """Search already-attached endpoints and return the supplied node/edge sequence."""
        if self._graph is None:
            raise RuntimeError('No accepted graph snapshot')
        graph = self._graph
        began = perf_counter()
        if self.search_method == 'astar':
            result = astar(graph, start_id, goal_id, heuristic_scale=self._heuristic_scale)
        else:
            result = dijkstra(graph, start_id, goal_id)
        search_ms = (perf_counter() - began) * 1000
        path = result['path']
        edges = [graph.edges[a][b] for a, b in zip(path, path[1:])]
        if result['success']:
            audited_cost = fsum(edge.cost for edge in edges)
            if (not path or path[0] != start_id or path[-1] != goal_id
                    or not isfinite(audited_cost)
                    or not isclose(audited_cost, result['total_cost'],
                                   rel_tol=self.cost_tolerance, abs_tol=self.cost_tolerance)):
                raise ValueError('Search result failed its edge/cost audit')
        nodes = [graph.nodes[node_id] for node_id in path]
        return {
            **result, 'nodes': nodes, 'edges': edges,
            'transition_edges': [edge for edge in edges
                                 if graph.nodes[edge.source_id].mode is not None
                                 and graph.nodes[edge.target_id].mode is not None
                                 and graph.nodes[edge.source_id].mode
                                 != graph.nodes[edge.target_id].mode],
            'graph_revision': self._revision, 'snapshot_id': self._snapshot_id,
            'search_method': self.search_method, 'search_ms': search_ms,
            'validation_scope': 'supplied_graph_edges_and_costs',
        }

    def replan(self, graph, revision, start_id, goal_id):
        """Accept the replacement graph, then search from the supplied current start."""
        self.set_graph(graph, revision)
        return self.plan(start_id, goal_id)
