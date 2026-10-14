"""Global route search over a complete graph supplied by the graph-generation task."""

from math import dist, fsum, isclose, isfinite
from time import perf_counter

from m4_global_planner.astar import astar, dijkstra
from m4_global_planner.graph import Edge, Graph, MotionMode, Node


class GlobalPlanner:
    """Own search and graph updates; never generate or collision-check connections."""

    def __init__(self, *, search_method='astar', cost_tolerance=1e-9):
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
        if not hasattr(graph, 'nodes') or not hasattr(graph, 'edges'):
            raise ValueError('Graph must provide node and edge dictionaries')

        copied = Graph()
        for node_id, node in graph.nodes.items():
            if type(node_id) is not int or not hasattr(node, 'node_id') or node.node_id != node_id:
                raise ValueError('Node IDs must be integers matching their dictionary keys')
            node_x = getattr(node, 'x', None)
            node_y = getattr(node, 'y', None)
            node_z = getattr(node, 'z', None)
            node_yaw = getattr(node, 'yaw', None)
            try:
                node_mode = MotionMode(node.mode) if getattr(node, 'mode', None) is not None else None
            except (TypeError, ValueError) as exc:
                raise ValueError('Node mode must be a valid MotionMode value') from exc
            try:
                copied.add_node(Node(node_id, node_x, node_y, node_z, node_mode, node_yaw))
            except (TypeError, ValueError) as exc:
                raise ValueError('Node record is malformed') from exc
        if set(graph.edges) != set(copied.nodes):
            raise ValueError('Adjacency entries must match the supplied nodes')
        lower_bound = float('inf')
        heuristic_available = all(
            node.x is not None and node.y is not None and node.z is not None
            for node in copied.nodes.values()
        )
        for source, outgoing in graph.edges.items():
            if not hasattr(outgoing, 'items'):
                raise ValueError('Adjacency entries must be dictionaries-like mappings')
            for target, edge in outgoing.items():
                if type(target) is not int:
                    raise ValueError('Edge targets must be integer node IDs')
                if getattr(edge, 'source_id', None) != source or getattr(edge, 'target_id', None) != target:
                    raise ValueError('Edge endpoints must match their adjacency keys')
                if getattr(edge, 'valid', True) is not True:
                    raise ValueError('The supplier must exclude invalid edges')
                try:
                    edge_cost = float(getattr(edge, 'cost'))
                except (TypeError, ValueError) as exc:
                    raise ValueError('Edge cost must be a finite nonnegative number') from exc
                if not isfinite(edge_cost) or edge_cost < 0:
                    raise ValueError('Edge cost must be a finite nonnegative number')
                copied.add_edge(Edge(source, target, edge_cost))
                if heuristic_available:
                    a, b = copied.nodes[source], copied.nodes[target]
                    distance = dist((a.x, a.y, a.z), (b.x, b.y, b.z))
                    if not isfinite(distance):
                        heuristic_available = False
                    elif distance > 0:
                        lower_bound = min(lower_bound, edge_cost / distance)
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
        if result['success']:
            if not path or path[0] != start_id or path[-1] != goal_id:
                raise ValueError('Search result failed its endpoint audit')
            try:
                edges = [graph.edges[a][b] for a, b in zip(path, path[1:])]
            except KeyError as exc:
                raise ValueError('Search result contains an edge absent from the accepted graph') from exc
            audited_cost = fsum(edge.cost for edge in edges)
            if (not isfinite(audited_cost)
                    or not isclose(audited_cost, result['total_cost'],
                                   rel_tol=self.cost_tolerance, abs_tol=self.cost_tolerance)):
                raise ValueError('Search result failed its edge/cost audit')
            result = {**result, 'total_cost': audited_cost}
        else:
            edges = []
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
