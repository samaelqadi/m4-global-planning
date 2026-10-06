"""Finite joint geometric WALK/FLY search; no takeoff or landing model."""

from math import dist, isclose, isfinite

from m4_global_planner.astar import astar
from m4_global_planner.graph import Edge, MotionMode, Node
from m4_global_planner.representation.fly_graph import build_graph as build_fly_graph
from m4_global_planner.representation.fly_validator import position
from m4_global_planner.representation.states import PoseState
from m4_global_planner.search_comparison import dijkstra
from m4_global_planner.walk_comparison import explicit_lattice, Motions


class JointPlanner:
    """Search one complete geometric graph; Dijkstra is the default."""

    def __init__(self, parameters, settings, planning_map, builder, endpoints, seed=0):
        self.cost_tolerance = settings.get('cost_tolerance', 1e-10)
        if not isfinite(self.cost_tolerance) or self.cost_tolerance <= 0:
            raise ValueError('Cost tolerance must be finite and positive')
        self.search_method = settings.get('search_method', 'dijkstra')
        self.search_weight = settings.get('weighted_astar_weight', 1.5)
        if self.search_method not in ('dijkstra', 'astar', 'weighted_astar'):
            raise ValueError('Unsupported joint search method')
        if not isfinite(self.search_weight) or self.search_weight < 1:
            raise ValueError('Weighted A* weight must be finite and at least one')
        self.map, self.builder = planning_map, builder
        self.motions = Motions(parameters, planning_map)
        self.penalty = parameters['transition_extra_cost']
        self.fly_rate = settings['fly_graph']['distance_cost_per_meter']
        radius = settings['endpoint_radius']
        if not isfinite(self.penalty) or self.penalty < 0:
            raise ValueError('Transition penalty must be finite and nonnegative')
        if not isfinite(radius) or not 0 < radius <= parameters['straight_walk']['max_distance']:
            raise ValueError('Endpoint radius must be positive and within WALK motion limit')
        self.anchors = builder.build(planning_map, endpoints)
        walk_states = [anchor.walk_pose for anchor in self.anchors.transitions]
        walk_states += [self.pose(node) for node in endpoints.values()
                        if node.mode == MotionMode.WALK]
        graph, _, walk_ids, paths = explicit_lattice(
            parameters, settings, self.motions, None, None, walk_states,
        )
        fly, mapping, _ = build_fly_graph(
            settings['fly_approach'], planning_map, builder, self.anchors,
            settings['fly_graph'], settings['fly_tuning'], seed,
        )
        offset = len(graph.nodes)
        for node in fly.nodes.values():
            graph.add_node(Node(node.node_id + offset, *position(node), node.mode, node.yaw))
        for edges in fly.edges.values():
            for edge in edges.values():
                graph.add_edge(Edge(edge.source_id + offset, edge.target_id + offset, edge.cost))
        for anchor in self.anchors.transitions:
            walk_id, fly_id = walk_ids[anchor.walk_pose], mapping[anchor.fly_id] + offset
            for direction in anchor.directions:
                source, target = ((walk_id, fly_id) if direction == 'walk_to_fly'
                                  else (fly_id, walk_id))
                graph.add_edge(Edge(source, target, self.penalty))
        self.graph, self.paths = graph, paths
        self.endpoints = {
            name: (walk_ids[self.pose(node)] if node.mode == MotionMode.WALK
                   else mapping[node.node_id] + offset)
            for name, node in self.anchors.endpoints
        }
        # All movement costs bound Euclidean distance by at least this rate;
        # stationary morphs have zero displacement and nonnegative cost.
        self.scale = min(self.fly_rate, self.motions.edges.cost.rate)

    @staticmethod
    def pose(node):
        return PoseState(*position(node), node.yaw)

    def validate_route(self, route, start, goal, cost):
        """Recompute every primitive sweep, morph and cost from the current map."""
        if not route or route[0] != start or route[-1] != goal:
            raise ValueError('Route does not preserve endpoints')
        total = 0.0
        for node_id in route:
            node = self.graph.nodes[node_id]
            valid = (self.builder.fly.is_valid(node, self.map) if node.mode == MotionMode.FLY
                     else self.motions.validator.is_valid(self.pose(node), self.map))
            if not valid:
                raise ValueError('Invalid route state')
        for source, target in zip(route, route[1:]):
            a, b = self.graph.nodes[source], self.graph.nodes[target]
            if a.mode == b.mode == MotionMode.WALK:
                segment = self.paths.get((source, target), [self.pose(a), self.pose(b)])
                if segment[0] != self.pose(a) or segment[-1] != self.pose(b):
                    raise ValueError('Invalid WALK connector endpoints')
                expected = 0.0
                for first, second in zip(segment, segment[1:]):
                    edge = self.motions.edge(first, second)
                    if edge is None:
                        raise ValueError('Invalid WALK sweep')
                    expected += edge.cost
            elif a.mode == b.mode == MotionMode.FLY:
                if not self.builder.fly.is_edge_valid(a, b, self.map):
                    raise ValueError('Invalid FLY sweep')
                expected = dist(position(a), position(b)) * self.fly_rate
            else:
                walk, fly = (a, b) if a.mode == MotionMode.WALK else (b, a)
                direction = 'walk_to_fly' if a.mode == MotionMode.WALK else 'fly_to_walk'
                if not self.builder.transition.is_valid(self.pose(walk), fly, self.map, direction):
                    raise ValueError('Invalid stationary morph')
                expected = self.penalty
            edge = self.graph.edges[source][target]
            if not isclose(edge.cost, expected, rel_tol=self.cost_tolerance,
                           abs_tol=self.cost_tolerance):
                raise ValueError('Invalid edge cost')
            total += expected
        if not isclose(total, cost, rel_tol=self.cost_tolerance,
                       abs_tol=self.cost_tolerance):
            raise ValueError('Invalid total cost')

    def plan(self, start='start', goal='goal'):
        source, target = self.endpoints[start], self.endpoints[goal]
        if self.search_method == 'dijkstra':
            result = dijkstra(self.graph, source, target)
        else:
            weight = self.search_weight if self.search_method == 'weighted_astar' else 1.0
            result = astar(self.graph, source, target, heuristic_scale=self.scale,
                           heuristic_weight=weight)
        if result['success']:
            self.validate_route(result['path'], source, target, result['total_cost'])
        return result
