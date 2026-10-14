"""Additional search implementations used by the algorithm evaluation harness."""

import heapq
from collections import deque
from math import dist, fsum, inf, isclose, isfinite


def safe_heuristic_scale(graph):
    """Return a graph-wide admissible Euclidean cost scale, or zero if unavailable."""
    if not all(
        node.x is not None and node.y is not None and node.z is not None
        for node in graph.nodes.values()
    ):
        return 0.0

    lower_bound = inf
    for source, outgoing in graph.edges.items():
        for target, edge in outgoing.items():
            a, b = graph.nodes[source], graph.nodes[target]
            distance = dist((a.x, a.y, a.z), (b.x, b.y, b.z))
            if not isfinite(distance):
                return 0.0
            if distance > 0:
                lower_bound = min(lower_bound, edge.cost / distance)
    return lower_bound if isfinite(lower_bound) else 0.0


def _result(success, path, cost, expanded, generated, edge_evaluations):
    return {
        'success': success,
        'path': path,
        'total_cost': cost,
        'nodes_expanded': expanded,
        'nodes_explored': expanded,
        'nodes_generated': generated,
        'edge_evaluations': edge_evaluations,
        'processed_nodes': expanded,
    }


def weighted_astar(graph, start_id, goal_id, weight=1.0, heuristic_scale=0.0):
    """A* with a weighted admissible heuristic; weight 1 is ordinary A*."""
    if not isfinite(weight) or weight < 1:
        raise ValueError('Weight must be finite and at least 1')
    if not isfinite(heuristic_scale) or heuristic_scale < 0:
        raise ValueError('Heuristic scale must be finite and nonnegative')
    start, goal = graph.get_node(start_id), graph.get_node(goal_id)
    costs = {start_id: 0.0}
    parents = {}
    generated = {start_id}
    expanded = edge_evaluations = 0
    start_h = _heuristic(start, goal, heuristic_scale)
    queue = [(weight * start_h, 0.0, start_id)]

    while queue:
        _, current_cost, current_id = heapq.heappop(queue)
        if current_cost > costs[current_id]:
            continue
        expanded += 1
        if current_id == goal_id:
            path = _reconstruct_forward(parents, goal_id)
            return _result(True, path, current_cost, expanded, len(generated),
                           edge_evaluations)

        for edge in graph.get_neighbors(current_id):
            edge_evaluations += 1
            candidate = current_cost + edge.cost
            if candidate < costs.get(edge.target_id, inf):
                costs[edge.target_id] = candidate
                parents[edge.target_id] = current_id
                generated.add(edge.target_id)
                heuristic = _heuristic(graph.get_node(edge.target_id), goal,
                                       heuristic_scale)
                heapq.heappush(queue, (candidate + weight * heuristic,
                                       candidate, edge.target_id))

    return _result(False, [], inf, expanded, len(generated), edge_evaluations)


def bidirectional_dijkstra(graph, start_id, goal_id):
    """Exact bidirectional Dijkstra for directed graphs."""
    return _bidirectional_search(graph, start_id, goal_id, None)


def bidirectional_astar(graph, start_id, goal_id, heuristic_scale=None):
    """Exact balanced-potential bidirectional A* over nonnegative reweighted arcs."""
    scale = _incremental_scale(graph, heuristic_scale)
    return _bidirectional_search(graph, start_id, goal_id, scale)


def _bidirectional_search(graph, start_id, goal_id, heuristic_scale):
    graph.get_node(start_id)
    graph.get_node(goal_id)
    if start_id == goal_id:
        return _result(True, [start_id], 0.0, 1, 1, 0)

    potential = {}
    if heuristic_scale is not None and heuristic_scale:
        start, goal = graph.nodes[start_id], graph.nodes[goal_id]
        for node_id, node in graph.nodes.items():
            from_start = _heuristic(start, node, heuristic_scale)
            to_goal = _heuristic(node, goal, heuristic_scale)
            potential[node_id] = 0.5 * (from_start - to_goal)
    else:
        potential = {node_id: 0.0 for node_id in graph.nodes}

    reverse = {node_id: [] for node_id in graph.nodes}
    for source, outgoing in graph.edges.items():
        for target, edge in outgoing.items():
            reverse[target].append((source, edge))

    forward_cost = {start_id: 0.0}
    backward_cost = {goal_id: 0.0}
    forward_parent = {}
    backward_next = {}
    forward_queue = [(0.0, start_id)]
    backward_queue = [(0.0, goal_id)]
    forward_settled, backward_settled = set(), set()
    generated = {start_id, goal_id}
    expanded = edge_evaluations = 0
    best_cost = inf
    bridge = None

    while forward_queue and backward_queue:
        _discard_stale(forward_queue, forward_cost)
        _discard_stale(backward_queue, backward_cost)
        if not forward_queue or not backward_queue:
            break
        if forward_queue[0][0] + backward_queue[0][0] >= best_cost:
            break

        forward = forward_queue[0][0] <= backward_queue[0][0]
        queue = forward_queue if forward else backward_queue
        costs = forward_cost if forward else backward_cost
        other_costs = backward_cost if forward else forward_cost
        settled = forward_settled if forward else backward_settled
        current_cost, current = heapq.heappop(queue)
        if current in settled or current_cost > costs[current]:
            continue
        settled.add(current)
        expanded += 1
        if current in other_costs and current_cost + other_costs[current] < best_cost:
            best_cost = current_cost + other_costs[current]
            bridge = (current, current)

        arcs = (
            [(edge.target_id, edge) for edge in graph.get_neighbors(current)]
            if forward else reverse[current]
        )
        for neighbor, edge in arcs:
            edge_evaluations += 1
            adjusted = edge.cost + potential[edge.source_id] - potential[edge.target_id]
            if adjusted < 0 and adjusted > -1e-12:
                adjusted = 0.0
            if adjusted < 0:
                raise ValueError('Heuristic potential produced a negative reweighted edge')
            candidate = current_cost + adjusted
            if candidate < costs.get(neighbor, inf):
                costs[neighbor] = candidate
                generated.add(neighbor)
                if forward:
                    forward_parent[neighbor] = current
                else:
                    backward_next[neighbor] = current
                heapq.heappush(queue, (candidate, neighbor))
            if neighbor in other_costs:
                if forward:
                    original_candidate = costs[current] + adjusted + other_costs[neighbor]
                    candidate_bridge = (current, neighbor)
                else:
                    original_candidate = other_costs[neighbor] + adjusted + costs[current]
                    candidate_bridge = (neighbor, current)
                if original_candidate < best_cost:
                    best_cost, bridge = original_candidate, candidate_bridge

    if bridge is None:
        return _result(False, [], inf, expanded, len(generated), edge_evaluations)

    left, right = bridge
    path_to_left = _reconstruct_forward(forward_parent, left)
    path_from_right = []
    node = right
    while True:
        path_from_right.append(node)
        if node not in backward_next:
            break
        node = backward_next[node]
    path = path_to_left + path_from_right[1:] if left == right else path_to_left + path_from_right
    original_cost = fsum(graph.edges[a][b].cost for a, b in zip(path, path[1:]))
    return _result(True, path, original_cost, expanded, len(generated), edge_evaluations)


class LPAStar:
    """Lifelong Planning A* for repeated changes to directed edge costs."""

    def __init__(self, graph, start_id, goal_id, heuristic_scale=None):
        graph.get_node(start_id)
        graph.get_node(goal_id)
        self.graph = graph
        self._zero_cost_edges = sum(edge.cost == 0 for outgoing in graph.edges.values()
                                    for edge in outgoing.values())
        self.repair_strategy = "incremental"
        self.start = start_id
        self.goal = goal_id
        self.scale = _incremental_scale(graph, heuristic_scale)
        self.costs = {node_id: inf for node_id in graph.nodes}
        self.rhs = {node_id: inf for node_id in graph.nodes}
        self.rhs[start_id] = 0.0
        self.parents = {node_id: set() for node_id in graph.nodes}
        for source, outgoing in graph.edges.items():
            for target, edge in outgoing.items():
                self.parents[target].add(source)
        self.queue = []
        self.queued = {}
        self.counter = 0
        self.discovered = {start_id}
        self._enqueue(start_id)

    def _key(self, node_id):
        value = min(self.costs[node_id], self.rhs[node_id])
        return (value + _heuristic(self.graph.nodes[node_id],
                                   self.graph.nodes[self.goal], self.scale), value)

    def _enqueue(self, node_id):
        key = self._key(node_id)
        self.queued[node_id] = key
        self.counter += 1
        heapq.heappush(self.queue, (key, self.counter, node_id))

    def _top(self):
        while self.queue:
            key, _, node_id = self.queue[0]
            if self.queued.get(node_id) == key:
                return key
            heapq.heappop(self.queue)
        return None

    def _update_vertex(self, node_id):
        if node_id != self.start:
            self.rhs[node_id] = min(
                (self.costs[parent] + self.graph.edges[parent][node_id].cost
                 for parent in self.parents[node_id]
                 if node_id in self.graph.edges[parent]),
                default=inf,
            )
        self.queued.pop(node_id, None)
        if self.costs[node_id] != self.rhs[node_id]:
            self.discovered.add(node_id)
            self._enqueue(node_id)

    def update_edge(self, source, target, cost):
        _validate_edge_update(self.graph, source, target, cost)
        disable_heuristic = _heuristic_would_be_invalid(
            self.graph, source, target, cost, self.scale
        )
        outgoing = self.graph.edges[source]
        previous = outgoing.get(target)
        reset_zero_graph = self._zero_cost_edges > 0 or cost == 0
        self._zero_cost_edges += int(cost == 0) - int(previous is not None and previous.cost == 0)
        if cost is None:
            outgoing.pop(target, None)
            self.parents[target].discard(source)
        else:
            outgoing[target] = _EdgeView(source, target, cost)
            self.parents[target].add(source)
        if reset_zero_graph:
            # A full reset prevents zero-cost cycles from retaining stale cost labels.
            self.__init__(self.graph, self.start, self.goal)
            self.repair_strategy = "full_reset_zero_cost"
            return
        if disable_heuristic:
            self.scale = 0.0
            self.queue.clear()
            self.queued.clear()
            self._update_vertex(target)
            for node_id in self.graph.nodes:
                if node_id != target and self.costs[node_id] != self.rhs[node_id]:
                    self._enqueue(node_id)
        else:
            self._update_vertex(target)

    def plan(self):
        return self._compute()

    def replan(self):
        return self._compute()

    def _compute(self):
        expanded = edge_evaluations = 0
        explored = set()
        while True:
            top = self._top()
            goal_key = self._key(self.goal)
            if (top is None or (top >= goal_key and
                                self.rhs[self.goal] == self.costs[self.goal])):
                break
            old_key, _, current = heapq.heappop(self.queue)
            if self.queued.get(current) != old_key:
                continue
            self.queued.pop(current, None)
            new_key = self._key(current)
            if old_key < new_key:
                self._enqueue(current)
            elif self.costs[current] > self.rhs[current]:
                self.costs[current] = self.rhs[current]
                explored.add(current)
                expanded += 1
                for target in self.graph.edges[current]:
                    edge_evaluations += 1
                    self._update_vertex(target)
            else:
                self.costs[current] = inf
                explored.add(current)
                expanded += 1
                self._update_vertex(current)
                for target in self.graph.edges[current]:
                    edge_evaluations += 1
                    self._update_vertex(target)
        path = _tight_path_from_start(self.graph, self.costs, self.start, self.goal)
        success = bool(path)
        result = _result(success, path, self.costs[self.goal] if success else inf,
                         expanded, len(self.discovered), edge_evaluations)
        result['repair_strategy'] = self.repair_strategy
        return result


class DStarLite:
    """D* Lite for repeated directed edge updates with a fixed goal."""

    def __init__(self, graph, start_id, goal_id, heuristic_scale=None):
        graph.get_node(start_id)
        graph.get_node(goal_id)
        self.graph = graph
        self._zero_cost_edges = sum(edge.cost == 0 for outgoing in graph.edges.values()
                                    for edge in outgoing.values())
        self.repair_strategy = "incremental"
        self.start = start_id
        self.last_start = start_id
        self.goal = goal_id
        self.scale = _incremental_scale(graph, heuristic_scale)
        self.costs = {node_id: inf for node_id in graph.nodes}
        self.rhs = {node_id: inf for node_id in graph.nodes}
        self.rhs[goal_id] = 0.0
        self.parents = {node_id: set() for node_id in graph.nodes}
        for source, outgoing in graph.edges.items():
            for target in outgoing:
                self.parents[target].add(source)
        self.queue = []
        self.queued = {}
        self.counter = 0
        self.km = 0.0
        self.discovered = {start_id, goal_id}
        self._enqueue(goal_id)

    def _distance(self, left, right):
        return _heuristic(self.graph.nodes[left], self.graph.nodes[right], self.scale)

    def _key(self, node_id):
        value = min(self.costs[node_id], self.rhs[node_id])
        return (value + self._distance(self.start, node_id) + self.km, value)

    def _enqueue(self, node_id):
        key = self._key(node_id)
        self.queued[node_id] = key
        self.counter += 1
        heapq.heappush(self.queue, (key, self.counter, node_id))

    def _top(self):
        while self.queue:
            key, _, node_id = self.queue[0]
            if self.queued.get(node_id) == key:
                return key
            heapq.heappop(self.queue)
        return None

    def _update_vertex(self, node_id):
        if node_id != self.goal:
            self.rhs[node_id] = min(
                (edge.cost + self.costs[target]
                 for target, edge in self.graph.edges[node_id].items()),
                default=inf,
            )
        self.queued.pop(node_id, None)
        if self.costs[node_id] != self.rhs[node_id]:
            self.discovered.add(node_id)
            self._enqueue(node_id)

    def update_edge(self, source, target, cost):
        _validate_edge_update(self.graph, source, target, cost)
        disable_heuristic = _heuristic_would_be_invalid(
            self.graph, source, target, cost, self.scale
        )
        outgoing = self.graph.edges[source]
        previous = outgoing.get(target)
        reset_zero_graph = self._zero_cost_edges > 0 or cost == 0
        self._zero_cost_edges += int(cost == 0) - int(previous is not None and previous.cost == 0)
        if cost is None:
            outgoing.pop(target, None)
            self.parents[target].discard(source)
        else:
            outgoing[target] = _EdgeView(source, target, cost)
            self.parents[target].add(source)
        if reset_zero_graph:
            # A full reset prevents zero-cost cycles from retaining stale cost labels.
            self.__init__(self.graph, self.start, self.goal)
            self.repair_strategy = "full_reset_zero_cost"
            return
        if disable_heuristic:
            self.scale = 0.0
            self.km = 0.0
            self.queue.clear()
            self.queued.clear()
            self._update_vertex(source)
            for node_id in self.graph.nodes:
                if node_id != source and self.costs[node_id] != self.rhs[node_id]:
                    self._enqueue(node_id)
        else:
            self._update_vertex(source)

    def move_start(self, start_id):
        self.graph.get_node(start_id)
        self.km += self._distance(self.last_start, start_id)
        self.last_start = start_id
        self.start = start_id

    def plan(self):
        return self._compute()

    def replan(self):
        return self._compute()

    def _compute(self):
        expanded = edge_evaluations = 0
        explored = set()
        while True:
            top = self._top()
            start_key = self._key(self.start)
            if (top is None or (top >= start_key and
                                self.rhs[self.start] == self.costs[self.start])):
                break
            old_key, _, current = heapq.heappop(self.queue)
            if self.queued.get(current) != old_key:
                continue
            self.queued.pop(current, None)
            new_key = self._key(current)
            if old_key < new_key:
                self._enqueue(current)
            elif self.costs[current] > self.rhs[current]:
                self.costs[current] = self.rhs[current]
                explored.add(current)
                expanded += 1
                for predecessor in self._predecessors(current):
                    edge_evaluations += 1
                    self._update_vertex(predecessor)
            else:
                self.costs[current] = inf
                explored.add(current)
                expanded += 1
                self._update_vertex(current)
                for predecessor in self._predecessors(current):
                    edge_evaluations += 1
                    self._update_vertex(predecessor)
        path = _tight_path_to_goal(self.graph, self.costs, self.start, self.goal)
        success = bool(path)
        result = _result(success, path, self.costs[self.start] if success else inf,
                         expanded, len(self.discovered), edge_evaluations)
        result['repair_strategy'] = self.repair_strategy
        return result

    def _predecessors(self, node_id):
        return self.parents[node_id]


class _EdgeView:
    __slots__ = ('source_id', 'target_id', 'cost')

    def __init__(self, source_id, target_id, cost):
        self.source_id = source_id
        self.target_id = target_id
        self.cost = cost


def _validate_edge_update(graph, source, target, cost):
    graph.get_node(source)
    graph.get_node(target)
    if source == target:
        raise ValueError('Edge cannot connect a node to itself')
    if cost is not None and (not isfinite(cost) or cost < 0):
        raise ValueError('Edge cost must be finite and nonnegative')


def _incremental_scale(graph, heuristic_scale):
    upper_bound = safe_heuristic_scale(graph)
    scale = upper_bound if heuristic_scale is None else heuristic_scale
    if not isfinite(scale) or scale < 0:
        raise ValueError('Heuristic scale must be finite and nonnegative')
    if scale > upper_bound + max(1e-15, upper_bound * 1e-12):
        raise ValueError('Incremental heuristic scale must be admissible for the graph')
    return scale


def _heuristic_would_be_invalid(graph, source, target, cost, scale):
    if cost is None or not scale:
        return False
    a, b = graph.nodes[source], graph.nodes[target]
    if a.x is None or b.x is None:
        return False
    distance = dist((a.x, a.y, a.z), (b.x, b.y, b.z))
    lower_bound = scale * distance
    return cost + max(1e-15, lower_bound * 1e-12) < lower_bound


def _heuristic(node, goal, scale):
    if not scale:
        return 0.0
    return scale * dist((node.x, node.y, node.z), (goal.x, goal.y, goal.z))


def _reconstruct_forward(parents, node_id):
    path = [node_id]
    while node_id in parents:
        node_id = parents[node_id]
        path.append(node_id)
    path.reverse()
    return path


def _discard_stale(queue, costs):
    while queue and queue[0][0] > costs.get(queue[0][1], inf):
        heapq.heappop(queue)


def _tight_path_from_start(graph, costs, start, goal):
    if not isfinite(costs[goal]):
        return []
    queue = deque([start])
    parents = {start: None}
    while queue:
        source = queue.popleft()
        if source == goal:
            return _path_from_parent_map(parents, goal)
        for target, edge in graph.edges[source].items():
            if target not in parents and isclose(
                costs[source] + edge.cost, costs[target], rel_tol=1e-12, abs_tol=1e-12
            ):
                parents[target] = source
                queue.append(target)
    raise RuntimeError('LPA* cost labels do not contain a route to the goal')


def _tight_path_to_goal(graph, costs, start, goal):
    if not isfinite(costs[start]):
        return []
    queue = deque([start])
    parents = {start: None}
    while queue:
        source = queue.popleft()
        if source == goal:
            return _path_from_parent_map(parents, goal)
        for target, edge in graph.edges[source].items():
            if target not in parents and isclose(
                edge.cost + costs[target], costs[source],
                rel_tol=1e-12, abs_tol=1e-12
            ):
                parents[target] = source
                queue.append(target)
    raise RuntimeError('D* Lite cost labels do not contain a route to the goal')


def _path_from_parent_map(parents, goal):
    path = [goal]
    while parents[path[-1]] is not None:
        path.append(parents[path[-1]])
    path.reverse()
    return path
