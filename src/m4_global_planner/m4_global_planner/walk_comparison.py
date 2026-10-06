"""WALK representations sharing executable primitives and endpoint checks."""

from collections import defaultdict
import heapq
from itertools import count
from math import atan2, cos, hypot, isfinite, pi, sin
import random
import time

from m4_global_planner.graph import Edge, Graph, MotionMode, Node
from m4_global_planner.representation.state_lattice import StateLatticeGenerator
from m4_global_planner.representation.states import PoseState
from m4_global_planner.representation.walk_edges import (
    WalkMotionEdgeGenerator, WalkMotionValidator,
)
from m4_global_planner.representation.walk_graph import (
    ForwardWalkGraphGenerator, WalkMotionNeighbors,
)
from m4_global_planner.representation.walk_validator import WalkStateValidator


APPROACHES = ('lattice', 'grid', 'hybrid', 'roadmap')


def pose(record):
    return PoseState(*record['position'], record['heading_deg'] * pi / 180)


def angle(value):
    return atan2(sin(value), cos(value))


class Motions:
    def __init__(self, parameters, planning_map):
        self.map = planning_map
        self.validator = WalkStateValidator(parameters)
        self.edges = WalkMotionEdgeGenerator(parameters, WalkMotionValidator(parameters,
                                                                             self.validator))
        self.calls = 0

    def edge(self, start, end):
        self.calls += 1
        return self.edges.generate_edge(start, end, self.map)

    def turn(self, start, heading):
        delta = angle(heading - start.heading)
        if abs(delta) <= self.edges.validator.straight.heading_tolerance:
            return [start], 0.0
        pieces = []
        current, cost = start, 0.0
        limit = self.edges.validator.max_pivot_angle
        while abs(delta) > self.edges.validator.straight.heading_tolerance:
            step = max(-limit, min(limit, delta))
            target = heading if abs(delta) <= limit else current.heading + step
            end = PoseState(start.x, start.y, start.z, target)
            edge = self.edge(current, end)
            if edge is None:
                return None
            pieces.append(end)
            cost += edge.cost
            current = end
            delta = angle(heading - current.heading)
        return [start] + pieces, cost

    def connect(self, start, end):
        if start == end:
            return [start], 0.0
        direct = self.edge(start, end)
        if direct is not None:
            return [start, end], direct.cost
        if (start.x, start.y) == (end.x, end.y):
            return self.turn(start, end.heading)
        heading = atan2(end.y - start.y, end.x - start.x)
        options = [heading]
        if self.edges.validator.reverse_allowed:
            options.append(angle(heading + pi))
        best = None
        for heading in options:
            before = self.turn(start, heading)
            if before is None:
                continue
            arrival = PoseState(end.x, end.y, end.z, before[0][-1].heading)
            # Keep the requested heading when the difference is roundoff.
            if (abs(angle(end.heading - arrival.heading))
                    <= self.edges.validator.straight.heading_tolerance):
                arrival = end
            travel = self.edge(before[0][-1], arrival)
            after = self.turn(arrival, end.heading)
            if travel is None or after is None:
                continue
            route = before[0] + [arrival] + after[0][1:]
            cost = before[1] + travel.cost + after[1]
            if best is None or cost < best[1]:
                best = route, cost
        return best


def add_point(graph, xyz, yaw=None):
    node_id = len(graph.nodes)
    graph.add_node(Node(node_id, *xyz, MotionMode.WALK, yaw))
    return node_id


def buckets(points, width):
    index = defaultdict(list)
    for node_id, xyz in points.items():
        index[int(xyz[0] // width), int(xyz[1] // width)].append(node_id)
    return index


def nearby(xyz, points, index, radius):
    ix, iy = int(xyz[0] // radius), int(xyz[1] // radius)
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            for node_id in index.get((ix + dx, iy + dy), ()):
                target = points[node_id]
                distance = hypot(target[0] - xyz[0], target[1] - xyz[1])
                if 0 < distance <= radius:
                    yield distance, node_id


def spatial_graph(approach, parameters, settings, motions, start, goal, seed):
    graph = Graph()
    points, ids = {}, {}
    line_nodes = set()
    xmin, xmax, ymin, ymax = motions.map.get_bounds()
    resolution = parameters['spatial_resolution']
    headings = StateLatticeGenerator(resolution,
                                     parameters['heading_resolution_deg']).generate_headings()

    def insert(x, y):
        z = motions.map.get_ground_height(x, y)
        if z is None or not any(motions.validator.is_valid(PoseState(x, y, z, h),
                                                           motions.map) for h in headings):
            return None
        xyz = x, y, z
        if xyz not in ids:
            node_id = add_point(graph, xyz)
            points[node_id], ids[xyz] = xyz, node_id
        return ids[xyz]

    if approach == 'grid':
        for ix in range(int((xmax - xmin) / resolution) + 1):
            for iy in range(int((ymax - ymin) / resolution) + 1):
                insert(xmin + ix * resolution, ymin + iy * resolution)
    else:
        rng = random.Random(seed)
        line_trials = int(settings['roadmap_samples'] * settings['roadmap_goal_line_fraction'])
        for trial in range(settings['roadmap_samples'] * settings['roadmap_trial_factor']):
            if trial < line_trials:
                fraction = rng.random()
                node_id = insert(start.x + fraction * (goal.x - start.x),
                                 start.y + fraction * (goal.y - start.y))
                if node_id is not None:
                    line_nodes.add(node_id)
            else:
                insert(rng.uniform(xmin, xmax), rng.uniform(ymin, ymax))
            if len(points) >= settings['roadmap_samples']:
                break
    base_count = len(points)
    endpoints = [insert(start.x, start.y), insert(goal.x, goal.y)]
    line_links = defaultdict(list)
    if approach == 'roadmap' and settings['roadmap_line_chain']:
        line_nodes.update(endpoints)
        ordered = sorted(line_nodes, key=lambda n: (
            (points[n][0] - start.x) * (goal.x - start.x)
            + (points[n][1] - start.y) * (goal.y - start.y)
        ))
        for a, b in zip(ordered, ordered[1:]):
            if hypot(points[a][0] - points[b][0],
                     points[a][1] - points[b][1]) <= settings['roadmap_radius']:
                line_links[a].append(b)
                line_links[b].append(a)
    radius = settings['roadmap_radius'] if approach == 'roadmap' else settings['endpoint_radius']
    index = buckets(points, radius)
    cells = {(round((xyz[0] - xmin) / resolution), round((xyz[1] - ymin) / resolution)): n
             for n, xyz in points.items() if n < base_count}

    for source, xyz in points.items():
        if approach == 'roadmap':
            targets = [n for _, n in sorted(nearby(xyz, points, index,
                                                   radius))[:settings['roadmap_neighbors']]]
            targets += line_links[source]
        elif source in endpoints or source >= base_count:
            targets = [n for _, n in nearby(xyz, points, index, radius)]
        else:
            ix, iy = round((xyz[0] - xmin) / resolution), round((xyz[1] - ymin) / resolution)
            targets = []
            for step in settings['grid_neighbor_steps']:
                for dx, dy in ((step, 0), (-step, 0), (0, step), (0, -step),
                               (step, step), (step, -step), (-step, step), (-step, -step)):
                    target = cells.get((ix + dx, iy + dy))
                    if target is not None:
                        targets.append(target)
            targets += [n for n in endpoints if
                        0 < hypot(points[n][0] - xyz[0], points[n][1] - xyz[1]) <= radius]
        for target in sorted(set(targets)):
            end = points[target]
            heading = atan2(end[1] - xyz[1], end[0] - xyz[0])
            edge = motions.edge(PoseState(*xyz, heading), PoseState(*end, heading))
            if edge is not None:
                graph.add_edge(Edge(source, target, edge.cost))
    return graph, points, ids, endpoints


def explicit_lattice(parameters, settings, motions, start, goal, attachments=()):
    generator = ForwardWalkGraphGenerator(
        WalkMotionNeighbors(parameters),
        StateLatticeGenerator(parameters['spatial_resolution'],
                              parameters['heading_resolution_deg']),
        motions.validator, motions.edges,
    )
    graph = generator.generate_graph(motions.map)
    motions.calls += generator.candidate_attempts
    poses = {n.node_id: PoseState(n.x, n.y, n.z, n.yaw) for n in graph.nodes.values()}
    ids = {state: node_id for node_id, state in poses.items()}
    paths = {}
    extra = sorted(set(attachments), key=lambda state: (
        state.x, state.y, state.z, state.heading,
    ))
    # Lattice anchors already have complete primitive successors. Only new
    # exact poses need local, bidirectional connector proposals.
    missing = [state for state in extra if state not in ids]
    for state in missing:
        node_id = add_point(graph, (state.x, state.y, state.z), state.heading)
        poses[node_id], ids[state] = state, node_id
    proposals = [(start, True), (goal, False)] if start is not None else []
    proposals += [(state, direction) for state in missing for direction in (True, False)]
    for state, outgoing in proposals:
        if state not in ids:
            node_id = add_point(graph, (state.x, state.y, state.z), state.heading)
            poses[node_id], ids[state] = state, node_id
        node_id = ids[state]
        for target, other in list(poses.items()):
            if target == node_id or hypot(other.x - state.x,
                                          other.y - state.y) > settings['endpoint_radius']:
                continue
            source_id, target_id = (node_id, target) if outgoing else (target, node_id)
            a, b = poses[source_id], poses[target_id]
            connection = motions.connect(a, b)
            if connection:
                previous = graph.edges[source_id].get(target_id)
                if previous is None:
                    graph.add_edge(Edge(source_id, target_id, connection[1]))
                    paths[source_id, target_id] = connection[0]
                elif connection[1] < previous.cost:
                    graph.edges[source_id][target_id] = Edge(source_id, target_id, connection[1])
                    paths[source_id, target_id] = connection[0]
    return graph, poses, ids, paths


def plan(approach, parameters, settings, planning_map, start, goal, seed=0,
         capture_search_graph=False):
    began = time.perf_counter()
    for field in ('endpoint_radius', 'roadmap_radius', 'hybrid_xy_bin',
                  'hybrid_heading_bin_deg', 'cost_tolerance'):
        if not isfinite(settings[field]) or settings[field] <= 0:
            raise ValueError(f'{field} must be finite and positive')
    for field in ('roadmap_samples', 'roadmap_neighbors', 'roadmap_trial_factor',
                  'max_expansions'):
        if type(settings[field]) is not int or settings[field] <= 0:
            raise ValueError(f'{field} must be a positive integer')
    if not 0 <= settings['roadmap_goal_line_fraction'] <= 1:
        raise ValueError('Roadmap line fraction must be in [0, 1]')
    maximum = parameters['straight_walk']['max_distance']
    if any(settings[field] > maximum for field in ('endpoint_radius', 'roadmap_radius')):
        raise ValueError('Connection radii must not exceed the motion distance limit')
    if any(
        not isfinite(length) or not 0 < length <= maximum
        for length in settings['hybrid_step_lengths']
    ):
        raise ValueError('Hybrid lengths must be positive and within the motion limit')
    motions = Motions(parameters, planning_map)
    result = {'status': 'no_path', 'success': False, 'route': [], 'cost': None,
              'nodes': 0, 'edges': 0, 'expanded': 0, 'discovered': 0, 'rejected_route': False}
    for state, status in ((start, 'invalid_start'), (goal, 'invalid_goal')):
        if not motions.validator.is_valid(state, planning_map):
            result['status'] = status
            result['total_ms'] = (time.perf_counter() - began) * 1000
            result.update(construction_ms=0, planning_ms=0, audit_ms=0)
            return result
    if not planning_map.supports_flat_walk_edges():
        result.update(status='unsupported_nonflat', total_ms=(time.perf_counter() - began) * 1000,
                      construction_ms=0, planning_ms=0, audit_ms=0)
        return result
    rate = motions.edges.cost.rate
    graph = None
    if approach == 'lattice':
        graph, poses, ids, paths = explicit_lattice(parameters, settings, motions, start, goal)

        def successors(state):
            source = ids[state]
            for edge in graph.get_neighbors(source):
                target = poses[edge.target_id]
                yield target, paths.get((source, edge.target_id), [state, target]), edge.cost
    elif approach in ('grid', 'roadmap'):
        graph, points, ids, endpoints = spatial_graph(approach, parameters, settings, motions,
                                                      start, goal, seed)

        def successors(state):
            source = ids[(state.x, state.y, state.z)]
            if source == endpoints[1] and state != goal:
                connection = motions.connect(state, goal)
                if connection:
                    yield goal, *connection
            for edge in graph.get_neighbors(source):
                xyz = points[edge.target_id]
                heading = atan2(xyz[1] - state.y, xyz[0] - state.x)
                headings = [heading]
                if parameters['reverse_allowed']:
                    headings.append(angle(heading + pi))
                for heading in headings:
                    target = goal if edge.target_id == endpoints[1] else PoseState(*xyz, heading)
                    connection = motions.connect(state, target)
                    if connection:
                        yield target, *connection
    elif approach == 'hybrid':
        step_angle = parameters['heading_resolution_deg'] * pi / 180

        def successors(state):
            if state != goal and hypot(goal.x - state.x,
                                       goal.y - state.y) <= settings['endpoint_radius']:
                connection = motions.connect(state, goal)
                if connection:
                    yield goal, *connection
            for length in settings['hybrid_step_lengths']:
                for sign in (1, -1) if parameters['reverse_allowed'] else (1,):
                    x = state.x + sign * length * cos(state.heading)
                    y = state.y + sign * length * sin(state.heading)
                    z = planning_map.get_ground_height(x, y)
                    if z is None:
                        continue
                    target = PoseState(x, y, z, state.heading)
                    edge = motions.edge(state, target)
                    if edge:
                        yield target, [state, target], edge.cost
            if parameters['pivot_allowed']:
                for step in parameters['pivot_walk']['neighbor_heading_steps']:
                    for sign in (-1, 1):
                        target = PoseState(state.x, state.y, state.z,
                                           angle(state.heading + sign * step * step_angle))
                        edge = motions.edge(state, target)
                        if edge:
                            yield target, [state, target], edge.cost
    else:
        raise ValueError('Unknown WALK approach')

    built = time.perf_counter()
    result['construction_ms'] = (built - began) * 1000
    if graph:
        result['nodes'], result['edges'] = len(graph.nodes), sum(map(len, graph.edges.values()))

    def key(state):
        if state == goal:
            return 'exact_goal'
        if approach != 'hybrid':
            return state
        return (round(state.x / settings['hybrid_xy_bin']),
                round(state.y / settings['hybrid_xy_bin']),
                round((state.heading % (2 * pi)) / (settings['hybrid_heading_bin_deg'] * pi / 180))
                % round(360 / settings['hybrid_heading_bin_deg']))

    serial = count()
    start_key = key(start)
    best = {start_key: 0.0}
    queue = [(rate * hypot(goal.x - start.x, goal.y - start.y), next(serial), 0.0, start, ())]
    work_edges = 0
    search_ids, search_edges, search_segments = {}, {}, {}

    def search_id(state):
        if state not in search_ids:
            search_ids[state] = len(search_ids)
        return search_ids[state]

    if capture_search_graph:
        search_id(start)
    while queue:
        _, _, cost, current, chain = heapq.heappop(queue)
        if cost > best[key(current)]:
            continue
        result['expanded'] += 1
        if current == goal:
            segments = []
            while chain:
                segment, chain = chain
                segments.append(segment)
            route = [start]
            for segment in reversed(segments):
                route += segment[1:]
            result.update(success=True, status='success', route=route, cost=cost)
            if not capture_search_graph:
                break
            # Audit mode finishes the finite graph after saving A*'s goal path.
            continue
        if result['expanded'] >= settings['max_expansions']:
            result['status'] = 'expansion_limit'
            break
        source_id = search_id(current) if capture_search_graph else None
        for target, segment, extra in successors(current):
            work_edges += 1
            if capture_search_graph:
                target_id = search_id(target)
                edge_key = source_id, target_id
                if edge_key not in search_edges or extra < search_edges[edge_key]:
                    search_edges[edge_key] = extra
                    search_segments[edge_key] = tuple(segment)
            new = cost + extra
            target_key = key(target)
            if new >= best.get(target_key, float('inf')):
                continue
            best[target_key] = new
            h = rate * hypot(goal.x - target.x, goal.y - target.y)
            heapq.heappush(queue, (new + h, next(serial), new, target, (segment, chain)))
    planned = time.perf_counter()
    result['planning_ms'] = (planned - built) * 1000
    result['discovered'], result['work_edges'] = len(best), work_edges
    if approach == 'hybrid':
        result['nodes'], result['edges'] = len(best), work_edges
    if result['success']:
        route = result['route']
        audit_cost = 0.0
        valid = route[0] == start and route[-1] == goal
        for a, b in zip(route, route[1:]):
            edge = motions.edge(a, b)
            if edge is None:
                valid = False
                break
            audit_cost += edge.cost
        if not valid or abs(audit_cost - result['cost']) > settings['cost_tolerance']:
            result.update(success=False, status='invalid_route', rejected_route=True, cost=None)
    ended = time.perf_counter()
    result.update(audit_ms=(ended - planned) * 1000, total_ms=(ended - began) * 1000,
                  validation_calls=motions.calls)
    if capture_search_graph:
        captured = Graph()
        states_by_id = {node_id: state for state, node_id in search_ids.items()}
        for node_id, state in states_by_id.items():
            captured.add_node(Node(node_id, state.x, state.y, state.z, MotionMode.WALK,
                                   state.heading))
        for (source_id, target_id), edge_cost in search_edges.items():
            captured.add_edge(Edge(source_id, target_id, edge_cost))
        result['_search_graph'] = captured
        result['_search_states'] = states_by_id
        result['_search_segments'] = search_segments
        result['_search_start_id'] = search_ids[start]
        result['_search_goal_id'] = search_ids.get(goal)
        result['_search_graph_complete'] = result['status'] != 'expansion_limit'
    return result
