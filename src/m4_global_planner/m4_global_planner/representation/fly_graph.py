"""Two geometric FLY topologies with shared attachment, costs and route audit."""

from itertools import product
from math import dist, floor, isclose, isfinite
from random import Random
from time import perf_counter

from m4_global_planner.astar import astar
from m4_global_planner.graph import Edge, Graph, MotionMode, Node
from m4_global_planner.representation.fly_validator import position


class BudgetExceeded(RuntimeError):
    pass


def positive(settings, key, integer=False):
    value = settings[key]
    if (integer and type(value) is not int) or not isfinite(value) or value <= 0:
        raise ValueError(f'{key} must be finite and positive')
    return value


def usable_bounds(planning_map, validator):
    bounds = planning_map.get_3d_bounds()
    lower = [bounds[0], bounds[2], max(bounds[4], validator.floor(planning_map))]
    upper = [bounds[1], bounds[3], bounds[5]]
    lower = tuple(max(a - b, validator.altitude_min) if i == 2 else a - b
                  for i, (a, b) in enumerate(zip(lower, validator.body.lower)))
    upper = tuple(min(a - b, validator.altitude_max) if i == 2 else a - b
                  for i, (a, b) in enumerate(zip(upper, validator.body.upper)))
    if any(a >= b for a, b in zip(lower, upper)):
        raise ValueError('No positive-volume body-safe sampling bounds')
    return lower, upper


def lattice_points(planning_map, validator, settings, limit):
    coarse = positive(settings, 'coarse_resolution')
    fine = positive(settings, 'fine_resolution')
    if fine > coarse:
        raise ValueError('Fine resolution cannot exceed coarse resolution')
    lower, upper = usable_bounds(planning_map, validator)
    obstacles = []
    for obstacle in planning_map.get_occupied_volumes():
        lo = tuple(
            a - b - validator.body.tolerance
            for a, b in zip(obstacle.lower, validator.body.upper)
        )
        hi = tuple(
            a - b + validator.body.tolerance
            for a, b in zip(obstacle.upper, validator.body.lower)
        )
        obstacles.append((lo, hi))
    visited = 0

    def cells(lo, hi):
        nonlocal visited
        visited += 1
        if visited > limit:
            raise BudgetExceeded('Lattice cell budget exceeded')
        intersecting = [(a, b) for a, b in obstacles
                        if all(x <= v and y >= u for x, y, u, v in zip(lo, hi, a, b))]
        if any(all(a <= x and y <= b for a, b, x, y in zip(obslo, obshi, lo, hi))
               for obslo, obshi in intersecting):
            return
        target = fine if intersecting else coarse
        axes = []
        for a, b in zip(lo, hi):
            mid = (a + b) / 2
            axes.append(((a, mid), (mid, b)) if b - a > target else ((a, b),))
        if any(len(axis) > 1 for axis in axes):
            for child in product(*axes):
                yield from cells(tuple(a for a, b in child), tuple(b for a, b in child))
        else:
            yield tuple((a + b) / 2 for a, b in zip(lo, hi))

    yield from cells(lower, upper)


def roadmap_points(planning_map, validator, settings, seed):
    count = positive(settings, 'samples', True)
    attempts = positive(settings, 'max_sample_attempts', True)
    if count > attempts:
        raise ValueError('Sample attempt budget is smaller than sample count')
    lower, upper = usable_bounds(planning_map, validator)
    random = Random(seed)
    accepted = 0
    for _ in range(attempts):
        xyz = tuple(random.uniform(a, b) for a, b in zip(lower, upper))
        if validator.is_valid(Node(0, *xyz, MotionMode.FLY), planning_map):
            yield xyz
            accepted += 1
            if accepted == count:
                return
    raise BudgetExceeded('Roadmap sample attempt budget exhausted')


def connect(graph, planning_map, validator, settings):
    radius = positive(settings, 'connection_radius')
    count = positive(settings, 'max_neighbors', True)
    budget = positive(settings, 'max_connection_attempts', True)
    edge_budget = positive(settings, 'max_edges', True)
    rate = positive(settings, 'distance_cost_per_meter')
    if radius > validator.maximum:
        raise ValueError('Connection radius exceeds validated motion limit')
    bins = {}

    def key(node):
        return tuple(floor(value / radius) for value in position(node))

    for node in graph.nodes.values():
        bins.setdefault(key(node), []).append(node.node_id)
    attempted = set()
    sum_edges = 0
    for node in graph.nodes.values():
        cell = key(node)
        near = []
        for offset in product((-1, 0, 1), repeat=3):
            for other_id in bins.get(tuple(a + b for a, b in zip(cell, offset)), ()):
                if other_id == node.node_id:
                    continue
                length = dist(position(node), position(graph.nodes[other_id]))
                if validator.minimum < length <= radius:
                    near.append((length, other_id))
        # Union of local nearest-neighbor proposals; collision is checked once.
        for length, other_id in sorted(near)[:count]:
            pair = tuple(sorted((node.node_id, other_id)))
            if pair in attempted:
                continue
            if len(attempted) >= budget:
                raise BudgetExceeded('Connection attempt budget exceeded')
            attempted.add(pair)
            other = graph.nodes[other_id]
            if validator.is_edge_valid(node, other, planning_map):
                if sum_edges + 2 > edge_budget:
                    raise BudgetExceeded('Edge budget exceeded')
                graph.add_edge(Edge(node.node_id, other_id, length * rate))
                graph.add_edge(Edge(other_id, node.node_id, length * rate))
                sum_edges += 2
    return len(attempted)


def build_graph(approach, planning_map, builder, anchors, settings, tuning, seed):
    graph = Graph()
    mapping = builder.insert(anchors, graph, planning_map)
    limit = positive(settings, 'max_nodes', True)
    if len(graph.nodes) > limit:
        raise BudgetExceeded('Mandatory anchors exceed node budget')
    if approach == 'adaptive_lattice':
        points = lattice_points(
            planning_map, builder.fly, tuning, positive(settings, 'max_lattice_cells', True)
        )
    elif approach == 'sparse_roadmap':
        points = roadmap_points(planning_map, builder.fly, tuning, seed)
    else:
        raise ValueError(f'Unknown FLY representation: {approach}')
    existing = {position(node) for node in graph.nodes.values()}
    for xyz in points:
        if xyz in existing:
            continue
        node = Node(len(graph.nodes), *xyz, MotionMode.FLY)
        if builder.fly.is_valid(node, planning_map):
            if len(graph.nodes) >= limit:
                raise BudgetExceeded('Node budget exceeded')
            graph.add_node(node)
            existing.add(xyz)
    attempts = connect(graph, planning_map, builder.fly, settings)
    return graph, mapping, attempts


def validate_route(graph, route, endpoints, validator, planning_map, rate, cost):
    if not route or route[0] != endpoints[0] or route[-1] != endpoints[-1]:
        raise ValueError('Route does not preserve exact endpoints')
    cursor = 0
    for endpoint in endpoints:
        while cursor < len(route) and route[cursor] != endpoint:
            cursor += 1
        if cursor == len(route):
            raise ValueError('Route omits an ordered mandatory waypoint')
    total = 0.0
    for node_id in route:
        if not validator.is_valid(graph.nodes[node_id], planning_map):
            raise ValueError('Invalid route state')
    for source, target in zip(route, route[1:]):
        start, end = graph.nodes[source], graph.nodes[target]
        if not validator.is_edge_valid(start, end, planning_map):
            raise ValueError('Invalid route sweep')
        expected = dist(position(start), position(end)) * rate
        edge = graph.edges[source][target]
        if not isclose(edge.cost, expected, rel_tol=1e-10, abs_tol=1e-10):
            raise ValueError('Invalid edge cost')
        total += expected
    if not isclose(total, cost, rel_tol=1e-10, abs_tol=1e-10):
        raise ValueError('Invalid route cost')


def plan(approach, planning_map, builder, anchors, settings, tuning, seed, endpoint_order):
    start = perf_counter()
    graph, mapping, attempts = build_graph(
        approach, planning_map, builder, anchors, settings, tuning, seed
    )
    constructed = perf_counter()
    endpoints = dict(anchors.endpoints)
    ids = [mapping[endpoints[name].node_id] for name in endpoint_order]
    route, cost, expanded = [ids[0]], 0.0, 0
    success = True
    for source, target in zip(ids, ids[1:]):
        result = astar(graph, source, target, heuristic_scale=settings['distance_cost_per_meter'])
        expanded += result['nodes_expanded']
        if not result['success']:
            success, route, cost = False, [], None
            break
        route.extend(result['path'][1:])
        cost += result['total_cost']
    planned = perf_counter()
    if success:
        validate_route(
            graph, route, ids, builder.fly, planning_map,
            settings['distance_cost_per_meter'], cost,
        )
    finished = perf_counter()
    return {
        'status': 'success' if success else 'no_path',
        'success': success,
        'cost': cost,
        'construction_ms': (constructed - start) * 1000,
        'planning_ms': (planned - constructed) * 1000,
        'audit_ms': (finished - planned) * 1000,
        'total_ms': (finished - start) * 1000,
        'nodes': len(graph.nodes),
        'edges': sum(len(edges) for edges in graph.edges.values()),
        'connection_attempts': attempts,
        'expanded': expanded,
        'mandatory_nodes': len(anchors.fly_nodes),
        'route': [position(graph.nodes[node_id]) for node_id in route],
    }
