"""Deterministic ready-graph fixtures for mixed WALK/FLY planner validation."""

from dataclasses import dataclass

from m4_global_planner.graph import Edge, Graph, MotionMode, Node


START_ID = 0
GOAL_ID = 1
INTERMEDIATE_STATES = 10
FAMILIES = ('walk', 'fly', 'mixed')

PROFILES = {
    'walk_favored': {
        'walk_walk': 0.5,
        'fly_fly': 0.6,
        'transition': 1.5,
        'family_toll': {'walk': 0.0, 'fly': 0.0, 'mixed': 0.0},
        'expected_family': 'walk',
    },
    'fly_favored': {
        'walk_walk': 1.2,
        'fly_fly': 0.1,
        'transition': 0.4,
        'family_toll': {'walk': 0.0, 'fly': 0.0, 'mixed': 0.0},
        'expected_family': 'fly',
    },
    'mixed_favored': {
        'walk_walk': 0.3,
        'fly_fly': 0.3,
        'transition': 0.1,
        'family_toll': {'walk': 1.0, 'fly': 1.0, 'mixed': 0.0},
        'expected_family': 'mixed',
    },
}


@dataclass
class MixedModeFixture:
    graph: Graph
    alternatives_per_family: int
    lane_edges: dict
    start: int = START_ID
    goal: int = GOAL_ID


def build_mixed_mode_fixture(alternatives_per_family=128):
    """Build disjoint complete route alternatives sharing start and goal."""
    if type(alternatives_per_family) is not int or alternatives_per_family < 1:
        raise ValueError('alternatives_per_family must be a positive integer')

    graph = Graph()
    graph.add_node(Node(START_ID, 0, 0, 0, MotionMode.WALK))
    graph.add_node(Node(GOAL_ID, INTERMEDIATE_STATES + 1, 0, 0, MotionMode.WALK))
    lane_edges = {family: [] for family in FAMILIES}
    next_node_id = 2

    for family in FAMILIES:
        for lane_index in range(alternatives_per_family):
            lane = []
            for step in range(1, INTERMEDIATE_STATES + 1):
                if family == 'walk' or (family == 'mixed' and step <= 5):
                    mode = MotionMode.WALK
                else:
                    mode = MotionMode.FLY
                node_id = next_node_id
                next_node_id += 1
                graph.add_node(Node(node_id, step, 0, 0, mode))
                lane.append(node_id)

            route = [START_ID, *lane, GOAL_ID]
            edges = []
            for source, target in zip(route, route[1:]):
                graph.add_edge(Edge(source, target, 1.0))
                edges.append((source, target))
            lane_edges[family].append(edges)

    return MixedModeFixture(
        graph=graph,
        alternatives_per_family=alternatives_per_family,
        lane_edges=lane_edges,
    )


def apply_cost_profile(fixture, profile_name):
    """Change supplied edge weights without changing graph states or connections."""
    profile = PROFILES[profile_name]
    graph = fixture.graph
    for family, lanes in fixture.lane_edges.items():
        for lane_index, edges in enumerate(lanes):
            for edge_index, (source, target) in enumerate(edges):
                source_mode = graph.nodes[source].mode
                target_mode = graph.nodes[target].mode
                if source_mode == target_mode == MotionMode.WALK:
                    cost = profile['walk_walk']
                elif source_mode == target_mode == MotionMode.FLY:
                    cost = profile['fly_fly']
                else:
                    cost = profile['transition']
                if edge_index == 0:
                    cost += profile['family_toll'][family] + lane_index * 1e-6
                graph.edges[source][target] = Edge(source, target, cost)


def selected_route_family(result):
    modes = [node.mode for node in result['nodes'][1:-1]]
    walk_count = modes.count(MotionMode.WALK)
    fly_count = modes.count(MotionMode.FLY)
    if walk_count == INTERMEDIATE_STATES:
        return 'walk'
    if fly_count == INTERMEDIATE_STATES:
        return 'fly'
    if walk_count and fly_count:
        return 'mixed'
    return 'invalid'
