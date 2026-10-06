from dataclasses import dataclass
from math import ceil, floor, isfinite

from m4_global_planner.graph import MotionMode, Node
from m4_global_planner.representation.fly_validator import position, require_policy
from m4_global_planner.representation.state_lattice import StateLatticeGenerator
from m4_global_planner.representation.states import PoseState


@dataclass(frozen=True, slots=True)
class TransitionAnchor:
    walk_pose: PoseState
    fly_id: int
    directions: tuple[str, ...]


@dataclass(frozen=True, slots=True)
class MandatoryAnchors:
    endpoints: tuple
    fly_nodes: tuple[Node, ...]
    transitions: tuple[TransitionAnchor, ...]
    sampled_walk_states: int
    rejected_transition_states: int


class MandatoryAnchorBuilder:
    """Prepare one shared anchor set before either candidate samples space."""

    def __init__(self, parameters, walk_validator, fly_validator, transition_validator):
        settings = parameters.get('mandatory_anchors', {})
        required = (
            'endpoint_policy', 'transition_sampling_policy', 'transition_spatial_resolution',
            'transition_heading_resolution_deg', 'max_sampled_walk_states',
        )
        missing = [key for key in required if key not in settings]
        if missing:
            raise ValueError(f"Missing anchor settings: {', '.join(missing)}")
        require_policy(settings, 'endpoint_policy', 'exact')
        require_policy(settings, 'transition_sampling_policy', 'ground_lattice')
        resolution = settings['transition_spatial_resolution']
        heading = settings['transition_heading_resolution_deg']
        if (not isfinite(resolution) or resolution <= 0 or not isfinite(heading)
                or not 0 < heading <= 360):
            raise ValueError('Anchor resolutions must be finite and positive')
        self.limit = settings['max_sampled_walk_states']
        if type(self.limit) is not int or self.limit <= 0:
            raise ValueError('Anchor state limit must be a positive integer')
        self.generator = StateLatticeGenerator(resolution, heading)
        self.walk, self.fly, self.transition = walk_validator, fly_validator, transition_validator

    def validate_endpoint(self, name, node, planning_map):
        if node.mode == MotionMode.FLY:
            valid = self.fly.is_valid(node, planning_map)
            state = None
        elif node.mode == MotionMode.WALK:
            if node.yaw is None:
                raise ValueError(f'WALK endpoint {name} requires a heading')
            state = PoseState(*position(node), node.yaw)
            valid = self.walk.is_valid(state, planning_map)
        else:
            raise NotImplementedError(f'Unsupported endpoint mode: {node.mode}')
        if not valid:
            raise ValueError(f'Invalid endpoint: {name}')
        return state

    def build(self, planning_map, endpoints=None):
        endpoints = endpoints or {}
        self.fly.floor(planning_map)
        endpoint_walk = set()
        fly_positions = set()
        for name, node in sorted(endpoints.items()):
            state = self.validate_endpoint(name, node, planning_map)
            if state is None:
                fly_positions.add(position(node))
            else:
                endpoint_walk.add(state)

        xmin, xmax, ymin, ymax = planning_map.get_bounds()
        if not all(isfinite(value) for value in (xmin, xmax, ymin,
                                                 ymax)) or xmin > xmax or ymin > ymax:
            raise ValueError('Anchor map bounds must be finite and ordered')
        resolution = self.generator.spatial_resolution
        possible = (
            (floor((xmax - xmin) / resolution) + 1)
            * (floor((ymax - ymin) / resolution) + 1)
        )
        possible *= ceil(360 / self.generator.heading_resolution_deg)
        if possible > self.limit:
            raise NotImplementedError('Configured anchor sampling budget is insufficient')
        sampled = self.generator.generate_states(planning_map, self.walk)
        states = set(sampled) | endpoint_walk
        accepted, rejected = [], 0
        for state in sorted(states, key=lambda s: (*position(s), s.heading)):
            node = Node(0, *position(state), MotionMode.FLY)
            directions = tuple(direction for direction in self.transition.directions
                               if self.transition.is_valid(state, node, planning_map, direction))
            if directions:
                fly_positions.add(position(state))
                accepted.append((state, directions))
            else:
                rejected += 1
        fly_nodes = tuple(
            Node(index, *xyz, MotionMode.FLY) for index, xyz in enumerate(sorted(fly_positions))
        )
        ids = {position(node): node.node_id for node in fly_nodes}
        transitions = tuple(
            TransitionAnchor(state, ids[position(state)], directions)
            for state, directions in accepted
        )
        normalized_endpoints = tuple(
            (name, fly_nodes[ids[position(node)]] if node.mode == MotionMode.FLY else node)
            for name, node in sorted(endpoints.items())
        )
        return MandatoryAnchors(
            normalized_endpoints, fly_nodes, transitions, len(sampled), rejected
        )

    def insert(self, anchors, graph, planning_map):
        """Insert exact FLY nodes; return anchor IDs mapped to candidate IDs."""
        # Validate the snapshot before mutating either candidate graph.
        for name, node in anchors.endpoints:
            self.validate_endpoint(name, node, planning_map)
        for node in anchors.fly_nodes:
            if not self.fly.is_valid(node, planning_map):
                raise ValueError('Mandatory anchor is no longer valid')
        for anchor in anchors.transitions:
            node = anchors.fly_nodes[anchor.fly_id]
            if any(not self.transition.is_valid(anchor.walk_pose, node, planning_map, direction)
                   for direction in anchor.directions):
                raise ValueError('Transition anchor is no longer valid')
        existing = {}
        for node in graph.nodes.values():
            if node.mode != MotionMode.FLY:
                continue
            key = (*position(node), node.yaw)
            if key in existing:
                raise ValueError('Candidate graph has duplicate FLY poses')
            existing[key] = node.node_id
        next_id = max(graph.nodes, default=-1) + 1
        result = {}
        for node in anchors.fly_nodes:
            key = (*position(node), node.yaw)
            if key not in existing:
                graph.add_node(Node(next_id, *position(node), MotionMode.FLY, node.yaw))
                existing[key] = next_id
                next_id += 1
            result[node.node_id] = existing[key]
        return result
