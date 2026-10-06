from math import cos, hypot, isfinite, sin

from m4_global_planner.graph import Edge, Graph, MotionMode, Node
from m4_global_planner.representation.state_lattice import StateLatticeGenerator


class ForwardWalkNeighbors:
    """Indexed lattice lookup using configured grid shells, never node pairs."""

    def __init__(self, parameters):
        settings = parameters['straight_walk']
        self.resolution = parameters['spatial_resolution']
        self.steps = settings['neighbor_grid_steps']
        self.index_tolerance = settings['grid_index_tolerance']
        self.max_distance = settings['max_distance']
        self.lateral_tolerance = settings['lateral_tolerance']
        if not isfinite(self.resolution) or self.resolution <= 0:
            raise ValueError('Spatial resolution must be finite and positive')
        if (
            not isfinite(self.index_tolerance)
            or not 0 < self.index_tolerance < self.resolution / 2
        ):
            raise ValueError('Grid tolerance must be positive and below half a cell')
        if not self.steps or any(
            type(step) is not int or step <= 0 for step in self.steps
        ) or len(set(self.steps)) != len(self.steps):
            raise ValueError('Neighbor grid steps must be unique positive integers')
        if not all(isfinite(value) and value > 0 for value in (
            self.max_distance, self.lateral_tolerance,
        )):
            raise ValueError('Neighbor distance limits must be finite and positive')

    def offsets(self, heading):
        c, s = cos(heading), sin(heading)
        for step in sorted(self.steps):
            if step * self.resolution > self.max_distance:
                continue
            for ix in range(-step, step + 1):
                for iy in range(-step, step + 1):
                    if max(abs(ix), abs(iy)) != step:
                        continue
                    dx, dy = ix * self.resolution, iy * self.resolution
                    if (
                        hypot(dx, dy) <= self.max_distance
                        and dx * c + dy * s > 0
                        and abs(-dx * s + dy * c) <= self.lateral_tolerance
                    ):
                        yield ix, iy

    def key(self, state, origin):
        indices = []
        for coordinate, base in zip((state.x, state.y), origin):
            index = round((coordinate - base) / self.resolution)
            if abs(coordinate - (base + index * self.resolution)) > self.index_tolerance:
                raise ValueError('State does not lie on the configured lattice')
            indices.append(index)
        # One ground height per x/y and exact generator headings are required.
        return *indices, state.heading

    def heading_targets(self, headings):
        return {heading: () for heading in headings}

    def candidate_pairs(self, states, origin):
        index = {}
        for state in states:
            key = self.key(state, origin)
            if key in index:
                raise ValueError('Duplicate lattice state key')
            index[key] = state
        headings = {heading for _, _, heading in index}
        offsets = {heading: tuple(self.offsets(heading)) for heading in headings}
        rotations = self.heading_targets(headings)
        for key, start in index.items():
            ix, iy, heading = key
            for dx, dy in offsets[heading]:
                end = index.get((ix + dx, iy + dy, heading))
                if end is not None:
                    yield start, end
            for target_heading in rotations[heading]:
                end = index.get((ix, iy, target_heading))
                if end is not None:
                    yield start, end


class WalkMotionNeighbors(ForwardWalkNeighbors):
    """Add indexed reverse translations and colocated heading neighbors."""

    def __init__(self, parameters):
        super().__init__(parameters)
        self.reverse_allowed = parameters['reverse_allowed']
        self.pivot_allowed = parameters['pivot_allowed']
        if any(type(value) is not bool for value in (
            self.reverse_allowed, self.pivot_allowed,
        )):
            raise ValueError('Motion permissions must be booleans')
        self.heading_steps = parameters['pivot_walk']['neighbor_heading_steps']
        if not self.heading_steps or any(type(step) is not int or step <= 0
                                         for step in self.heading_steps):
            raise ValueError('Pivot heading steps must be positive integers')
        if len(set(self.heading_steps)) != len(self.heading_steps):
            raise ValueError('Pivot heading steps must be unique')
        self.configured_headings = StateLatticeGenerator(
            parameters['spatial_resolution'], parameters['heading_resolution_deg'],
        ).generate_headings()
        if max(self.heading_steps) >= len(self.configured_headings):
            raise ValueError('Pivot heading steps must be below configured heading count')

    def offsets(self, heading):
        for dx, dy in super().offsets(heading):
            yield dx, dy
            if self.reverse_allowed:
                yield -dx, -dy

    def heading_targets(self, headings):
        ordered = self.configured_headings
        if not self.pivot_allowed:
            return super().heading_targets(headings)
        return {
            heading: tuple(sorted({
                ordered[(index + sign * step) % len(ordered)]
                for step in self.heading_steps for sign in (-1, 1)
            })) for index, heading in enumerate(ordered) if heading in headings
        }


class ForwardWalkGraphGenerator:
    """Adapt lattice poses and edges supplied by the injected WALK modules."""

    def __init__(self, neighbors, state_generator, state_validator, edge_generator):
        self.neighbors = neighbors
        self.state_generator = state_generator
        self.state_validator = state_validator
        self.edge_generator = edge_generator
        self.candidate_attempts = 0

    def generate_graph(self, planning_map):
        self.candidate_attempts = 0
        states = self.state_generator.generate_states(planning_map, self.state_validator)
        graph = Graph()
        state_ids = {}
        for node_id, state in enumerate(states):
            graph.add_node(Node(
                node_id, state.x, state.y, state.z, MotionMode.WALK, state.heading,
            ))
            state_ids[state] = node_id
        xmin, _, ymin, _ = planning_map.get_bounds()
        for start, end in self.neighbors.candidate_pairs(states, (xmin, ymin)):
            self.candidate_attempts += 1
            edge = self.edge_generator.generate_edge(start, end, planning_map)
            if edge is not None:
                graph.add_edge(Edge(state_ids[start], state_ids[end], edge.cost))
        return graph
