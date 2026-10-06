from dataclasses import dataclass
from enum import Enum
from math import isfinite


class MotionMode(str, Enum):
    WALK = 'walk'  # robot is walking
    FLY = 'fly'    # robot is flying


@dataclass(frozen=True, slots=True)
class Node:
    node_id: int              # unique node number
    x: float                  # x position in meters
    y: float                  # y position in meters
    z: float                  # z position in meters
    mode: MotionMode          # walk or fly
    yaw: float | None = None  # heading if we need it later

    def __post_init__(self):
        # make sure the position values are valid
        if not all(isfinite(value) for value in (self.x, self.y, self.z)):
            raise ValueError('Node position is not valid')

        # make sure yaw is valid if it is used
        if self.yaw is not None and not isfinite(self.yaw):
            raise ValueError('Node yaw is not valid')


@dataclass(frozen=True, slots=True)
class Edge:
    source_id: int  # node where movement starts
    target_id: int  # node where movement ends
    cost: float     # cost of moving through this edge

    def __post_init__(self):
        # edge cannot connect a node to itself
        if self.source_id == self.target_id:
            raise ValueError('Edge cannot connect a node to itself')

        # A* needs a valid non-negative edge cost
        if not isfinite(self.cost) or self.cost < 0:
            raise ValueError('Edge cost must be a valid non-negative number')


class Graph:
    def __init__(self):
        self.nodes = {}  # stores all nodes using their ids
        self.edges = {}  # stores outgoing edges from every node

    def add_node(self, node):
        # node ids must be unique
        if node.node_id in self.nodes:
            raise ValueError('Node already exists')

        self.nodes[node.node_id] = node
        self.edges[node.node_id] = {}

    def add_edge(self, edge):
        # source node must exist
        if edge.source_id not in self.nodes:
            raise ValueError('Source node does not exist')

        # target node must exist
        if edge.target_id not in self.nodes:
            raise ValueError('Target node does not exist')

        # do not add the same directed edge twice
        if edge.target_id in self.edges[edge.source_id]:
            raise ValueError('Edge already exists')

        self.edges[edge.source_id][edge.target_id] = edge

    def get_node(self, node_id):
        # make sure the node exists
        if node_id not in self.nodes:
            raise ValueError('Node does not exist')

        return self.nodes[node_id]

    def get_neighbors(self, node_id):
        # make sure the node exists
        if node_id not in self.nodes:
            raise ValueError('Node does not exist')

        # return all edges leaving this node
        return list(self.edges[node_id].values())
