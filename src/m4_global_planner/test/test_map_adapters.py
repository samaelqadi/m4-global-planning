"""Equivalent adapter geometry, height knowledge, costs and rebuilt joint routes."""

from copy import deepcopy

from m4_global_planner.dummy_map import DummyPlanningMap
from m4_global_planner.graph import MotionMode, Node
from m4_global_planner.height_grid_map import HeightGridMap
from m4_global_planner.representation.anchors import MandatoryAnchorBuilder
from m4_global_planner.representation.fly_validator import FlyStateValidator, TransitionValidator
from m4_global_planner.representation.joint_graph import JointPlanner
from m4_global_planner.representation.states import PoseState
from m4_global_planner.representation.walk_validator import WalkStateValidator
import pytest
from test_graph_optimality import dijkstra_path
from test_straight_walk_edges import DATA


def maps(occupied=None, cost=0):
    occupied = occupied or {}
    settings = deepcopy(DATA['height_grid_mock'])
    settings['unknown_policy'] = 'reject_map'
    settings['cells'] = [[{'ground_height': 0, 'occupied_intervals': occupied.get((x, y), []),
                           'cost': cost} for x in range(8)] for y in range(4)]
    boxes = [{'min': [x, y, a], 'max': [x + 1, y + 1, b]}
             for (x, y), intervals in occupied.items() for a, b in intervals]
    world = {'bounds': [0, 8, 0, 4, 0, 3], 'ground': {'type': 'flat', 'z': 0},
             'obstacles_3d': boxes, 'walk_cost_regions': (
                 [{'min': [0, 0], 'max': [8, 4], 'cost': cost}] if cost else [])}
    return DummyPlanningMap({'world': world}), HeightGridMap(settings)


def planner(planning_map):
    parameters, settings = deepcopy(DATA['test_parameters']), deepcopy(DATA['joint_planner'])
    parameters['walk_terrain_cost_mode'] = settings['walk_terrain_cost_mode']
    walk, fly = WalkStateValidator(parameters), FlyStateValidator(parameters)
    builder = MandatoryAnchorBuilder(parameters, walk, fly,
                                     TransitionValidator(parameters, walk, fly))
    endpoints = {'start': Node(0, 1, 2, 0, MotionMode.WALK, 0),
                 'goal': Node(1, 7, 2, 0, MotionMode.WALK, 0)}
    return JointPlanner(parameters, settings, planning_map, builder, endpoints)


def audit(current, result):
    current.validate_result(result)
    minimum, path = dijkstra_path(current.graph, current.endpoints['start'],
                                  current.endpoints['goal'])
    assert result['success']
    assert result['total_cost'] == pytest.approx(minimum)
    current.validate_route(path, current.endpoints['start'], current.endpoints['goal'], minimum)
    assert result['route_kind'] == 'geometric' and result['executable'] is False


@pytest.mark.parametrize('occupied,cost', [
    ({}, 0), ({(3, 1): [[0, 2]], (3, 2): [[0, 2]]}, 0), ({}, 2),
    ({(3, 1): [[1, 1.5]], (3, 2): [[1, 1.5]]}, 1),
])
def test_same_joint_graph_and_cost_on_equivalent_worlds(occupied, cost):
    box, grid = maps(occupied, cost)
    a, b = planner(box), planner(grid)
    assert a.graph.nodes == b.graph.nodes
    assert set(a.graph.edges) == set(b.graph.edges)
    for source, edges in a.graph.edges.items():
        assert set(edges) == set(b.graph.edges[source])
        for target, edge in edges.items():
            assert edge.cost == pytest.approx(b.graph.edges[source][target].cost)
    first, second = a.plan(), b.plan()
    audit(a, first)
    audit(b, second)
    assert first['total_cost'] == pytest.approx(second['total_cost'])


def test_ground_top_and_intervals_are_distinct_and_overhead_clearance_matches():
    box, grid = maps({(3, 1): [[1, 1.5]], (3, 2): [[1, 1.5]]})
    parameters = DATA['test_parameters']
    for planning_map in (box, grid):
        assert planning_map.get_ground_height(3.5, 1.5) == 0
        assert planning_map.get_obstacle_top_height(3.5, 1.5) == 1.5
        assert planning_map.get_occupied_height_intervals(3.5, 1.5) == ((1, 1.5),)
        walk, fly = WalkStateValidator(parameters), FlyStateValidator(parameters)
        assert walk.is_valid(PoseState(3.5, 1.5, 0, 0), planning_map)
        assert not TransitionValidator(parameters, walk, fly).is_valid(
            PoseState(3.5, 1.5, 0, 0), Node(0, 3.5, 1.5, 0, MotionMode.FLY), planning_map)
        for z, valid in ((.2, True), (.8, False), (1.6, True)):
            assert fly.is_valid(Node(0, 3.5, 1.5, z, MotionMode.FLY), planning_map) == valid


@pytest.mark.parametrize('cell', [
    {'ground_height': 0}, {'ground_height': 0, 'obstacle_top_height': 1.5},
    {'ground_height': 0, 'occupied_intervals': None},
    {'ground_height': None, 'occupied_intervals': []},
    {'ground_height': 0, 'occupied_intervals': [], 'unknown': True},
])
def test_missing_vertical_data_never_certifies_free_space(cell):
    _, grid = maps()
    updated = grid.apply_update({'set_cells': [{'index': [3, 1], 'cell': cell}]})
    assert updated.get_occupied_height_intervals(3.5, 1.5) is None
    with pytest.raises(NotImplementedError, match='Unknown'):
        planner(updated)
    with pytest.raises(NotImplementedError, match='Unknown'):
        WalkStateValidator(DATA['test_parameters']).is_valid(PoseState(1, 2, 0, 0), updated)


def test_unknown_box_and_grid_policies_match():
    box, grid = maps()
    box.world['unknown_regions'] = [{'min': [3, 1], 'max': [4, 2]}]
    grid = grid.apply_update({'set_cells': [{'index': [3, 1], 'cell': {
        'ground_height': 0, 'occupied_intervals': [], 'unknown': True,
    }}]})
    assert tuple(box.get_unknown_regions()) == grid.get_unknown_regions()
    for planning_map in (box, grid):
        with pytest.raises(NotImplementedError, match='Unknown'):
            planner(planning_map)


def test_adapter_native_updates_rebuild_equivalent_graphs_and_clear_costs():
    box, grid = maps(cost=1)
    a, b = planner(box), planner(grid)
    old_a, old_b = a.plan(), b.plan()
    first = a.replan({'add_obstacles': [{'min': [3, 1, 0], 'max': [4, 2, 2]}]})
    second = b.replan({'set_cells': [{'index': [3, 1], 'cell': {
        'ground_height': 0, 'occupied_intervals': [[0, 2]], 'cost': 1,
    }}]})
    audit(a, first)
    audit(b, second)
    assert first['total_cost'] == pytest.approx(second['total_cost'])
    for current, old in ((a, old_a), (b, old_b)):
        with pytest.raises(ValueError, match='Stale'):
            current.validate_result(old)
    first = a.replan({'set_walk_cost_regions': []})
    second = b.replan({'set_cells': [
        {'index': [x, y], 'cell': {
            'ground_height': 0, 'cost': 0,
            'occupied_intervals': [[0, 2]] if (x, y) == (3, 1) else [],
        }}
        for y in range(4) for x in range(8)
    ]})
    audit(a, first)
    audit(b, second)
    assert first['total_cost'] == pytest.approx(second['total_cost'])


def test_cell_boundaries_do_not_double_charge_cost():
    _, grid = maps(cost=2)
    current = planner(grid)
    edge = current.motions.edge(PoseState(1, 2, 0, 0), PoseState(2, 2, 0, 0))
    assert edge.cost == pytest.approx(3)  # base rate 1 + extra density 2, not two rows


def test_grid_origin_resolution_and_outside_queries():
    _, grid = maps()
    settings = deepcopy(grid.settings)
    settings.update(origin=[-2, 3], resolution=.5, vertical_bounds=[-1, 4])
    updated = HeightGridMap(settings)
    assert updated.get_3d_bounds() == (-2, 2, 3, 5, -1, 4)
    assert updated.get_ground_height(-1.75, 3.25) == 0
    assert updated.get_ground_height(2, 3) is None


@pytest.mark.parametrize('change', [
    {'unknown_policy': 'free'}, {'inflation_policy': 'preinflated'},
    {'height_reference': 'relative'}, {'resolution': 0},
    {'vertical_bounds': [2, 1]}, {'default_cost': -1},
])
def test_unsupported_or_invalid_grid_configuration(change):
    _, grid = maps()
    settings = {**grid.settings, **change}
    with pytest.raises((ValueError, NotImplementedError)):
        HeightGridMap(settings)


def test_grid_update_rejection_preserves_snapshot():
    _, grid = maps()
    with pytest.raises(ValueError):
        grid.apply_update({'set_cells': [{'index': [99, 0], 'cell': {}}]})
    with pytest.raises(ValueError):
        grid.apply_update({'set_cells': [{'index': [0, 0], 'cell': {
            'ground_height': 0, 'occupied_intervals': [[1, 2]], 'obstacle_top_height': 1,
        }}]})
    assert grid.revision == 0 and grid.get_unknown_regions() == ()


def test_multiple_occupied_layers_preserve_vertical_gap():
    for planning_map in maps({(3, 1): [[0, .5], [2, 2.5]]}):
        assert planning_map.get_occupied_height_intervals(3.5, 1.5) == ((0, .5), (2, 2.5))
        assert planning_map.get_obstacle_top_height(3.5, 1.5) == 2.5
        fly = FlyStateValidator(DATA['test_parameters'])
        assert fly.is_valid(Node(0, 3.5, 1.5, 1, MotionMode.FLY), planning_map)
        assert not fly.is_valid(Node(0, 3.5, 1.5, 2, MotionMode.FLY), planning_map)


@pytest.mark.parametrize('policy', ['block', 'reject_map'])
def test_independent_unknown_column_policy(policy):
    settings = deepcopy(DATA['height_grid_mock'])
    settings['unknown_policy'] = policy
    settings['cells'] = [[{'ground_height': 0, 'occupied_intervals': []}
                          for _ in range(8)] for _ in range(4)]
    settings['cells'][1][3] = {'ground_height': 0, 'obstacle_top_height': 1}
    grid = HeightGridMap(settings)
    walk = WalkStateValidator(DATA['test_parameters'])
    fly = FlyStateValidator(DATA['test_parameters'])
    if policy == 'reject_map':
        with pytest.raises(NotImplementedError, match='Unknown'):
            walk.is_valid(PoseState(1.5, 1.5, 0, 0), grid)
        with pytest.raises(NotImplementedError, match='Unknown'):
            fly.is_valid(Node(0, 1.5, 1.5, 1, MotionMode.FLY), grid)
    else:
        assert walk.is_valid(PoseState(1.5, 1.5, 0, 0), grid)
        assert not walk.is_valid(PoseState(3.5, 1.5, 0, 0), grid)
        assert fly.is_valid(Node(0, 1.5, 1.5, 1, MotionMode.FLY), grid)
        assert not fly.is_valid(Node(0, 3.5, 1.5, 2, MotionMode.FLY), grid)
        assert not fly.is_edge_valid(Node(0, 2.5, 1.5, 2, MotionMode.FLY),
                                     Node(1, 4.5, 1.5, 2, MotionMode.FLY), grid)
        current = planner(grid)
        audit(current, current.plan())
    assert grid.get_occupied_height_intervals(3.5, 1.5) is None


@pytest.mark.parametrize('x,y,expected', [
    (3, 1.5, ((0, 2),)), (4, 1.5, ()), (3.999, 1.5, ((0, 2),)),
    (8, 1.5, None), (-.01, 1.5, None), (3.5, 4, None),
    (0, 0, ()), (float('nan'), 0, None),
])
def test_independent_half_open_point_queries(x, y, expected):
    _, grid = maps({(3, 1): [[0, 2]]})
    assert grid.get_occupied_height_intervals(x, y) == expected
    top = max((b for a, b in expected), default=None) if expected is not None else None
    assert grid.get_obstacle_top_height(x, y) == top


def test_immutable_cached_snapshot_and_update_invalidation():
    _, grid = maps()
    cached = grid.get_occupied_volumes()
    assert grid.get_occupied_volumes() is cached
    exposed = grid.cells
    exposed[1][3]['occupied_intervals'] = [[0, 2]]
    assert grid.get_occupied_height_intervals(3.5, 1.5) == ()
    updated = grid.apply_update({'set_cells': [{'index': [3, 1], 'cell': {
        'ground_height': 0, 'occupied_intervals': [[0, 2]], 'cost': 5,
    }}]})
    assert updated.revision == 1 and grid.revision == 0
    assert updated.get_occupied_volumes() is not cached
    assert grid.get_occupied_height_intervals(3.5, 1.5) == ()
    assert updated.get_occupied_height_intervals(3.5, 1.5) == ((0, 2),)
    assert len(updated.query_occupied_volumes((3.5, 1.5), (3.5, 1.5))) == 1
    assert updated.query_occupied_volumes((6, 2), (7, 3)) == ()


def test_independent_grid_cost_ownership_and_updated_price():
    _, grid = maps()
    grid = grid.apply_update({'set_cells': [{'index': [3, 1], 'cell': {
        'ground_height': 0, 'occupied_intervals': [], 'cost': 5,
    }}]})
    current = planner(grid)
    assert current.motions.edge(PoseState(3, 1.5, 0, 0),
                                PoseState(4, 1.5, 0, 0)).cost == pytest.approx(6)
    assert current.motions.edge(PoseState(3, 2, 0, 0),
                                PoseState(4, 2, 0, 0)).cost == pytest.approx(1)


def test_nonflat_known_terrain_rejected_and_missing_ground_blocked():
    _, grid = maps()
    nonflat = grid.apply_update({'set_cells': [{'index': [3, 1], 'cell': {
        'ground_height': .25, 'occupied_intervals': [],
    }}]})
    assert not nonflat.supports_flat_walk_edges()
    with pytest.raises(NotImplementedError, match='flat'):
        planner(nonflat)
    settings = grid.settings
    settings['unknown_policy'] = 'block'
    settings['cells'][1][3] = {'occupied_intervals': []}
    blocked = HeightGridMap(settings)
    assert blocked.supports_flat_walk_edges()
    assert blocked.get_ground_height(3.5, 1.5) is None
    assert not FlyStateValidator(DATA['test_parameters']).is_valid(
        Node(0, 3.5, 1.5, 2, MotionMode.FLY), blocked)


def test_box_unknown_blocking_uses_shared_validators():
    box, _ = maps()
    world = deepcopy(box.world)
    world['unknown_policy'] = 'block'
    world['unknown_regions'] = [{'min': [3, 1], 'max': [4, 2]}]
    blocked = DummyPlanningMap({'world': world})
    assert not WalkStateValidator(DATA['test_parameters']).is_valid(
        PoseState(3.5, 1.5, 0, 0), blocked)
    assert not FlyStateValidator(DATA['test_parameters']).is_valid(
        Node(0, 3.5, 1.5, 2, MotionMode.FLY), blocked)
    current = planner(blocked)
    audit(current, current.plan())
