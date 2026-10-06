from math import sqrt

import resource
import time

from m4_global_planner.graph import MotionMode


def distance(node1, node2):
    # calculate straight-line distance between two nodes
    dx = node2.x - node1.x
    dy = node2.y - node1.y
    dz = node2.z - node1.z

    return sqrt(dx**2 + dy**2 + dz**2)


def calculate_path_metrics(graph, path):
    total_distance = 0.0
    walk_distance = 0.0
    fly_distance = 0.0
    transition_distance = 0.0
    transitions = 0

    # check every movement in the final path
    for i in range(len(path) - 1):
        node1 = graph.get_node(path[i])
        node2 = graph.get_node(path[i + 1])

        segment_distance = distance(node1, node2)
        total_distance += segment_distance

        # movement stays in WALK mode
        if node1.mode == MotionMode.WALK and node2.mode == MotionMode.WALK:
            walk_distance += segment_distance

        # movement stays in FLY mode
        elif node1.mode == MotionMode.FLY and node2.mode == MotionMode.FLY:
            fly_distance += segment_distance

        # movement changes between WALK and FLY
        else:
            transition_distance += segment_distance
            transitions += 1

    return {
        'total_distance': total_distance,
        'walk_distance': walk_distance,
        'fly_distance': fly_distance,
        'transition_distance': transition_distance,
        'transitions': transitions
    }


def run_with_metrics(planner, graph, start_id, goal_id):
    # record CPU and real time before planning
    cpu_start = time.process_time()
    time_start = time.perf_counter()

    # run the planner
    result = planner(graph, start_id, goal_id)

    # calculate planning time
    planning_time = time.perf_counter() - time_start
    cpu_time = time.process_time() - cpu_start

    # get peak RAM used by this Python process
    peak_ram_mb = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024

    # calculate path measurements if a path was found
    if result['success']:
        path_metrics = calculate_path_metrics(graph, result['path'])
    else:
        path_metrics = {
            'total_distance': 0.0,
            'walk_distance': 0.0,
            'fly_distance': 0.0,
            'transition_distance': 0.0,
            'transitions': 0
        }

    # combine planner results and metrics
    return {
        **result,
        **path_metrics,
        'planning_time_ms': planning_time * 1000,
        'cpu_time_ms': cpu_time * 1000,
        'peak_ram_mb': peak_ram_mb
    }
