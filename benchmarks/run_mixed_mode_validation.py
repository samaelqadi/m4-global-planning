"""Measure GlobalPlanner and Dijkstra on deterministic large mixed-mode inputs."""

import argparse
import csv
import json
import math
import platform
import statistics
import sys
import time
from copy import deepcopy
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src' / 'm4_global_planner'))

from m4_global_planner.astar import dijkstra
from m4_global_planner.global_planner import GlobalPlanner
from mixed_mode_fixtures import (
    GOAL_ID,
    START_ID,
    apply_cost_profile,
    build_mixed_mode_fixture,
    selected_route_family,
)


SCENARIOS = (
    ('walk_favored', 128, 'walk_favored', 'walk', False),
    ('fly_favored', 128, 'fly_favored', 'fly', False),
    ('mixed_favored', 128, 'mixed_favored', 'mixed', False),
    ('updated_to_fly', 128, 'fly_favored', 'fly', True),
    ('large_mixed_scale', 512, 'mixed_favored', 'mixed', False),
)
ALGORITHMS = ('astar_safe', 'dijkstra')


def _timing_stats(samples):
    return {
        'runtime_median_ms': statistics.median(samples),
        'runtime_mean_ms': statistics.fmean(samples),
        'runtime_stddev_ms': statistics.stdev(samples),
        'runtime_min_ms': min(samples),
        'runtime_max_ms': max(samples),
    }


def _assert_valid(graph, result, reference, expected_family, scenario, algorithm):
    label = f'{scenario}/{algorithm}'
    path = result['path']
    if not result['success'] or not path or path[0] != START_ID or path[-1] != GOAL_ID:
        raise RuntimeError(f'{label}: route success/endpoints invalid')
    if any(target not in graph.edges.get(source, {})
           for source, target in zip(path, path[1:])):
        raise RuntimeError(f'{label}: route contains a missing supplied connection')
    recomputed = math.fsum(
        graph.edges[source][target].cost for source, target in zip(path, path[1:])
    )
    if not math.isclose(recomputed, result['total_cost'], rel_tol=1e-12, abs_tol=1e-12):
        raise RuntimeError(f'{label}: returned route cost does not match supplied edges')
    if not math.isclose(recomputed, reference['total_cost'],
                        rel_tol=1e-12, abs_tol=1e-12):
        raise RuntimeError(f'{label}: route does not match Dijkstra optimum')
    family = selected_route_family(result)
    if family != expected_family:
        raise RuntimeError(f'{label}: expected {expected_family} route, got {family}')
    return recomputed


def _run_case(scenario, alternatives, profile_name, expected_family, is_update, runs):
    fixture = build_mixed_mode_fixture(alternatives)
    if is_update:
        apply_cost_profile(fixture, 'walk_favored')
        base_graph = deepcopy(fixture.graph)
        apply_cost_profile(fixture, profile_name)
    else:
        apply_cost_profile(fixture, profile_name)
        base_graph = None
    graph = fixture.graph
    reference = dijkstra(graph, START_ID, GOAL_ID)
    node_count = len(graph.nodes)
    edge_count = sum(map(len, graph.edges.values()))
    mode_counts = {
        'walk_states': sum(node.mode.value == 'walk' for node in graph.nodes.values()),
        'fly_states': sum(node.mode.value == 'fly' for node in graph.nodes.values()),
    }
    raw_rows = []
    summary_rows = []

    for algorithm in ALGORITHMS:
        search_method = 'astar' if algorithm == 'astar_safe' else 'dijkstra'
        runtime_samples = []
        search_samples = []
        update_samples = []
        results = []
        for repetition in range(1, runs + 1):
            planner = GlobalPlanner(search_method=search_method)
            accepted_graph = base_graph if is_update else graph
            planner.set_graph(accepted_graph, f'{scenario}-base' if is_update else scenario)

            update_ms = 0.0
            if is_update:
                updated_graph = deepcopy(base_graph)
                update_fixture = type(fixture)(
                    graph=updated_graph,
                    alternatives_per_family=alternatives,
                    lane_edges=fixture.lane_edges,
                )
                began_update = time.perf_counter_ns()
                apply_cost_profile(update_fixture, profile_name)
                planner.set_graph(updated_graph, scenario)
                update_ms = (time.perf_counter_ns() - began_update) / 1e6

            began_plan = time.perf_counter_ns()
            result = planner.plan(START_ID, GOAL_ID)
            plan_ms = (time.perf_counter_ns() - began_plan) / 1e6
            _assert_valid(graph, result, reference, expected_family, scenario, algorithm)
            runtime_samples.append(plan_ms)
            search_samples.append(result['search_ms'])
            update_samples.append(update_ms)
            results.append(result)
            raw_rows.append({
                'scenario': scenario,
                'algorithm': algorithm,
                'run': repetition,
                '|V|': node_count,
                '|E|': edge_count,
                'alternative_routes': alternatives * len(fixture.lane_edges),
                'success': result['success'],
                'path_valid': True,
                'path_state_count': len(result['path']),
                'path_family': selected_route_family(result),
                'path_cost': result['total_cost'],
                'dijkstra_reference_cost': reference['total_cost'],
                'optimality_ratio': 1.0,
                'suboptimality_pct': 0.0,
                'expanded_states': result['nodes_expanded'],
                'generated_states': result['nodes_generated'],
                'processed_states': result['processed_nodes'],
                'connection_evaluations': result['edge_evaluations'],
                'planning_runtime_ms': plan_ms,
                'search_runtime_ms': result['search_ms'],
                'update_processing_ms': update_ms,
                'total_update_to_route_ms': update_ms + plan_ms if is_update else '',
            })

        representative = results[0]
        row = {
            'scenario': scenario,
            'algorithm': algorithm,
            'cost_profile': profile_name,
            'expected_route_family': expected_family,
            'runs': runs,
            '|V|': node_count,
            '|E|': edge_count,
            'walk_states_in_input': mode_counts['walk_states'],
            'fly_states_in_input': mode_counts['fly_states'],
            'alternatives_per_family': alternatives,
            'complete_route_alternatives': alternatives * len(fixture.lane_edges),
            'start': START_ID,
            'goal': GOAL_ID,
            'success': representative['success'],
            'path_valid': True,
            'path_state_count': len(representative['path']),
            'path_family': selected_route_family(representative),
            'path_walk_states': sum(n.mode.value == 'walk' for n in representative['nodes']),
            'path_fly_states': sum(n.mode.value == 'fly' for n in representative['nodes']),
            'path_cost': representative['total_cost'],
            'dijkstra_reference_cost': reference['total_cost'],
            'optimality_ratio': 1.0,
            'suboptimality_pct': 0.0,
            'expanded_states': representative['nodes_expanded'],
            'generated_states': representative['nodes_generated'],
            'processed_states': representative['processed_nodes'],
            'connection_evaluations': representative['edge_evaluations'],
            **_timing_stats(runtime_samples),
            'search_runtime_median_ms': statistics.median(search_samples),
            'update_processing_median_ms': statistics.median(update_samples),
            'update_processing_mean_ms': statistics.fmean(update_samples),
            'update_processing_stddev_ms': statistics.stdev(update_samples),
            'update_processing_min_ms': min(update_samples),
            'update_processing_max_ms': max(update_samples),
            'total_update_to_route_median_ms': (
                statistics.median(
                    [u + p for u, p in zip(update_samples, runtime_samples)]
                ) if is_update else ''
            ),
            'exact_cost_match': True,
        }
        summary_rows.append(row)
    return summary_rows, raw_rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runs', type=int, default=7)
    parser.add_argument('--output-dir', type=Path, default=ROOT / 'benchmark_results')
    args = parser.parse_args()
    if args.runs < 2:
        parser.error('Use at least two measured runs to report standard deviation')
    summaries = []
    raw = []
    for scenario, alternatives, profile, expected, is_update in SCENARIOS:
        summary_rows, raw_rows = _run_case(
            scenario, alternatives, profile, expected, is_update, args.runs
        )
        summaries.extend(summary_rows)
        raw.extend(raw_rows)

    args.output_dir.mkdir(parents=True, exist_ok=True)
    for filename, rows in (
        ('benchmark_mixed_mode.csv', summaries),
        ('benchmark_mixed_mode_runs.csv', raw),
    ):
        with (args.output_dir / filename).open('w', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)

    manifest = {
        'python': platform.python_version(),
        'platform': platform.platform(),
        'runs_per_algorithm_scenario': args.runs,
        'timing_clock': 'time.perf_counter_ns',
        'planner_runtime': 'external wall time around GlobalPlanner.plan',
        'update_processing': (
            'cost-only profile application and replacement snapshot acceptance; '
            'graph copy excluded'
        ),
        'path_validation': (
            'start/goal, supplied directed connections, recomputed edge-cost sum, '
            'expected route mode family, and Dijkstra optimal cost'
        ),
        'scenarios': [
            {
                'name': scenario,
                'alternatives_per_family': alternatives,
                'expected_route_family': expected,
                'cost_profile': profile,
                'cost_only_update': is_update,
                'nodes': summaries[index * len(ALGORITHMS)]['|V|'],
                'edges': summaries[index * len(ALGORITHMS)]['|E|'],
                'walk_states': summaries[index * len(ALGORITHMS)]['walk_states_in_input'],
                'fly_states': summaries[index * len(ALGORITHMS)]['fly_states_in_input'],
            }
            for index, (scenario, alternatives, profile, expected, is_update)
            in enumerate(SCENARIOS)
        ],
        'memory_metric': 'not measured for these mixed-mode scenarios',
    }
    (args.output_dir / 'benchmark_mixed_mode_manifest.json').write_text(
        json.dumps(manifest, indent=2), encoding='utf-8'
    )

    for row in summaries:
        print(
            f"{row['scenario']}/{row['algorithm']}: "
            f"|V|={row['|V|']} |E|={row['|E|']} "
            f"route={row['path_family']} cost={row['path_cost']:.6f} "
            f"expanded={row['expanded_states']} "
            f"median={row['runtime_median_ms']:.4f} ms"
        )
    print(f'Wrote {len(summaries)} summaries and {len(raw)} raw runs to {args.output_dir}')


if __name__ == '__main__':
    main()
