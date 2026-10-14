"""Run repeated static and dynamic search comparisons on identical fixtures."""

import argparse
import csv
import gc
import json
import math
import platform
import statistics
import sys
import time
import tracemalloc
from copy import deepcopy
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src' / 'm4_global_planner'))

from m4_global_planner.astar import astar, dijkstra
from m4_global_planner.benchmark_algorithms import (
    DStarLite,
    LPAStar,
    bidirectional_astar,
    bidirectional_dijkstra,
    safe_heuristic_scale,
    weighted_astar,
)
from m4_global_planner.graph import Edge
from fixtures import dynamic_corridor, scenarios


WEIGHTS = (1.0, 1.25, 1.5, 2.0)
STATIC_ALGORITHMS = (
    'dijkstra',
    'astar_h0',
    'astar_safe',
    'weighted_astar_w1',
    'bidirectional_dijkstra',
    'bidirectional_astar',
    'lpa_star',
    'd_star_lite',
)


def _timer_ns(function):
    began = time.perf_counter_ns()
    result = function()
    return result, time.perf_counter_ns() - began


def _run_static(name, graph, start, goal, scale):
    if name == 'dijkstra':
        return dijkstra(graph, start, goal)
    if name == 'astar_h0':
        return astar(graph, start, goal, heuristic_scale=0.0)
    if name == 'astar_safe':
        return astar(graph, start, goal, heuristic_scale=scale)
    if name == 'weighted_astar_w1':
        return weighted_astar(graph, start, goal, 1.0, scale)
    if name == 'bidirectional_dijkstra':
        return bidirectional_dijkstra(graph, start, goal)
    if name == 'bidirectional_astar':
        return bidirectional_astar(graph, start, goal, scale)
    if name == 'lpa_star':
        return LPAStar(graph, start, goal, scale).plan()
    if name == 'd_star_lite':
        return DStarLite(graph, start, goal, scale).plan()
    raise ValueError(f'Unknown static algorithm: {name}')


def _validate(graph, start, goal, result, reference, label, optimal=False):
    success = result['success']
    path = result['path']
    if success:
        if not path or path[0] != start or path[-1] != goal:
            raise RuntimeError(f'{label}: path has incorrect endpoints: {path[:4]}')
        if any(target not in graph.edges.get(source, {}) for source, target in zip(path, path[1:])):
            raise RuntimeError(f'{label}: path contains an edge absent from the graph')
        cost = math.fsum(graph.edges[source][target].cost
                         for source, target in zip(path, path[1:]))
        if not math.isclose(cost, result['total_cost'], rel_tol=1e-9, abs_tol=1e-9):
            raise RuntimeError(f'{label}: recomputed path cost differs from reported cost')
        if not reference['success']:
            raise RuntimeError(f'{label}: reported a route where Dijkstra found no path')
        if cost < reference['total_cost'] and not math.isclose(
            cost, reference['total_cost'], rel_tol=1e-9, abs_tol=1e-9
        ):
            raise RuntimeError(f'{label}: route cost is below the Dijkstra reference')
        if optimal and not math.isclose(
            cost, reference['total_cost'], rel_tol=1e-9, abs_tol=1e-9
        ):
            raise RuntimeError(f'{label}: exact search differs from Dijkstra cost')
    else:
        if path or reference['success'] or not math.isinf(result['total_cost']):
            raise RuntimeError(f'{label}: incorrect no-path result')
    return True


def _ratio(result, reference):
    if not reference['success'] or not result['success']:
        return '', ''
    if math.isclose(result['total_cost'], reference['total_cost'],
                    rel_tol=1e-12, abs_tol=1e-12):
        return 1.0, 0.0
    if reference['total_cost'] == 0:
        ratio = 1.0 if result['total_cost'] == 0 else math.inf
    else:
        ratio = result['total_cost'] / reference['total_cost']
    return ratio, (ratio - 1.0) * 100.0


def _stats(values):
    return {
        'median_runtime_ms': statistics.median(values) / 1e6,
        'mean_runtime_ms': statistics.fmean(values) / 1e6,
        'runtime_stddev_ms': statistics.stdev(values) / 1e6 if len(values) > 1 else 0.0,
        'min_runtime_ms': min(values) / 1e6,
        'max_runtime_ms': max(values) / 1e6,
    }


def _peak_python_memory(name, graph, start, goal, scale):
    gc.collect()
    tracemalloc.start()
    try:
        result = _run_static(name, graph, start, goal, scale)
        _, peak = tracemalloc.get_traced_memory()
        return result, peak
    finally:
        tracemalloc.stop()


def _write_csv(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        raise RuntimeError(f'No benchmark rows produced for {path.name}')
    with path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def run_static(cases, runs, output_dir):
    summary_rows, raw_rows, weighted_summary, weighted_raw = [], [], [], []
    for scenario, case in cases.items():
        graph, start, goal = case['graph'], case['start'], case['goal']
        reference = dijkstra(graph, start, goal)
        scale = safe_heuristic_scale(graph)
        node_count = len(graph.nodes)
        edge_count = sum(map(len, graph.edges.values()))
        walk_count = sum(getattr(node.mode, 'value', node.mode) == 'walk'
                         for node in graph.nodes.values())
        fly_count = sum(getattr(node.mode, 'value', node.mode) == 'fly'
                        for node in graph.nodes.values())

        for algorithm in STATIC_ALGORITHMS:
            measurements, results = [], []
            for run_index in range(runs):
                result, elapsed = _timer_ns(
                    lambda: _run_static(algorithm, graph, start, goal, scale)
                )
                _validate(graph, start, goal, result, reference,
                          f'{scenario}/{algorithm}/run-{run_index + 1}',
                          optimal=True)
                measurements.append(elapsed)
                results.append(result)
                raw_rows.append({
                    'scenario': scenario,
                    'algorithm': algorithm,
                    'run': run_index + 1,
                    'runtime_ms': elapsed / 1e6,
                    'success': result['success'],
                    'path_cost': result['total_cost'],
                    'expanded_nodes': result['nodes_expanded'],
                    'generated_nodes': result.get('nodes_generated', ''),
                    'processed_nodes': result.get('processed_nodes',
                                                  result['nodes_expanded']),
                    'edge_evaluations': result.get('edge_evaluations', ''),
                })
            peak_bytes = ''
            if scenario == 'small_static_grid':
                memory_result, peak_bytes = _peak_python_memory(
                    algorithm, graph, start, goal, scale
                )
                _validate(graph, start, goal, memory_result, reference,
                          f'{scenario}/{algorithm}/memory-run', optimal=True)
            representative = results[0]
            ratio, degradation = _ratio(representative, reference)
            summary_rows.append({
                'scenario': scenario,
                'purpose': case['purpose'],
                'algorithm': algorithm,
                'weight': 1.0 if algorithm == 'weighted_astar_w1' else '',
                'heuristic': (
                    'zero' if algorithm == 'astar_h0'
                    else 'safe_admissible' if algorithm in (
                        'astar_safe', 'weighted_astar_w1', 'bidirectional_astar',
                        'lpa_star', 'd_star_lite',
                    ) else 'not_applicable'
                ),
                'runs': runs,
                '|V|': node_count,
                '|E|': edge_count,
                'start': start,
                'goal': goal,
                'walk_nodes': walk_count,
                'fly_nodes': fly_count,
                'success': representative['success'],
                'path_valid': True,
                'path_state_count': len(representative['path']),
                'path_cost': representative['total_cost'],
                'dijkstra_reference_cost': reference['total_cost'],
                'optimality_ratio': ratio,
                'suboptimality_pct': degradation,
                'expanded_nodes': representative['nodes_expanded'],
                'generated_nodes': representative.get('nodes_generated', ''),
                'processed_nodes': representative.get(
                    'processed_nodes', representative['nodes_expanded']
                ),
                'edge_evaluations': representative.get('edge_evaluations', ''),
                **_stats(measurements),
                'peak_python_memory_bytes': peak_bytes,
            })

        for weight in WEIGHTS:
            measurements, results = [], []
            for run_index in range(runs):
                result, elapsed = _timer_ns(
                    lambda: weighted_astar(
                        graph, start, goal, weight, scale
                    )
                )
                _validate(graph, start, goal, result, reference,
                          f'{scenario}/weighted_astar-{weight}/run-{run_index + 1}',
                          optimal=(weight == 1.0))
                measurements.append(elapsed)
                results.append(result)
                weighted_raw.append({
                    'scenario': scenario,
                    'weight': weight,
                    'run': run_index + 1,
                    'runtime_ms': elapsed / 1e6,
                    'success': result['success'],
                    'path_cost': result['total_cost'],
                    'expanded_nodes': result['nodes_expanded'],
                    'generated_nodes': result.get('nodes_generated', ''),
                    'edge_evaluations': result.get('edge_evaluations', ''),
                })
            representative = results[0]
            ratio, degradation = _ratio(representative, reference)
            weighted_summary.append({
                'scenario': scenario,
                'purpose': case['purpose'],
                'weight': weight,
                'runs': runs,
                '|V|': node_count,
                '|E|': edge_count,
                'start': start,
                'goal': goal,
                'success': representative['success'],
                'path_valid': True,
                'path_state_count': len(representative['path']),
                'path_cost': representative['total_cost'],
                'dijkstra_reference_cost': reference['total_cost'],
                'optimality_ratio': ratio,
                'suboptimality_pct': degradation,
                'expanded_nodes': representative['nodes_expanded'],
                'generated_nodes': representative.get('nodes_generated', ''),
                'processed_nodes': representative.get(
                    'processed_nodes', representative['nodes_expanded']
                ),
                'edge_evaluations': representative.get('edge_evaluations', ''),
                **_stats(measurements),
            })

    _write_csv(output_dir / 'benchmark_static.csv', summary_rows)
    _write_csv(output_dir / 'benchmark_static_runs.csv', raw_rows)
    _write_csv(output_dir / 'benchmark_weighted_astar.csv', weighted_summary)
    _write_csv(output_dir / 'benchmark_weighted_astar_runs.csv', weighted_raw)
    return summary_rows, weighted_summary


def _edit_pair(graph, pair, cost):
    source, target = pair
    if cost is None:
        graph.edges[source].pop(target, None)
        graph.edges[target].pop(source, None)
    else:
        graph.edges[source][target] = Edge(source, target, cost)
        graph.edges[target][source] = Edge(target, source, cost)


def _updates_for_dynamic(graph, start, goal):
    state = deepcopy(graph)
    initial = astar(state, start, goal, safe_heuristic_scale(state))
    if not initial['success'] or len(initial['path']) < 6:
        raise RuntimeError('Dynamic fixture must have a nontrivial initial route')
    first_pair = tuple(initial['path'][len(initial['path']) // 2:
                                        len(initial['path']) // 2 + 2])
    updates = [{'type': 'block_edge', 'pair': first_pair, 'cost': None}]
    _edit_pair(state, first_pair, None)
    route = astar(state, start, goal, safe_heuristic_scale(state))
    second_pair = tuple(route['path'][len(route['path']) // 3:
                                      len(route['path']) // 3 + 2])
    if set(second_pair) == set(first_pair):
        second_pair = tuple(route['path'][max(1, len(route['path']) // 4):
                                          max(1, len(route['path']) // 4) + 2])
    updates.append({'type': 'increase_edge_cost', 'pair': second_pair, 'cost': 12.0})
    _edit_pair(state, second_pair, 12.0)
    route = astar(state, start, goal, safe_heuristic_scale(state))
    updates.append({'type': 'reopen_edge', 'pair': first_pair, 'cost': 1.0})
    _edit_pair(state, first_pair, 1.0)
    route = astar(state, start, goal, safe_heuristic_scale(state))
    candidates = [
        tuple(route['path'][index:index + 2])
        for index in range(1, len(route['path']) - 1)
        if set(route['path'][index:index + 2]) not in
        (set(first_pair), set(second_pair))
    ]
    if not candidates:
        raise RuntimeError('Dynamic fixture did not provide an edge for final blockage')
    final_pair = candidates[len(candidates) // 2]
    updates.append({'type': 'block_edge', 'pair': final_pair, 'cost': None})
    return updates


def _dynamic_one_trial(scenario, case, updates, repetition):
    source_graph, start, goal = case['graph'], case['start'], case['goal']
    algorithms = ('fresh_astar', 'lpa_star', 'd_star_lite')
    result_rows = []
    for algorithm in algorithms:
        graph = deepcopy(source_graph)
        reference_graph = deepcopy(source_graph)
        scale = safe_heuristic_scale(graph)
        initialization_ms = 0.0
        if algorithm == 'lpa_star':
            planner, elapsed = _timer_ns(lambda: LPAStar(graph, start, goal, scale))
            initialization_ms = elapsed / 1e6
        elif algorithm == 'd_star_lite':
            planner, elapsed = _timer_ns(lambda: DStarLite(graph, start, goal, scale))
            initialization_ms = elapsed / 1e6
        else:
            planner = None

        if planner is None:
            result, initial_ns = _timer_ns(lambda: astar(graph, start, goal, scale))
        else:
            result, initial_ns = _timer_ns(planner.plan)
        initial_reference = dijkstra(reference_graph, start, goal)
        _validate(graph, start, goal, result, initial_reference,
                  f'{scenario}/{algorithm}/initial/{repetition}', optimal=True)
        for step_number, update in enumerate(updates, 1):
            update_began = time.perf_counter_ns()
            if algorithm == 'fresh_astar':
                _edit_pair(graph, update['pair'], update['cost'])
            else:
                u, v = update['pair']
                planner.update_edge(u, v, update['cost'])
                planner.update_edge(v, u, update['cost'])
                graph = planner.graph
            update_ns = time.perf_counter_ns() - update_began
            _edit_pair(reference_graph, update['pair'], update['cost'])
            if algorithm == 'fresh_astar':
                result, replan_ns = _timer_ns(
                    lambda: astar(graph, start, goal, scale)
                )
            else:
                result, replan_ns = _timer_ns(planner.replan)
            reference = dijkstra(reference_graph, start, goal)
            _validate(graph, start, goal, result, reference,
                      f'{scenario}/{algorithm}/update-{step_number}/{repetition}',
                      optimal=True)
            ratio, degradation = _ratio(result, reference)
            result_rows.append({
                'scenario': scenario,
                'algorithm': algorithm,
                'repetition': repetition,
                'update_number': step_number,
                'update_type': update['type'],
                'changed_edge': f'{update["pair"][0]}->{update["pair"][1]} and reverse',
                'initialization_ms': initialization_ms,
                'initial_plan_ms': initial_ns / 1e6,
                'update_processing_ms': update_ns / 1e6,
                'replanning_ms': replan_ns / 1e6,
                'total_update_to_route_ms': (update_ns + replan_ns) / 1e6,
                'success': result['success'],
                'path_valid': True,
                'path_state_count': len(result['path']),
                'replanned_path_cost': result['total_cost'],
                'updated_dijkstra_reference_cost': reference['total_cost'],
                'optimality_ratio': ratio,
                'suboptimality_pct': degradation,
                'expanded_nodes_after_update': result['nodes_expanded'],
                'generated_nodes_after_update': result.get('nodes_generated', ''),
                'edge_evaluations_after_update': result.get('edge_evaluations', ''),
            })
    return result_rows


def run_dynamic(cases, repetitions, output_dir):
    scenario = 'dynamic_two_corridor_graph'
    graph, start, goal = dynamic_corridor()
    case = {'graph': graph, 'start': start, 'goal': goal}
    updates = _updates_for_dynamic(graph, case['start'], case['goal'])
    raw_rows = []
    for repetition in range(1, repetitions + 1):
        raw_rows.extend(_dynamic_one_trial(scenario, case, updates, repetition))

    summary_rows = []
    keys = sorted({(row['update_number'], row['algorithm']) for row in raw_rows})
    for update_number, algorithm in keys:
        rows = [row for row in raw_rows
                if row['update_number'] == update_number and row['algorithm'] == algorithm]
        fresh_rows = [row for row in raw_rows
                      if row['update_number'] == update_number
                      and row['algorithm'] == 'fresh_astar']
        times = [row['total_update_to_route_ms'] for row in rows]
        fresh_median = statistics.median(
            row['total_update_to_route_ms'] for row in fresh_rows
        )
        own_median = statistics.median(times)
        representative = rows[0]
        expanded_median = statistics.median(
            row['expanded_nodes_after_update'] for row in rows
        )
        fresh_expanded = statistics.median(
            row['expanded_nodes_after_update'] for row in fresh_rows
        )
        summary_rows.append({
            'scenario': scenario,
            'algorithm': algorithm,
            'repetitions': repetitions,
            'update_number': update_number,
            'update_type': representative['update_type'],
            'changed_edge': representative['changed_edge'],
            'initialization_ms': representative['initialization_ms'],
            'initial_plan_median_ms': statistics.median(
                row['initial_plan_ms'] for row in rows
            ),
            'initialization_median_ms': statistics.median(
                row['initialization_ms'] for row in rows
            ),
            'update_processing_median_ms': statistics.median(
                row['update_processing_ms'] for row in rows
            ),
            'replanning_median_ms': statistics.median(
                row['replanning_ms'] for row in rows
            ),
            'total_update_to_route_median_ms': own_median,
            'total_update_to_route_mean_ms': statistics.fmean(times),
            'total_update_to_route_stddev_ms': (
                statistics.stdev(times) if len(times) > 1 else 0.0
            ),
            'total_update_to_route_min_ms': min(times),
            'total_update_to_route_max_ms': max(times),
            'speedup_vs_fresh_astar': fresh_median / own_median if own_median else '',
            'expanded_nodes_median': expanded_median,
            'fresh_astar_expanded_nodes_median': fresh_expanded,
            'expansion_reduction_pct': (
                (fresh_expanded - expanded_median) / fresh_expanded * 100
                if fresh_expanded else ''
            ),
            'replanned_path_cost': representative['replanned_path_cost'],
            'updated_dijkstra_reference_cost': representative[
                'updated_dijkstra_reference_cost'
            ],
            'optimality_ratio': representative['optimality_ratio'],
            'suboptimality_pct': representative['suboptimality_pct'],
            'success': representative['success'],
            'path_valid': True,
        })
    _write_csv(output_dir / 'benchmark_dynamic.csv', summary_rows)
    _write_csv(output_dir / 'benchmark_dynamic_runs.csv', raw_rows)
    return summary_rows, updates


def _make_plots(static_rows, weighted_rows, dynamic_rows, output_dir):
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        return False

    graph_sizes = {}
    for row in static_rows:
        if row['scenario'] not in graph_sizes:
            graph_sizes[row['scenario']] = (
                int(row['|V|']), int(row['|E|']), row['purpose']
            )
    scalable = sorted(
        (row for row in static_rows if 'static_' in row['scenario']),
        key=lambda row: int(row['|V|']),
    )
    for metric, ylabel, filename in (
        ('median_runtime_ms', 'Median planning runtime (ms)', 'runtime_vs_graph_size.png'),
        ('expanded_nodes', 'Expanded nodes', 'expanded_nodes_vs_graph_size.png'),
    ):
        fig, axis = plt.subplots(figsize=(9, 5))
        for algorithm in ('dijkstra', 'astar_h0', 'astar_safe',
                          'bidirectional_dijkstra', 'bidirectional_astar',
                          'lpa_star', 'd_star_lite'):
            rows = [r for r in scalable if r['algorithm'] == algorithm]
            axis.plot([int(r['|V|']) for r in rows], [float(r[metric]) for r in rows],
                      marker='o', label=algorithm)
        axis.set_xlabel('Graph nodes |V|')
        axis.set_ylabel(ylabel)
        axis.set_xscale('log')
        if metric == 'median_runtime_ms':
            axis.set_yscale('log')
        axis.grid(True, alpha=0.25)
        axis.legend(fontsize='small')
        fig.tight_layout()
        fig.savefig(output_dir / filename, dpi=150)
        plt.close(fig)

    quality = [row for row in static_rows if row['optimality_ratio'] != '']
    fig, axis = plt.subplots(figsize=(12, 5))
    for algorithm in STATIC_ALGORITHMS:
        selected = [r for r in quality if r['algorithm'] == algorithm]
        axis.plot([r['scenario'] for r in selected],
                  [float(r['suboptimality_pct']) for r in selected],
                  marker='o', label=algorithm)
    axis.set_ylabel('Suboptimality (%)')
    axis.tick_params(axis='x', labelrotation=45)
    axis.grid(True, alpha=0.25)
    axis.legend(fontsize='small', ncol=2)
    fig.tight_layout()
    fig.savefig(output_dir / 'path_optimality_comparison.png', dpi=150)
    plt.close(fig)

    fig, axis = plt.subplots(figsize=(9, 5))
    for algorithm in ('fresh_astar', 'lpa_star', 'd_star_lite'):
        rows = sorted((r for r in dynamic_rows if r['algorithm'] == algorithm),
                      key=lambda r: r['update_number'])
        axis.plot([r['update_number'] for r in rows],
                  [float(r['total_update_to_route_median_ms']) for r in rows],
                  marker='o', label=algorithm)
    axis.set_xlabel('Sequential graph update')
    axis.set_ylabel('Median update-to-route time (ms)')
    axis.set_xticks((1, 2, 3, 4))
    axis.grid(True, alpha=0.25)
    axis.legend()
    fig.tight_layout()
    fig.savefig(output_dir / 'dynamic_replanning_runtime.png', dpi=150)
    plt.close(fig)

    selected_scenarios = ('large_static_grid', 'very_large_static_grid')
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.5))
    for scenario in selected_scenarios:
        rows = sorted((r for r in weighted_rows if r['scenario'] == scenario),
                      key=lambda r: float(r['weight']))
        if rows:
            axes[0].plot([float(r['weight']) for r in rows],
                         [float(r['median_runtime_ms']) for r in rows],
                         marker='o', label=scenario)
            axes[1].plot([float(r['weight']) for r in rows],
                         [float(r['suboptimality_pct'])
                          if r['suboptimality_pct'] != '' else 0 for r in rows],
                         marker='o', label=scenario)
    axes[0].set_ylabel('Median runtime (ms)')
    axes[1].set_ylabel('Suboptimality (%)')
    for axis in axes:
        axis.set_xlabel('Weighted A* weight')
        axis.set_xticks(WEIGHTS)
        axis.grid(True, alpha=0.25)
        axis.legend(fontsize='small')
    fig.tight_layout()
    fig.savefig(output_dir / 'weighted_astar_tradeoff.png', dpi=150)
    plt.close(fig)
    return True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runs', type=int, default=10,
                        help='measured runs per static algorithm/scenario (default: 10)')
    parser.add_argument('--dynamic-runs', type=int, default=5,
                        help='independent full update sequences (default: 5)')
    parser.add_argument('--scenario', action='append',
                        help='run this static scenario only; repeat to select more than one')
    parser.add_argument('--output-dir', type=Path, default=ROOT / 'benchmark_results')
    args = parser.parse_args()
    if args.runs < 2 or args.dynamic_runs < 2:
        parser.error('Use at least two repetitions to report standard deviation')
    cases = scenarios()
    selected_cases = cases
    if args.scenario:
        unknown = set(args.scenario) - set(cases)
        if unknown:
            parser.error(f'Unknown scenario(s): {", ".join(sorted(unknown))}')
        selected_cases = {name: cases[name] for name in args.scenario}
    args.output_dir.mkdir(parents=True, exist_ok=True)
    static_rows, weighted_rows = run_static(selected_cases, args.runs, args.output_dir)
    dynamic_rows, updates = run_dynamic(cases, args.dynamic_runs, args.output_dir)
    has_plots = _make_plots(static_rows, weighted_rows, dynamic_rows, args.output_dir)
    dynamic_graph, _, _ = dynamic_corridor()
    manifest = {
        'python': platform.python_version(),
        'platform': platform.platform(),
        'static_runs': args.runs,
        'dynamic_sequences': args.dynamic_runs,
        'weights': WEIGHTS,
        'static_scenarios': list(selected_cases),
        'scenarios': {
            name: {
                'purpose': case['purpose'],
                'nodes': len(case['graph'].nodes),
                'edges': sum(len(edges) for edges in case['graph'].edges.values()),
                'start': case['start'],
                'goal': case['goal'],
                'walk_nodes': sum(getattr(node.mode, 'value', node.mode) == 'walk'
                                  for node in case['graph'].nodes.values()),
                'fly_nodes': sum(getattr(node.mode, 'value', node.mode) == 'fly'
                                 for node in case['graph'].nodes.values()),
            }
            for name, case in cases.items()
        },
        'dynamic_updates': [
            {'type': update['type'], 'pair': update['pair'], 'cost': update['cost']}
            for update in updates
        ],
        'dynamic_scenario': {
            'name': 'dynamic_two_corridor_graph',
            'purpose': 'Repeated block/increase/reopen/block across a short primary route and longer alternate route',
            'nodes': len(dynamic_graph.nodes),
            'edges': sum(len(edges) for edges in dynamic_graph.edges.values()),
            'start': 0,
            'goal': 300,
            'walk_nodes': len(dynamic_graph.nodes),
            'fly_nodes': 0,
        },
        'plots_generated': has_plots,
        'timing_clock': 'time.perf_counter_ns',
        'memory_metric': 'tracemalloc peak Python allocations; graph input excluded',
        'path_validation': 'endpoints, every edge, recomputed cost, no-path vs Dijkstra',
    }
    (args.output_dir / 'benchmark_manifest.json').write_text(
        json.dumps(manifest, indent=2), encoding='utf-8'
    )
    print(f'Static rows: {len(static_rows)}; weighted rows: {len(weighted_rows)}')
    print(f'Dynamic rows: {len(dynamic_rows)}; plots generated: {has_plots}')
    print(f'Results: {args.output_dir}')


if __name__ == '__main__':
    main()
