"""Checkpointed, serial searches over one saved joint graph per configuration."""

import argparse
import gc
import hashlib
import json
from math import ceil, isclose, isfinite, radians
from pathlib import Path
import pickle
import platform
import resource
import statistics
import subprocess
import sys
from time import perf_counter
import tracemalloc

from m4_global_planner.astar import astar
from m4_global_planner.dummy_map import DummyPlanningMap
from m4_global_planner.graph import MotionMode, Node
from m4_global_planner.metrics import run_with_metrics
from m4_global_planner.representation.anchors import MandatoryAnchorBuilder
from m4_global_planner.representation.fly_validator import FlyStateValidator, TransitionValidator
from m4_global_planner.representation.joint_graph import JointPlanner
from m4_global_planner.representation.walk_validator import WalkStateValidator
from m4_global_planner.scenario_loader import (
    _merge_settings, default_scenario_path, get_scenario_parameters, load_scenario_data,
)
from m4_global_planner.search_comparison import dijkstra, verify_heuristic


def methods(settings):
    weights = settings['weighted_astar_weights']
    if not weights or any(not isfinite(w) or w <= 1 for w in weights):
        raise ValueError('Weighted A* weights must be finite and greater than one')
    if len(set(weights)) != len(weights):
        raise ValueError('Duplicate weighted A* weights')
    return [('dijkstra', 0.0), ('astar', 1.0)] + [('weighted_astar', w) for w in weights]


def fingerprint(planner):
    """Hash actual ordered nodes, edges, endpoints and connector primitives."""
    digest = hashlib.sha256()
    for node in planner.graph.nodes.values():
        digest.update(repr(node).encode())
        for edge in planner.graph.edges[node.node_id].values():
            digest.update(repr(edge).encode())
    digest.update(repr(planner.endpoints).encode())
    digest.update(repr(planner.paths).encode())
    return digest.hexdigest()


def prepare(data, case, snapshot):
    settings = data['joint_search_benchmark']
    parameters = _merge_settings(get_scenario_parameters(data, case['scenario']),
                                 case.get('parameter_overrides', {}))
    tuning = _merge_settings(data['joint_planner'], case.get('planner_overrides', {}))
    parameters['walk_terrain_cost_mode'] = tuning['walk_terrain_cost_mode']
    world = data['scenarios'][case['scenario']]
    request = settings['requests'].get(case['scenario'], world)
    endpoints = {}
    for i, key in enumerate(('start', 'goal')):
        record = request[key]
        mode = MotionMode(record.get('mode', 'walk'))
        yaw = radians(record['heading_deg']) if mode == MotionMode.WALK else None
        endpoints[key] = Node(i, *record['position'], mode, yaw)
    walk, fly = WalkStateValidator(parameters), FlyStateValidator(parameters)
    builder = MandatoryAnchorBuilder(parameters, walk, fly,
                                     TransitionValidator(parameters, walk, fly))
    start = perf_counter()
    planner = JointPlanner(parameters, tuning, DummyPlanningMap(world), builder, endpoints,
                           settings['seed'])
    construction_ms = (perf_counter() - start) * 1000
    construction_peak = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024
    start = perf_counter()
    source, goal = (planner.endpoints[key] for key in ('start', 'goal'))
    verification = verify_heuristic(planner.graph, goal, planner.scale,
                                    settings['heuristic_tolerance'])
    reference = dijkstra(planner.graph, source, goal)
    if reference['success']:
        planner.validate_route(reference['path'], source, goal, reference['total_cost'])
    status = 'success' if reference['success'] else 'no_path'
    if status != case['expected']:
        raise ValueError(f'Graph outcome {status} differs from expected {case["expected"]}')
    audit_ms = (perf_counter() - start) * 1000
    payload = pickle.dumps(planner)
    snapshot.write_bytes(payload)
    return {'construction_ms': construction_ms, 'preparation_audit_ms': audit_ms,
            'construction_peak_rss_mib': construction_peak, 'heuristic': verification,
            'scale': planner.scale, 'nodes': len(planner.graph.nodes),
            'edges': sum(map(len, planner.graph.edges.values())),
            'reference_cost': reference['total_cost'] if reference['success'] else None,
            'reference_success': reference['success'], 'graph_sha256': fingerprint(planner),
            'snapshot_sha256': hashlib.sha256(payload).hexdigest(),
            'parameters': parameters, 'planner_settings': tuning}


def run_case(job):
    payload = Path(job['snapshot']).read_bytes()
    if hashlib.sha256(payload).hexdigest() != job['snapshot_sha256']:
        raise ValueError('Saved graph snapshot changed')
    # Only load snapshots produced locally by this runner.
    planner = pickle.loads(payload)
    del payload
    if fingerprint(planner) != job['graph_sha256']:
        raise ValueError('Graph identity changed')
    source, goal = (planner.endpoints[key] for key in ('start', 'goal'))

    def search(graph, start, end):
        if job['method'] == 'dijkstra':
            return dijkstra(graph, start, end)
        return astar(graph, start, end, heuristic_scale=planner.scale,
                     heuristic_weight=job['weight'])

    gc.collect()
    baseline = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024
    result = run_with_metrics(search, planner.graph, source, goal)
    start = perf_counter()
    if result['success'] != job['reference_success']:
        raise ValueError('Search success differs from Dijkstra reference')
    gap, gap_pct = None, None
    if result['success']:
        planner.validate_route(result['path'], source, goal, result['total_cost'])
        reference = job['reference_cost']
        gap = result['total_cost'] - reference
        if gap < -job['cost_tolerance']:
            raise ValueError('Search cost is below the Dijkstra reference')
        if job['method'] != 'weighted_astar' and not isclose(
                result['total_cost'], reference, rel_tol=1e-10, abs_tol=job['cost_tolerance']):
            raise ValueError('Exact search differs from Dijkstra cost')
        gap_pct = 100 * gap / reference if reference else (0.0 if gap == 0 else None)
    if fingerprint(planner) != job['graph_sha256']:
        raise ValueError('Search mutated the graph')
    audit_ms = (perf_counter() - start) * 1000
    # Separate replay: allocation instrumentation never affects reported search time.
    gc.collect()
    tracemalloc.start()
    replay = search(planner.graph, source, goal)
    _, peak = tracemalloc.get_traced_memory()
    tracemalloc.stop()
    if replay != {key: result[key] for key in replay}:
        raise ValueError('Memory replay differs from timed search')
    if replay['success']:
        planner.validate_route(replay['path'], source, goal, replay['total_cost'])
    if fingerprint(planner) != job['graph_sha256']:
        raise ValueError('Memory replay mutated the graph')
    return {**job, 'success': result['success'], 'status': 'success' if result['success'] else
            'no_path', 'cost': result['total_cost'] if result['success'] else None,
            'cost_gap': gap, 'cost_gap_pct': gap_pct, 'path': result['path'],
            'nodes_expanded': result['nodes_expanded'], 'nodes_explored': result['nodes_explored'],
            'nodes_generated': result['nodes_generated'], 'search_ms': result['planning_time_ms'],
            'search_cpu_ms': result['cpu_time_ms'], 'audit_ms': audit_ms,
            'peak_rss_mib': result['peak_ram_mb'], 'baseline_peak_rss_mib': baseline,
            'peak_rss_growth_mib': max(0.0, result['peak_ram_mb'] - baseline),
            'search_python_peak_mib': peak / 1024**2, 'validated': True}


def summary(rows, output):
    lines = ['# Joint graph search comparison', '',
             'Serial fresh-process repeats per method; one saved graph per configuration.',
             'All methods use the same configuration mix. Gaps are matched to Dijkstra per graph.',
             '', '| Method | Valid outcomes | Successful routes | Median cost | Max gap % | '
             'Median explored / pops | Search median / P95 ms | Python peak / RSS MiB |',
             '|---|---:|---:|---:|---:|---:|---:|---:|']
    for method, weight in dict.fromkeys((r['method'], r['weight']) for r in rows):
        runs = [r for r in rows if r['method'] == method and r['weight'] == weight]
        valid = [r for r in runs if r.get('validated')]
        successes = [r for r in valid if r['success']]
        if not valid:
            continue

        def median(field):
            return statistics.median(r[field] for r in valid)

        times = sorted(r['search_ms'] for r in valid)
        p95 = times[ceil(.95 * len(times)) - 1]
        label = method if method != 'weighted_astar' else f'wA* {weight:g}'
        cost = f'{statistics.median(r["cost"] for r in successes):.6f}' if successes else '—'
        gaps = [r['cost_gap_pct'] for r in successes if r['cost_gap_pct'] is not None]
        gap = f'{max(gaps):.3f}' if gaps else '—'
        lines.append(
            f'| {label} | {len(valid)}/{len(runs)} | {len(successes)}/{len(runs)} | '
            f'{cost} | {gap} | {median("nodes_explored"):.0f} / '
            f'{median("nodes_expanded"):.0f} | {median("search_ms"):.3f} / {p95:.3f} | '
            f'{median("search_python_peak_mib"):.3f} / {median("peak_rss_mib"):.1f} |')
    configurations = {r['case']['name']: r for r in rows if r.get('validated')}
    if configurations:
        builds = [r['construction_ms'] for r in configurations.values()]
        lines += ['', f'{len(configurations)} configurations; graph construction median '
                  f'{statistics.median(builds):.1f} ms, '
                  f'range {min(builds):.1f}–{max(builds):.1f} ms.',
                  'Configurations: ' + ', '.join(configurations) + '.']
    lines += ['', 'Cost median pools successful requests with the same configuration mix;',
              'Raw rows provide per-graph costs and paired timing/expansion comparisons.',
              'Build time excludes reference search, audits, snapshot I/O and worker startup.',
              'Search statistics include no-path attempts and exclude validation, I/O and replay.',
              'Explored counts unique popped states; pops include re-expansions. Allocation peak',
              'is a separate identical tracemalloc replay. RSS is the timed-worker high-water',
              'through search, including imports and loading; its baseline/growth is in raw rows.',
              'No-path costs/gaps are undefined. All returned paths and totals are validated.',
              'A* lower bound and consistency are checked over every edge. Weighted gaps are',
              'measurements only; no theoretical weighted bound is claimed.', '']
    output.write_text('\n'.join(lines))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--worker', action='store_true')
    parser.add_argument('--scenarios', type=Path, default=default_scenario_path())
    parser.add_argument('--output', type=Path, default=Path('joint_search_results'))
    parser.add_argument('--case', type=int)
    args = parser.parse_args()
    if args.worker:
        job = json.load(sys.stdin)
        result = (prepare(load_scenario_data(job['scenarios']), job['case'], Path(job['snapshot']))
                  if job.get('prepare') else run_case(job))
        print(json.dumps(result, allow_nan=False))
        return
    data = load_scenario_data(args.scenarios)
    settings = data['joint_search_benchmark']
    candidates = methods(settings)
    for field in ('repeats', 'timeout_seconds'):
        if type(settings[field]) is not int or settings[field] <= 0:
            raise ValueError(f'{field} must be a positive integer')
    args.output.mkdir(parents=True, exist_ok=True)
    files = list(Path(__file__).parent.rglob('*.py')) + [args.scenarios]
    hashes = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    signature = hashlib.sha256(json.dumps(hashes, sort_keys=True).encode()).hexdigest()
    manifest = args.output / 'manifest.json'
    if manifest.exists() and json.loads(manifest.read_text())['signature'] != signature:
        raise RuntimeError('Source/config changed; choose a new output directory')
    manifest.write_text(json.dumps({
        'signature': signature, 'sources': hashes, 'settings': settings,
        'python': sys.version, 'platform': platform.platform(),
    }, indent=2))
    (args.output / 'scenarios.yaml').write_bytes(args.scenarios.read_bytes())
    raw = args.output / 'raw.jsonl'
    rows = [json.loads(line) for line in raw.read_text().splitlines()] if raw.exists() else []
    done = {r['run_id'] for r in rows if r.get('validated')}

    def worker(job):
        completed = subprocess.run(
            [sys.executable, '-m', 'm4_global_planner.joint_search_benchmark', '--worker'],
            input=json.dumps(job), text=True, capture_output=True, check=True,
            timeout=settings['timeout_seconds'],
        )
        return json.loads(completed.stdout)

    for index, case in enumerate(settings['cases']):
        if args.case is not None and args.case != index:
            continue
        snapshot = (args.output / f'graph_{index}.pickle').resolve()
        metadata = args.output / f'graph_{index}.json'
        if metadata.exists():
            shared = json.loads(metadata.read_text())
        else:
            shared = worker({'prepare': True, 'case': case, 'snapshot': str(snapshot),
                             'scenarios': str(args.scenarios.resolve())})
            metadata.write_text(json.dumps(shared, indent=2))
        for repeat in range(settings['repeats']):
            # Rotate method order between repeats to reduce ordering bias.
            ordered = candidates[repeat % len(candidates):] + candidates[:repeat % len(candidates)]
            for method, weight in ordered:
                run_id = f'{index}:{method}:{weight}:{repeat}'
                if run_id in done:
                    continue
                job = dict(run_id=run_id, case=case, method=method, weight=weight, repeat=repeat,
                           snapshot=str(snapshot), cost_tolerance=settings['cost_tolerance'],
                           **shared)
                try:
                    row = worker(job)
                except (subprocess.SubprocessError, ValueError) as error:
                    row = {**job, 'status': 'worker_error', 'success': False, 'validated': False,
                           'error': str(error),
                           'stderr': getattr(error, 'stderr', None)}
                with raw.open('a') as file:
                    file.write(json.dumps(row, allow_nan=False) + '\n')
                rows.append(row)
                summary(rows, args.output / 'comparison.md')
                if not row.get('validated'):
                    raise RuntimeError(f'Invalid benchmark run: {row}')
        print(case['name'], shared['nodes'], shared['edges'], flush=True)


if __name__ == '__main__':
    main()
