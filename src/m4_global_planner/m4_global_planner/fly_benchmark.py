"""Checkpointed FLY comparison using saved common anchors and fresh workers."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import pickle
import platform
import resource
import statistics
import subprocess
import sys
from time import perf_counter

from m4_global_planner.dummy_map import DummyPlanningMap
from m4_global_planner.graph import MotionMode, Node
from m4_global_planner.representation.anchors import MandatoryAnchorBuilder
from m4_global_planner.representation.fly_graph import plan
from m4_global_planner.representation.fly_validator import FlyStateValidator, TransitionValidator
from m4_global_planner.representation.walk_validator import WalkStateValidator
from m4_global_planner.scenario_loader import (
    default_scenario_path, get_scenario_parameters, load_scenario_data,
)

SCENARIOS = default_scenario_path()
APPROACHES = ('adaptive_lattice', 'sparse_roadmap')


def setup(data, case):
    parameters = get_scenario_parameters(data, case['scenario'])
    parameters['mandatory_anchors'].update(data['fly_benchmark']['anchor_overrides'])
    walk, fly = WalkStateValidator(parameters), FlyStateValidator(parameters)
    transition = TransitionValidator(parameters, walk, fly)
    builder = MandatoryAnchorBuilder(parameters, walk, fly, transition)
    return DummyPlanningMap(data['scenarios'][case['scenario']]), builder


def prepare(data, case, output):
    start = perf_counter()
    planning_map, builder = setup(data, case)
    endpoints = {str(i): Node(i, *xyz, MotionMode.FLY) for i, xyz in enumerate(case['positions'])}
    anchors = builder.build(planning_map, endpoints)
    elapsed = (perf_counter() - start) * 1000
    payload = pickle.dumps(anchors)
    output.write_bytes(payload)
    return {
        'anchor_ms': elapsed,
        'anchor_sha256': hashlib.sha256(payload).hexdigest(),
        'anchor_peak_rss_mib': resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024,
        'anchor_nodes': len(anchors.fly_nodes),
        'transition_records': len(anchors.transitions),
        'endpoint_order': list(endpoints),
    }


def run_case(job):
    data = load_scenario_data(job['scenarios'])
    settings = data['fly_benchmark']
    planning_map, builder = setup(data, job['case'])
    payload = Path(job['snapshot']).read_bytes()
    if hashlib.sha256(payload).hexdigest() != job['anchor_sha256']:
        raise ValueError('Mandatory snapshot fingerprint changed')
    # Only deserialize snapshots produced locally by this runner.
    anchors = pickle.loads(payload)
    baseline = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024
    result = plan(job['approach'], planning_map, builder, anchors, settings['graph'],
                  settings['sweep'][job['tier']], job['seed'], job['endpoint_order'])
    peak = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024
    result.update(candidate_construction_ms=result['construction_ms'],
                  candidate_total_ms=result['total_ms'],
                  peak_rss_mib=max(peak, job['anchor_peak_rss_mib']), worker_peak_rss_mib=peak,
                  peak_rss_growth_mib=max(0.0, peak-baseline), baseline_peak_rss_mib=baseline,
                  correct=result['status'] == job['case']['expected'])
    result['construction_ms'] += job['anchor_ms']
    result['total_ms'] += job['anchor_ms']
    return {**job, **result}


def summary(rows, path):
    lines = ['# FLY geometric comparison', '',
             'Successful routes use shared snapshots, swept box checks, local union-kNN,',
             'Costs use distance and A*. Three fresh-process runs per tier; seeds 11/29/47.', '',
             '| Candidate / tier | Correct outcomes | Valid routes / expected | Mean cost¹ |',
             'Median build / plan / total ms² | Peak / growth MiB³ | Nodes / directed edges |',
             '|---|---:|---:|---:|---:|---:|---:|']
    for approach in APPROACHES:
        for tier in ('coarse', 'fine'):
            group = [r for r in rows if r['approach'] == approach and r['tier'] == tier]
            measured = [r for r in group if 'total_ms' in r]
            successes = [r for r in group if r.get('success')]

            def median(field):
                return statistics.median(r[field] for r in measured) if measured else 0

            mean = statistics.mean(r['cost'] for r in successes) if successes else 0
            expected = sum(r['case']['expected'] == 'success' for r in group)
            correct = sum(r.get('correct', False) for r in group)
            row = (f'| {approach} / {tier} | {correct}/{len(group)} | '
                   f'{len(successes)}/{expected} | {mean:.3f} | '
                   f'{median("construction_ms"):.1f} / {median("planning_ms"):.1f} / '
                   f'{median("total_ms"):.1f} | {median("peak_rss_mib"):.1f} / '
                   f'{median("peak_rss_growth_mib"):.1f} | '
                   f'{median("nodes"):.0f} / {median("edges"):.0f} |')
            lines.append(row)
    lines += [
        '',
        '¹ Pooled success-only means; use matched per-case raw costs when success sets differ.',
        '² Includes saved preparation and candidate construction/search/audit; excludes startup',
        'and YAML/snapshot I/O. No-path attempts are included; raw rows separate phases.',
        '³ Median Linux process high-water RSS per run, including imports and shared preparation.',
        'Growth is worker increase over pre-build high-water RSS; neither is live allocation.',
        '',
        'Scope: F01–F06 and FLY-only G01/G02/G04/G05 requests. F06 includes a high waypoint.',
        'G03 WALK costs and G06 unknown occupancy are unsupported. Ground dock anchors are',
        'geometric; takeoff/landing and executable multimodal routes are not claimed.',
        '',
    ]
    path.write_text('\n'.join(lines))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--worker', action='store_true')
    parser.add_argument('--scenarios', type=Path, default=SCENARIOS)
    parser.add_argument('--output', type=Path, default=Path('fly_results'))
    parser.add_argument('--case', type=int)
    args = parser.parse_args()
    if args.worker:
        job = json.load(sys.stdin)
        try:
            if job.get('prepare'):
                result = prepare(load_scenario_data(job['scenarios']), job['case'],
                                 Path(job['snapshot']))
            else:
                result = run_case(job)
        except Exception as error:
            result = {**job, 'status': 'error', 'correct': False, 'success': False,
                      'error': f'{type(error).__name__}: {error}'}
        print(json.dumps(result, allow_nan=False))
        return
    data = load_scenario_data(args.scenarios)
    settings = data['fly_benchmark']
    if settings['repeats'] > len(settings['seeds']):
        raise ValueError('Each repeat requires a recorded seed')
    args.output.mkdir(parents=True, exist_ok=True)
    sources = list(Path(__file__).parent.rglob('*.py')) + [args.scenarios]
    fingerprints = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
    signature = hashlib.sha256(json.dumps(fingerprints, sort_keys=True).encode()).hexdigest()
    manifest = args.output / 'manifest.json'
    if manifest.exists() and json.loads(manifest.read_text())['signature'] != signature:
        raise RuntimeError('Source/config changed; choose a new output directory')
    manifest.write_text(json.dumps({
        'signature': signature,
        'sources': fingerprints,
        'settings': settings,
        'python': sys.version,
        'platform': platform.platform(),
        'cpu_count': os.cpu_count(),
    }, indent=2))
    (args.output / 'scenarios.yaml').write_bytes(args.scenarios.read_bytes())
    raw = args.output / 'raw.jsonl'
    rows = [json.loads(line) for line in raw.read_text().splitlines()] if raw.exists() else []
    done = {r['run_id'] for r in rows}

    def worker(job):
        process = subprocess.run([sys.executable, '-m', 'm4_global_planner.fly_benchmark',
                                  '--worker'],
                                 input=json.dumps(job), text=True, capture_output=True,
                                 timeout=settings['timeout_seconds'])
        if process.returncode:
            raise RuntimeError(process.stderr)
        return json.loads(process.stdout)

    for index, case in enumerate(settings['cases']):
        if args.case is not None and args.case != index:
            continue
        snapshot = (args.output / f'anchors_{index}.pickle').resolve()
        metadata = args.output / f'anchors_{index}.json'
        if metadata.exists():
            shared = json.loads(metadata.read_text())
        else:
            shared = worker({
                'prepare': True,
                'case': case,
                'scenarios': str(args.scenarios.resolve()),
                'snapshot': str(snapshot),
            })
            if 'error' in shared:
                raise RuntimeError(shared['error'])
            metadata.write_text(json.dumps(shared, indent=2))
        for tier in settings['sweep']:
            for approach in APPROACHES:
                for repeat, seed in enumerate(settings['seeds'][:settings['repeats']]):
                    run_id = f'{index}:{tier}:{approach}:{repeat}'
                    if run_id in done:
                        continue
                    job = dict(run_id=run_id, case=case, case_index=index, tier=tier,
                               approach=approach,
                               repeat=repeat, seed=seed, scenarios=str(args.scenarios.resolve()),
                               snapshot=str(snapshot), **shared)
                    try:
                        row = worker(job)
                    except subprocess.TimeoutExpired:
                        row = {**job, 'status': 'timeout', 'success': False, 'correct': False}
                    rows.append(row)
                    with raw.open('a') as file:
                        file.write(json.dumps(row, allow_nan=False) + '\n')
                    summary(rows, args.output / 'comparison.md')
                    print(run_id, row['status'], round(row.get('total_ms', 0), 1), flush=True)


if __name__ == '__main__':
    main()
