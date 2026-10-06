"""Fresh-process WALK runs with saved seeds, routes, and measurements."""

import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import platform
import resource
import statistics
import subprocess
import sys

from m4_global_planner.dummy_map import DummyPlanningMap
from m4_global_planner.scenario_loader import (
    default_scenario_path, get_scenario_parameters, load_scenario_data,
)
from m4_global_planner.walk_comparison import APPROACHES, plan, pose


SCENARIOS = default_scenario_path()


def run_case(job):
    data = load_scenario_data(job['scenarios'])
    case = job['case']
    name = case['scenario']
    scenario = data['scenarios'][name]
    settings = data['walk_benchmark']
    parameters = get_scenario_parameters(data, name)
    for key in ('reverse_allowed', 'pivot_allowed'):
        if key in case:
            parameters[key] = case[key]
    if 'resolution' in case:
        parameters['spatial_resolution'] = case['resolution']
    parameters['walk_terrain_cost_mode'] = settings['terrain_cost_mode']
    request = settings['requests'].get(name, scenario)
    start = pose(request.get('start', request.get('from')))
    goal = pose(request.get('goal', request.get('to')))
    baseline = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024
    result = plan(job['approach'], parameters, settings, DummyPlanningMap(scenario), start,
                  goal, job['seed'])
    peak = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024
    result['route'] = [asdict(state) for state in result['route']]
    result.update(peak_rss_mib=peak, peak_rss_growth_mib=max(0.0, peak - baseline),
                  baseline_peak_rss_mib=baseline, correct=result['status'] == case['expected'])
    return {**job, **result}


def summary(rows, output):
    lines = ['# WALK comparison', '',
             'Fresh-process runs revalidate routes. Times include planning and construction,',
             'and route audit.', '',
             '| Approach | Correct outcomes | Valid routes / expected routes | Mean route cost¹ |',
             'Median / P95 total ms² | Median peak / growth MiB³ | Median nodes / edges⁴ |',
             '|---|---:|---:|---:|---:|---:|---:|']
    for approach in APPROACHES:
        group = [row for row in rows if row['approach'] == approach]
        successful = [row for row in group if row.get('success')]
        attempts = [row for row in group if row['case']['expected'] == 'success']
        measured = [
            row for row in group
            if 'total_ms' in row
            and row['case']['expected'] == 'success'
            and row['case']['scenario'] != 'S01_start_equals_goal'
        ]
        times = sorted(row['total_ms'] for row in measured)

        def median(field):
            return statistics.median(row[field] for row in measured) if measured else 0

        mean_cost = statistics.mean(row['cost'] for row in successful) if successful else 0
        p95 = times[min(len(times) - 1, int(0.95 * len(times)))] if times else 0
        lines.append(f"| {approach} | {sum(row.get('correct',
                                                   False) for row in group)}/{len(group)} | "
                     f'{len(successful)}/{len(attempts)} | {mean_cost:.3f} | '
                     f"{median('total_ms'):.2f} / {p95:.2f} | "
                     f"{median('peak_rss_mib'):.2f} / {median('peak_rss_growth_mib'):.2f} | "
                     f"{median('nodes'):.0f} / {median('edges'):.0f} |")
    lines += [
        '',
        '¹ Success-only costs have different mixes if an approach fails; use per-case costs.',
        '² Expected-success cases include failed attempts; failures and timeouts are recorded.',
        '³ Each run uses a fresh Linux worker. Peak RSS includes imports; growth is measured from',
        'pre-plan high-water mark, not live allocation size.',
        '⁴ Lattice: explicit pose graph; grid/roadmap: spatial graph retaining heading in search.',
        'Hybrid: discovered bins and yielded motion candidates. These counts are not equivalent.',
        '',
    ]
    output.write_text('\n'.join(lines))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--worker', action='store_true')
    parser.add_argument('--scenarios', type=Path, default=SCENARIOS)
    parser.add_argument('--output', type=Path, default=Path('walk_results'))
    parser.add_argument('--case', type=int)
    args = parser.parse_args()
    if args.worker:
        job = json.load(sys.stdin)
        try:
            print(json.dumps(run_case(job), allow_nan=False))
        except Exception as error:
            print(json.dumps({**job, 'status': 'error', 'correct': False,
                              'success': False, 'error': f'{type(error).__name__}: {error}'}))
        return
    data = load_scenario_data(args.scenarios)
    settings = data['walk_benchmark']
    args.output.mkdir(parents=True, exist_ok=True)
    source_files = list(Path(__file__).parent.rglob('*.py')) + [args.scenarios]
    fingerprints = {
        str(path): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in source_files
    }
    signature = hashlib.sha256(json.dumps(fingerprints, sort_keys=True).encode()).hexdigest()
    manifest = args.output / 'manifest.json'
    if manifest.exists() and json.loads(manifest.read_text())['signature'] != signature:
        raise RuntimeError('Source/config changed; choose a new output directory')
    manifest.write_text(json.dumps({'signature': signature, 'sources': fingerprints,
                                    'utc': datetime.now(timezone.utc).isoformat(),
                                    'python': sys.version, 'platform': platform.platform(),
                                    'cpu_count': os.cpu_count(), 'settings': settings}, indent=2))
    raw = args.output / 'raw.jsonl'
    rows = [json.loads(line) for line in raw.read_text().splitlines()] if raw.exists() else []
    finished = {row['run_id'] for row in rows}
    for index, case in enumerate(settings['cases']):
        if args.case is not None and index != args.case:
            continue
        for approach in APPROACHES:
            for repeat in range(settings['repeats']):
                seed = settings['roadmap_seeds'][repeat % len(settings['roadmap_seeds'])]
                run_id = f'{index}:{approach}:{repeat}'
                if run_id in finished:
                    continue
                job = {'run_id': run_id, 'case_index': index, 'case': case,
                       'approach': approach, 'repeat': repeat, 'seed': seed,
                       'scenarios': str(args.scenarios.resolve())}
                try:
                    completed = subprocess.run(
                        [sys.executable, '-m', 'm4_global_planner.walk_benchmark', '--worker'],
                        input=json.dumps(job), text=True, capture_output=True,
                        timeout=settings['timeout_seconds'], check=True,
                    )
                    row = json.loads(completed.stdout)
                except subprocess.TimeoutExpired:
                    row = {**job, 'status': 'timeout', 'success': False, 'correct': False}
                except Exception as error:
                    row = {**job, 'status': 'worker_error', 'success': False, 'correct': False,
                           'error': str(error)}
                with raw.open('a') as file:
                    file.write(json.dumps(row, allow_nan=False) + '\n')
                rows.append(row)
                summary(rows, args.output / 'comparison.md')
        print(f"Saved case {index + 1}/{len(settings['cases'])}: {case['scenario']}", flush=True)


if __name__ == '__main__':
    main()
