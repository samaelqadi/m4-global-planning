> Historical evaluation record. The current report is [GLOBAL_SEARCH.md](GLOBAL_SEARCH.md). The independent review added four zero-cost-update regressions and a full-reset fallback in evaluation-only LPA* and D* Lite (54 tests at that checkpoint); the final pre-commit audit added route-audit and large-finite-cost regressions, bringing the current suite to 59. Timing tables below predate the zero-cost correction and are not post-correction measurements.

# Global Planner Algorithm Evaluation

## Purpose and responsibility boundary

The planner searches a complete supplied weighted directed graph and returns a route. It validates the graph snapshot, selects an algorithm, verifies the returned route, and reports the supplied edge-cost sum. It does not generate nodes, generate movement or WALK/FLY transition edges, calculate slope/roughness/clearance/energy/traversal costs, or decide physical feasibility. Those are responsibilities of the upstream planning-map and graph-generation subsystem. Every candidate below received the same graph, start, goal, and edge weights within each case.

The final producer message, cost units, coordinate frame, graph completeness, start/goal attachment, and controller interface remain unintegrated. Thus these results compare algorithms on repeatable synthetic supplied graphs; they do not certify robot performance or physical safety.

## Algorithms evaluated

The additional algorithms live in `src/m4_global_planner/m4_global_planner/benchmark_algorithms.py` for evaluation and tests. They are not selectable through the production `GlobalPlanner` API.

| Algorithm | Implementation and reason for evaluation | Current production role |
|---|---|---|
| Dijkstra | Existing exact, geometry-independent search; used as the per-snapshot optimal-cost reference. | Selectable exact fallback/reference. |
| A*, `h=0` | Existing A* measured without guidance as a Dijkstra-equivalent control. | Benchmark control; production uses zero guidance when safe guidance is unavailable. |
| A*, safe heuristic | Existing production A* with an admissible graph-derived Euclidean scale. | Production default for this prototype. |
| Weighted A* | Evaluation implementation tested at `w=1.0`, `1.25`, `1.5`, and `2.0` to measure speed/quality tradeoffs. | Not exposed; nonoptimal behavior needs explicit product approval. |
| Bidirectional Dijkstra | Evaluation-only exact forward/reverse search for directed graphs. | Not exposed; reverse-index construction lost on measured call time. |
| Bidirectional A* | Evaluation-only exact balanced-potential search using safe guidance where available. | Not exposed; preprocessing lost on measured call time. |
| LPA* | Evaluation-only incremental forward repair for repeated edge-cost changes at fixed start/goal. | Not exposed; slower end-to-end on the tested repeated-update workload. |
| D* Lite | Evaluation-only incremental backward repair; supports moving the start, though the measured sequence held it fixed. | Not exposed; slower end-to-end on the tested repeated-update workload. |

## Metrics

| Metric group | Fields actually saved | Scope and limits |
|---|---|---|
| Static correctness and quality | `success`, `path_valid`, `path_state_count`, `path_cost`, `dijkstra_reference_cost`, `optimality_ratio`, `suboptimality_pct`. | Route endpoints, every supplied directed edge, recomputed route cost and no-path status are validated before results are accepted. |
| Static search work | `expanded_nodes`, `generated_nodes`, `processed_nodes`, `edge_evaluations`. | These counters are algorithm-specific; incremental edge-evaluation counters do not include all repair/path-recovery scans. There is no separate relaxation count. |
| Static timing | Median, mean, sample standard deviation, minimum and maximum; each raw per-run runtime. | Ten timed runs per algorithm/configuration/scenario using `time.perf_counter_ns()`. |
| Fixture and request | Algorithm, weight, heuristic, purpose, `|V|`, `|E|`, start, goal, WALK and FLY node counts. | Same immutable weighted graph, endpoints and edge weights are used for every method in a scenario. |
| Memory | `peak_python_memory_bytes` from `tracemalloc`. | Recorded only for the eight small-grid configurations; graph input is excluded. No RSS or dynamic-memory measurement. |
| Dynamic update and timing | Update number/type/edge, initialization, initial plan, update-processing, replan, total update-to-route; median, mean, sample standard deviation, min and max of total time. | Five runs of the full four-update sequence. Update processing and replan distributions are retained in raw runs; aggregate subphase fields are medians. |
| Dynamic correctness and work | Success, route validity, path state count, replanned and Dijkstra costs, ratio, suboptimality, expanded/generated nodes, edge evaluations, fresh-A* comparisons, speedup and expansion reduction. | Generated nodes and edge evaluations are retained per update in raw rows; their meanings are method-specific. |
| Unavailable measures | RSS, physical validity, map/graph construction time, a distinct relaxation count, dynamic memory, target-hardware latency. | Not inferred from Python allocation tracing, graph size or search counters. Scalability is a cross-size comparison, not a directly stored metric. |

The detailed numerical counter/timing tables and dynamic comparison are in the current full report, `GLOBAL_SEARCH.md`. The complete machine-readable values remain authoritative in the summary and raw CSVs listed below; the short tables in this historical evaluation document do not replace them.

## Methodology

- Static tests use ten measured runs per algorithm/scenario, no unmeasured warm-up, and `time.perf_counter_ns()`. Median is the principal timing statistic; mean, sample standard deviation, min and max are also saved.
- Dynamic tests use five independent repetitions of the whole four-update sequence. Each candidate starts from an identical graph copy; the update definitions are fixed before candidate execution.
- A Dijkstra result on the current graph is the reference. Every result is automatically checked for endpoint correctness, edge membership, recomputed path-cost agreement, and correct reachability. Exact methods must also match Dijkstra cost within `1e-9`; invalid output aborts the campaign.
- Snapshot/fixture construction, reference searches, route validation, CSV writing and plotting are outside static timing. The ordinary searches time the call. Incremental static timings include construction of the algorithm's per-graph state and its initial plan; dynamic initialization is reported separately. Heuristic scale is prepared before the ordinary A*/weighted A* search timer, consistent with graph-snapshot preparation.
- Memory tracing is a separate replay and does not contaminate timed runs. Plot generation is optional and was skipped because matplotlib was not installed in the existing environment.
- The grids use eight-connected geometric edges; the edge weights are supplied Euclidean step lengths except explicitly expensive crossings. All algorithm variants consume those same immutable fixture costs.

Reproduce with:

```powershell
.\.venv\Scripts\python.exe -m pytest -q
.\.venv\Scripts\python.exe benchmarks\run_benchmarks.py
```

## Scenarios and graph sizes

| Scenario | Purpose | `|V|` | `|E|` | Start → goal | WALK / FLY |
|---|---|---:|---:|---|---:|
| `small_static_grid` | Small static reference grid | 144 | 1,012 | 0 → 143 | 144 / 0 |
| `medium_static_grid` | Medium scalability grid | 1,600 | 12,324 | 0 → 1,599 | 1,600 / 0 |
| `large_static_grid` | Large scalability grid | 10,000 | 78,804 | 0 → 9,999 | 10,000 / 0 |
| `very_large_static_grid` | Very large scalability grid | 40,000 | 317,604 | 0 → 39,999 | 40,000 / 0 |
| `walk_only_grid` | WALK-only route | 900 | 6,844 | 0 → 899 | 900 / 0 |
| `multimodal_supplied_graph` | Existing WALK/FLY supplied demo graph | 5 | 5 | 0 → 4 | 3 / 2 |
| `weighted_astar_tradeoff` | Long optimal corridor vs. short attractive suboptimal route | 1,004 | 1,004 | 0 → 1,003 | 1,004 / 0 |
| `unreachable_goal` | Goal disconnected by a complete vertical cut | 1,600 | 12,088 | 0 → 1,599 | 1,600 / 0 |
| `narrow_constrained_route` | Long narrow corridor with dead-end branches | 599 | 1,196 | 0 → 299 | 599 / 0 |
| `multiple_alternative_costs` | Several alternatives and localized expensive crossings | 3,600 | 28,084 | 0 → 3,599 | 3,600 / 0 |
| `dynamic_two_corridor_graph` | Short primary and longer alternate route under sequential changes | 901 | 1,802 | 0 → 300 | 901 / 0 |

The exact fixture descriptions and dynamic edge sequence are also in `benchmark_results/benchmark_manifest.json`.

## Static scalability results

Each cell is `median ms / expanded nodes`; all successful routes matched the Dijkstra reference cost. These seven methods were compared on the same four grids.

| Algorithm | 144 nodes | 1,600 nodes | 10,000 nodes | 40,000 nodes |
|---|---:|---:|---:|---:|
| Dijkstra | 0.14445 / 144 | 1.85945 / 1,600 | 16.02930 / 10,000 | 68.33525 / 40,000 |
| A*, `h=0` | 0.17485 / 144 | 2.53875 / 1,600 | 18.59490 / 10,000 | 88.85675 / 40,000 |
| A*, safe heuristic | 0.03200 / 12 | 0.12300 / 40 | 0.32955 / 100 | 0.65070 / 200 |
| Bidirectional Dijkstra | 0.28825 / 100 | 4.62225 / 1,130 | 64.85775 / 7,070 | 279.40925 / 28,282 |
| Bidirectional A* | 0.40800 / 11 | 5.46945 / 39 | 60.94965 / 99 | 291.45355 / 199 |
| LPA* (fresh initial state) | 0.59660 / 12 | 5.77685 / 40 | 36.10150 / 100 | 147.70050 / 200 |
| D* Lite (fresh initial state) | 0.82150 / 12 | 5.17405 / 40 | 34.87650 / 100 | 144.97350 / 200 |

The complete static table, including cost, counters, variability and the other six scenarios, is `benchmark_results/benchmark_static.csv`. All ten raw timing repetitions per row are in `benchmark_results/benchmark_static_runs.csv`. The separate `weighted_astar_w1` row is retained in that complete table for the requested weight-1 comparison.

All seven named exact-search methods found valid paths and matched the reference on reachable fixtures; the separately retained `weighted_astar_w1` configuration did as well. All configurations correctly reported no path on the disconnected fixture. Bidirectional methods include per-query reverse-adjacency/potential setup in their measured call time. LPA*/D* Lite static results include their initial per-graph state construction, so these are not presented as a cached multi-query comparison.

## Weighted A* results

The crafted tradeoff graph provides an explicit, repeatable quality/speed boundary. Full results for every weight on every scenario, with means, standard deviations, minima, maxima and counters, are in `benchmark_results/benchmark_weighted_astar.csv`; raw samples are in `benchmark_results/benchmark_weighted_astar_runs.csv`.

| Weight | Median ms | Mean ± SD ms | Min–max ms | Expanded nodes | Route cost | Dijkstra cost | Ratio | Suboptimality |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 1.00 | 0.68555 | 0.68228 ± 0.07033 | 0.6022–0.7648 | 1,003 | 12 | 12 | 1.000 | 0% |
| 1.25 | 0.62685 | 0.62496 ± 0.01077 | 0.6129–0.6471 | 1,003 | 12 | 12 | 1.000 | 0% |
| 1.50 | 0.00280 | 0.00328 ± 0.00132 | 0.0025–0.0068 | 3 | 15 | 12 | 1.250 | 25% |
| 2.00 | 0.00260 | 0.00277 ± 0.00053 | 0.0024–0.0042 | 3 | 15 | 12 | 1.250 | 25% |

On the remaining reachable fixtures all tested weights returned the optimum; this does not establish a general quality guarantee beyond weighted A*'s algorithmic bound. On the crafted fixture, weights 1.5 and 2 reduced expansions by 99.7% (1,003 to 3) and the median call time by about 99.6%, at 25% greater path cost. Weight 1.25 gave no expansion reduction or measured time advantage. Therefore no weight is selected automatically; any nonoptimal mode must be an explicit, application-approved tradeoff.

## Heuristic comparison

`h=0` is exact and behaves as Dijkstra; the safe heuristic remains admissible because it is scaled by the minimum supplied cost per positive geometric edge length. In the scalable grids:

| Nodes | `h=0`: median / expanded | Safe heuristic: median / expanded | Expansion reduction | Median runtime reduction |
|---:|---:|---:|---:|---:|
| 144 | 0.17485 ms / 144 | 0.03200 ms / 12 | 91.67% | 81.7% |
| 1,600 | 2.53875 ms / 1,600 | 0.12300 ms / 40 | 97.50% | 95.2% |
| 10,000 | 18.59490 ms / 10,000 | 0.32955 ms / 100 | 99.00% | 98.2% |
| 40,000 | 88.85675 ms / 40,000 | 0.65070 ms / 200 | 99.50% | 99.3% |

In the WALK+FLY five-node graph, safe A* expanded 4 nodes and `h=0` expanded 5; the route cost was 3 for both. The zero-heuristic route is the exact-cost confirmation. Geometry and cost in these synthetic fixtures are deliberately aligned; a producer with absent/incompatible geometry will yield `h=0` or a weak heuristic.

## Dynamic replanning results

Five runs per full sequence: block a primary-route edge, increase an alternate edge cost, reopen the first edge, then block another primary-route edge. Each edge change is applied in both directions. The reported speedup is fresh A* total update-to-route median divided by the incremental candidate's total median; below `1.0` means the incremental candidate was slower.

| Update | Type | Algorithm | Update ms | Replan ms | Total median ms | Fresh A* speedup | Expanded vs fresh | Replanned / reference cost |
|---:|---|---|---:|---:|---:|---:|---:|---:|
| 1 | Block | Fresh A* | 0.0025 | 0.6644 | 0.6662 | 1.000 | 752 / 752 | 601 / 601 |
| 1 | Block | LPA* | 0.0122 | 2.1781 | 2.1888 | 0.304 | 467 / 752 | 601 / 601 |
| 1 | Block | D* Lite | 0.0102 | 2.0039 | 2.0136 | 0.331 | 468 / 752 | 601 / 601 |
| 2 | Increase cost | Fresh A* | 0.0083 | 0.6271 | 0.6357 | 1.000 | 752 / 752 | 612 / 612 |
| 2 | Increase cost | LPA* | 0.0140 | 3.9022 | 3.9220 | 0.162 | 802 / 752 | 612 / 612 |
| 2 | Increase cost | D* Lite | 0.0127 | 1.9059 | 1.9188 | 0.331 | 402 / 752 | 612 / 612 |
| 3 | Reopen | Fresh A* | 0.0054 | 0.5124 | 0.5171 | 1.000 | 573 / 573 | 300 / 300 |
| 3 | Reopen | LPA* | 0.0108 | 0.7368 | 0.7467 | 0.693 | 150 / 573 | 300 / 300 |
| 3 | Reopen | D* Lite | 0.0106 | 0.6105 | 0.6211 | 0.833 | 151 / 573 | 300 / 300 |
| 4 | Block | Fresh A* | 0.0018 | 0.6284 | 0.6303 | 1.000 | 753 / 753 | 612 / 612 |
| 4 | Block | LPA* | 0.0072 | 1.0417 | 1.0508 | 0.600 | 150 / 753 | 612 / 612 |
| 4 | Block | D* Lite | 0.0072 | 0.9534 | 0.9602 | 0.656 | 153 / 753 | 612 / 612 |

The complete dynamic means, standard deviations, min/max, initial setup, every individual run and edge-evaluation counts are in `benchmark_results/benchmark_dynamic.csv` and `benchmark_results/benchmark_dynamic_runs.csv`. All three planners matched the updated Dijkstra cost after every change. Incremental search reduced expansions on most updates, but its setup and repair overhead made it slower end-to-end than fresh A* on all four measured updates; LPA* expanded 6.65% more nodes than fresh A* on update 2.

## Scalability and selection

On the 40,000-node geometric grid, safe A* returned the Dijkstra cost 281.4285 while reducing median expansions from 40,000 to 200 (99.5%) and median runtime from 68.33525 ms to 0.65070 ms (about 99.05% lower, or 105× faster). Dijkstra remains the geometry-independent reference. `h=0` A* was slower than Dijkstra at every tested grid size.

Bidirectional Dijkstra reduced expansions versus one-direction Dijkstra on these grid paths, but its per-call reverse-index construction produced substantially higher medians. Bidirectional A* cut expansions further, yet per-query preprocessing still made it much slower than ordinary safe A*. The implemented versions are not selected for this API without an amortized-index workload demonstrating a benefit.

LPA* and D* Lite were slower for initial plans and for every measured end-to-end update on the two-corridor dynamic case. Their fewer repair expansions did not offset initialization, queue/state maintenance and repair costs in this Python prototype. Fresh safe A* is therefore the measured choice for both initial planning and full-snapshot replanning on these cases. The result does not claim incremental search is universally slower: larger maps, denser updates, moving starts, many queries per snapshot or optimized data structures could change it.

**Decision for the current prototype, subject to producer integration:**

- **Initial planning:** safe A* when its graph-derived admissible heuristic is available; it is the exact measured winner on the geometric scale fixtures. This is now the `GlobalPlanner` default.
- **Dynamic replanning:** fresh safe A* for the current full-snapshot API and measured workload. Keep D* Lite/LPA* out of the production API until a representative update campaign shows an end-to-end win.
- **Reference/validation:** Dijkstra.
- **Optional fast/suboptimal mode:** Weighted A* is not enabled by default. `w=1.5` or `2` is only a candidate for an explicitly accepted cost-quality tradeoff; the measured constructed case returned a 25% more expensive route.

## Limitations and future work

- All graphs are synthetic; the supplied multimodal fixture is a five-node demo, not a producer graph. No terrain, clearance or physical feasibility is represented.
- The heuristic experiment uses consistent Euclidean geometry and edge costs. Real costs may be weakly correlated or unrelated to distance, making A* behave like `h=0`.
- Timing is a single Python/Windows environment, with no hardware normalization, target deadline or production load. Small absolute times are especially sensitive to scheduler noise. Medians and distribution statistics are included; do not extrapolate them as real-time guarantees.
- Static incremental timings include per-instance preprocessing; dynamic results include separately reported initialization. The update campaign uses one fixed start and one fixed graph family.
- Peak Python allocations were measured only on the small case; no large-process RSS or memory limit is claimed.
- Matplotlib was absent, so no plots were generated. CSV and JSON evidence are the authoritative outputs.
- Future work: rerun with real producer snapshots, actual graph revisions and update rates; verify cost units/coordinate frame; add producer/controller integration; benchmark moving starts and patch-vs-snapshot updates; and set target hardware latency and memory budgets before deployment.

## Result files

- `benchmark_results/benchmark_static.csv` — complete static summary (80 rows).
- `benchmark_results/benchmark_static_runs.csv` — all 800 static timed samples.
- `benchmark_results/benchmark_weighted_astar.csv` — all weights and scenarios (40 rows).
- `benchmark_results/benchmark_weighted_astar_runs.csv` — all 400 weighted timed samples.
- `benchmark_results/benchmark_dynamic.csv` — dynamic summary (12 rows).
- `benchmark_results/benchmark_dynamic_runs.csv` — all 60 update observations.
- `benchmark_results/benchmark_manifest.json` — fixture, environment and validation metadata.
