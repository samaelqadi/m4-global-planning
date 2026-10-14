> Historical evaluation record. The current report is [GLOBAL_SEARCH.md](GLOBAL_SEARCH.md). The independent review added four zero-cost-update regressions and a full-reset fallback in evaluation-only LPA* and D* Lite; the final pre-commit audit added direct route-audit and large-finite-cost regressions. The current suite has 59 tests. Timing tables below predate the zero-cost correction and are not post-correction measurements.

# Global Planner Final Handoff

## 1. Project responsibility

The global planner receives a complete weighted directed graph from the graph-generation task. It validates the graph's data contract, searches the supplied edges, and returns a minimum-cost global route.

The graph producer owns map-derived work: node and edge creation; WALK, FLY and WALK↔FLY transition construction; edge-cost calculation; and collision or physical-feasibility validation. Mapping and SLAM belong to their owners. Local path tracking, motion control and immediate obstacle avoidance belong to the execution owner.

The global planner can identify a mode-changing edge when both endpoint nodes carry mode metadata. This is classification of an already supplied edge, not transition generation or validation.

## 2. Final system pipeline

```text
Mapping / environmental data
        -> planning map and costs
        -> graph-generation task
        -> complete weighted directed graph + version
        -> GLOBAL PLANNER (validate snapshot -> search -> audit route cost)
        -> ordered global route + cost + graph version
        -> local planner / controller
```

The planner does not produce a planning map, attach an unrepresented pose to a node, infer missing edges, or execute the route.

## 3. Input contract

### Current Python interface

```python
planner = GlobalPlanner(search_method='astar', cost_tolerance=1e-9)
planner.set_graph(graph, revision='graph-1')
result = planner.plan(start_id, goal_id)
result = planner.replan(updated_graph, 'graph-2', current_start_id, goal_id)
```

`Graph` is the current in-process input type. The graph version is passed separately to `set_graph` or `replan`; it is not currently a field on `Graph`.

| Input item | Current rule |
|---|---|
| Graph | An explicit `Graph` instance with node and adjacency dictionaries; lazy graph subclasses/builders are rejected. |
| Version/revision | Integer or nonempty string. Each accepted/rejected replacement advances an internal snapshot counter. Version order is not compared. |
| Node ID | Integer key matching `node.node_id`; IDs must be unique in the graph. |
| Node geometry | Optional x/y/z, all present together or all omitted. Supplied coordinates must be finite. Geometry is used only for the current A* lower-bound heuristic. |
| Node metadata | WALK/FLY mode and yaw are optional. A mode is copied as `MotionMode`; invalid supplied modes are rejected. |
| Edge | Directed `source_id`, `target_id`, and finite nonnegative supplied `cost`; source and target must exist and agree with adjacency keys. |
| Edge validity | A supplied edge with an explicit `valid` value other than `True` is rejected. Physical validity itself is the producer's responsibility. |
| Duplicate/self edges | The current `Graph` data type permits at most one edge per ordered node pair and rejects self-edges. |
| Start/goal | Explicit existing node IDs. The planner does not attach poses, insert nodes or guess connectors. |
| Edge metadata | No separate action ID or arbitrary metadata field is currently retained. Extend and agree the message contract before such data is needed. |

This is an internal Python prototype contract, not the producer's final wire or ROS message schema. The producer and controller schema, edge identity/action payload, coordinate frame, and start/goal attachment remain to be agreed.

## 4. Output contract and error behavior

`GlobalPlanner.plan(start_id, goal_id)` returns a dictionary containing:

| Field | Meaning |
|---|---|
| `success` | `True` for a route, `False` when no directed route exists. |
| `path` | Ordered node IDs. Empty on no-path; one node when start equals goal. |
| `nodes`, `edges` | The copied nodes and directed supplied edges in route order. |
| `transition_edges` | Route edges whose known endpoint modes differ; empty if mode metadata is absent. |
| `total_cost` | Sum of supplied edge costs; positive infinity on no-path. |
| `nodes_expanded`, `nodes_explored`, `nodes_generated` | Search counters returned by the selected implementation. |
| `graph_revision`, `snapshot_id` | External revision label and this planner's internal snapshot identity. |
| `search_method`, `search_ms` | Selected algorithm and time around the search call only. |
| `validation_scope` | `supplied_graph_edges_and_costs`; not physical validation. |

No separate `status` string is currently returned. Invalid inputs raise `TypeError` or `ValueError`; calling `plan` without an accepted graph raises `RuntimeError`. Invalid input is not converted into a successful-shaped no-path response.

`is_current(result)` returns whether both revision and internal snapshot ID match the latest accepted graph for that planner instance. The consumer must call it before using a route. The planner does not asynchronously cancel a result, issue a replan request, or stop a controller that is already executing.

## 5. Current code architecture

| Path | Role and important API | Calls / dependencies | Deliberately does not do |
|---|---|---|---|
| `src/m4_global_planner/m4_global_planner/graph.py` | `MotionMode`, immutable `Node` and `Edge`, and explicit directed `Graph` storage/access. Nodes allow optional geometry and mode metadata. | Used by `global_planner.py`, tests and the example. | Does not sample nodes, generate connections, price terrain or validate physical clearance. |
| `src/m4_global_planner/m4_global_planner/astar.py` | `dijkstra`, `astar`, `heuristic`, and `build_path`. Implements exact nonnegative-cost graph search. | Called by `GlobalPlanner.plan`. | Does not create or repair graph edges. A* takes a nonnegative heuristic scale; scale zero gives `h=0`. |
| `src/m4_global_planner/m4_global_planner/benchmark_algorithms.py` | Evaluation-only Weighted A*, bidirectional Dijkstra/A*, LPA*, and D* Lite. | Used by benchmark harness/tests only. | Not exposed as production search choices. |
| `src/m4_global_planner/m4_global_planner/global_planner.py` | `GlobalPlanner.set_graph`, `plan`, `replan`, `is_current`. Copies and validates snapshots, derives safe A* scale when geometry allows, runs search, and audits route endpoints/edges/cost. Safe A* is the default; Dijkstra remains selectable. | Imports shared graph types and exact production search functions. | Does not import mapping, WALK/FLY builders, transition generators, costmaps or robot validators. |
| `src/m4_global_planner/m4_global_planner/__init__.py` | Package marker. | Loaded by Python packaging. | No runtime logic. |
| `src/m4_global_planner/test/test_global_planner.py` | Unit/contract tests and independent repeated-relaxation optimal-cost reference. | Uses synthetic explicit graphs and pytest. | Does not test map generation, simulation or hardware. |
| `src/m4_global_planner/test/test_benchmark_algorithms.py` | Evaluation-only algorithm correctness and incremental-update tests. | Uses deterministic synthetic graphs. | Does not establish real producer performance. |
| `benchmarks/fixtures.py`, `benchmarks/run_benchmarks.py` | Fixed supplied-graph scenarios and repeated benchmark runner with route checks. | Writes CSV/JSON benchmark evidence. | No graph generation or physical cost assignment. |
| `benchmark_results/` | Final aggregate/raw static, weighted and dynamic measurements plus manifest. | Machine-readable results. | Synthetic workloads only. |
| `GLOBAL_PLANNER_ALGORITHM_EVALUATION.md` | Focused benchmark methodology and result summary. | Complements the detailed current measurements and audit in `GLOBAL_SEARCH.md`; static/dynamic timings predate the zero-cost update fallback. | Synthetic data only. |
| `examples/supplied_graph_demo.py` | Small producer-shaped toy graph demonstrating planning, a cost update, and no-path. | Imports the public graph/planner modules. | Is not a real producer integration or performance benchmark. |
| `pyproject.toml` | Build metadata, package discovery and pytest configuration; test extra supplies pytest. | Used for installation/build/test. | No ROS runtime dependency is declared. |
| `GLOBAL_SEARCH.md` | Current architecture, responsibility boundary, historical search evidence and known limitations. | Documentation. | Does not make the historical graph-builder work active planner scope. |

The evaluation-only algorithms are separate from the production planner API; graph generation and edge-cost assignment remain upstream responsibilities.

## 6. Search algorithms

### Dijkstra

The active implementation repeatedly expands the lowest accumulated supplied cost. It is exact for finite directed graphs with nonnegative edge costs and needs no geometry or heuristic. It is the Dijkstra reference and selectable fallback. Its main limitation is that it may expand more nodes than safe informed search.

### A*

The active implementation is the same general A* search in `astar.py`. In the wrapper, if every node has finite coordinates and endpoint distances remain finite, it derives:

```text
alpha = min(edge cost / Euclidean endpoint distance)
        over edges with positive endpoint distance
h(n) = alpha * Euclidean distance(n, goal)
```

For every graph edge, the selected scale bounds the supplied cost below by scaled geometric edge length. The triangle inequality then makes the estimate a lower bound on every route cost. If geometry is omitted/unusable, or the derived scale is zero, the wrapper sets the scale to zero (`h=0`), so A* reduces to exact Dijkstra ordering. This is safe for arbitrary nonnegative supplied costs but gives no guidance.

Unscaled Euclidean distance is not generally admissible when cost units and geometric distance have no proven relationship. For example, a one-meter edge may have cost 0.01; using one meter as a cost estimate could overestimate the actual route cost.

The current prototype does not accept a caller-provided heuristic callback or a producer-supplied lower-bound constant. The current synthetic measurements and independent checks are in [GLOBAL_SEARCH.md](./GLOBAL_SEARCH.md); actual producer workload still needs measurement.

### Other evaluated algorithms

Weighted A*, bidirectional Dijkstra, balanced-potential bidirectional A*, LPA*, and D* Lite have benchmark-only implementations in `benchmark_algorithms.py`. Their exact implementation status and evidence-based selection are documented in the benchmark report. They are not production API options.

### D* Lite

Implemented for evaluation and tested through repeated changes, but not selected for production: on the tested synthetic dynamic sequence its full update-to-route median was slower than fresh A* despite usually reducing expansions. Zero-cost graph updates explicitly trigger a full reset to avoid stale labels in zero-cost cycles. This conclusion is limited to the measured graph and update pattern; the timing rows predate that fallback, and current independent checks are in [GLOBAL_SEARCH.md](./GLOBAL_SEARCH.md).

## 7. Static search test campaign

Current tests use explicit deterministic graphs. The independent reference repeatedly relaxes all supplied edges and does not call Dijkstra or A*. A test computes a deterministic SHA-256 graph fingerprint and runs Dijkstra and A* against the same unchanged multimodal fixture/version. No candidate rebuilds the graph.

| Comparison graph | Version | SHA-256 |
|---|---|---|
| Five-node supplied multimodal fixture | `frozen-fixture` | `9a05323daa7ee77eb42bfa1152c8a479a363490a6da5ace0bfeccf69fb91e3d8` |

This fingerprint is test-fixture evidence only. It is not a hash of a real producer graph.

| Case | Fixture purpose and check | Expected and actual current result |
|---|---|---|
| G1 | Single uniquely cheapest route. | Correct start/goal path and minimum supplied cost. |
| G2 | Two routes with distinct costs. | Select lower supplied sum. |
| G3 | Identical coordinates, swapped edge costs. | Selected path changes with supplied costs, not geometry. |
| G4 | Directed graph. | Missing reverse edge is not invented. |
| G5 | Disconnected nodes. | `success=False`, empty route, infinite cost. |
| G6 | Start equals goal. | One-node route, cost zero. |
| G7 | Multiple equal-cost routes. | Any valid minimum-cost route is accepted. |
| G8 | Cycles including zero-cost edges. | Search terminates and agrees with independent optimum. |
| G9 | Dense 12-node graph. | Dijkstra and A* match repeated relaxation. |
| G10 | Deterministic 250-node sparse graph. | Both methods match the chain optimum, cost 249. |
| G11 | WALK/FLY mode metadata and transitions. | Route uses supplied multimodal edges; two mode changes in the main fixture. |
| G12 | Increase cost on a route edge. | Replan selects the alternative route. |
| G13 | Remove necessary edges. | Replan returns no-path. |
| G14 | Add a cheaper route. | Replan selects the newly supplied lower-cost route. |
| G15 | Change start after replacement. | New request's start is used. |
| G16 | Change goal after replacement. | New request's goal is used. |
| G17 | Replace snapshot and reuse a revision label. | Old result is no longer current because snapshot ID changed. |
| G18 | Invalid edge, cost, mode, revision, adjacency or endpoint. | Input rejected; previous snapshot is disabled after rejected replacement. |

Additional tests cover arbitrary node pairs on the multimodal fixture, three fixed-seed directed graphs, zero-cost edges and incremental zero-cost cycles, caller mutation after snapshot acceptance, missing endpoints, absent geometry/mode metadata, invalid coordinates, large finite coordinates and route costs, invalid route edges, and configured route-cost tolerance.

The final suite passes 59 tests. This validates software behavior on synthetic supplied graphs, not completeness or physical safety of a real graph from the upstream producer.

## 8. Replanning test campaign

| Update | Current behavior tested | Current limitation |
|---|---|---|
| R1: raise one edge cost | Dijkstra and A* full-snapshot replanning selects the alternative. | No separate graph-copy/update latency metric. |
| R2: remove route edges | Returns no-path rather than a partial route. | No incremental repair implementation. |
| R3: add a cheaper route | Replacement snapshot uses the new lower-cost route. | Synthetic only. |
| R4: change several costs at once | Uses the combined new costs and chooses the new minimum. | No production update stream. |
| R5: change start | `replan` searches from the passed current node. | Caller must supply/attach it. |
| R6: change goal | `replan` searches to the newly passed goal. | Controller goal protocol is not integrated. |
| R7: full snapshot replacement | Snapshot copied; revision and internal ID are returned; old result fails `is_current`. | No ordering, cancellation, concurrency or executing-route stop policy. |

The separate repeated dynamic benchmark measures update processing, replanning, total update-to-route time, expansions, cost against an updated Dijkstra reference, speedup and expansion reduction. Its historical results and current independent checks are summarized in [GLOBAL_SEARCH.md](./GLOBAL_SEARCH.md).

## 9. Algorithm comparison and decision

### Current capability

| Method | Active implementation | Correctness contract | Current decision |
|---|---|---|---|
| Dijkstra | Yes | Exact for nonnegative supplied edge costs. | Reference/validation; selectable production fallback. |
| A* | Yes | Exact with derived admissible scale; otherwise zero heuristic. | Production default for initial planning and fresh replanning. |
| Weighted A* | Evaluation-only | Approximate for weights above 1. | Optional only when measured cost degradation is explicitly acceptable. |
| Bidirectional Dijkstra | Evaluation-only | Exact for nonnegative supplied edge costs. | Not selected; measured preprocessing overhead outweighed expansion savings in tested scenarios. |
| Bidirectional A* | Evaluation-only | Exact using balanced potentials. | Not selected; measured overhead outweighed expansion savings in tested scenarios. |
| LPA* | Evaluation-only | Incremental exact repair for tested changes. | Not selected for current snapshot workload; update-to-route time trailed fresh A*. |
| D* Lite | Evaluation-only | Incremental exact repair for tested changes. | Not selected for current snapshot workload; update-to-route time trailed fresh A*. |

### Historical evidence (not a current production benchmark)

An earlier reconstructed 11-label comparison recorded single observations. It is historical and superseded for algorithm selection by the repeated identical-graph campaign and independent review in [GLOBAL_SEARCH.md](./GLOBAL_SEARCH.md).

A separate historical 12-configuration run recorded 144 serial observations, with three repeats per method/configuration. It reported these medians/P95 values:

| Historical method | Correct outcomes | Successful routes | Search median / P95 |
|---|---:|---:|---:|
| Dijkstra | 36/36 | 30/36 | 1.620 / 12.495 ms |
| A* | 36/36 | 30/36 | 1.544 / 12.480 ms |
| Weighted A*, weight 1.5 | 36/36 | 30/36 | 0.538 / 27.768 ms |
| Weighted A*, weight 2 | 36/36 | 30/36 | 0.527 / 7.272 ms |

Dijkstra and A* agreed on supplied route costs in that campaign. Weighted A* is approximate: the recorded maximum cost increases were 5.574% and 19.148%, so it does not meet the minimum-cost contract and was removed. This campaign predates the current graph-input-only package, and its timings are not current Windows end-to-end budgets.

Historical process-memory values used peak working set/RSS including worker startup and imports, not isolated planner live memory. Search time excluded graph building and route audit. The older benchmark evidence and its limitations are retained under `evidence/global_planner/history/` and summarized in `GLOBAL_SEARCH.md`.

### Decision

**Current evidence-based choice:** safe A* is the production default for initial and fresh replanning; Dijkstra is the optimal reference; incremental algorithms remain evaluation-only. The benchmark report gives measured results and makes clear that synthetic graphs are not a substitute for producer workload evidence.

## 10. Metrics and how to interpret them

| Metric | Definition/unit | Why it matters | Current measurement / limitation |
|---|---|---|---|
| Success/no-path | Boolean and route presence | Detects connectivity result. | Tested across graph fixtures. |
| Optimality | Supplied total cost vs independent reference | Core correctness criterion. | Repeated-relaxation oracle in tests. |
| Route validity | All consecutive node pairs have supplied directed edges | Prevents invented connections. | Wrapper maps returned pairs through its copied adjacency; tests also check edges. |
| Version freshness | Revision + planner-local snapshot identity | Stops stale route consumption. | `is_current`; caller must check. |
| Total cost | Sum of supplied edge costs | Objective minimized by search. | Audited with `math.fsum` and tolerance. |
| Route size | Number of returned nodes/edges | Search-output size only, not physical quality. | Derivable as `len(path)` / `len(edges)`; not a route-quality metric. |
| `search_ms` | Milliseconds around the search function | Isolates algorithm call cost. | Excludes validation/copy, route audit, I/O, startup and message conversion. |
| Replanning latency | Update processing, search/repair, and total update-to-route time | Operational update response. | Measured across four repeated synthetic changes; see the evaluation report. |
| Nodes expanded/explored/generated and edge evaluations | Integer counters from each search/update. | Indicates graph work performed. | Recorded in benchmark summaries and raw samples. |
| Peak memory | Peak bytes/MiB during relevant operation | Resource sizing. | `tracemalloc` peak Python allocation is recorded only for the small static case; not RSS. |
| Runtime distribution | Median, mean, sample standard deviation, min and max | Captures central tendency and variability. | Ten static repetitions; five complete dynamic-sequence repetitions. |

No clearance, curvature, slope, terrain, reverse-motion or physical feasibility metrics are planner metrics. The producer owns those checks.

## 11. Failure and fix history

| Issue | Category/root cause | Resolution and verification |
|---|---|---|
| Graph-generation work was inside the earlier project scope. | Responsibility boundary mismatch; planner generated nodes/edges/transitions already owned upstream. | Active package was narrowed to explicit graph data, search and snapshot handling; imports/tests reviewed; current package suite passes. |
| Dijkstra's input unnecessarily required node positions. | Data contract coupled exact search to optional geometry. | Node geometry/mode are now optional; Dijkstra works without either, and A* falls back to `h=0`; tests cover both. |
| Euclidean distance alone is unsafe for arbitrary edge costs. | Heuristic units/lower bound were not guaranteed by geometry alone. | A* derives a graph-wide cost-per-distance lower bound when complete finite geometry exists; otherwise uses zero heuristic. Tests cover zero-cost, large coordinates and absent geometry. |
| Old search fixture labels did not consistently reproduce old graph evidence. | Reconstructed fixtures were not identical to earlier graph/route artifacts. | Historical comparison is explicitly not used to select a winner; final correctness tests fingerprint a fixed input for both active methods. |
| Stale route could be confused with a new graph reusing its label. | External revision may repeat. | Internal snapshot ID increments on every replacement; `is_current` checks both. Tested with repeated labels. |
| Invalid replacement could leave old graph silently active. | Unsafe fallback to an earlier graph after rejected input. | Snapshot is cleared before validation; tests verify invalid updates disable planning. |
| Earlier full workspace had failing graph-generation/integration checks. | Those tests required removed generation code and an old benchmark memory helper; not current search-only regressions. | Original output remains in `evidence/global_planner/graph_input_scope/full.txt`; 347 passed and 61 failed in that old broad run. The failures are not claimed as fixed. |
| Python environment drift. | Previous project runtime/build backend was incomplete. | Current validation used the configured repository `.venv` (Python 3.12.14); exact current command and output are below. Packaging evidence is separate and its build-environment limitation is recorded. |

## 12. Development sequence (newest to oldest)

1. **Current supplied-graph prototype:** Added optional geometry/mode metadata support without changing the graph producer boundary; exact search, route/cost audit, full-snapshot replanning and stale-result detection are active.
2. **Search-layer validation:** Added deterministic directed, multimodal, tied-route, cyclic, dense, large-sparse, update and malformed-input tests; both active methods are checked against independent repeated relaxation.
3. **Responsibility correction:** Removed active graph-generation, map, robot-feasibility and related benchmark code from the planner package. Historical results remain evidence only.
4. **Earlier algorithm studies:** Dijkstra, A*, weighted A*, optimized exact A* and D* Lite were explored using generated/reconstructed graph campaigns. Fixture identity and timing limitations prevent those studies from establishing a production speed winner.
5. **Earlier representation studies:** WALK, FLY and transition generation were developed in this repository before the task boundary was clarified. They are upstream concerns, not part of the active global-planner package.

## 13. Code cleanup and evidence

### Active production files kept

The active package files are listed in Section 5. The planner imports no map, costmap, WALK/FLY roadmap, transition-generation or robot-validation module.

### Removed files

The verified scope-cleanup inventory records **398 removed paths**, including **36 old package source modules** and **33 old test modules**. Categories include:

- WALK/FLY/transition graph generation and geometry/feasibility validators;
- map adapters, scenario loaders, map/cost utilities and old planning representations;
- graph-generation and search-comparison runners, benchmark/memory helpers and their old tests;
- ROS packaging/demo boilerplate and duplicate reports/results that were outside the active planner.

The exact removed path list and cleanup audit are preserved in `evidence/global_planner/scope_cleanup/removals.json` and `evidence/global_planner/scope_cleanup/verification.json`. Files were removed only after checking active imports, tests and package discovery. The removed 33 tests are not represented as passing in the current suite.

### Evidence retained

- `evidence/global_planner/graph_input_scope/full.txt` and `full.xml`: prior broad suite, including its 61 failures.
- `evidence/global_planner/graph_input_scope/focused.txt` and `focused.xml`: earlier 15-pass focused checkpoint.
- `evidence/global_planner/scope_cleanup/final.txt` and `final.xml`: earlier 29-pass final checkpoint, before the latest optional-metadata and additional graph campaign tests.
- `evidence/global_planner/scope_cleanup/demo.txt`: earlier demo output.
- `evidence/global_planner/scope_cleanup/package.txt` and `package_demo.txt`: prior package build/example evidence with its build-tool limitation.
- `evidence/global_planner/history/`: prior search raw data, manifests, audit and reconstructed comparisons.

The repeated algorithm campaign and independent review are summarized in [GLOBAL_SEARCH.md](./GLOBAL_SEARCH.md), with raw and summary CSVs under `benchmark_results/`. These are synthetic supplied graphs and do not define production latency budgets.

## 14. Test evidence and reproduction

Current environment: repository `.venv`, Python 3.12.14, Windows 11.

Commands:

```powershell
.\.venv\Scripts\python.exe -m pytest -q
.\.venv\Scripts\python.exe examples\supplied_graph_demo.py
```

Latest regression output:

```text
59 passed
```

The demo output:

```text
revision: 1 success: True path: [0, 2, 3, 4] cost: 3.0
revision: 2 success: True path: [0, 1, 4] cost: 6.0
revision: 3 success: False path: [] cost: inf
```

These are current software results. No current producer-message benchmark command is available because the producer interface/fixture has not been delivered.

## 15. Repository publication details

| Item | Project detail |
|---|---|
| Remote | `https://github.com/samaelqadi/m4-global-planning` |
| Branch | `main` |
| Runtime/generated paths | `.venv/`, `build/`, `dist/`, `*.egg-info/`, `__pycache__/`, `.pytest_cache/`, `*.py[cod]`; ignored by `.gitignore`. |
| Implementation commit | `Finalize supplied-graph global planner`. |

The source, tests, documentation and preserved historical evidence were reviewed before finalizing the project tree. Runtime environments, build products, bytecode and test caches are excluded from version control.

## 16. Status by confidence

- **PROVEN BY TEST:** Production Dijkstra/A* and evaluation algorithms passed their synthetic correctness checks; route validity/cost audits, invalid-input rejection and repeated graph-change checks pass.
- **MEASURED ON SYNTHETIC FIXTURES:** Safe A* substantially reduced work on geometric grids; fresh A* beat incremental methods in total update-to-route time on the tested dynamic sequence.
- **IMPLEMENTED CURRENT PROTOTYPE:** Explicit complete-graph snapshot input; safe A* default, selectable exact Dijkstra reference; synchronous full-snapshot replanning.
- **SUPPORTED BY THEORY:** Dijkstra exactness for nonnegative costs; A* exactness under the documented lower-bound heuristic or `h=0`.
- **CONFIGURABLE:** Search method (`dijkstra` or `astar`) and cost-audit tolerance.
- **NOT YET CONFIRMED:** Actual producer contract, graph completeness/physical validity, coordinate frame and common cost units.
- **REQUIRES INTEGRATION:** Producer message conversion, live graph updates, start/goal IDs, controller output, stale-result response policy, and system-level timing/memory budgets.
- **REQUIRES SIMULATION:** End-to-end route consumption/update behavior under a representative changing environment.
- **REQUIRES REAL ROBOT VALIDATION:** Physical execution, calibration, controller safety and hardware timing.

## 17. External integration still needed

1. Producer schema for graph version, nodes, directed edges, costs and optional metadata.
2. Agreed cost units, numeric ranges, coordinate frame and duplicate-action/parallel-edge handling.
3. Definition of how current pose and goal become IDs in the supplied graph.
4. Conversion from the producer transport format to `Graph`.
5. Output schema for route node/edge identity and controller action payload.
6. Policy for graph updates during route execution: freshness check, replan request, controller acknowledgement and stop/replacement behavior.
7. Representative producer graph fixtures and end-to-end correctness tests.
8. Agreed end-to-end planning/replanning latency and peak-memory targets.

## 18. Limitations and assumptions

### Current assumptions

- The producer supplies a finite graph with valid unique integer node IDs and all route-feasible directed edges.
- Edge costs are finite, nonnegative scalars in consistent units across modes.
- The producer has already checked collision, terrain and robot-motion feasibility.
- The graph version identifies the complete snapshot; the planner accepts full replacements, not patches.
- A caller provides start and goal node IDs already in the graph.
- Updates and planning are sequential in one planner instance; no concurrency guarantee exists.
- Nodes may omit geometry; A* then uses `h=0`. If geometry is supplied for A*, coordinates are finite and use one consistent frame/unit.
- The current graph structure has no parallel edges or independent edge/action IDs.

### Known limitations

- The graph is an internal Python structure; producer wire schema and ROS integration are not implemented.
- Route result edges retain only endpoints and cost; rich action/trajectory metadata is not preserved.
- No pose attachment, local path generation, collision validation, physical feasibility or control is performed.
- `is_current` is planner-instance local and must be called by the consumer. It is not cross-process authentication.
- Revision values are not ordered; delayed out-of-order graph deliveries are not detected.
- No asynchronous cancellation, concurrent update handling, execution acknowledgment or controller stop command exists.
- `search_ms` excludes input validation, snapshot copy, output validation, message conversion and I/O.
- Current tests are synthetic. No actual upstream graph, simulation trial, real-robot run, latency target or memory target is available.
- Historical performance results cannot replace benchmarking the actual producer graphs and updates.

## 19. Future ROS 2 architecture

ROS 2 is not implemented by this task. If required, one planner node can wrap the Python library; each module does not need its own node:

```text
graph producer topic/service
        -> graph snapshot + version
        -> global planner node
        -> global route topic/service
        -> local planner/controller
```

The wrapper should translate the agreed message to `Graph`, pass start/goal IDs, publish route and version, and define how consumers react to stale results. Keep graph generation upstream.

## 20. Final project status

### Done in software

- Complete explicit graph snapshot validation and copying.
- Exact Dijkstra search and A* search with safe heuristic scaling/fallback.
- Minimum-cost route edge/cost audit.
- No-path behavior.
- Full-snapshot synchronous replanning and snapshot freshness checks.
- Optional node geometry/mode metadata for search independence.

### Tested

- 59 current pytest cases pass.
- Supplied-graph demo passes.
- Active searches agree with independent reference costs across covered synthetic graphs.

### Selected

- Safe A* is the production default for initial planning and fresh full-snapshot replanning in the tested workload.
- Dijkstra is the optimal reference and selectable fallback.
- Weighted A* is not automatic; weights 1.5 and 2 traded 25% higher cost for 99.7% fewer expansions on the constructed tradeoff fixture.
- LPA* and D* Lite remain evaluation-only because measured end-to-end update times did not beat fresh A* on the tested sequence.
- These are synthetic benchmark decisions, not final validation on actual producer graphs.

### Not yet integrated

- Actual producer schema/converter and producer graph fixtures.
- Live graph-update transport, controller output contract and execution freshness policy.
- Representative producer-graph, end-to-end latency and memory measurements.

### Not our responsibility

- Mapping/SLAM, map/costmap creation, node/edge/transition generation, edge cost calculation, physical feasibility/collision checks, local path tracking and motor control.

### Future work

- Agree and connect producer/consumer messages.
- Re-run the identical-graph campaign with versioned producer snapshots, real update rates, and end-to-end graph preparation and search measurements.
- Reconsider optimized exact A* or D* Lite only if real workload evidence justifies their complexity.
- Integrate with simulation and later perform real-robot validation through their owners.
