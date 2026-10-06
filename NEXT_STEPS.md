# Next steps

Selections remain provisional: Hybrid A* is the flat-ground WALK baseline, and the coarse adaptive 3D lattice is the current geometric FLY reference. The fine sparse roadmap remains a useful alternative. Existing evidence compares route validity and performance on finite graphs; it does not establish continuous-space or robot-level optimality.

## 1. Check graph-relative optimality — focused audit complete

For each explicit WALK graph (lattice, grid, roadmap) and both FLY graphs, compare the returned A* cost with an independent Dijkstra result on the *same constructed graph*, endpoints, costs, and waypoint sequence. Audit route edges and exact endpoints as today. For Hybrid A*, report optimality only against its finite explored graph if that graph is exposed; its coupled motion search is not an explicit representation-wide graph. This check proves optimality only within the graph searched.

The completed audit compares A* with independent Dijkstra over the full constructed WALK lattice graph and the full reachable heading-aware successor graphs for lattice/grid/roadmap at G02, plus both full FLY graphs at F05/F06 (each waypoint leg). Audit mode continues after the A* goal to exhaust finite reachable successors; normal planning still stops at the goal. Reconstructed routes are checked with existing motion/state/swept validators. The focused regression set passed 130 checks, preserving the prior 123.

Grid and roadmap use lazy successors; audit mode fully expands the finite reachable successor graph at configured resolution, while their spatial proposal graphs omit heading/connector costs and are invalid cost references. Hybrid remains a separate reference-relative comparison; its bins prevent a representation-wide proof. See Stage 10 in the report for matched cases and limitations.

Reproduce only the bounded audit and related focused regressions with:

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=src/m4_global_planner python3 -m pytest -q -p no:cacheprovider src/m4_global_planner/test/test_straight_walk_edges.py src/m4_global_planner/test/test_walk_motions.py src/m4_global_planner/test/test_walk_comparison.py src/m4_global_planner/test/test_fly_validation.py src/m4_global_planner/test/test_anchors.py src/m4_global_planner/test/test_fly_graph.py src/m4_global_planner/test/test_graph_optimality.py
```

## 2. Plan minimum cost across modes — geometric integration complete

`representation/joint_graph.py` provides `JointPlanner(parameters, settings, planning_map,
builder, endpoints, seed=0)`. Pass `data['joint_planner']` as settings, and explicitly apply
its `walk_terrain_cost_mode` to parameters before constructing the WALK modules. Supply
named `Node` endpoints; `plan(start='start', goal='goal')` runs one configured query over both
complete configured graphs and all directed, validated stationary morph edges.
YAML `joint_planner.search_method` defaults to `dijkstra`; alternatives are `astar` and
`weighted_astar`, which uses `weighted_astar_weight` (default 1.5, finite and at least 1).
YAML `cost_tolerance` configures relative and absolute edge/route cost comparisons
(default 1e-10; finite and positive). Omitting the new settings also defaults to Dijkstra. A* uses weight 1 regardless of the
weighted setting. Dijkstra and A* minimize configured cost on the constructed graph;
weighted A* can return a higher cost, with no bound claimed here. The
returned path contains joint graph IDs; WALK connector primitives are in `planner.paths`.
Every successful route is revalidated and its costs recomputed. Hybrid remains separate
and provisional. Joint optimality is relative to this explicit finite graph only.

The WALK backbone reuses the exact motion lattice. Missing exact WALK endpoints and
transition poses receive local bidirectional validated connectors within `endpoint_radius`;
existing lattice anchors retain the lattice's primitive adjacency. FLY reuses the existing
adaptive lattice or sparse roadmap and mandatory anchors. Transition cost is
`transition_extra_cost` per directed stationary morph. Reverse/pivot penalties and terrain
distance-density costs remain in existing WALK modules; FLY retains its configured
per-meter cost. The heuristic uses the smaller movement rate, including zero WALK rate.

T01–T06 joint regressions compare A* against independent Dijkstra over identical complete
constructed graphs, including no-path outcomes and both FLY topologies at T05. They audit
both returned paths, exact off-grid endpoints/modes/headings, blocked-morph detours,
direction restrictions, penalty changes, cost corruption and stale morph geometry.
See Stage 11 in `representation_comparison.md` for outcomes and limits.

Reproduce the full package checks with:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select m4_global_planner
colcon test --packages-select m4_global_planner --event-handlers console_direct+ --pytest-args -q -p no:cacheprovider
colcon test-result --test-result-base build/m4_global_planner
```

## 3. Resolve unsupported assumptions

The FLY model is a fixed, axis-aligned bottom-center box with absent yaw, straight translation, flat known floor, and configured geometric margins. Unknown regions and non-flat floors are rejected. Takeoff, landing, attitude changes, dynamics, energy costs, and calibrated body/morphing envelopes are unsupported. Validate these assumptions in simulation and on the robot before operational use; define altitude, clearance, landing-heading, and transition-cost requirements with measured vehicle data.

## 4. Compare joint graph searches — bounded measurement complete

Use `python3 -m m4_global_planner.joint_search_benchmark --output <new-directory>` with
`PYTHONPATH=src/m4_global_planner`. `joint_search_benchmark` in YAML configures weights,
repeats, fixed graph seed, explicit synthetic endpoint requests, supported case overrides,
comparison tolerances and worker timeout. Each graph is constructed once and saved; all
methods/repeats load its identical hash-checked snapshot. Timing excludes graph construction,
loading and route audit; memory replay is separately instrumented. Existing result directories
are protected by source/config fingerprints; use a new directory after computational changes.

The completed comparison has 144 validated timed runs on 12 graphs (three repeats of
Dijkstra, A*, weighted A* 1.5 and 2). A* matched Dijkstra everywhere and its heuristic lower
bound was verified across all graph edges. Maximum measured weighted gaps were 5.574% and
19.148%; no theoretical weighted bound is claimed. Raw per-run costs, explored states/pops,
timings, memory, paths and fingerprints are in `joint_search_results/raw.jsonl`, with one
concise table in `comparison.md`. See Stage 12 for matched scale/clutter tradeoffs.

Recommend Dijkstra for the current bounded exact-cost workloads absent a hard latency
requirement; retain A* for cases where its savings are measured, and weighted A* as an
experiment pending acceptable cost-gap/latency targets. The public joint planner now
defaults to Dijkstra (Stage 13), retaining configurable alternatives and the same result
fields and route audit. The separate provisional Hybrid baseline is unchanged.

## 5. Public joint search selection — complete

`JointPlanner` reuses the existing independent Dijkstra and A* implementations; no search
loop or graph construction is duplicated. All successful public calls revalidate every
WALK connector primitive, FLY sweep, stationary morph and total cost. Public-interface tests
cover each method on WALK-only T06, required bridge T05, blocked morph T02, and low/high
penalty T04; exact methods match the independent test Dijkstra minimum. Invalid method
names and nonfinite/sub-unit weights are rejected before graph construction. Package build
succeeded; full package tests passed 188 checks (including lint), with one existing copyright
skip. All prior 168 passing checks are preserved. Prior saved
measurements remain historical snapshots; no benchmark was repeated for this change.

## Exact next focused task

Define separately validated takeoff and landing before treating joint paths as executable
routes. Calibrate heading/attitude and morphing envelopes, clearance/altitude and physical
costs. Specify acceptable route-cost gaps and search/construction latency/memory budgets
before choosing weighted search or optimizing scale. Measure repeated live graph queries
and broader workloads if those budgets require it. Joint waypoint ordering, map updates,
WALK construction resource limits and continuous-space coverage remain pending. Preserve
all saved benchmark checkpoints; finite-graph optimality and these bounded measurements
do not establish physical execution or a final representation selection.
