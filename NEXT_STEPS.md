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

## 6. Minimum takeoff/landing contract — documented, physical support blocked

See [TAKEOFF_LANDING_CONTRACT.md](TAKEOFF_LANDING_CONTRACT.md) for the minimum
operation-edge contract and the evidence/missing-information matrix. Repository requirements
are geometric test settings and `pending_changes`; no calibrated robot/controller operation
requirements were found. Keep stationary WALK↔FLY morphing separate from ground-FLY↔airborne-FLY
physical takeoff/landing, including separate directions, preconditions, swept clearance,
endpoint phases, motion bounds, terminal guarantees and objective costs. This proposed
interface is not an implemented physical capability. Current FLY nodes have no phase or
controller-readiness state; ordinary translations must not be used to bypass future operation
validation. Reuse shared body geometry where valid; generic altitude/distance limits are not
physical operation constraints, and `transition_extra_cost` prices stationary morphs only.

YAML `joint_planner.execution_policy` supports only `geometric_only`. Every public joint
result is labeled `route_kind='geometric'`, `executable=False`, including no-path outcomes.
`plan(require_execution=True)` rejects the request before search, even for WALK-only paths.
Existing validators continue to reject explicitly named takeoff/landing motions, morph
directions and policies enabling unimplemented operations. Floor-to-air translation may be
geometrically clear but does not authorize physical takeoff or landing. All successful
geometric routes retain existing segment and total-cost audits; exact optimality is only
within the constructed graph and configured costs.

Verification from `61fded2`: 89 affected checks passed, then the full package suite passed
200 checks with one existing copyright skip. Package build, flake8 and pep257 passed;
all 193 preceding passing checks are preserved. Seven new regressions cover explicit
operation/policy rejection and the public execution boundary. Saved benchmark results
were neither modified nor rerun; `git diff --check` is clean.

## 7. Supported map updates and replanning — full snapshot rebuild

`DummyPlanningMap` now accepts R01/R02 `initial_world` as well as the existing `world`
format. `apply_update(update)` returns an independent map snapshot with an incremented
revision. Supported operations are `add_obstacles` (3D boxes), `remove_walk_forbidden`
(exact matching 2D center-path exclusion regions), and `set_walk_cost_regions` (complete
replacement list of 2D regions with finite nonnegative distance densities). An empty cost
list clears costs. Unknown operations, malformed/inverted/nonfinite bounds, negative costs,
and removal of an absent exclusion are rejected without changing the old map. Floor,
bounds, unknown occupancy, body/controller calibration and arbitrary region/parameter
patching are not supported update operations.

`JointPlanner.replan(update, start='start', goal='goal')` applies the snapshot update,
fully rebuilds WALK nodes/edges/connectors, shared anchors, FLY nodes/edges and morphs,
then runs the existing joint Dijkstra and segment/total-cost audit. YAML
`joint_planner.replanning_policy` supports only `rebuild_dijkstra`; no incremental search
or edge/cost reuse is implemented. Replanning switches subsequent `plan()` calls to
Dijkstra even if the initial planner selected A*. Resolutions, graph budgets, movement/
terrain/morph costs, seed, cost tolerance and exact endpoint poses retain existing settings.
Every result contains `map_revision`; `validate_result(result)` rejects older revisions
before interpreting IDs. IDs are local to each rebuilt graph; do not carry raw ID paths
across snapshots. `validate_route` is the lower-level auditor for current-graph IDs;
consumers retaining results should use `validate_result`.

Once a valid update is accepted, old graph/connector/endpoint data are cleared immediately.
A rebuild failure (for example an invalidated endpoint) leaves the new revision active with
planning disabled, rather than returning the old route. Invalid updates leave the prior
snapshot usable. A completed no-path rebuild publishes its new graph and empty path;
old success results stay stale. Replan timing reports update+rebuild, search+audit and total
elapsed time separately. These are diagnostics, not a new benchmark or latency guarantee.

R01 verifies the old route is blocked and a newly audited detour is found. R02 verifies
removing its exclusion opens a lower-cost route. Cost replacement tests show repricing,
a mode change and restoration after clearing costs; a complete added barrier yields no
path. Successful updated routes and independent test Dijkstra paths are audited against
the same updated graph. Invalid-update isolation, invalid-endpoint failure, revision
rejection, forced Dijkstra selection and unsupported incremental policy are covered.
All results remain geometric and non-executable; an execution-required replan fails before
applying its update. Saved benchmark checkpoints are unchanged.

Verification: 70 affected checks passed before the final two dispatch/policy regressions;
the final full suite passed 215 checks with one existing copyright skip. All preceding
200 passing checks are preserved, with 15 new replanning checks. Package build, flake8,
pep257 and `git diff --check` passed.

## Exact next focused task

Replanning is synchronous on complete flat-floor test maps and retains fixed requested
endpoints. It assumes all changes enter through the supported update/replan API; direct
mutation of exposed world/config/graph dictionaries, concurrent updates, cross-planner
revision identity, moving-start localization, transport/version conflicts and real map feeds
are not supported. After an accepted update that invalidates an endpoint, supply a valid
new planning request/map rather than reusing an older route. Full rebuild performance and
operational update frequency/latency budgets remain unmeasured; do not introduce incremental
search until measured requirements justify it.

Obtain the missing robot/controller pose, clearance, motion and cost requirements in
TAKEOFF_LANDING_CONTRACT.md before adding physical edges. Takeoff, landing and execution
remain unsupported. Joint waypoint ordering, general map/update semantics, WALK construction
resource limits, continuous-space coverage and operational latency/cost-gap requirements
remain pending. No unrelated benchmarks were rerun, and no commit or push is authorized.

## 8. Map abstraction audit — provisional height-grid input

See MAP_ADAPTER_CONTRACT.md for the shared interface and mock adapter. A future 2D
costmap with heights is likely, but no upstream schema is available. Planners now consume
adapter-neutral geometric regions instead of dummy storage fields. HeightGridMap keeps
row/column storage, native cell updates, height/unknown semantics and cost ownership inside
the adapter, reusing existing validators and geometry. Ground height is not obstacle top;
missing occupied intervals cannot certify flight clearance. Both WALK and FLY reject unknown
coverage. Footprints/margins belong to validators; preinflated grid input is unsupported.
Before real integration obtain frames, z datum, cost/occupancy/unknown meanings, vertical
completeness and overhead representation, rasterization/inflation provenance and update
versioning. The remaining physical execution contract is unchanged. No benchmark rerun,
commit or push is authorized. Review artifacts are listed in REVIEW_HANDOFF.md.

Adapter verification: full run 236 passed, one existing skip and one formatting-only lint
failure; after correction 24 affected adapter/lint checks passed. 237 is derived accounting, not a recorded green full-suite run; the preceding full
replanning run passed 215 checks. The latest recorded focused run passed 24 checks. Package build passed. See REVIEW_HANDOFF.md
and review_packet/README.md for source packet and evidence. No saved benchmark was modified.

## 9. Adapter review fixes — current clean checks

Confirmed policy/scanning/point-query issues were fixed with explicit `block` versus
`reject_map`, consistent half-open grid points, per-revision immutable region/floor caches
and sparse local cell queries. Missing vertical data blocks full columns, never implies free
flight. Flat-ground limitation remains, with independent rejection tests. Full package run:
251 passed, one existing copyright skip; build and lint passed. This is a recorded green
run, superseding earlier aggregate check accounting. Focused before/after query measurements
are in adapter_query_results; broad benchmark artifacts remain untouched. Further work:
real schema, frames, height/cost/inflation semantics, live updates/controller requirements,
cache-memory/construction and operational query budgets. Mock work need not wait for those
integration inputs. No commit or push is authorized.
