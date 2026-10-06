# Review handoff

2026-10-06. Local HEAD: `61fded2ad2a8600fab148567a2fe3e36164bf016` (joint planner,
search comparison and saved results). Local `origin/main` points to this same commit, and its reflog records `update by push`.
An earlier attempt failed authentication, but available Git evidence shows a later successful push;
no fresh remote query was performed for this reconciliation. Repository metadata is `.git-metadata`.

## Architecture

Paths below are under `src/m4_global_planner/m4_global_planner/` unless linked otherwise.

- [representation/joint_graph.py](src/m4_global_planner/m4_global_planner/representation/joint_graph.py): `JointPlanner` builds one directed WALK/FLY/morph graph, searches it, audits routes and rebuilds after supported updates. Results carry geometric/execution labels and map revision; `validate_result` rejects stale revisions.
- [walk_comparison.py](src/m4_global_planner/m4_global_planner/walk_comparison.py), `representation/walk_graph.py`, `walk_edges.py`, `walk_costs.py`: explicit heading-aware WALK lattice and validated connector primitives/costs. Hybrid stays separate.
- [representation/fly_graph.py](src/m4_global_planner/m4_global_planner/representation/fly_graph.py), `anchors.py`, `fly_validator.py`, `geometry.py`: shared exact anchors, adaptive lattice/roadmap, swept boxes and stationary morph checks.
- [search_comparison.py](src/m4_global_planner/m4_global_planner/search_comparison.py) and [astar.py](src/m4_global_planner/m4_global_planner/astar.py): independent Dijkstra, heuristic audit, A*/weighted A*.
- [dummy_map.py](src/m4_global_planner/m4_global_planner/dummy_map.py): complete test-world adapter and revisioned update snapshots. [scenarios.yaml](src/m4_global_planner/test_data/scenarios.yaml): synthetic requirements, costs, tunable policies and R01/R02 updates.

## Decisions and historical measurements

Dijkstra is the joint default; A* and weighted A* remain configurable. Historical Dijkstra
per-case median search times were below 12.5 ms, versus 166–1820 ms construction. A* helped
scale but slowed clutter; weighted weights 1.5/2 had maximum measured gaps 5.574%/19.148%.
No weighted bound or latency guarantee is claimed. Replanning therefore uses the simplest
full snapshot rebuild followed by Dijkstra; operational rebuild performance is unmeasured.

See the [joint comparison table](joint_search_results/comparison.md), [144 raw runs](joint_search_results/raw.jsonl),
[manifest](joint_search_results/manifest.json) and [artifact audit](joint_search_results/audit.json).
These are **historical**, frozen before later default/contract/replanning changes; their source
fingerprints do not describe today's working tree. Three serial repeats per method on 12
once-built graphs excluded construction/loading/audits from search time. Memory replay was
separate; RSS includes imports/loading. Other host load was uncontrolled. Older
[WALK](walk_results/comparison.md) and [FLY](fly_results/comparison.md) measurements support
only provisional representation selections. Details: [stage report](representation_comparison.md).

## Current checks and guarantees

Prior replanning full package run: **215 passed, 1 existing copyright skip**; build, flake8 and pep257
passed. All preceding 200 passing checks remain, with 15 new replanning checks. Earlier
contract-stage checks passed 200; the latest replanning affected run passed 70 before two
final checks were added and included in the full suite. No benchmarks were rerun for these
stages or the later adapter audit. See [NEXT_STEPS.md](NEXT_STEPS.md),
[joint tests](src/m4_global_planner/test/test_joint_graph.py),
[replanning tests](src/m4_global_planner/test/test_replanning.py) and
[search tests](src/m4_global_planner/test/test_search_comparison.py).

Dijkstra minimizes nonnegative configured costs **on the constructed finite graph**.
A* has the same guarantee with the admissible minimum-movement-rate heuristic; historical
checks verified every edge's distance lower bound/consistency and matched independent
Dijkstra. Updated-graph tests also match independent Dijkstra. Every successful route audits
states, WALK connector primitives, FLY sweeps, stationary morphs, edge prices and total cost
within configured tolerance. No-path means no route in this graph, not continuous-space
impossibility. Hybrid has no representation-wide optimality proof.

## Assumptions, unsupported behavior and next work

Bounded flat-floor maps with known or conservatively blocked occupancy; WALK ground-contact center and heading; fixed axis-aligned,
bottom-center FLY box without yaw; synthetic dimensions, margins and costs. Stationary
WALK↔FLY morphing is separate from physical takeoff/landing. All public joint results are
`route_kind='geometric'`, `executable=False`; execution-required requests fail before search/update.
[TAKEOFF_LANDING_CONTRACT.md](TAKEOFF_LANDING_CONTRACT.md) distinguishes evidence from missing
robot/controller frames, envelopes, support/contact rules, motion bounds, terminal guarantees
and independent costs. No physical limits were invented.

Updates support adding obstacle boxes, removing exact WALK exclusions and replacing WALK
cost regions. Accepted updates clear old planning data; rebuild failure disables planning;
invalid updates preserve the prior snapshot. Replanning forces Dijkstra, with fixed endpoints
and synchronous snapshots. Direct dictionary mutation, concurrent updates, cross-planner
revision identity, moving-start localization, real map feeds and incremental search are unsupported.
Pending: robot/controller inputs and simulation validation; general map/waypoint integration,
WALK construction budgets, coverage, and measured operational latency/cost-gap requirements.
Details and next task: [NEXT_STEPS.md](NEXT_STEPS.md#exact-next-focused-task).

## Uncommitted work

Modified: `NEXT_STEPS.md`, package `dummy_map.py`, `representation/joint_graph.py`,
`test/test_fly_validation.py`, `test/test_joint_graph.py`, and `test_data/scenarios.yaml`.
New before the adapter audit: this handoff, `TAKEOFF_LANDING_CONTRACT.md`, and
`test/test_replanning.py`. Additional adapter-stage changes are listed below.
These implement execution guards/contract and update/replanning support; nothing was committed
or pushed after `61fded2`. Unrelated untracked `src/m4_first_test/` and empty
`src/m4_global_planner/dummy_map.py` remain untouched and should stay outside planner changes.

## Reproduction (workspace root)

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select m4_global_planner
colcon test --packages-select m4_global_planner --event-handlers console_direct+ --pytest-args -q -p no:cacheprovider
colcon test-result --test-result-base build/m4_global_planner
# Focused update/route/validation checks instead of the full suite:
colcon test --packages-select m4_global_planner --event-handlers console_direct+ --pytest-args -q -p no:cacheprovider -k 'replanning or joint_graph or fly_validation or flake8 or pep257'
git --git-dir=.git-metadata --work-tree=. status --short
```

Benchmark reproduction instructions live in [NEXT_STEPS.md](NEXT_STEPS.md#4-compare-joint-graph-searches--bounded-measurement-complete).
If later authorized, use a new output directory; current sources cannot reproduce the old
source signature exactly. Preserve the saved result directories and trusted historical snapshots.

## Current map abstraction audit

Future **2D costmap + height input is provisional**; the real format is unavailable.
[MAP_ADAPTER_CONTRACT.md](MAP_ADAPTER_CONTRACT.md) records the audited dictionary/box
couplings, shared query contract, vertical and inflation semantics, mock schema and missing
integration requirements. The existing `PlanningMap` now covers all map queries and snapshot
updates. Typed geometric regions isolate storage fields from shared validators/cost/sampling
code. The box adapter remains; `height_grid_map.py` adds a mock configurable row/column grid.
Ground height, obstacle top and explicit occupied intervals are distinct; absent intervals
are unknown, not flight clearance. Raw geometry is required; validators own body inflation.
Grid costs use half-open cell ownership to avoid seam double charging. Both WALK and FLY follow explicit adapter unknown policy: conservative full-height
blocking, or strict map rejection, never certification of missing vertical data as free.

Additional uncommitted files: modified `representation/interfaces.py`, `geometry.py`,
`walk_validator.py`, `walk_edges.py`, `fly_validator.py`, `fly_graph.py`, and the box adapter
and YAML already listed above; new `height_grid_map.py`, `test/test_map_adapters.py`,
`MAP_ADAPTER_CONTRACT.md` and the review packet. No commit/push or historical benchmark
modifications. Adapter checks compare the same graph and costs across equivalent supported
encodings and retain independent Dijkstra route audits and non-executable labels.

[review_packet/README.md](review_packet/README.md) inventories the concise archive and its
scope. Historical benchmark artifacts remain linked, not copied into the packet. Integration
still requires real frames/z datum, occupancy/height completeness, cost units, rasterization
and inflation provenance, update transport/versioning, and robot/controller information.

Current adapter evidence: the full package run passed 236 checks with one existing copyright
skip and one formatting-only lint failure. After its correction, all 24 focused adapter/lint
checks passed. The figure 237 is arithmetic accounting (215 earlier passes plus 22 adapter
cases), not a recorded green full-suite run or a newly measured current total. Package build and pep257 passed. No research or broad
benchmark campaign was repeated. Shared-consumer scan found no `.world`, `.cells` or native
region dictionary reads. Remaining box-shaped geometry is an explicit current query contract,
not a claim that upstream storage must be obstacle boxes. Full-run duration was not used as
performance evidence. Use the focused command below for this stage:

```bash
colcon test --packages-select m4_global_planner --event-handlers console_direct+ --pytest-args -q -p no:cacheprovider -k 'map_adapters or flake8 or pep257'
```

Historical pre-reconciliation bundle: [review_packet.zip](review_packet/review_packet.zip).
Its embedded documents predate the corrections below; use the Claude packet for current review.
The bundle is an overlay/review packet, not a complete standalone ROS workspace.

## Reconciled test evidence and review scope

| Run and scope | Recorded result | Evidence |
|---|---|---|
| Before map adapter changes: full replanning suite | 215 passed, 1 skipped | [replanning log](log/test_2026-10-06_14-43-29/m4_global_planner/stdout.log) |
| After adapter changes: full package suite | 236 passed, 1 failed (lint indentation), 1 skipped | [adapter full-run log](log/test_2026-10-06_15-09-46/m4_global_planner/stdout.log) |
| After formatting correction: adapter tests + flake8 + pep257 | 24 passed, 214 deselected | [focused log](log/test_2026-10-06_17-52-57/m4_global_planner/stdout.log), [latest XML](build/m4_global_planner/pytest.xml) |

The local logs/XML are ignored build artifacts, not portable committed evidence. There is
no recorded `237 passed` full run. No tests were rerun during this reconciliation. Historical
benchmark outcomes describe older frozen sources and cannot establish current performance.

Both adapters and their update payloads are documented in MAP_ADAPTER_CONTRACT.md:
DummyPlanningMap wraps box-world dictionaries; HeightGridMap wraps configurable cells and
explicit occupied intervals. They share validators and geometric regions. Adapter changes
that could affect old results are listed below, with the smallest follow-up checks:

| Change | Possible effect | Smallest check |
|---|---|---|
| Typed regions replace native dictionary reads | Occupancy bounds, cost conversion, sampling or allocation/time can change. | Equivalent adapter graph/edge-cost tests plus shared WALK/FLY validation tests. |
| Whole-map unknown rejection now also applies to WALK | Unknown-map WALK behavior changes; supported known worlds should remain valid. | Unknown adapter tests and supported WALK state/motion regressions. |
| Half-open grid cost cells | Seam ownership differs from additive closed box cost regions. | Cell-seam cost test and cost-bearing equivalent-world route audits. |
| Explicit intervals, missing-height policy and flat-ground capability | FLY clearance, morph feasibility and sampling can change for grid input. | Overhead/multi-layer/top-only/missing-height tests; existing FLY/morph checks. |
| Native updates and snapshot conversion | Rebuilt geometry/costs and stale-result behavior can differ. | Adapter update equivalence and R01/R02 replanning tests. |

A focused future check can use `-k 'map_adapters or walk_motions or straight_walk_edges or
fly_validation or replanning or flake8 or pep257'`. Repeat the full suite after substantive
code fixes; benchmark only if a specific performance question warrants it. No alternatives
were removed: lattice/grid/roadmap WALK, separate Hybrid, both FLY graphs and all three
search methods remain. Missing real input/robot requirements do not block further mock-based
correctness tests; they block production schema/calibration and physical-execution claims.

A smaller Claude source packet is [claude_review_packet.zip](review_packet/claude_review_packet.zip);
its scope and base-workspace dependencies are listed in [CLAUDE_README.md](review_packet/CLAUDE_README.md).

## Current verified adapter review fixes (supersedes earlier check counts)

Current clean full package run: **251 passed, 1 existing copyright skip**. Build, flake8 and
pep257 passed. Evidence: [full-run log](log/test_2026-10-06_18-15-05/m4_global_planner/stdout.log)
and [latest XML](build/m4_global_planner/pytest.xml). This is an actual green full run;
the earlier 215, 236-plus-lint-failure and derived 237 figures remain historical scopes.
Fourteen new independent cases cover the reviewed policies, boundaries, cache/update
isolation, costs and unsupported terrain. No current total was inferred from earlier runs.

Confirmed: whole-map strict rejection; flat-ground motion limitation; repeated whole-grid
region rebuilding; inconsistent seam/outside interval-versus-top queries. Real costmap
unusability was conditional on unknown coverage and desired policy, not an established fact.
Equivalent-box tests alone do not establish future input correctness or general geometry.

Fixes: mock grid defaults to `block`; unknown/missing-height columns occupy the full vertical
domain for both modes, with strict `reject_map` retained. Grid intervals/top queries share
half-open cell ownership; `None` denotes unknown/outside and empty intervals denote only
known clearance. Region/floor caches and sparse cell bins are per snapshot; validators and
costs use conservative local query bounds. Public grid storage is copied on access; updates
rebuild caches. Flat-ground rejection remains explicit. Existing alternatives are unchanged.

[Focused query diagnostic](adapter_query_results/README.md) records seven warm samples per
query on 50×50 and 200×200 known grids before/after fixes. At 200×200, median known-data check
was 43.954 ms before and below 0.001 ms after; WALK state 83.278→0.013 ms and short FLY edge
172.799→0.029 ms. Construction, cache memory and end-to-end searches were not measured;
other host load was uncontrolled. No broad benchmark was rerun; historical results remain
frozen. The current contract is MAP_ADAPTER_CONTRACT.md; stale pre-fix review ZIPs are not
current-code evidence. A refreshed Claude packet accompanies these fixes.

Remaining: real schema/frames/z datum, cost interpretation, inflation provenance, vertical
coverage and support rules, live update semantics, calibrated controller limits and physical
execution. Non-flat motion, region merging, worst-case indexing performance and cache memory
are not solved here. These do not block mock tests. No commit or push was made.

## Publication evidence

The clean 251-pass result is the recorded **2026-10-06** adapter-fix run, not a new run
for this commit. A portable copy is in [test_evidence/2026-10-06_adapter_fixes](test_evidence/2026-10-06_adapter_fixes/README.md).
Git/uncommitted descriptions above are the review snapshot before publication from
`61fded2`. Review ZIPs, their manifests/extracted packets and VERIFICATION.txt remain
local-only and are excluded from publication. No new tests or benchmarks were run.
