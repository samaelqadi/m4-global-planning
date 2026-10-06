# M4 planner progress

Student engineering report — 2026-10-06.

The package plans geometric WALK–FLY routes on finite graphs. It does not produce robot
commands or certify execution. Local HEAD is `61fded2`; execution guards, replanning,
map-interface changes and their tests remain uncommitted. Local `origin/main` matches HEAD, and its reflog records an update by push. An earlier
authentication failure was followed by a successful push according to that local evidence. File inventory and reproduction commands are in
[REVIEW_HANDOFF.md](REVIEW_HANDOFF.md).

## What we built

Map adapters supply bounds, ground height, occupied volumes, costs, exclusions and unknown
coverage through one interface. The box-world adapter remains available. A mock height-grid
adapter adds explicit vertical intervals and revisioned cell updates.

The WALK builder produces heading-aware lattice states and validated motion edges. FLY
builders produce an adaptive lattice or sparse roadmap. Shared anchors add directed,
stationary morph edges at matching positions. One joint search chooses the route and morph
locations using the total configured cost. Every successful route is checked again.

| Module group | Role |
|---|---|
| `representation/interfaces.py`, `dummy_map.py`, `height_grid_map.py` | Shared queries and storage adapters. |
| `walk_graph.py`, `walk_edges.py`, `walk_validator.py`, `walk_costs.py` | WALK states, footprints, sweeps, pivots and costs. |
| `fly_graph.py`, `fly_validator.py`, `anchors.py`, `geometry.py` | FLY sampling, shared box geometry and morph checks. |
| `joint_graph.py`, `search_comparison.py`, `astar.py` | Joint assembly, Dijkstra/A*/weighted A*, route audits and replanning. |
| [scenarios.yaml](src/m4_global_planner/test_data/scenarios.yaml) | Synthetic scenarios and configurable settings. |

Module paths are under `src/m4_global_planner/m4_global_planner/`; the handoff links them.
Replanning applies a supported update, rebuilds all graph data, and runs Dijkstra. Old
results are rejected by map revision. A failed rebuild disables planning instead of returning
an older route; a completed no-path rebuild keeps its new graph.

## Comparisons and measured reasons

The numbers below are **historical benchmarks**, not measurements of the latest adapters
or replanning implementation. Their source fingerprints predate later changes. No benchmark
was repeated for this report.

| Comparison | Recorded result | Engineering implication |
|---|---|---|
| WALK representations | Hybrid matched 60/60 expected outcomes and solved all 39 feasible requests. Coarse lattice/grid missed a sampled narrow passage; roadmap missed some requests. | Keep Hybrid as a separate WALK alternative, but retain the explicit lattice for joint graph audits. |
| FLY representations | Coarse adaptive lattice: 30/30 correct outcomes, 75.6 ms median total. Fine roadmap: 30/30, 402.2 ms. Coarse roadmap: 25/30. | Use coarse adaptive lattice as the deterministic reference; retain fine roadmap for matched route-cost/clutter tradeoffs. |
| Joint searches | 144 serial runs on 12 once-built graphs. A* matched Dijkstra costs. Median search: Dijkstra 1.620 ms, A* 1.544 ms; construction median 363.9 ms. | Dijkstra is the simplest current default. A* helped scale but was slower in clutter. |
| Weighted A* | Weights 1.5/2 had maximum measured gaps 5.574%/19.148%. Weight 1.5 was slower than exact searches in the clutter case. | Keep configurable, without a theoretical bound or universal speed claim. |

Sources: [WALK results](walk_results/comparison.md), [FLY results](fly_results/comparison.md),
[joint table](joint_search_results/comparison.md), [joint raw runs](joint_search_results/raw.jsonl)
and [detailed stage report](representation_comparison.md). Small repeat counts and host load
limit timing conclusions. Success-only pooled costs should not replace matched comparisons.

## Stable design decisions

Adapters own storage; validators own robot footprints and geometric margins. Preinflated
grid input is rejected to avoid applying the body clearance twice. Ground height, obstacle
top and occupied intervals have different meanings. Missing vertical intervals are unknown,
not evidence of clear flight space. Grid policy can block whole unknown columns or reject
the map strictly; the mock default is blocking.

Representations share validation and costs. One joint route search includes movement and
morph penalties, rather than choosing transitions separately. Stationary morphing is separate
from takeoff and landing. Resolution, origin, margins, penalties, policies and search choices
are configured rather than calibrated inside the algorithms.

## Choices that may change

| Provisional choice | Evidence needed to change it | Retained alternative |
|---|---|---|
| Explicit WALK lattice as joint reference | A validated alternative exposing its complete heading-aware graph, with better coverage/resource use on matched requests. | Existing grid/roadmap implementations; Hybrid remains separate until suitable joint integration is justified. |
| Hybrid WALK alternative | Real turning/controller limits or broader tests showing binning loses required routes or exceeds budgets. | Explicit lattice reference. |
| Adaptive FLY lattice | Matched coverage, route cost, memory and latency on real vertical maps favor another representation. | Fine sparse roadmap. |
| Dijkstra default | Defined latency/update budgets and matched measurements show worthwhile heuristic savings. | A*; weighted A* only with an accepted cost-gap policy. |
| Assumed 2D costmap with heights | Actual upstream schema, frames, occupancy/height completeness, costs and inflation provenance. | Box-world adapter and shared interface; the mock grid is not a production format. |

Full rebuild replanning also remains a starting approach. Incremental search needs evidence
that rebuild cost fails an operational requirement; no such requirement is available yet.

## Current tests and limits

An earlier adapter-stage full run passed 236 checks with one existing copyright skip and one
formatting-only lint failure. After correction, all 24 focused adapter/lint checks passed.
The earlier full replanning run passed 215 checks with one skip. The number 237 is derived
from that earlier scope plus 22 adapter cases; it is not a recorded all-green full-suite run
or a new current test total. Build and pep257 passed. The handoff links the exact logs/XML.
These recorded checks are separate from historical benchmarks; none were repeated here.

Tests cover equivalent graphs/costs through both adapters, obstacles, terrain costs, cell
seams, unknown/top-only heights, overhead clearance, multiple occupied layers and updates.
R01/R02 cover blocked and newly available routes. Further checks cover repricing, no-path,
stale results, failed rebuilds and unsupported execution requests.

Dijkstra finds the minimum configured cost **within the constructed graph**. A* has the
same guarantee with its admissible heuristic; independent Dijkstra comparisons check tested
graphs. Route audits recompute each segment and total within tolerance. This does not prove
continuous-space optimality or completeness. A sampled graph may miss a valid route, and
Hybrid has no representation-wide optimality proof.

Assumptions are bounded maps with a common known flat floor and known or conservatively
blocked occupancy, synthetic dimensions/costs, heading-aware
WALK, and fixed axis-aligned FLY bodies without yaw. Updates are synchronous with fixed
endpoints. Real map feeds, concurrent updates, moving-start localization, general waypoint
integration and live configuration mutation are unsupported.

Takeoff, landing, dynamics and physical execution are unsupported. Public results are
geometric and non-executable; execution-required requests are rejected. The
[operation contract](TAKEOFF_LANDING_CONTRACT.md) lists missing controller/robot information;
the [map contract](MAP_ADAPTER_CONTRACT.md) lists missing upstream integration details.

## Next steps

1. Obtain the real map schema, frame/z datum, vertical completeness, cost meaning, inflation provenance and update/versioning rules.
2. Obtain calibrated envelopes, support/contact rules, motion limits, controller guarantees and separate takeoff/landing costs before implementing physical edges.
3. Integrate moving requests and real updates, then validate in simulation and on the robot.
4. Define latency, memory and acceptable cost-gap requirements before changing representations, search or rebuild strategy.

[Pending work](NEXT_STEPS.md) and the [review packet](review_packet/README.md) provide the
implementation details. This report adds no new performance or execution evidence.

## Adapter review follow-up

Both storage adapters and the shared query contract are described in
[MAP_ADAPTER_CONTRACT.md](MAP_ADAPTER_CONTRACT.md). Typed region conversion, stricter WALK
unknown handling, grid seam cost ownership, explicit vertical intervals and native updates
could affect previous behavior or timing. The smallest checks are adapter equivalence,
seam/unknown/clearance tests, shared WALK/FLY validation and R01/R02 rebuild audits. Existing
results cover these tested scopes; they do not remeasure old benchmark performance.
The [handoff](REVIEW_HANDOFF.md#reconciled-test-evidence-and-review-scope) identifies each
run and follow-up scope. Search and representation alternatives remain available.

Missing input schema, frames/z datum, inflation provenance, vertical completeness, costs,
live-update semantics and calibrated controller limits remain integration requirements.
Mock-world development and correctness tests can continue while those details are obtained.

## Current adapter review fixes and checks

A subsequent clean full package run passed **251 tests with one existing copyright skip**;
build, flake8 and pep257 passed. This recorded full result supersedes the earlier 215/236
scopes and arithmetic 237 figure above. Exact log/XML links are in REVIEW_HANDOFF.md.
No broad benchmark was repeated; all search and representation alternatives remain.

The review confirmed strict rejection, repeated grid scanning, seam/outside inconsistency
and flat-ground limits. Unknown cells now conservatively block the configured full height
for WALK and FLY, while strict map rejection remains selectable. Point queries share half-open
cell ownership and return unknown outside. Cached immutable regions/floor data and sparse
cell bins replace repeated full-grid queries; updates rebuild them. Flat terrain remains
an explicit requirement, with rejection tests rather than invented terrain support.

Fourteen independent tests cover unknown policies, seams, outer boundaries, updates, costs
and non-flat rejection. [Focused local measurements](adapter_query_results/README.md) show
200×200 median WALK state query 83.278→0.013 ms and short FLY edge 172.799→0.029 ms, excluding
construction. These warm-query results do not establish complete route latency, cache memory
or live-map performance. Real schema/height/cost/inflation and robot/controller requirements
remain pending; mock development can proceed. All changes remain uncommitted.

## Publication evidence

The clean 251-pass result is the recorded **2026-10-06** adapter-fix run, not a new run
for this commit. A portable copy is in [test_evidence/2026-10-06_adapter_fixes](test_evidence/2026-10-06_adapter_fixes/README.md).
Git/uncommitted descriptions above are the review snapshot before publication from
`61fded2`. Review ZIPs, their manifests/extracted packets and VERIFICATION.txt remain
local-only and are excluded from publication. No new tests or benchmarks were run.
