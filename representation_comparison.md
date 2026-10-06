# M4 representation comparison

Date: 2026-10-05. **Decision: selection deferred.** Compare only the two candidates specified in `src/m4_global_planner/test_data/scenarios.yaml`. Shared FLY/transition geometry validation is now implemented and checked (stage 6); neither FLY candidate graph exists, so there is no coverage/performance basis to prefer one. A third candidate is not justified.

**Current WALK decision:** the stop-and-pivot Hybrid A* implementation is the **provisional flat-ground WALK baseline**. Retain the heading-aware lattice as the explicit graph reference. The complete M4 representation remains unselected pending common FLY/transition validation and the two-candidate comparison.

## Original research snapshot (historical)

The sections through “Search comparison” describe the earlier inspection. The stage update below supersedes their pending status where explicitly verified.

## Scope and evidence

Inspected package source, interfaces, setup metadata, tests, and the complete scenario file. No separate requirements document or applicable AGENTS.md was found in this workspace; scenario purposes, checks, rules, and pending changes are the available documented requirements. Broader M4 requirements remain an evidence gap. No implementation files changed, package rebuilt, or planner/scenario tests run for this report.

Evidence labels below distinguish **C** (source inspection; not an executed test), **U** (user-reported execution), **T** (theory or analytical inference), **P** (pending software evidence), and **R** (simulation/real-robot validation).

| Evidence | Finding and limit |
|---|---|
| C | `representation/state_lattice.py:30–60` samples x/y and heading, obtains WALK z from the map, and skips `None`. This is a pose sampler, not yet a motion-primitive lattice graph. |
| C | `dummy_map.py` treats missing-region boundaries inclusively and supports flat/plane height. It exposes only x/y bounds and ignores obstacles, forbidden/cost/unknown regions and vertical limits. |
| U | User reports successful package build and N04: **864 states, zero inside missing ground** after the edit. Not independently reproduced here; this does not establish collision, edge, or flight correctness. |
| T/P | N04 count is consistent with `(13 × 9 − 3 × 3) × 8 = 864`. Expected N01/N03 counts are 936 and N02 is 704 with documented 0.5 m/45° sampling. These are calculated expectations, not post-edit passes; N01–N03 remain unrechecked. |
| C/P | `interfaces.py:25` requires `generate_states(map, validator)` but the implementation accepts only `map`; no validator is invoked. PoseState has no mode; Graph.Node separately has mode and optional yaw. Conversion/identity conventions are unresolved. |
| C/P | No WALK edge builder, FLY generator, collision validator, transition checker, request validator, or map-update adapter exists in inspected source. The loader returns only `scenarios`, dropping top-level parameters/comparison metadata; it does not resolve overrides or execute checks. DummyPlanningMap expects `world`, whereas R01/R02 supply `initial_world`. |
| C | Existing tests are template lint/copyright checks, not scenario assertions. `benchmark.py` measures A* on a hand-authored eight-node graph, not either representation. |

## Requirements and minimum candidates

Both candidates use a **heading-aware WALK state lattice** with map-derived z, directed feasible motion primitives, and a **shared transition checker**. They differ only in FLY representation. Neither can rely on a height map alone: F04 explicitly requires real 3D obstacle information.

| Documented requirement | Candidate 1: adaptive 3D lattice | Candidate 2: sparse 3D roadmap |
|---|---|---|
| N01–N06: bounds, headings, ground, hard forbidden versus soft costly terrain | Shared WALK implementation must add footprint/feasibility validation; costly ground retains states. Current sampler handles only height availability. | Same shared requirement and gap. |
| W01–W08: no sideways jumps, turns, penalized reverse/pivot, swept collision and clearance | Shared motion primitives must encode actual WALK behavior and check full swept volume. | Same; roadmap flight connections must not be reused as WALK moves. |
| F01–F06: free-volume coverage, overhead/ceiling clearance, altitude choice, small openings | T: systematic sampling can be repeatable; refinement near openings may preserve connectivity. Coarse/fine interfaces and refinement triggers can still disconnect valid space. | T: fewer samples/edges may reduce storage; narrow openings and disconnected sampling are key risks. Seeds and targeted passage sampling are needed for reproducibility. |
| T01–T06: zero-distance morphing, envelope clearance, multiple choices, WALK–FLY–WALK | Must explicitly connect compatible mode states at feasible transition poses, including off-grid terrain heights. | Must explicitly insert/connect feasible transition poses; chance sampling cannot guarantee they appear. |
| G01–G06: disconnection, cost preference, ties, scale, unknown policy | T: regular structure helps inspection; volume/resolution growth may dominate runtime and memory. Adaptive rules have not been specified. | T: sparse storage may help scale, but neighbor search, failed collision checks and coverage repair may dominate generation cost. Sampling/pruning rules have not been specified. |
| S01–S03, R01–R02: request validity and route updates | Requires validated endpoint attachment and affected-edge invalidation/revalidation. | Same, with explicit roadmap connectivity repair after updates. |

These are suitability arguments, not comparative performance results. State lattices encode feasible motion only when their primitives satisfy the vehicle model ([Pivtoraiko et al., 2009](https://www.cs.cmu.edu/~alonzo/pubs/papers/JFR_09Final.pdf)). Finite probabilistic roadmaps can miss free-space connectivity ([Kavraki et al., 1996](https://www.kavrakilab.org/publications/kavraki-svestka1996probabilistic-roadmaps-for.pdf)); that literature does not establish completeness for an unspecified sparse/pruned M4 roadmap.

**Analytical scale warning (T):** G04 at uniform 0.5 m resolution has 61 × 61 × 17 = 63,257 flight positions before footprint/bounds filtering; eight flight headings would give 506,056 poses. WALK has 29,768 poses before validation. These are uniform-grid estimates, not adaptive-lattice or roadmap measurements. Flight yaw is an unresolved multiplier.

## Assumptions and pending edits

Working assumptions: one ground height per x/y; heading captures WALK orientation sufficiently; a geometric FLY connection can later be validated against flight limits; mode is part of state identity; both candidates share exactly the same costs, footprints, unknown policy, transition logic and endpoint validation. None establishes robot feasibility. Missing ground prohibits WALK per N04; unknown occupancy is a separate configurable policy per G06.

Before implementation comparison, settle the smallest interface contract: pose reference point and footprint extents; 3D bounds/occupancy access; hard WALK feasibility versus soft costs; unknown semantics; directed transition geometry; endpoint attachment; cost units; and map updates. In particular, do not infer whether z denotes a body center or ground contact from the placeholder dimensions.

Pending edits, **not performed**: align generator/validator signatures; connect PoseState to mode-aware graph identity; preserve scenario defaults/overrides and handle update scenarios; implement shared state/swept-edge validation and WALK primitives; implement the two FLY generators and common transition checker; add scenario assertions and separate graph-generation/search instrumentation. Configure adaptive refinement/connectivity rules versus roadmap sampling, seed, connection and pruning rules explicitly.

Scenario dimensions, permissions and penalties are test values only. R: validate real footprints, morphing envelope, clearance margins, turning/pivot/reverse behavior, yaw and altitude needs, takeoff/landing, slip, localization/perception/mapping errors and actuation delay. Simulation and robot execution must verify paths and costs before operational endorsement.

## Smallest checks needed for selection

1. **Immediate regression:** rerun N01–N03 against source after the edit; assert counts plus every documented check, not counts alone. Retain N04's U label until independent reproduction. N02 does not establish general floating-point robustness; W07's 0.2/0.4/0.8 sweep should also check sampling and boundary behavior. Define rejection of nonfinite resolution inputs; current constructor lacks a finiteness check.
2. **Common validity gate:** implement/check N05/N06, W02/W05/W06/W08, F02–F04 and T01/T02 using one shared validator. Require zero false connections, correct soft costs, and envelope-valid zero-distance transitions. These gates test safety semantics rather than rank the candidates.
3. **First discriminating experiment:** compare both on F01, F05, F06, T03/T05 and G04/G05; reuse W07 for resolution sensitivity. Measure passage connectivity, missed valid routes, volume coverage with an agreed reference, states/edges, generation wall/CPU time and memory. Use identical hardware and policies, resolution/sample-budget sweeps, and repeated recorded roadmap seeds. Measure endpoint attachment and transition insertion costs too. A failed finite roadmap search alone is not proof of physical disconnection.
4. **Selection gate:** choose only if one candidate meets all hard validity checks and the agreed passage-success, latency and memory limits. Those limits are currently unspecified. If this small comparison reveals a clear winner, avoid an exhaustive representation study; otherwise expand only the unresolved scenarios/budgets. G01/G06/S01–S03/R01–R02 remain acceptance checks before deployment.

## Search comparison: gated next stage

No representation has been selected, so the requested post-selection search comparison is **pending**, with no search winner asserted. Once selected, compare the three algorithms already documented: Dijkstra as a nonnegative-cost optimality reference, A* as a heuristic-guided optimal search when its heuristic is admissible, and weighted A* as a speed/route-cost tradeoff requiring measured cost ratios and a correctly implemented bound if one is claimed. Use the same fixed graph, requests, costs and validation for all three. A fourth algorithm is unnecessary until update frequency/latency requirements justify incremental search; any-angle shortcuts would require fresh mode-specific motion, cost and swept-volume validation.

C: current A* prioritizes `(f, g, node_id)`, so ties favor smaller g then node ID; equal-cost alternatives do not replace parents. It permits reopening through improved g entries. Its unscaled Euclidean heuristic is supported by the dummy graph's distance-plus-nonnegative-extra costs, but Graph.Edge enforces only nonnegativity: arbitrary future costs may invalidate that heuristic. Require a proved lower bound `cost(edge) ≥ alpha × endpoint_distance` and use `h = alpha × Euclidean`, or use zero if none is available. Exclude speculative transition penalties from the lower bound.

Post-selection measurements: G01–G03, T04–T06, S01–S03 and stress worlds for success, feasible routes, total cost, expansions/generated nodes, planning wall/CPU time, memory, path/WALK/FLY distance and transition count; R01/R02 for replanning after shared graph updates. Existing metrics use endpoint distances rather than primitive trajectory lengths, count goal pops as expansions, and report process-lifetime peak RAM while excluding graph construction from search timing. Resolve those measurement definitions before ranking algorithms. No search timing or robot-validation evidence was generated here.

## WALK pose convention — accepted test assumption
x/y denote the horizontal footprint center; z denotes ground-contact
height. The WALK body extends upward by the configured footprint height.
Keep this convention in the shared validator. Revisit when integrating
real robot geometry or a costmap, and avoid duplicating checks that the
costmap already performs.

User-run node checks N01–N06 passed. These do not establish footprint,
swept-edge, flight, or transition validity.

## Stage 1 update — shared straight WALK edges (2026-10-05)

**Executed evidence (V): 24 focused pytest checks passed in 0.14 s against workspace source.** Command: `PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=src/m4_global_planner python3 -m pytest -q -p no:cacheprovider src/m4_global_planner/test/test_straight_walk_edges.py`. No package rebuild, flight work, or search comparison performed. Selection remains deferred.

Reused the existing footprint validator, straight-sweep/SAT geometry, scenario parameter merging, and accepted pose convention. Those node/configuration additions were already present on arrival; their original user-run results remain U evidence. The old sampler-only counts are historical, not expectations for the current footprint-aware policy.

| Verified check | Result |
|---|---|
| W01 | Forward edge exists; distance 1 m, configured cost 1. Both endpoints valid. |
| W02 | Valid endpoints; sideways edge absent. |
| W06 | Both endpoints valid; direct edge absent due to intervening swept-body collision. |
| N01–N06 WALK regression | Counts respectively **616, 444, 616, 544, 416, 616**; unique states, configured headings, map-derived z, existing footprint/bounds policy and independent lattice enumeration checked. N02 boundary grid, N03 variable z, N04 missing ground, N05 forbidden centers, N06 retained costly states checked. FLY independence in N05 remains untested. |
| Additional edge checks | Reverse, pivot, changed heading, excessive distance, interior missing/forbidden regions, thin obstacles, footprint-only collisions and touching obstacles rejected; overhead clearance accepted. Invalid endpoint z and invalid configuration rejected. Wrapped headings and changed distance cost accepted. Unsupported terrain cost raises explicitly; non-flat ground has no generated edge. |

**Module changes:** `dummy_map.py` adds exclusion/cost-region access and an explicit flat-ground capability. New `representation/walk_edges.py` separates validation from candidate-pair generation and returns immutable pose-based directed edges; `representation/walk_costs.py` owns distance costs. `test_data/scenarios.yaml` adds `straight_walk` configuration; `test/test_straight_walk_edges.py` reuses existing scenarios. Existing state lattice, state validator and geometry implementations were not modified in this stage.

**Test assumptions/configuration:** forward-only fixed-heading translation, at most 3 m, angular/lateral numerical tolerances of 1e-9 rad/m, and distance cost rate 1 test cost unit/m. These settings are configurable and not robot calibration. Maximum distance includes W06's 3 m candidate so rejection genuinely tests collision. The sweep checks continuous horizontal body volume against all 3D obstacle boxes with touching treated as collision; slight accepted misalignment receives conservative inflation. Existing full-body map bounds and center-path missing/forbidden semantics are preserved. Non-flat edges are conservatively unavailable; N03 node generation still works. Candidate pairs are supplied explicitly; automatic neighbor enumeration and Graph.Node/Edge adaptation remain pending.

**Missing settings/blockers:** terrain cost units/integration, real primitive reach, slope/body orientation and terrain support, safety margins, unknown-space policy, reverse/turn/pivot primitives and graph identity are unresolved. A geometrically valid edge crossing a soft cost region raises `NotImplementedError` rather than silently ignoring cost or labeling expensive terrain physically impossible. Node generation retains those states. Missing/forbidden ground is checked continuously along the center segment; full support-footprint terrain checks remain a robot/map-contract issue. Flat-ground capability currently belongs to the dummy map adapter; other map providers need an equivalent verified contract.

**Pending/next stage:** stop here. Next smallest WALK stage is configured reverse/pivot primitives and their swept validation (W04/W05), including missing cost semantics as needed. Turning routes, narrow-passage checks, neighbor generation, flight candidates, transitions, representation selection and search remain pending. No simulation or real-robot feasibility claim follows from these source-level checks.

## Stage 2 update — configured neighbors and Graph integration (2026-10-05)

**Executed evidence (V): 35 focused checks passed in 0.30 s**, using the same source-only pytest command above. This includes the 24 stage-1 checks and 11 new graph/configuration checks. No rebuild or search execution. This update supersedes stage 1's pending neighbor enumeration and WALK graph adaptation; representation selection remains deferred.

Added `representation/walk_graph.py`: `ForwardWalkNeighbors` indexes lattice states by integer x/y cells and exact generated heading. It computes configured local offsets once per distinct heading, then uses dictionary lookups for each state. It never tries every pair of nodes. `ForwardWalkGraphGenerator` reuses the state generator, state validator and existing edge generator/cost module, adapting poses to `Graph.Node` with WALK mode and yaw and accepted directed edges to `Graph.Edge`. It exposes the actual candidate-attempt count. IDs follow deterministic generator order and are local to each newly generated graph; persistent multimodal/update identity remains pending.

**YAML additions:** `straight_walk.neighbor_grid_steps: [1, 2, 3, 4, 5, 6]` specifies allowed Chebyshev radii in lattice cells. A candidate must also be forward, heading-aligned and within the existing 3 m maximum. Axial offsets are 0.5–3 m with current resolution; diagonal distances grow by sqrt(2), so some configured shells are excluded by that maximum. `grid_index_tolerance: 1e-9` m accepts floating-point grid reconstruction error; off-grid states and duplicate keys are rejected, with no endpoint snapping. Neighbor steps must be unique positive integers. These are test configuration values, not validated robot reach or connectivity guarantees.

| Generated scenario graph (V) | Nodes | Candidate attempts | Accepted directed edges | Required connection |
|---|---:|---:|---:|---|
| W01 | 264 | 452 | 452 | Present; cost 1 for the documented 1 m edge. |
| W02 | 392 | 932 | 932 | Sideways pair never proposed; connection absent. |
| W06 | 240 | 332 | 268 | Documented 3 m pair proposed, rejected by swept collision; connection absent despite valid endpoints. |

Verified every accepted scenario-graph edge against the existing validator and cost calculation; reverse connections at the documented heading remain absent. Repeated generation produces identical nodes/edges. Tests bound candidate attempts by configured local neighbors per state and below all-pairs counts. A one-cell-only configuration removes W01's direct 1 m edge while retaining its two 0.5 m connections; a changed cost rate propagates to Graph edges. Diagonal offsets, invalid neighbor settings and off-grid/duplicate indexing are checked. Counts above were measured separately using the same source modules and scenario fixtures.

**Limits and pending work:** one terrain height per cell and exact generator headings are assumptions of this WALK-only index. Unsupported terrain edge costs still raise through the reused cost path; non-flat terrain still has no forward edges. No new reverse/pivot/turn, flight, transition, unknown-space or search behavior was added. Node regressions N01–N06 remain passing. Next stage remains configured reverse/pivot primitives (W04/W05) and their swept validation, when requested. Stop after this stage.

## Stage 3 update — reverse and pivot WALK edges (2026-10-05)

**Executed evidence (V): 54 focused checks passed in 1.16 s.** Command: `PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=src/m4_global_planner python3 -m pytest -q -p no:cacheprovider src/m4_global_planner/test/test_straight_walk_edges.py src/m4_global_planner/test/test_walk_motions.py`. This retains all 35 previous checks and adds 19 motion/configuration checks. Reachability uses a visited-set graph traversal with no cost optimization, timing comparison, or search-algorithm ranking. No rebuild or flight work.

**Implementation:** added motion-aware validators/generators to `walk_edges.py`, configurable penalties in `walk_costs.py`, a disk/box geometry check in `geometry.py`, and `WalkMotionNeighbors` in `walk_graph.py`. The existing graph adapter accepts these injected modules. Forward-only modules remain available for their original regression checks. Reverse validation reuses the straight swept-body check with endpoints swapped; neighbor lookup adds backward offsets only when `reverse_allowed` is true. Pivot neighbors are colocated configured heading bins in both rotation directions, filtered by `pivot_allowed` and the pivot validator. Bins come from the full configured heading set, not the surviving valid headings, so removing invalid poses cannot silently enlarge an angular step. Indexed enumeration still avoids all-pairs connections.

**Configuration/cost assumptions:** existing `reverse_allowed`, `pivot_allowed`, `reverse_extra_cost: 20`, and `pivot_extra_cost: 8` are enforced. Penalties are finite, nonnegative test cost units **per edge**: reverse cost = configured distance cost + reverse penalty; pivot cost = pivot penalty, with zero translation distance. Subdividing a reverse or pivot maneuver therefore adds penalties; these are not measured energy/time costs. YAML adds `pivot_walk.max_angle_deg: 90` and `neighbor_heading_steps: [1, 2]`, meaning neighboring 45°/90° bins with current headings. Shortest wrapped heading change is checked; angular numerical tolerance is reused. Flags require booleans and pivot settings are validated. Real reverse travel/speed limits and pivot reliability remain robot-validation needs.

**Pivot sweep approximation:** the body rectangle is enclosed by a horizontal disk of radius `hypot(length, width)/2`, extruded upward by body height. For current test dimensions radius = 0.5 m. Bounds and 3D obstacle boxes are checked against that entire cylinder, continuously covering every intermediate orientation; touching counts as collision. This is conservative for partial rotations and may reject a feasible pivot or route. It is not merely endpoint or angular-sample collision checking. Flat ground and the existing center-based terrain-support policy are retained; terrain-aware body orientation/support remains pending.

| Check (V) | Result and evidence limit |
|---|---|
| W05 | Documented 0°→90° pivot edge exists at cost 8 and zero translation; graph goal is reachable when enabled, unreachable when disabled. One-bin neighbors omit the direct 90° edge but allow two validated 45° pivots. Changed penalties propagate to graph edges. |
| W04 reverse diagnostic | Valid diagnostic pose `(4.5, 1, 0, 0°)` reaches `(1, 1, 0, 0°)` only with reverse enabled; a 0.5 m reverse edge costs 20.5. These diagnostic poses are explicit test assumptions, not altered scenario endpoints or silent snapping. |
| W04 documented request — **blocked** | Start `(5, 1, 0, 0°)` has footprint front x=5.4, intersecting the end wall beginning x=5.3, so no valid start node exists. The 0.7 m corridor also cannot accommodate a pivot to the documented 180° goal heading: the 1 m pivot cylinder intersects the corridor walls. Even the valid diagnostic start cannot reach that goal heading. **The original W04 expected route is not verified and is inconsistent with current geometry.** |
| Full-sweep and regressions | Free pivot endpoints with an obstacle hit at an intermediate 8° orientation are rejected; overhead clearance is accepted and out-of-bounds pivot sweep rejected. W01 stays connected, W02's direct sideways edge absent, and W06's direct connections and route remain blocked with both motions enabled. Every accepted regression-graph edge is revalidated and its cost checked. N01–N06 remain passing. |
| Unsupported behavior | Reverse/pivot connections on non-flat terrain remain unavailable; connections touching soft-cost regions still raise explicit unsupported-terrain-cost errors. Flight remains unimplemented. |

**Pending/next stage:** resolve W04's endpoint and turning-space expectations explicitly before claiming scenario acceptance; do not weaken collision checks. A same-heading escape goal would test reverse escape, while a 180° goal needs physically sufficient turning space. Any approved scenario correction remains pending. Turning primitives, terrain costs/support, less conservative pivot geometry if necessary, flight candidates, transitions, representation selection, search comparison and real-robot validation remain pending. Stop after this stage.

## Stage 4 update — W04 correction and executed W03 route (2026-10-05)

**Executed evidence (V): 55 focused checks passed in 1.37 s**, using the stage-3 command. This retains existing focused regressions and executes an additional W03 check. No motion implementation, obstacle, footprint dimension, flight feature, or search comparison changed. Files changed: `test_data/scenarios.yaml`, `test/test_walk_motions.py`, and this report.

**W04 specification conflict resolved:** applied the authorized minimal correction: start x=4.5 instead of 5.0, goal heading=0° instead of 180°. Tests use the corrected YAML endpoints directly and verify both valid. The graph route exists only when reverse is enabled, even with pivots enabled. A validated witness is `(4.5,1,0°) → (1.5,1,0°) → (1,1,0°)`; its reverse edge costs are 23 and 20.5 with the current distance rate and per-edge penalty of 20. Each edge and Graph cost is checked against configured costs. This witness is not a minimum-cost claim.

The original endpoints `(5,1,0°)` and `(1,1,180°)` are preserved explicitly within the W04 tests as an invalid-start diagnostic. The original start is invalid, absent from the generated graph, and cannot produce an accepted edge; the original goal remains valid. The corrected start still cannot reach the original opposite-heading goal in that corridor. Stage 3's blocked status now applies only to this preserved original case, not the corrected W04 scenario. No silent snapping or weakened collision checks.

**W03 executed:** graph reachability passes. The explicit witness `(1.5,2,0°) → (4,2,0°) → (4,2,90°) → (4,4.5,90°)` consists of forward translation, pivot, forward translation. All poses and three full swept edges are validated and present in Graph with matching costs; distances are 2.5, 0, 2.5 m. Every generated W03 edge is additionally revalidated and its Graph cost checked. These results establish heading-aware corner navigation using existing stop-and-pivot primitives, not continuous-curvature turning or real-robot feasibility.

**Remaining work:** unsupported terrain costs/non-flat swept motion, narrow-passage and resolution checks, any future continuous-turn primitives, flight candidates, transitions, representation selection, search comparison, and robot validation remain pending. W04's specification blocker is closed for the corrected scenario. Stop after this stage.

## Stage 5 decision — provisional WALK baseline and FLY/transition contract

**Existing measured evidence (V), not rerun:** the saved [WALK comparison](walk_results/comparison.md) contains 240 final runs: Hybrid 60/60 expected outcomes, lattice/grid 57/60, roadmap 54/60; all successful routes passed audit. The 66-check result retains the original 55. Hybrid is provisional for flat-ground WALK; lattice remains the reference and was faster in dense clutter. Preserve the benchmark's test footprints, stopped-pivot model, conservative pivot cylinder, per-edge penalties, opt-in terrain density assumptions, small repetition count, binned-state limitations and absence of robot validation. Historical pending statuses above are superseded only by the saved measured results. No FLY selection follows from WALK results.

This stage inspected only F01–F06, T01–T06, relevant settings/pending settings, and shared validators/interfaces, plus this report's tail. **No candidate implementation, configuration edits, new test execution or WALK rerun.** The contract below is a design decision/proposal, not verified FLY behavior.

### Minimum common interface

Use existing `Graph`, mode-aware `Node`/`Edge`, configuration merging and scenario data. Keep `PoseState` and the WALK validators; adapt WALK nodes to poses at the mode boundary rather than duplicating WALK geometry or creating another state hierarchy. FLY needs x/y/z and optional yaw through existing Node fields; whether yaw can be omitted is pending.

| Shared boundary | Minimum responsibility |
|---|---|
| Map access | Expose 3D bounds, real 3D obstacles/free-space information and ground height through the existing map adapter. FLY altitude is independently planned; missing ground or WALK-forbidden regions must not automatically prohibit FLY. Ground support still governs WALK and landing anchors. Extend the narrow PlanningMap contract only with capabilities actually used by the validators. |
| FLY state validation | Validate finite coordinates, complete configured body/envelope inside bounds, ground/altitude policy and obstacle clearance. No dependence on either candidate's sampling layout. |
| FLY edge validation | Validate both endpoints and the entire translated 3D body sweep, independently of topology generation. For a fixed axis-aligned test envelope, reuse the slab-intersection idea in `geometry.py` for a 3D segment against obstacles expanded by body extents; the existing x/y-only interval helper cannot be used unchanged. Varying yaw/attitude requires another certified sweep and is initially unsupported. |
| Transition validation | Given a WALK pose, FLY node and direction, check supported modes, identical anchor coordinates, valid endpoint bodies, required ground/support and the entire configured morphing envelope. Morphing contributes zero translation distance. Takeoff/climb and descent/landing motion are separate validated FLY segments; their geometry/dynamics are not established by a clear morphing box. |
| Costs | Keep validation and cost calculation separate. Configure nonnegative FLY distance rate and direction-specific transition penalties in the same test cost units as WALK. Existing `transition_extra_cost: 3` can serve as the provisional per-switch test penalty; no measured transition time/energy is available. T04's terrain preference comes from WALK route costs, not an invented hard transition restriction or a distance charge for morphing. |
| Candidate generation | Both FLY generators return Graph plus a pose-to-ID lookup and use the same state/edge validators and costs. Insert identical validated mandatory request and transition anchors; connect them through bounded, swept-validated neighbors without snapping or all-pairs attempts. Adaptive cells versus seeded samples may differ only in topology/coverage choices. Distinct mode IDs permit a zero-distance transition edge without a Graph self-loop. |

**Integration limit:** Hybrid WALK generates continuous poses during search; its dominance bins are not a prebuilt WALK graph. Do not replace it with a discretized graph to simplify mode changes. A later mode dispatcher must expose validated WALK, FLY and transition successors; transition-anchor coverage and landing headings must remain explicit. Comparing FLY graphs and transition feasibility can precede that multimodal routing integration. FLY without yaw does not define a landing heading: retain/enumerate valid WALK headings at landing anchors until yaw requirements are settled.

### Pending test conventions and missing settings

Propose a **bottom-center FLY/morphing anchor**: x/y is footprint center and z is body bottom, matching the accepted WALK ground-contact anchor. An axis-aligned FLY box extends upward by 0.35 m; morphing extends upward by 1 m. This makes T01's same-coordinate z=0 transition geometrically meaningful. A center-z FLY convention would instead require an explicit coordinate transform and must not introduce fictitious morphing travel. Bottom-center is a pending test convention, not confirmed robot geometry. Ground-level FLY anchors represent geometric mode-change endpoints, not demonstrated airborne flight.

Before implementation, write explicit YAML choices for: anchor/body convention; FLY yaw/attitude policy; floor contact and airborne altitude limits; safety margin/contact tolerances; maximum flight connection distance; FLY cost rate and per-direction transition costs; takeoff/landing availability and unsupported dynamics. Keep current test dimensions unchanged. Proposed initial fixed-axis geometry is valid only if the configured FLY envelope covers the orientation being assumed; a square footprint is not invariant under arbitrary yaw.

Separate candidate tuning from these shared rules: adaptive base/minimum cell sizes, refinement trigger and coarse/fine neighbor rules; roadmap sample budget, retry limit, recorded seeds, connection radius/neighbor count and any sampling bias. Configure common mandatory-anchor generation resolution/headings, endpoint attachment radius, deterministic coverage probes and acceptance/runtime/memory budgets. Do not silently reuse WALK values as FLY requirements. No octree library, new collision framework or broad abstraction is justified before these simple rules are tested.

### Scenario implications and smallest next checks

- **F01:** define what coverage means using common independent probes; state count alone is not coverage. Record construction time, memory and nodes/edges for both candidates.
- **F02:** assert full-sweep collision rejection with valid endpoints, not merely absence caused by a short connection radius. **F03/F04:** check body clearance against ceiling/overhead boxes, not just center occupancy or a height map.
- **F05:** the opening is 1.2 m wide/high versus the 0.8 m wide, 0.35 m high test FLY body. Under the proposed anchor and zero margin, y=3/z=2 is geometrically admissible (analytical, not executed); refinement/sampling must preserve it. **F06:** ensure a higher route exists independently of ground z; minimum climb/dynamic requirements remain unspecified.
- **T01/T02:** distinguish clear endpoint bodies from a blocked morphing envelope. T02's overhead box starts at z=0.75, above both ground-level endpoint bodies but inside the 1 m envelope. **T03:** require full-envelope clearance in the center, not a hardcoded zone; with zero margin and touching blocked, x must lie strictly between 4.5 and 5.5 (analytical, not executed).
- **T04/T05:** preserve multiple feasible transition anchors with soft cost preferences; insert those anchors identically into both FLY graphs. Check WALK–FLY–WALK using reachability after shared geometry gates pass. **T06:** verify WALK remains possible; the 0.6 m ceiling blocks the 1 m morphing envelope. It does not prove that the 0.35 m FLY body cannot fit geometrically.

Proposed next stage: shared FLY body/sweep and transition validation with focused F02–F04 and T01/T02 checks, followed by anchor/connectivity checks. Resolve the test conventions in configuration first. Candidate performance comparison and any real-robot endorsement remain pending. Stop after this interface review.

## Stage 6 update — shared FLY and stationary transition validation

**Executed evidence (V): 94 focused checks passed in 3.23 s**, retaining all 66 existing checks, including the original 55. Command: `PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=src/m4_global_planner python3 -m pytest -q -p no:cacheprovider src/m4_global_planner/test/test_straight_walk_edges.py src/m4_global_planner/test/test_walk_motions.py src/m4_global_planner/test/test_walk_comparison.py src/m4_global_planner/test/test_fly_validation.py`. No candidate generation, performance comparison, benchmark rerun or package rebuild.

**Implemented modules:** new `representation/fly_validator.py` contains shared box-body geometry, `FlyStateValidator.is_valid`, `is_edge_valid` and `TransitionValidator.is_valid`. FLY consumes existing mode-aware Nodes; transitions consume a WALK PoseState and FLY Node. Existing WALK validators remain authoritative for ground endpoints. `geometry.py` generalizes the existing slab helper to 2D/3D, preserving its WALK wrapper. `dummy_map.py` exposes flat floor height and unknown regions through map methods, avoiding validator access to world dictionaries. Added `test/test_fly_validation.py` and YAML validation settings; no costs or motion edges are generated by these validators.

**Configured test assumptions:** `fly_validation.pose_anchor: bottom_center`, fixed axis-aligned box, absent yaw, complete test-box world and flat floor. These are now accepted geometric test conventions, not confirmed robot geometry. Body dimensions remain the existing 0.8 × 0.8 × 0.35 m FLY and 1 × 1 × 1 m morphing boxes. YAML supplies isotropic safety margin 0 m, conservative collision tolerance 1e-9 m, absolute anchor-z limits [0,8] m and straight-edge length limits (1e-9,6] m. Map bounds/floor further restrict the entire body. Bounds never use tolerance to permit outward penetration; obstacle bounds are expanded by tolerance, so touching/near contact is blocked. Margin pads all six body faces: a positive margin excludes zero-height docks under the strict floor check. Floor contact is allowed only where the padded body stays above the floor.

**Supported sweep:** any straight 3D translation of the fixed box within configured limits, including horizontal, vertical and diagonal segments. Obstacles are expanded by the body's asymmetric bottom-center extents and margin; continuous anchor-segment intersection checks the entire translated volume, without point sampling. Valid endpoints guarantee whole-sweep bounds/floor compliance because supported bounds and floor are convex. This is exact for the configured fixed-box model, with conservative margin/tolerance inflation; it is not a flight-dynamics certificate.

**Stationary morphing:** both configured directions require exactly equal anchor coordinates, valid WALK/FLY bodies and complete static swept-envelope clearance. The envelope must cover endpoint bodies; undersized envelopes are explicitly rejected. No positional tolerance permits a moving transition. Morphing has zero geometric displacement; takeoff/landing are separate operations and explicitly unsupported. A ground-level FLY node is only a geometric dock. A clear vertical straight segment does not establish executable takeoff/landing. Transition cost/time and flight cost remain separate future concerns; the existing extra-cost setting is untouched.

| Verified gate (V) | Result |
|---|---|
| F02 | Both documented endpoints valid; 6 m segment rejected by intervening swept collision, then accepted after removing obstacles. Thin body-side obstacles also rejected, proving full-body rather than anchor-line checking. |
| F03 | Body bottom z=0.8 is clear; z=0.9 is blocked because body top crosses the 1.2 m ceiling. Horizontal body bounds and floor penetration are rejected; a zero-height geometric dock is allowed at zero margin. |
| F04 | Body clearance detects the overhead box even when the anchor is below it. Clear below/above endpoints cannot connect vertically through it; a vertical segment outside its footprint is accepted. Diagonal sweeps and body top bounds are also checked. |
| T01/T02 | Both endpoint bodies valid in both directions. T01 stationary morphing accepted; T02 rejected because the larger envelope intersects its overhead box. Even a 1e-12 m anchor displacement is rejected. Morphing bounds in x and z are checked separately from endpoint validity. |
| Configuration/unsupported cases | Margin, altitude/length limits and contact tolerance affect results. Invalid numerics rejected. Unsupported anchor/yaw/ground/unknown/contact/motion policies, explicit yaw states, curved/dynamic/rotating flight, takeoff/landing, non-flat floors and unknown maps raise explicit errors rather than being labeled feasible. |

**Limits and next stage:** known flat test-floor geometry is independent of missing WALK height samples; missing ground still prevents a transition through the existing WALK validator. Any unknown region currently rejects the entire map for FLY, deliberately stricter than region-specific occupancy handling; this is a configured temporary policy, not a final exploration policy. Yaw/roll/pitch changes, altitude-relative-to-ground semantics, real safety margins, calibrated body/morphing envelopes, aerodynamics, takeoff/landing support and simulation/robot validation remain unresolved. F01/F05/F06 coverage, T03–T06 anchor/connectivity behavior and both candidate generators remain pending. The provisional Hybrid WALK baseline and lattice reference are unchanged. Stop after this validation stage.

## Stage 7 update — shared mandatory endpoint and transition anchors

**Executed evidence (V): 105 focused checks passed in 3.04 s**, preserving all 94 previous checks and adding 11 anchor checks. The stage-6 pytest command now also includes `src/m4_global_planner/test/test_anchors.py`. No candidate generation/benchmarking, WALK performance rerun, or package rebuild. Only `representation/anchors.py`, `test/test_anchors.py`, YAML anchor settings and this report changed.

**Common input to both future candidates:** `MandatoryAnchorBuilder.build(map, endpoints)` prepares one immutable, deterministically ordered snapshot with validated exact endpoints, deduplicated FLY Nodes and WALK-heading transition records referring to those nodes. Endpoint inputs are named mode-aware Nodes; WALK requires explicit heading, FLY retains absent yaw. Off-grid endpoint coordinates/headings are preserved. Valid WALK endpoints are additionally tested for morphing at their exact poses, independently of the sampled transition grid; blocked morphing does not invalidate a valid WALK request. Both candidate builders must consume this same snapshot, not independently resample transition anchors.

`insert(snapshot, graph, map)` reuses exact existing FLY poses or inserts mandatory FLY nodes, returning common-anchor-ID → candidate-node-ID mappings. Candidate-local IDs may differ; mandatory pose sets are identical. Insertion is idempotent, does not snap, validates endpoint/body/morphing state again before any mutation, rejects stale invalid anchors and ambiguous duplicate FLY poses, and adds **no edges or costs**. WALK endpoints and landing headings remain snapshot metadata, not a replacement WALK graph. The existing validators perform all physical geometry checks; existing cost modules are untouched. Flight attachment, costed transition edges and mode dispatch remain separate future work.

**YAML additions:** exact endpoint policy, `ground_lattice` transition sampling, independent anchor spatial/heading resolutions 0.5 m/45°, and a 10,000-state sampling budget. The existing WALK pose generator supplies candidate ground poses; shared transition validation filters them. The budget guards the estimated raw sampling grid before generation, rather than silently truncating valid or mandatory anchors. Positive finite settings and required fields are checked. Missing anchor settings fail explicitly; snapping/random transition policies and insufficient sampling budget raise explicit unsupported errors. Sampling resolution does not establish continuous transition-space coverage.

| Supported anchor gate (V) | Result and scope |
|---|---|
| T03 | 11 distinct FLY dock anchors and 88 WALK-heading records, all at x=5 within the full-envelope-clear center. 1,584 sampled WALK states rejected for morphing; both configured directions revalidated. |
| T04 | 209 FLY dock anchors and 1,672 heading records; anchors exist in both soft-cost regions. Removing region costs produces the identical snapshot. This verifies feasibility independence from soft costs, not route preference or new transition pricing. |
| T05 | 98 FLY dock anchors and 784 heading records on both sides of forbidden ground; none inside the inclusive x=[4,6] exclusion. Exact requested WALK endpoints are represented. A supported straight FLY segment between dock anchors at x=3.5 and x=6.5 is geometrically clear; takeoff remains explicitly unsupported. **Executable WALK–FLY–WALK route acceptance remains pending.** |
| Shared insertion | Empty and differently numbered graphs receive identical mandatory FLY pose sets with explicit ID mappings and no edges. Repeated insertion adds no duplicates. Exact off-grid FLY/WALK endpoints, endpoint deduplication, stable input-order behavior, invalid endpoint rejection and stale morphing-envelope rejection checked. |

**Unsupported/missing requirements:** inherited non-flat floor, unknown-map and flight-yaw limitations still reject explicitly through shared validators. No airborne-state promotion, takeoff/landing, costed transition, flight neighbor attachment or multimodal route is claimed. A shared attachment radius/neighbor policy, FLY cost rate/direction costs and candidate-specific refinement/sampling parameters remain unconfigured. Continuous transition coverage and a stable interface from Hybrid's continuous WALK poses to mode successors remain pending. Real landing headings/support, calibrated morphing clearance and robot validation remain unresolved. Next stage can add shared swept-validated attachment/edge costs before implementing the two FLY topologies; no candidate-specific anchor logic is needed. Stop after this anchor stage.

## Stage 8 — adaptive 3D lattice versus sparse 3D roadmap

**Verified (V):** 123 focused checks passed in 7.38 s, preserving all 105 previous checks; log: `fly_results/focused_checks.txt`. No package rebuild or WALK benchmark rerun. New `representation/fly_graph.py` implements both explicit graphs with one shared insertion/connection/cost/audit path; `fly_benchmark.py` is the reusable checkpointed runner; `test/test_fly_graph.py` adds 18 checks. `astar.py` accepts an optional heuristic scale (default unchanged), matching the configured distance rate, including rates below one.

**Saved evidence:** `fly_results/raw.jsonl` has 120 unique completed runs: 10 cases × 2 tuning settings × 2 candidates × 3 repeats. There were 103 returned routes, all continuously swept-validated and cost/endpoint/ordered-waypoint audited; 12 correct G01 no-path outcomes and 5 roadmap coverage failures. No errors, budget exhaustion or timeouts. `fly_results/comparison.md` is the single comparison table. `manifest.json` records source hashes, configuration and host/Python information; `scenarios.yaml` saves the complete input. Ten `anchors_*.pickle` snapshots and accompanying JSON records preserve identical mandatory input: each case uses the same hash across both candidates, both tuning settings and all repeats. Snapshots are trusted local Python artifacts, not an interchange format or external input.

**Generators and shared policy (code-verified):** adaptive lattice uses deterministic hierarchical rectangular-cell centers, subdividing cells intersecting body-expanded obstacle boundaries. Fully occupied cells are discarded; empty cells remain coarse. The configured resolution is an upper bound on leaf axis lengths, rather than an exact uniform step. Existing validation accepts every generated state and every proposed straight edge. The roadmap samples uniformly in body-safe 3D bounds, rejecting occupied samples with the same validator, using recorded seeds 11/29/47. Both insert the exact same immutable endpoint/dock snapshot before sampling; neither generates transition edges. Spatial buckets enumerate local neighbors; the union of each node's nearest 16 proposals within 2.5 m is swept-validated once per unordered pair and inserted bidirectionally. No global all-pairs connections. A* uses the same scaled Euclidean heuristic, heap ties `(f,g,node_id)`, exact endpoints and scalar distance costs for both explicit graphs. Mandatory anchors do not bypass edge validation or neighbor limits.

**YAML additions:** `fly_benchmark` supplies geometric requests, 3 repeats/seeds, 120 s worker timeout, common anchor overrides (2 m/45°, 10,000 raw WALK-state budget), 2.5 m/16-neighbor attachment, distance cost 1 per m, budgets of 15,000 nodes, 300,000 directed edges/connection attempts and 100,000 visited lattice cells. Coarse/fine use 2 m free-space cells with boundary refinement ≤0.5/0.25 m versus 256/1,024 accepted roadmap samples, capped at 30,000 attempts. These benchmark overrides leave stage-7 defaults and regression expectations intact. Required tuning keys have no silent defaults; missing keys fail, invalid positive settings/oversized motion radius fail, exhausted budgets fail rather than silently truncating construction. Additional flight energy, directional costs, dynamics and unknown-space policies have no defined requirements and are not invented here.

**Request assumptions:** original scenario definitions/endpoints are unchanged. F01/F03/F04/G04/G05 receive explicit synthetic FLY queries; F02/F05 keep their documented coordinates. F06 keeps its endpoints and adds the identical exact `(4,2,2.2)` mandatory waypoint to require a higher route (an unqualified shortest path could use the valid low gap). G01/G02 use explicit airborne FLY-only versions, not their original WALK/multimodal requests. FLY body bounds, ceilings, overhead sweeps, missing narrow-opening coverage and disconnected-wall behavior are exercised; focused tests additionally audit all nodes/edges in F01–F06/G01/G02 graphs, deterministic sampling, mandatory insertion, budgets, changed cost rate and invalid-cost rejection. G03 only defines WALK cost regions and is excluded from FLY cost claims; G06 explicitly remains unsupported because shared validation rejects unknown maps. The bottom-center fixed-axis box remains a configurable test assumption, not calibrated robot geometry.

**Measured decision:** coarse adaptive lattice and fine roadmap each achieved 30/30 correct outcomes (27/27 valid requested routes); fine lattice also achieved 30/30. Coarse roadmap achieved 25/30, missing F05 for seeds 11/29 and G04 for all three seeds. Do not use that budget as a baseline. Coarse lattice median total time was 75.6 ms versus 402.2 ms for fine roadmap, with median peak RSS 23.0 versus 26.1 MiB. Fine roadmap had lower matched pooled route cost (12.022 versus 12.968), lower P95 total time (633.9 versus 1,262.1 ms), and faster dense-clutter construction (G05 roughly 0.59–0.83 s versus 1.26–1.52 s). Thus **retain coarse adaptive lattice as a provisional deterministic geometric FLY reference/default for these tests**, alongside the fine roadmap as the measured alternative when dense-clutter latency or route cost dominates. There is no universal winner or real-robot selection. Fine lattice raised median time to 489.5 ms and G05 to 10.8–12.3 s without improving success; it is not justified as the default. Capped nearest-neighbor graphs are not nested across refinements/sample counts, so finer resolution need not lower cost (observed in F02/F05/F06). A* is optimal only within each constructed graph; neither graph proves continuous-space completeness/optimality.

**Measurement limits:** construction includes the identical recorded shared-anchor preparation time plus candidate insertion/generation/connection; raw rows also separate candidate-only construction/total, planning and audit. Excluded costs are interpreter startup, worker YAML/snapshot I/O and controller overhead. Shared preparation is measured once in its own fresh worker and charged equally, not independently retimed per repetition. Per-run memory is fresh-worker Linux peak RSS, including imports, compared with the shared preparation peak; raw rows retain worker baseline/high-water growth. This is not allocator/live memory or memory solely owned by graph nodes. Host CPU was not pinned or isolated; focused regressions overlapped part of the late benchmark, so small timing differences are not significant. Three seeds on these cases establish observed robustness only. Ground-level geometric dock anchors are retained and can participate in geometric paths; such segments do not establish takeoff/landing feasibility. No real-robot validation is reported; previous user-run N04 results remain user-reported.

**Pending / next bounded work:** takeoff/landing requirements, calibrated safety margins/body/envelope, executable multimodal routing and landing headings, unknown/non-flat terrain, energy/dynamics and simulation/robot validation remain unresolved. A separate search comparison remains pending; this stage compares representations using one existing search only. Next useful correctness check is additional narrow-passage layouts/seeds and required altitude policies once specified; repeat performance only for a concrete deployment workload on an isolated host. Hybrid A* remains the provisional flat-ground WALK baseline, with the lattice reference and previous limitations retained.

Reproduce/resume without rebuilding: `PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=src/m4_global_planner python3 -m m4_global_planner.fly_benchmark --output fly_results` (existing checkpoints skip completed runs; changed source/config requires a new output directory). Use `--case INDEX` for a bounded stage. No further features are implemented here.

## Stage 9 — focused graph-relative optimality audit

Added `test/test_graph_optimality.py`; no planner implementation, YAML, benchmark configuration or saved benchmark output changed. Focused run: **34 passed in 8.90 s** using the command in `NEXT_STEPS.md`. The audit compares independent Dijkstra with A* on the same constructed explicit graph: W01's WALK lattice and both FLY graph candidates on F05/F06, including each leg through F06's required higher waypoint. FLY successful paths are checked again with the shared state, swept-edge and cost validator; the WALK planner's existing endpoint and edge audit remains active.

**Guarantee:** for these finite graphs and requests, A* returns the minimum cost in that graph (Dijkstra agrees), subject to the configured graph edges and costs. This says nothing about continuous-space completeness/optimality or which representation is best.

**Still unverified:** grid and roadmap WALK planning uses spatial graphs to propose neighbors, then lazily generates heading-aware successors and connector costs. Those costs and states are not in the exposed spatial graph, so a shortest-path result on that graph would not audit the graph the planner searches. Hybrid A* also does not expose its complete finite successor graph; its coupled search cannot receive this audit yet. Capturing or exposing each actual finite search graph is the smallest prerequisite. No Hybrid/FLY provisional selection, benchmark result, or route claim changed.

## Stage 10 — matched cost-optimality audit

Added optional `capture_search_graph` to WALK planning for audit runs. It records heading-aware successors and connector costs and, after retaining A*'s goal path, continues until the finite reachable graph is exhausted (goal is terminal). Default planning and saved benchmark output are unchanged. Independent test-only Dijkstra searches this captured graph and reconstructs stored motion segments for validation. The complete WALK lattice and both other reachable successor graphs are checked on G02. FLY is checked directly on the complete constructed coarse graph. Focused regressions: **130 passed**, preserving all 123 previous checks. No benchmark was rerun; Hybrid comparison below uses saved `walk_results/raw.jsonl`.

| Representation / case | Audited graph | A* cost | Dijkstra cost | Route check |
|---|---:|---:|---:|---|
| WALK lattice / G02 | 1,120 nodes / 10,634 edges (complete) | 32.0711 | 32.0711 | Exact endpoints and motion edges valid |
| WALK grid / G02, seed 11 | 1,574 reachable states / 28,186 arcs | 32.0711 | 32.0711 | Reconstructed heading-aware segments valid |
| WALK roadmap / G02, seed 11 | 2,473 reachable states / 45,640 arcs | 112.5144 | 112.5144 | Reconstructed heading-aware segments valid |
| FLY adaptive lattice / F05, coarse seed 47 | 566 nodes / 10,244 directed edges | 4.0331 | 4.0331 | Full swept route valid |
| FLY sparse roadmap / F05, coarse seed 47 | 262 nodes / 5,002 directed edges | 4.3266 | 4.3266 | Full swept route valid |
| FLY adaptive lattice / F06, coarse seed 47 | 93 nodes / 1,776 directed edges | 8.6628 total | 8.6628 total | Both high-waypoint legs valid |
| FLY sparse roadmap / F06, coarse seed 47 | 261 nodes / 4,860 directed edges | 7.7394 total | 7.7394 total | Both high-waypoint legs valid |

All audited A* costs match Dijkstra. G02's spatial proposal graph alone gives costs 7.000 (grid) and 7.837 (roadmap), far below planner costs 32.071 and 112.514: those graphs omit heading-aware connector/pivot costs. They are not valid cost references. The planner's captured motion arcs include those costs. These checks establish cost minimum on the complete reachable finite graph under the configured successor rules; they do not establish continuous-space completeness or optimality.

**Hybrid assessment from saved matched runs:**

| Matched comparison | Outcome counts (both / Hybrid only / lattice only / neither) | Hybrid relative cost gap on both-success pairs |
|---|---:|---:|
| Hybrid vs lattice, 60 case/repeat pairs | 36 / 3 / 0 / 21 | Mean -0.002%, median 0%, range -0.30% to +0.34% |

In G05 dense clutter Hybrid costs 0.34% more; in G03 it costs 0.30% less. No material cost winner appears in these saved matched cases. Hybrid retains continuous pose values but keeps one best representative per 0.25 m XY bin and 45° heading bin, with 0.5–3.0 m motion steps and an 80,000 expansion limit. The lattice reference uses 0.5 m spatial and 45° heading spacing. These discretizations, bin dominance, finite expansion limit, and the limited successful overlap make the comparison reference-relative; it is not a proof of Hybrid optimality.

The existing provisional Hybrid WALK and adaptive-lattice FLY selections remain unchanged. Executable multimodal minimum-cost planning and real-robot assumptions remain pending.

## Stage 11 — joint geometric WALK–FLY graph (2026-10-06)

Continued from committed checkpoint `b591515`; this stage is uncommitted. Reused the
explicit WALK lattice, primitive validators/costs, shared mandatory anchors and existing
FLY graph builders. Hybrid remains a separate provisional WALK planner. One A* query
chooses mode sequence and transition locations by total cost over the complete configured
joint graph. Only new exact WALK poses get local bidirectional composite connectors;
the existing lattice backbone retains its primitive edges. Connector primitive paths are
retained and audited. YAML `joint_planner` settings reuse FLY configuration through aliases.

Supported transitions are exactly directed `walk_to_fly` and `fly_to_walk` stationary
morphs at equal x/y/z, with a valid heading-aware WALK pose, yaw-free FLY pose and clear
fixed axis-aligned morphing envelope containing both bodies. Direction policies can enable
only one direction. FLY translation may touch the known flat floor, so bridge tests can
use ground-level FLY geometry. A ground morph followed by a geometric translation does
not establish takeoff; a translation followed by a ground morph does not establish landing.
Takeoff, landing, attitude/yaw evolution, dynamics, energy and physical execution are
unsupported. The envelope and costs are test values, not measured robot properties.

Independent test Dijkstra traverses the identical complete constructed joint graph, with
identical endpoints and edge costs. Every successful A* and Dijkstra route is checked for
exact endpoints, state validity, each primitive WALK sweep, each FLY sweep, each stationary
morph, individual edge costs and summed route cost. Euclidean heuristic scale is the minimum
WALK/FLY distance rate; nonnegative terrain and motion/morph penalties preserve admissibility.
The proof is graph-relative, not continuous-space or physical optimality.

| Existing scenario / configuration | Verified outcome |
|---|---|
| T01 open | Joint route; exact mixed-mode and off-grid endpoints preserved. |
| T02 blocked morph | No morph at (2.5, 2.5); a WALK→FLY request there travels to a clear anchor and returns geometrically. |
| T03 center zone | All inserted morphs remain at x=5. |
| T04 multiple choices | FLY rate 0.2, morph penalty 0: cost 1.6, morphs at (1,2) and (9,2). Penalty 100: WALK-only cost 10 with terrain costs. Restricting first morph to x≥5 raises cost and shifts its location. |
| T05 required bridge | Adaptive FLY graph: 914 nodes / 10856 directed edges; cost 14.823090672 with penalty 3 and two morphs. Both FLY topologies match Dijkstra at penalties 0, 3, 17; penalty increments add exactly twice the increment. One-way WALK→FLY policy yields no WALK-goal route. |
| T06 walk only | Cost 8, no morph anchors, no mode changes. |

Additional checks preserve exact WALK lattice adjacency/costs at T05, cover a trivial
identical endpoint, and reject corrupt totals, corrupt edge prices, stale morph clearance
and invalid penalties. All 23 new joint tests pass. `colcon build --packages-select m4_global_planner`
succeeded. Full `colcon test` passed 156 tests (including flake8 and pep257), with
one existing copyright-template skip; `git diff --check` is clean.

Pending: separately validated takeoff/landing, calibrated envelopes/attitude/physical costs,
unknown/non-flat world support, joint waypoint and map-update integration, WALK construction
resource budgets, coverage and performance measurement. Sampling and finite neighbor policies
can still cause false disconnection; no-path means no route in this graph. Synthetic endpoint
requests were supplied for T scenarios lacking endpoints; T04 uses the existing terrain regions
and the established synthetic distance-density interpretation. Historical benchmark outputs
were preserved; no selection or physical execution claim follows from these checks.

## Stage 12 — matched Dijkstra, A* and weighted A* (2026-10-06)

The comparison is saved in `joint_search_results/comparison.md` (one concise table),
`raw.jsonl` (all 144 timed runs), `graph_*.json` / `graph_*.pickle` (12 once-built
configuration snapshots), `manifest.json`, copied YAML, and `audit.json`. The runner is
`python3 -m m4_global_planner.joint_search_benchmark --output <new-directory>`.
It reuses `JointPlanner`, existing geometry/costs/anchors, scenario loading, and the
existing `metrics.run_with_metrics` timer. Weighted A* extends the existing A* loop with
`heuristic_weight`; Dijkstra uses an independent uniform-cost loop. Neither search mutates
the graph. Hashes verify ordered nodes/edges, endpoints and WALK connector primitives
before and after searches; every method loads the same serialized graph per configuration.

Three fresh workers per method/configuration ran serially, with method order rotated
between repeats and no concurrent tests. Weights 1.5 and 2 are set in YAML. Construction,
preparation/reference audit, search, route audit and I/O are separated. Search timing uses
an uninstrumented call; an identical separate tracemalloc replay measures Python search
allocation peak. Timed-worker RSS high-water, pre-search baseline and growth are also saved.
RSS includes imports and snapshot loading and can hide search allocations; tracemalloc
excludes the pre-existing graph and is not total RAM. All timed/replayed successful paths
and costs are revalidated. Unique explored states, total pops (including re-expansions),
generated states, wall/CPU time and per-run memory appear in raw rows.

Cases cover T01–T06, T04 low/high transition penalties, T05 adaptive/roadmap/one-way
variants, and supported G04 scale, G05 clutter and G01 disconnection. T requests without
endpoints are explicitly configured synthetic requests. G04/G05 use 1 m / 90 degree WALK
and anchor sampling to bound construction; those graphs are not the earlier fine-resolution
benchmark graphs. All methods matched expected outcomes: each returned 30 valid routes in
36 attempts, with six correct no-path results. Across 12 graphs, all 269570 directed edges
passed the heuristic checks: distance times the minimum movement rate is no greater than
edge cost, h(goal)=0, and goal-specific consistency holds within 1e-9 (maximum floating
residual 7.11e-15). Nonnegative penalties, zero-displacement morphs and Euclidean triangle
inequality establish the analytical lower bound for these graphs. A* matched independent
Dijkstra minimum costs in all 36 runs. Weighted gaps are measured only; no weighted bound
or continuous-space optimality claim is made.

Construction median was 363.9 ms (165.6–1819.9 ms). Pooled search medians were 1.620 ms
(Dijkstra), 1.544 ms (A*), 0.538 ms (weight 1.5) and 0.527 ms (weight 2). Maximum measured
weighted cost gaps were 5.574% and 19.148%. Matched cases expose tradeoffs hidden by pooled
medians: G04's 5229 nodes / 54876 edges cost 45.597980 for both exact methods, with median
search 12.495 ms for Dijkstra and 4.620 ms for A*. Weighted costs were 47.406689 and
54.329002. G05's 2360 nodes / 38388 edges cost 23.549618 for both exact methods, but
Dijkstra took 6.211 ms versus A*'s 12.480 ms. Weight 1.5 took 27.768 ms and 5382 pops for
928 unique explored states, versus Dijkstra's 2317 pops; weight 2 took 7.272 ms and cost
27.162610 (15.342% gap). Faster exploration is not guaranteed by a larger weight or fewer
unique explored states. Method tie-breaking uses numeric node IDs; fixed graph order and
seed preserve repeatable route/cost/count results.

Recommend Dijkstra as the simplest exact-cost method for the current bounded workloads:
its per-case median search times were 0.175–12.495 ms, graph construction dominated, and
no hard latency target or acceptable suboptimality threshold was supplied. Retain A* as
the exact alternative for larger or repeatedly queried graphs where its heuristic savings
are measured; it clearly helped G04 but regressed G05. Keep weighted A* experimental until
an acceptable cost gap and workload-specific latency requirement are defined. This is a
recommendation from the comparison; the existing `JointPlanner.plan()` A* API remains
unchanged. Hybrid's separate provisional WALK status is also unchanged.

Validation preserved all 156 prior passing checks and added 12 focused search/benchmark
checks. One full regression run passed 167 tests with the existing copyright skip and a
formatting-only lint failure; the formatting was corrected and lint plus the affected
benchmark checks rerun successfully. Final package build and focused checks are recorded
in `joint_search_results/validation.md`. No tests ran concurrently with measurements.
Results are local WSL2 host observations, with only three repeats and uncontrolled other
host load; P95 values are empirical nearest-rank summaries, not latency guarantees.

Remaining limits: sampled finite graphs can falsely disconnect continuous free space;
heuristic effectiveness varies with cheap FLY rates, costs, headings and clutter. Exact
costs apply only within each identical configured graph. Unknown/non-flat maps, calibrated
body/morphing/attitude models, takeoff/landing, dynamics, physical execution, joint waypoint
ordering, map-update integration and WALK graph resource budgets remain unsupported or
pending. Shared saved graphs remove rebuild noise but exclude cold graph loading and
construction from search timing; repeated live graph-query performance and stronger scale
or latency requirements remain unmeasured. No commit or push was made.

## Stage 13 — Dijkstra public default (2026-10-06)

`JointPlanner.plan(start='start', goal='goal')` now dispatches to the existing Dijkstra
implementation by default. YAML `joint_planner.search_method` selects `dijkstra`, `astar`
or `weighted_astar`; `weighted_astar_weight` configures the latter (default 1.5, finite
and at least 1). Omitted search settings retain the Dijkstra default. Ordinary A* always
uses weight 1 and the existing minimum movement-rate heuristic. Invalid methods and
weights are rejected before graph construction. Result fields, endpoint naming, graph
construction, directed morph validation and per-route primitive/cost audit are preserved.
No additional algorithm or dispatch abstraction was introduced.

Twenty public-interface checks cover all three configured methods on T06 WALK-only,
T05 required flight bridge, T02 detour around blocked morphing, and T04 low/high penalty
mode choices; they also verify YAML/default fallback and reject invalid configurations.
They check the selected implementation and heuristic weight, audit every returned segment
and total, and compare exact methods to independent test Dijkstra on the same graph.
Weighted results are checked for valid cost and route without asserting optimality or a
weighted bound. Existing directed/no-path, exact endpoint and corruption checks remain.

Dijkstra and admissible-heuristic A* minimize only the cost of the constructed finite graph
with configured movement, terrain and transition costs. Weighted A* may return a higher
cost. Stationary WALK→FLY and FLY→WALK morphs remain geometric operations; takeoff,
landing, dynamics and physical execution are unsupported. Sampling coverage, calibrated
costs/envelopes/attitude, waypoint/map-update integration, WALK construction budgets and
operational latency/cost-gap requirements remain pending. Hybrid stays separately
provisional. The Stage 12 saved measurements were preserved; no benchmarks were repeated.
The stage was validated before the publication review below.

Verification: `colcon build --packages-select m4_global_planner` succeeded; full package
`colcon test` passed **188 checks**, including flake8 and pep257, with one existing
copyright-template skip. All 168 prior passing checks are preserved. No benchmark output
was regenerated. `git diff --check` is clean.

## Stage 14 — publication review against b591515 (2026-10-06)

Reviewed only the planner integration, configurable searches, tests, YAML, documentation
and saved joint benchmark artifacts added or changed after `b591515`. Left the unrelated
untracked `src/m4_first_test/` package and empty top-level planner `dummy_map.py` untouched
and outside the commit. Existing tracked WALK/FLY results and all saved joint measurement
files were preserved; no benchmark was rerun.

No duplicated graph builder or weighted search loop was introduced. Dijkstra remains
independent of the A* search loop for the cost reference; its reuse in public planning
is intentional. Search selection, movement/morph penalties, representation settings,
benchmark weights/repeats/requests and sampling budgets are configurable. The review moved
the joint audit's previously embedded relative/absolute cost tolerance into YAML
`joint_planner.cost_tolerance` (default 1e-10 for existing callers), with finite-positive
validation and focused regressions. Numerical comparisons allow this configured tolerance;
optimality claims remain limited to the constructed graph and configured costs.

Saved benchmark manifests, copied YAML and `audit.json` record the measurement-time source
and settings. Later default-search and review changes intentionally differ from those source
fingerprints; the historical `source_hashes_match` field describes the audit at measurement
time, not the newly published sources. Existing-directory resume remains protected against
source/config changes. Local pickle snapshots are historical trusted artifacts for the
benchmark worker's direct graph queries; their old planner instances predate later settings
attributes. No claim of physical takeoff, landing, dynamics, continuous completeness or
weighted cost bounds is supported. Those limits and operational requirements remain pending.

Review verification: package build succeeded and the affected joint-planner/search/lint
checks passed **62 tests** (132 deselected). This includes five new tolerance regressions;
the preceding full package run passed 188 checks with one existing copyright skip.
Saved-result integrity checks confirmed 144 validated rows and all 12 snapshot hashes.
No measurements were repeated. `git diff --check` passed before staging.
