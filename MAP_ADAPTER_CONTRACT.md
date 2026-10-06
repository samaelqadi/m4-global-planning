# Provisional map adapter contract

The likely future input is a **2D costmap with height information**, but its exact format,
frames, units and vertical-knowledge semantics are unavailable. `HeightGridMap` is a mock
schema, not an upstream integration specification. See [REVIEW_HANDOFF.md](REVIEW_HANDOFF.md).

## Shared queries and storage boundary

[PlanningMap / MapRegion](src/m4_global_planner/m4_global_planner/representation/interfaces.py)
now describe the operations already needed by WALK, FLY, morphing, cost integration and
replanning: bounds; ground height; conservative occupied volumes; WALK center exclusions
and cost regions; unknown coverage; flat-floor capability/height; revision and native
snapshot updates. Optional point queries expose occupied height intervals and obstacle top.
The current collision/sampling implementation still consumes axis-aligned geometry; this
interface does not promise support for arbitrary meshes or height-varying trajectories.

Audit found that `PlanningMap` declared only two methods while actual callers required the
larger contract. Geometry, WALK validators/edges, FLY body validation and adaptive sampling
read dummy `min`/`max`/`cost` keys. Those callers now consume typed `MapRegion.lower`,
`.upper`, `.cost`, with existing shared intersection/footprint/sweep algorithms unchanged.
No planner reads adapter `.world`, `.cells` or native update payloads. Box dictionaries
remain solely in [DummyPlanningMap](src/m4_global_planner/m4_global_planner/dummy_map.py),
including its legacy `get_obstacles_3d` inspection method. Mock rows/columns remain in
[HeightGridMap](src/m4_global_planner/m4_global_planner/height_grid_map.py).

`MapRegion` is a geometric query value, not a storage format. Adapters supply conservative
raw occupied volumes; geometry is closed and contact counts as collision. Region cost is
extra WALK distance density, independent of occupancy. Existing box cost regions remain
additive when overlapping. Grid cost regions have half-open upper bounds: a path exactly
on a cell seam belongs to one cell and is not charged twice. Footprint inflation does not
apply to cost integration or the existing WALK center-exclusion convention.

## Mock schema and vertical knowledge

YAML [height_grid_mock](src/m4_global_planner/test_data/scenarios.yaml) configures resolution,
x/y origin, absolute map-z vertical bounds, default nonnegative cost, `block` or strict `reject_map` unknown
policy, `raw_geometry` inflation policy and `map_z` height reference. Caller-supplied
`cells[y][x]` contain:

| Field | Meaning |
|---|---|
| `ground_height` | Absolute supporting surface z, not obstacle height. Missing/null makes coverage unknown. |
| `occupied_intervals` | Explicit absolute `[bottom, top]` occupied z intervals for the whole cell. Multiple layers retain gaps. `[]` explicitly states no obstacle in the configured vertical domain; absent/null is unknown. |
| `obstacle_top_height` | Optional highest occupied top, informational only. With known intervals it must agree; without intervals it never certifies free space below or above. |
| `cost` | Extra WALK distance density; omitted uses `default_cost`. Not a ROS occupancy/costmap byte or physical energy. |
| `walk_forbidden`, `unknown` | Explicit center exclusion or incomplete coverage. |

Ground-contact floor remains separate from obstacle intervals. Unknown columns are occupied over the entire configured vertical domain under `block`
(the mock default), for both WALK and FLY. `reject_map` retains whole-map rejection.
FLY configuration uses `unknown_policy: map_policy` to follow the adapter; the legacy FLY
`reject_map` option remains strictly rejecting. Box maps configure their policy in `world`;
their default stays strict for compatibility. Missing vertical information never certifies
free flight. Only a common flat known floor is supported; unknown ground columns can be
blocked if other cells establish that floor, but an entirely missing floor is rejected.
Non-flat known heights, other unknown policies, relative heights and preinflated grids are
rejected. All grid point queries use half-open cell ownership `[lower, upper)` including
outer bounds: occupied intervals are `None` outside/unknown, `()` only for known clear cells.
Obstacle-top queries derive from those same known intervals; `None` alone is not a clearance
statement. Ground queries return `None` outside or where ground is missing. Collision sweeps
still use closed volumes, including both touching neighboring cells, independently of point
ownership. Interval query typing explicitly permits `None`; current callers are tests and
the focused diagnostic, not the route search. Occupancy knowledge is bounded by the configured
3D domain; it makes no claim about space outside those bounds.

**Inflation ownership:** adapters return raw uninflated geometry. Existing WALK footprints,
straight sweeps and pivot disks, and FLY/morphing `BoxBody` margins/tolerances do the body
clearance checks. Do not feed a preinflated costmap as raw occupied geometry: applying a body
again would double inflation. Grid input explicitly rejects a preinflated policy; box inputs
retain their established raw-geometry convention. The upstream producer must declare its
inflation/occupancy meanings; silently guessing or undoing inflation is unsupported.

## Updates, evidence and pending integration

The shared `apply_update` method returns a new validated snapshot/revision, but payloads
belong to each adapter. Box updates keep their supported region operations; grid updates
replace complete cells with `set_cells: [{index: [x, y], cell: {...}}]`. Bad indices/data
leave the old snapshot intact. Existing joint replanning rebuilds everything and uses
Dijkstra for either adapter; stale-result, failed-rebuild and non-executable guards remain.

[Adapter tests](src/m4_global_planner/test/test_map_adapters.py) compare exact nodes/adjacency
and edge costs on equivalently encoded supported worlds, then audit both returned paths
and independent Dijkstra on each graph. They cover obstacle volumes, terrain densities,
overhead/morph clearance, missing/top-only heights, explicit unknowns, multi-layer gaps,
origin/resolution, policy rejection and equivalent snapshot updates. This establishes finite
geometric equivalence for those cases, not equivalence for arbitrary rasterizations.

Still needed: real costmap schema/transport and timestamps; coordinate frames and z datum;
cell occupancy thresholds, costs/normalization, unknown and completeness semantics; how
height data covers free and occupied intervals (including overheads); grid boundary and
rasterization conservatism; support-surface interpretation; inflation provenance; live-update
versioning; localization/uncertainty and controller limits. Native update records are not yet
standardized. Performance, caching/indexing and real grid feeds remain unmeasured. Physical
takeoff, landing and execution remain unsupported; exact optimality remains graph-relative.

## Review fixes and focused performance

Immutable region tuples, sparse per-cell bins and cached floor knowledge are built once
per grid revision. Shared validators and cost integration request conservative local bounds;
box maps use a simple filtering fallback. Grid queries visit only cells intersecting those
bounds (plus lower-seam neighbors for closed contact). Public grid settings/cells return copies;
updates construct independent snapshots and caches. Whole-map sampling can still use full
cached geometry, and long query rectangles can include many cells. Construction/cache memory
and worst-case sweeps remain unmeasured; no indexing framework or region-merging scheme was added.

Independent tests now cover seam/outer-boundary/unknown point results, both unknown policies,
full-height blocking, known-space routes, snapshot isolation/invalidation, seam costs and
non-flat rejection. Clean current full run: 251 passed, one existing copyright skip; build
and lint passed. [Focused query samples](adapter_query_results/README.md) compare warm local
queries before/after fixes, excluding construction. They do not replace historical planner
benchmarks or establish operational budgets. The real input schema remains unavailable;
mock-based development can continue without inventing it.

## Publication evidence

The clean 251-pass result is the recorded **2026-10-06** adapter-fix run, not a new run
for this commit. A portable copy is in [test_evidence/2026-10-06_adapter_fixes](test_evidence/2026-10-06_adapter_fixes/README.md).
Git/uncommitted descriptions above are the review snapshot before publication from
`61fded2`. Review ZIPs, their manifests/extracted packets and VERIFICATION.txt remain
local-only and are excluded from publication. No new tests or benchmarks were run.
