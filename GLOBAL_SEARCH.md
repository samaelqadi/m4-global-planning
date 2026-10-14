# Global planner

## Current solution

**IMPLEMENTED — supplied-graph prototype; team integration remains open.**

Our input is a complete graph from the graph-generation task. It already contains WALK and FLY nodes, valid directed movement and transition edges, and their costs. We choose the lowest-cost route through that graph. We do not generate nodes, check robot clearance, create transitions, or calculate terrain and motion prices again.

Dijkstra is the default search. Ordinary A* is an alternative. Both aim for the same minimum supplied cost. Dijkstra is the simple baseline, not a proven speed winner for the future robot workload. The final producer message and controller message are not connected yet.

There are two modes, WALK and FLY. A transition is an edge between them. One search chooses the entire route, including where to change mode, from the connections supplied upstream.

```text
Mapping / cost information
          |
Graph-generation task
  complete nodes, valid directed edges, costs, revision
          |
Global planner
  validate and copy input -> search -> audit selected edges and total cost
          |
Route: ordered nodes and edges, mode changes, cost, revision, success/no-path
          |
Local planning / controller
```

## Your global-planner status

**Implemented:** accept and copy a complete graph snapshot; validate graph structure and supplied costs; search it with Dijkstra or A*; return and audit the minimum-cost supplied route; accept replacement snapshots and identify stale results.

**Still in your part:** agree the exact producer and controller message schemas; convert the producer's complete graph and costs into the `Graph` input; agree how the current position and goal are attached and represented by node IDs; connect graph updates and route results to their consumers; and test this wiring with real producer graphs, updates and agreed performance budgets.

**Not in your part:** create or sample nodes, generate WALK/FLY connections or transition edges, calculate edge costs, check map/robot collision feasibility, or execute the route. Those belong to graph generation and local execution. If an expected route is absent from the supplied graph, investigate graph generation; if the route exists in the supplied graph but search fails to find its minimum cost, investigate this planner.

## Responsibility boundary

| Work | Owner |
|---|---|
| Mapping, costmap and environmental data | Upstream tasks |
| WALK/FLY representation, sampling and connections | Graph-generation task |
| Collision, slope, step, gap, clearance and robot-motion feasibility | Graph-generation task |
| Transition locations, morphing clearance and allowed directions | Graph-generation task |
| Complete edge prices and their units | Graph-generation task; shared interface agreement |
| Attach robot/goal poses and supply their node IDs | Required upstream handoff; exact integration still to agree |
| Validate the supplied graph structure and numerical values | Our global planner |
| Select the minimum-cost whole route and check its edge-cost sum | Our global planner |
| Accept replacement graphs and invalidate old results | Our global planner |
| Execute motions and handle immediate obstacle avoidance | Local control / execution tasks |

Receiving a graph does not establish that its edges are physically safe. That is the supplier's responsibility. Our check establishes that the selected route uses supplied directed edges and adds their supplied costs correctly.

## Code and use

| File | Purpose |
|---|---|
| `src/m4_global_planner/m4_global_planner/graph.py` | Node, edge and directed graph data types; no sampling or graph-building algorithm |
| `src/m4_global_planner/m4_global_planner/astar.py` | Dijkstra and ordinary A* search |
| `src/m4_global_planner/m4_global_planner/global_planner.py` | Input snapshot validation, search selection, route audit and updates |
| `src/m4_global_planner/m4_global_planner/__init__.py` | Python package marker |
| `src/m4_global_planner/test/test_global_planner.py` | All active automated tests and an independent shortest-cost reference |
| `examples/supplied_graph_demo.py` | Small supplied-graph example: initial route, cost change and blocked route |
| `pyproject.toml` | Installation and test configuration; no ROS or runtime third-party dependency |
| `GLOBAL_SEARCH.md` | This report: architecture, decisions, history, tests and remaining work |
| `GLOBAL_PLANNER_FINAL_HANDOFF.md` | Detailed engineering handoff, contracts, tests, evidence, limitations and integration checklist |
| `evidence/global_planner/` | Raw test records and historical search evidence; not runtime inputs |
| `.gitignore` | Excludes local environments and generated Python/build files from new tracking |

The local `.venv` is the Python environment, not planner source. It is retained so the project can still run. The old package layout is kept to avoid needless import changes; it does not require ROS. The cleanup removed 398 approved files after verifying all 403 archived originals (removed or replaced files). The recovery archive is outside GP; its location and hash are recorded in `evidence/global_planner/scope_cleanup/removals.json`.

```python
from m4_global_planner.global_planner import GlobalPlanner

planner = GlobalPlanner(search_method='dijkstra', cost_tolerance=1e-9)
planner.set_graph(supplied_graph, revision='graph-1')
result = planner.plan(start_id, goal_id)
result = planner.replan(updated_graph, 'graph-2', current_start_id, goal_id)
```

From the project folder in PowerShell:

```powershell
.\.venv\Scripts\python.exe -m pip install -e ".[test]"
.\.venv\Scripts\python.exe -m pytest -q
.\.venv\Scripts\python.exe examples\supplied_graph_demo.py
```

## Input and output contract

The internal input is an explicit `Graph`. It is a prototype Python interface, not the final upstream transport format.

- A node has an integer ID. Coordinates and WALK/FLY mode metadata are optional; if all three coordinates are provided, they must be finite. Partial coordinates are invalid. Heading is optional and finite when provided.
- An edge has a source ID, target ID and finite nonnegative cost. Its endpoints must exist.
- Each ordered node pair has at most one edge. Self-edges are rejected. If upstream needs several actions between the same pair, the interface must be extended before integration; those actions must not be silently discarded.
- Adjacency entries and edge IDs must agree. Invalid edges must be excluded by the supplier. If an incoming edge explicitly says it is invalid, the snapshot is rejected.
- A revision is an integer or nonempty string. Start and goal must already be node IDs in the accepted graph. Missing endpoints raise an error; the planner does not guess a connection.
- Lazy graph builders are rejected. Search must not trigger upstream graph generation.

Each result contains success/no-path, an ordered node-ID path, node and edge objects, mode-changing edges, total cost, revision, internal snapshot ID, search method, search time and search counters. Edges currently use their endpoint pair as identity; there is no separate action ID or detailed trajectory payload. Those fields require agreement with the supplier and controller.

No-path returns an empty path and infinite cost. Start equal to goal returns that node and zero cost. A malformed input raises an error instead of becoming a no-path result.

## Search and cost checks

Dijkstra expands the cheapest reached route so far. A* also uses an estimate of the remaining cost. It does not change edge prices.

For A*, when every node has coordinates and every endpoint distance is finite, the wrapper finds `alpha = min(edge cost / endpoint distance)` over edges with positive endpoint distance. It uses `alpha × straight-line distance to goal` as the estimate. Every supplied edge costs at least its scaled endpoint distance, so the triangle inequality gives a lower bound for any route. If geometry is absent or unusable, no positive-distance edge exists, or a zero-cost movement forces alpha to zero, A* uses `h = 0`. That remains exact but gives no distance guidance.

This is a mathematical argument for the finite graph model, with ordinary floating-point limits. It is not a continuous-space or physical-optimality claim. Coordinates must use a consistent frame. Very large accumulated costs and application-specific numerical limits still need agreement.

The wrapper checks the route endpoints, verifies every edge through the accepted adjacency, recomputes the cost using `fsum`, and compares it with the search total. `cost_tolerance` is used as both absolute and relative audit tolerance. It does not change feasibility or add a planning penalty.

Search counters report generated/reached nodes, popped entries and unique explored nodes. A popped goal counts as an expansion. Tied minimum-cost routes need not have identical node sequences; minimum cost and valid directed edges are what matter.

## Updates and stale results

Input graphs are copied. Later caller edits cannot silently change the accepted snapshot. `replan` accepts a complete replacement, then searches from the supplied current start to the requested goal. It does not rebuild a map or infer changed edges.

Every replacement increments an internal snapshot counter, even if the external revision label is reused. `is_current(result)` checks both identities. Old successes and old failures become stale. If a replacement fails validation, the old graph is disabled; planning cannot quietly continue on it.

Updates are synchronous. Revision labels are not sorted, so out-of-order delivery is not detected. Cancellation, concurrency, controller acknowledgements and stopping an already executing route are not implemented. The caller must coordinate execution when an update arrives. Result freshness checking applies within the planner instance that issued the result; it is not a cross-process authentication mechanism.

## Current test setup and results

The tests use synthetic explicit graphs. They are not map-layout tests, simulation or robot trials. Costs are test values, not energy or seconds. No collision geometry is inferred from the positions.

The main graph has five nodes: 0 and 1 are WALK; 2 and 3 are FLY; 4 is WALK. WALK edges 0→1 and 1→4 each cost 3. The alternative edges 0→2, 2→3 and 3→4 each cost 1. Thus the complete WALK route costs 6 and the route using FLY costs 3. Reverse edges are absent unless a test adds them.

**Current active-suite result: 40 passed, zero failures or skips.** The supplied-graph demonstration also passed. A distribution built from the same source contains exactly four package modules; its extracted package ran the example successfully. The project environment lacked the setuptools build backend, so packaging was checked with the bundled build runtime; this does not claim a fresh dependency installation in the project environment. Earlier recorded outputs are under `evidence/global_planner/scope_cleanup/`; they predate the current optional-metadata tests. The earlier input-wrapper checkpoint was 15 passes. Its old whole-workspace run was 347 passes and 61 failures; see [previous full output](evidence/global_planner/graph_input_scope/full.txt). The smaller active suite is a changed scope, not a repair of those 61 failures.

| Test case | Expected check |
|---|---|
| Whole multimodal route | WALK → FLY → WALK chooses 0→2→3→4 at supplied cost 3 |
| Different supplied costs | Selects the cheaper weighted route; geometry is not treated as route cost |
| Directed behavior / disconnected pair | Reverse edges are not invented; unreachable pairs return no path |
| Same start and goal | Single-node route at cost 0 |
| Equal-cost alternatives and cycles | Returns a valid route at the independent minimum cost |
| Dense and large sparse graphs | Both methods match repeated-relaxation reference costs |
| Graph without geometry or mode metadata | Dijkstra and A* return a valid route; A* uses zero heuristic |
| Cost increase and edge removal | Replans using the updated snapshot; returns no path when blocked |
| New cheaper route, changed start and changed goal | Replanning follows the new supplied costs and request endpoints |
| Snapshot replacement and caller mutation | Copies input; old result is stale even if revision text is reused |
| Invalid revision or malformed graph | Rejects input and disables the previous snapshot |
| Invalid costs, modes, endpoints and explicit invalid edge | Rejects input rather than returning a successful route |
| Missing start/goal and lazy builder | Raises an input error; does not attach endpoints or trigger generation |
| Zero-cost movement | A* still matches the independent minimum |
| All pairs in a multimodal fixture and seeded directed graphs | Both algorithms agree with the independent repeated-relaxation reference |
| Large finite coordinates and non-finite values | Avoids distance overflow; rejects non-finite supplied values |

The independent reference repeatedly relaxes all edges up to node-count minus one times. It does not call Dijkstra or A*. The seeded tests and compact dense/sparse cases are synthetic contract checks, not physical scenarios. They are useful correctness evidence, not proof for every possible input.

`search_ms` includes only the search call. Snapshot validation/copying, heuristic-scale preparation, final route audit, imports, input loading, output formatting and I/O are excluded. Pytest duration includes test overhead and is not route latency. No new time/memory benchmark or speedup claim is made here. The run used native Windows 11 and Python 3.12.14. The saved verification record contains the OS-reported processor identifier; no CPU model name, RAM size or hardware performance claim is inferred.

## Development and decisions, newest first

### 1. Separate search from graph generation

The confirmed handoff is a complete graph with costs. Earlier work also built graphs from test maps, which duplicated the other task. The current wrapper now imports only graph data types and search. This scope correction is why the map adapters, geometry validators, WALK/FLY builders, transition generation and their tests were removed from active source. It is not a claim that every removed algorithm failed.

The old workspace's 61 failing checks were in graph generation/integration and the old benchmark memory helper. Their output remains intact. They have not become passes. Source and old reports were preserved in a verified recovery archive before removal, so they can be handed to the upstream owner. They are not dependencies of this package.

The unused optimized A*, weighted-search option, D* Lite implementation, old comparison runners, ROS packaging and duplicated reports were also removed. The active API keeps two exact searches. The low-level A* default is now zero guidance unless a valid scale is supplied; the wrapper derives the scale from the graph. The distance calculation uses `math.dist` to avoid unnecessary overflow from squaring large finite coordinates.

### 2. Reconstructed global-search comparison

An earlier comparison used Dijkstra, exact A*, an optimized A* variant and D* Lite on 11 reconstructed graph labels. All 44 recorded outcomes matched that run's expected results, and successful costs agreed. However, M2, M3, M5, M6, M7, M8, M10 and M11 did not reproduce the earlier graph/route evidence exactly. Matching scenario names was insufficient. These results cannot establish a fair cross-campaign winner.

The recorded static medians were 0.080 ms for Dijkstra, 0.110 ms for A*, 0.129 ms for optimized A*, and 1.125 ms for D* Lite. Each case/method had one observation. These tiny single-run differences are not a stable selection basis. Windows memory was process peak working set, including imports/loading/search, not live planner-only allocation. Graph preparation and audits were outside static search timing.

[Raw comparison CSV](evidence/global_planner/history/reconstructed_search.csv) and [recorded output](evidence/global_planner/history/reconstructed_search.txt) retain the actual observations. No exact reconstruction or new timing campaign was performed during cleanup. D* Lite and optimized A* were not retained because neither is needed by the current contract and the evidence does not justify their maintenance cost; they are not declared universally worse.

### 3. Earlier joint-search comparison

The earlier 12-graph comparison recorded 144 serial runs: 36 per method, including three repeats per graph. Dijkstra and A* returned the same costs. Weighted A* produced maximum observed cost increases of 5.574% at weight 1.5 and 19.148% at weight 2. It was not retained for a minimum-cost interface.

| Search | Correct outcomes | Successful routes | Search median / P95 |
|---|---:|---:|---:|
| Dijkstra | 36/36 | 30/36 | 1.620 / 12.495 ms |
| A* | 36/36 | 30/36 | 1.544 / 12.480 ms |
| Weighted A*, 1.5 | 36/36 | 30/36 | 0.538 / 27.768 ms |
| Weighted A*, 2 | 36/36 | 30/36 | 0.527 / 7.272 ms |

Correct outcomes include expected no-path cases. The configurations included transition choices, transition penalties, a required-flight bridge, one-way transitions, WALK-only routes, scale, clutter and disconnection. This was not an assumption of an uncluttered operating environment.

Graph construction had a historical median of 363.9 ms and range 165.6–1819.9 ms. It is upstream work under the present boundary. Search timing excluded route validation, startup, I/O and a separate allocation-measurement replay. RSS was worker lifetime high-water memory including imports/loading. These historical measurements do not establish current Windows or final-system budgets. [Raw rows](evidence/global_planner/history/joint_search_raw.jsonl), [manifest](evidence/global_planner/history/joint_search_manifest.json) and [audit](evidence/global_planner/history/joint_search_audit.json) are retained.

### 4. Earlier representation work

Before the boundary was agreed, roadmap, lattice and Hybrid variants were investigated to create WALK connections; FLY and transition graphs were also built. Connectivity misses, heading/turning limits and sampling-budget failures belonged to those graph builders. Changing Dijkstra to A* cannot recover an edge absent from the supplied graph. That is why the representation work is no longer part of this report's active algorithm selection or tests.

Those failures remain in the recovery evidence. They are not rewritten as global-search failures, and removing their code does not prove the supplier's future graph is complete. If a supplied graph misses a physical route, the upstream owner must inspect its nodes/connections; if the search misses an existing graph route, it is our defect.

## Assumptions and remaining work

| Current assumption or limit | Why it is used | What must be agreed or tested |
|---|---|---|
| Complete finite directed graph | Confirmed team handoff | Real message schema, IDs and action references |
| Nonnegative scalar costs | Exact-search requirement | Cost units, ranges and consistent units across modes |
| Supplier has checked motion feasibility | Global planner has no map/robot geometry | Upstream validation evidence and integration tests |
| One immutable snapshot per synchronous search | Simple consistent route calculation | Update ordering, concurrency, cancellation and execution handshake |
| Endpoints already attached | No generation in the search task | Who publishes current start/goal IDs and when |
| No parallel edges or self-edges | Current Graph format | Check whether action choices need separate edge IDs |
| Positions share a frame | Required for useful A* distance guidance | Actual frame, units and coordinate limits |
| Test graphs stand in for producer output | Producer integration is not supplied | Real small, clutter-derived, large and disconnected graph fixtures |
| No latency or memory target | No agreed operational budget | End-to-end copy/search/audit/update timing and peak/live memory budgets |

The remaining sequence is to define the producer/controller message schemas, add the message-to-`Graph` conversion, establish start/goal node-ID attachment, and verify route output and graph updates against real supplied graph fixtures. Then measure end-to-end timing and memory against agreed budgets. Any simulation or robot execution work belongs with the execution owner; this planner's role there is to supply and freshness-check a route, not to control motion. The conversion must consume supplied edges and costs, not regenerate them.

Selection follows correctness, represented-route success, minimum cost, then time and memory. No weighted score or isolated microsecond difference decides the final method. If both methods return the same minimum cost, representative end-to-end measurements decide whether A* guidance is worthwhile.

Software acceptance requires valid and invalid input tests, exact cost agreement on representative supplied graphs, correct mode/edge order, no stale results after updates, agreed output payloads, and agreed performance limits. Passing this suite alone does not finish autonomous navigation. Collision validity, local control, dynamic obstacles and real motion still need their owners' integration evidence.

If ROS is used later, it should wrap this planner with graph/request input and route output messages. The Python modules do not each need a ROS node. ROS integration, simulation and robot execution are not claimed by this prototype.
