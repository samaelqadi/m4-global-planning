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

## 2. Plan minimum cost across modes

Build one mode-aware planning graph from validated WALK motion edges, FLY motion edges, and shared transition anchors. Add explicitly configured transition costs (including `transition_extra_cost`) and preserve each mode's movement costs, endpoint mode, and WALK heading. Run one shortest-path query over that combined graph, then revalidate every movement sweep and stationary morph. Test cases where the cheapest route changes with transition cost and where endpoints require WALK, FLY, or a mode change. Until takeoff/landing edges are supported, describe results only as geometric graph paths, not executable multimodal routes.

## 3. Resolve unsupported assumptions

The FLY model is a fixed, axis-aligned bottom-center box with absent yaw, straight translation, flat known floor, and configured geometric margins. Unknown regions and non-flat floors are rejected. Takeoff, landing, attitude changes, dynamics, energy costs, and calibrated body/morphing envelopes are unsupported. Validate these assumptions in simulation and on the robot before operational use; define altitude, clearance, landing-heading, and transition-cost requirements with measured vehicle data.

## Exact next focused task and commands

The first cost-optimality audit is complete within the limits above. The next task is the joint WALK/FLY/transition minimum-cost graph in section 2; do not benchmark or change provisional selections before that integration. The command above reproduces the completed audit regressions. For later isolated-host FLY measurement reproduction, use a new output directory so saved checkpoints remain intact:

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=src/m4_global_planner python3 -m pytest -q -p no:cacheprovider src/m4_global_planner/test/test_straight_walk_edges.py src/m4_global_planner/test/test_walk_motions.py src/m4_global_planner/test/test_walk_comparison.py src/m4_global_planner/test/test_fly_validation.py src/m4_global_planner/test/test_anchors.py src/m4_global_planner/test/test_fly_graph.py
```

For later isolated-host FLY measurement reproduction, use a new output directory so existing checkpoints remain intact:

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=src/m4_global_planner python3 -m m4_global_planner.fly_benchmark --output fly_results_repeat
```
