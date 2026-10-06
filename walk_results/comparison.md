# WALK comparison

Fresh-process runs; all returned successful routes were revalidated. Times include construction, planning and route audit.

| Approach | Correct outcomes | Valid routes / expected routes | Mean route cost¹ | Median / P95 total ms² | Median peak / growth MiB³ | Median nodes / edges⁴ |
|---|---:|---:|---:|---:|---:|---:|
| lattice | 57/60 | 36/39 | 23.829 | 178.88 / 4934.10 | 23.54 / 0.75 | 602 / 5848 |
| grid | 57/60 | 36/39 | 23.829 | 115.26 / 8051.66 | 22.84 / 0.00 | 88 / 593 |
| hybrid | 60/60 | 39/39 | 22.310 | 8.84 / 4502.79 | 22.82 / 0.00 | 99 / 258 |
| roadmap | 54/60 | 33/39 | 68.413 | 110.89 / 4999.82 | 22.86 / 0.00 | 162 / 1243 |

¹ Success-only pooled costs have different case mixes if an approach fails. Use raw per-case costs for comparisons.
² Nontrivial expected-success cases, including failed attempts; failures/timeouts are separately recorded.
³ Each run uses a fresh Linux worker. Peak RSS includes imports; growth is the increase over the pre-plan high-water mark, not live allocation size.
⁴ Lattice: explicit pose graph; grid/roadmap: spatial graph with heading retained during execution/search; Hybrid: discovered bins and yielded motion candidates. These counts are not equivalent measures.

**Decision:** prefer the implemented stop-and-pivot Hybrid A* variant for this tested flat-ground WALK model: 60/60 expected outcomes, all 39 feasible requests solved, and the lowest pooled median total time. This is a WALK benchmark selection, not a complete M4 representation or real-robot endorsement. Its binned dominance can lose alternatives; no general completeness or optimality claim is made.

The lattice and grid miss W07 at 0.8 m because its usable passage has no sampled row; both pass at 0.2/0.4 m. The roadmap misses W03 with seed 47, G04 with all three seeds, and G05 with seeds 11/29 at its 160-sample budget. These are missed valid routes, not proof of physical disconnection. All approaches correctly reject the tested blocked routes and invalid endpoints.

Tradeoffs: G04 median total time is 0.767 s Hybrid, 3.455 s grid, 4.934 s lattice; maximum worker RSS is 29.64, 45.52, 86.62 MiB respectively. In G05 the lattice is faster (1.302 s versus 4.503 s Hybrid and 8.052 s grid); its route cost is 66.950 versus Hybrid's 67.177. In G03 the lattice/grid detour costs 40.831, Hybrid 40.710, roadmap 48.000. These per-case comparisons avoid the table's unequal success mixes. Three repetitions are a small timing sample.

Shared assumptions: flat ground, current footprints, finite-length straight forward/reverse moves, stopped pivots, conservative continuous pivot cylinder, configured permissions and per-edge penalties. Benchmark terrain costs are opt-in additive centerline densities per meter, integrated over covered fractions; overlapping regions add and pivot terrain distance is zero. Legacy unsupported terrain behavior and the 55-check baseline are preserved. Exact endpoints are attached through shared validated connectors without snapping. G04/G05/W08 requests omitted by the original scenarios are explicitly supplied in YAML.

The 2D grid builds a spatial topology and retains heading while choosing/executing validated motion macros; it never accepts unchecked sideways motion or heading-free postprocessing. The roadmap uses continuous seeded ground samples, bounded spatial neighbors, and a configurable 20% request-line sampling attempt budget with adjacent validated line links, added after uniform sampling missed W04. Hybrid retains continuous x/y representatives, discretizes dominance keys, and generates primitives during heuristic search; it has no car-steering arcs. Explicit representations use the same queue policy, with heading execution lifted lazily for grid/roadmap. Thus measured time compares complete planners and construction, not isolated search algorithms.

Remaining unsupported cases: non-flat body sweeps, calibrated continuous turning, unknown-space policy, map updates/replanning, FLY/transitions and real-robot feasibility. Node-only N01–N06 retain their baseline tests rather than synthetic timing requests. The conservative pivot approximation can reject physically feasible partial rotations.

Artifacts: `raw.jsonl` records every final route, status, seed, cost, construction/planning/audit/total time, per-worker memory and graph/work counts. `manifest.json` records configuration, environment, source hashes, assumptions and verification; `scenarios.yaml` snapshots inputs; `focused_checks.txt` records 66 passing checks. `pilot/` preserves 240 exploratory runs, including rejected floating-heading endpoints; these are excluded from the final table. The exact-heading reconstruction fix retains strict route auditing.

Reproduce from the workspace with `PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=src/m4_global_planner python3 -m m4_global_planner.walk_benchmark --output walk_results_new`. Use `--scenarios walk_results/scenarios.yaml` for the saved inputs and `--case INDEX` for a single case. Finished runs are checkpointed; changing source/config requires a new output directory. No FLY work or separate search comparison was performed.
