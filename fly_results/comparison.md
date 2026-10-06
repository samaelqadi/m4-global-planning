# FLY geometric comparison

All successful routes revalidated; shared snapshots, swept box checks, local union-kNN, distance costs and A*. Three fresh-process runs per case/tier; roadmap seeds 11/29/47.

| Candidate / tier | Correct outcomes | Valid routes / expected | Mean cost¹ | Median build / plan / total ms² | Peak / growth MiB³ | Nodes / directed edges |
|---|---:|---:|---:|---:|---:|---:|
| adaptive_lattice / coarse | 30/30 | 27/27 | 12.968 | 74.6 / 0.4 / 75.6 | 23.0 / 0.4 | 277 / 5288 |
| adaptive_lattice / fine | 30/30 | 27/27 | 12.851 | 488.5 / 1.7 / 489.5 | 25.9 / 3.2 | 1142 / 18554 |
| sparse_roadmap / coarse | 25/30 | 22/27 | 8.114 | 76.5 / 0.3 / 77.0 | 23.0 / 0.3 | 262 / 4906 |
| sparse_roadmap / fine | 30/30 | 27/27 | 12.022 | 400.8 / 1.3 / 402.2 | 26.1 / 3.4 | 1030 / 19031 |

¹ Pooled success-only means; use matched per-case raw costs when success sets differ.
² Includes the identical saved preparation time for each case plus candidate construction/search/audit; excludes Python startup, YAML/ snapshot I/O. No-path attempts included. Raw rows separate each phase.
³ Median per-run Linux process high-water RSS, including imports and shared preparation peak; growth is worker increase over pre-build high-water RSS. Neither is live allocation size.

Scope: F01–F06 plus explicit FLY-only G01/G02/G04/G05 requests. F06 includes a high waypoint. G03 WALK costs and G06 unknown occupancy unsupported. Ground dock anchors are geometric; no executable takeoff/landing or multimodal claims.

Decision: coarse adaptive lattice is the provisional deterministic geometric reference; fine roadmap is the alternative for lower route cost and dense-clutter latency. Both achieved 30/30 correct outcomes. Coarse roadmap is insufficient (five coverage failures). No universal or real-robot winner.

Limits: three seeds, finite nonnested local graphs, synthetic FLY-only requests, geometric ground docks, uncalibrated box/margins; CPU not isolated and late regressions overlapped some runs. Shared preparation was measured once and charged equally. See the existing workspace report for scope and pending requirements.
