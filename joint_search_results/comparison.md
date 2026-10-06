# Joint graph search comparison

Serial fresh-process repeats per method; one saved graph per configuration.
All methods use the same configuration mix. Gaps are matched to Dijkstra per graph.

| Method | Valid outcomes | Successful routes | Median cost | Max gap % | Median explored / pops | Search median / P95 ms | Python peak / RSS MiB |
|---|---:|---:|---:|---:|---:|---:|---:|
| dijkstra | 36/36 | 30/36 | 10.515029 | 0.000 | 686 / 686 | 1.620 / 12.495 | 0.166 / 30.5 |
| astar | 36/36 | 30/36 | 10.515029 | 0.000 | 292 / 292 | 1.544 / 12.480 | 0.181 / 30.4 |
| wA* 1.5 | 36/36 | 30/36 | 10.515029 | 5.574 | 46 / 49 | 0.538 / 27.768 | 0.088 / 30.4 |
| wA* 2 | 36/36 | 30/36 | 10.515029 | 19.148 | 36 / 36 | 0.527 / 7.272 | 0.084 / 30.5 |

12 configurations; graph construction median 363.9 ms, range 165.6–1819.9 ms.
Configurations: T01, T02, T03, T04_low_penalty, T04_high_penalty, T05, T05_roadmap, T05_one_way, T06, G04_scale, G05_clutter, G01_disconnected.

Cost median pools successful requests with the same configuration mix;
Raw rows provide per-graph costs and paired timing/expansion comparisons.
Build time excludes reference search, audits, snapshot I/O and worker startup.
Search statistics include no-path attempts and exclude validation, I/O and replay.
Explored counts unique popped states; pops include re-expansions. Allocation peak
is a separate identical tracemalloc replay. RSS is the timed-worker high-water
through search, including imports and loading; its baseline/growth is in raw rows.
No-path costs/gaps are undefined. All returned paths and totals are validated.
A* lower bound and consistency are checked over every edge. Weighted gaps are
measurements only; no theoretical weighted bound is claimed.
