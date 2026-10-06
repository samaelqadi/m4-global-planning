# Focused height-grid query diagnostic

Local warm-query wall times: seven samples per query, median reported. Construction is excluded.
Sparse occupied cells, known flat ground, strict reject_map policy, same script and grid sizes.
No concurrent tests during the recorded measurements; other host load was uncontrolled.
Tiny cached times include timer/interpreter overhead. These are not planner latency guarantees.

| Grid | Query | Before median ms | After median ms |
|---|---|---:|---:|
| 50×50 | known_check | 1.202284 | 0.000236 |
| 50×50 | point_intervals | 5.506370 | 0.001125 |
| 50×50 | walk_state | 8.155419 | 0.019728 |
| 50×50 | fly_edge | 16.867404 | 0.044909 |
| 200×200 | known_check | 43.954337 | 0.000166 |
| 200×200 | point_intervals | 69.819112 | 0.000576 |
| 200×200 | walk_state | 83.278082 | 0.012828 |
| 200×200 | fly_edge | 172.798849 | 0.028660 |

Raw samples: before.json and after.json. Reproduction:

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=src/m4_global_planner python3 adapter_query_results/measure_queries.py /tmp/adapter_queries_current.json
```

The saved before samples precede caching/local queries; the script now measures current code.
Construction/cache memory and worst-case long sweeps were not measured. Sparse bins avoid
empty-cell index entries; broad FLY lattice construction still accesses full cached geometry.
