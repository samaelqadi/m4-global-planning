# Validation

- One full package regression run: 167 passed, 1 existing copyright skip, 1 formatting-only flake8 failure. The failure was corrected; no functional regression failed.
- Final package build: `colcon build --packages-select m4_global_planner` succeeded.
- Final focused package run: `colcon test --packages-select m4_global_planner --event-handlers console_direct+ --pytest-args -q -p no:cacheprovider -k 'flake8 or pep257 or search_comparison'` returned 14 passed, 155 deselected. `colcon test-result` reported zero errors/failures.
- All 156 prior passing checks are preserved; 12 new checks cover matched searches, independent reference costs, graph immutability, heuristic lower bounds, measurable weighted gaps, invalid settings, no-path/trivial requests, snapshot identity and memory replay.
- The 144 serial timed benchmark runs and identical memory replays passed validation. They ran after regression tests completed and before final focused checks, with no concurrent tests.
- Artifact audit verified three repeats per method/configuration, 12 immutable graph identities, snapshot/source hashes, matched endpoint/cost references and all 269570 directed-edge heuristic checks. See `audit.json`.
- No physical execution, continuous-space optimality, theoretical weighted cost bound or latency guarantee is established.
