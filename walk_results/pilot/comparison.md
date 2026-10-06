# WALK comparison

Fresh-process runs; all returned successful routes were revalidated. Times include construction, planning and route audit.

| Approach | Correct outcomes | Valid routes / expected routes | Mean route cost¹ | Median / P95 total ms² | Median peak / growth MiB³ | Median nodes / edges⁴ |
|---|---:|---:|---:|---:|---:|---:|
| lattice | 57/60 | 36/39 | 23.829 | 216.12 / 4679.70 | 23.75 / 0.66 | 602 / 5848 |
| grid | 57/60 | 36/39 | 23.829 | 116.47 / 7195.32 | 23.09 / 0.00 | 88 / 593 |
| hybrid | 57/60 | 36/39 | 21.499 | 9.66 / 4469.83 | 23.09 / 0.00 | 99 / 258 |
| roadmap | 54/60 | 33/39 | 68.413 | 140.18 / 4757.49 | 23.09 / 0.00 | 162 / 1243 |

¹ Success-only pooled costs have different case mixes if an approach fails. Use raw per-case costs for comparisons.
² Nontrivial expected-success cases, including failed attempts; failures/timeouts are separately recorded.
³ Each run uses a fresh Linux worker. Peak RSS includes imports; growth is the increase over the pre-plan high-water mark, not live allocation size.
⁴ Lattice: explicit pose graph; grid/roadmap: spatial graph with heading retained during execution/search; Hybrid: discovered bins and yielded motion candidates. These counts are not equivalent measures.
