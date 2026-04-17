# Arrow: tracking-engine

Computes continuous 6-DOF pose and velocity for tracked objects by fusing lighthouse angle measurements and IMU data through geometric solvers and an Extended Kalman Filter.

## Status

**MAPPED** - 2026-04-13. Full LLD written from brownfield reconnaissance. EARS specs written. Core tracking pipeline is mature and well-tested. Per-LH adaptive R scaling and innovation gate implemented (TE-PROC-038, TE-PROC-039). Key gap: no CI guard on generated math files.

## References

### HLD
- [docs/high-level-design.md](../high-level-design.md) — "Dual-Path Tracking Architecture", "Cross-Cutting Concerns / Symbolic Code Generation"

### LLD
- [docs/llds/tracking-engine.md](../llds/tracking-engine.md)

### EARS
- [docs/specs/tracking-engine-specs.md](../specs/tracking-engine-specs.md) (41 specs)

### Tests
- src/test_cases/barycentric_svd.c
- src/test_cases/kalman.c
- src/test_cases/optimizer.c
- src/test_cases/reproject.c
- src/test_cases/rotate_angvel.c
- src/test_cases/check_generated.c
- src/test_cases/test_replays.c

### Code
- src/poser.c, src/poser_barycentric_svd.c, src/poser_epnp.c
- src/poser_mpfit.c, src/poser_kalman_only.c
- src/poser_general_optimizer.c, src/poser_dummy.c
- src/barycentric_svd/barycentric_svd.c
- src/survive_kalman_tracker.c, src/survive_kalman_lighthouses.c
- src/survive_optimizer.c, src/survive_async_optimizer.c
- src/survive_reproject.c, src/survive_reproject_gen2.c
- src/generated/ (non-lighthouse files)
- libs/cnkalman/, libs/cnmatrix/
- tools/generate_math_functions/

## Architecture

**Purpose:** Turn calibrated angle measurements and IMU readings into smooth, continuous 6-DOF pose estimates using a geometric seed solver, nonlinear refinement, and Extended Kalman filtering.

**Key Components:**
1. `poser_barycentric_svd.c` + `barycentric_svd/` — Fast geometric seed pose via SVD on barycentric control points
2. `poser_mpfit.c` — Levenberg-Marquardt refinement minimizing reprojection residuals
3. `poser_general_optimizer.c` — Meta-wrapper tracking success/failure, enabling warm starts
4. `survive_kalman_tracker.c` — 19D error-state EKF fusing light + IMU; IEKF for light updates
5. `survive_kalman_lighthouses.c` — 7D Kalman filter per lighthouse (stationary prior)
6. `survive_reproject.c / _gen2.c` — Forward model: (pose, sensor, calibration) → angle
7. `libs/cnkalman/` — EKF/IEKF filter primitives
8. `libs/cnmatrix/` — BLAS/LAPACK matrix wrapper
9. `tools/generate_math_functions/` + `src/generated/` — Symbolic codegen pipeline

## EARS Coverage

| Category | Spec IDs | Implemented | Gaps | Deferred |
|----------|----------|-------------|------|----------|
| Poser interface | TE-API-001 to 005 | 5 | 0 | 0 |
| Geometric pose solvers | TE-PROC-010 to 015 | 6 | 0 | 0 |
| MPFIT optimizer | TE-PROC-020 to 025 | 6 | 0 | 0 |
| Kalman object tracker | TE-PROC-030 to 042 | 13 | 0 | 0 |
| Kalman lighthouse tracker | TE-PROC-050 to 051 | 2 | 0 | 0 |
| Reprojection model | TE-DATA-050 to 053 | 4 | 0 | 0 |
| Math infrastructure | TE-DATA-060 to 064 | 4 | 1 | 0 |

**Summary:** 40 of 41 active specs implemented; 1 active gap (CI guard for codegen TE-DATA-064); 0 deferred.

## Key Findings

1. **Per-LH adaptive R and innovation gate reduce multi-lighthouse jitter** — `light_residuals[lh]` EWMA now populated and used to scale observation noise R per lighthouse (TE-PROC-038); pre-update RMS innovation gate skips outlier LH batches (TE-PROC-039). Both land in `src/survive_kalman_tracker.c`. See `docs/reflection-rejection.md` for full context.

2. **Generated math files committed without CI guard** — `src/generated/*.gen.h` can silently diverge from `tools/generate_math_functions/*.py` sources. Any math change requires manual regeneration with no automated verification. This is the highest-risk maintenance issue in the codebase. (TE-DATA-064)

3. **IEKF divergence is silent** — When the IEKF terminates on max iterations rather than convergence, no warning is surfaced. The tracker proceeds with whatever state was reached. (`libs/cnkalman/src/iekf.c`, termination reason enum exists but is not surfaced upward)

4. **100+ config keys with no documented safe zone** — `survive_kalman_tracker.c` exposes an extremely large tuning surface. Many parameter combinations produce unstable behavior with no guidance on valid ranges.

5. **`poser_epnp.c` appears redundant** — Solves the same problem as BaryCentricSVD with similar algorithms. Unclear which is preferred or whether EPnP is maintained.

6. **IMU bias model update cadence undocumented** — The separate IMU bias Kalman state interaction with the main 19D tracker (which updates which, in what order) is not documented.

## Work Required

### Must Fix
1. Add CI/build check to detect divergence between `tools/generate_math_functions/*.py` and committed `src/generated/*.gen.h` (TE-DATA-064)

### Should Fix
2. Surface IEKF divergence (max-iteration termination) as a warning via the `printf` hook
3. Document IMU bias model update cadence and interaction with main Kalman state
4. Evaluate `poser_epnp.c` — document when to use over BaryCentricSVD, or mark deprecated

### Nice to Have
5. Provide a documented "recommended starting parameters" config block for the Kalman tracker for new device types
6. Consider exposing IEKF iteration count as a diagnostic metric
