# Property-Based Tests for libsurvive

## Approach

We use property-based testing to verify mathematical invariants in libsurvive's core math and reprojection code. Each test generates thousands of random inputs and checks that a property (invariant) holds for all of them. This catches edge cases that hand-written examples miss.

Tests use the existing test infrastructure (`test_case.h`, `REGISTER_LINKTIME`). No external dependencies — just C stdlib and libsurvive's own `linmath_normrand` for random generation. Each test prints its seed on failure for reproducibility.

## Implemented Tests

### 1. Quaternion / Pose Roundtrips (`quat_props.c`)

16 properties, 10,000 random trials each (160,000 total).

| Test | Property |
|---|---|
| `NormalizeProducesUnit` | `\|normalize(q)\| = 1.0` for any non-zero input |
| `ConjugateInvolution` | `conj(conj(q)) == q` |
| `MultiplyByInverseIsIdentity` | `q * q^-1 = [1,0,0,0]` |
| `RotateUnrotateIdentity` | `R(q^-1, R(q, v)) = v` |
| `RotationPreservesMagnitude` | `\|R(q, v)\| = \|v\|` |
| `PoseInvertRoundtrip` | `Apply(Invert(p), Apply(p, pt)) = pt` |
| `PoseComposeInverseIsIdentity` | `Compose(p, Invert(p)) = identity` |
| `MatrixRoundtrip` | `quatfrommatrix33(quattomatrix33(q)) = ±q` |
| `AxisAngleRoundtrip` | axis-angle -> quat -> axis-angle recovers original |
| `MultiplicationAssociative` | `(a*b)*c = a*(b*c)` |
| `SlerpBoundaries` | `slerp(a, b, 0) = a` and `slerp(a, b, 1) = b` |
| `SlerpProducesUnit` | `\|slerp(a, b, t)\| = 1.0` for any t in [0,1] |
| `QuatDistSelfIsZero` | `quatdist(q, q) = 0` for any unit quaternion |
| `QuatDistSymmetric` | `quatdist(q1, q2) = quatdist(q2, q1)` |
| `QuatDistNonNegativeAndBounded` | Result always in `[0, π]` |
| `QuatDistKnownAngle` | `quatdist(identity, q_θ) ≈ θ` for quaternion encoding known rotation θ |

2 bugs found and fixed — see Trophy Case.

### 2. Lighthouse Reprojection (`reproject_props.c`)

13 properties, 5,000 random trials each (65,000 total).

| Test | Property |
|---|---|
| `ZeroCalGeometry_Gen1` | Zero calibration -> angles = pure atan2 |
| `OnAxisZeroAngles_Gen1` | Point on boresight (x=0, y=0) -> angles = 0 |
| `XSymmetry_Gen1` | Negating x negates x-angle, preserves y-angle |
| `FullPipelineIdentity_Gen1` | `reproject_full` with identity poses = `reproject_xy` |
| `Continuity_Gen1` | Small perturbation -> small angle change |
| `OnAxisZeroAngles_Gen2` | Gen2 boresight -> angles = 0 |
| `FullPipelineIdentity_Gen2` | Gen2 full pipeline identity consistency |
| `Continuity_Gen2` | Gen2 continuity under perturbation |
| `NumericJacobian_Gen1` | Central-difference Jacobian predicts angle changes |
| `NumericJacobian_Gen2` | Same for Gen2 |
| `OutputFinite_Gen1` | No NaN/Inf for valid inputs with random calibration |
| `OutputFinite_Gen2` | No NaN/Inf for valid inputs with random calibration |
| `PoseTransformConsistency_Gen1` | Manual transform + `reproject_xy` = `reproject_full` |

No bugs found in the reprojection code.

### 3. Kabsch Point-Set Registration (`kabsch_props.c`)

5 properties, 5,000 random trials each (25,000 total).

Kabsch computes the optimal rigid transform aligning two point clouds. Used in calibration and multi-lighthouse alignment.

| Test | Property |
|---|---|
| `IdentityRecovery` | `Kabsch(pts, pts)` -> identity pose |
| `KnownTransformRecovery` | Apply random pose to pts -> Kabsch recovers the pose (RMS ≈ 0) |
| `ResidualMinimality` | After alignment of noisy points, RMS residual is small |
| `CenteredConsistency` | `KabschCentered` with pre-centered points -> same rotation as full `Kabsch` |
| `ScaleRecovery` | `KabschScaled` with uniformly scaled points -> finite positive scale factor |

No bugs found in the Kabsch code.

### 4. Kalman Predict State-Transition (`kalman_props.c`)

10 properties, 20–10,000 random trials each (~74,000,000 total steps).

Tests the generated `SurviveKalmanModelPredict` function which propagates the Kalman state forward in time. Tested without hardware or `SurviveContext` dependencies. The last 3 tests simulate long-duration tracking sessions (50K–200K predict steps per trial) to catch numerical drift that could cause pose dropouts.

| Test | Property |
|---|---|
| `ZeroVelocityPreservesPose` | Predict with zero velocity/acceleration preserves pose |
| `QuaternionStaysNormalized` | After predict, `\|state.Pose.Rot\| ≈ 1.0` |
| `ZeroDtIsIdentity` | Zero time step -> state unchanged |
| `LinearPositionComposition` | `predict(2*dt) ≈ predict(dt)` applied twice (zero angular velocity) |
| `VelocityIntegratesAcceleration` | `v_out = v_in + a * dt` |
| `AngularVelocityPreserved` | Angular velocity is constant across predict |
| `AccelerationPreserved` | Acceleration is constant across predict |
| `RepeatedPredictQuatStaysNormalized` | `\|q\| ≈ 1.0` after 200K predict steps with periodic renormalization (20 trials) |
| `ZeroVelocityLongDurationStable` | Stationary tracker position drift < 1e-3 after 100K steps (100 trials) |
| `ConstantVelocityAccumulation` | `p = p0 + v*T` within 0.1% after 50K steps (100 trials) |

No bugs found in the Kalman predict code. The predict math is numerically stable over long sessions — the 24-minute pose dropout is caused by the reporting gate cascade (light residual growth → `check_valid` failure → observation count reset), not by predict-step drift.

### 5. Numeric Robustness / NaN Propagation (`numeric_props.c`)

10 properties, 1,000–10,000 random trials each.

Fuzz math functions with adversarial and degenerate inputs to verify no crashes or silent NaN/Inf propagation.

| Test | Property |
|---|---|
| `QuatNormalizeZero` | `quatnormalize(zero)` doesn't crash; result is NaN or unit quaternion — not finite non-unit |
| `RotateWithNonUnitQuat` | `quatrotatevector` with non-unit q produces finite output |
| `ReprojectGen1Finite` | Gen1 reprojection with random calibration produces finite angles |
| `ReprojectGen2Finite` | Gen2 reprojection with random calibration produces finite angles |
| `Normalize3dZero` | `normalize3d(zero)` doesn't crash; result is NaN or unit vector — not finite non-unit |
| `IdentityPosePreservesPoint` | Identity pose applied to any point returns the same point |
| `ReprojectExtremeDistance` | Points at extreme distance (100–10,000m) produce finite, near-zero angles |
| `KabschCollinear` | Kabsch with collinear points doesn't crash, produces finite translation (rotation underdetermined) |
| `KabschCoplanar` | Kabsch with coplanar points doesn't crash, produces finite pose (rotation underdetermined around normal axis) |
| `LargeQuatNoOverflow` | `quatrotatevector` with very large quaternion components doesn't overflow |

No crashes found. `quatnormalize(zero)` and `normalize3d(zero)` produce NaN (documented, not a crash). Callers that may receive zero-magnitude input must guard against NaN propagation at their own boundary.

### 6. Reprojection Residual / BSVD Pose Solver (`reproject_residual_props.c`)

6 properties, 1,000 random trials each (6,000 total).

Full roundtrip tests: generate random pose at VP-realistic distances (1–4m) → reproject 12 sensor positions (golden-angle spiral on 5cm sphere) to get synthetic lighthouse angles → solve pose with BSVD → compare recovered pose to original. Then corrupt angles to simulate lighthouse reflections off shiny surfaces (truss, LED walls) and verify the solver is robust and residuals identify bad sensors.

| Test | Property |
|---|---|
| `CleanRoundtrip` | Reproject → BSVD solve → recovered pose matches original (< 0.02 error) |
| `ReprojectionResidualSmall` | After clean solve, all per-sensor residuals < 1e-4 rad |
| `SingleOutlierDetectable` | Corrupt 1 sensor 0.05–0.3 rad → its residual > 5x median (>90% detection rate) |
| `SingleOutlierPoseStable` | With 1 corrupted sensor out of 8+, pose error < 0.30 |
| `MultipleOutliersDegradeGracefully` | 2–3 corrupted sensors out of 9+, pose error < 0.60 |
| `ResidualIdentifiesCorruptedSensor` | Max-residual sensor is the corrupted one (>85% identification rate, 0.10 rad min offset) |

No bugs found. BSVD pose solver handles single-sensor corruption gracefully — pose remains stable and residuals reliably identify the corrupted sensor. The detection rates (90% / 85%) are probabilistic, not guaranteed: detection degrades when the corruption offset is small (near the 0.05–0.10 rad minimum tested here) or when the corrupted sensor has low geometric leverage (nearly collinear with the lighthouse axis). In VP environments with reflective surfaces (LED walls, truss), brief low-magnitude reflections may fall below the detection threshold.

### 7. Event Queue Circular Buffer (`event_queue_props.c`)

7 properties, 100–1,000 random trials each.

The Simple API uses a fixed 64-event circular buffer. These tests verify the queue's behavior under normal use, overflow, and edge conditions. Context: our tracker experienced a 24-minute pose dropout — one hypothesis was event queue overflow causing silent event loss.

| Test | Property |
|---|---|
| `InsertPopFIFO` | Insert N, pop N: all values recovered in FIFO order (1,000 trials) |
| `OverflowDropsOldest` | Insert 64+N events without popping: only last 64 recoverable (100 trials) |
| `InterleavedInsertPop` | Random insert/pop interleaving maintains FIFO order (200 trials, 500 ops each) |
| `SizeNeverExceedsMax` | `events_cnt` never exceeds `MAX_EVENT_SIZE` regardless of insertions (100 trials) |
| `EmptyPopReturnsFalse` | Pop on empty queue returns false, including after drain |
| `OverflowDetected` | `queue_insert` returns true when buffer was full |
| `WrapAroundStress` | Fill/drain cycles (1,000 cycles) with random batch sizes — circular indexing stays correct |

No bugs found. The circular buffer logic is correct. Queue overflow is silent by design (oldest events overwritten), which is documented but not a bug.

### 8. Back-Facing Normal Filter Geometry (`normal_filter_props.c`)

3 properties, 10,000 random trials each (30,000 total).

Tests the back-facing normal filter in `SurviveSensorActivations_check_outlier()`. Each test uses a random (non-identity) tracker rotation and calls `compute_facingness()` — the full coordinate transform path (quatrotatevector + ApplyPoseToPoint + normalize + dot3d) is exercised in every trial.

| Test | Property |
|---|---|
| `FacingnessKnownAngle` | `compute_facingness()` returns `cos(θ)` when sensor normal is at known angle θ from lighthouse direction; uses random rotation with normal back-rotated to body frame |
| `DirectlyFacingAlwaysAccepted` | Sensor normal aligned with lighthouse gives facingness ≈ 1.0, passes any threshold ≤ 1 |
| `BackFacingAlwaysRejected` | Sensor normal opposing lighthouse gives facingness ≈ −1.0, fails any threshold ≥ 0 |

No bugs found. The filter geometry is correct: at the default threshold 0.0, sensors within 90° of the lighthouse are accepted and sensors beyond 90° are rejected, which is the physically correct cutoff for a flat sensor surface.

## Summary

| Suite | File | Properties | Trials | Bugs Found |
|---|---|---|---|---|
| Quaternion/Pose | `quat_props.c` | 16 | 160,000 | 2 (matrix layout, quatdist clamp) |
| Reprojection | `reproject_props.c` | 13 | 65,000 | 0 |
| Kabsch | `kabsch_props.c` | 5 | 25,000 | 0 |
| Kalman Predict | `kalman_props.c` | 10 | ~74M steps | 0 |
| Numeric Robustness | `numeric_props.c` | 10 | ~80,000 | 0 |
| Reprojection Residual | `reproject_residual_props.c` | 6 | 6,000 | 0 |
| Event Queue | `event_queue_props.c` | 7 | ~3,500 | 0 |
| Normal Filter | `normal_filter_props.c` | 3 | 30,000 | 0 |
| **Total** | | **70** | | **2** |

## Running the Tests

```bash
# Build
cd build && cmake .. -DCMAKE_BUILD_TYPE=Release -DENABLE_TESTS=ON
cmake --build .

# Run all tests
ctest --output-on-failure

# Run just the property tests
./src/test_cases/test-quat_props
./src/test_cases/test-reproject_props
./src/test_cases/test-kabsch_props
./src/test_cases/test-kalman_props
./src/test_cases/test-numeric_props
./src/test_cases/test-reproject_residual_props
./src/test_cases/test-event_queue_props
./src/test_cases/test-normal_filter_props
```

Tests also run automatically in CI (`ci-property-tests.yml`) on every push and PR. Under ASan/UBSan, the random inputs also catch undefined behavior and memory errors.

## Limitations

These tests cover pure math and in-process logic. They do not substitute for hardware-in-the-loop testing of:

- **Lighthouse sweep synchronization** — timing relationships between sweep pulses depend on actual base station hardware and USB interrupt scheduling.
- **USB enumeration order** — which interface gets opened first (IMU vs. Lightcap) is determined by the kernel USB stack, not by libsurvive logic.
- **Kalman update behavior** — only the `predict` step is tested here; the `update` path requires real sensor observations with realistic noise characteristics.
- **Pose dropout under load** — long-duration stability issues are caused by the interaction of Kalman covariance growth, the reporting gate, and real sensor timing jitter — none of which can be driven by synthetic random inputs alone.

These scenarios require physical trackers, base stations, and logged field data to reproduce and verify.

---

## Trophy Case

Bugs found and fixed by property tests. Each entry records what the test was checking, what it
actually found, and why the bug mattered.

### Bug 1 — `quattomatrix33` / `quatfrommatrix33` row/column-major mismatch

**Found by:** `QuatProps.MatrixRoundtrip` (`quat_props.c`)
**File:** `redist/linmath.c` — `quattomatrix33()`
**Status:** Fixed; merged upstream as https://github.com/collabora/libsurvive/pull/347

**What the test checked:** `quatfrommatrix33(quattomatrix33(q)) ≈ ±q` for 10,000 random unit
quaternions.

**What it found:** The roundtrip produced the conjugate of the input (i.e. the inverse rotation)
instead of the original. The recovered quaternion was consistently `{w, -x, -y, -z}`.

**Root cause:** `quattomatrix33` wrote the matrix in column-major ("OpenGL") layout, but
`quatfrommatrix33` read it in row-major layout. The transposed matrix encodes the inverse of the
intended rotation, so every roundtrip silently returned the wrong result.

**Fix:** Changed `quattomatrix33` to write row-major. One-line reordering of the index arithmetic.

**Why it mattered:** Any code path that converts a quaternion to a 3×3 matrix and then back (e.g.
for interpolation helpers or external integrations) was silently getting the inverse rotation. This
was not caught by existing tests because there were no roundtrip tests for this function pair.

---

### Bug 2 — `quatdist` clamp arguments swapped — always returned 0

**Found by:** `QuatProps.QuatDistKnownAngle` (`quat_props.c`)
**File:** `redist/linmath.c` — `quatdist()`, line 299
**Status:** Fixed; merged upstream as https://github.com/collabora/libsurvive/pull/350

**What the test checked:** For a quaternion encoding a known rotation of θ radians around a random
axis, `quatdist(identity, q_θ) ≈ θ`. Checked for θ in (0, π) across 10,000 random axes.

**What it found:** `quatdist` returned 0.0 for every input, regardless of the actual angular
separation.

**Root cause:** The dot-product clamp before `acos` had its `min` and `max` arguments swapped:

```c
// WRONG — linmath_min(-1, rtn) is always ≤ -1, so linmath_max(1, ...) is always 1.0
// acos(FLT_FABS(1.0)) = 0.0 for every input
rtn = linmath_max(1., linmath_min(-1, rtn));
```

`linmath_min(-1, rtn)` returns a value ≤ −1 for any `rtn`. Then `linmath_max(1, ...)` clamps that
to exactly 1.0. So `acos(|1.0|) = 0` always.

**Fix:**

```c
// CORRECT — clamps dot product to [-1, 1] before acos
rtn = linmath_min(1., linmath_max(-1., rtn));
```

**Why it mattered:** The Kalman tracker's pose angular rate gate (`--kalman-max-pose-angular-rate`)
uses `quatdist` to measure the rotation between consecutive poses. With `quatdist` always returning
0, the gate would have been completely inert in production — no rotation jump, however large, would
ever exceed the threshold. The `QuatDistKnownAngle` test was written specifically to verify the
units and magnitude of the return value, and it caught this immediately.
