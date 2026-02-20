# Property-Based Tests for libsurvive

## Approach

We use property-based testing to verify mathematical invariants in libsurvive's core math and reprojection code. Each test generates thousands of random inputs and checks that a property (invariant) holds for all of them. This catches edge cases that hand-written examples miss.

Tests use the existing test infrastructure (`test_case.h`, `REGISTER_LINKTIME`). No external dependencies — just C stdlib and libsurvive's own `linmath_normrand` for random generation. Each test prints its seed on failure for reproducibility.

## Implemented Tests

### 1. Quaternion / Pose Roundtrips (`quat_props.c`)

12 properties, 10,000 random trials each (120,000 total).

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

#### Bug Found and Fixed

**`quattomatrix33` / `quatfrommatrix33` matrix layout mismatch (fixed).** `quattomatrix33` previously output column-major ("opengl major") but `quatfrommatrix33` read row-major, causing the roundtrip to produce the conjugate (inverse rotation). Fixed by changing `quattomatrix33` to output row-major. The `MatrixRoundtrip` test now verifies the proper roundtrip `recovered = ±q`.

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
| `QuatNormalizeZero` | `quatnormalize(zero)` doesn't crash (documents NaN behavior) |
| `RotateWithNonUnitQuat` | `quatrotatevector` with non-unit q produces finite output |
| `ReprojectGen1Finite` | Gen1 reprojection with random calibration produces finite angles |
| `ReprojectGen2Finite` | Gen2 reprojection with random calibration produces finite angles |
| `Normalize3dZero` | `normalize3d(zero)` doesn't crash |
| `IdentityPosePreservesPoint` | Identity pose applied to any point returns the same point |
| `ReprojectExtremeDistance` | Points at extreme distance (100–10,000m) produce finite, near-zero angles |
| `KabschCollinear` | Kabsch with collinear points doesn't crash, produces finite translation |
| `KabschCoplanar` | Kabsch with coplanar points doesn't crash, produces finite pose |
| `LargeQuatNoOverflow` | `quatrotatevector` with very large quaternion components doesn't overflow |

No crashes found. `quatnormalize(zero)` and `normalize3d(zero)` produce NaN (documented, not a crash).

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

No bugs found. BSVD pose solver handles single-sensor corruption gracefully — pose remains stable and residuals reliably identify the corrupted sensor.

### 7. Event Queue Circular Buffer (`event_queue_props.c`)

7 properties, 100–1,000 random trials each.

The Simple API uses a fixed 64-event circular buffer. These tests verify the queue's behavior under normal use, overflow, and edge conditions. Context: the stagehand agent experienced a 24-minute pose dropout — one hypothesis was event queue overflow causing silent event loss.

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

## Summary

| Suite | File | Properties | Trials | Bugs Found |
|---|---|---|---|---|
| Quaternion/Pose | `quat_props.c` | 12 | 120,000 | 1 (matrix layout mismatch, fixed) |
| Reprojection | `reproject_props.c` | 13 | 65,000 | 0 |
| Kabsch | `kabsch_props.c` | 5 | 25,000 | 0 |
| Kalman Predict | `kalman_props.c` | 10 | ~74M steps | 0 |
| Numeric Robustness | `numeric_props.c` | 10 | ~80,000 | 0 |
| Reprojection Residual | `reproject_residual_props.c` | 6 | 6,000 | 0 |
| Event Queue | `event_queue_props.c` | 7 | ~3,500 | 0 |
| **Total** | | **63** | | **1** |

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
```

Tests also run automatically in CI (`ci-property-tests.yml`) on every push and PR. Under ASan/UBSan, the random inputs also catch undefined behavior and memory errors.
