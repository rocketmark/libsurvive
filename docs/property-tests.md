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
| `MatrixRoundtrip_KNOWN_BUG` | Documents the col/row-major mismatch (see below) |
| `AxisAngleRoundtrip` | axis-angle -> quat -> axis-angle recovers original |
| `MultiplicationAssociative` | `(a*b)*c = a*(b*c)` |
| `SlerpBoundaries` | `slerp(a, b, 0) = a` and `slerp(a, b, 1) = b` |
| `SlerpProducesUnit` | `\|slerp(a, b, t)\| = 1.0` for any t in [0,1] |

#### Bug Found

**`quattomatrix33` / `quatfrommatrix33` matrix layout mismatch.** `quattomatrix33` outputs column-major (comment says "opengl major") but `quatfrommatrix33` reads row-major. The roundtrip produces the conjugate (inverse rotation) instead of the original quaternion. `quatfromcnMatrix` does not have this issue because it uses `cnMatrixGet(row, col)` which abstracts away layout. The `MatrixRoundtrip_KNOWN_BUG` test documents this by asserting `recovered = conj(q)`, and will fail if the bug is fixed (signaling the test should be updated to a proper roundtrip).

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

7 properties, 10,000 random trials each (70,000 total).

Tests the generated `SurviveKalmanModelPredict` function which propagates the Kalman state forward in time. Tested without hardware or `SurviveContext` dependencies.

| Test | Property |
|---|---|
| `ZeroVelocityPreservesPose` | Predict with zero velocity/acceleration preserves pose |
| `QuaternionStaysNormalized` | After predict, `\|state.Pose.Rot\| ≈ 1.0` |
| `ZeroDtIsIdentity` | Zero time step -> state unchanged |
| `LinearPositionComposition` | `predict(2*dt) ≈ predict(dt)` applied twice (zero angular velocity) |
| `VelocityIntegratesAcceleration` | `v_out = v_in + a * dt` |
| `AngularVelocityPreserved` | Angular velocity is constant across predict |
| `AccelerationPreserved` | Acceleration is constant across predict |

No bugs found in the Kalman predict code.

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

## Summary

| Suite | File | Properties | Trials | Bugs Found |
|---|---|---|---|---|
| Quaternion/Pose | `quat_props.c` | 12 | 120,000 | 1 (matrix layout mismatch) |
| Reprojection | `reproject_props.c` | 13 | 65,000 | 0 |
| Kabsch | `kabsch_props.c` | 5 | 25,000 | 0 |
| Kalman Predict | `kalman_props.c` | 7 | 70,000 | 0 |
| Numeric Robustness | `numeric_props.c` | 10 | ~80,000 | 0 |
| **Total** | | **47** | **~360,000** | **1** |

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
```

Tests also run automatically in CI (`cmake.yml` and `ci-sanitizers-fuzz.yml`) on every push and PR. Under ASan/UBSan, the random inputs also catch undefined behavior and memory errors.
