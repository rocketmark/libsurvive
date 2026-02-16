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

## Remaining Tests to Implement

### 3. Kabsch Point-Set Registration (`linmath.c`)

Kabsch computes the optimal rigid transform aligning two point clouds. Used in calibration and multi-lighthouse alignment. Tests would cover:

| Property | What it proves |
|---|---|
| Identity recovery | `Kabsch(pts, pts)` -> identity pose |
| Known transform recovery | Apply random pose to pts -> Kabsch recovers the pose |
| Residual minimality | After alignment, RMS distance between point sets near zero |
| Rotation-only (centered) | `KabschCentered` with pre-centered points -> same rotation as full Kabsch |
| Scale recovery | `KabschScaled` with uniformly scaled points -> recovers the scale factor |

High ROI because Kabsch involves SVD and multiple coordinate transforms. Off-by-one in matrix indexing, wrong transpose, or sign errors in the determinant check (for reflections) are classic bugs that property testing catches.

### 4. Kalman Predict State-Transition Consistency (`survive_kalman_tracker.c`)

The Kalman predict step propagates state forward in time. These tests verify invariants without hardware.

| Property | What it proves |
|---|---|
| Zero velocity -> pose unchanged | Predict with zero velocity/acceleration preserves pose |
| Quaternion stays normalized | After predict, `\|state.Pose.Rot\| = 1.0` |
| Predict(0) = identity | Zero time step -> state unchanged |
| Linearity for small dt | `predict(2*dt)` ~ `predict(dt)` applied twice |
| Covariance stays symmetric PSD | After predict, P = P^T and all eigenvalues >= 0 |

Medium-high ROI because the state transition has hand-coded axis-angle / quaternion conversions that can silently corrupt orientation.

### 5. Numeric Robustness / NaN Propagation

Fuzz math functions with adversarial inputs to verify graceful handling of degenerate cases.

| Property | What it proves |
|---|---|
| `quatnormalize(zero)` | Does not produce NaN (current code: `1/sqrt(0)` -> Inf) |
| `quatrotatevector` with non-unit q | Produces finite output |
| `reproject` with point at origin | No crash (division by zero in atan2) |
| `reproject` with point behind lighthouse | Handled gracefully (z > 0) |
| `normalize3d(zero)` | Does not produce NaN |
| `Kabsch` with coplanar/collinear points | SVD does not produce garbage |

Medium ROI because libsurvive has many `assert(isfinite(...))` checks, but asserts are disabled in release builds. These tests reveal cases where NaN/Inf silently propagates.

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
```

Tests also run automatically in CI (`cmake.yml` and `ci-sanitizers-fuzz.yml`) on every push and PR. Under ASan/UBSan, the random inputs also catch undefined behavior and memory errors.
