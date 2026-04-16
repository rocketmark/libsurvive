# Tracking Engine

**Created**: 2026-04-11
**Status**: Mapped from existing code
**Source**: Brownfield bootstrap via /map-codebase

## Context and Current State

The Tracking Engine is the scientific core of libsurvive. It takes calibrated
angle measurements (from the Protocol Intelligence cluster) and IMU readings
(from the Driver Stack) and produces smooth, continuous 6-DOF pose and velocity
estimates for tracked objects and lighthouses.

The cluster contains three conceptually distinct layers that are tightly
integrated in practice:

1. **Geometric Pose Solvers** — Instantaneous pose from angle correspondences
   (SVD, EPnP, MPFIT). These solve "given these angles right now, where is
   the object?"
2. **Kalman State Estimator** — Continuous state over time, fusing light
   observations and IMU. Solves "given everything we've seen, what is the
   best estimate of pose, velocity, and acceleration right now?"
3. **Math Infrastructure** — Matrix library (cnmatrix), Kalman filter library
   (cnkalman), and the symbolic code generation pipeline that produces
   analytically-differentiated C functions for both.

The cluster also contains `survive_reproject`, which implements the optical
forward model: given a known pose, predict what angles each sensor would report.
This model is used in both the optimizer residuals and the Kalman measurement
updates.

## Poser Architecture

### The Poser Contract

A poser is a pluggable pose-estimation backend. It receives a `PoserData` union
tagged with the event type, computes a pose, and reports back via callbacks:

```c
typedef int (*PoserCB)(SurviveObject *so, PoserData *pd);

// Poser reports object pose:
PoserData_poser_pose_func(pd, lighthouse_id, &pose);

// Poser reports lighthouse pose:
PoserData_lighthouse_pose_func(pd, lighthouse_id, &lh_pose);
```

The `PoserData` union carries:
- `POSERDATA_LIGHT` / `POSERDATA_LIGHT_GEN2` — a single angle measurement
- `POSERDATA_LIGHT_GEN1` — angle + acode + length (Gen1 specific)
- `POSERDATA_IMU` — accelerometer + gyroscope reading
- `POSERDATA_SYNC` — sync pulse event
- `POSERDATA_GLOBAL_SCENES` — multi-object, multi-lighthouse scene model
- `POSERDATA_DISASSOCIATE` — signal to clean up per-object state

Posers are loaded as plugins and selected via the `poser` config key (default:
`PoserMPFIT`). Multiple posers can run simultaneously on the same object via
chaining.

### Poser Result Routing (`poser.c`)

`PoserData_poser_pose_func()` does more than deliver a pose — it routes the
result to the Kalman tracker:

```
Poser reports pose
    │
    ├─ If custom callback installed → call it
    └─ Otherwise → survive_kalman_tracker_integrate_observation()
```

Lighthouse poses go through `PoserData_lighthouse_pose_func()`, which applies
scene normalization before passing to the lighthouse Kalman tracker:

```
Poser reports lighthouse pose
    │
    ├─ Frame normalization (gravity + Z-rotation to reference)
    ├─ Reference lighthouse selection (survive_get_reference_bsd)
    └─ survive_kalman_lighthouse_integrate_observation()
```

Scene normalization uses the object's accelerometer data to establish the
gravity direction, fixing the rotation ambiguity that would otherwise allow
an infinite family of lighthouse configurations to explain the same measurements.

## Geometric Pose Solvers

### Barycentric SVD (`poser_barycentric_svd.c`, `barycentric_svd/`)

The primary geometric solver. Implements the EPnP (Efficient Perspective-n-Point)
algorithm using a barycentric coordinate parameterization.

**Algorithm:**
1. Select 4 control points: centroid + 3 rotated basis vectors
2. Express each sensor 3D position as a weighted sum of control points
   (barycentric coordinates)
3. Each angle measurement gives a linear equation relating the projected
   control points to the observed angle
4. Stack all equations into matrix M; solve via SVD for control point
   positions in camera (lighthouse) frame
5. Recover rotation R and translation t from control point positions

**Key adaptation from standard EPnP:** Standard EPnP uses PCA to select
control points from the sensor distribution. VR sensor arrays (especially HMDs)
are nearly planar, which degenerates PCA-based control point selection. The
implementation uses a random rotation of the standard basis instead, which
avoids this degeneracy.

**Minimum measurements:** 8 by default (`epnp-required-meas` config). This is
conservative — 4 non-coplanar measurements are theoretically sufficient, but
8 provides numerical stability and outlier tolerance.

**Multi-lighthouse use:** When both lighthouses have measurements, the solver
runs independently per lighthouse and computes a blended pose. If only one
lighthouse is available, that lighthouse's result is used directly.

**Lighthouse calibration solving:** If the object pose is known but a lighthouse
position is not, the solver inverts the problem. The accelerometer is used to
validate gravity alignment of candidate solutions (rejects if >25° misalignment).

### EPnP (`poser_epnp.c`)

An alternative PnP solver with similar structure to BaryCentricSVD but using
a different control point selection strategy. Less commonly used in practice.
Projects angle measurements as `tan(angle)` to u/v coordinates before solving.

### General Optimizer Wrapper (`poser_general_optimizer.c`)

A meta-poser that wraps a seed poser (default: BaryCentricSVD) and validates
solutions against error thresholds. Implements failure recovery:

- Tracks consecutive successes and failures
- Resets the seed poser if failures exceed `failures-to-reset` threshold
- Passes `no_lighthouse_solve` and `assume_current_pose` flags to the seed poser
  for warm-start behavior after successful tracking is established
- Re-enables lighthouse solving after `successes-to-reset` good frames

This layer insulates the MPFIT optimizer from the cost of re-solving lighthouse
positions on every frame when tracking is stable.

### MPFIT Optimizer (`poser_mpfit.c`)

The primary production poser. Runs Levenberg-Marquardt nonlinear least squares
(via `redist/mpfit/`) to refine poses from the BaryCentricSVD seed.

**What it optimizes:** The parameter vector contains:
- Object pose (position + quaternion or axis-angle)
- Optional: object velocity (free parameter for smooth trajectories)
- Optional: per-lighthouse calibration corrections (scale, offset)

**Residuals:** For each (sensor, lighthouse, axis) triple with a valid angle
reading: `residual = observed_angle - reproject(sensor_position, object_pose, lh_pose, calibration)`

**Jacobians:** Analytically computed from the generated code in `src/generated/`.
Numerical fallback available but slower.

**Measurement batching:** Collects a time window of measurements rather than
solving instantaneously. The window widens when the object is stationary
(more time = more stable measurements). Variance per measurement increases
with age: `var = base_var + age_seconds × var_per_sec`.

**Async mode:** Can run the solver in a background thread (`survive_async_optimizer`)
to prevent latency spikes on slow solves. The main thread continues collecting
measurements while the background thread processes the previous batch.

**Serialization:** Can dump per-lighthouse subproblems to disk for offline
analysis (`serialize-lh-mpfit` config flag).

### Kalman-Only Poser (`poser_kalman_only.c`)

A fallback poser that runs the Kalman filter on IMU data alone, without
lighthouse input. Used when optical tracking is unavailable. Provides dead
reckoning for brief occlusions. Runs 30 dummy observations at startup to warm
the filter covariance before accepting real IMU data.

### Dummy Poser (`poser_dummy.c`)

Logs data without solving. Emits identity pose on each IMU update. Used for
pipeline debugging.

## Kalman State Estimator

### Object Tracker (`survive_kalman_tracker.c`)

The primary state estimator. Maintains a continuous 19-dimensional state
estimate for each tracked object using an Extended Kalman Filter (EKF):

```
State vector (19D):
  [0:2]   Position (x, y, z)
  [3:5]   Velocity (vx, vy, vz)
  [6:9]   Rotation quaternion (qw, qx, qy, qz)
  [10:12] Angular velocity (wx, wy, wz)
  [13:15] Acceleration (ax, ay, az)
  [16:18] Gyro bias (bx, by, bz)
```

An additional IMU bias model (separate Kalman state) tracks:
- Accelerometer scale
- IMU frame correction quaternion
- Accelerometer bias

**Error-state EKF:** Rotation is maintained as a quaternion (non-Euclidean
manifold), but Kalman updates operate on axis-angle error states. After each
update: `q_new = q_old ⊗ exp(Δθ/2)`, where Δθ is the axis-angle correction
from the Kalman gain.

**Process model:** Jerk-based (integrates acceleration → velocity → position,
angular velocity → rotation). Process noise parameters are separately tunable
for each state component:
- `process-weight-jerk` — acceleration change rate
- `process-weight-rotation` — rotation noise
- `process-weight-vel` — velocity noise
- `process-weight-acc` — acceleration noise
- `process-weight-ang-vel` — angular velocity noise
- `process-weight-gyro-bias` — gyro bias drift rate

**Measurement models integrated:**

| Model | Source | Observation |
|-------|--------|-------------|
| `obj_imu` | IMU | Predicted gravity vector in object frame; corrects gyro bias |
| `obj_lightcap` | Light angles | Raw angle per (sensor, lighthouse, axis); variance learned per channel |
| `obs_pose` | Poser output | 6-DOF pose with covariance |
| `zvu` | IMU (stationary) | Zero velocity update: forces v=0 and a=0 when stationary |
| `joint` | Light angles | Simultaneous object + lighthouse pose update |

**Adaptive measurement covariance:** The lightcap measurement noise covariance R
is updated from residuals: `R_k = 0.3×R_{k-1} + 0.7×(residual² + H×P×H^T)`.
This allows the filter to learn per-sensor noise levels from experience.

**Per-LH adaptive R (jitter reduction):** In addition to per-sensor adaptive R, each
lighthouse's base noise covariance is scaled by the ratio of that LH's EWMA residual to the
fleet mean: `lh_var = base_var * max(1.0, lh_res / mean_res)`. Lighthouses with
above-average residuals receive proportionally larger R values and contribute less Kalman
weight. The `light_residuals[lh]` EWMA (α=0.1) converges after ~20–30 samples per LH, so
the effect takes 1–2 minutes to fully stabilize. See `docs/reflection-rejection.md` for the
full implementation details and a not-yet-implemented cold-start ramp that could reduce this
warmup time.

**Per-LH innovation gate:** Where `light-outlier-threshold > 0`, the system performs a
dry-run of the measurement model before each LH batch update. If the pre-update RMS
innovation for a lighthouse exceeds `threshold × light_residuals_all`, that LH batch is
skipped for the current sync cycle. This fires on the frame of the anomaly itself (transient
reflections, interference), unlike the adaptive R which responds to persistent residual
history. Disabled by default; recommended starting value 5.0. See
`docs/reflection-rejection.md` for implementation details.

**`light_residuals_all` floor (`1e-3` rad):** To prevent the EWMA baseline from collapsing
to near-zero during unusually clean tracking, `light_residuals_all` is floored at `1e-3`
before computing the gate threshold. Without the floor, baseline residuals ~0.0001 rad
would produce an effective threshold of 5.0 × 0.0001 = 0.0005 rad — below typical clean-
tracking RMS (~0.001 rad) — causing the gate to fire spuriously on valid batches. With the
floor, the minimum threshold is 5.0 × 1e-3 = 0.005 rad, safely above clean noise.
Implementation: `linmath_max(tracker->light_residuals_all, 1e-3)` in
`survive_kalman_tracker.c`.

**Stationary detection and ZVU:** When IMU variance falls below threshold
(`kalman-stationary-*` config), the filter applies a Zero Velocity Update —
a pseudo-measurement forcing velocity and acceleration to zero. This prevents
the filter from drifting during stationary periods.

**Outlier rejection:** Light measurements with residuals above
`kalman-light-threshold-var` are rejected before the update step. This prevents
large outliers (e.g., reflected light, interference) from corrupting the filter.

**Input-level lightcap angular rate gate (`lc-angular-rate-max`):** Before
accepting a lightcap batch, the filter predicts the pose at the batch timestamp
and computes the angular distance from the last accepted batch's pose. If the
implied angular rate (rad/s) exceeds `lc_angular_rate_max`, the batch is
dropped without updating the Kalman state. This is a pre-update gate — the
filter state is never touched for rejected batches. After a batch passes, the
reference rotation and timestamp are updated for use on the next batch.

Config: `--lc-angular-rate-max` (default: disabled). The reference is stored in
`last_accepted_lc_rot` / `last_accepted_lc_time` in `SurviveKalmanTracker`.

**Note on "defending wrong state":** This gate can lock out valid data if a
prior reflection batch corrupted the Kalman state before the gate could fire —
subsequent valid batches appear inconsistent with the corrupted state and are
themselves rejected. The back-face filter in Protocol Intelligence (`Stage 1`
of `SurviveSensorActivations_check_outlier`) addresses this by preventing
reflections from entering the pipeline before the Kalman state can be corrupted.

**Output-level pose angular rate gate (`kalman-max-pose-angular-rate`):** After
a pose is computed in `report_state()`, the gate compares it to the last
*emitted* pose. If the angular rate between the two exceeds the threshold, the
pose is suppressed (not emitted to the application) and `stats.dropped_poses`
is incremented. Unlike the input gate above, the Kalman internal state is
already updated when this gate fires — it prevents bad output but does not
prevent internal state drift.

Config: `--kalman-max-pose-angular-rate` (default: -1, disabled). Reference
stored in `last_reported_pose_rot` / `last_report_time`.

See `docs/reflection-rejection.md` (Change 2) for implementation details.

**Iterative IEKF updates for light:** Light measurements use the Iterated EKF
(IEKF) update, which re-linearizes around the current estimate after each step.
This handles the nonlinearity of the reprojection model and prevents
destabilization from linearization errors when the initial pose estimate is far
from truth.

### Lighthouse Tracker (`survive_kalman_lighthouses.c`)

A simpler 7D Kalman filter (position + quaternion) for each lighthouse. Updated
by pose estimates from the geometric solvers. Much lighter than the object
tracker since lighthouses are stationary.

Measurement models:
- **IMU model** — Predicts gravity direction from accelerometer; corrects
  lighthouse orientation via gravity alignment
- **Observation model** — Direct 6-DOF pose observations from calibration solvers

Supports push/pop state for hypothesis testing (try a lighthouse position,
validate, roll back if rejected).

## Reprojection Model (`survive_reproject.c`, `survive_reproject_gen2.c`)

The reprojection model maps a known 3D sensor position and object/lighthouse
poses to a predicted angle. It is the forward model used to compute residuals
in both the MPFIT optimizer and the Kalman lightcap measurement.

**Gen1 model:**
```
// Transform sensor from object to world frame
p_world = lh_pose.rot * (obj_pose.rot * sensor_pos + obj_pose.pos) + lh_pose.pos

// Project to lighthouse angle (lighthouse -Z is forward)
raw_x = atan2(p_world.x, -p_world.z)
raw_y = atan2(p_world.y, -p_world.z)

// Apply calibration
angle_x = raw_x - phase_x - tilt_x×raw_y - curve_x×raw_y² - gibmag_x×sin(raw_x + gibpha_x)
angle_y = raw_y - phase_y - tilt_y×raw_x - curve_y×raw_x² - gibmag_y×sin(raw_y + gibpha_y)
```

**Gen2 model:** Adds a polynomial series correction (`calc_cal_series`) using
6 empirical coefficients, plus OGEE (ogeephase, ogeemag) sine modulation.
The two laser planes are tilted at ±30° rather than being orthogonal, requiring
a different angle decomposition.

**Coordinate convention:** Lighthouse frame has -Z forward, +X right, +Y up.
The 180° Y-axis rotation (`{0,0,1,0}` quaternion) that appears in the poser
code corrects for this when converting between lighthouse and camera conventions.

### Generated Jacobians

The reprojection functions are generated by Python (SymEngine/SymPy) in
`tools/generate_math_functions/`. The generator:
1. Defines the reprojection math symbolically
2. Differentiates with respect to each parameter (pose components, calibration)
3. Applies common-subexpression elimination (CSE)
4. Emits optimized `static inline` C functions

This produces per-function Jacobians like `survive_reproject_full_jac_obj_pose`
with ~95 intermediate variables. These are used by MPFIT for gradient computation
and by the Kalman filter for linearization.

Both quaternion and axis-angle parameterizations are generated, selected at
compile time via the `axis_angle_mode` flag in the Python source.

## Math Infrastructure

### cnmatrix (`libs/cnmatrix/`)

A thin matrix library wrapping BLAS/LAPACK. Provides:
- `CnMat` struct: rows, cols, stride, `FLT*` data pointer
- Operations: GEMM, SVD, inversion (LU/SVD), solve, Cholesky
- Stack allocation macro: `CN_CREATE_STACK_MAT(name, rows, cols)` — uses `alloca`
  to avoid heap allocation in hot paths

Backends selectable at build time: BLAS+LAPACK (default), Eigen (optional).
The `FLT` type (float or double) is configured globally and propagates through
all matrix operations.

### cnkalman (`libs/cnkalman/`)

An EKF/IEKF library that `survive_kalman_tracker` is built on. Key interfaces:

- `cnkalman_state_t` — state vector + covariance P + transition model
- `cnkalman_meas_model_t` — measurement model with Jacobian callback
- `cnkalman_predict_state()` — prediction step: advance state and grow P
- `cnkalman_meas_model_predict_update()` — update step: compute K, apply gain

Supports error-state EKF via `cnkalman_error_state_init()`, where the state
update function handles manifold constraints (e.g., quaternion normalization).

Jacobian modes:
- **User-provided** — analytical Jacobians from generated code (fastest)
- **Numerical 1-sided** — `(f(x+δ) - f(x)) / δ`
- **Numerical 2-sided** — `(f(x+δ) - f(x-δ)) / 2δ` (more accurate, slower)

Adaptive R update (from residuals) is built into the measurement model and
used by the lightcap measurement in the object tracker.

The IEKF implementation (`iekf.c`) iterates the linearization until convergence,
using a cost function: `v = 0.5 × (y^T R^{-1} y + Δx^T P^{-1} Δx)`. Termination
reasons are enumerated for diagnostics.

### Code Generation Pipeline (`tools/generate_math_functions/`)

The symbolic math pipeline that generates the C code in `src/generated/`:

```
SymEngine / SymPy (Python)
    │
    ├── common_math.py      — quaternion, pose, rotation primitives
    ├── lighthouse_gen1.py  — Gen1 reprojection model + calibration
    ├── lighthouse_gen2.py  — Gen2 reprojection model + calibration
    ├── imu_functions.py    — IMU prediction, gravity model, Kalman dynamics
    ├── reprojection_functions.py — wraps gen1/gen2, emits both parameterizations
    │
    └── codegen.py          — CSE optimizer + C emitter
            │
            ▼
    src/generated/*.gen.h   — static inline C functions with Jacobians
```

The codegen performs CSE to extract common subexpressions into temporaries
(`x0 = ..., x1 = ...`), dramatically reducing operation count for the large
Jacobian matrices. It also avoids `pow()` for integer exponents, instead
expanding to multiplications.

The `libs/cnkalman/cnkalman/codegen.py` extends this with struct-aware
generation: it can produce C code that accesses fields via `offsetof` macros,
enabling generated code that directly reads from C structs without manual
argument unpacking.

## Async Optimizer (`survive_async_optimizer.c`)

A double-buffered wrapper that runs MPFIT in a background thread to prevent
latency spikes:

```
Main thread:
  writes measurements → buffer[active]
  when batch ready → signal worker
  immediately available for next batch

Worker thread:
  reads buffer[!active] (previous batch)
  runs mpfit solver
  on completion → callback(result)
  ready for next signal
```

The double-buffer scheme ensures the main thread is never blocked waiting for
the solver to finish. The tradeoff is one frame of latency — poses are reported
one batch behind.

## Data Flow Through the Cluster

```
Protocol Intelligence cluster
    │  SurviveSensorActivations (angles per sensor/LH/axis)
    │  BaseStationCal (lighthouse calibration params)
    ▼
survive_process.c / survive_kalman_tracker (light ingestion path)
    │
    ├─[raw angle path]──► survive_kalman_tracker_integrate_light()
    │                          │  obj_lightcap measurement model
    │                          │  reproject(sensor, obj_pose, lh_pose, cal)
    │                          │  residual → IEKF update
    │                          ▼
    │                      Updated state estimate
    │
    └─[poser path]─────► PoserMPFIT (via poser dispatch)
                               │  BaryCentricSVD seed → rough pose
                               │  Levenberg-Marquardt refinement
                               │    residuals = angle - reproject(...)
                               │    Jacobians from src/generated/
                               │  Refined pose + covariance
                               ▼
                           PoserData_poser_pose_func()
                               │  survive_kalman_tracker_integrate_observation()
                               │  obs_pose measurement update
                               ▼
Driver Stack (IMU)         Updated state estimate
    │
    │  IMU (accel, gyro)
    ▼
survive_kalman_tracker_integrate_imu()
    │  obj_imu measurement: predicted_gravity = rot × [0,0,-1]
    │  gyro: update angular velocity, correct bias
    │  ZVU: if stationary, force velocity → 0
    ▼
Updated state estimate

survive_kalman_tracker_report_state()
    │  pose + velocity output via hook
    ▼
Library Infrastructure (pose hooks → application)
```

## Observed Design Decisions

| Decision | What was chosen | Evidence | Likely rationale |
|---|---|---|---|
| Error-state EKF for rotation | Quaternion state, axis-angle updates | `cnkalman_error_state_init`, generated `SurvivePoseToErrorModel` | Quaternion is 4D for a 3D quantity; error-state avoids renormalization complexity in the filter matrices |
| Analytical Jacobians via codegen | Python SymEngine → CSE-optimized C | `tools/generate_math_functions/`, `src/generated/*.gen.h` | Manual derivation error-prone; numerical Jacobians too slow for real-time; codegen gives best of both |
| Iterative EKF (IEKF) for light updates | Re-linearize around current estimate | `iekf.c`, `cnkalman_meas_model_predict_update` with iteration | Reprojection is strongly nonlinear; single linearization diverges when initial pose error is large; iterating maintains stability |
| MPFIT seed + refine architecture | BaryCentricSVD seed, MPFIT refine | `poser_general_optimizer.c`, `poser_mpfit.c` | SVD gives fast geometric solution; Levenberg-Marquardt refines with calibration model accounted for; each does what it's good at |
| Random rotation for SVD control points | Avoids PCA degeneracy on planar sensors | `barycentric_svd.c` comment at line ~143 | HMD sensor arrays are nearly planar; PCA would produce ill-conditioned control point basis |
| Separate lighthouse Kalman tracker | 7D filter independent of object tracker | `survive_kalman_lighthouses.c` | Lighthouses are stationary; 19D object tracker state is unnecessary; separate filter allows independent lighthouse refinement |
| Double-buffered async optimizer | Two buffers, worker thread processes previous batch | `survive_async_optimizer.c` | Prevents main thread blocking on long solves; one-frame latency tradeoff acceptable for VR |
| Adaptive R from residuals | R updated per-measurement from observed residuals | `cnkalman` adaptive R | Sensor noise varies by position, occlusion, reflection; learned R is more accurate than fixed R |
| Per-LH adaptive R scaling | `lh_var = base_var * max(1.0, lh_res / mean_res)` | `survive_kalman_tracker.c` per-LH batch loop; `light_residuals[lh]` EWMA | Fixed R gives equal Kalman weight to all LHs; miscalibrated LHs pull state proportionally harder with more LHs active, causing jitter that grows with LH count |
| Per-LH innovation gate disabled by default | `light_outlier_threshold = 0`; threshold of 5.0 recommended | Config item in `survive_kalman_tracker.c` | Gate fires on any anomalous frame; conservative default avoids false positives on new hardware or unusual geometry; opt-in once threshold is calibrated for the environment |
| Input-level lightcap angular rate gate | `lc_angular_rate_max`; pre-update, drops batch before Kalman touch | `SurviveKalmanTracker.lc_angular_rate_max`, `last_accepted_lc_rot` in header | Prevents high-velocity batches from being applied; pairs with back-face filter — filter prevents the "defending wrong state" failure mode where this gate itself locks out valid data after a corruption |
| Output-level pose angular rate gate | `max_pose_angular_rate`; post-update, suppresses emission | `survive_kalman_tracker_report_state()`, `stats.dropped_poses` | Guards downstream consumers from sudden pose jumps; Kalman state already updated, so this is a backstop for output quality, not state integrity |
| Back-face filter as upstream prevention | Geometry check in Protocol Intelligence before Kalman | `SurviveSensorActivations_check_outlier()`, `filterNormalFacingness` | Reflections enter the Kalman pipeline only if the filter misses them; back-face filter eliminates the most common class (impossible geometry) before any downstream gate can be challenged |
| FLT type (float/double configurable) | All math uses `FLT` typedef | `libs/cnmatrix/include/cnmatrix/cn_flt.h` | Float gives 2× speed on SIMD for acceptable precision; double available for calibration tools |

## Technical Debt & Inconsistencies

1. **`poser_epnp.c` is largely redundant** — EPnP and BaryCentricSVD solve
   the same problem with similar algorithms. It's unclear which is preferred
   or whether EPnP is maintained. No documentation distinguishes when to use one
   over the other.

2. **`poser_general_optimizer.c` coupling** — The general optimizer wraps
   BaryCentricSVD by name via config string (`seed-poser`), creating a hidden
   dependency. If BaryCentricSVD is removed or renamed, the default breaks
   silently.

3. **Hardcoded Gen2 polynomial coefficients** — `lighthouse_gen2.py` line ~52:
   `[-8.0108022e-06, 0.0028679863, ...]`. Origin undocumented in source.
   Appears empirically fit; if wrong, Gen2 tracking accuracy is silently degraded.

4. **100+ config keys in Kalman tracker** — `survive_kalman_tracker.c` exposes
   an extremely large configuration surface. Many combinations of these
   parameters produce unstable behavior. There's no documented "safe zone" of
   parameters.

5. **Covariance initialization is empirical** — Initial P matrix values are
   configured via `kalman-initial-*` config keys with no derivation documented.
   Wrong initialization causes slow convergence or divergence on startup.

6. **`src/generated/` files are checked in** — Generated files are committed to
   the repository. When the Python source changes, the generated files must be
   manually regenerated. There is no build step that enforces consistency.

7. **`survive_reproject.aux.generated.h` naming** — The "aux" generated file
   contains the axis-angle pose application function. The naming doesn't indicate
   its relationship to the main reprojection path.

8. **IMU bias model separation** — The IMU bias is tracked in a separate
   Kalman state from the main object state. The interaction between them
   (which updates which, in what order) is not documented.

## Behavioral Quirks

1. **30 dummy IMU observations at startup** — `poser_kalman_only.c` runs 30
   dummy observations with high variance before accepting real data. This warm-up
   prevents the filter from making large corrections on the first real measurement.

2. **Lighthouse Z-axis is backwards** — The lighthouse coordinate frame has -Z
   forward. A `{0,0,1,0}` quaternion (180° Y rotation) correction appears in
   multiple places. This is intentional but trips up anyone working in lighthouse
   coordinates.

3. **MPFIT measurement variance grows with age** — Older angle measurements
   are trusted less. This means the effective batch window is self-weighting
   toward recent measurements without explicit windowing logic.

4. **ZVU is a pseudo-measurement, not a constraint** — Stationary detection
   does not hard-constrain velocity to zero; it adds a soft zero-velocity
   observation. The filter can still drift slowly if process noise is high.

5. **Gravity validation rejects lighthouses at >25°** — If the accelerometer
   indicates the candidate lighthouse pose would require gravity to be >25° from
   vertical, the lighthouse pose is rejected. This assumes the tracked object is
   in a normal orientation when calibration runs.

6. **IEKF termination is silent** — The IEKF iterates until convergence or
   max iterations. If it terminates on max iterations (divergence), no warning
   is surfaced to the caller. The tracker proceeds with whatever state it reached.

## Open Questions

1. Is `poser_epnp.c` actively maintained? It appears to solve the same problem
   as `poser_barycentric_svd.c`. Is there a scenario where EPnP outperforms
   BaryCentricSVD?

2. The codegen pipeline produces files committed to the repo. What is the
   intended workflow when the math changes — is there a script to regenerate,
   or is it done manually?

3. The IMU bias model (separate from the main Kalman state) — what is the
   update cadence? Is it updated on every IMU measurement or only periodically?

4. The `process-weight-*` config keys interact in undocumented ways. Is there
   a tuning guide or recommended starting values for new device types?

## References

**Files in this cluster:**

Posers:
- [src/poser.c](../../src/poser.c)
- [src/poser_barycentric_svd.c](../../src/poser_barycentric_svd.c)
- [src/poser_epnp.c](../../src/poser_epnp.c)
- [src/poser_mpfit.c](../../src/poser_mpfit.c)
- [src/poser_kalman_only.c](../../src/poser_kalman_only.c)
- [src/poser_general_optimizer.c](../../src/poser_general_optimizer.c), [src/poser_general_optimizer.h](../../src/poser_general_optimizer.h)
- [src/poser_dummy.c](../../src/poser_dummy.c)
- [src/barycentric_svd/barycentric_svd.c](../../src/barycentric_svd/barycentric_svd.c), [src/barycentric_svd/barycentric_svd.h](../../src/barycentric_svd/barycentric_svd.h)

Kalman / optimizer:
- [src/survive_kalman_tracker.c](../../src/survive_kalman_tracker.c), [src/survive_kalman_tracker.h](../../src/survive_kalman_tracker.h)
- [src/survive_kalman_lighthouses.c](../../src/survive_kalman_lighthouses.c), [src/survive_kalman_lighthouses.h](../../src/survive_kalman_lighthouses.h)
- [src/survive_optimizer.c](../../src/survive_optimizer.c)
- [src/survive_async_optimizer.c](../../src/survive_async_optimizer.c), [src/survive_async_optimizer.h](../../src/survive_async_optimizer.h)
- [src/survive_reproject.c](../../src/survive_reproject.c)
- [src/survive_reproject_gen2.c](../../src/survive_reproject_gen2.c)

Generated math:
- [src/generated/common_math.gen.h](../../src/generated/common_math.gen.h), [src/generated/common_math.py](../../src/generated/common_math.py)
- [src/generated/imu_model.gen.h](../../src/generated/imu_model.gen.h), [src/generated/imu_model.py](../../src/generated/imu_model.py)
- [src/generated/kalman_kinematics.gen.h](../../src/generated/kalman_kinematics.gen.h), [src/generated/kalman_kinematics.py](../../src/generated/kalman_kinematics.py)
- [src/generated/survive_imu.generated.h](../../src/generated/survive_imu.generated.h)
- [src/generated/survive_reproject.aux.generated.h](../../src/generated/survive_reproject.aux.generated.h)
- [src/generated/survive_reproject.generated.h](../../src/generated/survive_reproject.generated.h)
- [src/generated/survive_types.py](../../src/generated/survive_types.py)
- [tools/generate_math_functions/](../../tools/generate_math_functions/)

Math libraries:
- [libs/cnkalman/](../../libs/cnkalman/)
- [libs/cnmatrix/](../../libs/cnmatrix/)

**Dependencies on other clusters:**
- → **Library Infrastructure**: uses `SurviveContext`, config system, hook callbacks, `SV_MALLOC`, plugin registration
- → **Lighthouse Protocol Intelligence**: consumes `SurviveSensorActivations` (angles) and `BaseStationCal`
- ← **Device Driver Stack**: `driver_global_scene_solver` calls poser functions directly (unusual reverse dependency)

**External dependencies:**
- BLAS / LAPACK — matrix operations in cnmatrix
- Eigen (optional) — alternative cnmatrix backend
- `redist/mpfit/` — Levenberg-Marquardt solver
- SymEngine / SymPy (Python, build-time) — symbolic math for codegen
- `redist/linmath.h` — quaternion and vector math utilities
