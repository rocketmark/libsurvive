# Tracking Engine — EARS Specs

Prefix: **TE**

## Poser Interface

- [x] **TE-API-001**: The system shall allow posers to register as plugins via `REGISTER_POSER`, selected at runtime via the `poser` config key.
- [x] **TE-API-002**: When a poser computes an object pose, the system shall route the result to `survive_kalman_tracker_integrate_observation` unless a custom callback is installed.
- [x] **TE-API-003**: When a poser computes a lighthouse pose, the system shall apply gravity-based frame normalization before passing the result to the lighthouse Kalman tracker.
- [x] **TE-API-004**: When a `POSERDATA_DISASSOCIATE` event is received, the system shall release all per-object state held by the poser.
- [x] **TE-API-005**: Where the `poser` config key names a threaded poser, the system shall run the poser in a dedicated background thread with synchronized data delivery.

## Geometric Pose Solvers

- [x] **TE-PROC-010**: When BaryCentricSVD is invoked, the system shall select 4 control points (centroid + 3 randomly-rotated basis vectors) to avoid PCA degeneracy on planar sensor arrays.
- [x] **TE-PROC-011**: When BaryCentricSVD is invoked, the system shall require at least `epnp-required-meas` (default 8) angle measurements before attempting a solve.
- [x] **TE-PROC-012**: When both lighthouses provide measurements, BaryCentricSVD shall solve independently per lighthouse and blend the results.
- [x] **TE-PROC-013**: When the object pose is known but a lighthouse position is unknown, BaryCentricSVD shall solve for the lighthouse position and validate via gravity alignment (reject if >25° from vertical).
- [x] **TE-PROC-014**: The GeneralOptimizerWrapper shall track consecutive solve failures and reset the seed poser when failures exceed the `failures-to-reset` threshold.
- [x] **TE-PROC-015**: While tracking is stable (consecutive successes above `successes-to-reset`), the GeneralOptimizerWrapper shall pass `assume_current_pose=true` to the seed poser to enable warm-start behavior.

## MPFIT Nonlinear Optimizer

- [x] **TE-PROC-020**: The system shall refine object poses using Levenberg-Marquardt minimization of reprojection residuals: `residual = observed_angle - reproject(sensor, obj_pose, lh_pose, calibration)`.
- [x] **TE-PROC-021**: The system shall compute MPFIT Jacobians analytically from generated code in `src/generated/` rather than via numerical differentiation.
- [x] **TE-PROC-022**: The system shall increase measurement variance with age: `var = base_var + age_seconds × var_per_sec`, so older measurements are weighted less.
- [x] **TE-PROC-023**: When the tracked object is stationary (IMU stationary detection active), the system shall widen the MPFIT measurement time window to accumulate more stable angle data.
- [x] **TE-PROC-024**: Where `async-mpfit` is enabled, the system shall run the MPFIT solver in a background thread using double-buffering so the main thread is never blocked waiting for solver completion.
- [x] **TE-PROC-025**: Where `serialize-lh-mpfit` is enabled, the system shall write per-lighthouse MPFIT subproblems to disk for offline debugging.

## Kalman Object Tracker

- [x] **TE-PROC-030**: The system shall maintain a 19-dimensional error-state EKF per tracked object with state: position (3), velocity (3), rotation quaternion (4), angular velocity (3), acceleration (3), gyro bias (3).
- [x] **TE-PROC-031**: The system shall represent rotation as a quaternion in the state vector and apply Kalman updates as axis-angle corrections, renormalizing the quaternion after each update.
- [x] **TE-PROC-032**: When an IMU reading is received, the system shall update the Kalman filter using a measurement model that predicts the expected gravity vector in the object frame given the current rotation estimate.
- [x] **TE-PROC-033**: When a light angle measurement is received, the system shall update the Kalman filter using the Iterated EKF (IEKF), re-linearizing around the current estimate until convergence.
- [x] **TE-PROC-034**: The system shall reject light angle measurements whose residual exceeds the `kalman-light-threshold-var` threshold before the IEKF update step.
- [x] **TE-PROC-035**: The system shall update each lightcap measurement's noise covariance R adaptively from observed residuals: `R_k = 0.3×R_{k-1} + 0.7×(residual² + H×P×H^T)`.
- [x] **TE-PROC-036**: While the tracked object is stationary (IMU variance below `kalman-stationary-*` thresholds), the system shall apply a Zero Velocity Update pseudo-measurement forcing velocity and acceleration toward zero.
- [x] **TE-PROC-037**: When tracking is lost (no valid measurements for the configured timeout period), the system shall reset the Kalman state covariance to reflect high uncertainty.
- [x] **TE-PROC-038**: The system shall scale each lighthouse's observation noise covariance R by the ratio of that lighthouse's EWMA residual to the fleet mean residual, so that lighthouses with above-average residuals receive proportionally less Kalman weight.
- [x] **TE-PROC-039**: Where `light-outlier-threshold` is > 0, the system shall compute the pre-update RMS innovation for each lighthouse batch and skip that lighthouse's Kalman update for the current sync cycle if the RMS exceeds `light-outlier-threshold × light_residuals_all`.
- [x] **TE-PROC-040**: Where `lc-angular-rate-max` is > 0, the system shall predict the Kalman pose at each incoming lightcap batch's timestamp and compute the angular rate implied by the rotation change from the last accepted batch; if the rate exceeds `lc_angular_rate_max` rad/s the entire batch shall be dropped without modifying the Kalman state.
- [x] **TE-PROC-041**: When a lightcap batch passes the `lc-angular-rate-max` gate, the system shall update `last_accepted_lc_rot` and `last_accepted_lc_time` to the predicted pose at that batch's timestamp, so subsequent batches are gated against a current reference.
- [x] **TE-PROC-042**: Where `kalman-max-pose-angular-rate` is > 0, the system shall compute the angular rate between the current predicted pose and the last emitted pose; if the rate exceeds `kalman-max-pose-angular-rate` rad/s the system shall suppress pose emission for that frame and increment `stats.dropped_poses` without modifying the Kalman state.

## Kalman Lighthouse Tracker

- [x] **TE-PROC-050**: The system shall maintain a 7-dimensional Kalman filter per lighthouse (position + quaternion) updated by pose observations from the geometric solvers.
- [x] **TE-PROC-051**: The system shall support push/pop of lighthouse Kalman state to enable hypothesis testing without committing to a new lighthouse position.

## Reprojection Model

- [x] **TE-DATA-050**: The system shall implement the Gen1 reprojection forward model mapping (sensor 3D position, object pose, lighthouse pose, BaseStationCal) → (angle_x, angle_y).
- [x] **TE-DATA-051**: The system shall implement the Gen2 reprojection forward model with polynomial series correction and ±30° tilted-plane geometry.
- [x] **TE-DATA-052**: The system shall use the lighthouse coordinate convention of -Z forward, +X right, +Y up throughout the reprojection model.
- [x] **TE-DATA-053**: The system shall provide reprojection Jacobians with respect to object pose (6-DOF) for use in both MPFIT and the Kalman linearization step.

## Math Infrastructure

- [x] **TE-DATA-060**: The system shall provide matrix operations (multiply, SVD, invert, solve, Cholesky) via a BLAS/LAPACK wrapper that supports both float and double precision via the `FLT` compile-time type.
- [x] **TE-DATA-061**: The system shall support stack-allocated temporary matrices via `CN_CREATE_STACK_MAT` to avoid heap allocation in tight Kalman update loops.
- [x] **TE-DATA-062**: The system shall provide EKF and IEKF filter primitives (predict, update, error-state update) as a reusable library (cnkalman) independent of libsurvive domain types.
- [x] **TE-DATA-063**: The system shall generate all Kalman model functions (state transition, process noise, measurement models) and their Jacobians from symbolic Python definitions with common-subexpression elimination.
- [ ] **TE-DATA-064**: The system shall provide a build step or CI check to detect divergence between the Python symbolic sources in `tools/generate_math_functions/` and the committed generated headers in `src/generated/`.
