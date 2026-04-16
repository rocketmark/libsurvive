# Reflection Artifact Rejection

Specular reflections (LED walls, truss, shiny floors) cause libsurvive to accept ghost sensor
readings that are geometrically consistent but physically wrong — the lighthouse sweep light
bounces off a surface and arrives at a sensor from the wrong direction. The Kalman filter accepts
these readings as valid, corrupts its internal state, and emits bad poses. Recovery is slow because
the internal model has already drifted.

This document describes the changes made to libsurvive to reject reflection-contaminated readings
and poses before they corrupt the Kalman state, plus the companion stagehand-level gate and the
quatdist bug found during implementation.

---

## Background: Why Reflections Are Hard

libsurvive processes lighthouse sweep hits at the sensor level (one hit = one sensor, one axis,
one lighthouse). By the time a bad pose is emitted it has already been integrated into the Kalman
state. The stagehand Python layer (`shtp_receiver.py`) gates bad output poses at 25°/frame, but
the Kalman state has already been updated by then. Recovery after a reflection burst is therefore
slow — the internal model has drifted and needs many good readings to pull it back.

**Empirical data from `reflect_test.cap`:**

| Measurement | Value |
|---|---|
| Smallest reflection jump | 30° |
| Largest reflection jump | 122.7° |
| Max jumps in a 2-second window | 15 |
| libsurvive reaction | Never loses tracking — accepts ghost pose as valid |

Reflections arrive as rotation-dominated bursts with no epoch transitions. libsurvive's existing
Chauvenet outlier filter and variance gate don't catch them because the reflected readings are
internally consistent — the reflected light traces a valid geometric path, just from the wrong
surface.

---

## Existing Stagehand Gate (Python Layer)

**File:** `stagehand/windows/stagehand_client/shtp_receiver.py`

Before the libsurvive-level changes were made, stagehand implemented a Python-side gate:

```python
REFLECTION_JUMP_DEG    = 25.0   # per-frame rotation jump threshold
REFLECTION_WINDOW_S    = 5.0    # sliding window to count jumps
REFLECTION_THRESHOLD   = 3      # jumps in window before muting output
REFLECTION_CLEAR_S     = 10.0   # silence period before allowing output again
REFLECTION_FILTER_ENABLED = True
```

**What it does:** If 3+ rotation jumps > 25° are detected in a 5-second window, pose output from
that tracker is muted for 10 seconds, and the stagehand application layer is notified.

**Limitation:** This gates bad *output* only. libsurvive's Kalman state is already corrupted by
the time the Python layer sees the pose jump. The 10-second mute period covers recovery time, not
an actual fix. The libsurvive-level changes below address the root cause.

**Note:** The developer-guide documentation at `stagehand/docs/developer-guide.md:493` incorrectly
states the default threshold is 15°. The actual value in code is 25°.

---

## Change 1 — Back-Facing Normal Filter

**Goal:** Reject individual sensor angle readings where the sensor's surface normal points away
from the lighthouse. A sensor cannot receive direct lighthouse light from a direction it doesn't
face — such readings are geometrically impossible and are almost certainly reflections.

**Reference:** This filter exists for the simulator in `src/driver_simulator.c:152–163`. These
changes port it to the real hardware path.

### Files Modified

#### `include/libsurvive/survive.h`

Added to `SurviveSensorActivations_params`:

```c
FLT filterNormalFacingness;     // minimum dot product; < -0.5 disables filter
FLT filterNormalMinConfidence;  // minimum pose confidence before filter activates
```

#### `src/survive_sensor_activations.c`

Added config items (makes the params tunable at runtime via CLI):

```c
STRUCT_CONFIG_ITEM("filter-normal-facingness",
    "Min dot product (sensor normal vs direction-to-lighthouse) to accept hit; < -0.5 disables",
    0.0, t->params.filterNormalFacingness)
STRUCT_CONFIG_ITEM("filter-normal-min-confidence",
    "Min pose confidence required before normal filter activates",
    0.1, t->params.filterNormalMinConfidence)
```

Added at the top of `SurviveSensorActivations_check_outlier()`, before the existing delta check:

```c
// Back-facing normal filter: reject hits from directions the sensor cannot face
if (self->params.filterNormalFacingness > -0.5 &&
    self->so && self->so->sensor_normals && self->so->sensor_locations &&
    self->so->poseConfidence >= self->params.filterNormalMinConfidence) {
    SurviveContext *ctx = self->so->ctx;
    if (ctx->bsd[lh].PositionSet) {
        LinmathVec3d normalInWorld;
        quatrotatevector(normalInWorld, self->so->OutPose.Rot, self->so->sensor_normals + sensor_id * 3);
        LinmathVec3d sensorInWorld;
        ApplyPoseToPoint(sensorInWorld, &self->so->OutPose, self->so->sensor_locations + sensor_id * 3);
        LinmathVec3d towardLh;
        sub3d(towardLh, ctx->bsd[lh].Pose.Pos, sensorInWorld);
        normalize3d(towardLh, towardLh);
        FLT facingness = dot3d(normalInWorld, towardLh);
        if (facingness < self->params.filterNormalFacingness) {
            SV_VERBOSE(105, "Rejecting back-facing hit: facingness %+7.4f (threshold %+7.4f) for %2d.%2d.%d",
                       facingness, self->params.filterNormalFacingness, lh, sensor_id, axis);
            return true;
        }
    }
}
```

**Guards:**
- `filterNormalFacingness < -0.5` → disabled (can be turned off without recompiling)
- `so->sensor_normals == NULL || so->sensor_locations == NULL` → skip (not all devices have normals)
- `poseConfidence < filterNormalMinConfidence` → skip during cold-start when pose is unreliable
- `!ctx->bsd[lh].PositionSet` → skip if lighthouse not yet calibrated

**Default behavior:** `filterNormalFacingness = 0.0` means facingness must be ≥ 0 (sensor facing
within 90° of the lighthouse). Sensors pointing more than 90° away from the lighthouse are
rejected. This matches the simulator's threshold.

---

## Change 2 — Pose Angular Rate Gate

**Goal:** After the Kalman tracker predicts a new pose, compare it to the last reported pose. If
the implied angular rate exceeds a threshold, suppress the pose emission. This prevents bad Kalman
output from reaching the application and limits (though does not eliminate) Kalman state
contamination.

**Note on state contamination:** The Kalman internal state is updated by
`cnkalman_meas_model_predict_update()` before `report_state` is called. This gate prevents bad
poses from being *emitted*, but the filter state may still drift slightly. Full state revert
(save/restore the P matrix) is a possible follow-up if drift proves problematic in practice.

### Files Modified

#### `src/survive_kalman_tracker.h`

Added after `last_light_time`:

```c
FLT max_pose_angular_rate;           // rad/s; -1 disables gate
LinmathQuat last_reported_pose_rot;  // rotation of last emitted pose (for angular rate gate)
```

#### `src/survive_kalman_tracker.c`

Added config item:

```c
STRUCT_CONFIG_ITEM("kalman-max-pose-angular-rate",
    "Maximum angular rate (rad/s) before suppressing pose output; -1 to disable",
    -1., t->max_pose_angular_rate)
```

Added in `survive_kalman_tracker_report_state()` after the pose predict call:

```c
if (tracker->max_pose_angular_rate > 0 && tracker->last_report_time > 0 &&
    quatmagnitude(tracker->last_reported_pose_rot) > 0.5) {
    FLT dt = t - tracker->last_report_time;
    if (dt > 0) {
        FLT ang_rad = quatdist(tracker->last_reported_pose_rot, pose.Rot);
        FLT ang_rate = ang_rad / dt;
        if (ang_rate > tracker->max_pose_angular_rate) {
            SV_VERBOSE(105, "Rejecting high angular rate pose: %.2f rad/s (threshold: %.2f) for %s",
                       ang_rate, tracker->max_pose_angular_rate, survive_colorize_codename(so));
            tracker->stats.dropped_poses++;
            return;
        }
    }
}
```

Updated `last_reported_pose_rot` just before `tracker->previous_state = tracker->state`:

```c
quatcopy(tracker->last_reported_pose_rot, pose.Rot);
```

**Default:** `-1` (disabled). The threshold must be calibrated from reflection test data before
enabling in production (see Calibration section below).

---

## Change 3 — Configurable Sync Cluster Window

**Goal:** The 0.5-second window used to compute the per-(lighthouse, axis) cluster mean for
Chauvenet outlier rejection was hard-coded. Making it configurable allows tightening the window so
the cluster is more reactive to current sensor state, making the Chauvenet filter more effective
at catching reflection-onset readings.

### Files Modified

#### `include/libsurvive/survive.h`

Added to `SurviveSensorActivations_params`:

```c
FLT syncClusterWindowS;  // seconds; default 0.5 preserves existing behaviour
```

#### `src/survive_sensor_activations.c`

Added config item:

```c
STRUCT_CONFIG_ITEM("sync-cluster-window",
    "Time window for sync cluster calculation in seconds",
    0.5, t->params.syncClusterWindowS)
```

Replaced the hard-coded constant in `SurviveSensorActivations_add_sync()`:

```c
// was: bool isRecent = timecode - sensor_timecode < 48000000 / 2;
survive_long_timecode sync_window_ticks =
    (survive_long_timecode)(self->params.syncClusterWindowS * 48000000.0);
bool isRecent = timecode - sensor_timecode < sync_window_ticks;
```

---

## Bonus: quatdist Bug Fix

During implementation of the angular rate gate, a pre-existing bug was discovered in
`redist/linmath.c` that caused `quatdist()` to always return 0 — making the gate silently inert.

**File:** `redist/linmath.c:299`

**Bug:** Min and max arguments were swapped:

```c
// WRONG — linmath_min(-1, rtn) is always ≤ -1, so linmath_max(1, ...) is always 1.0
// acos(FLT_FABS(1.0)) = 0.0 for every input
rtn = linmath_max(1., linmath_min(-1, rtn));
```

**Fix:**

```c
// CORRECT — clamps dot product to [-1, 1] before acos
rtn = linmath_min(1., linmath_max(-1., rtn));
```

This bug has been present since the function was written. Any code using `quatdist()` to measure
angular distance between poses was silently receiving 0 for all inputs. The `QuatDistKnownAngle`
property test was written specifically to catch this class of bug and confirmed the fix.

**Upstream:** This fix should be submitted to `collabora/libsurvive` as a standalone PR. The
fix is isolated to one line, and the new `quat_props.c` test suite can accompany it as verification.

---

## Property Tests Added

### `src/test_cases/quat_props.c` — 4 new tests for `quatdist`

Added to the existing `quat_props.c` test suite:

| Test | Property |
|---|---|
| `QuatDistSelfIsZero` | `quatdist(q, q) = 0` for any unit quaternion |
| `QuatDistSymmetric` | `quatdist(q1, q2) = quatdist(q2, q1)` |
| `QuatDistNonNegativeAndBounded` | Result always in `[0, π]` |
| `QuatDistKnownAngle` | `quatdist(identity, q_θ) ≈ θ` for quaternion encoding known rotation θ |

`QuatDistKnownAngle` is the test that caught the swapped-clamp bug.

### `src/test_cases/normal_filter_props.c` — 7 tests

Property tests for the back-facing normal filter geometric invariants:

| Test | Property |
|---|---|
| `SensorNormalBodyToWorldTransform` | sensor_normals[] are in body frame: `quatrotatevector(rot, normal_body)` produces the correct world-frame normal for 90°/180°/identity rotations |
| `FacingnessInRange` | Dot product of unit normal and unit direction is always in `[-1, 1]` |
| `FacingnessFlipsWithDirection` | `dot(n, d) + dot(n, -d) = 0` |
| `FacingnessKnownAngle` | Facingness equals `cos(θ)` for sensor at known angle θ from lighthouse |
| `FacingnessThresholdMonotonic` | Threshold at `f−ε` accepts, threshold at `f+ε` rejects |
| `DirectlyFacingAlwaysAccepted` | Sensor normal aligned with lighthouse always passes (facingness ≈ 1.0) |
| `BackFacingAlwaysRejected` | Sensor normal opposing lighthouse always rejects at threshold ≥ 0 |

### `src/test_cases/numeric_props.c` — 3 new sync window tests

Added to the existing `numeric_props.c` suite:

| Test | Property |
|---|---|
| `SyncWindowDefaultMatchesHardcoded` | `0.5 * 48000000.0 == 48000000 / 2` (new default matches old constant) |
| `SyncWindowTicksMonotonic` | Larger window seconds → larger tick count |
| `SyncWindowZeroExcludesAll` | Zero window excludes all readings |

### `src/test_cases/CMakeLists.txt`

Added `normal_filter_props` to `SURVIVE_TESTS`.

---

## Runtime Tuning

All new parameters are tunable via CLI flags — no recompile needed to adjust thresholds.

| Flag | Default | Description |
|---|---|---|
| `--filter-normal-facingness` | `0.0` | Min dot product (sensor normal vs to-LH). `< -0.5` disables. |
| `--filter-normal-min-confidence` | `0.1` | Min pose confidence before normal filter activates. |
| `--kalman-max-pose-angular-rate` | `-1` (disabled) | Max pose angular rate in rad/s before suppressing emission. |
| `--sync-cluster-window` | `0.5` | Cluster window in seconds (tightening to 0.15 makes Chauvenet more reactive). |

**Existing flags worth tuning for reflection resistance:**

| Flag | Current default | Recommended starting point |
|---|---|---|
| `--filter-threshold-ang-per-sec` | `50.0` | `30.0` (tighter, back off if false positives) |
| `--filter-light-outlier-criteria` | `0.5` | `0.70` |
| `--light-max-error` | `-1` (disabled) | `0.01` |

---

## Calibration

The normal filter and sync window can be enabled immediately. The angular rate gate requires
calibration from real data before enabling.

### Step 1 — Collect baseline angular rate data

Run with `--survive-verbose 105` and the angular rate gate disabled (default `-1`):

```bash
./survive-cli --survive-verbose 105 --filter-normal-facingness 0.0 \
              --filter-normal-min-confidence 0.1 --sync-cluster-window 0.15 \
              --kalman-max-pose-angular-rate -1
```

Grep the log for the verbose output added by the angular rate gate code path (by temporarily
lowering the suppress-threshold to something high like 10000 to log without rejecting):

```bash
./survive-cli --survive-verbose 105 --kalman-max-pose-angular-rate 10000 2>&1 \
    | grep "Rejecting high angular rate" | awk '{print $6}' | sort -n
```

This gives the distribution of angular rates during a clean tracking session. The gate threshold
should be set above the 99th percentile of clean rates and below the smallest reflection jump.

From the empirical data, the smallest reflection jump was 30° in one frame. At 60 Hz that's ~3.1
rad/s. A clean tracker should be well below 1 rad/s in normal VP operation. Start the gate at
**5–10 rad/s** and adjust.

### Step 2 — Verify normal filter rejection

With `--survive-verbose 105`, look for:

```
Rejecting back-facing hit: facingness -0.xxxx (threshold +0.0000) for LH.sensor.axis
```

These should appear during reflection bursts on the affected device and not during clean tracking.

### Step 3 — End-to-end pass criterion

The stagehand Python `REFLECTION_THRESHOLD` counter (log line: `REFLECTION BURST DETECTED`) should
trigger rarely or not at all during the reflection test scenario with libsurvive-level filters
active. Use this as the end-to-end acceptance check.

---

## Change 4 — Per-LH Innovation Gate

**Goal:** Detect and skip lighthouse batches whose pre-update Kalman innovation is anomalously
large in the current sync cycle. Reflections and transient interference cause one lighthouse's
sensor readings to spike while others remain clean — the innovation gate fires on the frame of
the spike itself, preventing that batch from corrupting the Kalman state before it propagates.

**Difference from the angular rate gate (Change 2):** The angular rate gate fires *after* the
Kalman update has been applied and a pose has been emitted. The innovation gate fires *before*
the update — the bad LH batch is never applied to the filter state at all.

**Difference from adaptive R (Fix #2 / Jitter Reduction section below):** Adaptive R
down-weights lighthouses that are *persistently* high-residual. The innovation gate fires on
*sudden* anomalous frames regardless of the LH's historical residual — it targets transient
events like reflections.

### Files Modified

#### `src/survive_kalman_tracker.h`

Added between `lightcap_max_error` and `light_rampin_length`:

```c
FLT light_outlier_threshold;
```

#### `src/survive_kalman_tracker.c`

Config item:

```c
STRUCT_CONFIG_ITEM("light-outlier-threshold",
                   "Per-LH RMS innovation gate: skip a LH batch if its pre-update RMS residual "
                   "exceeds this multiple of light_residuals_all (0 = disabled)", 0, t->light_outlier_threshold)
```

Gate logic added before R construction inside `survive_kalman_tracker_integrate_saved_light()`,
in the per-LH batch loop:

```c
// @spec TE-PROC-039
if (tracker->light_outlier_threshold > 0 && tracker->light_residuals_all > 0) {
    CN_CREATE_STACK_VEC(y_dry, cnt);
    bool dry_ok = map_light_data(&cbctx, &Z, &tracker->model.state, &y_dry, NULL);
    if (dry_ok) {
        const FLT *yv = cn_as_const_vector(&y_dry);
        FLT sq = 0;
        for (int i = 0; i < cnt; i++) sq += yv[i] * yv[i];
        FLT rms = FLT_SQRT(sq / cnt);
        if (rms > tracker->light_outlier_threshold * tracker->light_residuals_all) {
            tracker->stats.lightcap_model_dropped++;
            continue;
        }
    }
}
```

`map_light_data` is called with `H_k=NULL` for a dry-run: no Jacobian is computed and no
state update is applied. Only the innovation vector `y` is produced.

**Recovery:** The gate re-evaluates every sync cycle with no persistent LH state. As soon as
the reflection clears and the innovation drops below the threshold, the lighthouse is included
again automatically on the next sync cycle — no manual reset required.

**Default:** `light_outlier_threshold = 0` (disabled in libsurvive itself). Stagehand sets
this to `5.0` via `DEFAULT_SURVIVE_ARGS` in `scripts/stagehand-health`, so it is active by
default in all Stagehand deployments. Recommended starting value for other integrations: `5.0`
(skip any LH batch whose RMS innovation exceeds 5× the fleet mean).

### Property Tests Added

6 tests added to `src/test_cases/residual_cascade_props.c` (OutlierGate suite):

| Test | Property |
|---|---|
| `DisabledWhenThresholdZero` | Gate never fires when threshold is 0 |
| `DisabledOnColdStart` | Gate never fires when `light_residuals_all` is 0 (no history yet) |
| `FiresAboveThreshold` | Gate fires when RMS > threshold × mean |
| `DoesNotFireBelowThreshold` | Gate passes when RMS ≤ threshold × mean |
| `RmsNonNegativeAndZeroOnlyWhenAllZero` | RMS is always ≥ 0; equals 0 only on zero input |
| `SingleLargeInnovationTriggers` | One outlier sensor in a batch is sufficient to trigger the gate |

---

## Jitter Reduction — Per-LH Adaptive Noise Scaling

**Problem:** A tracker sitting still exhibits positional jitter that grows with the number of
active lighthouses. All lighthouses used the same fixed observation noise covariance
(`light_var = 1e-2`). If any lighthouse has calibration error, it pulls the Kalman state in
its direction every sync cycle. More lighthouses = more conflicting pulls = higher-frequency
oscillation.

**Fix:** Scale each lighthouse's R matrix by the ratio of that LH's EWMA residual to the
fleet mean. Higher-residual lighthouses receive proportionally larger R values (less Kalman
weight), so their influence on the state is reduced relative to well-calibrated lighthouses.

**Result:** Jitter from a still tracker is measurably reduced. It may take 1–2 minutes after
startup to reach full effect — see the warmup note below.

### Files Modified

#### `src/survive_kalman_tracker.c`

Inside the per-LH batch loop in `survive_kalman_tracker_integrate_saved_light()`, the
fixed-R construction:

```c
FLT light_var = tracker->light_var;
```

is replaced with the per-LH adaptive version:

```c
// @spec TE-PROC-038
FLT base_var = tracker->light_var;
FLT mean_res = tracker->light_residuals_all;
FLT lh_res   = tracker->light_residuals[lh];
FLT lh_var   = (mean_res > 0 && lh_res > 0)
    ? base_var * linmath_max(1.0, lh_res / mean_res)
    : base_var;
```

`linmath_max(1.0, ...)` ensures the factor never goes below 1.0 — we only ever increase R
relative to the base, never decrease it. Falls back to `base_var` on cold start (no residual
history).

Per-LH EWMA update added after each LH batch is processed:

```c
tracker->light_residuals[lh] *= .9;
tracker->light_residuals[lh] += .1 * lh_rtn;
tracker->stats.lightcap_error_by_lh[lh] += lh_rtn;
tracker->stats.lightcap_count_by_lh[lh]++;
```

The `lightcap_error_by_lh` and `lightcap_count_by_lh` fields were declared in the header
but never written before this change — they are now populated.

### Warmup Behavior

The EWMA uses α=0.1, which means it needs approximately 20–30 samples per lighthouse to
converge to ~90% of the true mean residual. During this warmup window all lighthouses receive
approximately equal weight, and jitter resembles the pre-fix behavior. After convergence the
scaling ratio stabilizes and jitter decreases.

**Observed in practice:** Jitter visibly calms down after 1–2 minutes of tracking. This is
expected and correct — the EWMA is working as designed.

### Property Tests Added

6 tests added to `src/test_cases/residual_cascade_props.c` (AdaptiveR suite):

| Test | Property |
|---|---|
| `AlwaysAtLeastBaseVar` | `lh_var >= base_var` always — R is never reduced below baseline |
| `EqualResidualIsIdentity` | When `lh_res == mean_res`, `lh_var == base_var` (ratio = 1.0) |
| `HighResidualInflatesVar` | When `lh_res > mean_res`, `lh_var > base_var` |
| `ColdStartFallsBackToBaseVar` | When `mean_res == 0`, falls back to `base_var` |
| `MonotonicInLhResidual` | `lh_var` is non-decreasing as `lh_res` increases |
| `PerLhEWMAConvergesIndependently` | Two lighthouses with different residuals converge to different EWMA values |

### Future Work — Cold-Start Ramp (Not Yet Implemented)

If the 1–2 minute warmup period is unacceptable, the EWMA convergence time can be reduced
with a cold-start ramp: use a higher α during the first N updates on each lighthouse, then
switch to the steady-state α.

```c
// Conceptual implementation — not currently in code
size_t cnt_lh = tracker->stats.lightcap_count_by_lh[lh];
FLT alpha = (cnt_lh < LIGHT_RESIDUAL_RAMP_SAMPLES) ? 0.3 : 0.1;
tracker->light_residuals[lh] *= (1.0 - alpha);
tracker->light_residuals[lh] += alpha * lh_rtn;
```

Where `LIGHT_RESIDUAL_RAMP_SAMPLES` would be a constant around 20 (covering roughly the
first 2 seconds of lightcap data at typical update rates). After 20 frames at α=0.3, the
EWMA is already at ~99% of the true mean; subsequent frames use α=0.1 for long-run stability.

**Trade-off:** Higher α is more reactive — faster convergence but noisier, and more sensitive
to transient events during the ramp window. For a static tracker the tradeoff is probably
worth it. For a fast-moving tracker, the ramp could chase a transient reflection and skew
the initial residual estimate.

**Counter-argument for accepting the current warmup:** The 1–2 minute window isn't idle time
— the filter is stabilizing the LH poses via GSS in parallel. By the time EWMA converges,
the LH geometry estimates have also stabilized, so both sources of jitter reduce together.

---

## What to Do Next

- [ ] Run `reflect_test.cap` with `--survive-verbose 105` to observe actual per-pose angular rates
      during reflection bursts (on affected Pi) vs clean tracking (on clean Pi)
- [ ] Set `--kalman-max-pose-angular-rate` based on observed data (expected: 5–10 rad/s)
- [ ] Enable `--light-max-error 0.01` in the agent config (no recompile needed)
- [ ] Fix documentation: `stagehand/docs/developer-guide.md:493` says REFLECTION_JUMP_DEG default
      is 15° but the actual code value is 25°
- [ ] Submit the `quatdist` fix to `collabora/libsurvive` upstream (isolated one-line change +
      `quat_props.c` test file + CMakeLists update — branch off `upstream/master`, not `master`)
