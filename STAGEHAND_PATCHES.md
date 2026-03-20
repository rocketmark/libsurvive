# Stagehand Patches to libsurvive

This fork ([rocketmark/libsurvive](https://github.com/rocketmark/libsurvive)) tracks [collabora/libsurvive](https://github.com/collabora/libsurvive) with patches for the [Stagehand](https://github.com/rocketmark/stagehand) project — a PoE-powered Raspberry Pi 4B that connects Vive Trackers over USB/IP to a Windows SteamVR PC for virtual production.

These patches fix bugs discovered while running libsurvive headless on a Pi with a single Vive Tracker 3 over USB (no wireless dongle, no HMD).

## Current State

Patches #1, #7, #2, #3, #4, and #8 are applied. Patches #5 and #6 (GSS off-by-one and
flag mapping) remain reverted — the wrapper's `--globalscenesolver 3` mitigates #6.

**Feb 2026 diagnostic confirmation:** Instrumented agent (imu_age logging) confirmed
that the dropout is USB-level: IMU stops first (~4 min), poses stop ~30s later (Kalman
coasting). Patch #7 was re-applied first (NaN crash on cold start), then #2-4 (endpoint
abandonment on timeout). Cold start must be re-verified after each deploy.

## Bug Fix Patches

### 1. Clear stalled endpoints on attach (`driver_vive.c`) — APPLIED

**Status:** Applied in fork via `clear_halt.patch`.

On Linux, the kernel `usbhid` driver grabs tracker interfaces on plug-in. libsurvive auto-detaches it with `libusb_set_auto_detach_kernel_driver()`, but the IMU endpoint (0x81) can be left in a STALL state after detach. Without IMU data the Kalman filter never produces poses.

```c
// In AttachInterface(), before libusb_submit_transfer():
libusb_clear_halt(devh, endpoint_num);
```

### 2. Transfer timeout handler silently abandons endpoint (`driver_vive.libusb.h`) — REVERTED

When `handle_transfer()` receives `LIBUSB_TRANSFER_TIMED_OUT`, upstream returns without resubmitting the transfer. The endpoint permanently stops receiving data — no warning, no recovery. This is the primary cause of tracking dying after 1-6 minutes over USB/IP.

**Planned fix:** Always resubmit after timeout. Declare "device turned off" only after 10 consecutive timeouts (10 seconds of silence).

### 3. Transfer error handler double-increments and falls through (`driver_vive.libusb.h`) — REVERTED

The upstream error path has two bugs:
- `error_count++` appears twice on the same path (`error_count++` then `if (error_count++ < 10)`)
- After a successful `libusb_submit_transfer()` retry, the code falls through to `goto disconnect` instead of returning

**Planned fix:** Single increment, `return` after successful resubmit, `goto disconnect` only after 10 consecutive errors.

### 4. No `libusb_clear_halt()` on STALL errors (`driver_vive.libusb.h`) — REVERTED

When a transfer completes with `LIBUSB_TRANSFER_STALL`, upstream retries without clearing the halt condition — so the retry also stalls.

**Planned fix:** Call `libusb_clear_halt()` before retrying when `transfer->status == LIBUSB_TRANSFER_STALL`.

### 5. Global scene solver off-by-one allows extra solve (`driver_global_scene_solver.c`) — REVERTED

`run_optimization()` and `check_object()` use `solve_counts > solve_count_max` which allows N+1 solves instead of N. The 2nd solve (at ~7 minutes) incorporated a bad scene, causing the MPFIT error to jump from 68 to 4661 (68x), corrupting lighthouse positions and killing tracking permanently.

**Planned fix:** Change to `solve_counts >= solve_count_max` in both locations.

### 6. Global scene solver flag=1 means unlimited (`driver_global_scene_solver.c`) — REVERTED

The upstream flag mapping `flag > 1 ? flag : -1` makes `--globalscenesolver 1` set `solve_count_max = -1` (unlimited). Only values >= 2 are respected as actual limits.

**Planned fix:** Change to `flag > 0 ? flag : -1` so `--globalscenesolver 1` means exactly 1 solve (initial calibration only).

**Note:** With upstream code, `--globalscenesolver 3` passes through correctly (flag > 1 → flag = 3). The wrapper now uses `--globalscenesolver 3` to avoid this issue.

### 7. Process noise t^7 explosion on IMU timing gaps (`survive_kalman_tracker.c`) — REVERTED

The jerk-model process noise scales as t^7. libsurvive warns at dt > 500ms but does not cap dt. Over USB/IP, IMU timestamp gaps cause catastrophic P matrix growth:

| dt | t^7 | Effect |
|----|-----|--------|
| 1ms (normal) | 1e-21 | fine |
| 50ms | 8e-10 | fine |
| 350ms | 0.006 | variance gate triggers |
| 500ms | 0.008 | light gate triggers |
| 1s | 1.0 | NaN/Inf in filter |

**Confirmed:** NaN assertion crash observed on cold start: `linmath.c:658: quatrotateabout: Assertion '!isnan(qout[i])' failed`. This is the highest priority patch to re-apply.

**Planned fix:** Cap `t` to 50ms at the top of `survive_kalman_tracker_process_noise()`. State prediction still uses the real dt; only uncertainty growth (Q matrix) is bounded.

### 8. Compiler warning fix (`survive_sensor_activations.c`) — APPLIED

**Commit:** 1d34e73

Moved `measured_dev` and `cnt` variable declarations to point of use to fix `-Werror=missing-field-initializers` style warnings. No behavioral change.

### 10. Atomic config write (`survive_config.c`) — APPLIED

`config_save()` previously used `fopen(path, "w")` which truncates the destination file immediately, then wrote with multiple `fprintf()` calls. A power loss during any write left a truncated or empty `config.json`. On the next start libsurvive either failed to parse it or silently discarded it, forcing a full recalibration from a corrupted baseline.

**Fix:** Write to `config.json.tmp` first, then `rename()` into place. `rename()` is atomic on Linux — either the old file survives intact or the new file is fully committed, never a partial.

```c
char tmp_path[FILENAME_MAX];
snprintf(tmp_path, sizeof(tmp_path), "%s.tmp", path);
FILE *f = fopen(tmp_path, "w");
// ...write...
fclose(f);
rename(tmp_path, path);
```

### 11. Lock GSS solver after first tracking (`driver_global_scene_solver.c`) — APPLIED

After initial calibration is established, lighthouse wake events deliver fresh OOTX data which calls `ootx_recv()` → `set_needs_solve()` → schedules a new GSS solve. Mid-session re-solves use scene data captured during the lighthouse transition (noisy, partial) and can produce a bad calibration that corrupts tracking for the remainder of the session.

**Fix:** Add an early return in `set_needs_solve()` guarded by `flushed_blind_scenes` (set by patch #9's `gss_flush_blind_scenes` logic once the first good pose is produced). Once tracking is established, no further re-solves are triggered regardless of lighthouse events. A restart is the correct response to genuine scene changes (lighthouse moved or replaced).

```c
if (gss->flushed_blind_scenes)
    return;
```

### 9. Reflection artifact rejection — APPLIED

Three source changes plus a bonus bug fix to reject reflection-contaminated sensor readings and
poses before they corrupt the Kalman state. Reflections (LED walls, truss, shiny floors) cause
libsurvive to accept ghost poses that are geometrically consistent but physically wrong.

- **Back-facing normal filter** (`survive_sensor_activations.c`, `survive.h`): rejects sensor hits
  where the sensor surface normal points away from the lighthouse. Configurable via
  `--filter-normal-facingness` (default 0.0) and `--filter-normal-min-confidence` (default 0.1).
- **Pose angular rate gate** (`survive_kalman_tracker.c`, `survive_kalman_tracker.h`): suppresses
  pose emission when the implied angular rate exceeds a threshold. Disabled by default
  (`--kalman-max-pose-angular-rate -1`); requires calibration from `reflect_test.cap` data before
  enabling (expected value: 5–10 rad/s).
- **Configurable sync cluster window** (`survive_sensor_activations.c`, `survive.h`): makes the
  0.5s Chauvenet cluster window configurable via `--sync-cluster-window` (default 0.5; tighten to
  0.15 for more reactive outlier detection).
- **quatdist bug fix** (`redist/linmath.c`): pre-existing bug where swapped min/max clamp args
  caused `quatdist()` to always return 0. Fixed: `linmath_max(1., linmath_min(-1, rtn))` →
  `linmath_min(1., linmath_max(-1., rtn))`. This made the angular rate gate silently inert before
  the fix was applied. Candidate for upstream PR to `collabora/libsurvive`.

Full details: `docs/reflection-rejection.md`

## Property Tests (new files)

Ten property test suites added in `src/test_cases/`:

| File | What it tests |
|------|---------------|
| `quat_props.c` | Quaternion math (normalization, rotation, slerp, quatdist) |
| `kabsch_props.c` | Kabsch algorithm (rigid body alignment) |
| `kalman_props.c` | Kalman filter properties (covariance, prediction) |
| `numeric_props.c` | Numerical utilities (matrix ops, SVD, sync cluster window) |
| `reproject_props.c` | Lighthouse reprojection model |
| `reproject_residual_props.c` | Reprojection residual calculations |
| `event_queue_props.c` | Event queue data structure |
| `residual_cascade_props.c` | Light error threshold cascade (currently disabled path) |
| `variance_gate_props.c` | Variance gate behavior, IMU gap sensitivity |
| `normal_filter_props.c` | Back-facing normal filter geometry (reflection rejection) |

CI workflow: `.github/workflows/ci-property-tests.yml`
Documentation: `docs/property-tests.md`, `docs/reflection-rejection.md`

## CI Changes

Upstream CI workflows (cmake, docker, nuget, wheels, publish-source) were removed and replaced with `ci-property-tests.yml` for the property test suite.

## Patch Status

| # | Patch | Applied? | Confirmed? | Re-apply priority |
|---|-------|----------|------------|-------------------|
| 1 | Clear halt on attach | YES | YES | — |
| 2 | Timeout resubmit | **APPLIED** | YES (1-6 min dropout) | — |
| 3 | Error handler fix | **APPLIED** | YES (code review) | — |
| 4 | STALL clear_halt | **APPLIED** | YES (code review) | — |
| 5 | GSS off-by-one | **REVERTED** | Confirmed corruption 60→15198→963755/meas; patch broke tracking, needs investigation | investigate |
| 6 | GSS flag mapping | **REVERTED** | flag=1 broke tracking (OOTX stuck); needs investigation | investigate |
| 7 | Process noise dt cap | **APPLIED** | YES (NaN crash on cold start) | — |
| 8 | Compiler warning | YES | YES | — |
| 9 | Reflection artifact rejection | **APPLIED** | YES (field confirmed) | — |
| 10 | Atomic config write | **APPLIED** | Pending deploy | — |
| 11 | Lock GSS after first tracking | **APPLIED** | Pending deploy | — |

## Re-apply Order

Patches should be re-applied one at a time. After each patch, deploy to Pi and verify cold start calibration still succeeds (both lighthouses detected, OOTX decoded, GSS solves, tracking goes green).

1. **Patch #7** (dt cap) — prevents NaN crash on cold start, confirmed by assertion failure
2. **Patches #2–4** (USB transfer handler) — prevents 1-6 minute dropout over USB/IP
3. **Patches #5–6** (GSS) — prevents 7-minute re-solve corruption

## Lessons Learned

- **Cold start calibration is fast (~20s)** when it works: OOTX decode takes ~18s, GSS solves in ~1s after that. The stuck-yellow failures were caused by the agent's 120s init timeout and the NaN crash, not slow calibration.
- **Test patches in isolation.** When multiple patches interact (GSS flag mapping + agent init timeout + NaN crash), failures are hard to attribute.

## Build

```bash
mkdir build && cd build
cmake .. -DUSE_HIDAPI=OFF
make -j4
```

For property tests:
```bash
cmake .. -DUSE_HIDAPI=OFF -DENABLE_TESTS=ON
make -j4
ctest --output-on-failure
```

## Runtime

The stagehand agent wrapper passes `--globalscenesolver 3` by default. With upstream GSS code (patches #5–6 reverted), this correctly limits to 3 solves. With patches #5–6 applied, any value >= 1 works as expected.
