# Stagehand Patches to libsurvive

This fork ([rocketmark/libsurvive](https://github.com/rocketmark/libsurvive)) tracks [collabora/libsurvive](https://github.com/collabora/libsurvive) with patches for the [Stagehand](https://github.com/rocketmark/stagehand) project — a PoE-powered Raspberry Pi 4B that connects Vive Trackers over USB/IP to a Windows SteamVR PC for virtual production.

These patches fix bugs discovered while running libsurvive headless on a Pi with a single Vive Tracker 3 over USB (no wireless dongle, no HMD).

## Bug Fix Patches

### 1. Clear stalled endpoints on attach (`driver_vive.c`)

**Commit:** not yet committed to fork (applied via `stagehand/agent/patches/clear_halt.patch`)

On Linux, the kernel `usbhid` driver grabs tracker interfaces on plug-in. libsurvive auto-detaches it with `libusb_set_auto_detach_kernel_driver()`, but the IMU endpoint (0x81) can be left in a STALL state after detach. Without IMU data the Kalman filter never produces poses.

```c
// In AttachInterface(), before libusb_submit_transfer():
libusb_clear_halt(devh, endpoint_num);
```

**Status:** Tracked as a `.patch` file in stagehand, not yet rolled into this fork.

### 2. Transfer timeout handler silently abandons endpoint (`driver_vive.libusb.h`)

**Commit:** 9dc326d

When `handle_transfer()` received `LIBUSB_TRANSFER_TIMED_OUT`, the original code returned without resubmitting the transfer. The endpoint permanently stopped receiving data — no warning, no recovery. This is the primary cause of tracking dying after 1-6 minutes over USB/IP.

**Fix:** Always resubmit after timeout. Declare "device turned off" only after 10 consecutive timeouts (10 seconds of silence).

### 3. Transfer error handler double-increments and falls through (`driver_vive.libusb.h`)

**Commit:** 9dc326d

The error path had two bugs:
- `error_count++` appeared twice on the same path (`error_count++` then `if (error_count++ < 10)`)
- After a successful `libusb_submit_transfer()` retry, the code fell through to `goto disconnect` instead of returning

**Fix:** Single increment, `return` after successful resubmit, `goto disconnect` only after 10 consecutive errors.

### 4. No `libusb_clear_halt()` on STALL errors (`driver_vive.libusb.h`)

**Commit:** 9dc326d

When a transfer completed with `LIBUSB_TRANSFER_STALL`, libsurvive retried without clearing the halt condition — so the retry would also stall.

**Fix:** Call `libusb_clear_halt()` before retrying when `transfer->status == LIBUSB_TRANSFER_STALL`.

### 5. Global scene solver off-by-one allows extra solve (`driver_global_scene_solver.c`)

**Commit:** 9dc326d

`run_optimization()` and `check_object()` used `solve_counts > solve_count_max` which allowed N+1 solves instead of N. The 2nd solve (at ~7 minutes) incorporated a bad scene, causing the MPFIT error to jump from 68 to 4661 (68x), corrupting lighthouse positions and killing tracking permanently.

**Fix:** Changed to `solve_counts >= solve_count_max` in both locations.

### 6. Global scene solver flag=1 means unlimited (`driver_global_scene_solver.c`)

**Commit:** 9dc326d

The flag mapping `flag > 1 ? flag : -1` made `--globalscenesolver 1` set `solve_count_max = -1` (unlimited). Only values >= 2 were respected as actual limits.

**Fix:** Changed to `flag > 0 ? flag : -1` so `--globalscenesolver 1` means exactly 1 solve (initial calibration only).

### 7. Process noise t^7 explosion on IMU timing gaps (`survive_kalman_tracker.c`)

**Commit:** 9dc326d

The jerk-model process noise scales as t^7. libsurvive warns at dt > 500ms but does not cap dt. Over USB/IP, IMU timestamp gaps cause catastrophic P matrix growth:

| dt | t^7 | Effect |
|----|-----|--------|
| 1ms (normal) | 1e-21 | fine |
| 50ms | 8e-10 | fine |
| 350ms | 0.006 | variance gate triggers |
| 500ms | 0.008 | light gate triggers |
| 1s | 1.0 | NaN/Inf in filter |

Proved by assertion failure: `variance.h:18: variance_measure_add: Assertion 'isfinite(d[i])' failed.`

**Fix:** Cap `t` to 50ms at the top of `survive_kalman_tracker_process_noise()`. State prediction still uses the real dt; only uncertainty growth (Q matrix) is bounded.

### 8. Compiler warning fix (`survive_sensor_activations.c`)

**Commit:** 1d34e73

Moved `measured_dev` and `cnt` variable declarations to point of use to fix `-Werror=missing-field-initializers` style warnings. No behavioral change.

## Property Tests (new files)

Nine property test suites added in `src/test_cases/`:

| File | What it tests |
|------|---------------|
| `quat_props.c` | Quaternion math (normalization, rotation, slerp) |
| `kabsch_props.c` | Kabsch algorithm (rigid body alignment) |
| `kalman_props.c` | Kalman filter properties (covariance, prediction) |
| `numeric_props.c` | Numerical utilities (matrix ops, SVD) |
| `reproject_props.c` | Lighthouse reprojection model |
| `reproject_residual_props.c` | Reprojection residual calculations |
| `event_queue_props.c` | Event queue data structure |
| `residual_cascade_props.c` | Light error threshold cascade (currently disabled path) |
| `variance_gate_props.c` | Variance gate behavior, IMU gap sensitivity |

CI workflow: `.github/workflows/ci-property-tests.yml`
Documentation: `docs/property-tests.md`

## CI Changes

Upstream CI workflows (cmake, docker, nuget, wheels, publish-source) were removed and replaced with `ci-property-tests.yml` for the property test suite.

## Patch Status

| # | Patch | In fork? | In `clear_halt.patch`? | Upstream PR? |
|---|-------|----------|----------------------|--------------|
| 1 | Clear halt on attach | **NO** | YES | No |
| 2 | Timeout resubmit | YES | No | No |
| 3 | Error handler fix | YES | No | No |
| 4 | STALL clear_halt | YES | No | No |
| 5 | GSS off-by-one | YES | No | No |
| 6 | GSS flag mapping | YES | No | No |
| 7 | Process noise dt cap | YES | No | No |
| 8 | Compiler warning | YES | No | No |

**Known gap:** Patch #1 (clear halt on attach) exists only in `stagehand/agent/patches/clear_halt.patch` and is applied manually on the Pi. It should be committed to this fork so the fork is the single source of truth.

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

The stagehand agent wrapper passes `--globalscenesolver 1` by default, which requires patches #5 and #6 to work correctly.
