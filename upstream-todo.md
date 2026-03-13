# Upstream Contribution TODO

Patches that should be submitted to collabora/libsurvive.
Reference: https://github.com/collabora/libsurvive

## Already merged upstream

- **quatdist clamp fix** (`redist/linmath.c`) — `linmath_max(1., linmath_min(-1, rtn))` →
  `linmath_min(1., linmath_max(-1., rtn))`. Merged as https://github.com/collabora/libsurvive/pull/350. Removed from
  `reflection_rejection.patch`.

## PR submitted — awaiting review

- **`variance_nan_guard`** — https://github.com/collabora/libsurvive/pull/351
- **Process noise dt cap** — https://github.com/collabora/libsurvive/pull/352 (references collabora/libsurvive#346)
- **`mpfit_nan_guard`** — https://github.com/collabora/libsurvive/pull/353
- **`gss_maxiter_guard`** — https://github.com/collabora/libsurvive/pull/354
- **`lightcap_unknown_report`** — https://github.com/collabora/libsurvive/pull/355

---

## High priority — bug fixes, no controversy

These are small correctness fixes with no API surface. High acceptance probability.

### `clear_halt`
- **File**: `src/driver_vive.c`
- **What**: Calls `libusb_clear_halt()` on the endpoint before `libusb_submit_transfer()`
  in `AttachInterface`. Stalled endpoints must be cleared before re-submission.
- **PR pitch**: "driver_vive: clear_halt before submit_transfer in AttachInterface"

---

## Medium priority — improvements worth submitting

### `sync-cluster-window` config (hunk from `reflection_rejection`)
- **File**: `src/survive_sensor_activations.c`
- **What**: Makes the sync cluster time window configurable (was hardcoded to
  `48000000 / 2` ticks = 0.5s). Adds `sync-cluster-window` config item.
- **Note**: Extract as a standalone patch.
- **PR pitch**: "sensor_activations: make sync cluster window configurable"

### `gss_flush_blind_scenes`
- **File**: `src/driver_global_scene_solver.c`
- **What**: After first pose is established, discards GSS scenes captured while
  poseConfidence was zero (normal filter inactive). Without this, the first GSS calibration
  always uses unfiltered data regardless of any sensor-level filtering.
- **Note**: Only makes sense if the normal filter (from `reflection_rejection`) is also
  upstream. Submit together or after.
- **PR pitch**: "gss: flush pre-pose scenes captured before sensor filter was active"

---

## Stagehand-specific — do not submit upstream as-is

### `buttons_timeout`
- Uses `_exit(1)` and relies on systemd for restart. Not appropriate for general use.
- The two bug fixes embedded in it could be extracted:
  - Double `error_count++` increment (was `if (iface->error_count++ < 10)`)
  - Missing `libusb_clear_halt()` before stall retry

### `usb_reset_on_open`
- Calls `libusb_reset_device()` on every open. Only safe in Stagehand's single-device,
  hard-exit (`_exit(99)`) model. Would be disruptive in multi-device or graceful-close use.

---

## Notes

- cntools/libsurvive redirects to collabora/libsurvive — they are the same repo.
- Submit PRs to collabora/libsurvive.
- The `quatdifference` function (upstream addition) is unrelated to our work.
