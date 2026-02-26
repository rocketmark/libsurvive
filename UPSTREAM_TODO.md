# Stagehand → libsurvive Upstream Contributions

Patches developed for [Stagehand](https://github.com/marks/stagehand) (a USB/IP tracker transport
for virtual production). Tested on a Raspberry Pi 4B running a Vive Tracker 3 (LHR-C6FDCE70) over
USB/IP with a 2-lighthouse LH gen 2 setup. All patches are in `stagehand/agent/patches/`.

---

## Priority 1 — Submit as a single PR (defensive NaN/solver fixes)

These three patches guard against corrupted optical data cascading into crashes or bad calibration
saves. They are independent of any Stagehand-specific architecture and should apply cleanly.

### `mpfit_nan_guard` — Skip NaN optical angles in `poser_mpfit.c`

**File:** `src/poser_mpfit.c`
**Function:** `construct_input_from_scene()`

**Problem:** If a non-finite (NaN/inf) angle reaches the MPFIT pose solver, the solver asserts or
produces garbage results. This can happen with corrupted optical data during USB disturbances (bad
FPGA timestamps). We observed this as an `mp_qrsolv` assertion failure that killed the process.

**Fix:** Skip measurements where `!isfinite(a[axis])` before adding them to the solver input.

**Before:**
```c
const FLT *a = scene->angles[sensor][lh];
survive_optimizer_measurement *meas = survive_optimizer_emplace_meas(...);
```

**After:**
```c
const FLT *a = scene->angles[sensor][lh];
if (!isfinite(a[axis])) {
    /* log once and skip */
    continue;
}
survive_optimizer_measurement *meas = survive_optimizer_emplace_meas(...);
```

**Note for upstream PR:** Replace the `static int warned` once-only suppression with a debug-level
log or rate-limited message — the once-only pattern is fine for our use case but may hide
recurring issues upstream.

---

### `gss_maxiter_guard` — Treat `MP_MAXITER` as failure in `solve_global_scene`

**File:** `src/poser_mpfit.c`
**Function:** `solve_global_scene()`

**Problem:** The upstream code treats only `res <= 0` as a solver failure. `MP_MAXITER` (solver
hit iteration cap without converging) returns a positive value and is incorrectly treated as
success, allowing a non-converged GSS solution to be saved to disk as calibration data.

**Fix:** One-line change:
```c
// Before:
bool status_failure = res <= 0;

// After:
bool status_failure = res <= 0 || res == MP_MAXITER;
```

This is unambiguously correct: a solver that didn't converge has not produced a valid result.

---

### `variance_nan_guard` — Drop non-finite values in `variance_measure_add`

**File:** `redist/variance.h`
**Function:** `variance_measure_add()`

**Problem:** The upstream code asserts `isfinite(d[i])` after accumulating the value. If the
assert fires, the process crashes AND the corrupted partial state may have already been written to
`meas->sum` / `meas->sumSq` (the assert is after the `addnd` call), potentially corrupting the
variance estimate even if the process were to recover.

**Fix:** Check for non-finite values *before* accumulating, and skip the measurement with a
warning rather than asserting:
```c
for (int i = 0; i < meas->size; i++) {
    if (!isfinite(d[i])) {
        fprintf(stderr, "[libsurvive] variance_measure_add: non-finite d[%d]=%f, dropping\n", i, (double)d[i]);
        return;
    }
}
// ... then accumulate as before, without the assert
```

**Note for upstream PR:** Remove the "Stagehand patch" comment before submitting.

---

## Priority 2 — Submit separately (USB fix, needs context)

### `clear_halt` — Call `libusb_clear_halt()` before `libusb_submit_transfer()` at attach

**File:** `src/driver_vive.c`
**Function:** `AttachInterface()`

**Problem:** If a USB endpoint was left in a halted state from a previous session (e.g. after an
unclean disconnect), `libusb_submit_transfer()` fails immediately. The endpoint never receives
transfers and the tracker appears silent.

**Fix:** Call `libusb_clear_halt(devh, endpoint_num)` once, just before submitting the first
transfer. This is a standard USB recovery operation and is safe to call even when the endpoint is
not halted.

```c
libusb_clear_halt(devh, endpoint_num);   // clear any stale halt from previous session
int rc = libusb_submit_transfer(tx);
```

**Note for upstream PR:** This is particularly relevant for USB/IP and embedded setups where
device state may persist across agent restarts without a full USB re-enumerate. Worth mentioning
this in the PR description.

---

## Priority 3 — File as bug report (architecture mismatch, fix needs redesign)

### `buttons_timeout` — EP 0x84 false-positive in the 10s silence exit

**File:** `src/driver_vive.libusb.h`
**Function:** `handle_transfer()`

**Problem:** The upstream 10s USB silence exit fires on *all* endpoints including EP 0x84
(Buttons). The Vive Tracker sends a ~10s status heartbeat on the Buttons interface even when no
buttons are pressed. With a 1000ms transfer timeout, `consecutive_timeouts` reaches 9 between
heartbeats — if the heartbeat is even 1 second late, the exit fires as a false positive.

**Our fix:** Exclude EP 0x84 from the timeout counter (`is_tracking_ep = (endpoint != 0x84)`).
Only IMU (0x81) and Lightcap (0x83) trigger the exit.

**Why not upstreamable as-is:** Our `_exit(1)` + systemd-restart design is Stagehand-specific.
Upstream libsurvive is typically run interactively. The fix would need to be redesigned for
upstream — possibly a configurable timeout per endpoint, or simply extending the threshold on the
Buttons interface.

**Action:** File a bug report describing the false-positive condition and the EP 0x84 heartbeat
timing, so upstream can decide on the right fix.

---

## Priority 4 — Low priority / optional

### `lightcap_unknown_report` — Downgrade unknown lightcap report from ERROR to WARN

**File:** `src/driver_vive.c`
**Function:** `survive_data_cb_locked()`

**Problem:** An unknown USB lightcap report type triggers `SV_ERROR`, which terminates the
process. This is overly aggressive — the unknown report can be logged and skipped without
affecting tracking.

**Fix:**
```c
// Before:
SV_ERROR(SURVIVE_ERROR_HARWARE_FAULT, "USB lightcap report is of an unknown type ...");

// After:
SV_WARN("USB lightcap report is of an unknown type ...; ignoring");
```

**Caveat:** The upstream author may have intentionally made this fatal to catch new firmware
variants. Worth raising as a discussion in the PR or issue rather than just changing it.

---

## Observed in production (for PR context)

From `journalctl -u stagehand-agent` on stagehand01 (2026-02-26):

- Agent runs stably for 10+ minutes at ~118 packets/s (7150 packets/min).
- After Lightcap EP 0x83 goes silent for 10s, agent exits cleanly and systemd restarts it.
- On restart, GSS calibration completes in ~70s (3 scenes, sensor_err < 0.0001 rad).
- `gss_maxiter_guard` prevented at least one corrupt calibration save during testing.
- `mpfit_nan_guard` prevented at least one `mp_qrsolv` assertion crash during testing.
