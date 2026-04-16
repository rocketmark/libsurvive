# Back-Face Hit Rejection — High-Level Design

**Created**: 2026-04-16
**Feature area**: Reflection artifact rejection in libsurvive tracking pipeline

## Problem Statement

Lighthouse sweep light reflects off hard surfaces (LED walls, monitor bezels, glass, shiny
floors) and arrives at tracker sensors from the wrong direction. These reflected hits are
geometrically consistent — they trace a valid optical path — so libsurvive's existing
Chauvenet outlier filter and sensor variance gates do not reject them. The Kalman filter
accepts them, its internal state drifts, and the tracker emits bad poses ("wild
spinning/freezing") before eventually recovering.

The existing stagehand defenses fire too late:

- **Python reflection filter** (`shtp_receiver.py`): gates bad *output* after the Kalman
  state is already corrupted. Recovery takes several seconds.
- **Per-LH innovation gate** (`--light-outlier-threshold 5.0`): catches
  moderate-to-heavy reflection bursts (rms > 0.001) but cannot distinguish mild
  reflections (rms 0.0001–0.0002) from legitimate noise — both are within 2× of the
  noise floor.
- **Angular rate gate** (`--lc-angular-rate-max`): Kalman-level input velocity gate.
  Once mild reflections corrupt state, the gate defends the *wrong* state and locks out
  valid data.

None of these gates can catch reflections that are geometrically correct but physically
impossible given the tracker's current orientation. Only a geometry-based filter can.

## Goals

1. Prevent reflected lighthouse hits from reaching the Kalman filter by rejecting them at
   the angle-measurement level.
2. Eliminate the "spinning/freeze" failure mode caused by mild reflections that slip
   through the innovation gate.
3. Zero operator configuration — no manual per-environment threshold tuning beyond the
   initial deployment value.
4. No regression on clean tracking (healthy room, no reflective surfaces).

## Target Users

Virtual production operators deploying stagehand in studio environments with LED walls,
monitor arrays, or reflective surfaces. Operators are not sysadmins — any fix requiring
environment-specific tuning is a design failure.

## System Architecture

The back-face filter sits at the earliest possible interception point in the libsurvive
pipeline: `SurviveSensorActivations_check_outlier()` in `survive_sensor_activations.c`,
called before any angle measurement reaches the Kalman tracker or geometric posers.

```
USB hardware
    │  raw photon timestamps
    ▼
driver_vive (USB driver)
    │  LightcapElement
    ▼
survive_process_gen1/2
    │  decoded angle measurements
    ▼
SurviveSensorActivations_check_outlier()   ← BACK-FACE FILTER FIRES HERE
    │  (reject if facingness < 0.1)
    │  (skip if pose confidence < 0.1)
    │  (skip if LH not yet calibrated)
    ▼
survive_kalman_tracker_integrate_saved_light()
    │  per-LH innovation gate (light-outlier-threshold)
    ▼
Kalman measurement update (IEKF)
    ▼
survive_kalman_tracker_report_state()
    │  angular rate gate (lc-angular-rate-max)
    ▼
pose hook → stagehand agent → SHTP → Python reflection filter
```

The filter is **already fully implemented** in the libsurvive fork as
`--filter-normal-facingness`. It is currently disabled in stagehand with
`--filter-normal-facingness -1`. This design activates it.

### Filter Geometry

For each incoming sensor hit `(sensor_id, lh, axis)`:

```
normalInWorld = quatrotatevector(OutPose.Rot, sensor_normals[sensor_id])
sensorInWorld = ApplyPoseToPoint(OutPose, sensor_locations[sensor_id])
towardLH      = normalize(bsd[lh].Pose.Pos - sensorInWorld)
facingness     = dot(normalInWorld, towardLH)

if facingness < threshold:  → reject (physically impossible hit)
```

A sensor with its face pointing away from the lighthouse cannot receive direct lighthouse
light. A reflection arriving from behind cannot be a real hit. Threshold = 0.1 rejects
sensors pointing more than ~84° away from the lighthouse — the last ~6° near the geometric
horizon where TS4231 sensitivity is negligible in practice.

## Key Design Decisions

| Decision | Choice | Rationale |
|---|---|---|
| Enable existing impl vs. reimplement | Enable existing (`--filter-normal-facingness`) | Zero new code; implementation is correct, documented, and has property tests. Reimplementing at `angle_process_func` hook would duplicate logic with no benefit. |
| Threshold | `0.1` (~84° acceptance cone) | Preserves all physically plausible hits; rejects TS4231 edge zone (near-90°) where sensitivity degrades anyway. Geometrically motivated — not empirically tuned. |
| Relationship to existing gates | Layer (keep all gates) | Defense-in-depth. Back-face filter prevents what the innovation gate cannot see. Innovation gate catches non-geometry anomalies (calibration errors, multipath scatter). Both run. Angular rate gate and Python reflection filter remain as downstream backstops. |
| Confidence guard threshold | `0.1` (existing default) | Prevents the filter from firing during cold-start before pose has converged — ensures filter activates only when it has a reliable world-frame normal to compare against. |

## Non-Goals

- Replacing the per-LH innovation gate or angular rate gate — these catch different failure
  modes.
- Eliminating the Python reflection filter — it handles application-layer burst detection.
- Per-environment threshold tuning — threshold 0.1 is fixed for stagehand deployments.
- Handling multipath (multiple-bounce reflections from a front-facing direction) —
  geometry cannot detect these; the innovation gate handles them.

## References

- [docs/reflection-rejection.md](reflection-rejection.md) — full implementation details
  (Change 1)
- `src/survive_sensor_activations.c` — filter implementation
- `src/test_cases/normal_filter_props.c` — 6 property tests for the filter geometry
- `stagehand/scripts/stagehand-health` — deployment change (one line: `-1` → `0.1`)
- [docs/llds/tracking-engine.md](llds/tracking-engine.md) — tracking engine LLD
  (back-face filter section)
- [docs/llds/lighthouse-protocol-intelligence.md](llds/lighthouse-protocol-intelligence.md)
  — Protocol Intelligence LLD (filter hooks into this cluster)
