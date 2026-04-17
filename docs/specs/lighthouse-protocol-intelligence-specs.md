# Lighthouse Protocol Intelligence — EARS Specs

Prefix: **LPI**

## Protocol Autodetection

- [x] **LPI-PROC-001**: When a device begins receiving light pulses, the system shall classify the hardware generation as Gen1 or Gen2 before passing angle data to the Tracking Engine.
- [x] **LPI-PROC-002**: When 10 or more consecutive light pulses arrive with lengths in the 3000–6500 μs range at 60/120 Hz, the system shall classify the device as Gen1.
- [x] **LPI-PROC-003**: When more than 500 light packets arrive without meeting the Gen1 pulse-length signature, the system shall classify the device as Gen2.
- [ ] **LPI-PROC-004**: If generation detection produces a conflicting classification mid-session, the system shall surface a warning and maintain the original classification.

## Gen1 Disambiguation

- [x] **LPI-PROC-010**: The system shall decode the 3-bit acode from each Gen1 sync pulse length using the formula `acode = (length_us - 3000) / 500`.
- [x] **LPI-PROC-011**: The system shall identify the sweep axis (X or Y) from acode bit 2 of each Gen1 sync pulse.
- [x] **LPI-PROC-012**: The system shall identify the data bit (OOTX payload) from acode bit 1 of each Gen1 sync pulse.
- [x] **LPI-PROC-013**: The system shall identify the skip flag from acode bit 0 of each Gen1 sync pulse and discard sweep data for skipped slots.
- [x] **LPI-PROC-014**: The system shall lock Gen1 sync timing to the 1,600,000-tick (40ms) absolute cycle shared across all tracked objects.
- [x] **LPI-PROC-015**: While Gen1 tracking is active, the system shall maintain a per-object confidence score and transition to LS_UNKNOWN state when confidence falls below threshold.
- [x] **LPI-PROC-016**: When the Gen1 state machine enters LS_UNKNOWN, the system shall reacquire sync state from the pulse history ring buffer without requiring a full session restart.

## OOTX Decoding

- [x] **LPI-PROC-020**: The system shall detect the OOTX preamble by watching for 32 consecutive zero bits followed by a one bit in the sync pulse data-bit stream.
- [x] **LPI-PROC-021**: The system shall validate every 17th bit in the OOTX stream as a sync bit equal to 1.
- [x] **LPI-PROC-022**: The system shall validate each OOTX packet with CRC32 and discard packets that fail validation.
- [x] **LPI-PROC-023**: The system shall decode Gen1 OOTX packets (v6 format) and extract calibration parameters: phase, tilt (stored as tan), curve, gibberish phase, and gibberish magnitude per axis.
- [x] **LPI-PROC-024**: The system shall decode Gen2 OOTX packets (v15 format) and extract calibration parameters: phase, tilt (stored directly), polynomial series coefficients, and OGEE phase/magnitude.
- [x] **LPI-PROC-025**: The system shall decode OOTX calibration half-floats (float16) to full FLT precision on extraction.
- [ ] **LPI-PROC-026**: If OOTX sync bit errors exceed a configurable threshold, the system shall emit a warning indicating degraded calibration data quality.
- [x] **LPI-PROC-027**: Where the `ignore-sync-bit-error` config flag is set, the system shall continue OOTX decoding despite sync bit errors.

## Gen2 LFSR Channel Decoding

- [x] **LPI-PROC-030**: The system shall identify the Gen2 lighthouse channel (0–31) by matching observed timing against 32 known LFSR polynomial sequences.
- [x] **LPI-PROC-031**: When decoding Gen2 channel identity, the system shall tolerate ±2-sample timing deviation from the expected LFSR sequence.
- [ ] **LPI-PROC-032**: The system shall remove debug output (fprintf calls) from the Gen2 LFSR decoder before production use.

## Angle Computation

- [x] **LPI-PROC-040**: The system shall compute Gen1 sweep angles as `angle = (timecode_in_sweep / 400000) × π` where 400,000 ticks equals one half-rotation.
- [x] **LPI-PROC-041**: The system shall reject Gen1 sweep measurements where timeinsweep exceeds 400,000 ticks.
- [x] **LPI-PROC-042**: The system shall distinguish sync pulses (sensor_id -1/-2/-3) from sweep pulses (sensor_id ≥ 0) and route each to the appropriate processing path.

## Sensor Activation Filtering

- [x] **LPI-PROC-050**: The system shall maintain a running mean and variance of angle measurements per (sensor, lighthouse, axis) triple.
- [x] **LPI-PROC-051**: When a new angle measurement deviates from the running mean by more than the configured Chauvenet threshold, the system shall reject the measurement as an outlier.
- [x] **LPI-PROC-052**: The system shall require valid measurements on both X and Y axes for a given lighthouse before passing that lighthouse's data to the Tracking Engine.
- [x] **LPI-PROC-053**: While the tracked object is in motion (detected via IMU or angle variance), the system shall reset per-sensor running statistics to prevent stale mean values from rejecting valid measurements.

## Back-Face Normal Filter

- [x] **LPI-PROC-060**: When `filter-normal-facingness` ≥ -0.5, the system shall evaluate each incoming sensor hit against the back-face criterion before any other filtering: compute `facingness = dot(quatrotatevector(OutPose.Rot, sensor_normals[sensor_id]), normalize(bsd[lh].Pose.Pos - sensorInWorld))` and reject the hit if `facingness < filterNormalFacingness`.
- [x] **LPI-PROC-061**: When `filter-normal-facingness` < -0.5, the system shall skip the back-face filter entirely and pass the hit to subsequent filtering stages unchanged.
- [x] **LPI-PROC-062**: If the tracked object's `sensor_normals` or `sensor_locations` field is NULL, the system shall skip the back-face filter for that object (not all device types carry sensor geometry data).
- [x] **LPI-PROC-063**: While `poseConfidence` is below `filter-normal-min-confidence`, the system shall skip the back-face filter (pose not yet reliable enough to compute a trustworthy world-frame normal).
- [x] **LPI-PROC-064**: If the lighthouse referenced by the incoming hit does not yet have `PositionSet`, the system shall skip the back-face filter for that hit (lighthouse position needed to compute direction-to-LH).
- [x] **LPI-PROC-065**: The back-face filter shall run before the Chauvenet outlier filter so that geometrically impossible hits are rejected upstream, before they influence the per-sensor running statistics.
- [x] **LPI-PROC-066**: When `filter-normal-facingness` ≥ -0.5 and a hit is rejected by the back-face filter, the system shall emit a `SV_VERBOSE(105)` log line including the measured facingness, the threshold, and the (lh, sensor_id, axis) triple.

## Reprojection Model

- [x] **LPI-DATA-060**: The system shall implement the Gen1 reprojection model: `angle = atan2(axis, -Z) - phase - tilt×other_axis - curve×other_axis² - gibmag×sin(angle + gibpha)`.
- [x] **LPI-DATA-061**: The system shall implement the Gen2 reprojection model including polynomial series correction using the six empirically-derived calibration coefficients and OGEE sine modulation.
- [x] **LPI-DATA-062**: The system shall provide full Jacobian matrices for both Gen1 and Gen2 reprojection models with respect to object pose, lighthouse pose, and calibration parameters.
- [x] **LPI-DATA-063**: The system shall generate reprojection functions and Jacobians from symbolic Python definitions using common-subexpression elimination to produce optimized static inline C.
