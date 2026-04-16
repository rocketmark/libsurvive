# Lighthouse Protocol Intelligence

**Created**: 2026-04-11
**Status**: Mapped from existing code
**Source**: Brownfield bootstrap via /map-codebase

## Context and Current State

This cluster is the part of libsurvive that understands the Valve Lighthouse
optical/RF protocol. Its job is to take raw photon timestamps — integers
representing when a sensor saw a flash of light — and produce two things:
calibrated sweep angles (the actual measurement data used for pose solving)
and basestation calibration packets (the factory-calibrated optical parameters
for each lighthouse).

The cluster sits between the Driver Stack (which produces raw `LightcapElement`
structs from hardware) and the Tracking Engine (which consumes angles and OOTX
calibration data to compute poses).

Lighthouse hardware exists in two generations with fundamentally different
protocols:

- **Gen1 (Vive, Vive Pro)**: Two rotating laser fans per basestation, each
  sweeping one axis. Up to two basestations alternate in a time-division
  multiplexed schedule. Sync pulses encode identity information as
  length-modulated pulses.
- **Gen2 (SteamVR 2.0, Index)**: Single rotor with two tilted planes. More
  basestations supported simultaneously. LFSR-encoded channel IDs replace
  pulse-length encoding.

Both generations require the cluster to solve the same fundamental problem:
given a stream of anonymous light-pulse timestamps, determine which lighthouse
produced each pulse, which axis it represents, and what angle it implies for
the receiving sensor.

## Gen1 Protocol: Pulse Length Disambiguation

### Physical Model

In Gen1, each basestation emits periodic sync pulses and continuous laser
sweeps. A sensor sees:
1. A sync pulse (typically 3000–6500 μs wide) that encodes identity
2. After a known delay, a sweep pulse (narrow, <200 μs) representing the laser
   crossing the sensor

The angular position of the sensor is derived from the time between the sync
pulse and the sweep pulse, normalized to the 400,000-tick rotation period
(1 revolution = π radians of useful sweep range = 400,000 ticks).

### State Machine Disambiguation (`disambiguator_statebased.c`)

The state-based disambiguator is the protocol master for Gen1. It maintains a
12-state state machine per tracked object:

```
LS_UNKNOWN
LS_WaitLHB_AX / LS_WaitLHB_AY   (waiting for lighthouse B sweeps)
LS_SweepAX / LS_SweepAY          (confirmed lighthouse A sweeps)
LS_SweepBX / LS_SweepBY          (confirmed lighthouse B sweeps)
... (additional transition states)
```

The state machine exploits the fixed 40ms cycle (1,600,000 ticks) of the Gen1
schedule, which allocates each lighthouse specific time slots:

| Time in cycle | Event                        |
|---------------|------------------------------|
| 0             | LH-A X-axis sync pulse       |
| 20,000 ticks  | LH-A X-axis sweep window     |
| ~420,000      | LH-A Y-axis sync + sweep     |
| ~800,000      | LH-B X-axis sync + sweep     |
| ~1,200,000    | LH-B Y-axis sync + sweep     |

The acode (3-bit value encoded in sync pulse length) carries:
- **bit 0**: skip flag (this sweep is skipped)
- **bit 1**: data bit (OOTX payload)
- **bit 2**: axis (X=0, Y=1)

Pulse length encoding: `acode = (length_in_us - 3000) / 500`, giving values
0–7 corresponding to 3000–6500 μs pulses.

A confidence mechanism tracks how well recent pulses match expected timing.
If confidence falls below threshold, the disambiguator reacquires from
LS_UNKNOWN, providing graceful recovery from occlusion.

### Sensor Activations and Outlier Rejection (`survive_sensor_activations.c`)

Raw angle readings from the disambiguator pass through a statistical filter
before reaching the Tracking Engine. For each (sensor, lighthouse, axis) triple,
the filter maintains:
- Running mean and variance of angle measurements
- Timecode of last valid reading
- Motion detection via gyro/accel/light variance thresholds

The filter is the earliest interception point in the pipeline and runs two
rejection stages in order before any measurement reaches the Kalman tracker.

#### Stage 1 — Back-Face Normal Filter

A geometry-based pre-filter that rejects sensor hits that are physically
impossible given the tracker's current orientation. Lighthouse sweep light
cannot illuminate a sensor whose face points away from the lighthouse; such
hits are almost certainly specular reflections from hard surfaces (LED walls,
glass, monitor bezels).

For each incoming hit `(sensor_id, lh, axis)`, the filter computes:

```c
normalInWorld = quatrotatevector(OutPose.Rot, sensor_normals[sensor_id])
sensorInWorld = ApplyPoseToPoint(OutPose, sensor_locations[sensor_id])
towardLH      = normalize(bsd[lh].Pose.Pos - sensorInWorld)
facingness     = dot(normalInWorld, towardLH)

if facingness < filterNormalFacingness: reject
```

The sensor normal is stored in the object's local frame (`sensor_normals`
field); `quatrotatevector` maps it to world frame using the current tracker
pose. `facingness = 1.0` means the sensor faces directly toward the lighthouse;
`facingness = 0.0` is the geometric horizon; `facingness = -1.0` is directly
away.

**Guards** (any failing guard skips the filter entirely, passing the hit):
- `filterNormalFacingness < -0.5` — disabled (threshold below -0.5 disables)
- `so->sensor_normals == NULL` — device has no normals data (not all trackers)
- `so->poseConfidence < filterNormalMinConfidence` — pose not yet reliable
  enough to trust the world-frame normal computation
- `!ctx->bsd[lh].PositionSet` — lighthouse not yet calibrated

**Config items:**

| Flag | Default | Description |
|---|---|---|
| `--filter-normal-facingness` | `0.0` | Min dot product to accept. `< -0.5` disables. Stagehand deploys `0.1` (~84° cone). |
| `--filter-normal-min-confidence` | `0.1` | Min pose confidence before filter activates. |

**Property tests:** `src/test_cases/normal_filter_props.c` (6 tests) covers
`FacingnessInRange`, `FacingnessFlipsWithDirection`, `FacingnessKnownAngle`,
`FacingnessThresholdMonotonic`, `DirectlyFacingAlwaysAccepted`,
`BackFacingAlwaysRejected`.

**Reference:** Ported from the simulator path at `src/driver_simulator.c:152–163`.
The same filter exists there for virtual hardware; this change brings it to the
real hardware path. See `docs/reflection-rejection.md` (Change 1) for full
implementation details and `docs/back-face-filter-hld.md` for design rationale.

#### Stage 2 — Chauvenet Statistical Outlier Rejection

Outliers are rejected using the Chauvenet criterion: a new measurement is
discarded if it falls more than N standard deviations from the running mean,
where the threshold is configured by `angle-reject-outliers`. This prevents
single corrupt readings from destabilizing the pose solver.

A validity check requires that at least 2 measurements (both X and Y axis) be
present per lighthouse before that lighthouse's data is passed to the Tracking
Engine. This ensures the solver always has enough geometric information.

## Gen2 Protocol: LFSR Channel Decoding

### Physical Model

Gen2 uses a fundamentally different approach. Instead of pulse-length encoding,
each basestation transmits a specific Linear Feedback Shift Register (LFSR)
bit sequence. Sensors capture timing of the two rotating laser planes and the
bit pattern encodes which basestation is which.

### LFSR Implementation (`lfsr.c`, `lfsr_lh2.c`)

The LFSR subsystem provides:
- **`lfsr_find()`** — Given a bit sequence, find the LFSR state that produced it
- **`lfsr_lookup_ctor()`** — Build a hash table for O(1) state lookup
- **`survive_decipher_channel()`** — Match observed timing against 32 known
  polynomial sequences to identify the lighthouse channel (0–31)

The Gen2 decoder maintains 32 competing LFSR sequences (one per possible
lighthouse channel) and selects the one that best matches observed timing.
A ±2-sample tolerance handles minor timing errors.

**Note:** The `lfsr_lh2.c` implementation contains extensive debug `fprintf`
calls, suggesting this path is still under active development compared to the
mature Gen1 path.

## OOTX: Factory Calibration Packet Decoding

### What OOTX Is

Each Gen1 lighthouse continuously transmits its factory calibration data by
modulating the data bit of sync pulses. This "Out-Of-The-Box Transmission"
(OOTX) is a slow serial channel (~30 Hz bit rate) carrying:
- Phase offset per axis
- Tilt per axis
- Curve (quadratic distortion) per axis
- Gibberish phase and magnitude (sine modulation) per axis

For Gen2, the equivalent parameters include OGEE polynomial series coefficients.

### Decoding (`ootx_decoder.c`)

The OOTX decoder implements a Manchester-style framing protocol:
1. **Preamble detection**: Watch for 32 consecutive zeros followed by a one
   (encoded as `0x00000001` in an 18-bit rolling window)
2. **Payload accumulation**: Read length (2 bytes) then payload
3. **Sync bit validation**: Every 17th bit is a sync bit and must be 1
4. **CRC32 verification**: Validate payload integrity

Calibration parameters are stored as half-floats (float16) in the packet.
The decoder converts these to full `FLT` values on extraction.

The decoder tracks statistics: `bits_seen`, `bad_crcs`, `bad_sync_bits`.
An `ignore_sync_bit_error` config flag can disable sync bit checking,
useful for marginal hardware.

### Gen1 vs Gen2 Calibration Differences

| Parameter    | Gen1 (v6)              | Gen2 (v15)              |
|--------------|------------------------|-------------------------|
| Phase        | Phase offset per axis  | Same                    |
| Tilt         | tan(tilt) stored       | Direct tilt stored      |
| Curve        | Quadratic correction   | Polynomial series (6 coefficients) |
| Gibberish    | phase+magnitude        | OGEE: ogeephase, ogeemag |

## Lighthouse Reprojection Model (`src/generated/lighthouse_gen1.gen.h`)

This generated file defines the mathematical inverse of the hardware: given a
3D sensor position and lighthouse pose, what angle would that sensor report?

**Gen1 model:**
```
angle_x = atan2(X, -Z) - phase_x - curve(Y, Z) - gibberish(angle_x)
angle_y = atan2(Y, -Z) - phase_y - curve(X, Z) - gibberish(angle_y)
```
Where `curve(other_axis, Z) = atan2(other_axis, -Z)² × curve_coefficient`.

The file also contains full Jacobian matrices (symbolic derivatives with
respect to each parameter), used by the optimizer to compute gradients.

These files are generated by `src/generated/lighthouse_gen1.py` using
SymEngine/SymPy symbolic differentiation — the Python code defines the math
symbolically and the code generator emits optimized C with common-subexpression
elimination.

**Gen2 model** adds a polynomial series correction (`calc_cal_series`) using
6 hardcoded coefficients: `[-8.0108e-06, 0.0028679, 5.37e-06, 7.61e-03, 0, 0]`.
These appear to be empirically fit to hardware behavior rather than derived from
first principles. The Gen2 tilt angles are offset by ±30° per axis due to the
tilted-plane sweep geometry.

## Protocol Autodetection (`survive_disambiguator.c`)

The top-level disambiguator detects whether attached hardware is Gen1 or Gen2
by observing pulse characteristics:
- **Gen1**: 10+ consecutive pulses in the 3000–6500 μs range at 60/120 Hz
- **Gen2**: 500+ light packets arrive without meeting the Gen1 signature

Once detected, `survive_notify_gen1()` or `survive_notify_gen2()` is called
to install the appropriate processing hooks. This detection is not reversible
during a session — the system commits to one generation per object.

## Data Flow Through the Cluster

```
Driver Stack
    │
    │  LightcapElement {sensor_id, timecode, length}
    ▼
survive_disambiguator.c
    │  (autodetect Gen1 vs Gen2)
    │
    ├─[Gen1]─► disambiguator_statebased.c
    │              │  state machine → (acode, axis, sweep timecode)
    │              │
    │           survive_process_gen1.c
    │              │  angle = (timecode_in_sweep / TICKS_PER_ROTATION) × π
    │              │  OOTX bits → ootx_decoder → BaseStationCal
    │              ▼
    │
    └─[Gen2]─► survive_process_gen2.c
                   │  lfsr_lh2 → channel_id
                   │  angle computation (plane-based)
                   ▼

survive_sensor_activations.c
    │  Stage 1: back-face normal filter (geometry-based)
    │  Stage 2: Chauvenet outlier rejection
    │  validity gating (require both axes per LH)
    ▼
Tracking Engine (angles + BaseStationCal)
```

## Observed Design Decisions

| Decision | What was chosen | Evidence | Likely rationale |
|---|---|---|---|
| Separate Gen1/Gen2 code paths | Entirely separate files for processing, LFSR, reprojection | `survive_process_gen1.c` vs `survive_process_gen2.c`; separate `lfsr_lh2` | Protocols are fundamentally different; no benefit to forced unification |
| State machine over frequency analysis for Gen1 | 12-state cycle-locked state machine | `disambiguator_statebased.c` state table | Cycle-locked approach handles multi-LH scheduling and confidence tracking; frequency analysis would miss axis identity |
| OOTX decoded from sync pulse data bits | Inline with normal sync processing | `survive_process_gen1.c:ootx_pump_bit()` called on data bits | OOTX is transmitted via the sync pulse data channel; no separate wire |
| Chauvenet criterion for outlier rejection | Statistical outlier filter with configurable threshold | `survive_sensor_activations.c:SurviveSensorActivations_add()` | Robust to sensor noise without tuning per-device; adapts to local variance |
| Generated code for reprojection math | Python → SymEngine → C with CSE | `src/generated/lighthouse_gen1.gen.h`, `lighthouse_gen1.py` | Analytical Jacobians are required for the optimizer; manual derivation is error-prone and unmaintainable |
| Half-float storage in OOTX packets | float16 decoded to FLT on extraction | `ootx_decoder.c` half-float unpacking | Bandwidth constraint: OOTX is a very slow channel; 16-bit precision is sufficient for calibration parameters |

## Technical Debt & Inconsistencies

1. **`lfsr_lh2.c` has extensive debug fprintf output** — Suggests Gen2 LFSR
   decoding is still in development. Should be removed or gated behind a
   debug flag before shipping. (`src/lfsr_lh2.c`, throughout)

2. **`survive_process_gen2.c` is nearly empty** — Gen2 light processing is
   marked as deferred to the Gen2 disambiguator, but the actual integration
   path is unclear from the code. The Gen2 path appears incomplete compared
   to Gen1. (`src/survive_process_gen2.c`)

3. **Gen2 polynomial series coefficients are hardcoded magic numbers** —
   `lighthouse_gen2.py` line 52: `[-8.0108022e-06, 0.0028679863, ...]`.
   Origin is undocumented. Likely fit from hardware measurements; should be
   commented with source. (`src/generated/lighthouse_gen2.py`)

4. **`ignore_sync_bit_error` config flag is undocumented** — A workaround for
   marginal hardware that produces sync bit errors. No documentation on when
   to enable it or what failure modes it masks.

5. **Gen1/Gen2 detection is one-way** — Once the system commits to Gen1 or
   Gen2 for a device, there is no path back. This means a mis-detection
   (possible in the 500-packet window for Gen2) results in a bad session.

## Behavioral Quirks

1. **Sensor activations require 2 axes before passing data downstream** —
   A lighthouse with only one axis of data is silently ignored. If a lighthouse
   is partially occluded (e.g., sensor sees X sweeps but not Y), that lighthouse
   contributes nothing to the pose solve. This is intentional for correctness
   but can cause confusing "lighthouse not seen" behavior during marginal
   tracking conditions.

2. **The 1.6M tick cycle is absolute, not object-relative** — The Gen1 state
   machine locks onto an absolute system-wide timing reference. If two objects
   are tracking simultaneously, they share the same timing reference. Mis-locked
   objects produce cross-talk symptoms.

3. **OOTX decoding has "guess bits"** — The decoder tracks `guess_bits`, used
   when sync bit errors force a guess at the correct value. This silently
   corrupts calibration data in degraded RF conditions. No application-level
   error is surfaced.

4. **Gen2 LFSR allows ±2-sample timing tolerance** — Provides some robustness
   to jitter but means ambiguous readings can match multiple channels. The
   decoder picks the best match without exposing the confidence level.

## Open Questions

1. What is the exact source of the Gen2 polynomial series coefficients in
   `lighthouse_gen2.py`? Are they universal constants or device-family-specific?

2. Is the Gen2 processing path (`survive_process_gen2.c`) intentionally minimal,
   or is it a work in progress? The code is nearly empty compared to the Gen1
   equivalent.

3. The `mod_offset` tracking in `disambiguator_statebased.c` handles dual-LH
   timing. How does this extend to Gen2 which supports more than 2 lighthouses?

## References

**Files in this cluster:**
- [src/lfsr.h](../../src/lfsr.h), [src/lfsr.c](../../src/lfsr.c)
- [src/lfsr_lh2.h](../../src/lfsr_lh2.h), [src/lfsr_lh2.c](../../src/lfsr_lh2.c)
- [src/ootx_decoder.h](../../src/ootx_decoder.h), [src/ootx_decoder.c](../../src/ootx_decoder.c)
- [src/disambiguator_statebased.c](../../src/disambiguator_statebased.c)
- [src/survive_disambiguator.c](../../src/survive_disambiguator.c)
- [src/survive_process.c](../../src/survive_process.c)
- [src/survive_process_gen1.c](../../src/survive_process_gen1.c)
- [src/survive_process_gen2.c](../../src/survive_process_gen2.c)
- [src/survive_sensor_activations.c](../../src/survive_sensor_activations.c)
- [src/generated/lighthouse_gen1.gen.h](../../src/generated/lighthouse_gen1.gen.h)
- [src/generated/lighthouse_gen1.py](../../src/generated/lighthouse_gen1.py)
- [src/generated/lighthouse_gen2.py](../../src/generated/lighthouse_gen2.py)
- [src/generated/lighthouse_model.gen.h](../../src/generated/lighthouse_model.gen.h)
- [src/generated/lighthouse_model.py](../../src/generated/lighthouse_model.py)

**Dependencies on other clusters:**
- → **Library Infrastructure**: uses `SurviveContext`, hook system, config, `SV_MALLOC`
- → **Device Driver Stack**: consumes `LightcapElement` events via `light_pulse` hook
- ← **Tracking Engine**: produces `SurviveSensorActivations` (angles) and `BaseStationCal`

**External dependencies:**
- `redist/crc32.{c,h}` — CRC32 for OOTX packet validation
- SymEngine/SymPy (Python, build-time only) — symbolic differentiation for codegen
