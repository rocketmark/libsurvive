# libsurvive — High-Level Design

**Created**: 2026-04-11
**Source**: Synthesized from brownfield LLDs

## System Purpose

libsurvive is an open-source C library for 6-DOF (six degrees of freedom)
tracking of HTC Vive, Valve Index, and SteamVR-compatible devices using Valve
Lighthouse base stations. It reads raw photon timestamps from USB-connected
hardware, decodes a proprietary optical protocol, fuses the resulting angle
measurements with IMU data through a nonlinear state estimator, and outputs
continuous pose (position + orientation) and velocity for each tracked device.
It is both a complete standalone tracker and a library that other applications
embed to gain tracking capability without depending on Valve's proprietary
SteamVR runtime.

## Architecture Overview

libsurvive is structured as five independent functional clusters connected by
a central hook system. Data flows inward from hardware through two decoding
stages, then into the tracking engine, and out through the library API.

```
┌──────────────────────────────────────────────────────────────────┐
│                        External World                            │
│   USB/HID hardware · BLE lighthouses · .rec.gz files · UDP      │
└────────────────────────────┬─────────────────────────────────────┘
                             │ raw LightcapElement, IMU, config
                             ▼
┌────────────────────────────────────────────────────────────────┐
│                    Device Driver Stack                         │
│  driver_vive · driver_gatt · driver_playback · driver_udp      │
│  driver_simulator · driver_openvr · driver_global_scene_solver │
└──────────┬─────────────────────────────────────────────────────┘
           │ lightcap hook (LightcapElement)
           │ raw_imu hook
           │ config hook (JSON blob)
           ▼
┌────────────────────────────────────────────────────────────────┐
│               Lighthouse Protocol Intelligence                  │
│  lfsr · ootx_decoder · disambiguator_statebased                │
│  survive_process_gen1/2 · survive_sensor_activations           │
└──────────┬─────────────────────────────────────────────────────┘
           │ angle hook (SurviveSensorActivations)
           │ imu hook (calibrated)
           │ ootx_received hook (BaseStationCal)
           ▼
┌────────────────────────────────────────────────────────────────┐
│                      Tracking Engine                           │
│  posers (SVD · EPnP · MPFit) · survive_kalman_tracker          │
│  survive_kalman_lighthouses · survive_reproject                │
│  cnkalman · cnmatrix · src/generated/ · codegen pipeline       │
└──────────┬─────────────────────────────────────────────────────┘
           │ pose hook
           │ velocity hook
           │ lighthouse_pose hook
           ▼
┌────────────────────────────────────────────────────────────────┐
│                   Library Infrastructure                       │
│  survive.c · survive_api · survive_config · survive_plugins    │
│  survive_recording · survive_default_devices · redist/         │
└──────────┬─────────────────────────────────────────────────────┘
           │ public API (survive.h · survive_api.h)
           ▼
┌────────────────────────────────────────────────────────────────┐
│                    Language Bindings                           │
│  Python (ctypes) · C# / Unity · ROS · OpenVR driver · Web viz  │
└────────────────────────────────────────────────────────────────┘
```

### Intra-cluster dependencies

```
Driver Stack ──────────────────────────────► Protocol Intelligence
Driver Stack ──(driver_global_scene_solver)► Tracking Engine (unusual)
Protocol Intelligence ──────────────────────► Tracking Engine
Tracking Engine ────────────────────────────► Library Infrastructure
All clusters ───────────────────────────────► Library Infrastructure
Language Bindings ───────────────────────────► Library Infrastructure
```

All inter-cluster communication flows through Library Infrastructure's hook
system, except for the `driver_global_scene_solver` anomaly (a Driver Stack
plugin that calls Tracking Engine poser functions directly).

## The Hook System: Connective Tissue

The hook system is the single architectural mechanism that connects all five
clusters. Each hook is a function pointer in `SurviveContext`, installed via
`survive_install_*_fn()`. The 24 hooks form an ordered pipeline:

```
Hardware events  →  lightcap  →  angle/imu/sync  →  pose/velocity
                     (raw)       (decoded)            (solved)
```

Hooks chain: each install returns the previous handler, which the new handler
must call at the end. This enables multiple consumers per event — recording,
the application, and the Tracking Engine can all receive the same pose event
without any of them knowing about each other.

The hook system is also how the two API styles (hook-based and Simple API)
coexist: the Simple API installs its own hooks on top of the existing chain
during `survive_simple_init()`, requiring no special support from the rest of
the system.

## Dual-Path Tracking Architecture

The Tracking Engine runs two parallel paths for computing pose that feed into
each other:

```
Angle measurements
        │
        ├──[direct]──► Kalman lightcap measurement update
        │                  (IEKF, per-sensor adaptive variance)
        │
        └──[poser]───► BaryCentricSVD (geometric seed)
                           │
                           ▼
                       MPFIT (Levenberg-Marquardt refinement)
                           │
                           ▼
                       Kalman observation update
                           │
                           ▼
IMU ────────────────► Kalman IMU update
                           │
                           ▼
                       State output (pose + velocity)
```

The geometric solvers (SVD, MPFIT) provide discrete pose estimates that
anchor the Kalman filter. The Kalman filter provides continuous state and
handles inter-frame motion via IMU integration. Neither path works well alone:
SVD/MPFIT without Kalman produces noisy, discontinuous output; Kalman without
geometric anchoring drifts. The combination produces smooth, accurate tracking.

## Lighthouse Generation Support

Gen1 (Vive, Vive Pro) and Gen2 (Index, SteamVR 2.0) use fundamentally
different protocols. The architecture maintains separate code paths through
the Protocol Intelligence cluster and separate reprojection models in the
Tracking Engine, unified at the Kalman filter level:

```
Gen1 hardware              Gen2 hardware
     │                          │
 driver_vive (raw0)         driver_vive (raw1)
     │                          │
 disambiguator_statebased    lfsr_lh2
 (pulse-length state machine) (LFSR polynomial matching)
     │                          │
 survive_process_gen1        survive_process_gen2
     │                          │
 lighthouse_gen1 reprojection  lighthouse_gen2 reprojection
      └──────────────┬──────────┘
                     │
             survive_kalman_tracker
             (generation-agnostic)
```

The Kalman filter and posers are generation-agnostic; they work on calibrated
angles regardless of how those angles were decoded.

## Cross-Cutting Concerns

### Configuration

Every tunable parameter — from Kalman process noise weights to USB timeout
thresholds — uses the same `STATIC_CONFIG_ITEM` registration mechanism. Config
items are declared at the point of use, not in a central registry. At startup,
Library Infrastructure collects all registrations, parses command-line arguments
and the JSON config file, and makes values available via `survive_configf/b/i/s`.

The large configuration surface (100+ keys in the Kalman tracker alone) is both
a strength (fine-grained tuning for new hardware) and a liability (no documented
safe parameter regions, combinatorial instability risk).

### Precision

All floating-point math uses the `FLT` typedef, which resolves to `float` or
`double` at compile time via `USE_SINGLE_PRECISION`. The choice propagates
through cnmatrix (BLAS call selection), cnkalman, all generated code, and the
C ABI exported to language bindings.

**Binding caveat:** Python and C# bindings hardcode `double`. Compiling
libsurvive with `USE_SINGLE_PRECISION=ON` silently breaks both binding layers.

### Symbolic Code Generation

A Python/SymEngine pipeline generates the analytically-differentiated C
functions used throughout the Tracking Engine:

```
tools/generate_math_functions/*.py  ──► src/generated/*.gen.h
libs/cnkalman/cnkalman/codegen.py   ──► (cnkalman internal)
```

Generated files are committed to the repository. This means:
- No SymEngine dependency at build time (only at math-change time)
- No guarantee that committed files match current Python source
- Changes to the math require a manual regeneration step

This is the single biggest maintenance hazard in the codebase: silent
divergence between the Python model and the committed C is undetectable
without running the codegen and diffing the output.

### Recording and Replay

The recording system (`survive_recording.c`) intercepts hooks and writes all
events to `.rec.gz`. The playback driver (`driver_playback.c`) reads these
files and re-injects events, making the Driver Stack appear identical to a
live hardware session. This enables:
- Offline development without hardware
- Regression testing (`src/test_cases/test_replays.c`)
- Bug reproduction from field captures

The recording/replay loop is the primary CI mechanism for algorithmic correctness.

### Threading Model

libsurvive uses a cooperative threading model:

| Component | Threading |
|-----------|-----------|
| Driver (hardware) | Background thread per driver; lock context on hook call |
| Driver (playback) | Background thread; paced by `os_generic` sleep |
| Button servicer | Dedicated thread; semaphore-signaled |
| MPFIT async mode | Worker thread; double-buffered job queue |
| Simple API | Background thread runs main poll loop |
| Kalman filter | Called on whichever thread delivers measurements |

All hook callbacks are guarded by the context recursive mutex. Drivers must
acquire the lock before calling hooks; the Simple API holds it across all
callbacks in a poll cycle.

### Error Handling and Logging

There is no exception mechanism or structured error propagation. Errors are
handled by:
- Return codes (0 = success, non-zero = failure) at the driver and poser levels
- `SV_WARN` / `SV_ERROR` macros that fire the `printf` hook (visible to callers)
- Silent degradation: outlier measurements are dropped; failed solves are skipped
- Kalman filter: divergence is silent (IEKF terminates on max iterations without warning)

The absence of structured error surfacing makes it difficult for callers to
distinguish "no tracking data yet" from "hardware failure" from "algorithm
divergence."

## Key Architectural Decisions

These decisions shaped the system at an architectural level and would require
significant rework to change:

| Decision | What | Why |
|----------|------|-----|
| Plugin system for drivers and posers | `.so`/`.dll` dynamic loading | New hardware and algorithms without recompilation; optional dependencies (libusb, gattlib, OpenVR) don't affect core |
| Hook system for inter-cluster communication | Function pointer chain, 24 hooks | Single mechanism for both the low-level and Simple API; enables recording without modifying any processing code |
| Error-state EKF for rotation | Quaternion state, axis-angle updates in Kalman | Quaternion normalization constraint cannot be maintained in standard EKF; error-state formulation keeps updates in Euclidean space |
| Analytical Jacobians via symbolic codegen | Python SymEngine → CSE-optimized C | Manual derivation is error-prone; numerical Jacobians too slow; codegen gives accuracy and speed |
| Geometric seed + nonlinear refinement | SVD/EPnP seed → MPFIT refinement | SVD is fast but ignores calibration; MPFIT is accurate but needs a good initial guess; each layer does what it is good at |
| Separate Gen1/Gen2 signal paths | Parallel decode paths, unified at Kalman | Protocols are fundamentally different; forced unification would add complexity without benefit |
| Text-based recording format | Human-readable timestamped lines | Inspectable and editable without tooling; round-trip via sscanf format strings |
| `FLT` type for float/double selection | Global compile-time switch | SIMD acceleration on float; double available for calibration and validation tools; single switch controls everything |

## Non-Goals

libsurvive explicitly does not:

- **Provide a VR runtime** — It tracks devices; it does not manage compositor,
  display, audio, or application lifecycle. OpenVR/SteamVR provide those.
- **Support non-Lighthouse tracking** — Camera-based (inside-out), optical
  marker, or magnetic tracking are out of scope.
- **Handle more than 2 Gen1 lighthouses** — The Gen1 protocol's time-division
  multiplexing supports at most 2 basestations. Gen2 supports more.
- **Provide a real-time guarantee** — No bounded latency or deadline enforcement.
  MPFIT solve time is unbounded.
- **Abstract hardware differences from the application** — Gen1 and Gen2
  require different driver configurations. The library does not auto-negotiate.
- **Synchronize to wall clock** — libsurvive timecodes are relative to USB
  clock start. ROS and other time-aware consumers must handle offset alignment
  themselves.

## References

**LLD documents:**
- [docs/llds/lighthouse-protocol-intelligence.md](llds/lighthouse-protocol-intelligence.md)
- [docs/llds/device-driver-stack.md](llds/device-driver-stack.md)
- [docs/llds/tracking-engine.md](llds/tracking-engine.md)
- [docs/llds/library-infrastructure.md](llds/library-infrastructure.md)
- [docs/llds/language-bindings.md](llds/language-bindings.md)

**Existing documentation:**
- [docs/architecture.md](architecture.md) — Original architecture notes (predates this HLD)
- [docs/writing_a_poser.md](writing_a_poser.md) — Poser plugin development guide
- [README.md](../README.md) — User-facing quick start
