# Device Driver Stack

**Created**: 2026-04-11
**Status**: Mapped from existing code
**Source**: Brownfield bootstrap via /map-codebase

## Context and Current State

The Device Driver Stack is the part of libsurvive that produces raw sensor
events — light pulses, IMU readings, button presses, and device configuration
blobs. Everything above this cluster (protocol decoding, pose solving) is
hardware-agnostic; this cluster is where hardware reality is translated into
libsurvive's internal event model.

All drivers share a common contract: they register with the driver manager at
startup, receive a `SurviveContext*`, and invoke hook callbacks to deliver
events. From the rest of the system's perspective, a live Vive HMD, a recorded
session replay, and a software simulator are indistinguishable.

Drivers are loaded as dynamic plugins (`.so`/`.dll`) on Unix/Windows, or linked
statically. The plugin system retries loading until all dependencies resolve,
handling load-order issues automatically.

## Driver Interface Contract

Every driver follows this pattern:

```c
// Registration (called at plugin load time via constructor attribute)
int DriverRegFoo(SurviveContext *ctx) {
    // allocate driver state
    // install hooks: ctx->lightcap_fn, ctx->imu_fn, etc.
    // register poll/close callbacks
    return 0; // success
}
REGISTER_LINKTIME(DriverRegFoo);
```

Drivers interact with the rest of the system exclusively through hooks:

| Hook | Data produced |
|------|--------------|
| `lightcap` | `LightcapElement` — raw sensor_id, timecode, pulse length |
| `raw_imu` | Raw accelerometer, gyroscope, magnetometer readings |
| `imu` | Calibrated IMU data (some drivers skip raw and emit calibrated directly) |
| `config` | JSON device configuration blob (sensor positions, normals, calibration) |
| `button` | Button/axis events from controllers |
| `pose` | Pre-solved poses (OpenVR driver, UDP driver) |
| `external_pose` | External reference pose for calibration |

The `config` hook is particularly important — it triggers device creation
(`survive_load_htc_config_format`) which allocates the `SurviveObject` and
populates sensor geometry before any light data is processed.

## Vive Driver (`driver_vive.c`)

The Vive driver is the primary hardware driver, supporting all HTC Vive and
Valve Index family devices over USB.

### Supported Devices

| Device | VID | PID | Notes |
|--------|-----|-----|-------|
| HTC Vive HMD | 0x0bb4 | 0x2c87 | USB, wired |
| Vive Controller (Watchman) | 0x28de | 0x2101 | RF dongle |
| Vive Tracker (2018) | 0x28de | 0x2300 | RF dongle |
| Knuckles / Index Controller | 0x28de | 0x2232 | RF dongle |
| Vive Lighthouse (LHB) | 0x28de | various | Not tracked, managed via GATT |

### USB Backend Abstraction

The driver supports two USB backends, selected at compile time:

- **libusb** (default on Linux/macOS) — Async control and interrupt transfers,
  hotplug support, kernel driver detachment. Feature reports sent as raw
  control transfers with 8-byte setup header.
- **HIDAPI** (default on Windows, fallback) — Synchronous HID feature reports
  via OS HID stack. Non-blocking mode uses per-thread polling.

The abstraction layer (`driver_vive.hidapi.h`, `driver_vive.libusb.h`) presents
a unified transfer submission API so `driver_vive.c` is backend-agnostic.

### Device Initialization State Machine

Device initialization follows a strict sequence, implemented as an async
state machine in `driver_vive.config.h`:

```
MAGICS
  │  Send power-on magic packet(s)
  │  (RF-only devices also need protocol-change and raw-mode packets)
  ▼
CONFIG
  │  Read device config in 256-byte chunks via feature report 0x10/0x11
  │  Gzip-compressed JSON; actual length may exceed advertised by 2 bytes
  ▼
VERSION
  │  Read firmware version (report 0x05 or 0x13)
  ▼
IMU_SCALES
  │  Query gyro/accel scale modes (report 0x07 or 0x04)
  ▼
ATTACH_INTERFACES
     Install IMU, lightcap, and button interrupt transfer callbacks
     Fire config hook → triggers SurviveObject creation
     Fire imu_scale_modes hook
```

Stall detection and retry logic (with exponential backoff) handle USB
enumeration races. The config blob is gzip-decompressed in-place using
`survive_simple_inflate()` from `redist/puff.c`.

### Lightcap Modes

The driver supports three raw lightcap modes, selected during initialization:

| Mode | Protocol | Report ID |
|------|----------|-----------|
| raw0 | Lighthouse Gen1 | 0x21 |
| raw1 | Lighthouse Gen2 | 0x25 / 0x27 |
| raw2 | Reserved | 0x28 |

Mode selection is sent as a magic packet sequence. The driver does not
interpret lightcap data — it passes raw `LightcapElement` structs to the
`lightcap` hook for the Protocol Intelligence cluster to decode.

### Watchman (RF Controllers)

Wireless controllers connect via USB RF dongle. The driver handles watchman
packets (report 0x23/0x24) which multiplex IMU and lightcap data from multiple
wireless devices over a single USB endpoint.

`parse_watchman_lightcap()` demultiplexes the packet stream and dispatches
events per device. Battery status and button events are also parsed here.

### HID Report IDs Reference

| Report ID | Purpose |
|-----------|---------|
| 0x04, 0x07 | Gyro/accel scale mode set |
| 0x05, 0x13 | Firmware version read |
| 0x10, 0x11 | Config download (feature report chunks) |
| 0x20 | IMU interrupt stream |
| 0x21 | Lightcap Gen1 interrupt stream |
| 0x23, 0x24 | RF Watchman (wireless IMU + lightcap) |
| 0x25, 0x27, 0x28 | Lightcap Gen2 variants |

## GATT Driver (`driver_gatt.c`)

Manages Gen2 basestation power and mode over Bluetooth LE. This driver does
not produce tracking data — its sole purpose is lifecycle control of lighthouses.

Using `gattlib` for BLE, it:
1. Scans for devices with name prefix `"LHB"`
2. Reads/writes two GATT characteristics per lighthouse:
   - `MODE_UUID` — operating mode (0=sleep, 1=standby, 3=tracking)
   - `POWER_UUID` — power control
3. Wakes sleeping basestations on startup
4. Optionally powers down at exit (`gatt-sleep-at-exit` config flag)

Mode conflicts (two basestations assigned the same channel) are detected and
resolved by reassigning to an unused slot.

## Playback Driver (`driver_playback.c`)

Replays recorded `.rec.gz` sessions for regression testing and offline
development. It is the primary way to reproduce bugs without hardware.

The driver reads a text-based recording format with time-prefixed event lines:

```
0.012345 LIGHTCAP SO HMD 23 4012345 1850
0.012346 IMU HMD 0.012 0.001 9.810 0.001 -0.002 0.003 0 0 0
```

Playback is driven by a background thread (`playback_thread`) that sleeps
between events to maintain real-time pacing (configurable via `time-factor`,
default 1.0×). Time scrubbing via `playback-start-time` and `playback-end-time`
allows replaying subsections.

Key config items:
- `playback` — path to `.rec.gz` file
- `playback-factor` — speed multiplier (2.0 = double speed)
- `playback-start-time`, `playback-end-time` — time window to replay
- `playback-rawlight` — emit raw lightcap instead of pre-disambiguated angles

The driver handles a backward-compatibility quirk: old recordings have 11-element
IMU lines (missing magnetometer data). Both formats are parsed.

## Simulator Driver (`driver_simulator.c`)

Generates synthetic tracking data for algorithm development without hardware.
Creates a virtual HMD object with known sensor positions and emits:
- Synthetic lightcap events at configurable lighthouse positions
- Synthetic IMU data consistent with a defined motion trajectory

Used primarily for Kalman filter and optimizer development and testing.

## UDP Driver (`driver_udp.c`)

Receives pre-solved pose data over UDP multicast for sensor fusion with
external tracking systems.

- Multicast group: `224.0.2.122`, port `2333`
- Protocol: 32-bit command word followed by payload
  - `1` = config blob (JSON)
  - `2` = pose data (position + quaternion)
- Creates a single dummy `SurviveObject` ("UP0") to receive poses

The driver fires the `config` and `pose` hooks, allowing UDP-sourced objects
to participate in the normal tracking pipeline. This enables hybrid setups
(e.g., fusing libsurvive with an external motion capture system).

## OpenVR Driver (`driver_openvr.cc`)

Bridges SteamVR tracking into libsurvive as an external pose source for
calibration reference. Written in C++.

The driver polls OpenVR device poses, then computes a `openvr2survive`
coordinate transform (via Kabsch alignment on lighthouse positions) to convert
SteamVR's coordinate space into libsurvive's. Calibrated transforms are fired
as `external_pose` and `external_velocity` hooks.

The first 500 frames are used for calibration bootstrapping before poses are
considered reliable. This driver is typically used for comparison/validation,
not as a primary tracking source.

## Global Scene Solver (`driver_global_scene_solver.c`)

Despite being registered as a driver plugin, this component is functionally
a passive optimization trigger. It accumulates sensor activation measurements
across all tracked objects and fires a global optimization solve when sufficient
coverage is achieved.

Coverage is tracked as a 5-bin histogram per (lighthouse, axis). When desired
coverage thresholds are met (`gss-desired-coverage` config) and the device is
stationary (detected via IMU), a global bundle adjustment is triggered using
the `PoserFn` interface.

This is distinct from per-object pose solving — it refines lighthouse positions
relative to each other using a global constraint set.

Key config items:
- `gss-threaded` — run solve in background thread
- `gss-desired-coverage` — fraction of histogram bins that must be filled
- `gss-auto-floor-height` — set floor Z from minimum tracked object height
- `disable-calibrate` — skip solve entirely

The "driver" registration is a convention quirk: it hooks into the driver
lifecycle (init/close) and installs measurement callbacks, but produces no
raw sensor data.

## Dummy Driver (`driver_dummy.c`)

A 47-line skeleton used for documentation and testing. Creates a single virtual
HMD with one sensor at the origin. Poll and close functions are no-ops. Disabled
by default (`dummy-driver-enable = false`). Demonstrates the minimum required
driver structure.

## USBMon Driver (`driver_usbmon.c`)

Integrates with Linux `usbmon` (USB packet capture) for offline protocol
analysis and debugging. Allows replaying USB captures from real hardware without
the device present.

## Driver Loading and Lifecycle

```
survive_init()
    │
    ├── survive_load_plugins()          // discover .so files in ./plugins/
    │       └── dlopen() each plugin    // REGISTER_LINKTIME constructors run
    │               └── RegisterDriver() appends to Drivers[]
    │
    ├── For each registered driver:
    │       └── driver->fn(ctx)         // init: allocate state, install hooks
    │
    └── survive_startup()
            └── Each driver's poll_fn() called in main loop
```

Drivers that need background threads spawn them during init and signal
completion via `os_generic` semaphores. The main polling loop calls each
driver's poll function; drivers that are entirely thread-driven return
immediately from poll.

## Observed Design Decisions

| Decision | What was chosen | Evidence | Likely rationale |
|---|---|---|---|
| Plugins over static linking | `.so`/`.dll` plugin system with retry loading | `survive_plugins.c`, `REGISTER_LINKTIME` macro | Allows users to add drivers without recompiling libsurvive; handles optional dependencies (libusb, gattlib) cleanly |
| libusb vs HIDAPI abstraction | Compile-time backend selection, unified transfer API | `driver_vive.hidapi.h` vs `driver_vive.libusb.h` | Platform differences are deep (async vs sync, hotplug support); runtime switching would add complexity |
| Async config state machine | Multi-step USB init as explicit state machine | `driver_vive.config.h` states: MAGICS→CONFIG→VERSION→IMU_SCALES | USB control transfers are async; state machine handles retries and stall detection cleanly |
| Text-based recording format | Human-readable timestamped lines with sscanf parsing | `driver_playback.c`, `survive_recording.h` format macros | Easy to inspect and edit recordings manually; forward/backward compatibility via format string changes |
| Global scene solver as driver | Registered as plugin, hooks into driver lifecycle | `driver_global_scene_solver.c`, `DriverRegGlobalSceneSolver` | Reuses plugin infrastructure for init/close; can be disabled like any other driver |
| Gzip compression for recordings | `.rec.gz` files via `survive_gz.h` zlib wrapper | `driver_playback.c:gzgetline()` | Recordings can be very large (raw lightcap at 1kHz); ~10× compression typical |

## Technical Debt & Inconsistencies

1. **`driver_usbmon.c` behavior unclear from code** — The file exists and is
   built, but its implementation details were not fully captured during
   reconnaissance. Its relationship to the libusb path is unclear.

2. **`driver_simulator.c` is sparse** — Registers `DriverRegSimulator` but
   appears to have minimal implementation relative to its potential. Limited
   usefulness for algorithm development in current state.

3. **`driver_global_scene_solver` is misclassified** — Named and registered as
   a driver but performs optimization work that conceptually belongs in the
   Tracking Engine. This creates a split where calibration logic is spread
   across two clusters.

4. **OpenVR driver requires 500-frame bootstrap** — Hardcoded constant with no
   config override. If calibration diverges in the first 500 frames, the
   session's external poses will be wrong.

5. **UDP driver creates "UP0" dummy object** — The object name is hardcoded
   and there's no mechanism for multiple UDP sources. Extending to multiple
   external pose sources would require significant changes.

6. **Playback timing is thread-sleep based** — The playback thread uses
   `os_generic` sleep to pace events. Under CPU load, playback can fall behind
   real-time without detection or compensation.

## Behavioral Quirks

1. **Config chunk length off-by-two** — The Vive device advertises config
   length N but the actual compressed data is N+2 bytes. The driver adds 2
   to the reported length. (`driver_vive.config.h`, comment: "actual config
   length may exceed advertised by 2 bytes")

2. **USB consecutive timeout detection** — Three consecutive 1000ms timeouts
   on non-HMD devices trigger a "device off" detection. The HMD has no timeout
   handling — it is assumed always present. (`driver_vive.libusb.h`)

3. **RF device mode switching requires specific packet order** — Wireless
   devices require power-on → protocol-change → raw-mode packets in sequence.
   Sending them out of order leaves the device in an unresponsive state requiring
   USB re-enumeration.

4. **GATT driver does not verify mode assignment success** — After writing a
   mode to a lighthouse, there is no readback to confirm the write took effect.
   Failures are silent.

5. **Playback skips raw light if raw_imu present** — When both raw light and
   raw IMU are present in a recording, raw light events are suppressed to avoid
   double-processing. This precedence is implicit, not configurable.
   (`driver_playback.c`)

## Open Questions

1. What is the intended state of `driver_simulator.c`? Is it used in CI or
   only for manual testing?

2. `driver_usbmon.c` — what USB capture format does it consume? Is it pcap
   or Linux usbmon text format?

3. Should `driver_global_scene_solver` be moved to the Tracking Engine cluster
   in a future refactor, given that it does no I/O?

## References

**Files in this cluster:**
- [src/driver_vive.c](../../src/driver_vive.c), [src/driver_vive.h](../../src/driver_vive.h)
- [src/driver_vive.config.h](../../src/driver_vive.config.h)
- [src/driver_vive.hidapi.h](../../src/driver_vive.hidapi.h)
- [src/driver_vive.libusb.h](../../src/driver_vive.libusb.h)
- [src/driver_gatt.c](../../src/driver_gatt.c)
- [src/driver_playback.c](../../src/driver_playback.c)
- [src/driver_simulator.c](../../src/driver_simulator.c)
- [src/driver_udp.c](../../src/driver_udp.c)
- [src/driver_dummy.c](../../src/driver_dummy.c)
- [src/driver_usbmon.c](../../src/driver_usbmon.c)
- [src/driver_openvr.cc](../../src/driver_openvr.cc)
- [src/driver_global_scene_solver.c](../../src/driver_global_scene_solver.c)
- [redist/hidapi.h](../../redist/hidapi.h)
- [redist/hid-linux.c](../../redist/hid-linux.c)
- [redist/hid-osx.c](../../redist/hid-osx.c)
- [redist/hid-windows.c](../../redist/hid-windows.c)

**Dependencies on other clusters:**
- → **Library Infrastructure**: uses plugin system, config, hook callbacks, `SurviveContext`, `SurviveObject` creation, recording
- → **Lighthouse Protocol Intelligence**: fires `lightcap` hook consumed by disambiguator
- ← **Tracking Engine**: `driver_global_scene_solver` calls poser functions directly (unusual coupling)

**External dependencies:**
- `libusb-1.0` — async USB transfers (Linux/macOS)
- `hidapi` (vendored in `redist/`) — HID device access
- `gattlib` — Bluetooth LE for GATT driver (optional)
- OpenVR SDK — for `driver_openvr.cc` (optional)
- `redist/puff.c` — deflate decompression for config blobs
- `redist/jsmn.h` — JSON parsing for device config
