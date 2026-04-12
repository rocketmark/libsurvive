# Arrow: device-driver-stack

Produces raw sensor events (light pulses, IMU, config, buttons) from physical hardware, recorded sessions, or synthetic sources via a unified plugin interface.

## Status

**MAPPED** - 2026-04-12. Full LLD written from brownfield reconnaissance. EARS specs written. Vive USB driver is mature; simulator is sparse; usbmon implementation details unclear.

## References

### HLD
- [docs/high-level-design.md](../high-level-design.md) — "Architecture Overview", "The Hook System"

### LLD
- [docs/llds/device-driver-stack.md](../llds/device-driver-stack.md)

### EARS
- [docs/specs/device-driver-stack-specs.md](../specs/device-driver-stack-specs.md) (28 specs)

### Tests
- src/test_cases/watchman.c
- src/test_cases/test_replays.c (exercises playback driver)
- src/test_cases/export_config.c

### Code
- src/driver_vive.c, src/driver_vive.h
- src/driver_vive.config.h, src/driver_vive.hidapi.h, src/driver_vive.libusb.h
- src/driver_gatt.c
- src/driver_playback.c
- src/driver_simulator.c
- src/driver_udp.c
- src/driver_dummy.c
- src/driver_usbmon.c
- src/driver_openvr.cc
- src/driver_global_scene_solver.c
- redist/hidapi.h, redist/hid-linux.c, redist/hid-osx.c, redist/hid-windows.c

## Architecture

**Purpose:** Translate hardware reality (USB, BLE, network, files) into `LightcapElement` and IMU events that the Protocol Intelligence cluster can process.

**Key Components:**
1. `driver_vive.c` — Primary USB driver for all HTC Vive / Valve Index family hardware
2. `driver_vive.config.h` — Async state machine for device initialization (MAGICS→CONFIG→VERSION→IMU_SCALES)
3. `driver_gatt.c` — Bluetooth LE lighthouse power/mode control
4. `driver_playback.c` — `.rec.gz` session replay for offline development and regression testing
5. `driver_global_scene_solver.c` — Passive calibration trigger (registered as driver, performs optimization work)
6. `redist/hidapi.*` — Cross-platform HID device access layer

## EARS Coverage

| Category | Spec IDs | Implemented | Gaps | Deferred |
|----------|----------|-------------|------|----------|
| Driver interface contract | DDS-API-001 to 006 | 6 | 0 | 0 |
| Vive USB driver | DDS-BE-010 to 017 | 8 | 0 | 0 |
| GATT lighthouse control | DDS-BE-020 to 023 | 4 | 0 | 0 |
| Playback driver | DDS-BE-030 to 034 | 5 | 0 | 0 |
| UDP driver | DDS-BE-040 to 042 | 3 | 0 | 0 |
| Global scene solver | DDS-BE-050 to 054 | 4 | 1 | 0 |

**Summary:** 30 of 31 active specs implemented; 1 active gap; 0 deferred.

## Key Findings

1. **`driver_global_scene_solver` is architecturally misplaced** — Registered as a driver plugin but directly calls Tracking Engine poser functions (`src/driver_global_scene_solver.c`). This is the only place where the Driver Stack bypasses the hook system to call into the Tracking Engine. (DDS-BE-050)

2. **Config chunk off-by-two is a silent quirk** — Vive firmware advertises config length N but delivers N+2 bytes. The driver adds 2 silently; this is not documented for anyone implementing a new USB backend. (`src/driver_vive.config.h`)

3. **`driver_simulator.c` is sparse** — The file registers `DriverRegSimulator` but the implementation is minimal. Not usable for algorithm development in current state.

4. **`driver_usbmon.c` behavior undetermined** — The file exists and builds but its input format and protocol are not documented in source.

5. **GATT mode write has no readback** — After writing a mode to a lighthouse via GATT, there is no confirmation read. Silent failures are undetectable. (DDS-BE-022)

## Work Required

### Must Fix
1. Add GSS solve-failure rejection path — if global scene solver result error exceeds threshold, retain previous calibration (DDS-BE-054)

### Should Fix
2. Document `driver_usbmon.c` — what format it consumes and how to use it
3. Document the off-by-two config length quirk in a comment visible to future USB backend implementors
4. Evaluate `driver_simulator.c` — either complete it or mark it as unfinished

### Nice to Have
5. Add GATT mode write confirmation readback (DDS-BE-022)
6. Consider moving `driver_global_scene_solver` to the Tracking Engine cluster to eliminate the cross-cluster direct call
