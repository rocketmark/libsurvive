# Arrow: language-bindings

Exposes libsurvive tracking to Python, C#/Unity, ROS, OpenVR, and browser-based visualization.

## Status

**MAPPED** - 2026-04-12. Full LLD written from brownfield reconnaissance. EARS specs written. Python and C# bindings are functional but have precision-mismatch and staleness risks. Unity coordinate conversion is broken. ROS has a timecode wraparound bug.

## References

### HLD
- [docs/high-level-design.md](../high-level-design.md) — "Non-Goals / Synchronize to wall clock"

### LLD
- [docs/llds/language-bindings.md](../llds/language-bindings.md)

### EARS
- [docs/specs/language-bindings-specs.md](../specs/language-bindings-specs.md) (26 specs)

### Tests
- bindings/python/examples/ (manual integration examples)
- bindings/cs/Demo/Program.cs

### Code
- bindings/python/pysurvive/
- bindings/cs/libsurvive.net/, bindings/cs/Demo/, bindings/cs/UnityViewer/
- tools/ros_publisher/
- tools/openvr_driver/
- tools/viz/

## Architecture

**Purpose:** Translate libsurvive's C API into idioms natural to each target language or platform, with no tracking logic of their own.

**Key Components:**
1. `pysurvive/pysurvive_generated.py` — ctypesgen-produced raw C bindings
2. `pysurvive/__init__.py` — Pythonic wrapper: SimpleContext, callbacks, argument parser
3. `pysurvive/recorder.py` — In-session data collection and matplotlib analysis
4. `bindings/cs/libsurvive.net/` — C# DllImport layer + SurviveAPI + SurviveContext
5. `bindings/cs/UnityViewer/` — Unity MonoBehaviour for real-time 3D tracking visualization
6. `tools/ros_publisher/` — ROS topic publisher (pose, joy, IMU) per tracked object
7. `tools/openvr_driver/` — Outbound OpenVR runtime driver (libsurvive → SteamVR)
8. `tools/viz/` — Three.js web visualization via WebSocket

## EARS Coverage

| Category | Spec IDs | Implemented | Gaps | Deferred |
|----------|----------|-------------|------|----------|
| Python bindings | LB-API-001 to 006 | 4 | 2 | 0 |
| Python data recording | LB-BE-010 to 012 | 3 | 0 | 0 |
| C# / .NET bindings | LB-API-020 to 024 | 4 | 1 | 0 |
| Unity integration | LB-API-030 to 031 | 1 | 1 | 0 |
| ROS integration | LB-API-040 to 044 | 3 | 2 | 0 |
| OpenVR runtime driver | LB-API-050 to 052 | 2 | 1 | 0 |
| Web visualization | LB-API-060 to 062 | 2 | 1 | 0 |

**Summary:** 19 of 26 active specs implemented; 8 active gaps; 0 deferred.

## Key Findings

1. **Unity coordinate handedness is broken** — `SurviveObject.cs` maps libsurvive quaternion components to Unity transforms without applying the right-hand → left-hand conversion. Object rotations are mirrored in Unity. (`bindings/cs/UnityViewer/Assets/SurviveObject.cs`, LB-API-031)

2. **Python and C# bindings assume FLT=double** — Both binding layers hardcode `double` for pose structs. Compiling libsurvive with `USE_SINGLE_PRECISION=ON` produces silently wrong values in all bindings. (LB-API-005, LB-API-024)

3. **`pysurvive_generated.py` can silently go stale** — No CI step regenerates or validates it against current headers. API changes in C are invisible to Python users until the next manual regeneration. (LB-API-006)

4. **ROS timecode wraps every ~89 seconds** — The 32-bit 48 MHz USB timecode overflows every 89 seconds. The ROS publisher does not handle wraparound, causing ROS time to jump backward during long sessions. (LB-API-043)

5. **ROS coordinate frame not converted** — Published poses are in libsurvive's native coordinate frame. ROS REP-103 expects X-forward, Y-left, Z-up. Standard ROS navigation and manipulation stacks will interpret poses incorrectly. (LB-API-044)

6. **OpenVR coordinate transform is session-scoped** — Computed once at startup via Kabsch; not recomputed if lighthouse positions change mid-session. (LB-API-052)

7. **Web viz WebSocket port is hardcoded** — `survive_viewer.js` connects to a fixed port. Multiple concurrent libsurvive instances require source edits. (LB-API-062)

## Work Required

### Must Fix
1. Fix Unity coordinate system conversion in `SurviveObject.cs` — apply right-hand to left-hand transform on quaternion (LB-API-031)
2. Fix ROS timecode wraparound — detect 32-bit overflow and maintain monotonic timestamps (LB-API-043)

### Should Fix
3. Add FLT precision check in Python bindings at import time — raise error if struct sizes don't match (LB-API-005)
4. Add ROS REP-103 coordinate frame conversion to `tools/ros_publisher/` (LB-API-044)
5. Add CI step to regenerate and diff `pysurvive_generated.py` (LB-API-006)
6. Make WebSocket port configurable in `survive_viewer.js` (LB-API-062)

### Nice to Have
7. Add mid-session coordinate transform recomputation to OpenVR driver on re-calibration event (LB-API-052)
8. Add C# struct size validation test (LB-API-024)
