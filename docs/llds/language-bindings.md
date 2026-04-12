# Language Bindings

**Created**: 2026-04-11
**Status**: Mapped from existing code
**Source**: Brownfield bootstrap via /map-codebase

## Context and Current State

The Language Bindings cluster exposes libsurvive to non-C callers and
integrates it into external ecosystems. It contains Python ctypes bindings,
C# / .NET / Unity bindings, a ROS publisher, an OpenVR runtime driver, and
browser-based visualization tooling.

All bindings are thin wrappers over the Library Infrastructure's public API
(`survive_api.h` and `survive.h`). None of them implement tracking logic —
they translate libsurvive's C interface into idioms natural to the target
language or platform.

The cluster also includes integration tools (`tools/ros_publisher`,
`tools/openvr_driver`, `tools/viz`) that embed libsurvive into larger systems.
These sit here rather than in the Driver Stack because they are outbound
integrations (libsurvive data flowing out) rather than inbound data sources.

## Python Bindings (`bindings/python/pysurvive/`)

### Architecture

The Python bindings use two layers:

1. **`pysurvive_generated.py`** — Raw ctypes wrappers auto-generated from C
   headers via `ctypesgen`. Contains every exported function, struct, and
   callback typedef as ctypes objects. Regenerated when the C API changes.

2. **`__init__.py`** — Hand-written Pythonic API on top of the generated
   layer. Provides `SimpleContext`, `SimpleObject`, callback installation
   helpers, and argument parsing integration.

### Simple API Wrapper

```python
import pysurvive

ctx = pysurvive.init(sys.argv)
# or: ctx = pysurvive.SimpleContext(sys.argv)

def pose_handler(ctx, obj, time, pose):
    print(obj, pose)

pysurvive.install_pose_fn(ctx, pose_handler)

while ctx.running():
    ctx.poll()
```

`SimpleContext` wraps `SurviveSimpleContext` and provides Python-style
iteration over objects:

```python
for obj in ctx.objects():
    pose, timestamp = obj.get_pose()
```

### Callback Lifetime Management

A critical implementation detail: Python callback functions passed to ctypes
must be kept alive by the caller. If the Python GC collects a callback object
while C still holds a pointer to it, a segfault results.

`__init__.py` stores all installed callbacks in a module-level list
(`_callbacks`) to prevent collection. This is a manual pattern, not enforced
by the API — callers who install callbacks via the generated layer directly
are responsible for their own lifetime management.

### Cross-Platform Library Loading (`CustomLibraryLoader.py`)

`CustomLibraryLoader` handles platform-specific library discovery:

- **Linux** — Parses `/etc/ld.so.cache` to find `libsurvive.so`; falls back
  to standard path search
- **macOS** — Searches `DYLD_LIBRARY_PATH` and standard macOS library paths
- **Windows** — Searches `PATH` and common install locations

This replaces the default ctypes `CDLL` loader, which often fails to find
libsurvive when it is installed to a non-standard prefix.

### Data Recording and Plotting (`recorder.py`)

A Python-side data recorder that collects events into in-memory arrays for
post-session analysis:

```python
from pysurvive.recorder import Recorder
rec = Recorder()
rec.install(ctx)
# ... run session ...
rec.plot_pose_diff(reference_poses)
rec.plot_imu()
```

`RecordedData` collects: IMU (accel/gyro), poses, light angles, velocities,
and button events, all timestamped.

Analysis features:
- **Standstill detection** — `time_since_move` tracks last motion; used to
  gate calibration comparisons
- **Kabsch alignment** — Aligns recorded poses to a reference frame for
  ground-truth comparison
- **scipy rotation matching** — Finds optimal rotation between pose sequences
- **matplotlib plots** — `plot_imu`, `plot_light`, `plot_pose_diff`,
  `plot_angle_error`

### GUI Launcher (`__main__.py`)

Running `python -m pysurvive` launches a Gooey-based GUI that converts all
registered config items into form fields. The GUI wraps the argument parser
returned by `create_argument_parser()`, which enumerates all `STATIC_CONFIG_ITEM`
registrations via `config_items()`.

Can also launch `survive-websocketd` for browser-based visualization.

### Python Examples

| File | Demonstrates |
|------|-------------|
| `example.py` | Minimal `SimpleContext` polling loop |
| `full-example.py` | `init()` style with custom IMU callback |
| `button-example.py` | Button event handling with event type mapping |
| `graph-example.py` | Full recording + matplotlib graphing session |

## C# / .NET Bindings (`bindings/cs/`)

### Architecture

Three layers, matching the Python structure:

1. **`cfunctions.cs`** — Raw `[DllImport]` declarations for all exported C
   functions and callback typedefs. ~50 imported functions, ~10 callback types.

2. **`SurviveContext.cs`** — Mid-level wrapper that installs delegates for
   all hooks and holds delegate references to prevent GC collection (same
   problem as Python).

3. **`SurviveAPI.cs`** — High-level wrapper around `SurviveSimpleContext`,
   with `Start()` threading, object enumeration, and `WaitForUpdate()`.

### Core Types

```csharp
// Pose type (matches C struct layout)
[StructLayout(LayoutKind.Sequential)]
public struct SurvivePose {
    [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)]
    public double[] Pos;    // x, y, z
    [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
    public double[] Rot;    // quaternion: w, x, y, z
}

// IMU data
[StructLayout(LayoutKind.Sequential)]
public struct SurviveIMUData {
    public double[] Accel;  // 3-element
    public double[] Gyro;   // 3-element
    public double[] Mag;    // 3-element
}
```

All structs use `[StructLayout(LayoutKind.Sequential)]` with explicit array
sizes to match C memory layout exactly. The `double[]` fields use `FLT=double`
— the C# bindings do not support single-precision builds.

### Delegate Lifetime Management

`SurviveContext.cs` stores all installed delegates as private fields:

```csharp
private Cfunctions.pose_process_func _poseDelegate;

public void InstallPoseHandler(Action<IntPtr, SurvivePose> handler) {
    _poseDelegate = (so, pose) => handler(so, pose);
    Cfunctions.survive_install_pose_fn(ctx, _poseDelegate);
}
```

The private field prevents GC collection. Unlike the Python bindings, this
is enforced by the class design — callers cannot install callbacks without
going through a method that stores the delegate.

### Default Handler Chaining

`SurviveContext.cs` stores the previous handler returned by each
`survive_install_*_fn()` call and invokes it at the end of each delegate.
This correctly maintains the processing chain and ensures recording, the
Simple API queue, and other installed hooks continue to work.

### Unity Integration (`bindings/cs/UnityViewer/`)

A Unity project demonstrating real-time tracked object visualization:

```csharp
// SurviveObject.cs (MonoBehaviour)
void Start() {
    api = new SurviveAPI(args);
    api.Start();
}

void Update() {
    api.WaitForUpdate();
    foreach (var obj in api.Objects()) {
        var pose = obj.GetPose();
        // Instantiate or update GameObject transform
        gameObject.transform.position = new Vector3(pose.Pos[0], ...);
        gameObject.transform.rotation = new Quaternion(pose.Rot[1], ...);
    }
}
```

A prototype GameObject is cloned per tracked object on first detection. The
Unity coordinate system conversion (Y-up, left-handed) versus libsurvive's
coordinate system (Y-up, right-handed) is not explicitly handled in the
current code — this is a known gap.

## ROS Publisher (`tools/ros_publisher/`)

Publishes libsurvive data as ROS topics for robotics integration.

### Published Topics (per tracked object)

| Topic | Type | Content |
|-------|------|---------|
| `/{object}/pose` | `geometry_msgs/PoseStamped` | 6-DOF pose as ROS TF frame |
| `/{object}/joy` | `sensor_msgs/Joy` | Controller buttons (packed) and axes |
| `/{object}/imu` | `sensor_msgs/Imu` | Linear acceleration + angular velocity |

### Time Synchronization

libsurvive timecodes (48 MHz USB clock) are mapped to ROS time via an offset
computed on the first received event:

```
ros_time = first_ros_time + (survive_timecode - first_survive_timecode) / 48e6
```

This provides consistent relative timing within a session but does not
synchronize to wall clock or ROS master time.

### Coordinate Frame

libsurvive poses are published in libsurvive's native coordinate frame.
No transform to ROS REP-103 conventions (X-forward, Y-left, Z-up) is applied.
Callers using this with navigation or manipulation stacks need to add a
static TF transform.

## OpenVR Runtime Driver (`tools/openvr_driver/`)

Exposes libsurvive as an OpenVR tracked device driver, allowing SteamVR
applications to use libsurvive tracking instead of Valve's proprietary stack.

### Architecture

Implements the OpenVR `ITrackedDeviceServerDriver` interface per tracked
object. At SteamVR startup:

1. The driver registers all tracked objects with SteamVR's device manager
2. Each frame, it queries libsurvive for current poses
3. Converts poses from libsurvive coordinates to OpenVR coordinates via a
   pre-computed transform
4. Reports poses to SteamVR via `IVRServerDriverHost::TrackedDevicePoseUpdated`

### Coordinate Transform

libsurvive and OpenVR use different coordinate systems. The driver computes
an `openvr2survive` transform via the Kabsch algorithm using known lighthouse
positions as corresponding point pairs. This transform is computed once at
startup and reused for the session lifetime.

Note: This driver (`tools/openvr_driver/`) is the outbound direction (libsurvive
→ OpenVR). The inbound direction (OpenVR → libsurvive as a reference source)
is `src/driver_openvr.cc` in the Device Driver Stack cluster.

### Manifest

`driver.vrdrivermanifest` declares the driver to SteamVR:
```json
{
  "driver": "libsurvive",
  "description": "libsurvive OpenVR Driver"
}
```

The driver `.so`/`.dll` must be placed in SteamVR's driver directory for
discovery.

## Web Visualization (`tools/viz/`)

A Three.js web application for real-time tracking visualization.

### Architecture

Connects to a backend via WebSocket (typically `survive-cli` piped through
`websocketd`). Receives JSON pose and sensor activation updates.

### Visualization Features

- **3D scene** — Orbit camera, optional first-person camera mode
- **Tracked objects** — Rendered as 3D models or coordinate frames
- **Position trails** — Configurable-length history of position (cleared on
  reset)
- **Lighthouse geometry** — Rendered with per-lighthouse color coding
- **Sensor activation points** — Shows which sensors are currently seeing light
- **IMU visualization** — Accelerometer vector overlay
- **Time scrubbing** — Can replay recorded data with time control

### Interaction

- Mouse orbit / zoom / pan
- FPV camera toggle
- Copy/paste camera pose as JSON (for reproducing views)
- Toggle trails, models, IMU vectors

The visualization requires no build step — it's static HTML + JS served
directly, with Three.js loaded from `tools/viz/lib/`.

## Observed Design Decisions

| Decision | What was chosen | Evidence | Likely rationale |
|---|---|---|---|
| ctypesgen for Python raw layer | Auto-generated from C headers | `pysurvive_generated.py` file header | Keeps raw bindings in sync with C API changes without manual maintenance; generates complete coverage |
| Hand-written Pythonic wrapper | `__init__.py` over generated layer | `pysurvive/__init__.py` | Generated ctypes code is not idiomatic Python; hand-written layer provides natural API for users |
| Module-level callback list in Python | `_callbacks` list prevents GC | `pysurvive/__init__.py` | ctypes callbacks are GC'd if not explicitly retained; module-level list is simplest solution |
| Private delegate fields in C# | Stores delegate references in class | `SurviveContext.cs` private fields | Enforces correct delegate lifetime without requiring caller discipline; C# GC is more aggressive than Python's |
| Default handler chaining in C# | C# wrapper explicitly chains | `SurviveContext.cs` install methods | Learned from Python where missing chain silently broke pipeline; C# wrapper makes it automatic |
| ROS publisher uses relative time | Offset from first event, not wall clock | `tools/ros_publisher/main.cc` time sync | USB timecodes have no absolute reference; relative timing is correct within a session |
| Three.js for web viz | No build step, static files | `tools/viz/` (no package.json, no bundler) | Minimizes toolchain requirements; anyone can run it with `python -m http.server` |

## Technical Debt & Inconsistencies

1. **Unity coordinate system not converted** — `SurviveObject.cs` applies
   libsurvive quaternion components to Unity transforms without accounting for
   the right-hand vs left-hand coordinate system difference. Object rotations
   will be mirrored in Unity. This is a known gap in the Unity viewer.

2. **ROS publisher ignores REP-103** — No coordinate frame conversion to
   standard ROS conventions. This limits interoperability with standard ROS
   navigation and manipulation packages.

3. **Python `FLT` assumption** — `pysurvive_generated.py` assumes `FLT=double`.
   If libsurvive is compiled with `USE_SINGLE_PRECISION=ON`, the Python bindings
   will silently read wrong values (mismatched struct sizes).

4. **C# bindings hardcode `double`** — Same issue as Python. `SurvivePose.Pos`
   and `SurvivePose.Rot` are `double[]` regardless of libsurvive's compile-time
   `FLT` setting.

5. **`pysurvive_generated.py` is committed and may be stale** — The generated
   file reflects the API at time of generation. If C headers change without
   regenerating, the Python bindings silently expose the wrong API. There is no
   CI check that the generated file matches the current headers.

6. **OpenVR driver Kabsch calibration is session-scoped** — The coordinate
   transform is computed once at startup. If lighthouse positions shift during
   a session (e.g., re-calibration), the transform becomes stale and poses
   drift. There is no mechanism to recompute mid-session.

7. **Web viz WebSocket protocol is undocumented** — The message format between
   `survive-cli`/`websocketd` and `survive_viewer.js` is implicit. Any change
   to the CLI output format silently breaks the visualization.

## Behavioral Quirks

1. **Python `SimpleContext` wraps the Simple API, not the hook API** — Callers
   who want raw lightcap or angle data must use the hook API directly via
   `pysurvive.init()`, not `SimpleContext`. The naming is not obvious.

2. **`python -m pysurvive` requires Gooey** — The GUI launcher has a hard
   dependency on the `Gooey` package, which is not a standard Python library.
   Running without Gooey installed gives an `ImportError` rather than falling
   back to CLI mode.

3. **ROS publisher timecodes saturate at 32 bits** — The libsurvive timecode
   is a 32-bit counter at 48 MHz, wrapping every ~89 seconds. The ROS publisher
   does not handle wraparound, causing time jumps in published topics during
   long sessions.

4. **OpenVR driver requires SteamVR to be running** — Unlike `driver_openvr.cc`
   (which polls OpenVR), the outbound driver is a passive server that SteamVR
   must discover and load. If SteamVR isn't running, the driver does nothing
   with no error.

5. **Three.js viz has a hardcoded WebSocket port** — `survive_viewer.js`
   connects to a fixed port. Running multiple libsurvive instances requires
   manually editing the JS file.

## Open Questions

1. Is `pysurvive_generated.py` regenerated as part of any CI step, or is it
   purely manual?

2. The Unity coordinate handedness bug — is this a known issue or was it
   working at some point with a different libsurvive coordinate convention?

3. The ROS publisher timecode wraparound — is this a known issue in practice,
   or do sessions never run long enough to hit it at 48 MHz / 89 seconds?

## References

**Files in this cluster:**

Python:
- [bindings/python/pysurvive/\_\_init\_\_.py](../../bindings/python/pysurvive/__init__.py)
- [bindings/python/pysurvive/pysurvive\_generated.py](../../bindings/python/pysurvive/pysurvive_generated.py)
- [bindings/python/pysurvive/CustomLibraryLoader.py](../../bindings/python/pysurvive/CustomLibraryLoader.py)
- [bindings/python/pysurvive/recorder.py](../../bindings/python/pysurvive/recorder.py)
- [bindings/python/pysurvive/\_\_main\_\_.py](../../bindings/python/pysurvive/__main__.py)
- [bindings/python/examples/](../../bindings/python/examples/)

C# / Unity:
- [bindings/cs/libsurvive.net/SurviveAPI.cs](../../bindings/cs/libsurvive.net/SurviveAPI.cs)
- [bindings/cs/libsurvive.net/SurviveContext.cs](../../bindings/cs/libsurvive.net/SurviveContext.cs)
- [bindings/cs/libsurvive.net/cfunctions.cs](../../bindings/cs/libsurvive.net/cfunctions.cs)
- [bindings/cs/Demo/Program.cs](../../bindings/cs/Demo/Program.cs)
- [bindings/cs/UnityViewer/Assets/SurviveObject.cs](../../bindings/cs/UnityViewer/Assets/SurviveObject.cs)
- [bindings/cs/UnityViewer/Assets/ExitBehavior.cs](../../bindings/cs/UnityViewer/Assets/ExitBehavior.cs)

Integration tools:
- [tools/ros\_publisher/](../../tools/ros_publisher/)
- [tools/openvr\_driver/](../../tools/openvr_driver/)
- [tools/viz/](../../tools/viz/)

**Dependencies on other clusters:**
- → **Library Infrastructure**: all bindings wrap `survive_api.h` / `survive.h`; no direct dependency on other clusters
- → **Device Driver Stack**: `tools/openvr_driver/` is the outbound complement to `src/driver_openvr.cc`

**External dependencies:**
- ctypesgen (Python, build-time) — generates `pysurvive_generated.py`
- Gooey (Python, optional) — GUI launcher
- matplotlib, scipy (Python, optional) — recorder plotting
- .NET / Mono — C# bindings runtime
- Unity Engine — UnityViewer project
- ROS (rospy / roscpp) — ROS publisher
- OpenVR SDK — OpenVR driver
- Three.js (vendored in `tools/viz/lib/`) — web visualization
