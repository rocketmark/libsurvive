# Language Bindings — EARS Specs

Prefix: **LB**

## Python Bindings

- [x] **LB-API-001**: The system shall expose all libsurvive C API functions and callback typedefs to Python via a ctypes-generated binding layer (`pysurvive_generated.py`).
- [x] **LB-API-002**: The system shall provide a Pythonic wrapper (`pysurvive/__init__.py`) over the generated layer, offering `SimpleContext`, `SimpleObject`, and callback installation helpers.
- [x] **LB-API-003**: When a Python callback is installed, the system shall retain a reference to the callback object to prevent garbage collection while it is registered.
- [x] **LB-API-004**: The system shall locate the libsurvive shared library on Linux, macOS, and Windows using platform-specific search strategies (ld.so.cache, DYLD paths, PATH).
- [ ] **LB-API-005**: The system shall verify at import time that the `pysurvive_generated.py` struct sizes match the compiled libsurvive FLT precision, and raise an error if they do not.
- [ ] **LB-API-006**: The system shall regenerate `pysurvive_generated.py` as part of the CI build when C API headers change.

## Python Data Recording

- [x] **LB-BE-010**: The system shall provide a Python `Recorder` class that collects IMU, pose, light angle, velocity, and button events into in-memory arrays during a live session.
- [x] **LB-BE-011**: The system shall provide matplotlib-based plot functions for recorded IMU, light, pose, and angle error data.
- [x] **LB-BE-012**: When comparing recorded poses to a reference, the system shall apply Kabsch alignment to find the optimal rigid transform before computing error metrics.

## C# / .NET Bindings

- [x] **LB-API-020**: The system shall expose all libsurvive C API functions to C# via `[DllImport]` declarations in `cfunctions.cs`.
- [x] **LB-API-021**: When a C# delegate is installed as a hook callback, the system shall store a reference to the delegate in a private field to prevent garbage collection.
- [x] **LB-API-022**: When a C# hook wrapper installs a new callback, the system shall automatically chain to the previously-installed callback at the end of each invocation.
- [x] **LB-API-023**: The system shall provide a high-level `SurviveAPI` class that wraps `SurviveSimpleContext` with `Start()`, `WaitForUpdate()`, and object enumeration.
- [ ] **LB-API-024**: The system shall document the required `StructLayout` and array size constraints for C# structs that must match C memory layout, and provide a test that validates struct sizes at runtime.

## Unity Integration

- [x] **LB-API-030**: The system shall provide a Unity `MonoBehaviour` (`SurviveObject.cs`) that initializes libsurvive, instantiates GameObjects per tracked device, and updates their transforms each frame.
- [ ] **LB-API-031**: When mapping libsurvive poses to Unity transforms, the system shall apply the coordinate system conversion from libsurvive's right-handed convention to Unity's left-handed convention.

## ROS Integration

- [x] **LB-API-040**: The system shall publish libsurvive object poses as ROS `geometry_msgs/PoseStamped` messages on a per-object topic.
- [x] **LB-API-041**: The system shall publish controller button and axis state as ROS `sensor_msgs/Joy` messages on a per-object topic.
- [x] **LB-API-042**: The system shall publish IMU data (linear acceleration + angular velocity) as ROS `sensor_msgs/Imu` messages on a per-object topic.
- [ ] **LB-API-043**: When the libsurvive 32-bit timecode wraps around (every ~89 seconds at 48 MHz), the system shall detect the wrap and maintain monotonically-increasing ROS timestamps.
- [ ] **LB-API-044**: The system shall apply a coordinate frame transform from libsurvive's native convention to ROS REP-103 (X-forward, Y-left, Z-up) before publishing poses.

## OpenVR Runtime Driver

- [x] **LB-API-050**: The system shall expose libsurvive tracked objects as OpenVR devices by implementing `ITrackedDeviceServerDriver` per object.
- [x] **LB-API-051**: The system shall compute the libsurvive-to-OpenVR coordinate transform once at startup using known lighthouse positions as corresponding point pairs via the Kabsch algorithm.
- [ ] **LB-API-052**: When lighthouse positions change mid-session (re-calibration), the system shall recompute the libsurvive-to-OpenVR coordinate transform.

## Web Visualization

- [x] **LB-API-060**: The system shall provide a Three.js web application that connects to a libsurvive WebSocket backend and renders tracked objects, lighthouses, and sensor activations in 3D.
- [x] **LB-API-061**: The system shall render configurable-length position trails for each tracked object to visualize recent motion history.
- [ ] **LB-API-062**: The system shall make the WebSocket backend port configurable without requiring source code edits to `survive_viewer.js`.
