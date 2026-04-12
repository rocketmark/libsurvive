# Library Infrastructure

**Created**: 2026-04-11
**Status**: Mapped from existing code
**Source**: Brownfield bootstrap via /map-codebase

## Context and Current State

Library Infrastructure is the connective tissue of libsurvive. It provides
the context lifecycle, the hook routing system that connects all other clusters,
the configuration system, the plugin loader, data recording, and both the
low-level and high-level public APIs.

This cluster does not do any tracking math or hardware I/O. It is the runtime
that the other four clusters run inside. A caller who only ever uses the Simple
API will only directly touch this cluster — everything else is invoked
indirectly through the hook and plugin systems.

## Public API Surface

libsurvive exposes two distinct APIs with different design philosophies:

### Low-Level Hook API (`include/libsurvive/survive.h`)

The full-featured API for callers who need fine-grained control. Callers
install callbacks (hooks) that fire on each event:

```c
SurviveContext *ctx = survive_init(argc, argv);
survive_install_pose_fn(ctx, my_pose_callback);
survive_install_imu_fn(ctx, my_imu_callback);
survive_startup(ctx);
while (survive_poll(ctx) == 0) { /* event loop */ }
survive_close(ctx);
```

Every hook follows the chaining pattern: `survive_install_*_fn()` returns
the previous handler. Callers must call the previous handler at the end of
their callback to maintain the processing chain. Failing to do so silently
breaks downstream processing (e.g., if recording is installed and a caller
replaces the pose hook without chaining, recording stops).

### High-Level Simple API (`include/libsurvive/survive_api.h`)

An opaque wrapper that hides threading and callback complexity. Designed for
applications that only need pose updates:

```c
SurviveSimpleContext *actx = survive_simple_init(argc, argv);
survive_simple_start_thread(actx);
while (survive_simple_wait_for_update(actx) != 0) {
    for (const SurviveSimpleObject *obj = survive_simple_get_first_object(actx);
         obj != NULL;
         obj = survive_simple_get_next_object(actx, obj)) {
        SurvivePose pose;
        survive_simple_object_get_latest_pose(obj, &pose);
    }
}
survive_simple_close(actx);
```

The Simple API runs the main polling loop in a background thread and exposes
events through an opaque `SurviveSimpleEvent` union:
- `SURVIVE_SIMPLE_BUTTON_EVENT` — controller button/axis state change
- `SURVIVE_SIMPLE_POSE_UPDATE_EVENT` — new pose available
- `SURVIVE_SIMPLE_OBJECT_ADDED/REMOVED` — device connected/disconnected
- `SURVIVE_SIMPLE_CONFIG_EVENT` — device config received

Events are buffered in a circular queue. Overflow silently drops old events.

### API Choice and Tradeoffs

The low-level API allows installing hooks that fire at every processing stage
(raw lightcap, disambiguated angles, IMU, pose, velocity). The Simple API
only surfaces final pose, velocity, and button events. Applications that need
to log raw sensor data, implement custom posers, or monitor filter internals
must use the low-level API.

## Hook System

The hook system is the primary inter-cluster communication mechanism.
`include/libsurvive/survive_hooks.h` defines all hook names via an X-macro:

```c
// survive_hooks.h (simplified)
SURVIVE_HOOK_PROCESS_DEF(lightcap, ...)
SURVIVE_HOOK_PROCESS_DEF(imu, ...)
SURVIVE_HOOK_PROCESS_DEF(pose, ...)
SURVIVE_HOOK_PROCESS_DEF(angle, ...)
// ... 24 hooks total
```

The X-macro is expanded multiple times in `survive.h` to generate:
- Function pointer fields in `SurviveContext`
- `survive_install_*_fn()` setter functions
- Timing stat fields per hook (for profiling)

**Hook timing stats:** Every hook invocation records duration. These are
accessible via `survive_get_hook_timings()` for performance analysis.

**Default handlers:** Each hook has a `survive_default_*_process()` function
that implements the standard behavior (e.g., `survive_default_pose_process`
fires the pose to the Simple API queue). Callers who install hooks and fail to
call the default handler at the end of their chain silently disable that
default behavior.

**Full hook list:**

| Hook | Direction | Purpose |
|------|-----------|---------|
| `new_object` | Driver → App | Device connected |
| `disconnect` | Driver → App | Device disconnected |
| `config` | Driver → Engine | JSON config blob received |
| `lightcap` | Driver → Protocol | Raw light pulse |
| `light` | Protocol → Engine | Disambiguated light event |
| `light_pulse` | Protocol → Engine | Gen1 light pulse with acode |
| `angle` | Protocol → Engine | Calibrated sweep angle |
| `sync` | Protocol → Engine | Sync pulse event |
| `sweep` | Protocol → Engine | Sweep event |
| `sweep_angle` | Protocol → Engine | Sweep with angle |
| `raw_imu` | Driver → Protocol | Raw accelerometer/gyro/mag |
| `imu` | Protocol → Engine | Calibrated IMU |
| `button` | Driver → App | Controller button/axis |
| `imupose` | Engine → App | IMU-only pose estimate |
| `pose` | Engine → App | Full tracked pose |
| `velocity` | Engine → App | Pose velocity |
| `external_pose` | Driver → Engine | External reference pose |
| `external_velocity` | Driver → Engine | External reference velocity |
| `raw_lighthouse_pose` | Engine → Engine | Pre-normalized lighthouse pose |
| `lighthouse_pose` | Engine → App | Calibrated lighthouse pose |
| `ootx_received` | Protocol → App | Lighthouse calibration packet |
| `gen_detected` | Protocol → Engine | Gen1/Gen2 detection event |
| `printf` | Any → App | Log message |
| `log` | Any → App | Structured log event |
| `datalog` | Any → App | Data logging event |

## Context Lifecycle (`survive.c`)

### Initialization

`survive_init(argc, argv)` performs:

1. **Sanity checks** — Verify `sizeof(FLT)` matches compile-time expectation
2. **Allocate `SurviveContext`** — Zero-initialized; private extension
   (`SurviveContext_private`) allocated separately
3. **Install default hooks** — Each hook's function pointer set to
   `survive_default_*_process`
4. **Initialize config system** — Load static-registered config variables,
   parse command-line arguments
5. **Load config file** — Read JSON config from disk if present
6. **Load plugins** — Call `survive_load_plugins()` which discovers and
   dlopen's driver `.so` files; registered drivers are initialized
7. **Start button servicer thread** — Background thread drains the button
   event queue via semaphore

### Polling

`survive_poll(ctx)` is the main event loop iteration. It:
- Calls each registered driver's `poll_fn()` in sequence
- Drivers that are fully threaded return immediately; polling drivers do I/O
- Returns 0 to continue, non-zero to stop

### Shutdown

`survive_close(ctx)` signals all driver threads to stop, calls each driver's
`close_fn()`, frees all allocated state, and unloads plugins.

### Thread Safety

The context has a recursive mutex (`survive_get_ctx_lock()` /
`survive_release_ctx_lock()`). Drivers that spawn background threads must hold
the context lock when invoking hooks. The Simple API holds the lock internally
around all callback invocations.

## Configuration System (`survive_config.c`)

### Static-Time Registration

Config variables are registered at compile time using macros:

```c
// In any .c file:
STATIC_CONFIG_ITEM(my_param, "my-param", 'f', "description", 1.0)
```

These register a global linked list entry at program startup. The config
system iterates this list during `survive_init` to build the runtime config
table.

### Runtime Access

Drivers and posers read config via:
```c
FLT value = survive_configf(ctx, "param-name", SC_GET, default_value);
int flag   = survive_configb(ctx, "enable-flag", SC_GET, 0);
```

`SC_GET` reads without modifying. `SC_SET` writes and persists.

### Live Variable Binding

Config values can be bound directly to C variables:
```c
SURVIVE_ATTACH_CONFIG(ctx, "my-param", 'f', &my_variable);
```

After binding, the variable is automatically updated when the config value
changes at runtime (via command-line or file reload).

### Persistence

Config is read from `~/.config/libsurvive/config.json` (or a path specified
via `--config`). `survive_config_save()` writes the current config back.
Values set programmatically persist across sessions.

### Priority

Config values are resolved in order:
1. Command-line arguments (highest priority)
2. Config file on disk
3. Programmatic defaults (from `STATIC_CONFIG_ITEM`)

## Plugin System (`survive_plugins.c`)

### Discovery

`survive_load_plugins()` searches for `.so` files in:
- `./plugins/` relative to the current directory
- `./libsurvive/plugins/` relative to `libsurvive.so` location
- Path of the running executable

### Load Order Resolution

Plugins may depend on each other (e.g., a driver plugin depends on the
cnkalman math plugin). The loader uses a retry strategy:

```
Collect all .so files
Repeat until stable:
    For each unloaded plugin:
        try dlopen()
        if success → mark loaded
        if fail → leave for retry
Until no new plugins loaded in a pass (all remaining fail permanently)
```

This handles dependency ordering without explicit manifests. Plugins that fail
all retries are skipped with a warning.

`SURVIVE_PLUGIN_DEBUG=1` environment variable enables verbose load logging.

### Registration

Each plugin exports a registration function via constructor attribute:

```c
__attribute__((constructor))
void survive_register_foo() {
    RegisterDriver("DriverRegFoo", DriverRegFoo);
}
```

The `REGISTER_LINKTIME(fn)` macro wraps this pattern. For posers, `REGISTER_POSER(fn)` is used instead.

## Device Factory (`survive_default_devices.c`)

When the `config` hook fires (driver delivers device JSON), the device factory:

1. **Parses JSON** — Reads sensor positions (3D), normals (3D), and device
   metadata from the HTC/Valve config format
2. **Allocates `SurviveObject`** — Sets sensor count, timebase (48 MHz),
   IMU frequency
3. **Initializes Kalman tracker** — Calls `survive_kalman_tracker_init()`
   per object
4. **Resolves name collisions** — If a device codename is already in use,
   increments the last character (e.g., `HMD` → `HME`)
5. **Fires `new_object` hook** — Notifies the application

The factory also handles `survive_load_steamvr_lighthousedb()` for loading
previously-solved lighthouse positions from a JSON database file, allowing
sessions to resume without re-running calibration.

## Recording System (`survive_recording.c`)

The recording system intercepts hooks and writes a timestamped log of all
events to a `.rec.gz` file (gzip-compressed text).

### Activation

Recording is enabled via `--record <filename>` or the `record` config key.
`survive_recording_init()` installs recording wrappers on top of existing
hooks — each wrapper logs the event then calls the original handler (chaining).

### Format

Each line is: `<runtime_seconds> <EVENT_TYPE> <object_name> <fields...>`

Examples:
```
0.012345 LIGHTCAP HMD 23 4012345 1850
0.012346 IMU HMD 0.012 0.001 9.810 0.001 -0.002 0.003 0 0 0
0.023456 ANGLE HMD 0 1 0 1.234
0.034567 POSE HMD 0.1 0.2 1.5 1.0 0.0 0.0 0.0
```

Format strings are defined as macros in `survive_recording.h`
(e.g., `SWEEP_PRINTF`, `SYNC_SCANF`) to ensure the write and read paths
stay in sync.

### Configuration

- `record-rawlight` — record raw lightcap (default: on)
- `record-imu` — record IMU events (default: on)
- `record-angle` — record disambiguated angles (default: on)
- `record-data-matrices` — record internal optimizer matrices (diagnostic)

Output can go to file, stdout, or both simultaneously.

## Vendored Utilities (`redist/`)

Non-HID third-party code bundled with the library:

| File | Origin | Purpose |
|------|--------|---------|
| `linmath.h/.c` | cnmatrix + custom | Quaternion, vector, pose math; Kabsch alignment |
| `os_generic.h` + platform files | Custom | Threads, mutexes, semaphores, timing — portable across Linux/macOS/Windows |
| `jsmn.h` | jsmn library | Minimal JSON tokenizer |
| `json_helpers.c/.h` | Custom | JSON read/write utilities on top of jsmn |
| `puff.c/.h` | zlib/puff | Standalone DEFLATE decompressor (no zlib dependency) |
| `crc32.c/.h` | MIT licensed | CRC32 for OOTX packet validation |
| `mpfit/` | MINPACK-1 C port | Levenberg-Marquardt optimizer |
| `variance.h` | Custom | Running variance / statistics (header-only) |
| `mymath.c` | Custom | Additional rotation math (Kabsch-adjacent) |
| `dclapack.h` / `dclhelpers.c/.h` | Custom | Header-only LU decomposition macros |
| `symbol_enumerator.c/.h` | Custom | ELF / DbgHelp symbol enumeration for plugin loading |
| `CNFG3D.h` / `CNFGFunctions.h` | CNFG library | 3D software rendering for visualization tools |
| `glutil.h/.c` | Custom (stub) | OpenGL utility placeholder (nearly empty) |

### `os_generic` Design

`os_generic.h` is the only OS portability layer for threading. It provides:
- `OGCreateThread` / `OGJoinThread` — thread lifecycle
- `OGCreateMutex` / `OGLockMutex` / `OGUnlockMutex` — recursive mutexes
- `OGCreateSema` / `OGReleaseSema` / `OGAcquireSema` — semaphores
- `OGCreateConditionVariable` / `OGWaitForCondition` — condition variables
- `OGGetAbsoluteTime()` — seconds since epoch as `double`
- `OGGetAbsoluteTimeUS()` — microseconds

Implementations are in `os_generic.unix.h` (pthread) and
`os_generic.windows.h` (Win32 CRITICAL_SECTION + CreateThread).

## Test Suite (`src/test_cases/`)

A lightweight custom test framework using `REGISTER_LINKTIME` for test
discovery. Tests are registered at link time and run by the `main.c` harness.

Framework macros:
- `TEST(suite, name)` — define a test function
- `ASSERT_EQ`, `ASSERT_DOUBLE_EQ`, `ASSERT_QUAT_EQ`, `ASSERT_GE` — assertions
- `ASSERT_SUCCESS` — assert a function returns 0

Test categories:

| File | What it tests |
|------|--------------|
| `barycentric_svd.c` | SVD pose recovery from known transforms |
| `kalman.c` | EKF convergence, noise filtering, observer correctness |
| `optimizer.c` | MPFIT convergence, covariance properties |
| `reproject.c` | Gen1/Gen2 reprojection correctness at known poses |
| `rotate_angvel.c` | Angular velocity frame transformation |
| `str.c` | Dynamic string buffer correctness |
| `test_replays.c` | Full pipeline regression against recorded sessions |
| `watchman.c` | Watchman USB packet parsing |
| `export_config.c` | Device config JSON round-trip |
| `check_generated.c` | Generated math function consistency (~11K LOC) |

`test_replays.c` is the most valuable for regression testing — it runs full
sessions from `.rec.gz` files and validates pose/lighthouse accuracy against
thresholds. The `LIBSURVIVE_IGNORE_MISMATCH_TESTS` environment variable
suppresses threshold failures for known-bad baselines.

## Observed Design Decisions

| Decision | What was chosen | Evidence | Likely rationale |
|---|---|---|---|
| X-macro for hook definitions | Single `survive_hooks.h` expanded multiple ways | `survive_hooks.h`, `survive.h` expansion sites | Single source of truth for hook list; adding a hook requires one line; all downstream (struct fields, install functions, timing stats) auto-generated |
| Chaining hook pattern | `install_fn` returns previous, caller must chain | `survive_install_*_fn()` signatures | Allows multiple consumers per hook (recording + application + poser); avoids observer list complexity |
| Dual API (hook + Simple) | Both maintained in parallel | `survive_api.c`, `include/survive_api.h` | Hook API needed for advanced use; Simple API needed for ease-of-use; neither can fully replace the other |
| Static config registration | `STATIC_CONFIG_ITEM` global list built at link time | `survive_config.c` static_conf_t linked list | No central config registry file to maintain; new config items added where they're used; discoverable via `--help` |
| Retry-based plugin loading | Load all, retry failures until stable | `survive_plugins.c` loop | Avoids explicit dependency manifests; works for the small plugin count libsurvive has |
| Gzip recording by default | `.rec.gz` via zlib wrapper | `survive_gz.h`, `survive_recording.c` | Raw lightcap at 1kHz produces very large files; ~10× compression ratio makes long sessions practical |
| `puff.c` for decompression | Standalone DEFLATE, no zlib | `redist/puff.c`, used in `driver_vive.config.h` | Config decompression needed before zlib may be initialized; puff has no dependencies |

## Technical Debt & Inconsistencies

1. **Hook chaining is fragile and undocumented** — Callers who install hooks
   must call the previous handler, but this contract is not enforced. A missed
   chain call silently breaks recording, the Simple API event queue, or
   downstream processing. No compile-time or runtime warning is issued.

2. **Circular event buffer silently drops events** — The Simple API event queue
   overflows silently. Under high button-event rates or slow consumers, events
   are lost with no notification to the application.

3. **`json_helpers.c` destructively parses** — The JSON parser modifies the
   input string in place (null-terminates tokens). Callers must pass a mutable
   copy. This is not documented in the header.

4. **`mymath.c` duplicates `linmath.h`** — Several functions in `mymath.c`
   appear to duplicate quaternion operations already in `linmath.h`. The split
   is historical; the correct canonical implementation is unclear for some
   operations.

5. **`glutil.h/.c` are stubs** — Nearly empty (10 lines). The files exist,
   presumably as placeholders, but add noise to the codebase.

6. **`os_generic.windows.h` uses undocumented `NtQuerySemaphore`** — Semaphore
   count querying on Windows uses an undocumented NT API. This could break on
   future Windows versions.

7. **`survive_private.h` limits external objects to 16** — The external pose
   tracking array (`SurviveContext_private`) is fixed at 16 entries. Fusion
   with large external tracking systems (e.g., OptiTrack with many markers) is
   silently capped.

8. **Config file path is platform-specific but not abstracted** — The default
   config path uses `~/.config/libsurvive/` (Unix convention). Windows behavior
   is undocumented.

## Behavioral Quirks

1. **First `survive_load_plugins()` call initializes the global driver table** —
   The driver registry is a static global array (max 32 entries). There is no
   mechanism to unregister a driver once registered, making hot-reload impossible.

2. **`survive_close()` must be called from the same thread as `survive_init()`** —
   The context lock is recursive but not cross-thread. Closing from a callback
   thread (e.g., in response to a device disconnect) can deadlock.

3. **Recording hooks install in fixed order relative to other hooks** —
   Recording wraps whatever hooks are installed at the time recording is enabled.
   Hooks installed after recording starts are not recorded. The expected usage
   is to enable recording before calling `survive_startup()`.

4. **Button servicer runs at semaphore-wake cadence** — The button event thread
   wakes on semaphore signals from the driver. Under rapid button events, the
   servicer may batch-process multiple events per wakeup in FIFO order.

5. **Command-line argument parsing is greedy** — `survive_init(argc, argv)`
   consumes all arguments matching registered config keys. Callers cannot pass
   unknown arguments through to their own argument parser without conflict.

## Open Questions

1. Is there a plan to make the hook chaining contract enforced? (e.g., a debug
   mode that detects broken chains at runtime)

2. The Simple API circular event buffer size — what is it? Is it tunable?

3. `survive_close()` thread-safety: is there documented guidance on which
   thread must call it?

## References

**Files in this cluster:**
- [src/survive.c](../../src/survive.c)
- [src/survive_api.c](../../src/survive_api.c)
- [src/survive_internal.h](../../src/survive_internal.h)
- [src/survive_private.h](../../src/survive_private.h)
- [src/survive_config.c](../../src/survive_config.c), [src/survive_config.h](../../src/survive_config.h)
- [src/survive_default_devices.c](../../src/survive_default_devices.c), [src/survive_default_devices.h](../../src/survive_default_devices.h)
- [src/survive_driverman.c](../../src/survive_driverman.c)
- [src/survive_plugins.c](../../src/survive_plugins.c)
- [src/survive_plugins.unix.h](../../src/survive_plugins.unix.h), [src/survive_plugins.windows.h](../../src/survive_plugins.windows.h)
- [src/survive_recording.c](../../src/survive_recording.c), [src/survive_recording.h](../../src/survive_recording.h)
- [src/survive_str.c](../../src/survive_str.c), [src/survive_str.h](../../src/survive_str.h)
- [src/survive_buildinfo.c](../../src/survive_buildinfo.c)
- [src/survive_gz.h](../../src/survive_gz.h)
- [src/test_cases/](../../src/test_cases/)
- [include/libsurvive/](../../include/libsurvive/)
- [redist/](../../redist/) (non-HID files)

**Dependencies on other clusters:**
- ← **Device Driver Stack**: drivers register via this cluster's plugin system and invoke hooks
- ← **Lighthouse Protocol Intelligence**: processing functions installed as hooks
- ← **Tracking Engine**: posers and Kalman tracker registered and invoked via hook system
- ← **Language Bindings**: bindings wrap this cluster's public API (`survive_api.h`, `survive.h`)

**External dependencies:**
- `zlib` (optional) — gzip compression for recording; falls back to stdio if `NOZLIB` defined
- `dl` (Linux) / `LoadLibrary` (Windows) — dynamic plugin loading
- pthread (Linux/macOS) / Win32 threads (Windows) — via `os_generic`
