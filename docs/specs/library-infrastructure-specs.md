# Library Infrastructure — EARS Specs

Prefix: **LI**

## Context Lifecycle

- [x] **LI-API-001**: The system shall initialize a `SurviveContext` via `survive_init(argc, argv)`, parsing command-line arguments and loading the config file before returning.
- [x] **LI-API-002**: The system shall install default hook handlers for all 24 hooks during `survive_init` so the pipeline is functional before any caller installs custom handlers.
- [x] **LI-API-003**: When `survive_startup()` is called, the system shall begin event delivery by calling each registered driver's poll function on each call to `survive_poll()`.
- [x] **LI-API-004**: When `survive_close()` is called, the system shall signal all driver threads to stop, call each driver's close function, and free all allocated context state.
- [ ] **LI-API-005**: If `survive_close()` is called from a thread other than the one that called `survive_init()`, the system shall detect the condition and emit a warning rather than deadlocking.

## Hook System

- [x] **LI-BE-010**: The system shall implement all inter-cluster communication via 24 named hook function pointers in `SurviveContext`, covering the full pipeline from `lightcap` through `pose` and `velocity`.
- [x] **LI-BE-011**: When a hook handler is installed via `survive_install_*_fn()`, the system shall return the previously-installed handler so the caller can chain to it.
- [x] **LI-BE-012**: The system shall record wall-clock duration of every hook invocation in per-hook timing statistics accessible for performance profiling.
- [ ] **LI-BE-013**: Where debug mode is enabled, the system shall detect broken hook chains (a hook installed without chaining to the previous handler) and emit a warning.

## Configuration System

- [x] **LI-BE-020**: The system shall allow any module to register a config variable at compile time via `STATIC_CONFIG_ITEM`, with a default value, type, and description.
- [x] **LI-BE-021**: When `survive_init` is called, the system shall collect all statically-registered config variables, then apply values from command-line arguments and the JSON config file in priority order (command-line > file > default).
- [x] **LI-BE-022**: The system shall support runtime config access via `survive_configf`, `survive_configb`, `survive_configi`, and `survive_configs` without requiring callers to declare variables ahead of time.
- [x] **LI-BE-023**: The system shall support live variable binding via `SURVIVE_ATTACH_CONFIG`, automatically updating a C variable when its config value changes.
- [x] **LI-BE-024**: When `survive_config_save()` is called, the system shall write the current config state to the JSON config file for persistence across sessions.

## Plugin System

- [x] **LI-BE-030**: The system shall discover driver and poser plugins by searching for `.so`/`.dll` files in `./plugins/`, adjacent to `libsurvive.so`, and adjacent to the running executable.
- [x] **LI-BE-031**: When loading plugins, the system shall retry failed loads until no new plugins load successfully in a pass, resolving inter-plugin dependencies without explicit manifests.
- [x] **LI-BE-032**: The system shall support a maximum of 32 simultaneously-registered drivers.
- [ ] **LI-BE-033**: If a plugin fails to load after all retry passes, the system shall emit a diagnostic message identifying the missing symbol or dependency that caused the failure.

## Device Factory

- [x] **LI-BE-040**: When a `config` hook fires with a valid HTC/Valve JSON device config, the system shall parse sensor positions and normals, allocate a `SurviveObject`, and initialize a Kalman tracker for it.
- [x] **LI-BE-041**: When a device codename collides with an already-registered object, the system shall resolve the conflict by incrementing the last character of the codename.
- [x] **LI-BE-042**: When a lighthouse database file is present, the system shall load previously-solved lighthouse positions on startup to resume tracking without re-calibration.

## Simple API

- [x] **LI-API-050**: The system shall provide a high-level `SurviveSimpleContext` API that runs the main poll loop in a background thread, hiding hook management from the caller.
- [x] **LI-API-051**: The system shall deliver Simple API events (pose updates, button events, device connect/disconnect, config received) through a `SurviveSimpleEvent` union polled via `survive_simple_wait_for_event`.
- [x] **LI-API-052**: When the Simple API event queue overflows, the system shall drop the oldest event to make room for the newest.
- [ ] **LI-API-053**: When the Simple API event queue drops an event due to overflow, the system shall increment a counter accessible to the caller.

## Recording System

- [x] **LI-BE-060**: When recording is enabled, the system shall intercept hooks and write timestamped event lines to a gzip-compressed `.rec.gz` file.
- [x] **LI-BE-061**: The system shall use matching printf/scanf format string macros for recording and playback to ensure round-trip fidelity of all event types.
- [x] **LI-BE-062**: When recording is enabled, the system shall chain to the previously-installed hook handler after writing each event, preserving all other processing.
- [ ] **LI-BE-063**: If a recording file cannot be opened or written, the system shall emit an error and disable recording rather than silently dropping events.

## Threading

- [x] **LI-BE-070**: The system shall protect the `SurviveContext` with a recursive mutex (`survive_get_ctx_lock`) that driver background threads must acquire before invoking any hook.
- [x] **LI-BE-071**: The system shall process button events in a dedicated background thread, woken by semaphore, to prevent button delivery from blocking driver polling.
- [x] **LI-BE-072**: The system shall provide a portable OS abstraction (`os_generic`) for threads, mutexes, semaphores, condition variables, and high-resolution timing across Linux, macOS, and Windows.
