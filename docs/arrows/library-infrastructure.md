# Arrow: library-infrastructure

Provides context lifecycle, hook routing, configuration, plugin loading, recording, device factory, and both the low-level hook API and high-level Simple API.

## Status

**MAPPED** - 2026-04-12. Full LLD written from brownfield reconnaissance. EARS specs written. Core infrastructure is stable and well-tested. Key gaps: broken hook chain detection, event queue overflow visibility.

## References

### HLD
- [docs/high-level-design.md](../high-level-design.md) — "The Hook System: Connective Tissue", "Cross-Cutting Concerns"

### LLD
- [docs/llds/library-infrastructure.md](../llds/library-infrastructure.md)

### EARS
- [docs/specs/library-infrastructure-specs.md](../specs/library-infrastructure-specs.md) (29 specs)

### Tests
- src/test_cases/main.c, src/test_cases/test_case.h
- src/test_cases/str.c
- src/test_cases/export_config.c
- src/test_cases/test_replays.c

### Code
- src/survive.c, src/survive_api.c
- src/survive_config.c, src/survive_config.h
- src/survive_plugins.c, src/survive_driverman.c
- src/survive_recording.c, src/survive_recording.h
- src/survive_default_devices.c
- src/survive_str.c, src/survive_buildinfo.c, src/survive_gz.h
- src/survive_internal.h, src/survive_private.h
- include/libsurvive/
- redist/ (non-HID)
- src/test_cases/

## Architecture

**Purpose:** Provide the runtime environment — context, hooks, config, plugins, recording — that all other clusters run inside.

**Key Components:**
1. `survive.c` — Context init/poll/close; default hook implementations; button servicer thread
2. `survive_api.c` — Simple API: background poll thread, event queue, object wrappers
3. `survive_config.c` — Static-time registration + runtime access + file persistence
4. `survive_plugins.c` — Retry-based `.so`/`.dll` discovery and loading
5. `survive_recording.c` — Hook-intercepting recorder to `.rec.gz`
6. `survive_default_devices.c` — `SurviveObject` factory from JSON config blobs
7. `redist/os_generic.h` — Portable threads/mutexes/semaphores/timing

## EARS Coverage

| Category | Spec IDs | Implemented | Gaps | Deferred |
|----------|----------|-------------|------|----------|
| Context lifecycle | LI-API-001 to 005 | 4 | 1 | 0 |
| Hook system | LI-BE-010 to 013 | 3 | 1 | 0 |
| Configuration system | LI-BE-020 to 024 | 5 | 0 | 0 |
| Plugin system | LI-BE-030 to 033 | 3 | 1 | 0 |
| Device factory | LI-BE-040 to 042 | 3 | 0 | 0 |
| Simple API | LI-API-050 to 053 | 3 | 1 | 0 |
| Recording system | LI-BE-060 to 063 | 3 | 1 | 0 |
| Threading | LI-BE-070 to 072 | 3 | 0 | 0 |

**Summary:** 27 of 29 active specs implemented; 6 active gaps; 0 deferred.

## Key Findings

1. **Hook chaining is silently fragile** — Callers who install hooks but fail to chain to the previous handler silently break recording, the Simple API queue, and downstream processing. No runtime detection exists. (LI-BE-013)

2. **Simple API event queue overflows silently** — Events are dropped without any counter or notification to the application. High button-event rates or slow consumers lose data invisibly. (LI-API-053)

3. **`survive_close()` thread-safety undocumented** — The context mutex is recursive but not cross-thread-safe for close. Calling `survive_close()` from a callback thread can deadlock. No documentation warns against this. (LI-API-005)

4. **Plugin failure diagnostics are weak** — When a plugin fails all load retries, the error message does not identify the missing symbol. Debugging new plugin dependencies requires manual `dlopen` inspection. (LI-BE-033)

5. **`json_helpers.c` destructive parsing is undocumented** — The parser modifies its input string in-place. This is a correctness trap for callers who pass string literals or shared buffers. (`redist/json_helpers.c`)

## Work Required

### Must Fix
1. Document `survive_close()` threading constraint — must be called from the init thread (LI-API-005)

### Should Fix
2. Add overflow counter to Simple API event queue so callers can detect dropped events (LI-API-053)
3. Add debug-mode broken hook chain detection — warn when a hook is installed without chaining (LI-BE-013)
4. Improve plugin load failure diagnostics to identify the missing symbol (LI-BE-033)
5. Document `json_helpers.c` destructive parsing in the header

### Nice to Have
6. Add recording-open failure error path with graceful disable (LI-BE-063)
7. Consider raising the 32-driver limit or making it dynamic (LI-BE-032)
