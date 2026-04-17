# libsurvive Launch-Readiness Triage

**Created**: 2026-04-17
**Context**: Stagehand virtual-production shoots. Robustness + stability over all else. Fail closed and loudly — grey failures are the worst failure mode. Cheaper to stop a shoot than fix bad data in post.
**Source docs**: `specs/*`, `arrows/*`, `reflection-rejection.md`

## Summary

33 open items in the libsurvive docs, bucketed by shoot-readiness impact.

- 6 **launch blockers** — silent corruption or grey failures
- 5 **should fix before launch** — diagnosability gaps
- 7 **nice to have** — robustness / dev hygiene
- 8 **save for later** — irrelevant to stagehand (third-party bindings)

---

## Launch Blockers

Each item either (a) lets bad data through without visibility, or (b) makes on-set incident response harder because docs/specs don't match reality.

- [x] **LB-1 — Close spec drift on back-face filter** (`LPI-PROC-060` through `LPI-PROC-066`)
  - Category: **coherence / operational safety**
  - Feature deployed to both Pis with `--filter-normal-facingness 0.1`. Verified implemented in `src/survive_sensor_activations.c:139-183` (2026-04-17). All 7 specs marked `[x]` in [lighthouse-protocol-intelligence-specs.md](specs/lighthouse-protocol-intelligence-specs.md).
  - Arrow-doc coverage table still needs updating in [lighthouse-protocol-intelligence.md](arrows/lighthouse-protocol-intelligence.md).

- [ ] **LB-2 — Verify GSS solve-failure rejection** (`DDS-BE-054`)
  - Category: **fail-closed on calibration corruption**
  - Spec: if global scene solver error exceeds threshold, reject result and retain previous calibration.
  - `--max-cal-error 0.01` clamps the initial solve, but mid-session re-solve rejection may not be wired.
  - Fix: audit `driver_global_scene_solver.c` for rejection path; implement if missing.
  - Risk if skipped: bad GSS solve silently overwrites good calibration mid-shoot.

- [ ] **LB-3 — Recording file open/write failure = loud error** (`LI-BE-063`)
  - Category: **fail-loud (directly matches stagehand principle)**
  - Currently libsurvive can silently drop recording events if the file cannot be written.
  - For shoots using `.rec.gz` as authoritative capture, silent drop = unrecoverable grey failure.
  - Fix: emit error and disable recording instead of silent drop.

- [ ] **LB-4 — Remove debug `fprintf` from Gen2 LFSR decoder** (`LPI-PROC-032`)
  - Category: **operational safety**
  - Uncontrolled stdout in hot path — pollutes logs, can distort timing.
  - Flagged in [lighthouse-protocol-intelligence.md](arrows/lighthouse-protocol-intelligence.md) "Must Fix #1". Cheap.
  - Fix: wrap in `SV_VERBOSE` or delete.

- [ ] **LB-5 — Resolve duplicate spec IDs `TE-PROC-040/041`**
  - Category: **coherence**
  - Defined twice in [tracking-engine-specs.md](specs/tracking-engine-specs.md): once for `lc-angular-rate-max` (lines 43–44), once for Kalman lighthouse tracker (lines 49–50).
  - Breaks grep-based traceability during incident response.
  - Fix: renumber the Kalman-lighthouse-tracker pair to unused IDs.

- [ ] **LB-6 — Measure + enable pose-emission gates** (`kalman-max-pose-angular-rate`, `--light-max-error`)
  - Category: **operational safety / fail-closed on output**
  - From [reflection-rejection.md](../reflection-rejection.md) bottom checklist. Orthogonal to existing input-side defenses.
  - Without these, a reflection that slips past `light-outlier-threshold` + `filter-normal-facingness` reaches the output stream.
  - Fix:
    - [ ] Run `reflect_test.cap` with `--survive-verbose 105`, record per-pose angular rates during reflection bursts and clean tracking.
    - [ ] Set `--kalman-max-pose-angular-rate` from the data (expected 5–10 rad/s).
    - [ ] Enable `--light-max-error 0.01` in the agent config.

## Should Fix Before Launch

Diagnosability. Won't corrupt data but make grey failures hard to spot.

- [ ] **SF-1 — Warn on high OOTX sync bit error rate** (`LPI-PROC-026`)
  - Category: **fail-loud on degraded calibration**
  - Silent OOTX degradation is the classic grey failure.

- [ ] **SF-2 — Surface IEKF divergence as a warning** (tracking-engine arrow, Should Fix #2)
  - Category: **fail-loud**
  - IEKF hitting max-iter without convergence = Kalman is guessing. Currently invisible.
  - Fix: emit warning via `printf` hook on max-iter termination.

- [ ] **SF-3 — Warn on conflicting Gen1/Gen2 generation classification** (`LPI-PROC-004`)
  - Category: **fail-loud**
  - Mid-session generation flips mean calibration decoded under the wrong model.

- [ ] **SF-4 — Implement angular-rate gates** (`TE-PROC-040`, `TE-PROC-041`, `TE-PROC-042`)
  - Category: **operational safety**
  - Overlaps with LB-6. These specs cover both the Kalman-input gate and the pose-emission gate.
  - Fix: implement the three specs end-to-end.

- [ ] **SF-5 — Fix stagehand developer-guide `REFLECTION_JUMP_DEG` doc**
  - Category: **fail-loud during incident response**
  - `stagehand/docs/developer-guide.md:493` says default is 15°; code value is 25°.
  - Misleading docs at 3am on-set is exactly when wrong numbers hurt.

## Nice to Have

Robustness + dev hygiene. Won't block a shoot.

- [ ] **NH-1 — `survive_close()` cross-thread warning instead of deadlock** (`LI-API-005`)
- [ ] **NH-2 — Simple API event queue overflow counter** (`LI-API-053`)
- [ ] **NH-3 — Plugin load failure symbol diagnostics** (`LI-BE-033`)
- [ ] **NH-4 — CI guard: `src/generated/` vs Python sources** (`TE-DATA-064`)
- [ ] **NH-5 — Debug-mode broken hook-chain detection** (`LI-BE-013`)
- [ ] **NH-6 — Arrow-doc documentation items**
  - `survive_close()` threading constraint
  - `driver_usbmon.c` format + usage
  - Off-by-two config length quirk
  - Evaluate `driver_simulator.c` (complete or mark unfinished)
  - Evaluate `poser_epnp.c` (when vs BaryCentricSVD)
  - IMU bias model update cadence + interaction with main Kalman
- [ ] **NH-7 — Submit `quatdist` fix upstream to collabora/libsurvive**
  - Reduces future rebase pain. From [reflection-rejection.md](../reflection-rejection.md).

## Save for Later

Irrelevant to stagehand's C-embedded + custom Python receiver architecture. Valuable for third-party libsurvive users.

- [ ] **SL-1** — `LB-API-005` — Python binding FLT precision check
- [ ] **SL-2** — `LB-API-006` — CI regen for `pysurvive_generated.py`
- [ ] **SL-3** — `LB-API-024` — C# StructLayout size test
- [ ] **SL-4** — `LB-API-031` — Unity handedness bug fix
- [ ] **SL-5** — `LB-API-043` — ROS 32-bit timecode wraparound
- [ ] **SL-6** — `LB-API-044` — ROS REP-103 coordinate frame transform
- [ ] **SL-7** — `LB-API-052` — OpenVR mid-session coord transform recompute
- [ ] **SL-8** — `LB-API-062` — WebSocket backend port configurability

## Recommended Sequence

1. **Zero-risk first**: LB-1, LB-4, LB-5, SF-5 — pure doc/cleanup fixes, no code risk.
2. **Investigation next**: LB-2 (GSS rejection audit), LB-6 Phase 1 (measure angular rates).
3. **Implementation**: LB-3, LB-6 Phase 2 (enable gates), SF-1 → SF-4.
4. **Post-launch**: NH-1 through NH-7, then SL-*.

## References

- [specs/](specs/) — all spec files
- [arrows/](arrows/) — arrow status + Work Required lists
- [arrows/index.yaml](arrows/index.yaml) — arrow dependency graph
- [reflection-rejection.md](reflection-rejection.md) — reflection defense layering + bottom checklist
- [back-face-filter-hld.md](back-face-filter-hld.md) — back-face filter design rationale
