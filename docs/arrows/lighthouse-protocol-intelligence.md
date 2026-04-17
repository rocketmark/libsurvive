# Arrow: lighthouse-protocol-intelligence

Decodes raw photon timestamps into calibrated sweep angles and lighthouse calibration data using the Valve Lighthouse Gen1 and Gen2 optical protocols.

## Status

**MAPPED** - 2026-04-12. Full LLD written from brownfield reconnaissance. EARS specs written. Gen1 path well-understood; Gen2 path has known gaps (debug output, incomplete process_gen2.c).

## References

### HLD
- [docs/high-level-design.md](../high-level-design.md) — "Lighthouse Generation Support", "Dual-Path Tracking Architecture"

### LLD
- [docs/llds/lighthouse-protocol-intelligence.md](../llds/lighthouse-protocol-intelligence.md)

### EARS
- [docs/specs/lighthouse-protocol-intelligence-specs.md](../specs/lighthouse-protocol-intelligence-specs.md) (40 specs)

### Tests
- src/test_cases/watchman.c
- src/test_cases/check_generated.c

### Code
- src/lfsr.c, src/lfsr.h
- src/lfsr_lh2.c, src/lfsr_lh2.h
- src/ootx_decoder.c, src/ootx_decoder.h
- src/disambiguator_statebased.c
- src/survive_disambiguator.c
- src/survive_process.c, src/survive_process_gen1.c, src/survive_process_gen2.c
- src/survive_sensor_activations.c
- src/generated/lighthouse_gen1.{py,gen.h}
- src/generated/lighthouse_gen2.py
- src/generated/lighthouse_model.{py,gen.h}

## Architecture

**Purpose:** Translate raw `LightcapElement` pulse timestamps from the Driver Stack into calibrated angle measurements and `BaseStationCal` structs for the Tracking Engine.

**Key Components:**
1. `survive_disambiguator.c` — Gen1/Gen2 autodetection; routes to correct decoder
2. `disambiguator_statebased.c` — Gen1 12-state cycle-locked state machine; acode extraction
3. `ootx_decoder.c` — Bitstream decoder extracting factory calibration from sync pulse data bits
4. `lfsr_lh2.c` — Gen2 LFSR polynomial matching for channel identity (0–31)
5. `survive_process_gen1.c` — Angle computation and OOTX bit injection for Gen1
6. `survive_sensor_activations.c` — Chauvenet outlier rejection; 2-axis validity gating
7. `src/generated/lighthouse_gen1.gen.h` — Codegen-produced reprojection + Jacobians

## EARS Coverage

| Category | Spec IDs | Implemented | Gaps | Deferred |
|----------|----------|-------------|------|----------|
| Protocol autodetection | LPI-PROC-001 to 004 | 3 | 1 | 0 |
| Gen1 disambiguation | LPI-PROC-010 to 016 | 7 | 0 | 0 |
| OOTX decoding | LPI-PROC-020 to 027 | 6 | 1 | 0 |
| Gen2 LFSR | LPI-PROC-030 to 032 | 2 | 1 | 0 |
| Angle computation | LPI-PROC-040 to 042 | 3 | 0 | 0 |
| Sensor activation filtering | LPI-PROC-050 to 053 | 4 | 0 | 0 |
| Back-face normal filter | LPI-PROC-060 to 066 | 7 | 0 | 0 |
| Reprojection model | LPI-DATA-060 to 063 | 4 | 0 | 0 |

**Summary:** 36 of 40 specs implemented; 3 active gaps; 0 deferred.

## Key Findings

1. **Gen2 path is incomplete** — `src/survive_process_gen2.c` is nearly empty; Gen2 LFSR decoder (`lfsr_lh2.c`) contains extensive debug `fprintf` calls indicating active development state. (LPI-PROC-032)

2. **One-way generation detection** — Once the system commits to Gen1 or Gen2 for a device, there is no recovery path. A mis-detection in the 500-packet window produces a bad session with no error surfaced. (LPI-PROC-004)

3. **OOTX sync bit errors are silent** — The decoder tracks `guess_bits` but does not surface degraded calibration data to the application. Corrupt calibration silently reduces tracking accuracy. (LPI-PROC-026)

4. **Gen2 polynomial coefficients are undocumented magic numbers** — `src/generated/lighthouse_gen2.py` line ~52 contains hardcoded empirical coefficients with no source documented. (LPI-DATA-061)

5. **Generated files committed without CI guard** — `src/generated/*.gen.h` can silently diverge from their Python sources. (TE-DATA-064 also affected)

## Work Required

### Must Fix
1. Remove debug `fprintf` calls from `src/lfsr_lh2.c` before any production Gen2 use (LPI-PROC-032)
2. Complete `src/survive_process_gen2.c` — currently the Gen2 processing path is non-functional for full angle extraction (LPI-PROC-032, gap in spec coverage)

### Should Fix
3. Surface OOTX quality metrics (bad_crcs, bad_sync_bits, guess_bits) to callers so degraded calibration is visible (LPI-PROC-026)
4. Add warning when generation detection produces conflicting classification (LPI-PROC-004)
5. Document the origin of Gen2 polynomial series coefficients in `lighthouse_gen2.py`

### Nice to Have
6. Add CI check to detect divergence between `src/generated/*.py` sources and committed `.gen.h` files
