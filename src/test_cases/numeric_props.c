// Property-based tests for numeric robustness / NaN propagation
//
// Fuzz math functions with adversarial and degenerate inputs to verify
// graceful handling. These tests catch cases where NaN/Inf silently
// propagates (asserts are disabled in release builds).

#include "test_case.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <survive_reproject.h>
#include <survive_reproject_gen2.h>

// ── Helpers ──────────────────────────────────────────────────────────

#define N_TRIALS 10000
#define ROBUST_TOL 1e-5

static FLT rand_range(FLT min, FLT max) {
	return min + (max - min) * ((FLT)rand() / (FLT)RAND_MAX);
}

static int is_finite_3d(const FLT *v) {
	return isfinite(v[0]) && isfinite(v[1]) && isfinite(v[2]);
}

static int is_finite_quat(const FLT *q) {
	return isfinite(q[0]) && isfinite(q[1]) && isfinite(q[2]) && isfinite(q[3]);
}

// ── Property Tests ──────────────────────────────────────────────────

// 1. quatnormalize(zero) behavior — documents whether it produces NaN
//    Current code: 1/sqrt(0) -> Inf, so components become NaN.
//    This test documents the behavior rather than asserting a specific outcome.
TEST(NumericProps, QuatNormalizeZero) {
	LinmathQuat zero = {0, 0, 0, 0};
	LinmathQuat result;
	quatnormalize(result, zero);

	// Just verify no crash. Document that the result is non-finite.
	// This is a known limitation — calling normalize on a zero quaternion
	// is undefined behavior in the library.
	if (is_finite_quat(result)) {
		// If someone fixes this to return identity, that's fine too
		FLT mag = quatmagnitude(result);
		if (fabs(mag - 1.0) > ROBUST_TOL) {
			fprintf(stderr, "QuatNormalizeZero: finite but not unit? |q|=%.10f\n", mag);
			return -1;
		}
	}
	// Either NaN (current) or unit quat (if fixed) is acceptable
	return 0;
}

// 2. quatrotatevector with non-unit quaternion still produces finite output
TEST(NumericProps, RotateWithNonUnitQuat) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	for (int trial = 0; trial < N_TRIALS; trial++) {
		LinmathQuat q;
		LinmathPoint3d v, result;

		// Random non-unit quaternion (could be very large or small)
		for (int i = 0; i < 4; i++)
			q[i] = rand_range(-100.0, 100.0);
		if (fabs(q[0]) + fabs(q[1]) + fabs(q[2]) + fabs(q[3]) < 1e-10) {
			q[0] = 1.0; // avoid total zero
		}

		v[0] = rand_range(-10.0, 10.0);
		v[1] = rand_range(-10.0, 10.0);
		v[2] = rand_range(-10.0, 10.0);

		quatrotatevector(result, q, v);

		if (!is_finite_3d(result)) {
			fprintf(stderr, "RotateWithNonUnitQuat FAILED (seed=%u, trial=%d)\n", seed, trial);
			fprintf(stderr, "  q: [%.6f, %.6f, %.6f, %.6f]\n", q[0], q[1], q[2], q[3]);
			fprintf(stderr, "  v: [%.6f, %.6f, %.6f]\n", v[0], v[1], v[2]);
			fprintf(stderr, "  result: [%.6f, %.6f, %.6f]\n", result[0], result[1], result[2]);
			return -1;
		}
	}
	return 0;
}

// 3. Gen1 reprojection with valid inputs (point in front of lighthouse) produces finite angles
TEST(NumericProps, ReprojectGen1Finite) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	for (int trial = 0; trial < N_TRIALS; trial++) {
		BaseStationCal bcal[2];
		for (int a = 0; a < 2; a++) {
			bcal[a].phase = rand_range(-0.01, 0.01);
			bcal[a].tilt = rand_range(-0.01, 0.01);
			bcal[a].curve = rand_range(-0.01, 0.01);
			bcal[a].gibpha = rand_range(-LINMATHPI, LINMATHPI);
			bcal[a].gibmag = rand_range(-0.01, 0.01);
		}

		// Point in front of lighthouse (z < 0 in lighthouse frame)
		LinmathVec3d pt = {
			rand_range(-5.0, 5.0),
			rand_range(-5.0, 5.0),
			rand_range(-10.0, -0.1) // in front
		};

		SurviveAngleReading out;
		survive_reproject_xy(bcal, pt, out);

		if (!isfinite(out[0]) || !isfinite(out[1])) {
			fprintf(stderr, "ReprojectGen1Finite FAILED (seed=%u, trial=%d)\n", seed, trial);
			fprintf(stderr, "  pt: [%.6f, %.6f, %.6f]\n", pt[0], pt[1], pt[2]);
			fprintf(stderr, "  angles: [%.10f, %.10f]\n", out[0], out[1]);
			return -1;
		}
	}
	return 0;
}

// 4. Gen2 reprojection with valid inputs produces finite angles
TEST(NumericProps, ReprojectGen2Finite) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	for (int trial = 0; trial < N_TRIALS; trial++) {
		BaseStationCal bcal[2];
		for (int a = 0; a < 2; a++) {
			bcal[a].phase = rand_range(-0.01, 0.01);
			bcal[a].tilt = rand_range(-0.01, 0.01);
			bcal[a].curve = rand_range(-0.01, 0.01);
			bcal[a].gibpha = rand_range(-LINMATHPI, LINMATHPI);
			bcal[a].gibmag = rand_range(-0.01, 0.01);
			bcal[a].ogeephase = rand_range(-LINMATHPI, LINMATHPI);
			bcal[a].ogeemag = rand_range(-0.01, 0.01);
		}

		LinmathVec3d pt = {
			rand_range(-5.0, 5.0),
			rand_range(-5.0, 5.0),
			rand_range(-10.0, -0.1)
		};

		SurviveAngleReading out;
		survive_reproject_xy_gen2(bcal, pt, out);

		if (!isfinite(out[0]) || !isfinite(out[1])) {
			fprintf(stderr, "ReprojectGen2Finite FAILED (seed=%u, trial=%d)\n", seed, trial);
			fprintf(stderr, "  pt: [%.6f, %.6f, %.6f]\n", pt[0], pt[1], pt[2]);
			fprintf(stderr, "  angles: [%.10f, %.10f]\n", out[0], out[1]);
			return -1;
		}
	}
	return 0;
}

// 5. normalize3d(zero) — documents NaN behavior for zero vector
TEST(NumericProps, Normalize3dZero) {
	FLT zero[3] = {0, 0, 0};
	FLT result[3];
	normalize3d(result, zero);

	// 1/sqrt(0) -> Inf, result = {0*Inf, 0*Inf, 0*Inf} = {NaN, NaN, NaN}
	// Just verify no crash. Either NaN or some handled value is acceptable.
	(void)result;
	return 0;
}

// 6. ApplyPoseToPoint with identity pose preserves the point
TEST(NumericProps, IdentityPosePreservesPoint) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	for (int trial = 0; trial < N_TRIALS; trial++) {
		LinmathPose identity = {.Pos = {0, 0, 0}, .Rot = {1, 0, 0, 0}};
		LinmathPoint3d pt, result;
		pt[0] = rand_range(-1000.0, 1000.0);
		pt[1] = rand_range(-1000.0, 1000.0);
		pt[2] = rand_range(-1000.0, 1000.0);

		ApplyPoseToPoint(result, &identity, pt);

		for (int j = 0; j < 3; j++) {
			if (fabs(result[j] - pt[j]) > ROBUST_TOL) {
				fprintf(stderr, "IdentityPosePreservesPoint FAILED (seed=%u, trial=%d)\n", seed, trial);
				fprintf(stderr, "  pt[%d]: %.10f, result: %.10f\n", j, pt[j], result[j]);
				return -1;
			}
		}
	}
	return 0;
}

// 7. Reprojection of point on boresight at extreme distance — tests far field
TEST(NumericProps, ReprojectExtremeDistance) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	for (int trial = 0; trial < N_TRIALS; trial++) {
		BaseStationCal bcal[2] = {{0}, {0}}; // zero calibration

		// Point directly ahead at extreme distance
		FLT dist = rand_range(100.0, 10000.0);
		LinmathVec3d pt = {
			rand_range(-0.01, 0.01), // tiny offset from boresight
			rand_range(-0.01, 0.01),
			-dist
		};

		SurviveAngleReading out;
		survive_reproject_xy(bcal, pt, out);

		if (!isfinite(out[0]) || !isfinite(out[1])) {
			fprintf(stderr, "ReprojectExtremeDistance FAILED (seed=%u, trial=%d)\n", seed, trial);
			fprintf(stderr, "  pt: [%.6f, %.6f, %.6f]\n", pt[0], pt[1], pt[2]);
			fprintf(stderr, "  angles: [%.10f, %.10f]\n", out[0], out[1]);
			return -1;
		}

		// Angles should be near zero for near-boresight point
		if (fabs(out[0]) > 0.01 || fabs(out[1]) > 0.01) {
			fprintf(stderr, "ReprojectExtremeDistance: unexpectedly large angles (seed=%u, trial=%d)\n", seed, trial);
			fprintf(stderr, "  angles: [%.10f, %.10f]\n", out[0], out[1]);
			return -1;
		}
	}
	return 0;
}

// 8. Kabsch with collinear points — should not crash
TEST(NumericProps, KabschCollinear) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	for (int trial = 0; trial < 1000; trial++) {
		// Generate collinear points (all on a line)
		int n_pts = 4 + (rand() % 7);
		FLT ptsA[10 * 3], ptsB[10 * 3];

		LinmathPoint3d dir;
		dir[0] = rand_range(-1.0, 1.0);
		dir[1] = rand_range(-1.0, 1.0);
		dir[2] = rand_range(-1.0, 1.0);

		for (int i = 0; i < n_pts; i++) {
			FLT t = rand_range(-10.0, 10.0);
			ptsA[i * 3 + 0] = dir[0] * t;
			ptsA[i * 3 + 1] = dir[1] * t;
			ptsA[i * 3 + 2] = dir[2] * t;
			// ptsB = ptsA shifted
			ptsB[i * 3 + 0] = ptsA[i * 3 + 0] + 1.0;
			ptsB[i * 3 + 1] = ptsA[i * 3 + 1] + 2.0;
			ptsB[i * 3 + 2] = ptsA[i * 3 + 2] + 3.0;
		}

		LinmathPose result;
		Kabsch(&result, ptsA, ptsB, n_pts);

		// Just verify no crash / no NaN in position
		if (!is_finite_3d(result.Pos)) {
			fprintf(stderr, "KabschCollinear FAILED (seed=%u, trial=%d)\n", seed, trial);
			fprintf(stderr, "  result.Pos: [%.10f, %.10f, %.10f]\n",
					result.Pos[0], result.Pos[1], result.Pos[2]);
			return -1;
		}
	}
	return 0;
}

// 9. Kabsch with coplanar points — should not crash
TEST(NumericProps, KabschCoplanar) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	for (int trial = 0; trial < 1000; trial++) {
		int n_pts = 4 + (rand() % 7);
		FLT ptsA[10 * 3], ptsB[10 * 3];

		// All points in the XY plane (z=0)
		for (int i = 0; i < n_pts; i++) {
			ptsA[i * 3 + 0] = rand_range(-10.0, 10.0);
			ptsA[i * 3 + 1] = rand_range(-10.0, 10.0);
			ptsA[i * 3 + 2] = 0.0; // coplanar

			ptsB[i * 3 + 0] = ptsA[i * 3 + 0] + 1.0;
			ptsB[i * 3 + 1] = ptsA[i * 3 + 1] + 2.0;
			ptsB[i * 3 + 2] = 3.0;
		}

		LinmathPose result;
		Kabsch(&result, ptsA, ptsB, n_pts);

		if (!is_finite_3d(result.Pos) || !is_finite_quat(result.Rot)) {
			fprintf(stderr, "KabschCoplanar FAILED (seed=%u, trial=%d)\n", seed, trial);
			fprintf(stderr, "  result.Pos: [%.10f, %.10f, %.10f]\n",
					result.Pos[0], result.Pos[1], result.Pos[2]);
			fprintf(stderr, "  result.Rot: [%.10f, %.10f, %.10f, %.10f]\n",
					result.Rot[0], result.Rot[1], result.Rot[2], result.Rot[3]);
			return -1;
		}
	}
	return 0;
}

// 10. Large quaternion components don't cause overflow in quatrotatevector
TEST(NumericProps, LargeQuatNoOverflow) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	for (int trial = 0; trial < N_TRIALS; trial++) {
		LinmathQuat q;
		LinmathPoint3d v, result;

		// Very large quaternion
		for (int i = 0; i < 4; i++)
			q[i] = rand_range(-1e6, 1e6);
		if (fabs(q[0]) + fabs(q[1]) + fabs(q[2]) + fabs(q[3]) < 1.0)
			q[0] = 1e6;

		v[0] = rand_range(-1.0, 1.0);
		v[1] = rand_range(-1.0, 1.0);
		v[2] = rand_range(-1.0, 1.0);

		quatrotatevector(result, q, v);

		if (!is_finite_3d(result)) {
			fprintf(stderr, "LargeQuatNoOverflow FAILED (seed=%u, trial=%d)\n", seed, trial);
			fprintf(stderr, "  q: [%.2f, %.2f, %.2f, %.2f]\n", q[0], q[1], q[2], q[3]);
			fprintf(stderr, "  result: [%e, %e, %e]\n", result[0], result[1], result[2]);
			return -1;
		}
	}
	return 0;
}
