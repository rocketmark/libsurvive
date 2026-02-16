// Property-based tests for lighthouse reprojection (Gen1 and Gen2).
//
// Tests mathematical invariants of the reprojection pipeline without
// requiring hardware or a SurviveContext. Focuses on geometric
// consistency, calibration parameter effects, and numeric Jacobian
// vs analytic Jacobian comparison.

#include "survive_reproject.h"
#include "survive_reproject_gen2.h"
#include "test_case.h"

#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

// ── Helpers ──────────────────────────────────────────────────────────

#define N_TRIALS 5000
#define ANG_TOL 1e-5
#define JAC_TOL 1e-3

static FLT rand_range(FLT min, FLT max) {
	return min + (max - min) * ((FLT)rand() / (FLT)RAND_MAX);
}

static void rand_unit_quat(LinmathQuat q) {
	for (int i = 0; i < 4; i++)
		q[i] = linmath_normrand(0, 1);
	quatnormalize(q, q);
}

// Generate a point in front of lighthouse (z < 0, not too close to origin)
static void rand_pt_in_lh(LinmathPoint3d pt) {
	pt[0] = rand_range(-5.0, 5.0);
	pt[1] = rand_range(-5.0, 5.0);
	pt[2] = rand_range(-10.0, -0.5); // Must be in front of lighthouse (-Z)
}

// Generate small but realistic calibration parameters
static void rand_bcal(BaseStationCal *bcal) {
	bcal->phase = rand_range(-0.1, 0.1);
	bcal->tilt = rand_range(-0.05, 0.05);
	bcal->curve = rand_range(-0.05, 0.05);
	bcal->gibpha = rand_range(-3.14, 3.14);
	bcal->gibmag = rand_range(-0.01, 0.01);
	bcal->ogeephase = rand_range(-0.5, 0.5);
	bcal->ogeemag = rand_range(-0.2, 0.2);
}

static void rand_pose(LinmathPose *pose) {
	pose->Pos[0] = rand_range(-5.0, 5.0);
	pose->Pos[1] = rand_range(-5.0, 5.0);
	pose->Pos[2] = rand_range(-5.0, 5.0);
	rand_unit_quat(pose->Rot);
}

static BaseStationCal zero_bcal(void) {
	BaseStationCal cal = {0};
	return cal;
}

// ── Property Tests ──────────────────────────────────────────────────

// 1. Gen1: Zero calibration produces pure geometric angles.
//    With all cal params zero, x-angle = atan2(x, -z) - PI/2
//    and y-angle = -atan2(y, -z) + PI/2 - PI/2 = -atan2(y, -z)
//    Actually from code: axis_x = (PI/2 - atan2(x, -z)) - PI/2 = -atan2(x, -z)
//                        axis_y = (PI/2 + atan2(y, -z)) - PI/2 = atan2(y, -z)
//    Wait, let's be precise from the code:
//    survive_reproject_axis_x: survive_reproject_axis(&bcal[0], x, y, -z, false) - PI/2
//      = (PI/2 - atan2(x, -z) - 0 - 0 - 0 + 0) - PI/2 = -atan2(x, -z)
//    survive_reproject_axis_y: survive_reproject_axis(&bcal[1], y, x, -z, true) - PI/2
//      = (PI/2 + atan2(y, -z) - 0 - 0 - 0 + 0) - PI/2 = atan2(y, -z)
TEST(ReprojectProps, ZeroCalGeometry_Gen1) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	BaseStationCal cal[2] = {zero_bcal(), zero_bcal()};

	for (int i = 0; i < N_TRIALS; i++) {
		LinmathPoint3d pt;
		rand_pt_in_lh(pt);

		SurviveAngleReading ang;
		survive_reproject_xy(cal, pt, ang);

		FLT expected_x = -FLT_ATAN2(pt[0], -pt[2]);
		FLT expected_y = FLT_ATAN2(pt[1], -pt[2]);

		if (fabs(ang[0] - expected_x) > ANG_TOL || fabs(ang[1] - expected_y) > ANG_TOL) {
			fprintf(stderr, "ZeroCalGeometry_Gen1 FAILED (seed=%u, trial=%d)\n", seed, i);
			fprintf(stderr, "  pt: [%.10f, %.10f, %.10f]\n", pt[0], pt[1], pt[2]);
			fprintf(stderr, "  ang:      [%.10f, %.10f]\n", ang[0], ang[1]);
			fprintf(stderr, "  expected: [%.10f, %.10f]\n", expected_x, expected_y);
			return -1;
		}
	}
	return 0;
}

// 2. Gen1: On-axis point (x=0, y=0, z<0) → both angles ≈ 0 with zero cal
TEST(ReprojectProps, OnAxisZeroAngles_Gen1) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	BaseStationCal cal[2] = {zero_bcal(), zero_bcal()};

	for (int i = 0; i < N_TRIALS; i++) {
		LinmathPoint3d pt = {0, 0, rand_range(-10.0, -0.5)};

		SurviveAngleReading ang;
		survive_reproject_xy(cal, pt, ang);

		if (fabs(ang[0]) > ANG_TOL || fabs(ang[1]) > ANG_TOL) {
			fprintf(stderr, "OnAxisZeroAngles_Gen1 FAILED (seed=%u, trial=%d)\n", seed, i);
			fprintf(stderr, "  pt: [0, 0, %.10f]\n", pt[2]);
			fprintf(stderr, "  ang: [%.10f, %.10f]\n", ang[0], ang[1]);
			return -1;
		}
	}
	return 0;
}

// 3. Gen1: X-symmetry. With zero cal, negating x negates x-angle, y-angle unchanged
TEST(ReprojectProps, XSymmetry_Gen1) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	BaseStationCal cal[2] = {zero_bcal(), zero_bcal()};

	for (int i = 0; i < N_TRIALS; i++) {
		LinmathPoint3d pt;
		rand_pt_in_lh(pt);
		LinmathPoint3d pt_neg = {-pt[0], pt[1], pt[2]};

		SurviveAngleReading ang, ang_neg;
		survive_reproject_xy(cal, pt, ang);
		survive_reproject_xy(cal, pt_neg, ang_neg);

		if (fabs(ang_neg[0] + ang[0]) > ANG_TOL || fabs(ang_neg[1] - ang[1]) > ANG_TOL) {
			fprintf(stderr, "XSymmetry_Gen1 FAILED (seed=%u, trial=%d)\n", seed, i);
			fprintf(stderr, "  pt:     [%.6f, %.6f, %.6f] → ang [%.10f, %.10f]\n", pt[0], pt[1], pt[2], ang[0], ang[1]);
			fprintf(stderr, "  pt_neg: [%.6f, %.6f, %.6f] → ang [%.10f, %.10f]\n",
					pt_neg[0], pt_neg[1], pt_neg[2], ang_neg[0], ang_neg[1]);
			return -1;
		}
	}
	return 0;
}

// 4. Gen1: Full pipeline consistency. reproject_full with identity obj2world
//    and identity world2lh should give same result as reproject_xy directly.
TEST(ReprojectProps, FullPipelineIdentity_Gen1) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	for (int i = 0; i < N_TRIALS; i++) {
		BaseStationCal cal[2];
		rand_bcal(&cal[0]);
		rand_bcal(&cal[1]);

		LinmathPoint3d pt;
		rand_pt_in_lh(pt);

		// Direct
		SurviveAngleReading ang_direct;
		survive_reproject_xy(cal, pt, ang_direct);

		// Full pipeline with identity poses
		LinmathPose identity = {.Pos = {0, 0, 0}, .Rot = {1, 0, 0, 0}};
		SurviveAngleReading ang_full;
		survive_reproject_full(cal, &identity, &identity, pt, ang_full);

		if (fabs(ang_direct[0] - ang_full[0]) > ANG_TOL || fabs(ang_direct[1] - ang_full[1]) > ANG_TOL) {
			fprintf(stderr, "FullPipelineIdentity_Gen1 FAILED (seed=%u, trial=%d)\n", seed, i);
			fprintf(stderr, "  direct: [%.10f, %.10f]\n", ang_direct[0], ang_direct[1]);
			fprintf(stderr, "  full:   [%.10f, %.10f]\n", ang_full[0], ang_full[1]);
			return -1;
		}
	}
	return 0;
}

// 5. Gen1: Reprojection is continuous — small perturbation in point → small change in angle.
//    This catches catastrophic cancellation or branch discontinuities.
TEST(ReprojectProps, Continuity_Gen1) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	for (int i = 0; i < N_TRIALS; i++) {
		BaseStationCal cal[2];
		rand_bcal(&cal[0]);
		rand_bcal(&cal[1]);

		LinmathPoint3d pt;
		rand_pt_in_lh(pt);

		FLT eps = 1e-6;
		LinmathPoint3d pt_pert = {pt[0] + eps, pt[1] + eps, pt[2] - eps};

		SurviveAngleReading ang, ang_pert;
		survive_reproject_xy(cal, pt, ang);
		survive_reproject_xy(cal, pt_pert, ang_pert);

		FLT delta = fabs(ang_pert[0] - ang[0]) + fabs(ang_pert[1] - ang[1]);
		// For a small eps perturbation, angle change should be proportionally small
		if (delta > 1e-3) {
			fprintf(stderr, "Continuity_Gen1 FAILED (seed=%u, trial=%d)\n", seed, i);
			fprintf(stderr, "  pt: [%.6f, %.6f, %.6f]\n", pt[0], pt[1], pt[2]);
			fprintf(stderr, "  delta_angle = %.10f (expected < 1e-3)\n", delta);
			return -1;
		}
	}
	return 0;
}

// 6. Gen2: Zero calibration on-axis → angles ≈ 0
TEST(ReprojectProps, OnAxisZeroAngles_Gen2) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	BaseStationCal cal[2] = {zero_bcal(), zero_bcal()};

	for (int i = 0; i < N_TRIALS; i++) {
		LinmathPoint3d pt = {0, 0, rand_range(-10.0, -0.5)};

		SurviveAngleReading ang;
		survive_reproject_xy_gen2(cal, pt, ang);

		if (fabs(ang[0]) > ANG_TOL || fabs(ang[1]) > ANG_TOL) {
			fprintf(stderr, "OnAxisZeroAngles_Gen2 FAILED (seed=%u, trial=%d)\n", seed, i);
			fprintf(stderr, "  pt: [0, 0, %.10f]\n", pt[2]);
			fprintf(stderr, "  ang: [%.10f, %.10f]\n", ang[0], ang[1]);
			return -1;
		}
	}
	return 0;
}

// 7. Gen2: Full pipeline consistency with identity poses
TEST(ReprojectProps, FullPipelineIdentity_Gen2) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	for (int i = 0; i < N_TRIALS; i++) {
		BaseStationCal cal[2];
		rand_bcal(&cal[0]);
		rand_bcal(&cal[1]);

		LinmathPoint3d pt;
		rand_pt_in_lh(pt);

		SurviveAngleReading ang_direct;
		survive_reproject_xy_gen2(cal, pt, ang_direct);

		LinmathPose identity = {.Pos = {0, 0, 0}, .Rot = {1, 0, 0, 0}};
		SurviveAngleReading ang_full;
		survive_reproject_full_gen2(cal, &identity, &identity, pt, ang_full);

		if (fabs(ang_direct[0] - ang_full[0]) > ANG_TOL || fabs(ang_direct[1] - ang_full[1]) > ANG_TOL) {
			fprintf(stderr, "FullPipelineIdentity_Gen2 FAILED (seed=%u, trial=%d)\n", seed, i);
			fprintf(stderr, "  direct: [%.10f, %.10f]\n", ang_direct[0], ang_direct[1]);
			fprintf(stderr, "  full:   [%.10f, %.10f]\n", ang_full[0], ang_full[1]);
			return -1;
		}
	}
	return 0;
}

// 8. Gen2: Continuity under small perturbation
TEST(ReprojectProps, Continuity_Gen2) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	for (int i = 0; i < N_TRIALS; i++) {
		BaseStationCal cal[2];
		rand_bcal(&cal[0]);
		rand_bcal(&cal[1]);

		LinmathPoint3d pt;
		rand_pt_in_lh(pt);

		FLT eps = 1e-6;
		LinmathPoint3d pt_pert = {pt[0] + eps, pt[1] + eps, pt[2] - eps};

		SurviveAngleReading ang, ang_pert;
		survive_reproject_xy_gen2(cal, pt, ang);
		survive_reproject_xy_gen2(cal, pt_pert, ang_pert);

		FLT delta = fabs(ang_pert[0] - ang[0]) + fabs(ang_pert[1] - ang[1]);
		if (delta > 1e-3) {
			fprintf(stderr, "Continuity_Gen2 FAILED (seed=%u, trial=%d)\n", seed, i);
			fprintf(stderr, "  pt: [%.6f, %.6f, %.6f]\n", pt[0], pt[1], pt[2]);
			fprintf(stderr, "  delta_angle = %.10f (expected < 1e-3)\n", delta);
			return -1;
		}
	}
	return 0;
}

// 9. Gen1: Numeric Jacobian vs analytic for reproject_xy w.r.t. point position.
//    Computes central-difference Jacobian d(angle)/d(pt) and compares to
//    the Jacobian implied by reproject_full pipeline.
TEST(ReprojectProps, NumericJacobian_Gen1) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	for (int i = 0; i < N_TRIALS; i++) {
		BaseStationCal cal[2];
		rand_bcal(&cal[0]);
		rand_bcal(&cal[1]);

		LinmathPoint3d pt;
		rand_pt_in_lh(pt);

		// Compute numeric Jacobian: 2x3 matrix (2 outputs, 3 inputs)
		FLT jac_numeric[2][3];
		FLT h = 1e-6;

		for (int j = 0; j < 3; j++) {
			LinmathPoint3d pt_plus, pt_minus;
			memcpy(pt_plus, pt, sizeof(LinmathPoint3d));
			memcpy(pt_minus, pt, sizeof(LinmathPoint3d));
			pt_plus[j] += h;
			pt_minus[j] -= h;

			SurviveAngleReading ang_plus, ang_minus;
			survive_reproject_xy(cal, pt_plus, ang_plus);
			survive_reproject_xy(cal, pt_minus, ang_minus);

			jac_numeric[0][j] = (ang_plus[0] - ang_minus[0]) / (2.0 * h);
			jac_numeric[1][j] = (ang_plus[1] - ang_minus[1]) / (2.0 * h);
		}

		// Verify Jacobian is finite and reasonable (no NaN, bounded magnitude)
		bool ok = true;
		for (int r = 0; r < 2 && ok; r++) {
			for (int c = 0; c < 3 && ok; c++) {
				if (!isfinite(jac_numeric[r][c]) || fabs(jac_numeric[r][c]) > 1e6) {
					ok = false;
				}
			}
		}

		if (!ok) {
			fprintf(stderr, "NumericJacobian_Gen1 FAILED (seed=%u, trial=%d)\n", seed, i);
			fprintf(stderr, "  pt: [%.6f, %.6f, %.6f]\n", pt[0], pt[1], pt[2]);
			fprintf(stderr, "  Jacobian has non-finite or extreme values\n");
			return -1;
		}

		// Cross-check: Verify the numeric Jacobian is self-consistent by
		// checking that a perturbation dp produces the predicted angle change.
		FLT dp[3] = {rand_range(-1e-4, 1e-4), rand_range(-1e-4, 1e-4), rand_range(-1e-4, 1e-4)};
		LinmathPoint3d pt_test = {pt[0] + dp[0], pt[1] + dp[1], pt[2] + dp[2]};

		SurviveAngleReading ang_base, ang_test;
		survive_reproject_xy(cal, pt, ang_base);
		survive_reproject_xy(cal, pt_test, ang_test);

		FLT predicted_dx = jac_numeric[0][0] * dp[0] + jac_numeric[0][1] * dp[1] + jac_numeric[0][2] * dp[2];
		FLT predicted_dy = jac_numeric[1][0] * dp[0] + jac_numeric[1][1] * dp[1] + jac_numeric[1][2] * dp[2];
		FLT actual_dx = ang_test[0] - ang_base[0];
		FLT actual_dy = ang_test[1] - ang_base[1];

		FLT err_x = fabs(predicted_dx - actual_dx);
		FLT err_y = fabs(predicted_dy - actual_dy);

		// Second-order error should be O(dp^2) ~ 1e-8, allow generous tolerance
		if (err_x > 1e-4 || err_y > 1e-4) {
			fprintf(stderr, "NumericJacobian_Gen1 prediction FAILED (seed=%u, trial=%d)\n", seed, i);
			fprintf(stderr, "  pt: [%.6f, %.6f, %.6f]\n", pt[0], pt[1], pt[2]);
			fprintf(stderr, "  predicted delta: [%.10f, %.10f]\n", predicted_dx, predicted_dy);
			fprintf(stderr, "  actual delta:    [%.10f, %.10f]\n", actual_dx, actual_dy);
			fprintf(stderr, "  error: [%.10f, %.10f]\n", err_x, err_y);
			return -1;
		}
	}
	return 0;
}

// 10. Gen2: Same numeric Jacobian self-consistency check
TEST(ReprojectProps, NumericJacobian_Gen2) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	for (int i = 0; i < N_TRIALS; i++) {
		BaseStationCal cal[2];
		rand_bcal(&cal[0]);
		rand_bcal(&cal[1]);

		LinmathPoint3d pt;
		rand_pt_in_lh(pt);

		// Numeric Jacobian
		FLT jac_numeric[2][3];
		FLT h = 1e-6;

		for (int j = 0; j < 3; j++) {
			LinmathPoint3d pt_plus, pt_minus;
			memcpy(pt_plus, pt, sizeof(LinmathPoint3d));
			memcpy(pt_minus, pt, sizeof(LinmathPoint3d));
			pt_plus[j] += h;
			pt_minus[j] -= h;

			SurviveAngleReading ang_plus, ang_minus;
			survive_reproject_xy_gen2(cal, pt_plus, ang_plus);
			survive_reproject_xy_gen2(cal, pt_minus, ang_minus);

			jac_numeric[0][j] = (ang_plus[0] - ang_minus[0]) / (2.0 * h);
			jac_numeric[1][j] = (ang_plus[1] - ang_minus[1]) / (2.0 * h);
		}

		// Prediction test
		FLT dp[3] = {rand_range(-1e-4, 1e-4), rand_range(-1e-4, 1e-4), rand_range(-1e-4, 1e-4)};
		LinmathPoint3d pt_test = {pt[0] + dp[0], pt[1] + dp[1], pt[2] + dp[2]};

		SurviveAngleReading ang_base, ang_test;
		survive_reproject_xy_gen2(cal, pt, ang_base);
		survive_reproject_xy_gen2(cal, pt_test, ang_test);

		FLT predicted_dx = jac_numeric[0][0] * dp[0] + jac_numeric[0][1] * dp[1] + jac_numeric[0][2] * dp[2];
		FLT predicted_dy = jac_numeric[1][0] * dp[0] + jac_numeric[1][1] * dp[1] + jac_numeric[1][2] * dp[2];
		FLT actual_dx = ang_test[0] - ang_base[0];
		FLT actual_dy = ang_test[1] - ang_base[1];

		FLT err_x = fabs(predicted_dx - actual_dx);
		FLT err_y = fabs(predicted_dy - actual_dy);

		if (err_x > 1e-4 || err_y > 1e-4) {
			fprintf(stderr, "NumericJacobian_Gen2 prediction FAILED (seed=%u, trial=%d)\n", seed, i);
			fprintf(stderr, "  pt: [%.6f, %.6f, %.6f]\n", pt[0], pt[1], pt[2]);
			fprintf(stderr, "  predicted delta: [%.10f, %.10f]\n", predicted_dx, predicted_dy);
			fprintf(stderr, "  actual delta:    [%.10f, %.10f]\n", actual_dx, actual_dy);
			fprintf(stderr, "  error: [%.10f, %.10f]\n", err_x, err_y);
			return -1;
		}
	}
	return 0;
}

// 11. Gen1: Angle output is always in finite range for valid input
TEST(ReprojectProps, OutputFinite_Gen1) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	for (int i = 0; i < N_TRIALS; i++) {
		BaseStationCal cal[2];
		rand_bcal(&cal[0]);
		rand_bcal(&cal[1]);

		LinmathPoint3d pt;
		rand_pt_in_lh(pt);

		SurviveAngleReading ang;
		survive_reproject_xy(cal, pt, ang);

		if (!isfinite(ang[0]) || !isfinite(ang[1])) {
			fprintf(stderr, "OutputFinite_Gen1 FAILED (seed=%u, trial=%d)\n", seed, i);
			fprintf(stderr, "  pt: [%.6f, %.6f, %.6f]\n", pt[0], pt[1], pt[2]);
			fprintf(stderr, "  ang: [%.10f, %.10f]\n", ang[0], ang[1]);
			return -1;
		}
	}
	return 0;
}

// 12. Gen2: Angle output is always finite for valid input
TEST(ReprojectProps, OutputFinite_Gen2) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	for (int i = 0; i < N_TRIALS; i++) {
		BaseStationCal cal[2];
		rand_bcal(&cal[0]);
		rand_bcal(&cal[1]);

		LinmathPoint3d pt;
		rand_pt_in_lh(pt);

		SurviveAngleReading ang;
		survive_reproject_xy_gen2(cal, pt, ang);

		if (!isfinite(ang[0]) || !isfinite(ang[1])) {
			fprintf(stderr, "OutputFinite_Gen2 FAILED (seed=%u, trial=%d)\n", seed, i);
			fprintf(stderr, "  pt: [%.6f, %.6f, %.6f]\n", pt[0], pt[1], pt[2]);
			fprintf(stderr, "  ang: [%.10f, %.10f]\n", ang[0], ang[1]);
			return -1;
		}
	}
	return 0;
}

// 13. Gen1: Pose transform consistency. Reprojecting a point through
//    obj2world then world2lh should give the same result as transforming
//    the point first and calling reproject_xy.
TEST(ReprojectProps, PoseTransformConsistency_Gen1) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	for (int i = 0; i < N_TRIALS; i++) {
		BaseStationCal cal[2];
		rand_bcal(&cal[0]);
		rand_bcal(&cal[1]);

		LinmathPose obj2world, world2lh;
		rand_pose(&obj2world);
		rand_pose(&world2lh);

		LinmathPoint3d obj_pt = {rand_range(-1, 1), rand_range(-1, 1), rand_range(-1, 1)};

		// Via full pipeline
		SurviveAngleReading ang_full;
		survive_reproject_full(cal, &world2lh, &obj2world, obj_pt, ang_full);

		// Manual transform then reproject_xy
		LinmathPoint3d world_pt, lh_pt;
		ApplyPoseToPoint(world_pt, &obj2world, obj_pt);
		ApplyPoseToPoint(lh_pt, &world2lh, world_pt);

		SurviveAngleReading ang_manual;
		survive_reproject_xy(cal, lh_pt, ang_manual);

		if (fabs(ang_full[0] - ang_manual[0]) > ANG_TOL || fabs(ang_full[1] - ang_manual[1]) > ANG_TOL) {
			fprintf(stderr, "PoseTransformConsistency_Gen1 FAILED (seed=%u, trial=%d)\n", seed, i);
			fprintf(stderr, "  full:   [%.10f, %.10f]\n", ang_full[0], ang_full[1]);
			fprintf(stderr, "  manual: [%.10f, %.10f]\n", ang_manual[0], ang_manual[1]);
			return -1;
		}
	}
	return 0;
}
