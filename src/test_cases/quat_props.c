// Property-based tests for quaternion and pose math in linmath.c
//
// Each test generates N random inputs and checks that a mathematical
// invariant holds for all of them. Failures print the seed and input
// values for reproducibility.

#include "test_case.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

// ── Helpers ──────────────────────────────────────────────────────────

#define N_TRIALS 10000
#define QUAT_TOL 1e-5
#define POSE_TOL 1e-4

// Generate a random FLT in [min, max]
static FLT rand_range(FLT min, FLT max) {
	return min + (max - min) * ((FLT)rand() / (FLT)RAND_MAX);
}

// Generate a random unit quaternion (uniform on S3 via normalization of 4 gaussians)
static void rand_unit_quat(LinmathQuat q) {
	// Box-Muller pairs to get 4 independent normals
	for (int i = 0; i < 4; i++) {
		q[i] = linmath_normrand(0, 1);
	}
	quatnormalize(q, q);
}

// Generate a random non-zero quaternion (not necessarily unit)
static void rand_quat(LinmathQuat q) {
	for (int i = 0; i < 4; i++) {
		q[i] = rand_range(-10.0, 10.0);
	}
	// Ensure non-zero
	if (quatmagnitude(q) < 1e-8) {
		q[0] = 1.0;
	}
}

// Generate a random 3D point with bounded components
static void rand_point(LinmathPoint3d p) {
	p[0] = rand_range(-100.0, 100.0);
	p[1] = rand_range(-100.0, 100.0);
	p[2] = rand_range(-100.0, 100.0);
}

// Generate a random pose (unit quaternion, bounded position)
static void rand_pose(LinmathPose *pose) {
	rand_point(pose->Pos);
	rand_unit_quat(pose->Rot);
}

// ── Property Tests ──────────────────────────────────────────────────

// 1. quatnormalize always produces |q| ≈ 1.0
TEST(QuatProps, NormalizeProducesUnit) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	for (int i = 0; i < N_TRIALS; i++) {
		LinmathQuat q, qn;
		rand_quat(q);
		quatnormalize(qn, q);
		FLT mag = quatmagnitude(qn);

		if (fabs(mag - 1.0) > QUAT_TOL) {
			fprintf(stderr, "NormalizeProducesUnit FAILED (seed=%u, trial=%d)\n", seed, i);
			fprintf(stderr, "  input:  [%.10f, %.10f, %.10f, %.10f]\n", q[0], q[1], q[2], q[3]);
			fprintf(stderr, "  output: [%.10f, %.10f, %.10f, %.10f]\n", qn[0], qn[1], qn[2], qn[3]);
			fprintf(stderr, "  |qn| = %.15f\n", mag);
			return -1;
		}
	}
	return 0;
}

// 2. quatgetconjugate is an involution: conj(conj(q)) == q
TEST(QuatProps, ConjugateInvolution) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	for (int i = 0; i < N_TRIALS; i++) {
		LinmathQuat q, c1, c2;
		rand_quat(q);
		quatgetconjugate(c1, q);
		quatgetconjugate(c2, c1);

		for (int j = 0; j < 4; j++) {
			if (fabs(c2[j] - q[j]) > QUAT_TOL) {
				fprintf(stderr, "ConjugateInvolution FAILED (seed=%u, trial=%d)\n", seed, i);
				fprintf(stderr, "  q:           [%.10f, %.10f, %.10f, %.10f]\n", q[0], q[1], q[2], q[3]);
				fprintf(stderr, "  conj(conj):  [%.10f, %.10f, %.10f, %.10f]\n", c2[0], c2[1], c2[2], c2[3]);
				return -1;
			}
		}
	}
	return 0;
}

// 3. q * q^-1 ≈ identity [1,0,0,0]
TEST(QuatProps, MultiplyByInverseIsIdentity) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	for (int i = 0; i < N_TRIALS; i++) {
		LinmathQuat q, qinv, result;
		rand_unit_quat(q);
		quatgetreciprocal(qinv, q);
		quatrotateabout(result, q, qinv);

		// Normalize sign: identity can be [1,0,0,0] or [-1,0,0,0]
		FLT sign = result[0] < 0 ? -1.0 : 1.0;
		if (fabs(result[0] * sign - 1.0) > QUAT_TOL ||
			fabs(result[1] * sign) > QUAT_TOL ||
			fabs(result[2] * sign) > QUAT_TOL ||
			fabs(result[3] * sign) > QUAT_TOL) {
			fprintf(stderr, "MultiplyByInverseIsIdentity FAILED (seed=%u, trial=%d)\n", seed, i);
			fprintf(stderr, "  q:      [%.10f, %.10f, %.10f, %.10f]\n", q[0], q[1], q[2], q[3]);
			fprintf(stderr, "  q*q^-1: [%.10f, %.10f, %.10f, %.10f]\n", result[0], result[1], result[2], result[3]);
			return -1;
		}
	}
	return 0;
}

// 4. rotate then unrotate recovers original vector: R(q^-1, R(q, v)) ≈ v
TEST(QuatProps, RotateUnrotateIdentity) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	for (int i = 0; i < N_TRIALS; i++) {
		LinmathQuat q, qinv;
		LinmathPoint3d v, rotated, recovered;

		rand_unit_quat(q);
		quatgetconjugate(qinv, q); // For unit quaternion, conjugate == inverse
		rand_point(v);

		quatrotatevector(rotated, q, v);
		quatrotatevector(recovered, qinv, rotated);

		for (int j = 0; j < 3; j++) {
			if (fabs(recovered[j] - v[j]) > POSE_TOL) {
				fprintf(stderr, "RotateUnrotateIdentity FAILED (seed=%u, trial=%d)\n", seed, i);
				fprintf(stderr, "  q: [%.10f, %.10f, %.10f, %.10f]\n", q[0], q[1], q[2], q[3]);
				fprintf(stderr, "  v: [%.10f, %.10f, %.10f]\n", v[0], v[1], v[2]);
				fprintf(stderr, "  recovered: [%.10f, %.10f, %.10f]\n", recovered[0], recovered[1], recovered[2]);
				return -1;
			}
		}
	}
	return 0;
}

// 5. Rotation preserves vector magnitude: |R(q, v)| ≈ |v|
TEST(QuatProps, RotationPreservesMagnitude) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	for (int i = 0; i < N_TRIALS; i++) {
		LinmathQuat q;
		LinmathPoint3d v, rotated;

		rand_unit_quat(q);
		rand_point(v);
		quatrotatevector(rotated, q, v);

		FLT mag_v = magnitude3d(v);
		FLT mag_r = magnitude3d(rotated);

		if (fabs(mag_v - mag_r) > POSE_TOL) {
			fprintf(stderr, "RotationPreservesMagnitude FAILED (seed=%u, trial=%d)\n", seed, i);
			fprintf(stderr, "  q: [%.10f, %.10f, %.10f, %.10f]\n", q[0], q[1], q[2], q[3]);
			fprintf(stderr, "  |v| = %.15f, |R(q,v)| = %.15f\n", mag_v, mag_r);
			return -1;
		}
	}
	return 0;
}

// 6. ApplyPose(InvertPose(p), ApplyPose(p, pt)) ≈ pt
TEST(QuatProps, PoseInvertRoundtrip) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	for (int i = 0; i < N_TRIALS; i++) {
		LinmathPose pose, inv;
		LinmathPoint3d pt, transformed, recovered;

		rand_pose(&pose);
		rand_point(pt);
		InvertPose(&inv, &pose);

		ApplyPoseToPoint(transformed, &pose, pt);
		ApplyPoseToPoint(recovered, &inv, transformed);

		for (int j = 0; j < 3; j++) {
			if (fabs(recovered[j] - pt[j]) > POSE_TOL) {
				fprintf(stderr, "PoseInvertRoundtrip FAILED (seed=%u, trial=%d)\n", seed, i);
				fprintf(stderr, "  pose.Pos: [%.6f, %.6f, %.6f]\n", pose.Pos[0], pose.Pos[1], pose.Pos[2]);
				fprintf(stderr, "  pose.Rot: [%.6f, %.6f, %.6f, %.6f]\n", pose.Rot[0], pose.Rot[1], pose.Rot[2], pose.Rot[3]);
				fprintf(stderr, "  pt:        [%.6f, %.6f, %.6f]\n", pt[0], pt[1], pt[2]);
				fprintf(stderr, "  recovered: [%.6f, %.6f, %.6f]\n", recovered[0], recovered[1], recovered[2]);
				return -1;
			}
		}
	}
	return 0;
}

// 7. ApplyPoseToPose(p, InvertPose(p)) ≈ identity pose
TEST(QuatProps, PoseComposeInverseIsIdentity) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	for (int i = 0; i < N_TRIALS; i++) {
		LinmathPose pose, inv, result;
		rand_pose(&pose);
		InvertPose(&inv, &pose);
		ApplyPoseToPose(&result, &pose, &inv);

		// Position should be near zero
		FLT pos_err = magnitude3d(result.Pos);
		// Rotation should be near identity [1,0,0,0]
		FLT sign = result.Rot[0] < 0 ? -1.0 : 1.0;
		FLT rot_err = fabs(result.Rot[0] * sign - 1.0) +
					  fabs(result.Rot[1] * sign) +
					  fabs(result.Rot[2] * sign) +
					  fabs(result.Rot[3] * sign);

		if (pos_err > POSE_TOL || rot_err > QUAT_TOL) {
			fprintf(stderr, "PoseComposeInverseIsIdentity FAILED (seed=%u, trial=%d)\n", seed, i);
			fprintf(stderr, "  result.Pos: [%.10f, %.10f, %.10f] (err=%.10f)\n",
					result.Pos[0], result.Pos[1], result.Pos[2], pos_err);
			fprintf(stderr, "  result.Rot: [%.10f, %.10f, %.10f, %.10f] (err=%.10f)\n",
					result.Rot[0], result.Rot[1], result.Rot[2], result.Rot[3], rot_err);
			return -1;
		}
	}
	return 0;
}

// 8. quattomatrix33 → quatfrommatrix33 roundtrip: recovers ±q
TEST(QuatProps, MatrixRoundtrip) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	for (int i = 0; i < N_TRIALS; i++) {
		LinmathQuat q, recovered;
		FLT mat[9];

		rand_unit_quat(q);
		quattomatrix33(mat, q);
		quatfrommatrix33(recovered, mat);

		// recovered ≈ ±q (sign ambiguity is inherent to quat ↔ matrix)
		FLT sign = (quatinnerproduct(q, recovered) < 0) ? -1.0 : 1.0;

		for (int j = 0; j < 4; j++) {
			if (fabs(recovered[j] * sign - q[j]) > QUAT_TOL) {
				fprintf(stderr, "MatrixRoundtrip FAILED (seed=%u, trial=%d)\n", seed, i);
				fprintf(stderr, "  q:         [%.10f, %.10f, %.10f, %.10f]\n", q[0], q[1], q[2], q[3]);
				fprintf(stderr, "  recovered: [%.10f, %.10f, %.10f, %.10f]\n",
						recovered[0], recovered[1], recovered[2], recovered[3]);
				return -1;
			}
		}
	}
	return 0;
}

// 9. quatfromaxisanglemag → quattoaxisanglemag roundtrip
TEST(QuatProps, AxisAngleRoundtrip) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	for (int i = 0; i < N_TRIALS; i++) {
		// Generate axis-angle with magnitude < PI to avoid wrapping ambiguity
		LinmathAxisAngleMag aa_in, aa_out;
		LinmathQuat q;

		// Random axis direction, random angle in (0, PI)
		LinmathPoint3d axis;
		rand_point(axis);
		FLT axis_mag = magnitude3d(axis);
		if (axis_mag < 1e-8) {
			axis[0] = 1.0;
			axis_mag = 1.0;
		}
		FLT angle = rand_range(0.01, LINMATHPI - 0.01); // Avoid 0 and PI boundaries
		aa_in[0] = axis[0] / axis_mag * angle;
		aa_in[1] = axis[1] / axis_mag * angle;
		aa_in[2] = axis[2] / axis_mag * angle;

		quatfromaxisanglemag(q, aa_in);
		quattoaxisanglemag(aa_out, q);

		for (int j = 0; j < 3; j++) {
			if (fabs(aa_out[j] - aa_in[j]) > QUAT_TOL) {
				fprintf(stderr, "AxisAngleRoundtrip FAILED (seed=%u, trial=%d)\n", seed, i);
				fprintf(stderr, "  aa_in:  [%.10f, %.10f, %.10f]\n", aa_in[0], aa_in[1], aa_in[2]);
				fprintf(stderr, "  aa_out: [%.10f, %.10f, %.10f]\n", aa_out[0], aa_out[1], aa_out[2]);
				return -1;
			}
		}
	}
	return 0;
}

// 10. Quaternion multiplication is associative: (a*b)*c ≈ a*(b*c)
TEST(QuatProps, MultiplicationAssociative) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	for (int i = 0; i < N_TRIALS; i++) {
		LinmathQuat a, b, c;
		LinmathQuat ab, ab_c, bc, a_bc;

		rand_unit_quat(a);
		rand_unit_quat(b);
		rand_unit_quat(c);

		quatrotateabout(ab, a, b);
		quatrotateabout(ab_c, ab, c);

		quatrotateabout(bc, b, c);
		quatrotateabout(a_bc, a, bc);

		// Compare (a*b)*c vs a*(b*c) — account for sign ambiguity
		FLT sign = (quatinnerproduct(ab_c, a_bc) < 0) ? -1.0 : 1.0;

		for (int j = 0; j < 4; j++) {
			if (fabs(ab_c[j] - a_bc[j] * sign) > QUAT_TOL) {
				fprintf(stderr, "MultiplicationAssociative FAILED (seed=%u, trial=%d)\n", seed, i);
				fprintf(stderr, "  (a*b)*c: [%.10f, %.10f, %.10f, %.10f]\n", ab_c[0], ab_c[1], ab_c[2], ab_c[3]);
				fprintf(stderr, "  a*(b*c): [%.10f, %.10f, %.10f, %.10f]\n", a_bc[0], a_bc[1], a_bc[2], a_bc[3]);
				return -1;
			}
		}
	}
	return 0;
}

// 11. Quaternion slerp boundaries: slerp(a, b, 0) ≈ a, slerp(a, b, 1) ≈ b
TEST(QuatProps, SlerpBoundaries) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	for (int i = 0; i < N_TRIALS; i++) {
		LinmathQuat a, b, s0, s1;
		rand_unit_quat(a);
		rand_unit_quat(b);

		quatslerp(s0, a, b, 0.0);
		quatslerp(s1, a, b, 1.0);

		// slerp(a, b, 0) ≈ a
		FLT sign0 = (quatinnerproduct(s0, a) < 0) ? -1.0 : 1.0;
		for (int j = 0; j < 4; j++) {
			if (fabs(s0[j] * sign0 - a[j]) > QUAT_TOL) {
				fprintf(stderr, "SlerpBoundaries t=0 FAILED (seed=%u, trial=%d)\n", seed, i);
				fprintf(stderr, "  a:        [%.10f, %.10f, %.10f, %.10f]\n", a[0], a[1], a[2], a[3]);
				fprintf(stderr, "  slerp(0): [%.10f, %.10f, %.10f, %.10f]\n", s0[0], s0[1], s0[2], s0[3]);
				return -1;
			}
		}

		// slerp(a, b, 1) ≈ b
		FLT sign1 = (quatinnerproduct(s1, b) < 0) ? -1.0 : 1.0;
		for (int j = 0; j < 4; j++) {
			if (fabs(s1[j] * sign1 - b[j]) > QUAT_TOL) {
				fprintf(stderr, "SlerpBoundaries t=1 FAILED (seed=%u, trial=%d)\n", seed, i);
				fprintf(stderr, "  b:        [%.10f, %.10f, %.10f, %.10f]\n", b[0], b[1], b[2], b[3]);
				fprintf(stderr, "  slerp(1): [%.10f, %.10f, %.10f, %.10f]\n", s1[0], s1[1], s1[2], s1[3]);
				return -1;
			}
		}
	}
	return 0;
}

// 12. Slerp output is always a unit quaternion
TEST(QuatProps, SlerpProducesUnit) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	for (int i = 0; i < N_TRIALS; i++) {
		LinmathQuat a, b, result;
		rand_unit_quat(a);
		rand_unit_quat(b);
		FLT t = rand_range(0.0, 1.0);

		quatslerp(result, a, b, t);
		FLT mag = quatmagnitude(result);

		if (fabs(mag - 1.0) > QUAT_TOL) {
			fprintf(stderr, "SlerpProducesUnit FAILED (seed=%u, trial=%d)\n", seed, i);
			fprintf(stderr, "  t = %.10f, |slerp| = %.15f\n", t, mag);
			return -1;
		}
	}
	return 0;
}
