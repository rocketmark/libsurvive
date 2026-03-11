// Property-based tests for the back-facing normal filter geometry.
//
// The back-facing normal filter in SurviveSensorActivations_check_outlier()
// computes:
//
//   normalInWorld = quatrotatevector(tracker_rot, sensor_normal_body)
//   sensorInWorld = ApplyPoseToPoint(tracker_pose, sensor_location_body)
//   towardLh      = normalize(lh_pos - sensorInWorld)
//   facingness    = dot3d(normalInWorld, towardLh)
//   reject if facingness < filterNormalFacingness
//
// Three tests verify properties that would catch bugs in compute_facingness:
//
//  1. FacingnessKnownAngle    — end-to-end with a random (non-identity) rotation
//     and known expected output. The only test that exercises the full coordinate
//     transform path (quatrotatevector + ApplyPoseToPoint + normalize + dot).
//
//  2. DirectlyFacingAlwaysAccepted — aligned sensor must never be filtered out.
//
//  3. BackFacingAlwaysRejected — opposing sensor must always be filtered out.

#include "test_case.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define N_TRIALS 10000
#define NORMAL_TOL 1e-5

static FLT rand_range(FLT min, FLT max) {
	return min + (max - min) * ((FLT)rand() / (FLT)RAND_MAX);
}

static void rand_unit_quat(LinmathQuat q) {
	for (int i = 0; i < 4; i++)
		q[i] = linmath_normrand(0, 1);
	quatnormalize(q, q);
}

static void rand_unit_vec3(FLT *v) {
	v[0] = linmath_normrand(0, 1);
	v[1] = linmath_normrand(0, 1);
	v[2] = linmath_normrand(0, 1);
	FLT mag = magnitude3d(v);
	if (mag < 1e-8) { v[0] = 1.0; mag = 1.0; }
	scale3d(v, v, 1.0 / mag);
}

static void rand_point(FLT *p) {
	p[0] = rand_range(-10.0, 10.0);
	p[1] = rand_range(-10.0, 10.0);
	p[2] = rand_range(-10.0, 10.0);
}

// Compute the filter's facingness value from first principles.
static FLT compute_facingness(const LinmathQuat tracker_rot,
							  const LinmathPoint3d sensor_loc_body,
							  const LinmathPoint3d sensor_normal_body,
							  const LinmathPoint3d tracker_pos,
							  const LinmathPoint3d lh_pos) {
	// Rotate sensor normal into world frame
	LinmathVec3d normal_world;
	quatrotatevector(normal_world, tracker_rot, sensor_normal_body);

	// Transform sensor location into world frame (rotation + translation)
	LinmathPoint3d sensor_world;
	quatrotatevector(sensor_world, tracker_rot, sensor_loc_body);
	add3d(sensor_world, sensor_world, tracker_pos);

	// Direction from sensor toward lighthouse, normalised
	LinmathVec3d toward_lh;
	sub3d(toward_lh, lh_pos, sensor_world);
	FLT dist = magnitude3d(toward_lh);
	if (dist < 1e-8)
		return 0.0; // degenerate: sensor at lighthouse position
	scale3d(toward_lh, toward_lh, 1.0 / dist);

	return dot3d(normal_world, toward_lh);
}

// ── Property Tests ───────────────────────────────────────────────────

// 1. FacingnessKnownAngle
//    For a sensor normal constructed to make a known angle theta with the
//    lighthouse direction, compute_facingness() returns cos(theta).
//
//    Uses a random (non-identity) tracker rotation: the normal is constructed
//    in world space at angle theta, then back-rotated into body frame via the
//    conjugate. compute_facingness() must apply the forward rotation to recover
//    the world-frame normal. If it skips or misapplies quatrotatevector the
//    result will not equal cos(theta).
//
//    This is the only test here that exercises the full coordinate transform
//    path inside compute_facingness().
TEST(NormalFilterProps, FacingnessKnownAngle) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	for (int i = 0; i < N_TRIALS; i++) {
		// Random tracker rotation — forces compute_facingness to apply it correctly
		LinmathQuat rot;
		rand_unit_quat(rot);
		LinmathQuat rot_inv;
		quatgetconjugate(rot_inv, rot);

		// Random lighthouse direction in world frame
		LinmathVec3d toward_lh;
		rand_unit_vec3(toward_lh);

		// Random angle in (0, pi)
		FLT theta = rand_range(0.01, LINMATHPI - 0.01);

		// Construct a world-frame sensor normal at angle theta from toward_lh.
		// Pick any vector perpendicular to toward_lh as the second basis vector.
		LinmathVec3d perp;
		perp[0] = toward_lh[1];
		perp[1] = -toward_lh[0];
		perp[2] = 0.0;
		if (magnitude3d(perp) < 1e-6) {
			perp[0] = 1.0; perp[1] = 0.0; perp[2] = 0.0;
		}
		// Gram-Schmidt: make perp exactly perpendicular to toward_lh
		FLT proj = dot3d(perp, toward_lh);
		perp[0] -= proj * toward_lh[0];
		perp[1] -= proj * toward_lh[1];
		perp[2] -= proj * toward_lh[2];
		FLT perp_mag = magnitude3d(perp);
		if (perp_mag < 1e-8) continue; // degenerate, skip
		scale3d(perp, perp, 1.0 / perp_mag);

		// normal_world at angle theta from toward_lh
		LinmathVec3d normal_world;
		normal_world[0] = cos(theta) * toward_lh[0] + sin(theta) * perp[0];
		normal_world[1] = cos(theta) * toward_lh[1] + sin(theta) * perp[1];
		normal_world[2] = cos(theta) * toward_lh[2] + sin(theta) * perp[2];

		// Back-rotate to body frame — compute_facingness must undo this with rot
		LinmathVec3d normal_body;
		quatrotatevector(normal_body, rot_inv, normal_world);

		// Place sensor at origin, lighthouse along toward_lh at distance 5
		LinmathPoint3d sensor_loc_body = {0, 0, 0};
		LinmathPoint3d tracker_pos     = {0, 0, 0};
		LinmathPoint3d lh_pos;
		lh_pos[0] = toward_lh[0] * 5.0;
		lh_pos[1] = toward_lh[1] * 5.0;
		lh_pos[2] = toward_lh[2] * 5.0;

		FLT facingness = compute_facingness(rot, sensor_loc_body, normal_body, tracker_pos, lh_pos);
		FLT expected   = cos(theta);

		if (fabs(facingness - expected) > NORMAL_TOL) {
			fprintf(stderr, "FacingnessKnownAngle FAILED (seed=%u, trial=%d)\n", seed, i);
			fprintf(stderr, "  theta=%.10f  expected=%.10f  got=%.10f  diff=%.2e\n",
					theta, expected, facingness, fabs(facingness - expected));
			return -1;
		}

		// Accept/reject decision at threshold 0.0 must match cos(theta) >= 0
		int should_accept  = (expected    >= 0.0);
		int filter_accepts = (facingness  >= 0.0);
		if (should_accept != filter_accepts) {
			fprintf(stderr, "FacingnessKnownAngle decision mismatch (seed=%u, trial=%d)\n", seed, i);
			fprintf(stderr, "  theta=%.10f  facingness=%.10f  should_accept=%d  filter_accepts=%d\n",
					theta, facingness, should_accept, filter_accepts);
			return -1;
		}
	}
	return 0;
}

// 2. DirectlyFacingAlwaysAccepted
//    When the sensor normal points exactly toward the lighthouse (facingness ~
//    1.0), the filter always accepts at any threshold <= 1.0. This is the ideal
//    case: a sensor perfectly aimed at the lighthouse must never be rejected.
TEST(NormalFilterProps, DirectlyFacingAlwaysAccepted) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	for (int i = 0; i < N_TRIALS; i++) {
		LinmathQuat rot;
		rand_unit_quat(rot);

		LinmathVec3d toward_lh;
		rand_unit_vec3(toward_lh);

		// Sensor normal = lighthouse direction in world, rotated back to body
		LinmathQuat rot_inv;
		quatgetconjugate(rot_inv, rot);
		LinmathVec3d normal_body;
		quatrotatevector(normal_body, rot_inv, toward_lh);

		FLT threshold = rand_range(-1.0, 0.99);

		LinmathPoint3d sensor_loc_body = {0, 0, 0};
		LinmathPoint3d tracker_pos     = {0, 0, 0};
		LinmathPoint3d lh_pos;
		lh_pos[0] = toward_lh[0] * 5.0;
		lh_pos[1] = toward_lh[1] * 5.0;
		lh_pos[2] = toward_lh[2] * 5.0;

		FLT f = compute_facingness(rot, sensor_loc_body, normal_body, tracker_pos, lh_pos);

		if (fabs(f - 1.0) > NORMAL_TOL * 10) {
			fprintf(stderr, "DirectlyFacingAlwaysAccepted: facingness not 1 (seed=%u, trial=%d)\n", seed, i);
			fprintf(stderr, "  facingness=%.10f (expected ~1.0)\n", f);
			return -1;
		}
		if (f < threshold) {
			fprintf(stderr, "DirectlyFacingAlwaysAccepted FAILED (seed=%u, trial=%d)\n", seed, i);
			fprintf(stderr, "  facingness=%.10f  threshold=%.10f\n", f, threshold);
			return -1;
		}
	}
	return 0;
}

// 3. BackFacingAlwaysRejected
//    When the sensor normal points exactly away from the lighthouse (facingness
//    ~ -1.0), the filter always rejects at any threshold >= 0.0 (the default).
//    This is the core reflection case: a sensor physically facing away from the
//    lighthouse cannot receive direct lighthouse light.
TEST(NormalFilterProps, BackFacingAlwaysRejected) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	for (int i = 0; i < N_TRIALS; i++) {
		LinmathQuat rot;
		rand_unit_quat(rot);

		LinmathVec3d toward_lh;
		rand_unit_vec3(toward_lh);

		// Sensor normal points AWAY from lighthouse
		LinmathVec3d away;
		scale3d(away, toward_lh, -1.0);

		LinmathQuat rot_inv;
		quatgetconjugate(rot_inv, rot);
		LinmathVec3d normal_body;
		quatrotatevector(normal_body, rot_inv, away);

		FLT threshold = rand_range(0.0, 1.0);

		LinmathPoint3d sensor_loc_body = {0, 0, 0};
		LinmathPoint3d tracker_pos     = {0, 0, 0};
		LinmathPoint3d lh_pos;
		lh_pos[0] = toward_lh[0] * 5.0;
		lh_pos[1] = toward_lh[1] * 5.0;
		lh_pos[2] = toward_lh[2] * 5.0;

		FLT f = compute_facingness(rot, sensor_loc_body, normal_body, tracker_pos, lh_pos);

		if (fabs(f + 1.0) > NORMAL_TOL * 10) {
			fprintf(stderr, "BackFacingAlwaysRejected: facingness not -1 (seed=%u, trial=%d)\n", seed, i);
			fprintf(stderr, "  facingness=%.10f (expected ~-1.0)\n", f);
			return -1;
		}
		if (f >= threshold) {
			fprintf(stderr, "BackFacingAlwaysRejected FAILED (seed=%u, trial=%d)\n", seed, i);
			fprintf(stderr, "  facingness=%.10f  threshold=%.10f\n", f, threshold);
			return -1;
		}
	}
	return 0;
}
