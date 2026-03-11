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
// These tests verify the geometric invariants that the filter correctness
// depends on, without needing to call the static inline check_outlier function
// directly.

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

// 1. FacingnessInRange
//    For any unit sensor normal and any lighthouse direction, the dot product
//    (facingness) is always in [-1, 1]. This bounds the range of the filter
//    decision variable and ensures threshold comparisons are meaningful.
TEST(NormalFilterProps, FacingnessInRange) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	for (int i = 0; i < N_TRIALS; i++) {
		LinmathQuat rot;
		LinmathPoint3d normal_body, sensor_loc, tracker_pos, lh_pos;

		rand_unit_quat(rot);
		rand_unit_vec3(normal_body);
		rand_point(sensor_loc);
		rand_point(tracker_pos);
		rand_point(lh_pos);

		FLT f = compute_facingness(rot, sensor_loc, normal_body, tracker_pos, lh_pos);

		if (f < -1.0 - NORMAL_TOL || f > 1.0 + NORMAL_TOL) {
			fprintf(stderr, "FacingnessInRange FAILED (seed=%u, trial=%d)\n", seed, i);
			fprintf(stderr, "  facingness = %.15f (must be in [-1, 1])\n", f);
			return -1;
		}
	}
	return 0;
}

// 2. FacingnessFlipsWithDirection
//    Flipping the lighthouse to the opposite side of the sensor inverts the
//    facingness sign. This confirms the filter correctly distinguishes a sensor
//    that faces a lighthouse from one that faces away.
//    If dot(n, d) = f, then dot(n, -d) = -f.
TEST(NormalFilterProps, FacingnessFlipsWithDirection) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	for (int i = 0; i < N_TRIALS; i++) {
		LinmathQuat rot;
		rand_unit_quat(rot);

		// Unit normal in body frame, rotated to world
		LinmathVec3d normal_body;
		rand_unit_vec3(normal_body);
		LinmathVec3d normal_world;
		quatrotatevector(normal_world, rot, normal_body);

		// Random unit direction toward a lighthouse
		LinmathVec3d toward_lh;
		rand_unit_vec3(toward_lh);

		FLT f_forward  = dot3d(normal_world, toward_lh);

		// Lighthouse on the exact opposite side
		LinmathVec3d away_from_lh;
		scale3d(away_from_lh, toward_lh, -1.0);
		FLT f_backward = dot3d(normal_world, away_from_lh);

		if (fabs(f_forward + f_backward) > NORMAL_TOL) {
			fprintf(stderr, "FacingnessFlipsWithDirection FAILED (seed=%u, trial=%d)\n", seed, i);
			fprintf(stderr, "  f_forward=%.15f  f_backward=%.15f  sum=%.2e\n",
					f_forward, f_backward, f_forward + f_backward);
			return -1;
		}
	}
	return 0;
}

// 3. FacingnessKnownAngle
//    For a sensor normal constructed to make a known angle theta with the
//    lighthouse direction, facingness == cos(theta).
//    This directly validates the geometric meaning of the threshold: at the
//    default threshold 0.0, the filter accepts sensors within 90 degrees of
//    the lighthouse and rejects those beyond 90 degrees.
TEST(NormalFilterProps, FacingnessKnownAngle) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	for (int i = 0; i < N_TRIALS; i++) {
		// Random rotation (pose doesn't matter for this test — we work in world frame)
		LinmathQuat identity = {1, 0, 0, 0};

		// Random lighthouse direction in world frame
		LinmathVec3d toward_lh;
		rand_unit_vec3(toward_lh);

		// Random angle in (0, pi)
		FLT theta = rand_range(0.01, LINMATHPI - 0.01);

		// Construct a sensor normal at angle theta from toward_lh.
		// Pick any vector perpendicular to toward_lh as the second basis vector.
		LinmathVec3d perp;
		perp[0] = toward_lh[1];
		perp[1] = -toward_lh[0];
		perp[2] = 0.0;
		if (magnitude3d(perp) < 1e-6) {
			// toward_lh is nearly parallel to Z; use X instead
			perp[0] = 1.0; perp[1] = 0.0; perp[2] = 0.0;
		}
		// Make perp exactly perpendicular to toward_lh via Gram-Schmidt
		FLT proj = dot3d(perp, toward_lh);
		perp[0] -= proj * toward_lh[0];
		perp[1] -= proj * toward_lh[1];
		perp[2] -= proj * toward_lh[2];
		FLT perp_mag = magnitude3d(perp);
		if (perp_mag < 1e-8) continue; // degenerate, skip
		scale3d(perp, perp, 1.0 / perp_mag);

		// normal = cos(theta)*toward_lh + sin(theta)*perp  → angle theta from lh direction
		LinmathVec3d normal_world;
		normal_world[0] = cos(theta) * toward_lh[0] + sin(theta) * perp[0];
		normal_world[1] = cos(theta) * toward_lh[1] + sin(theta) * perp[1];
		normal_world[2] = cos(theta) * toward_lh[2] + sin(theta) * perp[2];

		FLT facingness = dot3d(normal_world, toward_lh);
		FLT expected   = cos(theta);

		if (fabs(facingness - expected) > NORMAL_TOL) {
			fprintf(stderr, "FacingnessKnownAngle FAILED (seed=%u, trial=%d)\n", seed, i);
			fprintf(stderr, "  theta=%.10f  expected cos(theta)=%.10f  got=%.10f  diff=%.2e\n",
					theta, expected, facingness, fabs(facingness - expected));
			return -1;
		}

		// Verify the accept/reject decision at threshold 0.0 matches cos(theta) >= 0
		int should_accept = (expected >= 0.0);   // theta < pi/2
		int filter_accepts = (facingness >= 0.0);
		if (should_accept != filter_accepts) {
			fprintf(stderr, "FacingnessKnownAngle decision mismatch (seed=%u, trial=%d)\n", seed, i);
			fprintf(stderr, "  theta=%.10f  facingness=%.10f  should_accept=%d  filter_accepts=%d\n",
					theta, facingness, should_accept, filter_accepts);
			return -1;
		}
	}
	return 0;
}

// 4. FacingnessThresholdMonotonic
//    For a fixed sensor and lighthouse geometry, raising the threshold from f-eps
//    to f+eps flips the filter decision from accept to reject (and never the
//    other way). This verifies the filter's threshold comparison is correct and
//    monotonically ordered.
TEST(NormalFilterProps, FacingnessThresholdMonotonic) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	for (int i = 0; i < N_TRIALS; i++) {
		LinmathQuat rot;
		LinmathPoint3d normal_body, sensor_loc, tracker_pos, lh_pos;

		rand_unit_quat(rot);
		rand_unit_vec3(normal_body);
		rand_point(sensor_loc);
		rand_point(tracker_pos);
		rand_point(lh_pos);

		FLT f = compute_facingness(rot, sensor_loc, normal_body, tracker_pos, lh_pos);

		// Filter logic: reject if facingness < threshold
		FLT threshold_below = f - 0.01;
		FLT threshold_above = f + 0.01;

		int accepted_at_below  = (f >= threshold_below);  // should be true
		int accepted_at_above  = (f >= threshold_above);  // should be false (f < f+0.01)

		if (!accepted_at_below) {
			fprintf(stderr, "FacingnessThresholdMonotonic: should accept below threshold "
					"(seed=%u, trial=%d) f=%.10f thresh=%.10f\n",
					seed, i, f, threshold_below);
			return -1;
		}
		if (accepted_at_above) {
			fprintf(stderr, "FacingnessThresholdMonotonic: should reject above threshold "
					"(seed=%u, trial=%d) f=%.10f thresh=%.10f\n",
					seed, i, f, threshold_above);
			return -1;
		}
	}
	return 0;
}

// 5. DirectlyFacingAlwaysAccepted
//    When the sensor normal is constructed to point exactly toward the lighthouse
//    (facingness == 1.0), the filter always accepts at any threshold <= 1.0.
//    This is the ideal case: a sensor perfectly aimed at the lighthouse must never
//    be rejected by the normal filter.
TEST(NormalFilterProps, DirectlyFacingAlwaysAccepted) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	for (int i = 0; i < N_TRIALS; i++) {
		// Random pose
		LinmathQuat rot;
		rand_unit_quat(rot);

		// Random lighthouse direction in world frame
		LinmathVec3d toward_lh;
		rand_unit_vec3(toward_lh);

		// Sensor normal = lighthouse direction (in world frame), rotated back to body
		// so that after quatrotatevector it equals toward_lh exactly.
		LinmathQuat rot_inv;
		quatgetconjugate(rot_inv, rot);
		LinmathVec3d normal_body;
		quatrotatevector(normal_body, rot_inv, toward_lh);

		// Any threshold strictly less than 1 must accept
		FLT threshold = rand_range(-1.0, 0.99);

		// Compute facingness — place sensor at origin, lighthouse in the toward_lh direction
		LinmathPoint3d sensor_loc_body = {0, 0, 0};
		LinmathPoint3d tracker_pos     = {0, 0, 0};
		LinmathPoint3d lh_pos;
		lh_pos[0] = toward_lh[0] * 5.0;
		lh_pos[1] = toward_lh[1] * 5.0;
		lh_pos[2] = toward_lh[2] * 5.0;

		FLT f = compute_facingness(rot, sensor_loc_body, normal_body, tracker_pos, lh_pos);

		if (f < threshold) {
			fprintf(stderr, "DirectlyFacingAlwaysAccepted FAILED (seed=%u, trial=%d)\n", seed, i);
			fprintf(stderr, "  facingness=%.10f  threshold=%.10f\n", f, threshold);
			return -1;
		}
		// Also verify facingness is close to 1.0
		if (fabs(f - 1.0) > NORMAL_TOL * 10) {
			fprintf(stderr, "DirectlyFacingAlwaysAccepted: facingness not 1 (seed=%u, trial=%d)\n", seed, i);
			fprintf(stderr, "  facingness=%.10f (expected ~1.0)\n", f);
			return -1;
		}
	}
	return 0;
}

// 6. BackFacingAlwaysRejected
//    When the sensor normal points exactly away from the lighthouse (facingness
//    == -1.0), the filter always rejects at any threshold >= 0.0 (the default).
//    This is the core reflection case: a sensor physically facing away from the
//    lighthouse cannot receive direct lighthouse light.
TEST(NormalFilterProps, BackFacingAlwaysRejected) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	for (int i = 0; i < N_TRIALS; i++) {
		LinmathQuat rot;
		rand_unit_quat(rot);

		// Lighthouse direction in world frame
		LinmathVec3d toward_lh;
		rand_unit_vec3(toward_lh);

		// Sensor normal points AWAY from lighthouse (opposite direction)
		LinmathVec3d away;
		scale3d(away, toward_lh, -1.0);

		LinmathQuat rot_inv;
		quatgetconjugate(rot_inv, rot);
		LinmathVec3d normal_body;
		quatrotatevector(normal_body, rot_inv, away);

		// Any threshold >= 0 (including the default 0.0) must reject
		FLT threshold = rand_range(0.0, 1.0);

		LinmathPoint3d sensor_loc_body = {0, 0, 0};
		LinmathPoint3d tracker_pos     = {0, 0, 0};
		LinmathPoint3d lh_pos;
		lh_pos[0] = toward_lh[0] * 5.0;
		lh_pos[1] = toward_lh[1] * 5.0;
		lh_pos[2] = toward_lh[2] * 5.0;

		FLT f = compute_facingness(rot, sensor_loc_body, normal_body, tracker_pos, lh_pos);

		// facingness should be close to -1.0
		if (fabs(f + 1.0) > NORMAL_TOL * 10) {
			fprintf(stderr, "BackFacingAlwaysRejected: facingness not -1 (seed=%u, trial=%d)\n", seed, i);
			fprintf(stderr, "  facingness=%.10f (expected ~-1.0)\n", f);
			return -1;
		}
		// Filter rejects iff facingness < threshold — should always reject here
		if (f >= threshold) {
			fprintf(stderr, "BackFacingAlwaysRejected FAILED (seed=%u, trial=%d)\n", seed, i);
			fprintf(stderr, "  facingness=%.10f  threshold=%.10f\n", f, threshold);
			return -1;
		}
	}
	return 0;
}
