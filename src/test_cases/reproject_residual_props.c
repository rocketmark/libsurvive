// Property-based tests for the reproject → BSVD solve → residual roundtrip.
//
// Validates that:
// 1. Clean roundtrip recovers original pose
// 2. Per-sensor reprojection residuals are near zero for clean data
// 3. Single-sensor corruption (simulating LH reflections) is detectable
//    via residual analysis
// 4. BSVD is reasonably robust to a single outlier
// 5. Multiple outliers degrade gracefully
// 6. Max-residual sensor identifies the corrupted sensor

#include "../barycentric_svd/barycentric_svd.h"
#include "survive_reproject.h"
#include "test_case.h"

#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

// ── Constants ────────────────────────────────────────────────────────

#define N_TRIALS      1000
#define N_SENSORS     12
#define SENSOR_RADIUS 0.05    // 5cm sphere (tracker-sized)
#define POSE_TOL      0.02    // 2cm + ~1 deg combined tolerance
#define RESIDUAL_TOL  1e-4    // Near-zero for clean solve
#define CORRUPT_MIN   0.05    // Min reflection corruption (radians)
#define CORRUPT_MAX   0.30    // Max reflection corruption (radians)
#define OUTLIER_RATIO 5.0     // Corrupted residual must be 5x max clean

// ── Helpers ──────────────────────────────────────────────────────────

static FLT rand_range(FLT min, FLT max) {
	return min + (max - min) * ((FLT)rand() / (FLT)RAND_MAX);
}

static void rand_unit_quat(LinmathQuat q) {
	for (int i = 0; i < 4; i++)
		q[i] = linmath_normrand(0, 1);
	quatnormalize(q, q);
}

// Golden-angle spiral: deterministic, well-distributed on sphere
static void generate_sensor_positions(LinmathPoint3d *sensors, int n) {
	FLT golden_angle = M_PI * (3.0 - sqrt(5.0));
	for (int i = 0; i < n; i++) {
		FLT theta = golden_angle * i;
		FLT z = 1.0 - (2.0 * i + 1.0) / n;
		FLT r = sqrt(1.0 - z * z);
		sensors[i][0] = SENSOR_RADIUS * r * cos(theta);
		sensors[i][1] = SENSOR_RADIUS * r * sin(theta);
		sensors[i][2] = SENSOR_RADIUS * z;
	}
}

// Random tracker pose at VP stage distance (1-4m from lighthouse)
static void rand_vp_pose(SurvivePose *pose) {
	pose->Pos[0] = rand_range(-1.5, 1.5);
	pose->Pos[1] = rand_range(-1.5, 1.5);
	pose->Pos[2] = rand_range(-4.0, -1.0);
	rand_unit_quat(pose->Rot);
}

// Filter sensors visible to lighthouse (z < 0 in LH frame)
static int filter_visible(const LinmathPoint3d *pts_lh, int n, int *out) {
	int count = 0;
	for (int i = 0; i < n; i++) {
		if (pts_lh[i][2] < -0.01)
			out[count++] = i;
	}
	return count;
}

// Reproject all sensors through a pose with zero calibration
static void reproject_sensors(const BaseStationCal *cal, const SurvivePose *obj2lh,
							  const LinmathPoint3d *sensors, int n,
							  SurviveAngleReading *angles, LinmathPoint3d *pts_lh) {
	for (int i = 0; i < n; i++) {
		ApplyPoseToPoint(pts_lh[i], obj2lh, sensors[i]);
		survive_reproject_xy(cal, pts_lh[i], angles[i]);
	}
}

// BSVD fill_M callback for Gen1
static void fill_m_gen1(void *user, FLT *eq, int axis, FLT angle) {
	(void)user;
	FLT sv = sin(angle), cv = cos(angle);
	switch (axis) {
	case 0: eq[0] = cv; eq[1] = 0;  eq[2] = -sv; break;
	case 1: eq[0] = 0;  eq[1] = cv; eq[2] = -sv; break;
	}
}

// Solve pose from angles using BSVD. Returns BSVD error metric.
static FLT solve_pose_bsvd(const LinmathPoint3d *sensors, int n_sensors,
						   const int *visible, int n_vis,
						   const SurviveAngleReading *vis_angles,
						   SurvivePose *out) {
	bc_svd bc = {0};
	bc_svd_bc_svd(&bc, NULL, fill_m_gen1, (LinmathPoint3d *)sensors, n_sensors);
	bc_svd_reset_correspondences(&bc);

	for (int i = 0; i < n_vis; i++)
		bc_svd_add_correspondence(&bc, visible[i], vis_angles[i][0], vis_angles[i][1]);

	FLT R[3][3];
	FLT err = bc_svd_compute_pose(&bc, R, out->Pos);

	// Post-process: Z-flip (matching barycentric_svd.c:97-116)
	LinmathQuat tmp;
	quatfrommatrix33(tmp, (const FLT *)R);
	const LinmathQuat rt = {0, 0, 1, 0};
	quatrotateabout(out->Rot, rt, tmp);
	out->Pos[0] = -out->Pos[0];
	out->Pos[2] = -out->Pos[2];

	if (out->Rot[0] < 0)
		scalend(out->Rot, out->Rot, -1, 4);

	bc_svd_dtor(&bc);
	return err;
}

// Compute per-sensor reprojection residuals
static void compute_residuals(const BaseStationCal *cal, const SurvivePose *pose,
							  const LinmathPoint3d *sensors, const int *visible,
							  int n_vis, const SurviveAngleReading *input_angles,
							  FLT *residuals) {
	for (int i = 0; i < n_vis; i++) {
		LinmathPoint3d ptInLH;
		ApplyPoseToPoint(ptInLH, pose, sensors[visible[i]]);
		SurviveAngleReading reproj;
		survive_reproject_xy(cal, ptInLH, reproj);
		FLT dx = reproj[0] - input_angles[i][0];
		FLT dy = reproj[1] - input_angles[i][1];
		residuals[i] = sqrt(dx * dx + dy * dy);
	}
}

// Combined pose distance: position L2 + angular distance
static FLT pose_distance(const SurvivePose *a, const SurvivePose *b) {
	FLT pos2 = 0;
	for (int i = 0; i < 3; i++)
		pos2 += (a->Pos[i] - b->Pos[i]) * (a->Pos[i] - b->Pos[i]);

	FLT dot = fabs(quatinnerproduct(a->Rot, b->Rot));
	if (dot > 1.0) dot = 1.0;
	return sqrt(pos2) + 2.0 * acos(dot);
}

// Common setup: generate pose, reproject, filter visible, collect angles.
// Returns n_vis (0 if not enough visible sensors).
static int setup_trial(const BaseStationCal *cal, const LinmathPoint3d *sensors,
					   int min_visible, SurvivePose *obj2lh,
					   SurviveAngleReading *all_angles, LinmathPoint3d *pts_lh,
					   int *visible, SurviveAngleReading *vis_angles) {
	rand_vp_pose(obj2lh);
	reproject_sensors(cal, obj2lh, sensors, N_SENSORS, all_angles, pts_lh);
	int n_vis = filter_visible(pts_lh, N_SENSORS, visible);
	if (n_vis < min_visible)
		return 0;
	for (int i = 0; i < n_vis; i++)
		memcpy(vis_angles[i], all_angles[visible[i]], sizeof(SurviveAngleReading));
	return n_vis;
}

// ── Tests ────────────────────────────────────────────────────────────

// 1. Clean roundtrip: reproject → BSVD solve → compare
TEST(ReprojectResidualProps, CleanRoundtrip) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	BaseStationCal cal[2] = {0};
	LinmathPoint3d sensors[N_SENSORS];
	generate_sensor_positions(sensors, N_SENSORS);

	for (int t = 0; t < N_TRIALS; t++) {
		SurvivePose obj2lh;
		SurviveAngleReading all_angles[N_SENSORS], vis_angles[N_SENSORS];
		LinmathPoint3d pts_lh[N_SENSORS];
		int visible[N_SENSORS];

		int n_vis = setup_trial(cal, sensors, 6, &obj2lh, all_angles, pts_lh,
								visible, vis_angles);
		if (!n_vis) continue;

		SurvivePose recovered;
		solve_pose_bsvd(sensors, N_SENSORS, visible, n_vis, vis_angles, &recovered);

		FLT err = pose_distance(&recovered, &obj2lh);
		if (err > POSE_TOL) {
			fprintf(stderr, "CleanRoundtrip FAILED (seed=%u, trial=%d, n_vis=%d)\n",
					seed, t, n_vis);
			fprintf(stderr, "  obj2lh: pos=[%.4f,%.4f,%.4f] rot=[%.4f,%.4f,%.4f,%.4f]\n",
					obj2lh.Pos[0], obj2lh.Pos[1], obj2lh.Pos[2],
					obj2lh.Rot[0], obj2lh.Rot[1], obj2lh.Rot[2], obj2lh.Rot[3]);
			fprintf(stderr, "  recovered: pos=[%.4f,%.4f,%.4f] rot=[%.4f,%.4f,%.4f,%.4f]\n",
					recovered.Pos[0], recovered.Pos[1], recovered.Pos[2],
					recovered.Rot[0], recovered.Rot[1], recovered.Rot[2], recovered.Rot[3]);
			fprintf(stderr, "  pose_distance = %.6f\n", err);
			return -1;
		}
	}
	return 0;
}

// 2. Clean solve → all per-sensor residuals near zero
TEST(ReprojectResidualProps, ReprojectionResidualSmall) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	BaseStationCal cal[2] = {0};
	LinmathPoint3d sensors[N_SENSORS];
	generate_sensor_positions(sensors, N_SENSORS);

	for (int t = 0; t < N_TRIALS; t++) {
		SurvivePose obj2lh;
		SurviveAngleReading all_angles[N_SENSORS], vis_angles[N_SENSORS];
		LinmathPoint3d pts_lh[N_SENSORS];
		int visible[N_SENSORS];

		int n_vis = setup_trial(cal, sensors, 6, &obj2lh, all_angles, pts_lh,
								visible, vis_angles);
		if (!n_vis) continue;

		SurvivePose recovered;
		solve_pose_bsvd(sensors, N_SENSORS, visible, n_vis, vis_angles, &recovered);

		FLT residuals[N_SENSORS];
		compute_residuals(cal, &recovered, sensors, visible, n_vis, vis_angles, residuals);

		for (int i = 0; i < n_vis; i++) {
			if (residuals[i] > RESIDUAL_TOL) {
				fprintf(stderr, "ReprojectionResidualSmall FAILED (seed=%u, trial=%d, sensor=%d)\n",
						seed, t, visible[i]);
				fprintf(stderr, "  residual = %.10f (tol = %.10f)\n", residuals[i], RESIDUAL_TOL);
				return -1;
			}
		}
	}
	return 0;
}

// 3. Single corrupted sensor → its residual is much larger than clean sensors
TEST(ReprojectResidualProps, SingleOutlierDetectable) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	BaseStationCal cal[2] = {0};
	LinmathPoint3d sensors[N_SENSORS];
	generate_sensor_positions(sensors, N_SENSORS);

	int detected = 0, tested = 0;

	for (int t = 0; t < N_TRIALS; t++) {
		SurvivePose obj2lh;
		SurviveAngleReading all_angles[N_SENSORS], vis_angles[N_SENSORS];
		LinmathPoint3d pts_lh[N_SENSORS];
		int visible[N_SENSORS];

		int n_vis = setup_trial(cal, sensors, 6, &obj2lh, all_angles, pts_lh,
								visible, vis_angles);
		if (!n_vis) continue;

		// Corrupt one sensor
		int corrupt_idx = rand() % n_vis;
		vis_angles[corrupt_idx][0] += rand_range(CORRUPT_MIN, CORRUPT_MAX) * (rand() % 2 ? 1 : -1);
		vis_angles[corrupt_idx][1] += rand_range(CORRUPT_MIN, CORRUPT_MAX) * (rand() % 2 ? 1 : -1);

		SurvivePose recovered;
		solve_pose_bsvd(sensors, N_SENSORS, visible, n_vis, vis_angles, &recovered);

		FLT residuals[N_SENSORS];
		compute_residuals(cal, &recovered, sensors, visible, n_vis, vis_angles, residuals);

		// Corrupted sensor's residual should be much larger than any clean sensor
		FLT max_clean = 0;
		for (int i = 0; i < n_vis; i++) {
			if (i != corrupt_idx && residuals[i] > max_clean)
				max_clean = residuals[i];
		}

		tested++;
		if (max_clean < 1e-10 || residuals[corrupt_idx] > max_clean * OUTLIER_RATIO)
			detected++;
	}

	FLT rate = (FLT)detected / (FLT)tested;
	if (rate < 0.90) {
		fprintf(stderr, "SingleOutlierDetectable FAILED (seed=%u)\n", seed);
		fprintf(stderr, "  detection rate = %.1f%% (%d/%d), need >90%%\n",
				rate * 100.0, detected, tested);
		return -1;
	}
	return 0;
}

// 4. Single corrupted sensor → pose still close to truth
TEST(ReprojectResidualProps, SingleOutlierPoseStable) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	BaseStationCal cal[2] = {0};
	LinmathPoint3d sensors[N_SENSORS];
	generate_sensor_positions(sensors, N_SENSORS);

	for (int t = 0; t < N_TRIALS; t++) {
		SurvivePose obj2lh;
		SurviveAngleReading all_angles[N_SENSORS], vis_angles[N_SENSORS];
		LinmathPoint3d pts_lh[N_SENSORS];
		int visible[N_SENSORS];

		int n_vis = setup_trial(cal, sensors, 8, &obj2lh, all_angles, pts_lh,
								visible, vis_angles);
		if (!n_vis) continue;

		int corrupt_idx = rand() % n_vis;
		vis_angles[corrupt_idx][0] += rand_range(CORRUPT_MIN, CORRUPT_MAX);
		vis_angles[corrupt_idx][1] += rand_range(CORRUPT_MIN, CORRUPT_MAX);

		SurvivePose recovered;
		solve_pose_bsvd(sensors, N_SENSORS, visible, n_vis, vis_angles, &recovered);

		FLT err = pose_distance(&recovered, &obj2lh);
		if (err > 0.30) {
			fprintf(stderr, "SingleOutlierPoseStable FAILED (seed=%u, trial=%d)\n", seed, t);
			fprintf(stderr, "  n_vis=%d, corrupt_idx=%d, pose_distance=%.4f\n",
					n_vis, corrupt_idx, err);
			return -1;
		}
	}
	return 0;
}

// 5. Multiple corrupted sensors → pose error bounded but larger
TEST(ReprojectResidualProps, MultipleOutliersDegradeGracefully) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	BaseStationCal cal[2] = {0};
	LinmathPoint3d sensors[N_SENSORS];
	generate_sensor_positions(sensors, N_SENSORS);

	for (int t = 0; t < N_TRIALS; t++) {
		SurvivePose obj2lh;
		SurviveAngleReading all_angles[N_SENSORS], vis_angles[N_SENSORS];
		LinmathPoint3d pts_lh[N_SENSORS];
		int visible[N_SENSORS];

		int n_vis = setup_trial(cal, sensors, 9, &obj2lh, all_angles, pts_lh,
								visible, vis_angles);
		if (!n_vis) continue;

		// Corrupt 2-3 sensors
		int n_corrupt = 2 + (rand() % 2);
		int corrupt_set[3] = {-1, -1, -1};
		for (int c = 0; c < n_corrupt; c++) {
			int idx;
			do {
				idx = rand() % n_vis;
			} while (idx == corrupt_set[0] || idx == corrupt_set[1]);
			corrupt_set[c] = idx;
			vis_angles[idx][0] += rand_range(CORRUPT_MIN, CORRUPT_MAX) * (rand() % 2 ? 1 : -1);
			vis_angles[idx][1] += rand_range(CORRUPT_MIN, CORRUPT_MAX) * (rand() % 2 ? 1 : -1);
		}

		SurvivePose recovered;
		solve_pose_bsvd(sensors, N_SENSORS, visible, n_vis, vis_angles, &recovered);

		FLT err = pose_distance(&recovered, &obj2lh);
		if (err > 0.60) {
			fprintf(stderr, "MultipleOutliersDegradeGracefully FAILED (seed=%u, trial=%d)\n",
					seed, t);
			fprintf(stderr, "  n_corrupt=%d, n_vis=%d, pose_distance=%.4f\n",
					n_corrupt, n_vis, err);
			return -1;
		}
	}
	return 0;
}

// 6. Max-residual sensor is the corrupted one
TEST(ReprojectResidualProps, ResidualIdentifiesCorruptedSensor) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	BaseStationCal cal[2] = {0};
	LinmathPoint3d sensors[N_SENSORS];
	generate_sensor_positions(sensors, N_SENSORS);

	int correct = 0, tested = 0;

	for (int t = 0; t < N_TRIALS; t++) {
		SurvivePose obj2lh;
		SurviveAngleReading all_angles[N_SENSORS], vis_angles[N_SENSORS];
		LinmathPoint3d pts_lh[N_SENSORS];
		int visible[N_SENSORS];

		int n_vis = setup_trial(cal, sensors, 8, &obj2lh, all_angles, pts_lh,
								visible, vis_angles);
		if (!n_vis) continue;

		// Use larger min offset for reliable identification
		int corrupt_idx = rand() % n_vis;
		vis_angles[corrupt_idx][0] += rand_range(0.10, CORRUPT_MAX) * (rand() % 2 ? 1 : -1);
		vis_angles[corrupt_idx][1] += rand_range(0.10, CORRUPT_MAX) * (rand() % 2 ? 1 : -1);

		SurvivePose recovered;
		solve_pose_bsvd(sensors, N_SENSORS, visible, n_vis, vis_angles, &recovered);

		FLT residuals[N_SENSORS];
		compute_residuals(cal, &recovered, sensors, visible, n_vis, vis_angles, residuals);

		int max_idx = 0;
		for (int i = 1; i < n_vis; i++) {
			if (residuals[i] > residuals[max_idx])
				max_idx = i;
		}

		tested++;
		if (max_idx == corrupt_idx)
			correct++;
	}

	FLT rate = (FLT)correct / (FLT)tested;
	if (rate < 0.85) {
		fprintf(stderr, "ResidualIdentifiesCorruptedSensor FAILED (seed=%u)\n", seed);
		fprintf(stderr, "  identification rate = %.1f%% (%d/%d), need >85%%\n",
				rate * 100.0, correct, tested);
		return -1;
	}
	return 0;
}
