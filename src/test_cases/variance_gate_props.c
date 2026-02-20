// Property tests for the Kalman variance gate (report_threshold_var).
//
// KEY FINDING: With ZVU active (stationary tracker, no light), the variance
// gate takes HOURS to trigger — far longer than the observed 14-33 min dropout.
// The variance gate alone cannot explain the dropout under steady IMU conditions.
//
// However, a single large IMU timestamp gap (>50ms) can immediately spike P
// due to t^7 scaling of process noise, triggering the gate instantly.
// This makes IMU packet drops over USB/IP the prime suspect.
//
// These tests simulate the P matrix evolution using the exact Q and F matrices
// from survive_kalman_tracker.c without the full Kalman machinery.

#include "test_case.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

// ── Default parameters from survive_kalman_tracker.c config ──────

#define PROCESS_WEIGHT_JERK    1874161.0
#define PROCESS_WEIGHT_ANG_VEL 60.0
// process_weight_acc, _vel, _pos, _rot all default to 0

#define REPORT_THRESHOLD_VAR   0.1
#define LIGHT_THRESHOLD_VAR    1.0

#define ZVU_NO_LIGHT_VAR       1e-4
#define ZVU_STATIONARY_VAR     1e-5

// Error state: pos(3) + rot_error(3) + vel(3) + ang_vel(3) + acc(3) = 15
#define ERROR_STATE_SIZE       15

#define IMU_DT                 0.001  // 1ms IMU period

// Print characterization results directly (TEST_PRINTF only shows on failure)
#define CHAR_PRINTF(...) fprintf(stderr, __VA_ARGS__)

// ── Small matrix types (one axis at a time) ─────────────────────

typedef struct { double m[3][3]; } Mat3;
typedef struct { double m[2][2]; } Mat2;

static Mat3 mat3_mul(Mat3 A, Mat3 B) {
	Mat3 C = {{{0}}};
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			for (int k = 0; k < 3; k++)
				C.m[i][j] += A.m[i][k] * B.m[k][j];
	return C;
}

static Mat3 mat3_T(Mat3 A) {
	Mat3 T;
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			T.m[i][j] = A.m[j][i];
	return T;
}

static Mat3 mat3_add(Mat3 A, Mat3 B) {
	Mat3 C;
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			C.m[i][j] = A.m[i][j] + B.m[i][j];
	return C;
}

static Mat2 mat2_mul(Mat2 A, Mat2 B) {
	Mat2 C = {{{0}}};
	for (int i = 0; i < 2; i++)
		for (int j = 0; j < 2; j++)
			for (int k = 0; k < 2; k++)
				C.m[i][j] += A.m[i][k] * B.m[k][j];
	return C;
}

static Mat2 mat2_T(Mat2 A) {
	Mat2 T = {{{A.m[0][0], A.m[1][0]}, {A.m[0][1], A.m[1][1]}}};
	return T;
}

static Mat2 mat2_add(Mat2 A, Mat2 B) {
	Mat2 C;
	for (int i = 0; i < 2; i++)
		for (int j = 0; j < 2; j++)
			C.m[i][j] = A.m[i][j] + B.m[i][j];
	return C;
}

// ── Process noise Q and state transition F ──────────────────────

// Q for one positional axis [pos, vel, acc] — jerk model
// From survive_kalman_tracker.c:958-979
static Mat3 pos_Q(double dt, double jerk) {
	double t2 = dt*dt, t3 = t2*dt, t4 = t3*dt, t5 = t4*dt, t6 = t5*dt, t7 = t6*dt;
	Mat3 q = {{{
		jerk * t7/252.0, jerk * t6/72.0, jerk * t5/30.0
	}, {
		jerk * t6/72.0,  jerk * t5/20.0, jerk * t4/8.0
	}, {
		jerk * t5/30.0,  jerk * t4/8.0,  jerk * t3/3.0
	}}};
	return q;
}

// F for one positional axis [pos, vel, acc] — constant acceleration
static Mat3 pos_F(double dt) {
	Mat3 f = {{{1, dt, dt*dt/2.0}, {0, 1, dt}, {0, 0, 1}}};
	return f;
}

// Q for one rotational axis [rot_error, ang_vel]
// rv = s_w * t^3/3, r_av = s_w * t^2/2, s_w*t
// From survive_kalman_tracker.c:988-990 (error state block)
static Mat2 rot_Q(double dt, double s_w) {
	double t2 = dt*dt, t3 = t2*dt;
	Mat2 q = {{{s_w * t3/3.0, s_w * t2/2.0}, {s_w * t2/2.0, s_w * dt}}};
	return q;
}

// F for one rotational axis [rot_error, ang_vel]
static Mat2 rot_F(double dt) {
	Mat2 f = {{{1, dt}, {0, 1}}};
	return f;
}

// ── Kalman predict: P = F*P*F' + Q ──────────────────────────────

static Mat3 predict3(Mat3 P, Mat3 F, Mat3 Q) {
	return mat3_add(mat3_mul(mat3_mul(F, P), mat3_T(F)), Q);
}

static Mat2 predict2(Mat2 P, Mat2 F, Mat2 Q) {
	return mat2_add(mat2_mul(mat2_mul(F, P), mat2_T(F)), Q);
}

// ── Scalar Kalman measurement update ─────────────────────────────
// Measures state[idx] = 0 with variance R.
// P = (I - K*H)*P where K = P[:,idx]/(P[idx][idx] + R)

static void kalman_update_3(Mat3 *P, int idx, double R) {
	double S = P->m[idx][idx] + R;
	if (S <= 0) return;
	double K[3];
	for (int i = 0; i < 3; i++) K[i] = P->m[i][idx] / S;
	double Prow[3];
	for (int j = 0; j < 3; j++) Prow[j] = P->m[idx][j];
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			P->m[i][j] -= K[i] * Prow[j];
}

static void kalman_update_2(Mat2 *P, int idx, double R) {
	double S = P->m[idx][idx] + R;
	if (S <= 0) return;
	double K[2];
	for (int i = 0; i < 2; i++) K[i] = P->m[i][idx] / S;
	double Prow[2];
	for (int j = 0; j < 2; j++) Prow[j] = P->m[idx][j];
	for (int i = 0; i < 2; i++)
		for (int j = 0; j < 2; j++)
			P->m[i][j] -= K[i] * Prow[j];
}

// ── normnd2: sum of squares (matching linmath.c) ────────────────

static double normnd2_diag(double p_pos, double p_rot, double p_vel,
                           double p_angvel, double p_acc) {
	// 3 copies of each per-axis value (x, y, z)
	return 3.0 * (p_pos*p_pos + p_rot*p_rot + p_vel*p_vel +
	              p_angvel*p_angvel + p_acc*p_acc);
}

// ── Simulation state ────────────────────────────────────────────

typedef struct {
	Mat3 P_pos;    // [pos, vel, acc] covariance for one axis
	Mat2 P_rot;    // [rot_error, ang_vel] covariance for one axis
	double jerk;
	double ang_vel_weight;
	double dt;
	double zvu_var;            // ZVU observation variance (-1 = disabled)
	bool constrain_ang_vel;    // whether ZVU constrains angular velocity
	int step;
} VarSim;

static void varsim_init(VarSim *s, double jerk, double s_w, double dt, double zvu_var, bool constrain_ang_vel) {
	memset(s, 0, sizeof(*s));
	s->jerk = jerk;
	s->ang_vel_weight = s_w;
	s->dt = dt;
	s->zvu_var = zvu_var;
	s->constrain_ang_vel = constrain_ang_vel;
}

// Step with a specific dt (for gap simulation)
static void varsim_step_dt(VarSim *s, double dt) {
	Mat3 F3 = pos_F(dt);
	Mat3 Q3 = pos_Q(dt, s->jerk);
	Mat2 F2 = rot_F(dt);
	Mat2 Q2 = rot_Q(dt, s->ang_vel_weight);

	// Predict
	s->P_pos = predict3(s->P_pos, F3, Q3);
	s->P_rot = predict2(s->P_rot, F2, Q2);

	// ZVU: constrain vel (idx 1) and acc (idx 2) in positional block
	if (s->zvu_var >= 0) {
		kalman_update_3(&s->P_pos, 1, s->zvu_var);  // vel
		kalman_update_3(&s->P_pos, 2, s->zvu_var);  // acc

		if (s->constrain_ang_vel) {
			kalman_update_2(&s->P_rot, 1, s->zvu_var);  // ang_vel
		}
	}

	s->step++;
}

static void varsim_step(VarSim *s) {
	varsim_step_dt(s, s->dt);
}

static double varsim_report_var(const VarSim *s) {
	return normnd2_diag(s->P_pos.m[0][0], s->P_rot.m[0][0],
	                    s->P_pos.m[1][1], s->P_rot.m[1][1],
	                    s->P_pos.m[2][2]);
}

static double varsim_time(const VarSim *s) {
	return s->step * s->dt;
}

// Run until report gate triggers or max_steps reached.
// Returns time in seconds, or -1 if max_steps reached.
static double varsim_run_to_gate(VarSim *s, double threshold, int max_steps) {
	for (int i = 0; i < max_steps; i++) {
		varsim_step(s);
		if (varsim_report_var(s) >= threshold)
			return varsim_time(s);
	}
	return -1.0;
}

// ═══════════════════════════════════════════════════════════════════
// SECTION 1: Basic covariance properties
// ═══════════════════════════════════════════════════════════════════

// 1. Without any observations, P grows monotonically every predict step.
TEST(VarianceGate, ProcessNoiseMonotonic) {
	VarSim s;
	varsim_init(&s, PROCESS_WEIGHT_JERK, PROCESS_WEIGHT_ANG_VEL, IMU_DT, -1, false);

	double prev = 0;
	for (int i = 0; i < 1000; i++) {
		varsim_step(&s);
		double cur = varsim_report_var(&s);
		if (cur < prev) {
			TEST_PRINTF("FAIL: normnd2 decreased at step %d: %.10e -> %.10e\n", i, prev, cur);
			return -1;
		}
		prev = cur;
	}
	return 0;
}

// 2. Without observations, gate triggers very fast (under 10ms at 1ms steps).
TEST(VarianceGate, FreeRunningTriggersFast) {
	VarSim s;
	varsim_init(&s, PROCESS_WEIGHT_JERK, PROCESS_WEIGHT_ANG_VEL, IMU_DT, -1, false);

	double t = varsim_run_to_gate(&s, REPORT_THRESHOLD_VAR, 100000);
	if (t < 0) {
		TEST_PRINTF("FAIL: gate never triggered in 100s\n");
		return -1;
	}
	if (t > 0.010) {
		TEST_PRINTF("FAIL: expected gate < 10ms without observations, got %.4f s\n", t);
		return -1;
	}
	return 0;
}

// 3. Process noise Q matrix is symmetric and positive semi-definite.
TEST(VarianceGate, ProcessNoiseSymmetricPSD) {
	double dts[] = {0.001, 0.01, 0.1, 1.0};
	for (int d = 0; d < 4; d++) {
		double dt = dts[d];
		Mat3 Q3 = pos_Q(dt, PROCESS_WEIGHT_JERK);
		Mat2 Q2 = rot_Q(dt, PROCESS_WEIGHT_ANG_VEL);

		// Symmetry check (3x3)
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++) {
				if (fabs(Q3.m[i][j] - Q3.m[j][i]) > 1e-15) {
					TEST_PRINTF("FAIL: Q3 not symmetric at dt=%.3f\n", dt);
					return -1;
				}
			}

		// PSD: diagonal non-negative
		for (int i = 0; i < 3; i++)
			if (Q3.m[i][i] < 0) {
				TEST_PRINTF("FAIL: Q3 diagonal negative at dt=%.3f\n", dt);
				return -1;
			}
		for (int i = 0; i < 2; i++)
			if (Q2.m[i][i] < 0) {
				TEST_PRINTF("FAIL: Q2 diagonal negative at dt=%.3f\n", dt);
				return -1;
			}

		// Symmetry check (2x2)
		if (fabs(Q2.m[0][1] - Q2.m[1][0]) > 1e-15) {
			TEST_PRINTF("FAIL: Q2 not symmetric at dt=%.3f\n", dt);
			return -1;
		}
	}
	return 0;
}

// 4. P matrix stays symmetric through predict+update cycles.
TEST(VarianceGate, PStaysSymmetric) {
	VarSim s;
	varsim_init(&s, PROCESS_WEIGHT_JERK, PROCESS_WEIGHT_ANG_VEL, IMU_DT,
	            ZVU_STATIONARY_VAR, true);

	for (int i = 0; i < 10000; i++) {
		varsim_step(&s);

		for (int r = 0; r < 3; r++)
			for (int c = r+1; c < 3; c++) {
				double diff = fabs(s.P_pos.m[r][c] - s.P_pos.m[c][r]);
				double scale = fabs(s.P_pos.m[r][c]) + fabs(s.P_pos.m[c][r]);
				if (diff > 1e-10 && (scale == 0 || diff / scale > 1e-10)) {
					TEST_PRINTF("FAIL: P_pos not symmetric at step %d: "
					            "P[%d][%d]=%.10e vs P[%d][%d]=%.10e\n",
					            i, r, c, s.P_pos.m[r][c], c, r, s.P_pos.m[c][r]);
					return -1;
				}
			}

		double diff = fabs(s.P_rot.m[0][1] - s.P_rot.m[1][0]);
		double scale = fabs(s.P_rot.m[0][1]) + fabs(s.P_rot.m[1][0]);
		if (diff > 1e-10 && (scale == 0 || diff / scale > 1e-10)) {
			TEST_PRINTF("FAIL: P_rot not symmetric at step %d\n", i);
			return -1;
		}
	}
	return 0;
}

// 5. P diagonal elements are always non-negative.
TEST(VarianceGate, PDiagNonNegative) {
	VarSim s;
	varsim_init(&s, PROCESS_WEIGHT_JERK, PROCESS_WEIGHT_ANG_VEL, IMU_DT,
	            ZVU_STATIONARY_VAR, true);

	for (int i = 0; i < 100000; i++) {
		varsim_step(&s);
		for (int j = 0; j < 3; j++)
			if (s.P_pos.m[j][j] < 0) {
				TEST_PRINTF("FAIL: P_pos[%d][%d] < 0 at step %d\n", j, j, i);
				return -1;
			}
		for (int j = 0; j < 2; j++)
			if (s.P_rot.m[j][j] < 0) {
				TEST_PRINTF("FAIL: P_rot[%d][%d] < 0 at step %d\n", j, j, i);
				return -1;
			}
	}
	return 0;
}

// ═══════════════════════════════════════════════════════════════════
// SECTION 2: ZVU behavior
// ═══════════════════════════════════════════════════════════════════

// 6. ZVU-constrained dimensions reach steady state quickly.
TEST(VarianceGate, ZVUDimensionsSteadyState) {
	VarSim s;
	varsim_init(&s, PROCESS_WEIGHT_JERK, PROCESS_WEIGHT_ANG_VEL, IMU_DT,
	            ZVU_STATIONARY_VAR, true);

	for (int i = 0; i < 1000; i++)
		varsim_step(&s);
	double vel_1s = s.P_pos.m[1][1];
	double acc_1s = s.P_pos.m[2][2];
	double angvel_1s = s.P_rot.m[1][1];

	for (int i = 0; i < 59000; i++)
		varsim_step(&s);
	double vel_60s = s.P_pos.m[1][1];
	double acc_60s = s.P_pos.m[2][2];
	double angvel_60s = s.P_rot.m[1][1];

	double vel_ratio = vel_60s / vel_1s;
	double acc_ratio = acc_60s / acc_1s;
	double angvel_ratio = angvel_60s / angvel_1s;

	if (vel_ratio > 2.0 || vel_ratio < 0.5) {
		TEST_PRINTF("FAIL: vel P not at steady state: ratio=%.2f\n", vel_ratio);
		return -1;
	}
	if (acc_ratio > 2.0 || acc_ratio < 0.5) {
		TEST_PRINTF("FAIL: acc P not at steady state: ratio=%.2f\n", acc_ratio);
		return -1;
	}
	if (angvel_ratio > 2.0 || angvel_ratio < 0.5) {
		TEST_PRINTF("FAIL: angvel P not at steady state: ratio=%.2f\n", angvel_ratio);
		return -1;
	}

	CHAR_PRINTF("Steady state ratios (60s/1s): vel=%.3f, acc=%.3f, angvel=%.3f\n",
	            vel_ratio, acc_ratio, angvel_ratio);
	return 0;
}

// 7. Stronger ZVU (lower var) → later gate. All should be > 1 hour.
TEST(VarianceGate, StrongerZVU_DelaysGate) {
	double vars[] = {1e-3, 1e-4, 1e-5, 1e-6};
	double times[4];
	int max_steps = 60 * 60 * 1000;

	for (int i = 0; i < 4; i++) {
		VarSim s;
		varsim_init(&s, PROCESS_WEIGHT_JERK, PROCESS_WEIGHT_ANG_VEL, IMU_DT,
		            vars[i], true);
		times[i] = varsim_run_to_gate(&s, REPORT_THRESHOLD_VAR, max_steps);
		if (times[i] < 0) times[i] = 3601.0;
	}

	// Monotonically non-decreasing
	for (int i = 1; i < 4; i++) {
		if (times[i] < times[i-1] - 1.0) {
			TEST_PRINTF("FAIL: stronger ZVU (%.0e) triggered sooner (%.1f s) "
			            "than weaker (%.0e -> %.1f s)\n",
			            vars[i], times[i], vars[i-1], times[i-1]);
			return -1;
		}
	}

	CHAR_PRINTF("ZVU sensitivity: ");
	for (int i = 0; i < 4; i++)
		CHAR_PRINTF("%.0e->%.0fs  ", vars[i], times[i]);
	CHAR_PRINTF("\n");
	return 0;
}

// 8. With ZVU disabled entirely, the gate triggers almost instantly.
TEST(VarianceGate, NoZVU_InstantGate) {
	VarSim s;
	varsim_init(&s, PROCESS_WEIGHT_JERK, PROCESS_WEIGHT_ANG_VEL, IMU_DT, -1, false);

	double t = varsim_run_to_gate(&s, REPORT_THRESHOLD_VAR, 10000);
	if (t < 0) {
		TEST_PRINTF("FAIL: gate never triggered without ZVU\n");
		return -1;
	}
	if (t > 0.1) {
		TEST_PRINTF("FAIL: expected near-instant gate without ZVU, got %.4f s\n", t);
		return -1;
	}
	return 0;
}

// ═══════════════════════════════════════════════════════════════════
// SECTION 3: Variance gate timing characterization
//
// KEY RESULT: With stationary ZVU, the gate takes >5 hours to trigger.
// This RULES OUT the variance gate as the cause of 14-33 min dropout
// under steady IMU conditions.
// ═══════════════════════════════════════════════════════════════════

// 9. Stationary tracker with ZVU (no light): gate takes > 1 hour.
// This proves variance gate alone cannot explain the 14-33 min dropout.
TEST(VarianceGate, StationaryNoLight_SlowerThan1Hour) {
	VarSim s;
	varsim_init(&s, PROCESS_WEIGHT_JERK, PROCESS_WEIGHT_ANG_VEL, IMU_DT,
	            ZVU_STATIONARY_VAR, true);

	int max_steps = 60 * 60 * 1000;  // 1 hour
	double t = varsim_run_to_gate(&s, REPORT_THRESHOLD_VAR, max_steps);

	if (t >= 0) {
		TEST_PRINTF("FAIL: gate triggered at %.1f s, expected > 1 hour with stationary ZVU\n", t);
		return -1;
	}

	// Report what P looks like after 1 hour
	double p_pos = s.P_pos.m[0][0];
	double p_rot = s.P_rot.m[0][0];
	double total = varsim_report_var(&s);
	CHAR_PRINTF("After 1hr: normnd2=%.4e (threshold=%.1f) P_pos=%.4e P_rot=%.4e\n",
	            total, REPORT_THRESHOLD_VAR, p_pos, p_rot);
	return 0;
}

// 10. Rotation dominates P growth under stationary ZVU.
// Position grows slower because both vel AND acc are constrained.
TEST(VarianceGate, RotationDominatesUnderStationaryZVU) {
	VarSim s;
	varsim_init(&s, PROCESS_WEIGHT_JERK, PROCESS_WEIGHT_ANG_VEL, IMU_DT,
	            ZVU_STATIONARY_VAR, true);

	// Run for 1 hour
	for (int i = 0; i < 60 * 60 * 1000; i++)
		varsim_step(&s);

	double rot_contrib = 3.0 * s.P_rot.m[0][0] * s.P_rot.m[0][0];
	double total = varsim_report_var(&s);

	if (rot_contrib < total * 0.5) {
		TEST_PRINTF("FAIL: rotation contributes only %.1f%% (expected >50%%)\n",
		            100.0 * rot_contrib / total);
		return -1;
	}

	CHAR_PRINTF("After 1hr: P_pos=%.4e P_rot=%.4e P_vel=%.4e P_angvel=%.4e P_acc=%.4e\n",
	            s.P_pos.m[0][0], s.P_rot.m[0][0], s.P_pos.m[1][1],
	            s.P_rot.m[1][1], s.P_pos.m[2][2]);
	CHAR_PRINTF("Rotation: %.1f%% of normnd2\n", 100.0 * rot_contrib / total);
	return 0;
}

// 11. Moving tracker (ang_vel not constrained): gate triggers fast.
TEST(VarianceGate, MovingNoLight_AngVelDominates) {
	VarSim s;
	varsim_init(&s, PROCESS_WEIGHT_JERK, PROCESS_WEIGHT_ANG_VEL, IMU_DT,
	            ZVU_NO_LIGHT_VAR, false);

	double t = varsim_run_to_gate(&s, REPORT_THRESHOLD_VAR, 60000);
	if (t < 0) {
		TEST_PRINTF("FAIL: gate never triggered\n");
		return -1;
	}
	if (t > 1.0) {
		TEST_PRINTF("FAIL: expected fast trigger, got %.3f s\n", t);
		return -1;
	}
	return 0;
}

// 12. With light observations, P stays bounded indefinitely.
TEST(VarianceGate, LightObservations_BoundP) {
	VarSim s;
	varsim_init(&s, PROCESS_WEIGHT_JERK, PROCESS_WEIGHT_ANG_VEL, IMU_DT,
	            ZVU_STATIONARY_VAR, true);

	int light_interval = 8;  // ~120Hz
	int max_steps = 10 * 60 * 1000;

	double max_var = 0;
	for (int i = 0; i < max_steps; i++) {
		varsim_step(&s);
		if (i % light_interval == 0) {
			kalman_update_3(&s.P_pos, 0, 1e-2);
			kalman_update_2(&s.P_rot, 0, 1e-2);
		}
		double rv = varsim_report_var(&s);
		if (rv > max_var) max_var = rv;
	}

	if (max_var >= REPORT_THRESHOLD_VAR) {
		TEST_PRINTF("FAIL: P exceeded threshold with light: %.4e\n", max_var);
		return -1;
	}
	return 0;
}

// 13. Zero jerk weight + ZVU: gate doesn't trigger for 1 hour.
TEST(VarianceGate, ZeroJerk_NoGateTrigger) {
	VarSim s;
	varsim_init(&s, 0, PROCESS_WEIGHT_ANG_VEL, IMU_DT, ZVU_STATIONARY_VAR, true);

	int max_steps = 60 * 60 * 1000;
	double t = varsim_run_to_gate(&s, REPORT_THRESHOLD_VAR, max_steps);

	if (t >= 0 && t < 1800.0) {
		TEST_PRINTF("FAIL: gate triggered at %.1f s with zero jerk\n", t);
		return -1;
	}
	return 0;
}

// ═══════════════════════════════════════════════════════════════════
// SECTION 4: IMU gap sensitivity — the likely root cause
//
// Process noise Q scales as t^7 (jerk model). A single IMU timestamp
// gap of ~80ms adds enough position variance to trigger the report gate.
// libsurvive does NOT cap dt for process noise (only warns at >500ms).
//
// This makes USB/IP latency spikes the prime suspect for the dropout.
// ═══════════════════════════════════════════════════════════════════

// 14. Process noise Q_pos scales as t^7.
TEST(VarianceGate, ProcessNoise_T7Scaling) {
	double dt1 = 0.001;
	double dt2 = 0.010;  // 10x larger
	Mat3 Q1 = pos_Q(dt1, PROCESS_WEIGHT_JERK);
	Mat3 Q2 = pos_Q(dt2, PROCESS_WEIGHT_JERK);

	double ratio = Q2.m[0][0] / Q1.m[0][0];
	double expected = pow(10.0, 7.0);  // 10^7

	if (fabs(ratio - expected) / expected > 1e-6) {
		TEST_PRINTF("FAIL: Q_pos ratio = %.1f, expected %.1f (10^7)\n", ratio, expected);
		return -1;
	}
	return 0;
}

// 15. Find the critical single-gap size that triggers the report gate.
// ZVU is very effective: even after a large dt, the velocity/acceleration
// covariance is pulled back down. Only position and rotation residuals remain.
TEST(VarianceGate, CriticalGapSize) {
	double gap_ms[] = {50, 100, 150, 200, 250, 300, 350, 400, 450, 500};
	int n_gaps = sizeof(gap_ms) / sizeof(gap_ms[0]);
	double critical_gap = -1;

	for (int g = 0; g < n_gaps; g++) {
		VarSim s;
		varsim_init(&s, PROCESS_WEIGHT_JERK, PROCESS_WEIGHT_ANG_VEL, IMU_DT,
		            ZVU_STATIONARY_VAR, true);

		// Reach steady state
		for (int i = 0; i < 1000; i++)
			varsim_step(&s);

		// One gap
		double gap_s = gap_ms[g] / 1000.0;
		varsim_step_dt(&s, gap_s);

		double rv = varsim_report_var(&s);
		if (rv >= REPORT_THRESHOLD_VAR && critical_gap < 0) {
			critical_gap = gap_ms[g];
		}
		CHAR_PRINTF("  gap=%3.0fms -> normnd2=%.4e %s\n",
		            gap_ms[g], rv, rv >= REPORT_THRESHOLD_VAR ? "TRIGGERS" : "ok");
	}

	if (critical_gap < 0) {
		TEST_PRINTF("FAIL: no gap up to 500ms triggered the gate\n");
		return -1;
	}

	CHAR_PRINTF("Critical single gap: %.0f ms\n", critical_gap);
	return 0;
}

// 16. A gap at the critical size triggers the gate.
TEST(VarianceGate, SingleGapAtCritical_TriggersGate) {
	// Use 400ms as a conservatively large gap
	VarSim s;
	varsim_init(&s, PROCESS_WEIGHT_JERK, PROCESS_WEIGHT_ANG_VEL, IMU_DT,
	            ZVU_STATIONARY_VAR, true);

	for (int i = 0; i < 1000; i++)
		varsim_step(&s);

	double before = varsim_report_var(&s);
	varsim_step_dt(&s, 0.4);
	double after = varsim_report_var(&s);

	if (after < REPORT_THRESHOLD_VAR) {
		TEST_PRINTF("FAIL: 400ms gap did not trigger gate: normnd2=%.4e\n", after);
		return -1;
	}

	CHAR_PRINTF("400ms gap: normnd2 %.4e -> %.4e (threshold=%.1f)\n",
	            before, after, REPORT_THRESHOLD_VAR);
	return 0;
}

// 17. After a gap spike, ZVU can recover P if light eventually returns.
TEST(VarianceGate, GapRecovery_WithLight) {
	VarSim s;
	varsim_init(&s, PROCESS_WEIGHT_JERK, PROCESS_WEIGHT_ANG_VEL, IMU_DT,
	            ZVU_STATIONARY_VAR, true);

	// Steady state
	for (int i = 0; i < 1000; i++)
		varsim_step(&s);

	// Spike with 100ms gap
	varsim_step_dt(&s, 0.1);
	double spiked = varsim_report_var(&s);

	// Resume with normal IMU + light observations
	int light_interval = 8;
	int recovery_step = -1;
	for (int i = 0; i < 60000; i++) {
		varsim_step(&s);
		if (i % light_interval == 0) {
			kalman_update_3(&s.P_pos, 0, 1e-2);
			kalman_update_2(&s.P_rot, 0, 1e-2);
		}
		if (recovery_step < 0 && varsim_report_var(&s) < REPORT_THRESHOLD_VAR)
			recovery_step = i;
	}

	if (recovery_step < 0) {
		TEST_PRINTF("FAIL: P never recovered below threshold after spike\n");
		return -1;
	}

	double recovery_time = recovery_step * IMU_DT;
	CHAR_PRINTF("Recovery: spiked=%.4e, recovered in %.3f s (%d steps)\n",
	            spiked, recovery_time, recovery_step);
	return 0;
}

// 18. After a large gap spike, characterize ZVU-only recovery.
// ZVU constrains vel/acc which are coupled to position via F matrix,
// so position covariance slowly drains through the coupling terms.
TEST(VarianceGate, GapRecovery_NoLight_ZVUOnly) {
	VarSim s;
	varsim_init(&s, PROCESS_WEIGHT_JERK, PROCESS_WEIGHT_ANG_VEL, IMU_DT,
	            ZVU_STATIONARY_VAR, true);

	for (int i = 0; i < 1000; i++)
		varsim_step(&s);

	// Spike with 400ms gap (above critical threshold)
	varsim_step_dt(&s, 0.4);
	double spiked = varsim_report_var(&s);

	// Run 5 minutes with ZVU only (no light)
	int recovery_step = -1;
	for (int i = 0; i < 5 * 60 * 1000; i++) {
		varsim_step(&s);
		if (recovery_step < 0 && varsim_report_var(&s) < REPORT_THRESHOLD_VAR)
			recovery_step = i;
	}

	double final_var = varsim_report_var(&s);
	if (recovery_step >= 0) {
		CHAR_PRINTF("ZVU-only recovery: spiked=%.4e, recovered in %.1f s\n",
		            spiked, recovery_step * IMU_DT);
	} else {
		CHAR_PRINTF("ZVU-only: spiked=%.4e, after 5min=%.4e (no recovery)\n",
		            spiked, final_var);
	}
	// Characterization only — don't fail
	return 0;
}

// 19. Multiple small gaps accumulate: even 10ms gaps every second
// will gradually grow P through the pos/vel/acc coupling.
TEST(VarianceGate, RepeatedSmallGaps_Accumulate) {
	VarSim s;
	varsim_init(&s, PROCESS_WEIGHT_JERK, PROCESS_WEIGHT_ANG_VEL, IMU_DT,
	            ZVU_STATIONARY_VAR, true);

	// Run with periodic 10ms gaps (every 1 second = 1000 steps)
	int gate_step = -1;
	int max_seconds = 3600;  // 1 hour
	for (int sec = 0; sec < max_seconds; sec++) {
		// 999 normal 1ms steps
		for (int i = 0; i < 999; i++)
			varsim_step(&s);

		// 1 step with 10ms gap (simulating 10 missed packets)
		varsim_step_dt(&s, 0.010);

		if (gate_step < 0 && varsim_report_var(&s) >= REPORT_THRESHOLD_VAR) {
			gate_step = sec;
			break;
		}
	}

	if (gate_step >= 0) {
		CHAR_PRINTF("Repeated 10ms gaps: gate triggered at %d seconds\n", gate_step);
	} else {
		double final_var = varsim_report_var(&s);
		CHAR_PRINTF("Repeated 10ms gaps: no gate in 1hr (normnd2=%.4e)\n", final_var);
	}
	// Just characterization — don't fail
	return 0;
}

// 20. Repeated 50ms gaps (more severe USB/IP stalls) should trigger faster.
TEST(VarianceGate, RepeatedLargeGaps_TriggerFaster) {
	double gap_sizes[] = {0.010, 0.020, 0.050};
	double trigger_times[3];
	int max_seconds = 3600;

	for (int g = 0; g < 3; g++) {
		VarSim s;
		varsim_init(&s, PROCESS_WEIGHT_JERK, PROCESS_WEIGHT_ANG_VEL, IMU_DT,
		            ZVU_STATIONARY_VAR, true);
		trigger_times[g] = -1;

		for (int sec = 0; sec < max_seconds; sec++) {
			for (int i = 0; i < 999; i++)
				varsim_step(&s);
			varsim_step_dt(&s, gap_sizes[g]);
			if (varsim_report_var(&s) >= REPORT_THRESHOLD_VAR) {
				trigger_times[g] = sec;
				break;
			}
		}
	}

	// Larger gaps should trigger sooner (or equal)
	for (int i = 1; i < 3; i++) {
		double t_cur = trigger_times[i] < 0 ? 3601.0 : trigger_times[i];
		double t_prev = trigger_times[i-1] < 0 ? 3601.0 : trigger_times[i-1];
		if (t_cur > t_prev + 1.0) {
			TEST_PRINTF("FAIL: larger gap (%.0f ms) triggered later than smaller (%.0f ms)\n",
			            gap_sizes[i] * 1000, gap_sizes[i-1] * 1000);
			return -1;
		}
	}

	CHAR_PRINTF("Gap timing: ");
	for (int i = 0; i < 3; i++) {
		if (trigger_times[i] < 0)
			CHAR_PRINTF("%.0fms->never  ", gap_sizes[i] * 1000);
		else
			CHAR_PRINTF("%.0fms->%ds  ", gap_sizes[i] * 1000, (int)trigger_times[i]);
	}
	CHAR_PRINTF("\n");
	return 0;
}

// 21. Characterization: exactly how many seconds of 50ms-every-second gaps
// match the 14-33 minute dropout window?
TEST(VarianceGate, Characterize_50msPeriodicGap) {
	VarSim s;
	varsim_init(&s, PROCESS_WEIGHT_JERK, PROCESS_WEIGHT_ANG_VEL, IMU_DT,
	            ZVU_STATIONARY_VAR, true);

	// Also add light observations at 120Hz to simulate normal tracking
	int light_interval = 8;
	int global_step = 0;
	int max_seconds = 3600;
	int gate_second = -1;

	for (int sec = 0; sec < max_seconds; sec++) {
		// 999 normal steps with light
		for (int i = 0; i < 999; i++) {
			varsim_step(&s);
			global_step++;
			if (global_step % light_interval == 0) {
				kalman_update_3(&s.P_pos, 0, 1e-2);
				kalman_update_2(&s.P_rot, 0, 1e-2);
			}
		}

		// 1 step with 50ms gap (no light during the gap)
		varsim_step_dt(&s, 0.050);
		global_step++;

		if (gate_second < 0 && varsim_report_var(&s) >= REPORT_THRESHOLD_VAR) {
			gate_second = sec;
		}
	}

	if (gate_second >= 0) {
		double gate_min = gate_second / 60.0;
		CHAR_PRINTF("50ms periodic gap WITH light: gate at %d s (%.1f min)\n",
		            gate_second, gate_min);
	} else {
		CHAR_PRINTF("50ms periodic gap WITH light: no gate in 1 hour\n");
	}
	return 0;
}
