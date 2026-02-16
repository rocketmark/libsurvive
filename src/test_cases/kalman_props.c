// Property-based tests for Kalman predict state-transition
// (SurviveKalmanModelPredict from kalman_kinematics.gen.h)
//
// The Kalman predict step propagates state forward in time. These tests
// verify invariants of the generated state transition function without
// hardware or SurviveContext dependencies.

#include "survive.h"
#include "../generated/kalman_kinematics.gen.h"
#include "test_case.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

// ── Helpers ──────────────────────────────────────────────────────────

#define N_TRIALS 10000
#define KALMAN_TOL 1e-5
#define KALMAN_POS_TOL 1e-4

static FLT rand_range(FLT min, FLT max) {
	return min + (max - min) * ((FLT)rand() / (FLT)RAND_MAX);
}

static void rand_unit_quat(LinmathQuat q) {
	for (int i = 0; i < 4; i++)
		q[i] = linmath_normrand(0, 1);
	quatnormalize(q, q);
}

static void rand_point(LinmathPoint3d p) {
	p[0] = rand_range(-10.0, 10.0);
	p[1] = rand_range(-10.0, 10.0);
	p[2] = rand_range(-10.0, 10.0);
}

// Generate a random KalmanModel state with unit quaternion
static void rand_kalman_state(SurviveKalmanModel *m) {
	memset(m, 0, sizeof(*m));
	rand_point(m->Pose.Pos);
	rand_unit_quat(m->Pose.Rot);
	for (int i = 0; i < 3; i++) {
		m->Velocity.Pos[i] = rand_range(-5.0, 5.0);
		m->Velocity.AxisAngleRot[i] = rand_range(-1.0, 1.0);
		m->Acc[i] = rand_range(-2.0, 2.0);
	}
}

// ── Property Tests ──────────────────────────────────────────────────

// 1. Zero velocity and zero acceleration -> pose unchanged
TEST(KalmanProps, ZeroVelocityPreservesPose) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	for (int trial = 0; trial < N_TRIALS; trial++) {
		SurviveKalmanModel in = {0}, out = {0};
		rand_point(in.Pose.Pos);
		rand_unit_quat(in.Pose.Rot);
		// Velocity and Acc are zero (memset)

		FLT dt = rand_range(0.001, 0.1);
		SurviveKalmanModelPredict(&out, dt, &in);

		// Position should be unchanged
		for (int j = 0; j < 3; j++) {
			if (fabs(out.Pose.Pos[j] - in.Pose.Pos[j]) > KALMAN_TOL) {
				fprintf(stderr, "ZeroVelocityPreservesPose FAILED (seed=%u, trial=%d)\n", seed, trial);
				fprintf(stderr, "  dt=%.6f, Pos[%d]: in=%.10f out=%.10f\n", dt, j, in.Pose.Pos[j], out.Pose.Pos[j]);
				return -1;
			}
		}

		// Rotation should be unchanged (zero angular velocity -> no rotation)
		FLT sign = (quatinnerproduct(out.Pose.Rot, in.Pose.Rot) < 0) ? -1.0 : 1.0;
		for (int j = 0; j < 4; j++) {
			if (fabs(out.Pose.Rot[j] * sign - in.Pose.Rot[j]) > KALMAN_TOL) {
				fprintf(stderr, "ZeroVelocityPreservesPose FAILED (seed=%u, trial=%d)\n", seed, trial);
				fprintf(stderr, "  dt=%.6f, Rot: in=[%.10f,%.10f,%.10f,%.10f] out=[%.10f,%.10f,%.10f,%.10f]\n",
						dt, in.Pose.Rot[0], in.Pose.Rot[1], in.Pose.Rot[2], in.Pose.Rot[3],
						out.Pose.Rot[0], out.Pose.Rot[1], out.Pose.Rot[2], out.Pose.Rot[3]);
				return -1;
			}
		}
	}
	return 0;
}

// 2. After predict, quaternion stays normalized
TEST(KalmanProps, QuaternionStaysNormalized) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	for (int trial = 0; trial < N_TRIALS; trial++) {
		SurviveKalmanModel in, out = {0};
		rand_kalman_state(&in);

		FLT dt = rand_range(0.001, 0.05);
		SurviveKalmanModelPredict(&out, dt, &in);

		FLT mag = quatmagnitude(out.Pose.Rot);
		if (fabs(mag - 1.0) > 1e-3) {
			fprintf(stderr, "QuaternionStaysNormalized FAILED (seed=%u, trial=%d)\n", seed, trial);
			fprintf(stderr, "  dt=%.6f, |q|=%.15f\n", dt, mag);
			fprintf(stderr, "  angVel=[%.6f,%.6f,%.6f]\n",
					in.Velocity.AxisAngleRot[0], in.Velocity.AxisAngleRot[1], in.Velocity.AxisAngleRot[2]);
			return -1;
		}
	}
	return 0;
}

// 3. Predict with dt=0 -> state unchanged
TEST(KalmanProps, ZeroDtIsIdentity) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	for (int trial = 0; trial < N_TRIALS; trial++) {
		SurviveKalmanModel in, out = {0};
		rand_kalman_state(&in);

		SurviveKalmanModelPredict(&out, 0.0, &in);

		// Position unchanged
		for (int j = 0; j < 3; j++) {
			if (fabs(out.Pose.Pos[j] - in.Pose.Pos[j]) > KALMAN_TOL) {
				fprintf(stderr, "ZeroDtIsIdentity FAILED pos (seed=%u, trial=%d)\n", seed, trial);
				fprintf(stderr, "  Pos[%d]: in=%.10f out=%.10f\n", j, in.Pose.Pos[j], out.Pose.Pos[j]);
				return -1;
			}
		}

		// Rotation unchanged
		FLT sign = (quatinnerproduct(out.Pose.Rot, in.Pose.Rot) < 0) ? -1.0 : 1.0;
		for (int j = 0; j < 4; j++) {
			if (fabs(out.Pose.Rot[j] * sign - in.Pose.Rot[j]) > KALMAN_TOL) {
				fprintf(stderr, "ZeroDtIsIdentity FAILED rot (seed=%u, trial=%d)\n", seed, trial);
				fprintf(stderr, "  Rot: in=[%.10f,%.10f,%.10f,%.10f] out=[%.10f,%.10f,%.10f,%.10f]\n",
						in.Pose.Rot[0], in.Pose.Rot[1], in.Pose.Rot[2], in.Pose.Rot[3],
						out.Pose.Rot[0], out.Pose.Rot[1], out.Pose.Rot[2], out.Pose.Rot[3]);
				return -1;
			}
		}

		// Velocity unchanged
		for (int j = 0; j < 3; j++) {
			if (fabs(out.Velocity.Pos[j] - in.Velocity.Pos[j]) > KALMAN_TOL) {
				fprintf(stderr, "ZeroDtIsIdentity FAILED vel (seed=%u, trial=%d)\n", seed, trial);
				return -1;
			}
		}
	}
	return 0;
}

// 4. For zero angular velocity, predict(2*dt) ≈ predict(dt) applied twice (linear position)
TEST(KalmanProps, LinearPositionComposition) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	for (int trial = 0; trial < N_TRIALS; trial++) {
		SurviveKalmanModel in = {0}, out_2dt = {0}, out_dt = {0}, out_2x = {0};
		rand_point(in.Pose.Pos);
		rand_unit_quat(in.Pose.Rot);
		for (int j = 0; j < 3; j++) {
			in.Velocity.Pos[j] = rand_range(-5.0, 5.0);
			in.Acc[j] = rand_range(-2.0, 2.0);
		}
		// Zero angular velocity so rotation doesn't complicate things

		FLT dt = rand_range(0.001, 0.02);

		// predict(2*dt) in one step
		SurviveKalmanModelPredict(&out_2dt, 2 * dt, &in);

		// predict(dt) twice
		SurviveKalmanModelPredict(&out_dt, dt, &in);
		SurviveKalmanModelPredict(&out_2x, dt, &out_dt);

		// Positions should match closely (linear kinematics: p + v*t + 0.5*a*t^2)
		for (int j = 0; j < 3; j++) {
			if (fabs(out_2dt.Pose.Pos[j] - out_2x.Pose.Pos[j]) > KALMAN_POS_TOL) {
				fprintf(stderr, "LinearPositionComposition FAILED (seed=%u, trial=%d)\n", seed, trial);
				fprintf(stderr, "  dt=%.6f, Pos[%d]: 2dt=%.10f, dt+dt=%.10f\n",
						dt, j, out_2dt.Pose.Pos[j], out_2x.Pose.Pos[j]);
				return -1;
			}
		}
	}
	return 0;
}

// 5. Velocity integrates acceleration correctly: v_out = v_in + a * dt
TEST(KalmanProps, VelocityIntegratesAcceleration) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	for (int trial = 0; trial < N_TRIALS; trial++) {
		SurviveKalmanModel in, out = {0};
		rand_kalman_state(&in);

		FLT dt = rand_range(0.001, 0.05);
		SurviveKalmanModelPredict(&out, dt, &in);

		for (int j = 0; j < 3; j++) {
			FLT expected_vel = in.Velocity.Pos[j] + in.Acc[j] * dt;
			if (fabs(out.Velocity.Pos[j] - expected_vel) > KALMAN_TOL) {
				fprintf(stderr, "VelocityIntegratesAcceleration FAILED (seed=%u, trial=%d)\n", seed, trial);
				fprintf(stderr, "  dt=%.6f, Vel[%d]: expected=%.10f got=%.10f\n",
						dt, j, expected_vel, out.Velocity.Pos[j]);
				return -1;
			}
		}
	}
	return 0;
}

// 6. Angular velocity is preserved (constant in this model)
TEST(KalmanProps, AngularVelocityPreserved) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	for (int trial = 0; trial < N_TRIALS; trial++) {
		SurviveKalmanModel in, out = {0};
		rand_kalman_state(&in);

		FLT dt = rand_range(0.001, 0.05);
		SurviveKalmanModelPredict(&out, dt, &in);

		for (int j = 0; j < 3; j++) {
			if (fabs(out.Velocity.AxisAngleRot[j] - in.Velocity.AxisAngleRot[j]) > KALMAN_TOL) {
				fprintf(stderr, "AngularVelocityPreserved FAILED (seed=%u, trial=%d)\n", seed, trial);
				fprintf(stderr, "  AngVel[%d]: in=%.10f out=%.10f\n",
						j, in.Velocity.AxisAngleRot[j], out.Velocity.AxisAngleRot[j]);
				return -1;
			}
		}
	}
	return 0;
}

// 7. Acceleration is preserved (constant in this model)
TEST(KalmanProps, AccelerationPreserved) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	for (int trial = 0; trial < N_TRIALS; trial++) {
		SurviveKalmanModel in, out = {0};
		rand_kalman_state(&in);

		FLT dt = rand_range(0.001, 0.05);
		SurviveKalmanModelPredict(&out, dt, &in);

		for (int j = 0; j < 3; j++) {
			if (fabs(out.Acc[j] - in.Acc[j]) > KALMAN_TOL) {
				fprintf(stderr, "AccelerationPreserved FAILED (seed=%u, trial=%d)\n", seed, trial);
				fprintf(stderr, "  Acc[%d]: in=%.10f out=%.10f\n", j, in.Acc[j], out.Acc[j]);
				return -1;
			}
		}
	}
	return 0;
}
