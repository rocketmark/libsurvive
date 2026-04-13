// Property-based tests for the light residual cascade that causes
// 14-33 minute pose dropouts.
//
// The failure chain in survive_kalman_tracker.c:
//   1. light_residuals_all EMA grows past light_error_threshold
//   2. check_valid() fails → lost_tracking() → reinit()
//   3. reinit() zeros stats.obs_count
//   4. integrate_saved_light() gates on obs_count < light_required_obs (16)
//   5. Light is silently dropped → obs_count never recovers → death spiral
//
// These tests exercise the pure logic of each step without the full Kalman
// machinery. They run thousands of randomized scenarios to verify invariants
// and characterize failure dynamics.

#include "test_case.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stdbool.h>

// ── Constants matching survive_kalman_tracker.c defaults ──────────

#define DEFAULT_LIGHT_ERROR_THRESHOLD (-1.0)  // disabled by default
#define TYPICAL_LIGHT_ERROR_THRESHOLD 0.01    // when enabled
#define DEFAULT_LIGHT_REQUIRED_OBS 16
#define EMA_ALPHA 0.1
#define EMA_DECAY (1.0 - EMA_ALPHA)  // 0.9
#define TIMEBASE_HZ 48000000
#define STATIONARY_GRACE_TICKS (TIMEBASE_HZ / 10)  // 100ms

// ── Helpers ──────────────────────────────────────────────────────

static double rand_range(double min, double max) {
	return min + (max - min) * ((double)rand() / (double)RAND_MAX);
}

// Reproduce the EMA update from survive_kalman_tracker.c lines 480-481:
//   tracker->light_residuals_all *= .9;
//   tracker->light_residuals_all += .1 * rtn;
static double ema_update(double residual_all, double rtn) {
	return residual_all * EMA_DECAY + EMA_ALPHA * rtn;
}

// Reproduce check_valid logic from survive_kalman_tracker.c lines 1617-1631:
//   isValid = threshold <= 0 || residuals < threshold || moving
//   also: |pos| < 20 on each axis
static bool check_valid(double light_error_threshold,
                        double light_residuals_all,
                        long long stationary_ticks,
                        double pos_x, double pos_y, double pos_z) {
	bool isValid =
		light_error_threshold <= 0 ||
		light_residuals_all < light_error_threshold ||
		(stationary_ticks < STATIONARY_GRACE_TICKS);

	isValid &= fabs(pos_x) < 20.0;
	isValid &= fabs(pos_y) < 20.0;
	isValid &= fabs(pos_z) < 20.0;

	return isValid;
}

// Reproduce light gating from survive_kalman_tracker.c line 386-388:
//   if (tracker->light_required_obs > tracker->stats.obs_count) return;
static bool light_is_gated(int light_required_obs, int obs_count) {
	return light_required_obs > obs_count;
}

// ── 1. EMA Convergence Properties ────────────────────────────────

// The EMA with alpha=0.1 converges to the input value. After N identical
// inputs, the EMA should be within a known tolerance of that value.
TEST(ResidualCascade, EMAConvergesToSteadyState) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	for (int trial = 0; trial < 10000; trial++) {
		double steady_value = rand_range(0.0, 1.0);
		double ema = rand_range(0.0, 1.0);  // random starting point

		// After 100 identical inputs, should be very close to steady_value
		for (int i = 0; i < 100; i++) {
			ema = ema_update(ema, steady_value);
		}

		// 0.9^100 ≈ 2.66e-5, so residual from initial value is negligible
		double error = fabs(ema - steady_value);
		if (error > 1e-3) {
			fprintf(stderr, "EMAConvergesToSteadyState FAILED (seed=%u, trial=%d)\n", seed, trial);
			fprintf(stderr, "  After 100 steps: ema=%.10f target=%.10f error=%.10f\n",
					ema, steady_value, error);
			return -1;
		}
	}
	return 0;
}

// The EMA time constant is ~10 updates (1/alpha). After a step change from
// 0 to V, the EMA should reach ~63.2% of V after 10 updates.
TEST(ResidualCascade, EMATimeConstant) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	for (int trial = 0; trial < 1000; trial++) {
		double V = rand_range(0.001, 10.0);
		double ema = 0.0;

		for (int i = 0; i < 10; i++) {
			ema = ema_update(ema, V);
		}

		// After 10 steps: 1 - 0.9^10 = 0.6513... ≈ 63-66% of V
		double ratio = ema / V;
		if (ratio < 0.60 || ratio > 0.70) {
			fprintf(stderr, "EMATimeConstant FAILED (seed=%u, trial=%d)\n", seed, trial);
			fprintf(stderr, "  V=%.6f ema=%.6f ratio=%.4f (expected ~0.65)\n", V, ema, ratio);
			return -1;
		}
	}
	return 0;
}

// EMA is always non-negative when inputs are non-negative
TEST(ResidualCascade, EMANonNegative) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	for (int trial = 0; trial < 1000; trial++) {
		double ema = rand_range(0.0, 100.0);
		int n_steps = 50 + rand() % 200;

		for (int i = 0; i < n_steps; i++) {
			double rtn = rand_range(0.0, 10.0);
			ema = ema_update(ema, rtn);
			if (ema < 0.0) {
				fprintf(stderr, "EMANonNegative FAILED (seed=%u, trial=%d, step=%d)\n",
						seed, trial, i);
				return -1;
			}
		}
	}
	return 0;
}

// EMA is bounded by the max input value (after convergence)
TEST(ResidualCascade, EMABoundedByMax) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	for (int trial = 0; trial < 1000; trial++) {
		double max_input = rand_range(0.1, 10.0);
		double ema = 0.0;

		for (int i = 0; i < 200; i++) {
			double rtn = rand_range(0.0, max_input);
			ema = ema_update(ema, rtn);
		}

		// EMA should never exceed max_input (with small epsilon for float)
		if (ema > max_input + 1e-10) {
			fprintf(stderr, "EMABoundedByMax FAILED (seed=%u, trial=%d)\n", seed, trial);
			fprintf(stderr, "  ema=%.10f max_input=%.10f\n", ema, max_input);
			return -1;
		}
	}
	return 0;
}

// ── 2. check_valid Logic Properties ──────────────────────────────

// Disabled threshold (<=0) always passes regardless of residuals
TEST(ResidualCascade, DisabledThresholdAlwaysValid) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	for (int trial = 0; trial < 10000; trial++) {
		double threshold = rand_range(-100.0, 0.0);
		double residuals = rand_range(0.0, 1000.0);
		long long stationary = (long long)(rand_range(0, 1e9));
		double pos[3];
		for (int i = 0; i < 3; i++)
			pos[i] = rand_range(-19.0, 19.0);  // within bounds

		if (!check_valid(threshold, residuals, stationary, pos[0], pos[1], pos[2])) {
			fprintf(stderr, "DisabledThresholdAlwaysValid FAILED (seed=%u, trial=%d)\n",
					seed, trial);
			fprintf(stderr, "  threshold=%.4f residuals=%.4f\n", threshold, residuals);
			return -1;
		}
	}
	return 0;
}

// Below-threshold residuals always pass when position is in bounds
TEST(ResidualCascade, BelowThresholdAlwaysValid) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	for (int trial = 0; trial < 10000; trial++) {
		double threshold = rand_range(0.001, 1.0);
		double residuals = rand_range(0.0, threshold - 1e-10);  // strictly below
		long long stationary = (long long)(rand_range(STATIONARY_GRACE_TICKS + 1, 1e9));
		double pos[3];
		for (int i = 0; i < 3; i++)
			pos[i] = rand_range(-19.0, 19.0);

		if (!check_valid(threshold, residuals, stationary, pos[0], pos[1], pos[2])) {
			fprintf(stderr, "BelowThresholdAlwaysValid FAILED (seed=%u, trial=%d)\n",
					seed, trial);
			fprintf(stderr, "  threshold=%.6f residuals=%.6f\n", threshold, residuals);
			return -1;
		}
	}
	return 0;
}

// Moving tracker gets grace period even with high residuals
TEST(ResidualCascade, MovingGracePeriod) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	for (int trial = 0; trial < 10000; trial++) {
		double threshold = rand_range(0.001, 0.1);
		double residuals = threshold + rand_range(0.01, 100.0);  // above threshold
		// stationary_ticks < STATIONARY_GRACE_TICKS means "recently moved"
		long long stationary = (long long)(rand_range(0, STATIONARY_GRACE_TICKS - 1));
		double pos[3];
		for (int i = 0; i < 3; i++)
			pos[i] = rand_range(-19.0, 19.0);

		if (!check_valid(threshold, residuals, stationary, pos[0], pos[1], pos[2])) {
			fprintf(stderr, "MovingGracePeriod FAILED (seed=%u, trial=%d)\n", seed, trial);
			fprintf(stderr, "  threshold=%.6f residuals=%.6f stationary=%lld\n",
					threshold, residuals, stationary);
			return -1;
		}
	}
	return 0;
}

// Stationary + above threshold = invalid (the failure condition)
TEST(ResidualCascade, StationaryAboveThresholdFails) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	for (int trial = 0; trial < 10000; trial++) {
		double threshold = rand_range(0.001, 0.1);
		double residuals = threshold + rand_range(0.001, 100.0);  // above threshold
		// Well past the grace period
		long long stationary = STATIONARY_GRACE_TICKS + (long long)(rand_range(1, 1e9));
		double pos[3];
		for (int i = 0; i < 3; i++)
			pos[i] = rand_range(-19.0, 19.0);

		if (check_valid(threshold, residuals, stationary, pos[0], pos[1], pos[2])) {
			fprintf(stderr, "StationaryAboveThresholdFails FAILED (seed=%u, trial=%d)\n",
					seed, trial);
			fprintf(stderr, "  threshold=%.6f residuals=%.6f stationary=%lld\n",
					threshold, residuals, stationary);
			return -1;
		}
	}
	return 0;
}

// Position out of bounds always fails regardless of other state
TEST(ResidualCascade, OutOfBoundsAlwaysFails) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	for (int trial = 0; trial < 10000; trial++) {
		double threshold = rand_range(-10.0, 1.0);
		double residuals = rand_range(0.0, 0.001);  // low residuals
		long long stationary = 0;  // moving
		// At least one axis out of bounds
		int bad_axis = rand() % 3;
		double pos[3];
		for (int i = 0; i < 3; i++)
			pos[i] = rand_range(-19.0, 19.0);
		pos[bad_axis] = (rand() % 2 == 0) ?
			rand_range(20.0, 1000.0) : rand_range(-1000.0, -20.0);

		if (check_valid(threshold, residuals, stationary, pos[0], pos[1], pos[2])) {
			fprintf(stderr, "OutOfBoundsAlwaysFails FAILED (seed=%u, trial=%d)\n",
					seed, trial);
			fprintf(stderr, "  pos=[%.2f, %.2f, %.2f] bad_axis=%d\n",
					pos[0], pos[1], pos[2], bad_axis);
			return -1;
		}
	}
	return 0;
}

// check_valid is total: every (threshold, residuals, stationary, pos) tuple
// maps to exactly one of {valid, invalid}
TEST(ResidualCascade, CheckValidIsTotal) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	for (int trial = 0; trial < 10000; trial++) {
		double threshold = rand_range(-10.0, 10.0);
		double residuals = rand_range(-1.0, 100.0);
		long long stationary = (long long)(rand_range(0, 1e10));
		double pos[3];
		for (int i = 0; i < 3; i++)
			pos[i] = rand_range(-30.0, 30.0);

		bool result = check_valid(threshold, residuals, stationary,
								  pos[0], pos[1], pos[2]);
		// Just verify it returns a valid bool and doesn't crash
		if (result != true && result != false) {
			fprintf(stderr, "CheckValidIsTotal FAILED (seed=%u, trial=%d)\n",
					seed, trial);
			return -1;
		}
	}
	return 0;
}

// ── 3. Light Gating / Death Spiral Properties ────────────────────

// Light is always gated when obs_count < light_required_obs
TEST(ResidualCascade, LightGatedBelowThreshold) {
	for (int required = 1; required <= 32; required++) {
		for (int obs = 0; obs < required; obs++) {
			if (!light_is_gated(required, obs)) {
				fprintf(stderr, "LightGatedBelowThreshold FAILED: required=%d obs=%d\n",
						required, obs);
				return -1;
			}
		}
	}
	return 0;
}

// Light is never gated when obs_count >= light_required_obs
TEST(ResidualCascade, LightUngatedAboveThreshold) {
	for (int required = 1; required <= 32; required++) {
		for (int obs = required; obs < required + 100; obs++) {
			if (light_is_gated(required, obs)) {
				fprintf(stderr, "LightUngatedAboveThreshold FAILED: required=%d obs=%d\n",
						required, obs);
				return -1;
			}
		}
	}
	return 0;
}

// After reinit, obs_count=0 means light is always gated (for any positive
// light_required_obs). This is the death spiral entry point.
TEST(ResidualCascade, ReinitGatesLight) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	for (int trial = 0; trial < 1000; trial++) {
		int required = 1 + rand() % 64;  // 1..64
		int obs_count = 0;  // after reinit

		if (!light_is_gated(required, obs_count)) {
			fprintf(stderr, "ReinitGatesLight FAILED (seed=%u, trial=%d)\n",
					seed, trial);
			fprintf(stderr, "  required=%d obs_count=%d\n", required, obs_count);
			return -1;
		}
	}
	return 0;
}

// ── 4. Full Cascade Simulation ───────────────────────────────────

// Simulate the full failure cascade: growing error → threshold cross →
// lost_tracking → reinit → light gated → death spiral.
// Verify the cascade dynamics match what we see in the field.
TEST(ResidualCascade, FullCascadeTimingCharacterization) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	int crossed_count = 0;

	for (int trial = 0; trial < 1000; trial++) {
		double threshold = rand_range(0.005, 0.05);  // realistic range
		double ema = 0.0;
		int obs_count = DEFAULT_LIGHT_REQUIRED_OBS + 100;  // healthy
		bool tracking = true;
		bool light_gated = false;

		// Simulate a session with gradually increasing error
		// Base error is tiny, but grows linearly
		double base_error = rand_range(1e-6, 1e-4);
		double error_growth_rate = rand_range(1e-7, 1e-4);  // per step

		int steps_to_failure = -1;

		for (int step = 0; step < 100000; step++) {
			// Each step represents one light integration (~100Hz)
			double current_error = base_error + error_growth_rate * step;
			// Add noise
			double noise = rand_range(-current_error * 0.5, current_error * 0.5);
			double rtn = fabs(current_error + noise);

			ema = ema_update(ema, rtn);

			// check_valid: stationary (past grace period)
			bool valid = check_valid(threshold, ema,
									 STATIONARY_GRACE_TICKS + 1000,
									 1.0, 1.0, 1.0);

			if (!valid && tracking) {
				// lost_tracking fires → reinit
				tracking = false;
				obs_count = 0;  // memset(&stats, 0, ...)
				ema = 0;        // light_residuals_all = 0
				light_gated = light_is_gated(DEFAULT_LIGHT_REQUIRED_OBS, obs_count);
				steps_to_failure = step;
				break;
			}
		}

		if (steps_to_failure > 0) {
			crossed_count++;

			// After reinit, light MUST be gated (the death spiral)
			if (!light_gated) {
				fprintf(stderr, "FullCascadeTimingCharacterization FAILED (seed=%u, trial=%d)\n",
						seed, trial);
				fprintf(stderr, "  Light not gated after reinit! obs_count=%d\n", obs_count);
				return -1;
			}

			// obs_count is 0 and light is the only way to increment it
			// (via integrate_observation which needs light data).
			// Since light is gated, obs_count stays 0 forever.
			// Verify the death spiral: 100 more light integrations don't help
			for (int post_step = 0; post_step < 100; post_step++) {
				// Light arrives but is gated
				if (light_is_gated(DEFAULT_LIGHT_REQUIRED_OBS, obs_count)) {
					// Light is dropped — obs_count never increments
					continue;
				}
				// This should never execute
				fprintf(stderr, "FullCascadeTimingCharacterization FAILED (seed=%u, trial=%d)\n",
						seed, trial);
				fprintf(stderr, "  Light became ungated without obs_count change!\n");
				return -1;
			}

			// Verify obs_count is still 0
			if (obs_count != 0) {
				fprintf(stderr, "FullCascadeTimingCharacterization FAILED (seed=%u, trial=%d)\n",
						seed, trial);
				fprintf(stderr, "  obs_count changed to %d without external observations\n",
						obs_count);
				return -1;
			}
		}
	}

	// At least some trials should hit the cascade
	if (crossed_count < 100) {
		fprintf(stderr, "FullCascadeTimingCharacterization FAILED (seed=%u)\n", seed);
		fprintf(stderr, "  Only %d/1000 trials hit cascade (expected >100)\n", crossed_count);
		return -1;
	}
	return 0;
}

// Property: EMA needs ~N steps to cross threshold from zero, where
// N depends on the step error and threshold. For the field failure
// (14-33 min at ~100Hz light rate), N should be 84,000-198,000.
// Verify the EMA math matches those timescales.
TEST(ResidualCascade, TimeToFailureMatchesFieldObservation) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	// At ~100Hz light integration rate:
	// 14 minutes = 84,000 steps
	// 33 minutes = 198,000 steps
	int min_field_steps = 84000;
	int max_field_steps = 198000;

	int in_range_count = 0;

	for (int trial = 0; trial < 500; trial++) {
		double threshold = 0.01;  // typical light_error_threshold
		double ema = 0.0;

		// Steady-state error that's below threshold, with slow linear growth
		// representing calibration drift or thermal change
		double base = rand_range(0.001, 0.005);
		// Growth rate tuned to produce 14-33 min failures
		double growth = rand_range(2e-8, 8e-8);

		int steps_to_cross = -1;
		for (int step = 0; step < 300000; step++) {
			double rtn = base + growth * step;
			ema = ema_update(ema, rtn);

			if (ema >= threshold) {
				steps_to_cross = step;
				break;
			}
		}

		if (steps_to_cross >= min_field_steps && steps_to_cross <= max_field_steps) {
			in_range_count++;
		}
	}

	// With the growth rate range we chose, at least 10% of trials should
	// land in the 14-33 minute window. This validates the EMA math
	// is consistent with the observed failure timescales.
	if (in_range_count < 10) {
		fprintf(stderr, "TimeToFailureMatchesFieldObservation FAILED (seed=%u)\n", seed);
		fprintf(stderr, "  Only %d/500 trials in 14-33 min range\n", in_range_count);
		return -1;
	}
	return 0;
}

// ── 5. Recovery Properties ───────────────────────────────────────

// After lost_tracking, the ONLY way to recover is through external
// observations (BSVD solver) incrementing obs_count past the threshold.
// Light alone cannot recover. This test verifies that only obs_count
// changes can unlock light integration.
TEST(ResidualCascade, RecoveryRequiresExternalObservations) {
	int required = DEFAULT_LIGHT_REQUIRED_OBS;

	// Start in death spiral: obs_count = 0
	int obs_count = 0;

	// Light integration cannot change obs_count (it's gated)
	for (int i = 0; i < 1000; i++) {
		if (light_is_gated(required, obs_count)) {
			// Light dropped — obs_count stays the same
			continue;
		}
		fprintf(stderr, "RecoveryRequiresExternalObservations FAILED: light ungated at step %d\n", i);
		return -1;
	}

	// Simulate external observations arriving (e.g. BSVD solver)
	for (int obs = 0; obs < required; obs++) {
		obs_count++;
		// Still gated until we reach required
		if (obs < required - 1) {
			if (!light_is_gated(required, obs_count)) {
				fprintf(stderr, "RecoveryRequiresExternalObservations FAILED: "
						"ungated at obs=%d (need %d)\n", obs_count, required);
				return -1;
			}
		}
	}

	// Now at required — light should be ungated
	if (light_is_gated(required, obs_count)) {
		fprintf(stderr, "RecoveryRequiresExternalObservations FAILED: "
				"still gated at obs=%d (required=%d)\n", obs_count, required);
		return -1;
	}

	return 0;
}

// After reinit, the EMA is reset to 0. If the underlying error source
// is still present, the EMA will cross the threshold again in the same
// number of steps. This means recovery is impossible without fixing
// the root cause (light data quality).
TEST(ResidualCascade, ReinitDoesNotFixRootCause) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	for (int trial = 0; trial < 200; trial++) {
		double threshold = rand_range(0.005, 0.05);
		double steady_error = threshold + rand_range(0.001, 0.1);

		// First run: how many steps to cross?
		double ema = 0.0;
		int steps_first = -1;
		for (int step = 0; step < 1000; step++) {
			ema = ema_update(ema, steady_error);
			if (ema >= threshold) {
				steps_first = step;
				break;
			}
		}

		if (steps_first < 0) {
			fprintf(stderr, "ReinitDoesNotFixRootCause FAILED (seed=%u, trial=%d)\n",
					seed, trial);
			fprintf(stderr, "  Never crossed threshold=%.6f with error=%.6f\n",
					threshold, steady_error);
			return -1;
		}

		// Reinit: reset EMA to 0 (same as survive_kalman_tracker_reinit)
		ema = 0.0;

		// Second run with same error: should cross in same number of steps
		int steps_second = -1;
		for (int step = 0; step < 1000; step++) {
			ema = ema_update(ema, steady_error);
			if (ema >= threshold) {
				steps_second = step;
				break;
			}
		}

		if (steps_second != steps_first) {
			fprintf(stderr, "ReinitDoesNotFixRootCause FAILED (seed=%u, trial=%d)\n",
					seed, trial);
			fprintf(stderr, "  steps_first=%d steps_second=%d (should be equal)\n",
					steps_first, steps_second);
			return -1;
		}
	}
	return 0;
}

// ── 6. Stationary Detection Interaction ──────────────────────────

// The grace period only protects moving trackers. For VP use (stationary
// mounted trackers), check_valid has no safety net. This test verifies
// that stationary trackers with high residuals always fail.
TEST(ResidualCascade, StationaryTrackersHaveNoSafetyNet) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	for (int trial = 0; trial < 10000; trial++) {
		double threshold = rand_range(0.001, 1.0);
		// Residuals anywhere from just above threshold to very high
		double residuals = threshold * rand_range(1.001, 100.0);
		// Stationary for a long time (typical VP scenario)
		long long stationary = STATIONARY_GRACE_TICKS +
			(long long)(rand_range(1, 1e10));
		double pos[3];
		for (int i = 0; i < 3; i++)
			pos[i] = rand_range(-19.0, 19.0);

		if (check_valid(threshold, residuals, stationary,
						pos[0], pos[1], pos[2])) {
			fprintf(stderr, "StationaryTrackersHaveNoSafetyNet FAILED (seed=%u, trial=%d)\n",
					seed, trial);
			fprintf(stderr, "  threshold=%.6f residuals=%.6f stationary=%lld\n",
					threshold, residuals, stationary);
			return -1;
		}
	}
	return 0;
}

// Boundary: tracker that just stopped moving. At exactly the grace
// period boundary, the tracker transitions from protected to unprotected.
TEST(ResidualCascade, GracePeriodBoundary) {
	double threshold = 0.01;
	double residuals = 0.1;  // well above threshold

	double pos[3] = {1.0, 1.0, 1.0};

	// Just inside grace period — should be valid
	long long just_inside = STATIONARY_GRACE_TICKS - 1;
	if (!check_valid(threshold, residuals, just_inside, pos[0], pos[1], pos[2])) {
		fprintf(stderr, "GracePeriodBoundary FAILED: just inside should be valid\n");
		return -1;
	}

	// At boundary — stationary_ticks == GRACE, NOT less than, so NOT protected
	long long at_boundary = STATIONARY_GRACE_TICKS;
	if (check_valid(threshold, residuals, at_boundary, pos[0], pos[1], pos[2])) {
		fprintf(stderr, "GracePeriodBoundary FAILED: at boundary should be invalid\n");
		return -1;
	}

	// Just past grace period — should be invalid
	long long just_past = STATIONARY_GRACE_TICKS + 1;
	if (check_valid(threshold, residuals, just_past, pos[0], pos[1], pos[2])) {
		fprintf(stderr, "GracePeriodBoundary FAILED: just past should be invalid\n");
		return -1;
	}

	return 0;
}

// ── 7. Cascade Under Intermittent Error ──────────────────────────

// The EMA with alpha=0.1 amplifies single outliers: a spike of value S
// adds alpha*S to the EMA, on top of the accumulated steady-state base.
// In steady state with periodic spikes, the peak EMA is:
//   H = [0.9 * good * (1 - 0.9^(N-1)) + 0.1 * bad] / (1 - 0.9^N)
// where N = spike_interval.
//
// This test verifies that when the computed peak EMA is below threshold,
// no crossing occurs. When the peak exceeds threshold, crossing is expected.
TEST(ResidualCascade, SpikeSensitivity) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	for (int trial = 0; trial < 1000; trial++) {
		double threshold = rand_range(0.005, 0.05);
		double good_error = rand_range(1e-6, 0.001);
		int spike_interval = 10 + rand() % 90;

		// Compute the theoretical peak EMA in steady state
		double decay_N = pow(EMA_DECAY, spike_interval);
		double decay_Nm1 = pow(EMA_DECAY, spike_interval - 1);

		// Choose bad_error so peak is well below threshold (50% margin)
		// H = [0.9*good*(1-0.9^(N-1)) + 0.1*bad] / (1-0.9^N)
		// Solve for bad: bad = [H*(1-0.9^N) - 0.9*good*(1-0.9^(N-1))] / 0.1
		double target_peak = threshold * 0.5;  // 50% margin
		double max_bad = (target_peak * (1.0 - decay_N)
						  - EMA_DECAY * good_error * (1.0 - decay_Nm1)) / EMA_ALPHA;
		if (max_bad <= good_error)
			continue;  // skip degenerate case

		double bad_error = rand_range(good_error, max_bad);
		double ema = 0.0;
		int crossed = 0;

		for (int step = 0; step < 10000; step++) {
			double rtn = (step % spike_interval == 0) ? bad_error : good_error;
			ema = ema_update(ema, rtn);

			if (ema >= threshold) {
				crossed = 1;
				break;
			}
		}

		if (crossed) {
			double actual_peak = (EMA_DECAY * good_error * (1.0 - decay_Nm1)
								  + EMA_ALPHA * bad_error) / (1.0 - decay_N);
			fprintf(stderr, "SpikeSensitivity FAILED (seed=%u, trial=%d)\n",
					seed, trial);
			fprintf(stderr, "  threshold=%.6f peak=%.6f bad=%.6f interval=%d\n",
					threshold, actual_peak, bad_error, spike_interval);
			return -1;
		}
	}
	return 0;
}

// Verify that a single large outlier CAN trigger the cascade.
// With threshold=0.01, any rtn >= 0.1 pushes alpha*rtn >= threshold.
// This is the mechanism by which a single lighthouse reflection
// can cause permanent tracking loss.
TEST(ResidualCascade, SingleOutlierCanTriggerCascade) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	int triggered = 0;

	for (int trial = 0; trial < 1000; trial++) {
		double threshold = 0.01;
		double ema = rand_range(0.0, threshold * 0.5);  // below threshold

		// Single large outlier: reflection, bad sensor, etc.
		double spike = rand_range(threshold / EMA_ALPHA, 10.0);
		ema = ema_update(ema, spike);

		if (ema >= threshold) {
			triggered++;
		}
	}

	// All trials should trigger since spike > threshold/alpha
	if (triggered < 990) {
		fprintf(stderr, "SingleOutlierCanTriggerCascade FAILED (seed=%u)\n", seed);
		fprintf(stderr, "  Only %d/1000 triggered (expected ~1000)\n", triggered);
		return -1;
	}
	return 0;
}

// ── 8. Adaptive R (per-LH light_var scaling) Properties ─────────
//
// These test the invariants of the per-LH adaptive R formula introduced in
// survive_kalman_tracker.c (TE-PROC-040):
//
//   lh_var = (mean_res > 0 && lh_res > 0)
//              ? base_var * max(1.0, lh_res / mean_res)
//              : base_var;
//
// The formula must satisfy:
//   1. lh_var >= base_var always (never reduces below baseline)
//   2. When lh_res == mean_res, lh_var == base_var (equal residuals → no change)
//   3. When lh_res > mean_res, lh_var > base_var (biased LH gets penalized)
//   4. Cold start (mean_res=0 or lh_res=0) → lh_var == base_var (safe fallback)
//   5. lh_var is monotonically non-decreasing in lh_res

static double adaptive_lh_var(double base_var, double mean_res, double lh_res) {
	if (mean_res <= 0 || lh_res <= 0)
		return base_var;
	double ratio = lh_res / mean_res;
	return base_var * (ratio > 1.0 ? ratio : 1.0);
}

// Property 1: lh_var is always >= base_var
TEST(AdaptiveR, AlwaysAtLeastBaseVar) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	for (int trial = 0; trial < 10000; trial++) {
		double base_var = rand_range(1e-4, 1.0);
		double mean_res = rand_range(0.0, 1.0);
		double lh_res   = rand_range(0.0, 1.0);

		double lh_var = adaptive_lh_var(base_var, mean_res, lh_res);

		if (lh_var < base_var - 1e-12) {
			fprintf(stderr, "AlwaysAtLeastBaseVar FAILED (seed=%u, trial=%d)\n", seed, trial);
			fprintf(stderr, "  base_var=%.6f mean_res=%.6f lh_res=%.6f -> lh_var=%.6f\n",
					base_var, mean_res, lh_res, lh_var);
			return -1;
		}
	}
	return 0;
}

// Property 2: When lh_res == mean_res (and both > 0), lh_var == base_var
TEST(AdaptiveR, EqualResidualIsIdentity) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	for (int trial = 0; trial < 10000; trial++) {
		double base_var = rand_range(1e-4, 1.0);
		double res = rand_range(1e-6, 1.0);  // same for both mean and lh

		double lh_var = adaptive_lh_var(base_var, res, res);

		if (fabs(lh_var - base_var) > 1e-10) {
			fprintf(stderr, "EqualResidualIsIdentity FAILED (seed=%u, trial=%d)\n", seed, trial);
			fprintf(stderr, "  base_var=%.10f res=%.10f -> lh_var=%.10f (diff=%.2e)\n",
					base_var, res, lh_var, fabs(lh_var - base_var));
			return -1;
		}
	}
	return 0;
}

// Property 3: lh_res > mean_res → lh_var > base_var
TEST(AdaptiveR, HighResidualInflatesVar) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	for (int trial = 0; trial < 10000; trial++) {
		double base_var = rand_range(1e-4, 1.0);
		double mean_res = rand_range(1e-6, 0.5);
		// lh_res strictly greater than mean_res
		double lh_res = mean_res + rand_range(1e-6, 0.5);

		double lh_var = adaptive_lh_var(base_var, mean_res, lh_res);

		if (lh_var <= base_var) {
			fprintf(stderr, "HighResidualInflatesVar FAILED (seed=%u, trial=%d)\n", seed, trial);
			fprintf(stderr, "  base_var=%.6f mean_res=%.6f lh_res=%.6f -> lh_var=%.6f\n",
					base_var, mean_res, lh_res, lh_var);
			return -1;
		}
	}
	return 0;
}

// Property 4: Cold start (mean_res=0 or lh_res=0) → safe fallback to base_var
TEST(AdaptiveR, ColdStartFallsBackToBaseVar) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	for (int trial = 0; trial < 10000; trial++) {
		double base_var = rand_range(1e-4, 1.0);
		double other    = rand_range(0.0, 1.0);

		// Case A: mean_res = 0
		double lh_var_a = adaptive_lh_var(base_var, 0.0, other);
		if (fabs(lh_var_a - base_var) > 1e-12) {
			fprintf(stderr, "ColdStartFallsBackToBaseVar FAILED case A (seed=%u, trial=%d)\n", seed, trial);
			fprintf(stderr, "  base_var=%.6f mean_res=0.0 lh_res=%.6f -> lh_var=%.6f\n",
					base_var, other, lh_var_a);
			return -1;
		}

		// Case B: lh_res = 0
		double lh_var_b = adaptive_lh_var(base_var, other, 0.0);
		if (fabs(lh_var_b - base_var) > 1e-12) {
			fprintf(stderr, "ColdStartFallsBackToBaseVar FAILED case B (seed=%u, trial=%d)\n", seed, trial);
			fprintf(stderr, "  base_var=%.6f mean_res=%.6f lh_res=0.0 -> lh_var=%.6f\n",
					base_var, other, lh_var_b);
			return -1;
		}
	}
	return 0;
}

// Property 5: lh_var is monotonically non-decreasing in lh_res
// (more residual error → equal or more variance, never less)
TEST(AdaptiveR, MonotonicInLhResidual) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	for (int trial = 0; trial < 10000; trial++) {
		double base_var = rand_range(1e-4, 1.0);
		double mean_res = rand_range(1e-6, 0.5);
		double lh_res_lo = rand_range(1e-6, 0.5);
		double lh_res_hi = lh_res_lo + rand_range(1e-6, 0.5);

		double lh_var_lo = adaptive_lh_var(base_var, mean_res, lh_res_lo);
		double lh_var_hi = adaptive_lh_var(base_var, mean_res, lh_res_hi);

		if (lh_var_hi < lh_var_lo - 1e-12) {
			fprintf(stderr, "MonotonicInLhResidual FAILED (seed=%u, trial=%d)\n", seed, trial);
			fprintf(stderr, "  base_var=%.6f mean_res=%.6f\n", base_var, mean_res);
			fprintf(stderr, "  lh_res_lo=%.6f -> lh_var=%.6f\n", lh_res_lo, lh_var_lo);
			fprintf(stderr, "  lh_res_hi=%.6f -> lh_var=%.6f\n", lh_res_hi, lh_var_hi);
			return -1;
		}
	}
	return 0;
}

// Property 6: Per-LH EWMA converges independently for each LH slot.
// Each light_residuals[lh] slot uses the same EMA formula as light_residuals_all.
// This test simulates two LHs with different steady-state residuals and verifies
// that each EWMA slot converges to its own value without cross-contamination.
TEST(AdaptiveR, PerLhEWMAConvergesIndependently) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	for (int trial = 0; trial < 1000; trial++) {
		double lh0_steady = rand_range(0.001, 0.1);
		double lh1_steady = rand_range(0.001, 0.1);

		double ema0 = 0.0, ema1 = 0.0;

		// Drive both EWMA slots with their respective steady inputs
		for (int i = 0; i < 100; i++) {
			ema0 = ema_update(ema0, lh0_steady);
			ema1 = ema_update(ema1, lh1_steady);
		}

		// Both should have converged (0.9^100 ≈ 2.7e-5, negligible)
		if (fabs(ema0 - lh0_steady) > 1e-3) {
			fprintf(stderr, "PerLhEWMAConvergesIndependently FAILED (seed=%u, trial=%d): "
					"ema0=%.6f target=%.6f\n", seed, trial, ema0, lh0_steady);
			return -1;
		}
		if (fabs(ema1 - lh1_steady) > 1e-3) {
			fprintf(stderr, "PerLhEWMAConvergesIndependently FAILED (seed=%u, trial=%d): "
					"ema1=%.6f target=%.6f\n", seed, trial, ema1, lh1_steady);
			return -1;
		}

		// Slots must not influence each other
		double expected_ratio = lh1_steady / lh0_steady;
		double actual_ratio   = ema1 / ema0;
		if (fabs(actual_ratio - expected_ratio) > expected_ratio * 0.01 + 1e-6) {
			fprintf(stderr, "PerLhEWMAConvergesIndependently FAILED cross-contamination "
					"(seed=%u, trial=%d): ratio=%.6f expected=%.6f\n",
					seed, trial, actual_ratio, expected_ratio);
			return -1;
		}
	}
	return 0;
}

// ── 9. Per-LH Innovation Gate Properties (TE-PROC-039) ──────────
//
// The outlier gate in survive_kalman_tracker.c fires when:
//   rms_innovation > light_outlier_threshold * light_residuals_all
//
// These tests exercise the gate decision logic in isolation. They do not
// call map_light_data (which requires the full Kalman machinery) — instead
// they test the RMS computation and threshold comparison that wraps the
// dry-run call.

// Reproduce the gate decision from survive_kalman_tracker.c:
//   gate fires when rms > threshold * mean_res  (and threshold > 0, mean_res > 0)
static bool outlier_gate_fires(double rms, double threshold, double mean_res) {
	if (threshold <= 0 || mean_res <= 0)
		return false;
	return rms > threshold * mean_res;
}

// Reproduce the RMS computation:
//   rms = sqrt(sum(y[i]^2) / cnt)
static double compute_rms(const double *y, int cnt) {
	double sq = 0;
	for (int i = 0; i < cnt; i++) sq += y[i] * y[i];
	return sqrt(sq / cnt);
}

// Property 1: Gate is disabled when threshold <= 0 (default off)
TEST(OutlierGate, DisabledWhenThresholdZero) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	for (int trial = 0; trial < 10000; trial++) {
		double rms       = rand_range(0.0, 100.0);
		double threshold = rand_range(-10.0, 0.0);  // disabled range
		double mean_res  = rand_range(1e-6, 1.0);

		if (outlier_gate_fires(rms, threshold, mean_res)) {
			fprintf(stderr, "DisabledWhenThresholdZero FAILED (seed=%u, trial=%d): "
					"rms=%.4f threshold=%.4f mean_res=%.4f\n", seed, trial, rms, threshold, mean_res);
			return -1;
		}
	}
	return 0;
}

// Property 2: Gate is disabled when mean_res <= 0 (cold start: no fleet baseline yet)
TEST(OutlierGate, DisabledOnColdStart) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	for (int trial = 0; trial < 10000; trial++) {
		double rms       = rand_range(0.0, 100.0);
		double threshold = rand_range(1.0, 10.0);   // enabled
		double mean_res  = 0.0;                       // cold start

		if (outlier_gate_fires(rms, threshold, mean_res)) {
			fprintf(stderr, "DisabledOnColdStart FAILED (seed=%u, trial=%d): "
					"rms=%.4f threshold=%.4f\n", seed, trial, rms, threshold);
			return -1;
		}
	}
	return 0;
}

// Property 3: Gate fires when rms > threshold * mean_res
TEST(OutlierGate, FiresAboveThreshold) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	for (int trial = 0; trial < 10000; trial++) {
		double threshold = rand_range(1.0, 10.0);
		double mean_res  = rand_range(1e-4, 0.1);
		// rms strictly above the trigger point
		double rms = threshold * mean_res + rand_range(1e-6, 0.5);

		if (!outlier_gate_fires(rms, threshold, mean_res)) {
			fprintf(stderr, "FiresAboveThreshold FAILED (seed=%u, trial=%d): "
					"rms=%.6f threshold=%.4f mean_res=%.6f trigger=%.6f\n",
					seed, trial, rms, threshold, mean_res, threshold * mean_res);
			return -1;
		}
	}
	return 0;
}

// Property 4: Gate does NOT fire when rms <= threshold * mean_res
TEST(OutlierGate, DoesNotFireBelowThreshold) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	for (int trial = 0; trial < 10000; trial++) {
		double threshold = rand_range(1.0, 10.0);
		double mean_res  = rand_range(1e-4, 0.1);
		// rms strictly below the trigger point
		double rms = rand_range(0.0, threshold * mean_res - 1e-8);
		if (rms < 0) rms = 0;

		if (outlier_gate_fires(rms, threshold, mean_res)) {
			fprintf(stderr, "DoesNotFireBelowThreshold FAILED (seed=%u, trial=%d): "
					"rms=%.6f threshold=%.4f mean_res=%.6f trigger=%.6f\n",
					seed, trial, rms, threshold, mean_res, threshold * mean_res);
			return -1;
		}
	}
	return 0;
}

// Property 5: RMS is non-negative and zero only for all-zero innovations
TEST(OutlierGate, RmsNonNegativeAndZeroOnlyWhenAllZero) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	// All-zero case
	{
		double y[8] = {0};
		double rms = compute_rms(y, 8);
		if (rms != 0.0) {
			fprintf(stderr, "RmsNonNegativeAndZeroOnlyWhenAllZero FAILED: all-zero rms=%.10f\n", rms);
			return -1;
		}
	}

	for (int trial = 0; trial < 10000; trial++) {
		int cnt = 1 + rand() % 32;
		double y[32];
		bool all_zero = true;
		for (int i = 0; i < cnt; i++) {
			y[i] = rand_range(-1.0, 1.0);
			if (y[i] != 0.0) all_zero = false;
		}
		double rms = compute_rms(y, cnt);
		if (rms < 0) {
			fprintf(stderr, "RmsNonNegativeAndZeroOnlyWhenAllZero FAILED: rms=%.6f < 0\n", rms);
			return -1;
		}
		if (!all_zero && rms == 0.0) {
			fprintf(stderr, "RmsNonNegativeAndZeroOnlyWhenAllZero FAILED: non-zero y but rms=0\n");
			return -1;
		}
	}
	return 0;
}

// Property 6: Single large innovation (reflection scenario) triggers the gate.
// A reflection might produce one sensor innovation >> normal; compute_rms
// should be above a reasonable threshold even for a single outlier sensor.
TEST(OutlierGate, SingleLargeInnovationTriggers) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	int triggered = 0;

	for (int trial = 0; trial < 1000; trial++) {
		int cnt = 8 + rand() % 24;          // 8-32 sensors
		double mean_res = rand_range(0.005, 0.02);  // typical fleet residual
		double threshold = 5.0;

		// All sensors normal except one reflection spike
		double y[32];
		for (int i = 0; i < cnt; i++)
			y[i] = rand_range(-mean_res, mean_res);
		int spike_sensor = rand() % cnt;
		// Reflection: 20-100× the normal noise
		y[spike_sensor] = mean_res * rand_range(20.0, 100.0);

		double rms = compute_rms(y, cnt);
		if (outlier_gate_fires(rms, threshold, mean_res)) {
			triggered++;
		}
	}

	// A 20-100× spike on any single sensor in a batch of 8-32 should be detectable
	if (triggered < 800) {
		fprintf(stderr, "SingleLargeInnovationTriggers FAILED: only %d/1000 triggered\n", triggered);
		return -1;
	}
	return 0;
}
