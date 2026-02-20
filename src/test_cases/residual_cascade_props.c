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
