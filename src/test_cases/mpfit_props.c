// Property tests for mpfit's robustness to non-finite residuals on the
// numerical-derivative path (Stagehand patch).
//
// redist/mpfit/mpfit.c's mp_fdjac2() computes a finite-difference Jacobian
// when no analytic derivative is supplied. Its only protection against a
// degenerate (non-finite) residual poisoning fjac was assert(isfinite(...)),
// which -DNDEBUG (the Pi's Release build) strips entirely. A NaN/Inf in fjac
// poisons fnorm/ratio in mpfit()'s outer Levenberg-Marquardt loop; since the
// iteration counter only advances inside the `ratio >= p0001` success branch,
// and every comparison against a NaN ratio is false, the loop never
// increments `iter`, the maxiter check never fires, and mpfit() spins at
// ~100% CPU forever instead of returning a clean failure status.
//
// The property we care about: for ANY user function on the numerical-
// derivative path that goes non-finite somewhere in the search space,
// mpfit() still terminates promptly with a finite result. This is fuzzed
// across randomized trigger thresholds, starting points, and one-/two-sided
// derivative modes, rather than pinned to the single mechanism originally
// found by hand -- a narrower fix that only patched that one shape would
// still fail this test under a different trial.
//
// A second property, orthogonal to the first: well-conditioned fits on the
// same code path still converge to the correct answer regardless of where a
// (never-triggered) non-finite threshold happens to sit -- i.e. the guard
// must not perturb the happy path.

#include "test_case.h"
#include <math.h>
#include "mpfit/mpfit.h"
#include <stdio.h>
#include <time.h>

#define N_TRIALS 500

static FLT rand_range(FLT min, FLT max) { return min + (max - min) * ((FLT)rand() / (FLT)RAND_MAX); }

// y = p0 * x, with no analytic derivative supplied so mpfit always falls
// back to mp_fdjac2's numerical derivative. The residual at the *current*
// search point (x[0]) always stays finite -- only the perturbed evaluation
// fdjac2 takes at x[0]+h (or x[0]-h for the two-sided derivative) crosses a
// randomized threshold and goes non-finite. This isolates the actual bug
// mechanism: a finite fvec/fnorm at the working point, but a non-finite
// fjac entry from the finite-difference probe step, which is what poisons
// ratio in the outer loop. (A residual that's already non-finite at the
// unperturbed point takes a different, already-correct early-exit path and
// would not exercise this guard.)
struct DegenerateCtx {
	FLT *x;
	FLT *y;
	int n;
	FLT trigger_threshold;
	int use_inf; // 1 -> Inf, 0 -> NaN
	FLT last_finite_call_p0;
};

static int linear_degenerate(int m, int n, FLT *p, FLT *dy, FLT **dvec, void *priv) {
	struct DegenerateCtx *d = (struct DegenerateCtx *)priv;
	// Only the probe step (p[0] != the most recent "settled" value fdjac2
	// calls back with) is allowed to go non-finite; treat any call whose
	// magnitude exceeds the threshold as a probe perturbation.
	int is_probe = fabs(p[0] - d->last_finite_call_p0) > 1e-30 && fabs(p[0]) > d->trigger_threshold;
	for (int i = 0; i < m; i++) {
		if (is_probe) {
			dy[i] = d->use_inf ? (FLT)INFINITY : (FLT)NAN;
		} else {
			dy[i] = d->y[i] - p[0] * d->x[i];
		}
	}
	if (!is_probe)
		d->last_finite_call_p0 = p[0];
	return 0;
}

struct LinearCtx {
	FLT *x;
	FLT *y;
};

static int linear_well_conditioned(int m, int n, FLT *p, FLT *dy, FLT **dvec, void *priv) {
	struct LinearCtx *d = (struct LinearCtx *)priv;
	for (int i = 0; i < m; i++) {
		dy[i] = d->y[i] - (p[0] * d->x[i] + p[1]);
	}
	return 0;
}

// 1. For randomized non-finite trigger points, starting magnitudes, and
//    derivative sidedness, mpfit() always returns promptly with a finite
//    result -- never hangs, never leaves NaN/Inf in its output.
TEST(MpfitProps, NonFiniteResidualNeverHangsOrLeavesNonFinite) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	FLT x[5] = {0, 1, 2, 3, 4};
	FLT y[5] = {0, 2, 4, 6, 8};

	for (int trial = 0; trial < N_TRIALS; trial++) {
		// Starting magnitude spans many orders of magnitude, since the
		// finite-difference step size h = eps*|x| scales with it -- the
		// probe perturbation needs to actually cross the threshold.
		FLT start_mag = rand_range(1e-2, 1e15);
		FLT start_sign = (rand() % 2) ? 1 : -1;
		FLT p[1] = {start_sign * start_mag};

		// Threshold sits strictly between the starting magnitude and the
		// perturbed probe magnitude (h = eps*|p0|, eps ~ 1.49e-8), so the
		// unperturbed base evaluation stays finite but the probe reliably
		// crosses it.
		struct DegenerateCtx d = {x, y, 5, start_mag * (FLT)(1.0 + 7e-9), rand() % 2, p[0]};

		mp_result result = {0};
		mp_config cfg = {0};
		cfg.maxiter = 50;
		// One- vs two-sided numerical derivative: side=0 (auto/one-sided) or
		// side=2 (two-sided), exercising both fdjac2 write sites the patch
		// guards.
		mp_par par = {0};
		par.side = (rand() % 2) ? 2 : 0;

		clock_t t0 = clock();
		int status = mpfit(linear_degenerate, 5, 1, p, &par, &cfg, &d, &result);
		double elapsed = (double)(clock() - t0) / CLOCKS_PER_SEC;

		if (elapsed > 2.0) {
			fprintf(stderr,
					"NonFiniteResidualNeverHangsOrLeavesNonFinite FAILED (seed=%u, trial=%d): "
					"took %.3fs (hang) threshold=%g use_inf=%d start=%g side=%d\n",
					seed, trial, elapsed, (double)d.trigger_threshold, d.use_inf, (double)start_mag, par.side);
			return -1;
		}
		if (!isfinite(result.bestnorm) || !isfinite(p[0])) {
			fprintf(stderr,
					"NonFiniteResidualNeverHangsOrLeavesNonFinite FAILED (seed=%u, trial=%d): "
					"status=%d bestnorm=%f p0=%f threshold=%g use_inf=%d start=%g side=%d\n",
					seed, trial, status, result.bestnorm, p[0], (double)d.trigger_threshold, d.use_inf,
					(double)start_mag, par.side);
			return -1;
		}
	}
	return 0;
}

// 2. The non-finite guard must not perturb the happy path: well-conditioned
//    fits across randomized slopes/intercepts/starting points still converge
//    to the correct answer.
TEST(MpfitProps, WellConditionedFitStillConverges) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	for (int trial = 0; trial < N_TRIALS; trial++) {
		FLT slope = rand_range(-10.0, 10.0);
		FLT intercept = rand_range(-10.0, 10.0);

		FLT x[6], y[6];
		for (int i = 0; i < 6; i++) {
			x[i] = (FLT)i;
			y[i] = slope * x[i] + intercept;
		}
		struct LinearCtx d = {x, y};

		FLT p[2] = {rand_range(-5.0, 5.0), rand_range(-5.0, 5.0)};
		mp_result result = {0};

		int status = mpfit(linear_well_conditioned, 6, 2, p, 0, 0, &d, &result);

		if (status <= 0) {
			fprintf(stderr, "WellConditionedFitStillConverges FAILED (seed=%u, trial=%d): mpfit status=%d\n", seed,
					trial, status);
			return -1;
		}
		if (fabs(p[0] - slope) > 1e-3 || fabs(p[1] - intercept) > 1e-3) {
			fprintf(stderr,
					"WellConditionedFitStillConverges FAILED (seed=%u, trial=%d): "
					"p=[%.6f,%.6f], want [%.6f,%.6f]\n",
					seed, trial, p[0], p[1], slope, intercept);
			return -1;
		}
	}
	return 0;
}
