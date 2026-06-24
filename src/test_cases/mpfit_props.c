// Regression test for the mpfit numerical-derivative NaN hang (Stagehand patch).
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
// This test drives mpfit() through the numerical-derivative path with a user
// function that returns a non-finite residual once perturbed by the
// finite-difference step, and checks mpfit() returns promptly with a finite
// result. It also confirms a well-conditioned fit on the same code path still
// converges normally (no regression on the happy path).

#include "test_case.h"
#include <math.h>
#include "mpfit/mpfit.h"
#include <stdio.h>

// y = p0 * x, with no analytic derivative supplied (dvec ignored) so mpfit
// falls back to mp_fdjac2's numerical derivative. Once p0 is perturbed past
// a threshold the residual goes non-finite, simulating a degenerate
// real-world residual (e.g. right after a Reinit-triggered scene reset).
struct LinearData {
	FLT *x;
	FLT *y;
	int n;
};

static int linear_degenerate(int m, int n, FLT *p, FLT *dy, FLT **dvec, void *priv) {
	struct LinearData *d = (struct LinearData *)priv;
	for (int i = 0; i < m; i++) {
		if (p[0] > 1e10) {
			dy[i] = INFINITY;
		} else {
			dy[i] = d->y[i] - p[0] * d->x[i];
		}
	}
	return 0;
}

static int linear_well_conditioned(int m, int n, FLT *p, FLT *dy, FLT **dvec, void *priv) {
	struct LinearData *d = (struct LinearData *)priv;
	for (int i = 0; i < m; i++) {
		dy[i] = d->y[i] - (p[0] * d->x[i] + p[1]);
	}
	return 0;
}

TEST(MpfitProps, NonFiniteResidualDoesNotHang) {
	FLT x[5] = {0, 1, 2, 3, 4};
	FLT y[5] = {0, 2, 4, 6, 8};
	struct LinearData d = {x, y, 5};

	// Start absurdly large so the finite-difference probe step pushes p[0]
	// past the 1e10 threshold and the user function returns Inf.
	FLT p[1] = {1e15};
	mp_result result = {0};
	mp_config cfg = {0};
	cfg.maxiter = 50;

	int status = mpfit(linear_degenerate, 5, 1, p, 0, &cfg, &d, &result);

	// The point of the patch is that this returns at all (no hang) and
	// doesn't leave NaN/Inf in the solver's outputs.
	if (!isfinite(result.bestnorm) || !isfinite(p[0])) {
		fprintf(stderr, "NonFiniteResidualDoesNotHang FAILED: status=%d bestnorm=%f p0=%f\n", status, result.bestnorm,
				p[0]);
		return -1;
	}
	return 0;
}

TEST(MpfitProps, WellConditionedFitStillConverges) {
	// y = 2x + 1, same numerical-derivative code path as the test above.
	FLT x[6] = {0, 1, 2, 3, 4, 5};
	FLT y[6] = {1, 3, 5, 7, 9, 11};
	struct LinearData d = {x, y, 6};

	FLT p[2] = {0, 0};
	mp_result result = {0};

	int status = mpfit(linear_well_conditioned, 6, 2, p, 0, 0, &d, &result);

	if (status <= 0) {
		fprintf(stderr, "WellConditionedFitStillConverges FAILED: mpfit status=%d\n", status);
		return -1;
	}
	if (fabs(p[0] - 2.0) > 1e-4 || fabs(p[1] - 1.0) > 1e-4) {
		fprintf(stderr, "WellConditionedFitStillConverges FAILED: p=[%.6f,%.6f], want [2,1]\n", p[0], p[1]);
		return -1;
	}
	return 0;
}
