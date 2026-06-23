// Property-based tests for the pulse-timing decode layer in
// disambiguator_statebased.c — find_acode, overlap_area, overlaps, and
// calculate_error. These are the pure functions that turn a raw USB
// LightcapElement (sensor_id, pulse length, timestamp) into an acode
// classification and overlap/error metrics, upstream of all sync/sweep
// state tracking. No SurviveContext/SurviveObject required.
//
// disable_lighthouse, find_inliers, find_relative_offset, and the rest of
// the state machine are not covered here: they need Disambiguator_data_t /
// SurviveContext and are out of scope for a pure-function property suite.

#include "test_case.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <survive_types.h>
#include <time.h>

#define N_TRIALS 10000

// Declared non-static in disambiguator_statebased.c for this test file.
int find_acode(uint32_t pulseLen);
int32_t overlap_area(const LightcapElement *a, const LightcapElement *b);
bool overlaps(const LightcapElement *a, const LightcapElement *b);
uint32_t calculate_error(int target_acode, const LightcapElement *le);
extern const int DATA_BIT;

#define ACODE_TIMING(acode) ((3000 + ((acode) & 1) * 500 + (((acode) >> 1) & 1) * 1000 + (((acode) >> 2) & 1) * 2000) - 250)

static uint32_t rand_u32(uint32_t min, uint32_t max) {
	return min + (uint32_t)((double)(max - min) * ((double)rand() / (double)RAND_MAX));
}

// 1. find_acode never returns outside its documented range [-1, 7].
TEST(DisambiguatorProps, FindAcodeRange) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	for (int t = 0; t < N_TRIALS; t++) {
		uint32_t pulseLen = rand_u32(0, 20000);
		int acode = find_acode(pulseLen);
		if (acode < -1 || acode > 7) {
			fprintf(stderr, "FindAcodeRange FAILED (seed=%u, pulseLen=%u): acode=%d out of [-1,7]\n",
					seed, pulseLen, acode);
			return -1;
		}
	}
	return 0;
}

// 2. find_acode is monotonic non-decreasing in pulseLen (each 500-tick
//    bucket maps to the next acode; never jumps backward).
TEST(DisambiguatorProps, FindAcodeMonotonic) {
	int prev = find_acode(0);
	for (uint32_t pulseLen = 1; pulseLen < 20000; pulseLen++) {
		int acode = find_acode(pulseLen);
		if (acode != -1 && prev != -1 && acode < prev) {
			fprintf(stderr, "FindAcodeMonotonic FAILED at pulseLen=%u: acode=%d < prev=%d\n",
					pulseLen, acode, prev);
			return -1;
		}
		if (acode != -1)
			prev = acode;
	}
	return 0;
}

// 3. find_acode at each bucket's documented center matches the expected
//    acode (regression-pins the offset=50, 500-tick-wide bucket table).
TEST(DisambiguatorProps, FindAcodeBucketCenters) {
	struct {
		uint32_t pulseLen;
		int expected;
	} cases[] = {
		{2500 + 50 + 1, 0}, {3000 + 50 + 1, 1}, {3500 + 50 + 1, 2}, {4000 + 50 + 1, 3},
		{4500 + 50 + 1, 4}, {5000 + 50 + 1, 5}, {5500 + 50 + 1, 6}, {6000 + 50 + 1, 7},
	};
	for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
		int acode = find_acode(cases[i].pulseLen);
		if (acode != cases[i].expected) {
			fprintf(stderr, "FindAcodeBucketCenters FAILED: pulseLen=%u expected=%d got=%d\n",
					cases[i].pulseLen, cases[i].expected, acode);
			return -1;
		}
	}
	// Below the lowest bucket and above the highest -> reject (-1), never
	// silently clamp into a valid acode. The lower bound is exclusive
	// (pulseLen < 2500+offset), so 2500+offset itself is bucket 0, not -1.
	if (find_acode(0) != -1 || find_acode(2500 + 50 - 1) != -1 || find_acode(6500 + 50) != -1) {
		fprintf(stderr, "FindAcodeBucketCenters FAILED: out-of-range pulseLen did not return -1\n");
		return -1;
	}
	return 0;
}

// 4. overlap_area is symmetric: overlap_area(a, b) == overlap_area(b, a).
TEST(DisambiguatorProps, OverlapAreaSymmetric) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	for (int t = 0; t < N_TRIALS; t++) {
		LightcapElement a = {.sensor_id = 0, .timestamp = rand_u32(0, 1000000), .length = (uint16_t)rand_u32(0, 10000)};
		LightcapElement b = {.sensor_id = 1, .timestamp = rand_u32(0, 1000000), .length = (uint16_t)rand_u32(0, 10000)};

		int32_t ab = overlap_area(&a, &b);
		int32_t ba = overlap_area(&b, &a);
		if (ab != ba) {
			fprintf(stderr, "OverlapAreaSymmetric FAILED (seed=%u, trial=%d): overlap_area(a,b)=%d != overlap_area(b,a)=%d\n",
					seed, t, ab, ba);
			return -1;
		}
	}
	return 0;
}

// 5. overlap_area of an element with itself equals its own length (full
//    self-overlap); never negative.
TEST(DisambiguatorProps, OverlapAreaSelfAndNonNegative) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	for (int t = 0; t < N_TRIALS; t++) {
		LightcapElement a = {.sensor_id = 0, .timestamp = rand_u32(0, 1000000), .length = (uint16_t)rand_u32(1, 10000)};

		int32_t self_overlap = overlap_area(&a, &a);
		if (self_overlap != a.length) {
			fprintf(stderr, "OverlapAreaSelfAndNonNegative FAILED (seed=%u, trial=%d): self overlap=%d expected length=%u\n",
					seed, t, self_overlap, a.length);
			return -1;
		}

		LightcapElement b = {.sensor_id = 1, .timestamp = rand_u32(0, 1000000), .length = (uint16_t)rand_u32(0, 10000)};
		int32_t ab = overlap_area(&a, &b);
		if (ab < 0) {
			fprintf(stderr, "OverlapAreaSelfAndNonNegative FAILED (seed=%u, trial=%d): overlap=%d < 0\n",
					seed, t, ab);
			return -1;
		}
	}
	return 0;
}

// 6. overlaps(a, b) is true iff overlap_area(a, b) exceeds half of a's own
//    length (the actual contract in disambiguator_statebased.c — overlap
//    is measured relative to the first argument, not a symmetric >0 test).
TEST(DisambiguatorProps, OverlapsConsistentWithArea) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	for (int t = 0; t < N_TRIALS; t++) {
		LightcapElement a = {.sensor_id = 0, .timestamp = rand_u32(0, 100000), .length = (uint16_t)rand_u32(1, 5000)};
		LightcapElement b = {.sensor_id = 1, .timestamp = rand_u32(0, 100000), .length = (uint16_t)rand_u32(1, 5000)};

		bool ov = overlaps(&a, &b);
		int32_t area = overlap_area(&a, &b);
		bool expected = area > a.length / 2;
		if (ov != expected) {
			fprintf(stderr, "OverlapsConsistentWithArea FAILED (seed=%u, trial=%d): overlaps=%d but area=%d a.length=%u\n",
					seed, t, ov, area, a.length);
			return -1;
		}
	}
	return 0;
}

// 7. calculate_error is symmetric in the DATA_BIT choice it's meant to
//    disambiguate: comparing a length exactly at ACODE_TIMING(acode) gives
//    zero error against that acode, and is always <= the d0/d1 distance by
//    construction (calculate_error takes the min of the two).
TEST(DisambiguatorProps, CalculateErrorZeroAtExactTiming) {
	for (int acode = 0; acode < 8; acode++) {
		LightcapElement le = {.sensor_id = 0, .timestamp = 0, .length = (uint16_t)ACODE_TIMING(acode)};
		uint32_t err = calculate_error(acode, &le);
		if (err != 0) {
			fprintf(stderr, "CalculateErrorZeroAtExactTiming FAILED: acode=%d length=%d err=%u (expected 0)\n",
					acode, le.length, err);
			return -1;
		}
	}
	return 0;
}

// 8. calculate_error never exceeds the distance to the un-databit timing
//    (it takes the min of d0/d1, so it's always <= either individually).
TEST(DisambiguatorProps, CalculateErrorIsMinOfTwoDistances) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	for (int t = 0; t < N_TRIALS; t++) {
		int acode = rand() % 8;
		LightcapElement le = {.sensor_id = 0, .timestamp = 0, .length = (uint16_t)rand_u32(2000, 7000)};

		uint32_t err = calculate_error(acode, &le);
		uint32_t d0 = (uint32_t)abs((int)ACODE_TIMING(acode) - (int)le.length);
		uint32_t d1 = (uint32_t)abs((int)ACODE_TIMING(acode | DATA_BIT) - (int)le.length);
		uint32_t expected_min = d0 < d1 ? d0 : d1;

		if (err != expected_min) {
			fprintf(stderr, "CalculateErrorIsMinOfTwoDistances FAILED (seed=%u, trial=%d): acode=%d length=%d err=%u expected=%u\n",
					seed, t, acode, le.length, err, expected_min);
			return -1;
		}
	}
	return 0;
}
