// Property-based tests for the Simple API event queue
//
// The Simple API uses a fixed 64-event circular buffer. These tests verify
// the queue's behavior under normal use, overflow, and edge conditions.
// The circular buffer logic is replicated here because the real functions
// are static in survive_api.c.
//
// Context: The stagehand agent experienced a 24-minute pose dropout.
// One hypothesis is event queue overflow causing silent event loss.

#include "test_case.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

// ── Circular buffer replica (from survive_api.c) ───────────────────

#define MAX_EVENT_SIZE 64

typedef struct {
	size_t events_cnt;
	size_t event_next_write;
	int events[MAX_EVENT_SIZE];  // simplified: just store an int tag
} event_queue_t;

static void queue_init(event_queue_t *q) {
	memset(q, 0, sizeof(*q));
}

// Returns true if the buffer was full (an event was overwritten)
static bool queue_insert(event_queue_t *q, int value) {
	bool was_full = q->events_cnt == MAX_EVENT_SIZE;

	q->events[q->event_next_write] = value;
	q->event_next_write = (q->event_next_write + 1) % MAX_EVENT_SIZE;
	if (!was_full)
		q->events_cnt++;

	return was_full;
}

static bool queue_pop(event_queue_t *q, int *value) {
	if (q->events_cnt == 0)
		return false;

	size_t read_idx = (MAX_EVENT_SIZE + q->event_next_write - q->events_cnt) % MAX_EVENT_SIZE;
	*value = q->events[read_idx];
	q->events_cnt--;
	return true;
}

// ── Property Tests ──────────────────────────────────────────────────

// 1. Insert N, pop N: all values recovered in FIFO order
TEST(EventQueueProps, InsertPopFIFO) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	for (int trial = 0; trial < 1000; trial++) {
		event_queue_t q;
		queue_init(&q);

		int n = 1 + (rand() % MAX_EVENT_SIZE);  // 1..64

		for (int i = 0; i < n; i++) {
			queue_insert(&q, i * 10 + trial);
		}

		for (int i = 0; i < n; i++) {
			int val;
			if (!queue_pop(&q, &val)) {
				fprintf(stderr, "InsertPopFIFO FAILED: pop returned false at i=%d (seed=%u, trial=%d)\n",
						i, seed, trial);
				return -1;
			}
			int expected = i * 10 + trial;
			if (val != expected) {
				fprintf(stderr, "InsertPopFIFO FAILED: expected %d, got %d (seed=%u, trial=%d)\n",
						expected, val, seed, trial);
				return -1;
			}
		}

		// Queue should be empty now
		int dummy;
		if (queue_pop(&q, &dummy)) {
			fprintf(stderr, "InsertPopFIFO FAILED: queue not empty after popping all (seed=%u, trial=%d)\n",
					seed, trial);
			return -1;
		}
	}
	return 0;
}

// 2. Overflow drops oldest events
//
// When more than MAX_EVENT_SIZE events are inserted without popping,
// the oldest events are silently overwritten. This test verifies that
// after inserting 64+N events, only the last 64 are recoverable.
TEST(EventQueueProps, OverflowDropsOldest) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	for (int trial = 0; trial < 100; trial++) {
		event_queue_t q;
		queue_init(&q);

		int overflow = 1 + (rand() % 100);  // 1..100 extra events
		int total = MAX_EVENT_SIZE + overflow;

		for (int i = 0; i < total; i++) {
			queue_insert(&q, i);
		}

		// Only the last MAX_EVENT_SIZE events should be in the queue
		if (q.events_cnt != MAX_EVENT_SIZE) {
			fprintf(stderr, "OverflowDropsOldest FAILED: events_cnt=%zu, expected %d (seed=%u, trial=%d)\n",
					q.events_cnt, MAX_EVENT_SIZE, seed, trial);
			return -1;
		}

		// Pop and verify: should get events [overflow..total-1]
		for (int i = 0; i < MAX_EVENT_SIZE; i++) {
			int val;
			if (!queue_pop(&q, &val)) {
				fprintf(stderr, "OverflowDropsOldest FAILED: pop returned false at i=%d (seed=%u, trial=%d)\n",
						i, seed, trial);
				return -1;
			}
			int expected = overflow + i;
			if (val != expected) {
				fprintf(stderr, "OverflowDropsOldest FAILED: expected %d, got %d at i=%d (seed=%u, trial=%d)\n",
						expected, val, i, seed, trial);
				return -1;
			}
		}
	}
	return 0;
}

// 3. Interleaved insert/pop maintains FIFO order
//
// Simulates a producer/consumer pattern: insert some events, pop some,
// insert more, pop more. Verifies that values always come out in order.
TEST(EventQueueProps, InterleavedInsertPop) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	for (int trial = 0; trial < 200; trial++) {
		event_queue_t q;
		queue_init(&q);

		int next_write = 0;
		int next_read = 0;

		// Do 500 random operations
		for (int op = 0; op < 500; op++) {
			if (rand() % 3 != 0 && q.events_cnt < MAX_EVENT_SIZE) {
				// Insert (2/3 probability, if not full)
				queue_insert(&q, next_write);
				next_write++;
			} else if (q.events_cnt > 0) {
				// Pop (1/3 probability, if not empty)
				int val;
				if (!queue_pop(&q, &val)) {
					fprintf(stderr, "InterleavedInsertPop FAILED: pop false with cnt=%zu (seed=%u, trial=%d)\n",
							q.events_cnt, seed, trial);
					return -1;
				}
				if (val != next_read) {
					fprintf(stderr, "InterleavedInsertPop FAILED: expected %d, got %d (seed=%u, trial=%d, op=%d)\n",
							next_read, val, seed, trial, op);
					return -1;
				}
				next_read++;
			}
		}

		// Drain remaining
		while (q.events_cnt > 0) {
			int val;
			queue_pop(&q, &val);
			if (val != next_read) {
				fprintf(stderr, "InterleavedInsertPop FAILED drain: expected %d, got %d (seed=%u, trial=%d)\n",
						next_read, val, seed, trial);
				return -1;
			}
			next_read++;
		}
	}
	return 0;
}

// 4. Queue size is always <= MAX_EVENT_SIZE
//
// No matter how many events are inserted, events_cnt never exceeds
// the buffer size.
TEST(EventQueueProps, SizeNeverExceedsMax) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	for (int trial = 0; trial < 100; trial++) {
		event_queue_t q;
		queue_init(&q);

		int n = 100 + (rand() % 1000);
		for (int i = 0; i < n; i++) {
			queue_insert(&q, i);
			if (q.events_cnt > MAX_EVENT_SIZE) {
				fprintf(stderr, "SizeNeverExceedsMax FAILED: cnt=%zu after %d inserts (seed=%u, trial=%d)\n",
						q.events_cnt, i + 1, seed, trial);
				return -1;
			}
		}
	}
	return 0;
}

// 5. Empty queue pop returns false
TEST(EventQueueProps, EmptyPopReturnsFalse) {
	event_queue_t q;
	queue_init(&q);

	int val;
	if (queue_pop(&q, &val)) {
		fprintf(stderr, "EmptyPopReturnsFalse FAILED: pop succeeded on empty queue\n");
		return -1;
	}

	// Insert and drain, then pop again
	queue_insert(&q, 42);
	queue_pop(&q, &val);
	if (queue_pop(&q, &val)) {
		fprintf(stderr, "EmptyPopReturnsFalse FAILED: pop succeeded on drained queue\n");
		return -1;
	}

	return 0;
}

// 6. Overflow detection: queue_insert returns true when buffer was full
TEST(EventQueueProps, OverflowDetected) {
	event_queue_t q;
	queue_init(&q);

	// Fill to capacity — none should report overflow
	for (int i = 0; i < MAX_EVENT_SIZE; i++) {
		bool was_full = queue_insert(&q, i);
		if (was_full) {
			fprintf(stderr, "OverflowDetected FAILED: false overflow at i=%d\n", i);
			return -1;
		}
	}

	// Next insert should report overflow
	for (int i = 0; i < 10; i++) {
		bool was_full = queue_insert(&q, 1000 + i);
		if (!was_full) {
			fprintf(stderr, "OverflowDetected FAILED: overflow not detected at i=%d\n", i);
			return -1;
		}
	}

	return 0;
}

// 7. Wrap-around stress: fill, drain, fill, drain many times
//
// The write pointer wraps around the buffer. After many cycles,
// verify the circular indexing still works correctly.
TEST(EventQueueProps, WrapAroundStress) {
	unsigned seed = (unsigned)time(NULL);
	srand(seed);

	event_queue_t q;
	queue_init(&q);

	int global_counter = 0;

	for (int cycle = 0; cycle < 1000; cycle++) {
		// Insert a random batch (up to capacity)
		int batch = 1 + (rand() % MAX_EVENT_SIZE);
		for (int i = 0; i < batch; i++) {
			queue_insert(&q, global_counter++);
		}

		// Drain all
		int expected = global_counter - (int)q.events_cnt;
		while (q.events_cnt > 0) {
			int val;
			queue_pop(&q, &val);
			if (val != expected) {
				fprintf(stderr, "WrapAroundStress FAILED: expected %d, got %d (cycle=%d, seed=%u)\n",
						expected, val, cycle, seed);
				return -1;
			}
			expected++;
		}
	}
	return 0;
}
