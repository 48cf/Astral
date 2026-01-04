#include <kernel/devfs.h>
#include <errno.h>
#include <string.h>
#include <logging.h>
#include <kernel/timekeeper.h>
#include <kernel/init.h>

#define MT_W 64
#define MT_N 312
#define MT_M 156
#define MT_R 31
#define MT_A 0xb5026f5aa96619e9
#define MT_U 29
#define MT_D 0x5555555555555555
#define MT_S 17
#define MT_B 0x71d67fffeda60000
#define MT_T 37
#define MT_C 0xfff7eee000000000
#define MT_L 43
#define MT_F 0x5851f42d4c957f2d

#define MT_UPPER_MASK ((uint64_t)(1 << MT_R) - 1)
#define MT_LOWER_MASK (~(MT_UPPER_MASK))

static spinlock_t mt_lock = SPINLOCK_INIT_VALUE;
static uint64_t mt_state[MT_N];
static int mt_index = MT_N;

static __attribute__((no_sanitize("undefined"))) void twist(void) {
	for (int i = 0; i < MT_N - 1; i++) {
		uint64_t x = (mt_state[i] & MT_UPPER_MASK) | (mt_state[(i + 1) % MT_N] & MT_LOWER_MASK);
		uint64_t x_a = x >> 1;
		if (x & 1)
			x_a ^= MT_A;
		mt_state[i] = mt_state[(i + MT_M) % MT_N] ^ x_a;
	}
	mt_index = 0;
}

static __attribute__((no_sanitize("undefined"))) uint64_t random_gen(void) {
	spinlock_acquire(&mt_lock);
	if (mt_index >= MT_N)
		twist();

	uint64_t y = mt_state[mt_index++];
	y ^= (y >> MT_U) & MT_D;
	y ^= (y << MT_S) & MT_B;
	y ^= (y << MT_T) & MT_C;
	y ^= (y >> MT_L);
	spinlock_release(&mt_lock);
	return y;
}

static int null_write(int minor, iovec_iterator_t *iovec_iterator, size_t count, uintmax_t offset, int flags, size_t *wcount) {
	*wcount = count;
	return 0;
}

static int full_write(int minor, iovec_iterator_t *iovec_iterator, size_t count, uintmax_t offset, int flags, size_t *wcount) {
	return ENOSPC;
}

static int zero_read(int minor, iovec_iterator_t *iovec_iterator, size_t count, uintmax_t offset, int flags, size_t *rcount) {
	*rcount = count;
	return iovec_iterator_memset(iovec_iterator, 0, count);
}

static int null_read(int minor, iovec_iterator_t *iovec_iterator, size_t count, uintmax_t offset, int flags, size_t *rcount) {
	*rcount = 0;
	return 0;
}

static int maxseek(int minor, size_t *max) {
	*max = 0;
	return 0;
}

static int urandom_read(int minor, iovec_iterator_t *iovec_iterator, size_t count, uintmax_t offset, int flags, size_t *rcount) {
	for (int i = 0; i < count; ++i) {
		uint64_t rand = random_gen();

		int err = iovec_iterator_copy_from_buffer(iovec_iterator, &rand, sizeof(uint64_t));
		if (err != 0)
			return err;
	}

	*rcount = count;
	return 0;
}

static devops_t nullops = {
	.read = null_read,
	.write = null_write,
	.maxseek = maxseek
};

static devops_t fullops = {
	.read = zero_read,
	.write = full_write,
	.maxseek = maxseek
};

static devops_t zeroops = {
	.read = zero_read,
	.write = null_write,
	.maxseek = maxseek
};

static devops_t urandomops = {
	.read = urandom_read,
	.write = null_write,
	.maxseek = maxseek
};

void pseudodevices_init() {
	uint64_t seed = timespec_ns(timekeeper_timefromboot());

	mt_state[0] = seed;
	mt_index = MT_N;
	for (int i = 1; i < MT_N - 1; i++) {
		mt_state[i] = MT_F * (mt_state[i - 1] ^ (mt_state[i - 1] >> (MT_W - 2))) + i;
	}

	__assert(devfs_register(&nullops, "null", V_TYPE_CHDEV, DEV_MAJOR_NULL, 0, 0666, NULL) == 0);
	__assert(devfs_register(&fullops, "full", V_TYPE_CHDEV, DEV_MAJOR_FULL, 0, 0666, NULL) == 0);
	__assert(devfs_register(&zeroops, "zero", V_TYPE_CHDEV, DEV_MAJOR_ZERO, 0, 0666, NULL) == 0);
	__assert(devfs_register(&urandomops, "urandom", V_TYPE_CHDEV, DEV_MAJOR_URANDOM, 0, 0666, NULL) == 0);
}

INIT_ROUTINE_DEFINE(pseudo_devices, INIT_ROUTINE_FLAGS_NONE, pseudodevices_init, devfs);
