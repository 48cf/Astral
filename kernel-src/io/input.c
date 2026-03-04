#include <kernel/alloc.h>
#include <kernel/console.h>
#include <kernel/devfs.h>
#include <kernel/init.h>
#include <kernel/input.h>
#include <kernel/usercopy.h>
#include <logging.h>

#define MAX_DEVICES 256

#define INPUT_DEV_BUFFER_SIZE (128 * sizeof(input_event_t))

static int input_dev_count = 0;
static int input_listener_count = 0;

static mutex_t input_dev_table_lock;
static hashtable_t input_dev_table;
static hashtable_t input_listener_table;

static devops_t input_dev_ops;
static devops_t input_listener_ops;

static void *get(int minor) {
	hashtable_t *table;
	if (minor >= MAX_DEVICES) {
		if (minor - MAX_DEVICES >= input_listener_count)
			return NULL;
		table = &input_listener_table;
		minor -= MAX_DEVICES;
	} else {
		if (minor >= input_dev_count)
			return NULL;
		table = &input_dev_table;
	}

	void *result;

	MUTEX_ACQUIRE(&input_dev_table_lock);
	int err = hashtable_get(table, &result, &minor, sizeof(minor));
	MUTEX_RELEASE(&input_dev_table_lock);

	return err ? NULL : result;
}

static int input_dev_open(int minor, vnode_t **vnode, int flags) {
	input_device_t *dev = get(minor);
	if (dev == NULL)
		return ENODEV;

	int res;
	int listener_minor = __atomic_fetch_add(&input_listener_count, 1, __ATOMIC_RELAXED);

	char tmp_name[20];
	snprintf(tmp_name, 20, "input/.listener%d", listener_minor);

	res = devfs_register(&input_listener_ops, tmp_name, V_TYPE_CHDEV, DEV_MAJOR_INPUT, listener_minor + MAX_DEVICES, 0, NULL);
	if (res != 0)
		return res;

	vnode_t *listener_vnode;
	__assert(devfs_getbyname(tmp_name, &listener_vnode) == 0);
	devfs_remove(tmp_name, DEV_MAJOR_INPUT, listener_minor + MAX_DEVICES);

	input_listener_t *listener = alloc(sizeof(input_listener_t));
	if (listener == NULL) {
		VOP_RELEASE(listener_vnode);
		return ENOMEM;
	}

	res = ringbuffer_init(&listener->buffer, INPUT_DEV_BUFFER_SIZE);
	if (res != 0) {
		VOP_RELEASE(listener_vnode);
		free(listener);
		return res;
	}

	MUTEX_ACQUIRE(&input_dev_table_lock);
	res = hashtable_set(&input_listener_table, listener, &listener_minor, sizeof(listener_minor), true);
	MUTEX_RELEASE(&input_dev_table_lock);

	if (res != 0) {
		ringbuffer_destroy(&listener->buffer);
		VOP_RELEASE(listener_vnode);
		free(listener);
		return res;
	}

	listener->device = dev;
	listener->vnode = listener_vnode;

	SPINLOCK_INIT(listener->lock);
	POLL_INITHEADER(&listener->poll_header);
	list_push_back(&dev->listeners, &listener->node);

	VOP_RELEASE(*vnode);
	*vnode = listener_vnode;

	return 0;
}

static devops_t input_dev_ops = {
	.open = input_dev_open
};

static int input_listener_close(int minor, int flags) {
	input_listener_t *lis = get(minor);
	if (lis == NULL)
		return ENODEV;

	MUTEX_ACQUIRE(&input_dev_table_lock);
	hashtable_remove(&input_listener_table, &minor, sizeof(minor));
	MUTEX_RELEASE(&input_dev_table_lock);

	list_remove(&lis->device->listeners, &lis->node);
	ringbuffer_destroy(&lis->buffer);
	free(lis);

	return 0;
}

static int input_listener_internal_poll(input_listener_t *lis, polldata_t *data, int events) {
	int revents = 0;

	if ((events & POLLIN) && RINGBUFFER_DATACOUNT(&lis->buffer) > 0)
		revents |= POLLIN;

	if (revents == 0 && data)
		poll_add(&lis->poll_header, data, events);

	return revents;
}

static int input_listener_read(int minor, iovec_iterator_t *iovec_iterator, size_t size, uintmax_t offset, int flags, size_t *readc) {
	input_listener_t *lis = get(minor);
	if (lis == NULL)
		return ENODEV;

	if (size % sizeof(input_event_t) != 0)
		return EINVAL;

	int err = 0;
	for (;;) {
		polldesc_t desc = {0};
		err = poll_initdesc(&desc, 1);
		if (err)
			return err;

		int revents = input_listener_internal_poll(lis, &desc.data[0], POLLIN);
		if (revents != 0) {
			*readc = iovec_iterator_read_from_ringbuffer(iovec_iterator, &lis->buffer, size);
			if (*readc == RINGBUFFER_USER_COPY_FAILED)
				err = EFAULT;

			poll_leave(&desc);
			poll_destroydesc(&desc);
			break;
		}

		if (flags & V_FFLAGS_NONBLOCKING) {
			poll_leave(&desc);
			poll_destroydesc(&desc);
			return EAGAIN;
		}

		err = poll_dowait(&desc, 0);

		poll_leave(&desc);
		poll_destroydesc(&desc);

		if (err)
			return err;
	}

	return err;
}

static int input_listener_poll(int minor, polldata_t *data, int events) {
	input_listener_t *lis = get(minor);
	if (lis == NULL)
		return POLLERR;

	bool int_state = interrupt_set(false);
	spinlock_acquire(&lis->lock);

	int revents = input_listener_internal_poll(lis, data, events);

	spinlock_release(&lis->lock);
	interrupt_set(int_state);

	return revents;
}

static int copy_bitmap_to_user(bitmap_t *bitmap, void *arg, size_t size) {
	int err = 0;

	long nlong = 0;
	if (bitmap != NULL) {
		nlong = ROUND_UP(bitmap->size, sizeof(long) * 8) / (sizeof(long) * 8);
		err = USERCOPY_POSSIBLY_TO_USER(arg, bitmap->data, min(size, nlong * sizeof(long)));
		if (err != 0)
			return err;
	}

	if (nlong * sizeof(long) < size)
		err = USERCOPY_POSSIBLY_MEMSET_TO_USER((void *)arg + nlong * sizeof(long), 0, size - nlong * sizeof(long));

	return err;
}

static int input_listener_ioctl(int minor, unsigned long request, void *arg, int *result, cred_t *cred) {
	input_listener_t *lis = get(minor);
	if (lis == NULL)
		return ENODEV;

	int number = request & 0xff;
	int type = (request >> 8) & 0xff;
	int size = (request >> 16) & 0x3fff;

	if (type != 'E')
		return ENOTTY;

#define EVIOCGVERSION 0x1
#define EVIOCGID 0x2
#define EVIOCGNAME 0x6
#define EVIOCGPHYS 0x7
#define EVIOCGUNIQ 0x8
#define EVIOCGPROP 0x9
#define EVIOCGKEY 0x18
#define EVIOCGLED 0x19
#define EVIOCGSND 0x1a
#define EVIOCGSW 0x1b
#define EVIOCGBIT 0x20
#define EVIOCGABS 0x40
#define EVIOVSABS 0xc0

#define ID_BUS 0
#define ID_VENDOR 1
#define ID_PRODUCT 2
#define ID_VERSION 3

	if (number == EVIOCGVERSION) {
		uint32_t version = 0;
		version |= lis->device->ver_major << 16;
		version |= lis->device->ver_minor << 8;
		version |= lis->device->ver_patch << 0;

		return USERCOPY_POSSIBLY_TO_USER(arg, &version, sizeof(version));
	} else if (number == EVIOCGID) {
		uint16_t id[4];
		id[ID_BUS] = lis->device->id_bus;
		id[ID_VENDOR] = lis->device->id_vendor;
		id[ID_PRODUCT] = lis->device->id_product;
		id[ID_VERSION] = lis->device->id_version;

		return USERCOPY_POSSIBLY_TO_USER(arg, &id, sizeof(id));
	} else if (number == EVIOCGNAME) {
		return USERCOPY_POSSIBLY_TO_USER(arg, &lis->device->name, min(size, sizeof(lis->device->name)));
	} else if (number == EVIOCGPHYS) {
		return USERCOPY_POSSIBLY_TO_USER(arg, &lis->device->phys, min(size, sizeof(lis->device->phys)));
	} else if (number == EVIOCGUNIQ) {
		return USERCOPY_POSSIBLY_TO_USER(arg, &lis->device->uniq, min(size, sizeof(lis->device->uniq)));
	} else if (number == EVIOCGPROP) {
		return copy_bitmap_to_user(&lis->device->prop_bits, arg, size);
	} else if (number == EVIOCGKEY || number == EVIOCGLED || number == EVIOCGSND || number == EVIOCGSW) {
		// TODO: idk maybe implement properly???? lol
		return copy_bitmap_to_user(NULL, arg, size);
	} else if (number == EVIOCGBIT) {
		return copy_bitmap_to_user(&lis->device->ev_bits, arg, size);
	} else if (number > EVIOCGBIT && number < EVIOCGBIT + INPUT_EV_CNT) {
		bitmap_t *bitmap = NULL;
		if (bitmap_get(&lis->device->ev_bits, number - EVIOCGBIT)) {
			switch (number - EVIOCGBIT) {
				case INPUT_EV_KEY:
					bitmap = &lis->device->key_bits;
					break;
				case INPUT_EV_REL:
					bitmap = &lis->device->rel_bits;
					break;
				case INPUT_EV_ABS:
					bitmap = &lis->device->abs_bits;
					break;
			}
		}

		return copy_bitmap_to_user(bitmap, arg, size);
	} else if (number >= EVIOCGABS && number < EVIOCGABS + INPUT_ABS_CNT) {
		if (!bitmap_get(&lis->device->ev_bits, INPUT_EV_ABS))
			return EINVAL;

		int abs_code = number - EVIOCGABS;
		if (!bitmap_get(&lis->device->abs_bits, abs_code))
			return EINVAL;

		input_absinfo_t absinfo;
		absinfo.value = -1;
		absinfo.min = -1;
		absinfo.max = -1;
		absinfo.fuzz = 0;
		absinfo.flat = 0;
		absinfo.res = 0;
		return USERCOPY_POSSIBLY_TO_USER(arg, &absinfo, sizeof(input_absinfo_t));
	} else {
		printf("input_ioctl: unhandled ioctl %lu (%u, %u, %u)\n", request, number, type, size);
		return ENOTTY;
	}
}

static devops_t input_listener_ops = {
	.close = input_listener_close,
	.read = input_listener_read,
	.poll = input_listener_poll,
	.ioctl = input_listener_ioctl,
};

input_device_t *input_new() {
	int minor = __atomic_fetch_add(&input_dev_count, 1, __ATOMIC_RELAXED);
	if (minor >= MAX_DEVICES)
		return NULL;

	char name_buffer[32];
	snprintf(name_buffer, sizeof(name_buffer), "input/event%d", minor);

	input_device_t *dev = alloc(sizeof(input_device_t));
	if (dev == NULL)
		return NULL;
	memset(dev, 0, sizeof(input_device_t));

	list_init(&dev->listeners);

	int res;
	res = bitmap_init(&dev->prop_bits, INPUT_PROP_CNT);
	if (res != 0)
		goto cleanup;

	res = bitmap_init(&dev->ev_bits, INPUT_EV_CNT);
	if (res != 0)
		goto cleanup;

	res = bitmap_init(&dev->syn_bits, INPUT_SYN_CNT);
	if (res != 0)
		goto cleanup;

	res = bitmap_init(&dev->key_bits, INPUT_KEY_CNT);
	if (res != 0)
		goto cleanup;

	res = bitmap_init(&dev->rel_bits, INPUT_REL_CNT);
	if (res != 0)
		goto cleanup;

	res = bitmap_init(&dev->abs_bits, INPUT_ABS_CNT);
	if (res != 0)
		goto cleanup;

	MUTEX_ACQUIRE(&input_dev_table_lock);
	res = hashtable_set(&input_dev_table, dev, &minor, sizeof(minor), true);
	MUTEX_RELEASE(&input_dev_table_lock);
	if (res != 0)
		goto cleanup;

	res = devfs_register(&input_dev_ops, name_buffer, V_TYPE_CHDEV, DEV_MAJOR_INPUT, minor, 0666, NULL);
	__assert(res == 0);

	bitmap_set(&dev->ev_bits, INPUT_EV_SYN, 1);
	bitmap_set(&dev->syn_bits, INPUT_SYN_REPORT, 1);
	bitmap_set(&dev->syn_bits, INPUT_SYN_DROPPED, 1);

	return dev;

cleanup:
	if (dev->ev_bits.data != NULL)
		bitmap_destroy(&dev->ev_bits);
	if (dev->syn_bits.data != NULL)
		bitmap_destroy(&dev->syn_bits);
	if (dev->key_bits.data != NULL)
		bitmap_destroy(&dev->key_bits);
	if (dev->rel_bits.data != NULL)
		bitmap_destroy(&dev->rel_bits);
	if (dev->abs_bits.data != NULL)
		bitmap_destroy(&dev->abs_bits);

	free(dev);
	return NULL;
}

static void input_queue_internal(input_listener_t *lis, input_event_t *event, bool signal) {
	input_event_t ev;
	memcpy(&ev, event, sizeof(input_event_t));

	if (lis->device->clock_id == CLOCK_REALTIME || lis->device->clock_id == CLOCK_MONOTONIC)
		ev.time = timekeeper_time();
	else if (lis->device->clock_id == CLOCK_BOOTTIME)
		ev.time = timekeeper_timefromboot();

	bool int_state = interrupt_set(false);
	spinlock_acquire(&lis->lock);

	size_t written = ringbuffer_write(&lis->buffer, &ev, sizeof(input_event_t));
	if (written == sizeof(input_event_t) && signal)
		poll_event(&lis->poll_header, POLLIN);

	spinlock_release(&lis->lock);
	interrupt_set(int_state);
}

static void input_queue_packet_internal(input_listener_t *lis, input_event_t *events, int count) {
	size_t needed_space = (count + 1) * sizeof(input_event_t);
	if (RINGBUFFER_FREESPACE(&lis->buffer) < needed_space) {
		// Not enough space, write a SYN_DROPPED event.
		input_event_t event;
		event.type = INPUT_EV_SYN;
		event.code = INPUT_SYN_DROPPED;
		event.value = 0;
		input_queue_internal(lis, &event, true);
	} else {
		// Queue all the events.
		for (int i = 0; i < count; ++i) {
			input_queue_internal(lis, &events[i], false);
		}

		input_event_t event;
		event.type = INPUT_EV_SYN;
		event.code = INPUT_SYN_REPORT;
		event.value = 0;
		input_queue_internal(lis, &event, true);
	}
}

void input_queue_packet(input_device_t *dev, input_event_t *events, int count) {
	if (count < 1)
		return;

	console_process_events(events, count);

	long ipl = interrupt_raiseipl(IPL_INPUT);
	spinlock_acquire(&dev->lock);

	list_for_each(&dev->listeners, node) {
		input_listener_t *lis = container_of(node, input_listener_t, node);
		input_queue_packet_internal(lis, events, count);
	}

	spinlock_release(&dev->lock);
	interrupt_loweripl(ipl);
}

static void input_init(void) {
	__assert(devfs_createdir("input") == 0);

	MUTEX_INIT(&input_dev_table_lock);
	__assert(hashtable_init(&input_dev_table, 32) == 0);
	__assert(hashtable_init(&input_listener_table, 128) == 0);
}

INIT_ROUTINE_DEFINE(input, INIT_ROUTINE_FLAGS_NONE, input_init, devfs);
