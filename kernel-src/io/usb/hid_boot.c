#include <kernel/alloc.h>
#include <kernel/input.h>
#include <kernel/usb.h>
#include <logging.h>

#define USB_CLASS_HID 0x03
#define USB_SUBCLASS_BOOT 0x01
#define USB_PROTOCOL_KEYBOARD 0x01
#define USB_PROTOCOL_MOUSE 0x02

// HID class-specific requests
#define HID_REQUEST_SET_IDLE 0x0a
#define HID_REQUEST_SET_PROTOCOL 0x0b

typedef struct {
	usb_device_t *dev;
	usb_interface_desc_t interface;
	usb_xfer_t intr_in_xfer;

	union {
		struct {
			input_device_t *input;

			bool left : 1;
			bool right : 1;
			bool middle : 1;
			bool side : 1;
			bool extra : 1;
		} mouse;

		struct {
			input_device_t *input;
			bitmap_t key_bits;
		} keyboard;
	} device;

	// Report buffer.
	uint8_t report_buffer[64];
} usb_hid_boot_data_t;

static inline bool find_key_in_report(uint8_t *keycodes, uint8_t keycode) {
	for (uint8_t i = 0; i < 6; i++) {
		if (keycodes[i] == keycode)
			return true;
	}
	return false;
}

static const uint8_t keycode_map[256] = {
	[0x04] = INPUT_KEY_A,
	[0x05] = INPUT_KEY_B,
	[0x06] = INPUT_KEY_C,
	[0x07] = INPUT_KEY_D,
	[0x08] = INPUT_KEY_E,
	[0x09] = INPUT_KEY_F,
	[0x0a] = INPUT_KEY_G,
	[0x0b] = INPUT_KEY_H,
	[0x0c] = INPUT_KEY_I,
	[0x0d] = INPUT_KEY_J,
	[0x0e] = INPUT_KEY_K,
	[0x0f] = INPUT_KEY_L,
	[0x10] = INPUT_KEY_M,
	[0x11] = INPUT_KEY_N,
	[0x12] = INPUT_KEY_O,
	[0x13] = INPUT_KEY_P,
	[0x14] = INPUT_KEY_Q,
	[0x15] = INPUT_KEY_R,
	[0x16] = INPUT_KEY_S,
	[0x17] = INPUT_KEY_T,
	[0x18] = INPUT_KEY_U,
	[0x19] = INPUT_KEY_V,
	[0x1a] = INPUT_KEY_W,
	[0x1b] = INPUT_KEY_X,
	[0x1c] = INPUT_KEY_Y,
	[0x1d] = INPUT_KEY_Z,
	[0x1e] = INPUT_KEY_1,
	[0x1f] = INPUT_KEY_2,
	[0x20] = INPUT_KEY_3,
	[0x21] = INPUT_KEY_4,
	[0x22] = INPUT_KEY_5,
	[0x23] = INPUT_KEY_6,
	[0x24] = INPUT_KEY_7,
	[0x25] = INPUT_KEY_8,
	[0x26] = INPUT_KEY_9,
	[0x27] = INPUT_KEY_0,
	[0x28] = INPUT_KEY_ENTER,
	[0x29] = INPUT_KEY_ESC,
	[0x2a] = INPUT_KEY_BACKSPACE,
	[0x2b] = INPUT_KEY_TAB,
	[0x2c] = INPUT_KEY_SPACE,
	[0x2d] = INPUT_KEY_MINUS,
	[0x2e] = INPUT_KEY_EQUAL,
	[0x2f] = INPUT_KEY_LEFTBRACE,
	[0x30] = INPUT_KEY_RIGHTBRACE,
	[0x31] = INPUT_KEY_BACKSLASH,
	[0x33] = INPUT_KEY_SEMICOLON,
	[0x34] = INPUT_KEY_APOSTROPHE,
	[0x35] = INPUT_KEY_GRAVE,
	[0x36] = INPUT_KEY_COMMA,
	[0x37] = INPUT_KEY_DOT,
	[0x38] = INPUT_KEY_SLASH,
	[0x39] = INPUT_KEY_CAPSLOCK,
	[0x3a] = INPUT_KEY_F1,
	[0x3b] = INPUT_KEY_F2,
	[0x3c] = INPUT_KEY_F3,
	[0x3d] = INPUT_KEY_F4,
	[0x3e] = INPUT_KEY_F5,
	[0x3f] = INPUT_KEY_F6,
	[0x40] = INPUT_KEY_F7,
	[0x41] = INPUT_KEY_F8,
	[0x42] = INPUT_KEY_F9,
	[0x43] = INPUT_KEY_F10,
	[0x44] = INPUT_KEY_F11,
	[0x45] = INPUT_KEY_F12,
	[0x49] = INPUT_KEY_INSERT,
	[0x4a] = INPUT_KEY_HOME,
	[0x4b] = INPUT_KEY_PAGEUP,
	[0x4c] = INPUT_KEY_DELETE,
	[0x4d] = INPUT_KEY_END,
	[0x4e] = INPUT_KEY_PAGEDOWN,
	[0x4f] = INPUT_KEY_RIGHT,
	[0x50] = INPUT_KEY_LEFT,
	[0x51] = INPUT_KEY_DOWN,
	[0x52] = INPUT_KEY_UP,

	[0xe0] = INPUT_KEY_LEFTCTRL,
	[0xe1] = INPUT_KEY_LEFTSHIFT,
	[0xe2] = INPUT_KEY_LEFTALT,
	[0xe4] = INPUT_KEY_RIGHTCTRL,
	[0xe5] = INPUT_KEY_RIGHTSHIFT,
	[0xe6] = INPUT_KEY_RIGHTALT,
};

static void process_kb_report(usb_hid_boot_data_t *data, uint8_t *report, uint32_t length) {
	__assert(length == 8);

	uint8_t mods = report[0];
	uint8_t keycodes[6];
	memcpy(keycodes, &report[2], 6);

	if (keycodes[0] == 0x1)
		return; // Ignore rollover error.

	input_event_t events[16];
	int event_count = 0;

	// Process modifier keys.
	for (int i = 0; i < 8; i++) {
		bool was_pressed = bitmap_get(&data->device.keyboard.key_bits, 0xe0 + i);
		bool is_pressed = (mods & (1 << i)) != 0;

		if (was_pressed != is_pressed) {
			uint8_t key = keycode_map[0xe0 + i];
			if (key == 0)
				continue;

			input_event_t *event = &events[event_count++];
			event->type = INPUT_EV_KEY;
			event->code = key;
			event->value = is_pressed ? 1 : 0;

			bitmap_set(&data->device.keyboard.key_bits, 0xe0 + i, is_pressed);
		}
	}

	// Process regular keys.
	for (int i = 0; i < 0xe0; i++) {
		bool was_pressed = bitmap_get(&data->device.keyboard.key_bits, i);
		bool is_pressed = find_key_in_report(keycodes, i);

		if (was_pressed != is_pressed) {
			uint8_t key = keycode_map[i];
			if (key == 0)
				continue;

			input_event_t *event = &events[event_count++];
			event->type = INPUT_EV_KEY;
			event->code = key;
			event->value = is_pressed ? 1 : 0;

			bitmap_set(&data->device.keyboard.key_bits, i, is_pressed);
		}
	}

	input_queue_packet(data->device.keyboard.input, events, event_count);
}

static void process_mouse_report(usb_hid_boot_data_t *data, uint8_t *report, uint32_t length) {
	__assert(length >= 3);

	uint8_t buttons = report[0];
	int8_t x_move = report[1];
	int8_t y_move = report[2];
	int8_t z_move = length >= 4 ? report[3] : 0;

	bool left = (buttons & (1 << 0)) != 0;
	bool right = (buttons & (1 << 1)) != 0;
	bool middle = (buttons & (1 << 2)) != 0;
	bool side = (buttons & (1 << 3)) != 0;
	bool extra = (buttons & (1 << 4)) != 0;

	input_event_t events[8];
	int event_count = 0;

	if (x_move != 0) {
		input_event_t *event = &events[event_count++];
		event->type = INPUT_EV_REL;
		event->code = INPUT_REL_X;
		event->value = x_move;
	}

	if (y_move != 0) {
		input_event_t *event = &events[event_count++];
		event->type = INPUT_EV_REL;
		event->code = INPUT_REL_Y;
		event->value = y_move;
	}

	if (z_move != 0) {
		input_event_t *event = &events[event_count++];
		event->type = INPUT_EV_REL;
		event->code = INPUT_REL_WHEEL;
		event->value = z_move;
	}

	if (left != data->device.mouse.left) {
		input_event_t *event = &events[event_count++];
		event->type = INPUT_EV_KEY;
		event->code = INPUT_KEY_BTN_LEFT;
		event->value = left ? 1 : 0;
	}

	if (right != data->device.mouse.right) {
		input_event_t *event = &events[event_count++];
		event->type = INPUT_EV_KEY;
		event->code = INPUT_KEY_BTN_RIGHT;
		event->value = right ? 1 : 0;
	}

	if (middle != data->device.mouse.middle) {
		input_event_t *event = &events[event_count++];
		event->type = INPUT_EV_KEY;
		event->code = INPUT_KEY_BTN_MIDDLE;
		event->value = middle ? 1 : 0;
	}

	if (side != data->device.mouse.side) {
		input_event_t *event = &events[event_count++];
		event->type = INPUT_EV_KEY;
		event->code = INPUT_KEY_BTN_SIDE;
		event->value = side ? 1 : 0;
	}

	if (extra != data->device.mouse.extra) {
		input_event_t *event = &events[event_count++];
		event->type = INPUT_EV_KEY;
		event->code = INPUT_KEY_BTN_EXTRA;
		event->value = extra ? 1 : 0;
	}

	data->device.mouse.left = left;
	data->device.mouse.right = right;
	data->device.mouse.middle = middle;
	data->device.mouse.side = side;
	data->device.mouse.extra = extra;

	input_queue_packet(data->device.mouse.input, events, event_count);
}

static void process_report(usb_xfer_t *xfer, usb_status_t status, uint32_t transferred) {
	usb_hid_boot_data_t *data = xfer->completion_ctx;

	// Process the HID report.
	if (data->interface.bInterfaceProtocol == USB_PROTOCOL_KEYBOARD)
		process_kb_report(data, data->report_buffer, transferred);
	else if (data->interface.bInterfaceProtocol == USB_PROTOCOL_MOUSE)
		process_mouse_report(data, data->report_buffer, transferred);

	// Re-submit the interrupt IN transfer.
	int res = usb_submit_xfer(data->dev, &data->intr_in_xfer);
	__assert(res == 0);
}

int usb_hid_attach_boot(usb_device_t *dev, usb_probe_ctx_t *ctx, void **driver_data) {
	usb_interface_desc_t *hid_interface = NULL;
	usb_for_each_descriptor(ctx->config_desc, desc) {
		if (desc->bDescriptorType != USB_DESCRIPTOR_TYPE_INTERFACE)
			continue;

		usb_interface_desc_t *iface_desc = (usb_interface_desc_t *)desc;
		if (iface_desc->bInterfaceClass != USB_CLASS_HID)
			continue;
		if (iface_desc->bInterfaceSubClass != USB_SUBCLASS_BOOT)
			continue;

		hid_interface = iface_desc;
		break;
	}
	if (hid_interface == NULL)
		return -1;

	// Find the interrupt IN endpoint descriptor.
	usb_endpoint_desc_t *intr_in_ep = NULL;
	usb_for_each_descriptor(ctx->config_desc, desc) {
		if (desc->bDescriptorType != USB_DESCRIPTOR_TYPE_ENDPOINT)
			continue;

		usb_endpoint_desc_t *ep_desc = (usb_endpoint_desc_t *)desc;
		if (!(ep_desc->bEndpointAddress & USB_ENDPOINT_ADDRESS_DIR_IN))
			continue;
		if ((ep_desc->bmAttributes & USB_ENDPOINT_ATTRIB_TYPE_MASK) != USB_ENDPOINT_ATTRIB_TYPE_INTR)
			continue;

		intr_in_ep = ep_desc;
		break;
	}
	__assert(intr_in_ep != NULL);

	// Enable the boot protocol.
	{
		usb_setup_t setup = {0};
		setup.bmRequestType = USB_REQUEST_RECIP_INTERFACE | USB_REQUEST_CLASS | USB_REQUEST_DIR_TO_DEVICE;
		setup.bRequest = HID_REQUEST_SET_PROTOCOL;
		setup.wValue = 0; // Boot Protocol
		setup.wIndex = hid_interface->bInterfaceNumber;
		setup.wLength = 0;

		int res = usb_control_xfer(dev, &setup, NULL);
		__assert(res == 0);
	}

	// Send a SET_IDLE request to the device.
	{
		usb_setup_t setup = {0};
		setup.bmRequestType = USB_REQUEST_RECIP_INTERFACE | USB_REQUEST_CLASS | USB_REQUEST_DIR_TO_DEVICE;
		setup.bRequest = HID_REQUEST_SET_IDLE;
		setup.wValue = 0;
		setup.wIndex = hid_interface->bInterfaceNumber;
		setup.wLength = 0;

		int res = usb_control_xfer(dev, &setup, NULL);
		__assert(res == 0);
	}

	// Configure the interrupt IN endpoint.
	usb_endpoint_t *ep = alloc(sizeof(usb_endpoint_t));
	__assert(ep != NULL);
	memcpy(&ep->desc, intr_in_ep, sizeof(usb_endpoint_desc_t));

	int res = usb_configure_endpoint(dev, ep);
	__assert(res == 0);

	usb_hid_boot_data_t *data = alloc(sizeof(usb_hid_boot_data_t));
	__assert(data != NULL);

	input_device_t *input_dev = input_new();
	__assert(input_dev != NULL);

	res = usb_get_string_descriptor(
		ctx->device, ctx->device_desc->iProduct, input_dev->name, sizeof(input_dev->name));
	__assert(res == 0);

	if (hid_interface->bInterfaceProtocol == USB_PROTOCOL_KEYBOARD) {
		bitmap_set(&input_dev->ev_bits, INPUT_EV_KEY, 1);
		for (int i = 0; i < 256; i++) {
			if (keycode_map[i] != 0)
				bitmap_set(&input_dev->key_bits, keycode_map[i], 1);
		}

		data->device.keyboard.input = input_dev;

		res = bitmap_init(&data->device.keyboard.key_bits, 256);
		__assert(res == 0);
	} else if (hid_interface->bInterfaceProtocol == USB_PROTOCOL_MOUSE) {
		bitmap_set(&input_dev->ev_bits, INPUT_EV_REL, 1);
		bitmap_set(&input_dev->rel_bits, INPUT_REL_X, 1);
		bitmap_set(&input_dev->rel_bits, INPUT_REL_Y, 1);
		bitmap_set(&input_dev->rel_bits, INPUT_REL_WHEEL, 1);
		bitmap_set(&input_dev->ev_bits, INPUT_EV_KEY, 1);
		bitmap_set(&input_dev->key_bits, INPUT_KEY_BTN_LEFT, 1);
		bitmap_set(&input_dev->key_bits, INPUT_KEY_BTN_RIGHT, 1);
		bitmap_set(&input_dev->key_bits, INPUT_KEY_BTN_MIDDLE, 1);
		bitmap_set(&input_dev->key_bits, INPUT_KEY_BTN_SIDE, 1);
		bitmap_set(&input_dev->key_bits, INPUT_KEY_BTN_EXTRA, 1);

		data->device.mouse.input = input_dev;
		data->device.mouse.left = false;
		data->device.mouse.right = false;
		data->device.mouse.middle = false;
		data->device.mouse.side = false;
		data->device.mouse.extra = false;
	}

	data->dev = dev;
	memcpy(&data->interface, hid_interface, sizeof(usb_interface_desc_t));

	data->intr_in_xfer.ep = ep;
	data->intr_in_xfer.flags = USB_XFER_FLAG_TO_HOST;
	data->intr_in_xfer.type = USB_XFER_TYPE_INTERRUPT;
	data->intr_in_xfer.data = data->report_buffer;
	data->intr_in_xfer.data_length = intr_in_ep->wMaxPacketSize;
	data->intr_in_xfer.completion = process_report;
	data->intr_in_xfer.completion_ctx = data;

	res = usb_submit_xfer(data->dev, &data->intr_in_xfer);
	__assert(res == 0);

	*driver_data = data;
	return 0;
}
