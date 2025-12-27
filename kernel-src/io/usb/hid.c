#include <kernel/alloc.h>
#include <kernel/keyboard.h>
#include <kernel/mouse.h>
#include <kernel/usb.h>
#include <kernel/init.h>
#include <logging.h>

#define USB_CLASS_HID 0x03
#define USB_SUBCLASS_BOOT 0x01
#define USB_PROTOCOL_KEYBOARD 0x01
#define USB_PROTOCOL_MOUSE 0x02

typedef struct {
	usb_device_t *dev;
	usb_interface_desc_t hid_interface;
	usb_xfer_t intr_in_xfer;

	union {
		keyboard_t *keyboard;
		mouse_t *mouse;
	} device;

	// Bitmap for key state tracking.
	uint64_t key_state[256 / 64];

	// Report buffer.
	uint8_t __attribute__((aligned(64))) report_buffer[64];
} usb_hid_driver_data_t;

static inline bool check_key(usb_hid_driver_data_t *data, uint8_t keycode) {
	uint32_t index = keycode / 64;
	uint32_t bit = keycode % 64;
	return (data->key_state[index] & ((uint64_t)1 << bit)) != 0;
}

static inline void set_key(usb_hid_driver_data_t *data, uint8_t keycode, bool pressed) {
	uint32_t index = keycode / 64;
	uint32_t bit = keycode % 64;
	if (pressed)
		data->key_state[index] |= ((uint64_t)1 << bit);
	else
		data->key_state[index] &= ~((uint64_t)1 << bit);
}

static inline bool find_key_in_report(uint8_t *keycodes, uint8_t keycode) {
	for (uint8_t i = 0; i < 6; i++) {
		if (keycodes[i] == keycode)
			return true;
	}
	return false;
}

static uint8_t usb_hid_to_keycode(uint8_t keycode) {
	const keycode_t map[256] = {
		[0x04] = KEYCODE_A,
		[0x05] = KEYCODE_B,
		[0x06] = KEYCODE_C,
		[0x07] = KEYCODE_D,
		[0x08] = KEYCODE_E,
		[0x09] = KEYCODE_F,
		[0x0a] = KEYCODE_G,
		[0x0b] = KEYCODE_H,
		[0x0c] = KEYCODE_I,
		[0x0d] = KEYCODE_J,
		[0x0e] = KEYCODE_K,
		[0x0f] = KEYCODE_L,
		[0x10] = KEYCODE_M,
		[0x11] = KEYCODE_N,
		[0x12] = KEYCODE_O,
		[0x13] = KEYCODE_P,
		[0x14] = KEYCODE_Q,
		[0x15] = KEYCODE_R,
		[0x16] = KEYCODE_S,
		[0x17] = KEYCODE_T,
		[0x18] = KEYCODE_U,
		[0x19] = KEYCODE_V,
		[0x1a] = KEYCODE_W,
		[0x1b] = KEYCODE_X,
		[0x1c] = KEYCODE_Y,
		[0x1d] = KEYCODE_Z,
		[0x1e] = KEYCODE_1,
		[0x1f] = KEYCODE_2,
		[0x20] = KEYCODE_3,
		[0x21] = KEYCODE_4,
		[0x22] = KEYCODE_5,
		[0x23] = KEYCODE_6,
		[0x24] = KEYCODE_7,
		[0x25] = KEYCODE_8,
		[0x26] = KEYCODE_9,
		[0x27] = KEYCODE_0,
		[0x28] = KEYCODE_ENTER,
		[0x29] = KEYCODE_ESCAPE,
		[0x2a] = KEYCODE_BACKSPACE,
		[0x2b] = KEYCODE_TAB,
		[0x2c] = KEYCODE_SPACE,
		[0x2d] = KEYCODE_MINUS,
		[0x2e] = KEYCODE_EQUAL,
		[0x2f] = KEYCODE_LEFTBRACE,
		[0x30] = KEYCODE_RIGHTBRACE,
		[0x31] = KEYCODE_BACKSLASH,
		[0x33] = KEYCODE_SEMICOLON,
		[0x34] = KEYCODE_APOSTROPHE,
		[0x35] = KEYCODE_GRAVE,
		[0x36] = KEYCODE_COMMA,
		[0x37] = KEYCODE_DOT,
		[0x38] = KEYCODE_SLASH,
		[0x39] = KEYCODE_CAPSLOCK,
		[0x3a] = KEYCODE_F1,
		[0x3b] = KEYCODE_F2,
		[0x3c] = KEYCODE_F3,
		[0x3d] = KEYCODE_F4,
		[0x3e] = KEYCODE_F5,
		[0x3f] = KEYCODE_F6,
		[0x40] = KEYCODE_F7,
		[0x41] = KEYCODE_F8,
		[0x42] = KEYCODE_F9,
		[0x43] = KEYCODE_F10,
		[0x44] = KEYCODE_F11,
		[0x45] = KEYCODE_F12,
		[0x49] = KEYCODE_INSERT,
		[0x4a] = KEYCODE_HOME,
		[0x4b] = KEYCODE_PAGEUP,
		[0x4c] = KEYCODE_DELETE,
		[0x4d] = KEYCODE_END,
		[0x4e] = KEYCODE_PAGEDOWN,
		[0x4f] = KEYCODE_RIGHT,
		[0x50] = KEYCODE_LEFT,
		[0x51] = KEYCODE_DOWN,
		[0x52] = KEYCODE_UP,

		[0xe0] = KEYCODE_LEFTCTRL,
		[0xe1] = KEYCODE_LEFTSHIFT,
		[0xe2] = KEYCODE_LEFTALT,
		[0xe4] = KEYCODE_RIGHTCTRL,
		[0xe5] = KEYCODE_RIGHTSHIFT,
		[0xe6] = KEYCODE_RIGHTALT,
	};

	return map[keycode];
}

static void usb_hid_process_boot_kb_report(usb_hid_driver_data_t *data, uint8_t *report, uint32_t length) {
	__assert(length == 8);

	uint8_t mods = report[0];
	uint8_t keycodes[6];
	memcpy(keycodes, &report[2], 6);

	if (keycodes[0] == 0x1)
		return; // Ignore rollover error.

	// Process modifier keys.
	for (int i = 0; i < 8; i++) {
		bool was_pressed = check_key(data, 0xe0 + i);
		bool is_pressed = (mods & (1 << i)) != 0;

		if (was_pressed != is_pressed) {
			uint8_t keycode = usb_hid_to_keycode(0xe0 + i);
			if (keycode == KEYCODE_RESERVED)
				continue;

			kbpacket_t pkt = {0};
			pkt.keycode = keycode;
			pkt.flags = is_pressed ? 0 : KBPACKET_FLAGS_RELEASED;

			set_key(data, 0xe0 + i, is_pressed);
			keyboard_sendpacket(data->device.keyboard, &pkt);
		}
	}

	// Process regular keys.
	for (int i = 0; i < 0xe0; i++) {
		bool was_pressed = check_key(data, i);
		bool is_pressed = find_key_in_report(keycodes, i);

		if (was_pressed != is_pressed) {
			uint8_t keycode = usb_hid_to_keycode(i);
			if (keycode == KEYCODE_RESERVED)
				continue;

			kbpacket_t pkt = {0};
			pkt.keycode = keycode;
			pkt.flags = is_pressed ? 0 : KBPACKET_FLAGS_RELEASED;

			set_key(data, i, is_pressed);
			keyboard_sendpacket(data->device.keyboard, &pkt);
		}
	}
}

static void usb_hid_process_boot_mouse_report(usb_hid_driver_data_t *data, uint8_t *report, uint32_t length) {
	__assert(length >= 3);

	uint8_t buttons = report[0];
	int8_t x_move = report[1];
	int8_t y_move = report[2];
	int8_t z_move = length >= 4 ? report[3] : 0;

	mousepacket_t packet = {0};
	packet.x = x_move;
	packet.y = y_move * -1;
	packet.z = z_move;

	if (buttons & (1 << 0))
		packet.flags |= MOUSE_FLAG_LB;
	if (buttons & (1 << 1))
		packet.flags |= MOUSE_FLAG_RB;
	if (buttons & (1 << 2))
		packet.flags |= MOUSE_FLAG_MB;
	if (buttons & (1 << 3))
		packet.flags |= MOUSE_FLAG_B4;
	if (buttons & (1 << 4))
		packet.flags |= MOUSE_FLAG_B5;

	mouse_packet(data->device.mouse, &packet);
}

static void usb_hid_process_report(usb_xfer_t *xfer, usb_status_t status, uint32_t transferred) {
	usb_hid_driver_data_t *data = container_of(xfer, usb_hid_driver_data_t, intr_in_xfer);

	// Process the HID report.
	if (data->hid_interface.bInterfaceProtocol == USB_PROTOCOL_KEYBOARD)
		usb_hid_process_boot_kb_report(data, data->report_buffer, transferred);
	else if (data->hid_interface.bInterfaceProtocol == USB_PROTOCOL_MOUSE)
		usb_hid_process_boot_mouse_report(data, data->report_buffer, transferred);

	// Re-submit the interrupt IN transfer
	int res = usb_submit_xfer(data->dev, xfer);
	__assert(res == 0);
}

static int usb_hid_probe(usb_probe_ctx_t *ctx) {
	usb_for_each_descriptor(ctx->config_desc, desc) {
		if (desc->bDescriptorType != USB_DESCRIPTOR_TYPE_INTERFACE)
			continue;

		usb_interface_desc_t *iface_desc = (usb_interface_desc_t *)desc;
		if (iface_desc->bInterfaceClass != USB_CLASS_HID)
			continue;
		if (iface_desc->bInterfaceSubClass != USB_SUBCLASS_BOOT)
			continue;
		if (iface_desc->bInterfaceProtocol != USB_PROTOCOL_KEYBOARD && iface_desc->bInterfaceProtocol != USB_PROTOCOL_MOUSE)
			continue;

		return USB_DRIVER_SCORE_PROTOCOL_MATCH;
	}

	return USB_DRIVER_SCORE_NONE;
}

static int usb_hid_attach(usb_device_t *dev, usb_probe_ctx_t *ctx, void **driver_data) {
	usb_interface_desc_t *boot_interface = NULL;
	usb_endpoint_desc_t *intr_in_ep = NULL;

	// Find the HID boot protocol interface descriptor.
	usb_for_each_descriptor(ctx->config_desc, desc) {
		if (desc->bDescriptorType != USB_DESCRIPTOR_TYPE_INTERFACE)
			continue;

		usb_interface_desc_t *iface_desc = (usb_interface_desc_t *)desc;
		if (iface_desc->bInterfaceClass != USB_CLASS_HID)
			continue;
		if (iface_desc->bInterfaceSubClass != USB_SUBCLASS_BOOT)
			continue;
		if (iface_desc->bInterfaceProtocol != USB_PROTOCOL_KEYBOARD && iface_desc->bInterfaceProtocol != USB_PROTOCOL_MOUSE)
			continue;

		boot_interface = iface_desc;
		break;
	}
	__assert(boot_interface != NULL);

	// Find the interrupt IN endpoint descriptor.
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
	usb_setup_t setup = {0};
	setup.bmRequestType = USB_REQUEST_RECIP_INTERFACE | USB_REQUEST_CLASS | USB_REQUEST_DIR_TO_DEVICE;
	setup.bRequest = 0x0b; // SET_PROTOCOL
	setup.wValue = 0; // Boot Protocol
	setup.wIndex = boot_interface->bInterfaceNumber;
	setup.wLength = 0;

	usb_xfer_t xfer = {0};
	xfer.dir = USB_TRANSFER_TO_DEVICE;
	xfer.type = USB_TRANSFER_CONTROL;
	xfer.setup = &setup;
	xfer.buffer = NULL;
	xfer.length = 0;
	xfer.completion = NULL;

	int res = usb_submit_xfer(dev, &xfer);
	__assert(res == 0);

	// Send a SET_IDLE request to the device.
	setup.bmRequestType = USB_REQUEST_RECIP_INTERFACE | USB_REQUEST_CLASS | USB_REQUEST_DIR_TO_DEVICE;
	setup.bRequest = 0x0a; // SET_IDLE
	setup.wValue = 0;
	setup.wIndex = boot_interface->bInterfaceNumber;
	setup.wLength = 0;

	xfer.dir = USB_TRANSFER_TO_DEVICE;
	xfer.type = USB_TRANSFER_CONTROL;
	xfer.setup = &setup;
	xfer.buffer = NULL;
	xfer.length = 0;
	xfer.completion = NULL;

	res = usb_submit_xfer(dev, &xfer);
	__assert(res == 0);

	// Configure the interrupt IN endpoint.
	usb_endpoint_t *ep = alloc(sizeof(usb_endpoint_t));
	__assert(ep != NULL);
	memcpy(&ep->desc, intr_in_ep, sizeof(usb_endpoint_desc_t));

	res = usb_configure_endpoint(dev, ep);
	__assert(res == 0);

	usb_hid_driver_data_t *data = alloc(sizeof(usb_hid_driver_data_t));
	__assert(data != NULL);

	if (boot_interface->bInterfaceProtocol == USB_PROTOCOL_KEYBOARD)
		data->device.keyboard = keyboard_new();
	else if (boot_interface->bInterfaceProtocol == USB_PROTOCOL_MOUSE)
		data->device.mouse = mouse_new();

	*driver_data = data;

	data->dev = dev;
	memcpy(&data->hid_interface, boot_interface, sizeof(usb_interface_desc_t));

	data->intr_in_xfer.ep = ep;
	data->intr_in_xfer.dir = USB_TRANSFER_TO_HOST;
	data->intr_in_xfer.type = USB_TRANSFER_INTERRUPT;
	data->intr_in_xfer.buffer = data->report_buffer;
	data->intr_in_xfer.length = sizeof(data->report_buffer);
	data->intr_in_xfer.completion = usb_hid_process_report;

	res = usb_submit_xfer(dev, &data->intr_in_xfer);
	__assert(res == 0);

	return 0;
}

static void usb_hid_detach(usb_device_t *dev, void *driver_data) {
	_panic("usb_hid_detach: not implemented yet", NULL);
}

static usb_class_driver_t usb_hid_driver = {
	.name = "usb-hid",
	.probe = usb_hid_probe,
	.attach = usb_hid_attach,
	.detach = usb_hid_detach,
};

static void usb_hid_init(void) {
	usb_register_class_driver(&usb_hid_driver);
}

INIT_ROUTINE_DEFINE(usb_hid, INIT_ROUTINE_FLAGS_NONE, usb_hid_init, acpi);
