#include <kernel/alloc.h>
#include <kernel/usb.h>
#include <kernel/init.h>
#include <kernel/input.h>
#include <logging.h>

#define USB_CLASS_HID 0x03
#define USB_SUBCLASS_BOOT 0x01
#define USB_PROTOCOL_KEYBOARD 0x01
#define USB_PROTOCOL_MOUSE 0x02

// static int usb_hid_probe(usb_probe_ctx_t *ctx) {
// 	int score = USB_DRIVER_SCORE_NONE;

// 	usb_for_each_descriptor(ctx->config_desc, desc) {
// 		if (desc->bDescriptorType != USB_DESCRIPTOR_TYPE_INTERFACE)
// 			continue;

// 		usb_interface_desc_t *iface_desc = (usb_interface_desc_t *)desc;
// 		if (iface_desc->bInterfaceClass != USB_CLASS_HID)
// 			continue;

// 		score = USB_DRIVER_SCORE_CLASS_MATCH;

// 		// Check if we have boot protocol with a supported protocol.
// 		if (iface_desc->bInterfaceSubClass != USB_SUBCLASS_BOOT)
// 			continue;
// 		if (iface_desc->bInterfaceProtocol == USB_PROTOCOL_KEYBOARD || iface_desc->bInterfaceProtocol == USB_PROTOCOL_MOUSE)
// 			return USB_DRIVER_SCORE_PROTOCOL_MATCH;
// 	}

// 	return score;
// }

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

// These are defined in hid_boot.c and hid_report.c respectively.
int usb_hid_attach_boot(usb_device_t *dev, usb_probe_ctx_t *ctx, void **driver_data);
// int usb_hid_attach_report(usb_device_t *dev, usb_probe_ctx_t *ctx, void **driver_data);

static int usb_hid_attach(usb_device_t *dev, usb_probe_ctx_t *ctx, void **driver_data) {
	// usb_interface_desc_t *interface = NULL;
	// usb_for_each_descriptor(ctx->config_desc, desc) {
	// 	if (desc->bDescriptorType != USB_DESCRIPTOR_TYPE_INTERFACE)
	// 		continue;

	// 	usb_interface_desc_t *iface_desc = (usb_interface_desc_t *)desc;
	// 	if (iface_desc->bInterfaceClass != USB_CLASS_HID)
	// 		continue;

	// 	interface = iface_desc;
	// 	break;
	// }
	// __assert(interface != NULL);

	// First, try to attach using the report protocol.
	// int res = usb_hid_attach_report(dev, ctx, driver_data);
	// if (res == 0)
	// 	return 0;

	// If that fails, try to attach using the boot protocol.
	int res = usb_hid_attach_boot(dev, ctx, driver_data);
	if (res == 0)
		return 0;

	return res;
}

static void usb_hid_detach(usb_device_t *dev, void *driver_data) {
	_panic("usb_hid_detach: not implemented yet", NULL);
}

DEFINE_USB_CLASS_DRIVER(usb_hid_driver,
	.name = "usb-hid",
	.probe = usb_hid_probe,
	.attach = usb_hid_attach,
	.detach = usb_hid_detach
)
