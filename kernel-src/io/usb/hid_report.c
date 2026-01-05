#include <kernel/usb.h>
#include <logging.h>

int usb_hid_attach_report(usb_device_t *dev, usb_probe_ctx_t *ctx, void **driver_data) {
	printf("hid_report: attach called, not implemented yet\n");

	// Not implemented yet.
	return -1;
}
