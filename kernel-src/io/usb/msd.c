#include <kernel/alloc.h>
#include <kernel/scsi.h>
#include <kernel/usb.h>
#include <logging.h>

#define USB_CLASS_MSD 0x08
#define USB_SUBCLASS_BBB 0x06
#define USB_PROTOCOL_BULK_ONLY 0x50

typedef struct {
	usb_device_t *dev;
	scsi_host_t scsi_host;
	usb_interface_desc_t msd_interface;
	usb_endpoint_t *bulk_in_ep;
	usb_endpoint_t *bulk_out_ep;
	mutex_t mutex;
} usb_msd_driver_data_t;

#define USB_MSD_CBW_SIGNATURE 0x43425355
#define USB_MSD_CBW_FLAGS_TO_HOST 0x80

typedef struct __attribute__((packed)) {
	uint32_t dCBWSignature;
	uint32_t dCBWTag;
	uint32_t dCBWDataTransferLength;
	uint8_t bmCBWFlags;
	uint8_t bCBWLUN;
	uint8_t bCBWCBLength;
	uint8_t CBWCB[16];
} usb_msd_cbw_t;

typedef struct __attribute__((packed)) {
	uint32_t dCSWSignature;
	uint32_t dCSWTag;
	uint32_t dCSWDataResidue;
	uint8_t bCSWStatus;
} usb_msd_csw_t;

static int usb_msd_scsi_submit(scsi_host_t *host, scsi_cmd_t *cmd) {
	usb_msd_driver_data_t *data = container_of(host, usb_msd_driver_data_t, scsi_host);

	__assert(cmd->cdb_length <= 16);

	usb_msd_cbw_t cbw = {0};
	memcpy(cbw.CBWCB, cmd->cdb, cmd->cdb_length);

	cbw.dCBWSignature = USB_MSD_CBW_SIGNATURE;
	cbw.dCBWTag = 1;
	cbw.dCBWDataTransferLength = cmd->iov->total_size;
	cbw.bmCBWFlags = cmd->dir == SCSI_CMD_DIR_TO_HOST ? USB_MSD_CBW_FLAGS_TO_HOST : 0;
	cbw.bCBWLUN = 0;
	cbw.bCBWCBLength = cmd->cdb_length;

	MUTEX_ACQUIRE(&data->mutex);

	{
		usb_xfer_t xfer = {0};
		xfer.ep = data->bulk_out_ep;
		xfer.flags = USB_XFER_FLAG_TO_DEVICE;
		xfer.type = USB_XFER_TYPE_BULK;
		xfer.data = &cbw;
		xfer.data_length = sizeof(cbw);

		int res = usb_submit_xfer(data->dev, &xfer);
		__assert(res == 0);
	}

	if (cbw.bmCBWFlags & USB_MSD_CBW_FLAGS_TO_HOST) {
		usb_xfer_t xfer = {0};
		xfer.ep = data->bulk_in_ep;
		xfer.flags = USB_XFER_FLAG_TO_HOST | USB_XFER_FLAG_IOVEC;
		xfer.type = USB_XFER_TYPE_BULK;
		xfer.iov = cmd->iov;

		int res = usb_submit_xfer(data->dev, &xfer);
		__assert(res == 0);
	} else {
		usb_xfer_t xfer = {0};
		xfer.ep = data->bulk_out_ep;
		xfer.flags = USB_XFER_FLAG_TO_DEVICE | USB_XFER_FLAG_IOVEC;
		xfer.type = USB_XFER_TYPE_BULK;
		xfer.iov = cmd->iov;

		int res = usb_submit_xfer(data->dev, &xfer);
		__assert(res == 0);
	}

	usb_msd_csw_t csw;
	usb_xfer_t xfer = {0};
	xfer.ep = data->bulk_in_ep;
	xfer.flags = USB_XFER_FLAG_TO_HOST;
	xfer.type = USB_XFER_TYPE_BULK;
	xfer.data = &csw;
	xfer.data_length = sizeof(csw);

	int res = usb_submit_xfer(data->dev, &xfer);
	__assert(res == 0);

	MUTEX_RELEASE(&data->mutex);

	cmd->status = csw.bCSWStatus;
	if (cmd->complete != NULL)
		cmd->complete(host, cmd, cmd->status);

	return 0;
}

static scsi_host_ops_t usb_msd_scsi_host_ops = {
	.submit = usb_msd_scsi_submit,
};

static int usb_msd_probe(usb_probe_ctx_t *ctx) {
	usb_for_each_descriptor(ctx->config_desc, desc) {
		if (desc->bDescriptorType != USB_DESCRIPTOR_TYPE_INTERFACE)
			continue;

		usb_interface_desc_t *iface_desc = (usb_interface_desc_t *)desc;
		if (iface_desc->bInterfaceClass != USB_CLASS_MSD)
			continue;
		if (iface_desc->bInterfaceSubClass != USB_SUBCLASS_BBB)
			continue;
		if (iface_desc->bInterfaceProtocol != USB_PROTOCOL_BULK_ONLY)
			continue;

		return USB_DRIVER_SCORE_PROTOCOL_MATCH;
	}

	return USB_DRIVER_SCORE_NONE;
}

static int usb_msd_attach(usb_device_t *dev, usb_probe_ctx_t *ctx, void **driver_data) {
	usb_interface_desc_t *msd_interface = NULL;
	usb_endpoint_desc_t *bulk_in_ep = NULL;
	usb_endpoint_desc_t *bulk_out_ep = NULL;

	// Find the MSD interface descriptor.
	usb_for_each_descriptor(ctx->config_desc, desc) {
		if (desc->bDescriptorType != USB_DESCRIPTOR_TYPE_INTERFACE)
			continue;

		usb_interface_desc_t *iface_desc = (usb_interface_desc_t *)desc;
		if (iface_desc->bInterfaceClass != USB_CLASS_MSD)
			continue;
		if (iface_desc->bInterfaceSubClass != USB_SUBCLASS_BBB)
			continue;
		if (iface_desc->bInterfaceProtocol != USB_PROTOCOL_BULK_ONLY)
			continue;

		msd_interface = iface_desc;
		break;
	}
	__assert(msd_interface != NULL);

	// Find the bulk IN and bulk OUT endpoint descriptors.
	usb_for_each_descriptor(ctx->config_desc, desc) {
		if (desc->bDescriptorType != USB_DESCRIPTOR_TYPE_ENDPOINT)
			continue;

		usb_endpoint_desc_t *ep_desc = (usb_endpoint_desc_t *)desc;
		if ((ep_desc->bmAttributes & USB_ENDPOINT_ATTRIB_TYPE_MASK) != USB_ENDPOINT_ATTRIB_TYPE_BULK)
			continue;

		if (ep_desc->bEndpointAddress & USB_ENDPOINT_ADDRESS_DIR_IN) {
			bulk_in_ep = ep_desc;
		} else {
			bulk_out_ep = ep_desc;
		}
	}
	__assert(bulk_in_ep != NULL);
	__assert(bulk_out_ep != NULL);

	// Send a Bulk-Only Mass Storage Reset request to the device.
	usb_setup_t setup = {0};
	setup.bmRequestType = USB_REQUEST_RECIP_INTERFACE | USB_REQUEST_CLASS | USB_REQUEST_DIR_TO_DEVICE;
	setup.bRequest = 0xff; // Bulk-Only Mass Storage Reset
	setup.wValue = 0;
	setup.wIndex = msd_interface->bInterfaceNumber;
	setup.wLength = 0;

	int res = usb_control_xfer(dev, &setup, NULL);
	__assert(res == 0);

	// Set up the IN and OUT endpoints.
	usb_endpoint_t *in_ep = alloc(sizeof(usb_endpoint_t));
	__assert(in_ep != NULL);
	memcpy(&in_ep->desc, bulk_in_ep, sizeof(usb_endpoint_desc_t));

	usb_endpoint_t *out_ep = alloc(sizeof(usb_endpoint_t));
	__assert(out_ep != NULL);
	memcpy(&out_ep->desc, bulk_out_ep, sizeof(usb_endpoint_desc_t));

	res = usb_configure_endpoint(dev, in_ep);
	__assert(res == 0);

	res = usb_configure_endpoint(dev, out_ep);
	__assert(res == 0);

	// Allocate and fill driver data.
	usb_msd_driver_data_t *data = alloc(sizeof(usb_msd_driver_data_t));
	__assert(data != NULL);

	MUTEX_INIT(&data->mutex);

	data->dev = dev;
	memcpy(&data->msd_interface, msd_interface, sizeof(usb_interface_desc_t));

	data->bulk_in_ep = in_ep;
	data->bulk_out_ep = out_ep;

	data->scsi_host.ops = &usb_msd_scsi_host_ops;
	scsi_register(&data->scsi_host);

	*driver_data = data;
	return 0;
}

static void usb_msd_detach(usb_device_t *dev, void *driver_data) {
	_panic("usb_msd_detach: not implemented yet", NULL);
}

DEFINE_USB_CLASS_DRIVER(usb_msd_driver,
	.name = "usb-msd",
	.probe = usb_msd_probe,
	.attach = usb_msd_attach,
	.detach = usb_msd_detach
)
