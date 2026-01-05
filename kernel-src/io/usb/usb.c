#include <kernel/alloc.h>
#include <kernel/pmm.h>
#include <kernel/usb.h>
#include <logging.h>

extern usb_class_driver_t *usb_class_drivers;
extern usb_class_driver_t *usb_class_drivers_end;

static int usb_probe_device(usb_device_t *dev) {
	int res;

	usb_device_desc_t dev_desc;
	res = usb_get_descriptor(dev, USB_DESCRIPTOR_TYPE_DEVICE, 0, &dev_desc, sizeof(dev_desc));
	__assert(res == 0);

	// Fetch all configuration descriptors.
	usb_config_desc_t **config_descs = alloc(sizeof(usb_config_desc_t *) * dev_desc.bNumConfigurations);
	__assert(config_descs != NULL);

	// Read all configuration descriptors.
	for (uint32_t i = 0; i < dev_desc.bNumConfigurations; i++) {
		usb_config_desc_t config_desc;
		res = usb_get_descriptor(dev, USB_DESCRIPTOR_TYPE_CONFIG, i, &config_desc, sizeof(usb_config_desc_t));
		__assert(res == 0);

		// Now allocate and read the full configuration descriptor.
		void *full_config_desc = alloc(config_desc.wTotalLength);
		__assert(full_config_desc != NULL);

		res = usb_get_descriptor(dev, USB_DESCRIPTOR_TYPE_CONFIG, i, full_config_desc, config_desc.wTotalLength);
		__assert(res == 0);

		config_descs[i] = full_config_desc;
	}

	// Try to find the best matching driver.
	usb_class_driver_t *best_driver = NULL;
	uint32_t best_config = 0;
	int best_score = 0;

	// Probe all configurations with all drivers.
	for (uint32_t i = 0; i < dev_desc.bNumConfigurations; i++) {
		usb_probe_ctx_t ctx;
		ctx.device = dev;
		ctx.device_desc = &dev_desc;
		ctx.config_desc = config_descs[i];

		for (usb_class_driver_t **it = &usb_class_drivers; it < &usb_class_drivers_end; ++it) {
			usb_class_driver_t *driver = *it;
			int score = driver->probe(&ctx);

			if (score > best_score) {
				best_score = score;
				best_driver = driver;
				best_config = i;
			}
		}
	}

	if (best_driver) {
		printf("usb: device matched with driver '%s' (score: %d, config: %u)\n",
			best_driver->name, best_score, best_config);

		// Enable the selected configuration.
		usb_set_configuration(dev, config_descs[best_config]->bConfigurationValue);

		usb_probe_ctx_t ctx;
		ctx.device = dev;
		ctx.device_desc = &dev_desc;
		ctx.config_desc = config_descs[best_config];

		void *driver_data = NULL;
		res = best_driver->attach(dev, &ctx, &driver_data);
		__assert(res == 0);

		dev->driver = best_driver;
		dev->driver_data = driver_data;
	}

	// Free configuration descriptors.
	for (uint32_t i = 0; i < dev_desc.bNumConfigurations; i++)
		free(config_descs[i]);

	return best_driver ? 0 : -ENODEV;
}

int usb_hub_start(usb_hub_t *hub) {
	// Power on all ports
	for (uint8_t port = 0; port < hub->port_count; port++) {
		int res = hub->ops->set_port_feature(hub, port + 1, USB_HUB_FEATURE_PORT_POWER);
		__assert(res == 0);
	}

	// Enumerate all devices on the hub
	int res = hub->ctrl->ops->enumerate(hub->ctrl);
	__assert(res == 0);

	return 0;
}

int usb_hub_enumerate(usb_hub_t *hub) {
	for (uint8_t port = 0; port < hub->port_count; port++) {
		int res = usb_hub_enumerate_port(hub, port + 1);
		__assert(res == 0);
	}

	return 0;
}

int usb_hub_enumerate_port(usb_hub_t *hub, uint8_t port) {
	if (port == 0 || port > hub->port_count)
		return -EINVAL;

	uint16_t status, change;

	int res = hub->ops->get_port_status(hub, port, &status, &change);
	if (res != 0)
		return res;

	// printf("usb: hub '%s' port %u status=0x%04x change=0x%04x\n", hub->name, port, status, change);

	usb_hub_port_t *hub_port = &hub->ports[port - 1];

	if (change & USB_HUB_PORT_CHANGE_PORT_CONNECTION) {
		res = hub->ops->clear_port_feature(hub, port, USB_HUB_FEATURE_C_PORT_CONNECTION);
		__assert(res == 0);

		if (hub_port->status == USB_HUB_PORT_DISCONNECTED && status & USB_HUB_PORT_STATUS_PORT_CONNECTION) {
			printf("usb: device connected: %s:%u\n", hub->name, port);
			hub_port->status = USB_HUB_PORT_RESETTING;

			res = hub->ops->set_port_feature(hub, port, USB_HUB_FEATURE_PORT_RESET);
			__assert(res == 0);
		} else if (hub_port->status != USB_HUB_PORT_DISCONNECTED && !(status & USB_HUB_PORT_STATUS_PORT_CONNECTION)) {
			printf("usb: device disconnected: %s:%u\n", hub->name, port);

			// Call detach on the driver if one is attached
			usb_device_t *dev = hub_port->device;
			if (dev != NULL && dev->driver != NULL) {
				dev->driver->detach(dev, dev->driver_data);
				dev->driver = NULL;
				dev->driver_data = NULL;
			}

			hub_port->status = USB_HUB_PORT_DISCONNECTED;
			hub_port->device = NULL;
		}
	}

	if (change & USB_HUB_PORT_CHANGE_PORT_RESET) {
		res = hub->ops->clear_port_feature(hub, port, USB_HUB_FEATURE_C_PORT_RESET);
		__assert(res == 0);

		if (status & USB_HUB_PORT_STATUS_PORT_ENABLE) {
			printf("usb: device enabled: %s:%u\n", hub->name, port);

			usb_device_t *dev;
			res = hub->ctrl->ops->address_device(hub->ctrl, hub, port, &dev);
			__assert(res == 0);

			hub_port->status = USB_HUB_PORT_ENABLED;
			hub_port->device = dev;

			res = usb_probe_device(dev);
			if (res != 0) {
				printf("usb: no driver for device on %s:%u\n", hub->name, port);

				hub_port->status = USB_HUB_PORT_DISCONNECTED;
				hub_port->device = NULL;

				// TODO: Free device structure and disable port
			}
		} else {
			hub_port->status = USB_HUB_PORT_DISCONNECTED;
		}
	}

	return 0;
}

int usb_submit_xfer(usb_device_t *dev, usb_xfer_t *xfer) {
	usb_ctrl_t *ctrl = dev->hub->ctrl;
	return ctrl->ops->xfer(ctrl, dev, xfer);
}

int usb_control_xfer(usb_device_t *dev, usb_setup_t *setup, void *buffer) {
	usb_xfer_t xfer = {0};
	xfer.flags = (setup->bmRequestType & USB_REQUEST_DIR_TO_HOST) ? USB_XFER_FLAG_TO_HOST : USB_XFER_FLAG_TO_DEVICE;
	xfer.type = USB_XFER_TYPE_CONTROL;
	xfer.setup = setup;
	xfer.data = buffer;
	xfer.data_length = setup->wLength;

	return usb_submit_xfer(dev, &xfer);
}

int usb_configure_endpoint(usb_device_t *dev, usb_endpoint_t *ep) {
	usb_ctrl_t *ctrl = dev->hub->ctrl;
	return ctrl->ops->configure_ep(ctrl, dev, ep);
}

int usb_get_descriptor(usb_device_t *dev, uint8_t desc_type, uint8_t desc_index, void *buffer, uint16_t length) {
	usb_setup_t setup = {0};
	setup.bmRequestType = USB_REQUEST_RECIP_DEVICE | USB_REQUEST_STANDARD | USB_REQUEST_DIR_TO_HOST;
	setup.bRequest = USB_REQUEST_GET_DESCRIPTOR;
	setup.wValue = ((uint16_t)desc_type << 8) | desc_index;
	setup.wIndex = 0;
	setup.wLength = length;

	return usb_control_xfer(dev, &setup, buffer);
}

int usb_set_configuration(usb_device_t *dev, uint8_t config_value) {
	usb_setup_t setup = {0};
	setup.bmRequestType = USB_REQUEST_RECIP_DEVICE | USB_REQUEST_STANDARD | USB_REQUEST_DIR_TO_DEVICE;
	setup.bRequest = USB_REQUEST_SET_CONFIGURATION;
	setup.wValue = config_value;
	setup.wIndex = 0;
	setup.wLength = 0;

	return usb_control_xfer(dev, &setup, NULL);
}

int usb_set_interface(usb_device_t *dev, uint8_t interface_number, uint8_t alt_setting) {
	usb_setup_t setup = {0};
	setup.bmRequestType = USB_REQUEST_RECIP_INTERFACE | USB_REQUEST_STANDARD | USB_REQUEST_DIR_TO_DEVICE;
	setup.bRequest = USB_REQUEST_SET_INTERFACE;
	setup.wValue = alt_setting;
	setup.wIndex = interface_number;
	setup.wLength = 0;

	return usb_control_xfer(dev, &setup, NULL);
}
