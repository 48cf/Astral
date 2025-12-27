#include <arch/cpu.h>
#include <kernel/alloc.h>
#include <kernel/init.h>
#include <kernel/interrupt.h>
#include <kernel/pci.h>
#include <kernel/pmm.h>
#include <kernel/usb.h>
#include <kernel/vmm.h>
#include <kernel/xhci.h>
#include <list.h>
#include <logging.h>
#include <util.h>

typedef struct {
	uint8_t ver_major;
	uint8_t ver_minor;
} xhci_port_protocol_t;

typedef struct xhci_submission {
	list_node_t node;
	xhci_trb_t *trb;

	// Filled in right before the callback is invoked.
	xhci_trb_t event_trb;
	xhci_trb_t associated_trb;

	// The callback to invoke.
	void (*completion)(struct xhci_submission *);
	void *completion_ctx;
} xhci_submission_t;

typedef struct {
	void *ring_phys;
	xhci_trb_t *ring;

	// Back pointer to the controller.
	struct xhci_ctrl *ctrl;

	// Ring size in TRBs.
	size_t size;

	// For event rings, the index of the next event to process.
	// For command and transfer rings, the index of the next free TRB.
	size_t index;

	// The doorbell for this ring.
	// This value is NULL for event rings.
	volatile uint32_t *doorbell;
	uint32_t doorbell_value;

	// Cycle bit for the ring.
	bool cycle;
} xhci_ring_t;

typedef struct xhci_ctrl {
	usb_ctrl_t ctrl;
	pcienum_t *pci_enum;

	size_t port_count;
	xhci_port_protocol_t *ports;

	xhci_ring_t command_ring;
	xhci_ring_t event_ring;

	semaphore_t sem;
	semaphore_t ev_sem;

	spinlock_t lock;
	list_t submissions;

	uint32_t ctx_stride;

	volatile xhci_caps_t *caps;
	volatile xhci_opregs_t *opregs;
	volatile xhci_rtregs_t *rtregs;
	volatile xhci_port_regs_t *portregs;

	volatile uint64_t *dcbaa;
	volatile uint32_t *dbs;
} xhci_ctrl_t;

typedef struct {
	usb_hub_t hub;
	uint8_t port_offset;
} xhci_root_hub_t;

typedef struct {
	usb_device_t device;
	xhci_ring_t ep_rings[31];

	uint32_t route_string;
	uint8_t device_tier;
	uint8_t slot_id;

	void *device_ctx_phys;
	void *device_ctx;
	void *input_ctx_phys;
	void *input_ctx;
} xhci_device_t;

#define CTX_PTR(CTRL, CTX, INDEX) ((uintptr_t)(CTX) + ((CTRL)->ctx_stride * (INDEX)))

static volatile xhci_input_ctx_t *xhci_get_input_ctrl_ctx(xhci_ctrl_t *xhci, xhci_device_t *dev) {
	return (volatile xhci_input_ctx_t *)CTX_PTR(xhci, dev->input_ctx, 0);
}

static volatile xhci_slot_ctx_t *xhci_get_input_slot_ctx(xhci_ctrl_t *xhci, xhci_device_t *dev) {
	return (volatile xhci_slot_ctx_t *)CTX_PTR(xhci, dev->input_ctx, 1);
}

static volatile xhci_ep_ctx_t *xhci_get_input_ep_ctx(xhci_ctrl_t *xhci, xhci_device_t *dev, uint8_t ep) {
	return (volatile xhci_ep_ctx_t *)CTX_PTR(xhci, dev->input_ctx, ep + 2);
}

static void xhci_update_input_context(xhci_ctrl_t *ctrl, xhci_device_t *dev) {
	uint8_t *input_ctx = dev->input_ctx;
	uint8_t *device_ctx = dev->device_ctx;

	// Copy over the slot context and all endpoint contexts.
	memcpy(input_ctx + ctrl->ctx_stride, device_ctx, ctrl->ctx_stride * (1 + 31));
	// Clear input context header.
	memset(input_ctx, 0, sizeof(uint32_t) * 2);
}

static bool xhci_alloc_ring(xhci_ctrl_t *ctrl, xhci_ring_t *r, volatile uint32_t *doorbell, uint32_t doorbell_value) {
	void *ring_phys = pmm_allocpage(PMM_SECTION_DEFAULT);
	if (ring_phys == NULL)
		return false;

	xhci_trb_t *ring = MAKE_HHDM(ring_phys);
	memset((void *)ring, 0, PAGE_SIZE);

	r->ring_phys = ring_phys;
	r->ring = ring;
	r->ctrl = ctrl;
	r->size = PAGE_SIZE / sizeof(xhci_trb_t);
	r->index = 0;
	r->doorbell = doorbell;
	r->doorbell_value = doorbell_value;
	r->cycle = true;

	return true;
}

static void xhci_ring_submit(xhci_ring_t *r, xhci_trb_t *trb, xhci_submission_t *sub, bool ring_db) {
	// Cycle bit must be 0 in the TRB being submitted.
	__assert((trb->dw3 & XHCI_TRB_DW3_C) == 0);

	if (r->index == r->size - 1) {
		// If we're at the end of the ring, we need to set up a link TRB.
		xhci_trb_t *link_trb = &r->ring[r->index];
		link_trb->parameters = (uint64_t)r->ring_phys;
		link_trb->dw2 = 0;
		link_trb->dw3 = XHCI_TRB_DW3_TYPE(TRB_LINK) | XHCI_TRB_DW3_TC | r->cycle;
		// Now wrap around to start of ring and flip cycle bit.
		r->index = 0;
		r->cycle ^= 1;
	}

	size_t idx = r->index++;
	xhci_trb_t *r_trb = &r->ring[idx];

	memcpy(r_trb, trb, sizeof(xhci_trb_t));

	// Set cycle bit appropriately.
	if (r->cycle)
		r_trb->dw3 |= XHCI_TRB_DW3_C;

	if (sub != NULL) {
		// Store back pointer to the submitted TRB so we can
		// associate the completion event with the submission later.
		sub->trb = r_trb;

		spinlock_acquire(&r->ctrl->lock);
		list_push_back(&r->ctrl->submissions, &sub->node);
		spinlock_release(&r->ctrl->lock);
	}

	// Ring the doorbell.
	if (ring_db)
		*r->doorbell = r->doorbell_value;
}

typedef struct {
	xhci_trb_t event_trb;
	xhci_trb_t associated_trb;
	semaphore_t *sem;
} xhci_submit_ctx_t;

static void xhci_ring_complete(xhci_submission_t *sub) {
	xhci_submit_ctx_t *ctx = sub->completion_ctx;

	memcpy(&ctx->event_trb, &sub->event_trb, sizeof(xhci_trb_t));
	memcpy(&ctx->associated_trb, &sub->associated_trb, sizeof(xhci_trb_t));

	semaphore_signal(ctx->sem);
}

static void xhci_ring_submit_and_wait(xhci_ring_t *r, xhci_trb_t *trb, xhci_trb_t *event_trb, xhci_trb_t *associated_trb) {
	semaphore_t sem;
	SEMAPHORE_INIT(&sem, 0);

	xhci_submit_ctx_t ctx;
	ctx.sem = &sem;

	xhci_submission_t sub;
	sub.completion = xhci_ring_complete;
	sub.completion_ctx = &ctx;

	xhci_ring_submit(r, trb, &sub, true);
	semaphore_wait(&sem, false);

	// Copy out the completed TRBs.
	if (event_trb != NULL)
		memcpy(event_trb, &ctx.event_trb, sizeof(xhci_trb_t));
	if (associated_trb != NULL)
		memcpy(associated_trb, &ctx.associated_trb, sizeof(xhci_trb_t));
}

static xhci_trb_t *xhci_ring_dequeue(xhci_ring_t *r) {
	xhci_trb_t *r_trb = &r->ring[r->index];

	// Make sure the cycle bit matches.
	if (((r_trb->dw3 & XHCI_TRB_DW3_C) != 0) != r->cycle)
		return NULL;

	// Advance offset and flip cycle bit if needed.
	r->index++;

	if (r->index == r->size) {
		r->index = 0;
		r->cycle ^= 1;
	}

	return r_trb;
}

static int xhci_halt(xhci_ctrl_t *ctrl) {
	if ((ctrl->opregs->usbcmd & XHCI_USBCMD_RS) == 0)
		return 0;

	ctrl->opregs->usbcmd &= ~XHCI_USBCMD_RS;

	timespec_t start = timekeeper_timefromboot();
	while ((ctrl->opregs->usbsts & XHCI_USBSTS_HCH) == 0) {
		sched_sleep_us(10 * 1000); // 10ms

		timespec_t now = timekeeper_timefromboot();
		if (timespec_diffms(now, start) > 1000)
			return -1;
	}

	return 0;
}

static int xhci_reset(xhci_ctrl_t *ctrl) {
	ctrl->opregs->usbcmd |= XHCI_USBCMD_HCRST;

	timespec_t start = timekeeper_timefromboot();
	while ((ctrl->opregs->usbsts & XHCI_USBSTS_CNR) != 0) {
		sched_sleep_us(10 * 1000); // 10ms

		timespec_t now = timekeeper_timefromboot();
		if (timespec_diffms(now, start) > 1000)
			return -1;
	}

	return 0;
}

static int xhci_run(xhci_ctrl_t *ctrl) {
	if ((ctrl->opregs->usbcmd & XHCI_USBCMD_RS) != 0)
		return 0;

	ctrl->opregs->usbcmd |= XHCI_USBCMD_RS | XHCI_USBCMD_INTE;

	timespec_t start = timekeeper_timefromboot();
	while ((ctrl->opregs->usbsts & XHCI_USBSTS_CNR) != 0) {
		sched_sleep_us(10 * 1000); // 10ms

		timespec_t now = timekeeper_timefromboot();
		if (timespec_diffms(now, start) > 1000)
			return -1;
	}

	return 0;
}

static int xhci_root_hub_reset_port(usb_hub_t *hub, uint8_t port) {
	_panic("xhci_root_hub_reset_port: not implemented yet", NULL);
}

static int xhci_root_hub_get_port_status(usb_hub_t *hub, uint8_t port, uint16_t *status, uint16_t *change) {
	xhci_root_hub_t *rh = container_of(hub, xhci_root_hub_t, hub);
	xhci_ctrl_t *ctrl = container_of(hub->ctrl, xhci_ctrl_t, ctrl);

	if (port == 0 || port > hub->port_count)
		return -EINVAL;

	uint32_t portsc = ctrl->portregs[rh->port_offset + port - 1].portsc;
	uint32_t port_speed = (portsc >> 10) & 0xf;

	*status = 0;
	*change = 0;

	if (port_speed == XHCI_PORT_SPEED_LOW)
		*status |= USB_HUB_PORT_STATUS_LOW_SPEED;
	else if (port_speed == XHCI_PORT_SPEED_HIGH)
		*status |= USB_HUB_PORT_STATUS_HIGH_SPEED;

	if (portsc & XHCI_PORTSC_CCS)
		*status |= USB_HUB_PORT_STATUS_PORT_CONNECTION;
	if (portsc & XHCI_PORTSC_PED)
		*status |= USB_HUB_PORT_STATUS_PORT_ENABLE;
	if (portsc & XHCI_PORTSC_OCA)
		*status |= USB_HUB_PORT_STATUS_PORT_OVER_CURRENT;
	if (portsc & XHCI_PORTSC_PR)
		*status |= USB_HUB_PORT_STATUS_PORT_RESET;
	if (portsc & XHCI_PORTSC_PP)
		*status |= USB_HUB_PORT_STATUS_PORT_POWER;

	if (portsc & XHCI_PORTSC_CSC)
		*change |= USB_HUB_PORT_CHANGE_PORT_CONNECTION;
	if (portsc & XHCI_PORTSC_PEC)
		*change |= USB_HUB_PORT_CHANGE_PORT_ENABLE;
	if (portsc & XHCI_PORTSC_OCC)
		*change |= USB_HUB_PORT_CHANGE_PORT_OVER_CURRENT;
	if (portsc & XHCI_PORTSC_PRC)
		*change |= USB_HUB_PORT_CHANGE_PORT_RESET;

	return 0;
}

static int xhci_root_hub_set_port_feature(usb_hub_t *hub, uint8_t port, uint16_t feature) {
	xhci_root_hub_t *rh = container_of(hub, xhci_root_hub_t, hub);
	xhci_ctrl_t *ctrl = container_of(hub->ctrl, xhci_ctrl_t, ctrl);

	if (port == 0 || port > hub->port_count)
		return -EINVAL;

	volatile xhci_port_regs_t *portregs = &ctrl->portregs[rh->port_offset + port - 1];

	uint32_t portsc = portregs->portsc;
	portsc &= ~(XHCI_PORTSC_CCS | XHCI_PORTSC_PED | XHCI_PORTSC_OCA | XHCI_PORTSC_PR);
	portsc &= ~(XHCI_PORTSC_CSC | XHCI_PORTSC_PEC | XHCI_PORTSC_OCC | XHCI_PORTSC_PRC);

	if (feature == USB_HUB_FEATURE_PORT_ENABLE)
		portsc |= XHCI_PORTSC_PED;
	else if (feature == USB_HUB_FEATURE_PORT_RESET)
		portsc |= XHCI_PORTSC_PR;
	else if (feature == USB_HUB_FEATURE_PORT_POWER)
		portsc |= XHCI_PORTSC_PP;
	else
		return -EINVAL;

	portregs->portsc = portsc;
	return 0;
}

static int xhci_root_hub_clear_port_feature(usb_hub_t *hub, uint8_t port, uint16_t feature) {
	xhci_root_hub_t *rh = container_of(hub, xhci_root_hub_t, hub);
	xhci_ctrl_t *ctrl = container_of(hub->ctrl, xhci_ctrl_t, ctrl);

	if (port == 0 || port > hub->port_count)
		return -EINVAL;

	volatile xhci_port_regs_t *portregs = &ctrl->portregs[rh->port_offset + port - 1];

	uint32_t portsc = portregs->portsc;
	portsc &= ~(XHCI_PORTSC_CCS | XHCI_PORTSC_PED | XHCI_PORTSC_OCA | XHCI_PORTSC_PR);
	portsc &= ~(XHCI_PORTSC_CSC | XHCI_PORTSC_PEC | XHCI_PORTSC_OCC | XHCI_PORTSC_PRC);

	if (feature == USB_HUB_FEATURE_PORT_ENABLE)
		portsc &= ~XHCI_PORTSC_PED;
	else if (feature == USB_HUB_FEATURE_PORT_RESET)
		portsc &= ~XHCI_PORTSC_PR;
	else if (feature == USB_HUB_FEATURE_PORT_POWER)
		portsc &= ~XHCI_PORTSC_PP;
	else if (feature == USB_HUB_FEATURE_C_PORT_CONNECTION)
		portsc |= XHCI_PORTSC_CSC;
	else if (feature == USB_HUB_FEATURE_C_PORT_RESET)
		portsc |= XHCI_PORTSC_PRC;
	else
		return -EINVAL;

	portregs->portsc = portsc;
	return 0;
}

static usb_hub_ops_t xhci_root_hub_ops = {
	.reset_port = xhci_root_hub_reset_port,
	.get_port_status = xhci_root_hub_get_port_status,
	.set_port_feature = xhci_root_hub_set_port_feature,
	.clear_port_feature = xhci_root_hub_clear_port_feature,
};

static void xhci_handle_events(xhci_ctrl_t *xhci) {
	volatile xhci_ir_t *ir = &xhci->rtregs->ir[0];

	// Process events from the event ring.
	xhci_trb_t *r_trb;

	if ((r_trb = xhci_ring_dequeue(&xhci->event_ring)) != NULL) {
		// Copy the event TRB so we can advance the ring dequeue pointer.
		xhci_trb_t event_trb;
		memcpy(&event_trb, r_trb, sizeof(xhci_trb_t));

		// Update the Event Ring Dequeue Pointer.
		uint64_t event_ring_phys = (uint64_t)xhci->event_ring.ring_phys;
		event_ring_phys += xhci->event_ring.index * sizeof(xhci_trb_t);
		ir->erdp = event_ring_phys | XHCI_ERDP_EHB;

		uint32_t type = (event_trb.dw3 >> 10) & 0x3f;
		if (type == TRB_COMMAND_COMPLETION_EVENT || type == TRB_XFER_COMPLETION_EVENT) {
			uint32_t status = (event_trb.dw2 >> 24) & 0xff;
			__assert(status == TRB_SUCCESS || status == TRB_SHORT_PACKET);

			xhci_trb_t *trb = MAKE_HHDM(event_trb.parameters);
			xhci_submission_t *sub = NULL;

			spinlock_acquire(&xhci->lock);

			list_for_each_safe(&xhci->submissions, node) {
				xhci_submission_t *s = container_of(node, xhci_submission_t, node);
				if (s->trb == trb) {
					sub = s;
					list_remove(&xhci->submissions, &s->node);
					break;
				}
			}

			spinlock_release(&xhci->lock);

			__assert(sub != NULL);
			__assert(sub->completion != NULL);

			// Copy the event TRB into the submission.
			memcpy(&sub->event_trb, &event_trb, sizeof(xhci_trb_t));
			memcpy(&sub->associated_trb, trb, sizeof(xhci_trb_t));

			// Invoke the completion callback.
			sub->completion(sub);
		} else {
			printf("xhci: unknown event TRB type %u\n", type);
		}
	}

	// Clear the interrupt pending bit.
	ir->iman |= XHCI_IMAN_IP;
}

static void xhci_isr(isr_t *isr, context_t *ctx) {
	xhci_ctrl_t *xhci = isr->priv;

	// Check if there is an interrupt pending.
	volatile xhci_ir_t *ir = &xhci->rtregs->ir[0];

	if ((ir->erdp & XHCI_ERDP_EHB) == 0)
		return;

	// Wake up the event thread.
	semaphore_signal(&xhci->ev_sem);
}

static int xhci_ctrl_start(usb_ctrl_t *ctrl) {
	int ret = 0;
	xhci_ctrl_t *xhci = container_of(ctrl, xhci_ctrl_t, ctrl);
	pcibar_t bar0 = pci_getbar(xhci->pci_enum, 0);

	pci_setcommand(xhci->pci_enum, PCI_COMMAND_MMIO, 1);
	pci_setcommand(xhci->pci_enum, PCI_COMMAND_IO, 0);
	pci_setcommand(xhci->pci_enum, PCI_COMMAND_IRQDISABLE, 1);
	pci_setcommand(xhci->pci_enum, PCI_COMMAND_BUSMASTER, 1);

	ret = xhci_halt(xhci);
	if (ret != 0) {
		printf("xhci: controller halt timeout\n");
		return ret;
	}

	ret = xhci_reset(xhci);
	if (ret != 0) {
		printf("xhci: controller reset timeout\n");
		return ret;
	}

	// Enable MSI interrupts.
	size_t intcount;
	if (xhci->pci_enum->msix.exists) {
		intcount = pci_initmsix(xhci->pci_enum);
	} else {
		printf("xhci: controller doesn't support msi-x\n");
		return -ENODEV;
	}
	__assert(intcount > 0);

	isr_t *isr = interrupt_allocate(xhci_isr, ARCH_EOI, IPL_USB);
	__assert(isr != NULL);
	isr->priv = ctrl;

	pci_msixadd(xhci->pci_enum, 0, INTERRUPT_IDTOVECTOR(isr->id), 1, 0);
	pci_msixsetmask(xhci->pci_enum, 0);

	uint32_t max_slots = xhci->caps->hcsparams1 & 0xff;
	uint32_t max_ports = (xhci->caps->hcsparams1 >> 24) & 0xff;

	printf("xhci: controller supports %u slots and %u ports\n", max_slots, max_ports);

	xhci->port_count = max_ports;
	xhci->ports = alloc(sizeof(xhci_port_protocol_t) * max_ports);
	__assert(xhci->ports != NULL);

	xhci->ctx_stride = (xhci->caps->hccparams1 & XHCI_HCCPARAMS1_CSZ) != 0 ? 64 : 32;

	for (uint32_t i = 0; i < max_ports; i++) {
		xhci->ports[i].ver_major = 0;
		xhci->ports[i].ver_minor = 0;
	}

	uint32_t ext_caps_offset = (xhci->caps->hccparams1 >> 16) & 0xffff;
	uint32_t *ext_caps = (uint32_t *)(bar0.address + ext_caps_offset * 4);

	for (;;) {
		uint8_t cap_id = *ext_caps & 0xff;
		uint8_t next_cap_off = (*ext_caps >> 8) & 0xff;

		if (cap_id == 2) {
			uint8_t ver_major = (*ext_caps >> 24) & 0xff;
			uint8_t ver_minor = (*ext_caps >> 16) & 0xff;

			uint8_t port_start = (ext_caps[2] & 0xff) - 1;
			uint8_t port_count = (ext_caps[2] >> 8) & 0xff;

			printf("xhci: %u usb %u.%u ports starting at port %u\n",
				port_count, ver_major, ver_minor, port_start + 1);

			for (uint32_t i = 0; i < port_count; i++) {
				xhci->ports[port_start + i].ver_major = ver_major;
				xhci->ports[port_start + i].ver_minor = ver_minor;
			}

			xhci_root_hub_t *hub = alloc(sizeof(xhci_root_hub_t));
			__assert(hub != NULL);

			snprintf(hub->hub.name, sizeof(hub->hub.name), "xhci-root-hub-%u.%u", ver_major, ver_minor);

			hub->hub.ctrl = ctrl;
			hub->hub.ops = &xhci_root_hub_ops;
			hub->hub.device = NULL;

			hub->hub.ports = alloc(sizeof(usb_hub_port_t) * port_count);
			hub->hub.port_count = port_count;

			hub->port_offset = port_start;

			list_push_back(&xhci->ctrl.hubs, &hub->hub.node);
		}

		if (next_cap_off == 0)
			break;
		ext_caps += next_cap_off;
	}

	__assert(xhci_alloc_ring(xhci, &xhci->command_ring, &xhci->dbs[0], 0));
	__assert(xhci_alloc_ring(xhci, &xhci->event_ring, NULL, 0));

	// Set up MaxSlotsEn field.
	xhci->opregs->config = max_slots;

	// Set up the Device Context Base Address Array.
	void *dcbaa_phys = pmm_allocpage(PMM_SECTION_DEFAULT);
	__assert(dcbaa_phys != NULL);

	uint64_t *dcbaa_virt = MAKE_HHDM(dcbaa_phys);
	memset(dcbaa_virt, 0, PAGE_SIZE);

	// Set up scratchpad buffers if requested.
	uint32_t max_scratchpads_lo = (xhci->caps->hcsparams2 >> 21) & 0x1f;
	uint32_t max_scratchpads_hi = (xhci->caps->hcsparams2 >> 27) & 0x1f;
	uint32_t max_scratchpads = max_scratchpads_lo | (max_scratchpads_hi << 5);

	if (max_scratchpads > 0) {
		__assert(max_scratchpads < PAGE_SIZE / sizeof(uint64_t));

		void *scratchpad_array_phys = pmm_allocpage(PMM_SECTION_DEFAULT);
		__assert(scratchpad_array_phys != NULL);
		uint64_t *scratchpad_array_virt = MAKE_HHDM(scratchpad_array_phys);

		printf("xhci: setting up %u scratchpad buffers\n", max_scratchpads);

		for (uint32_t i = 0; i < max_scratchpads; i++) {
			void *scratchpad_phys = pmm_allocpage(PMM_SECTION_DEFAULT);
			__assert(scratchpad_phys != NULL);
			memset(MAKE_HHDM(scratchpad_phys), 0, PAGE_SIZE);
			scratchpad_array_virt[i] = (uint64_t)scratchpad_phys;
		}

		dcbaa_virt[0] = (uint64_t)scratchpad_array_phys;
	}

	xhci->dcbaa = dcbaa_virt;
	xhci->opregs->dcbaap = (uint64_t)dcbaa_phys;

	// Set up command rings.
	xhci->opregs->crcr = (uint64_t)xhci->command_ring.ring_phys | XHCI_CRCR_RCS;

	// Set up Event Ring Segment Table.
	void *erst_phys = pmm_allocpage(PMM_SECTION_DEFAULT);
	__assert(erst_phys != NULL);

	xhci_erst_entry_t *erst_virt = MAKE_HHDM(erst_phys);
	memset(erst_virt, 0, PAGE_SIZE);

	erst_virt[0].ring_segment = (uint64_t)xhci->event_ring.ring_phys;
	erst_virt[0].ring_segment_size = xhci->event_ring.size;

	// Set up the interrupter.
	volatile xhci_ir_t *ir = &xhci->rtregs->ir[0];
	ir->iman = XHCI_IMAN_IE;
	ir->erstsz = 1;
	ir->erstba = (uint64_t)erst_phys;
	ir->erdp = (uint64_t)xhci->event_ring.ring_phys | XHCI_ERDP_EHB;

	// Start the controller.
	ret = xhci_run(xhci);
	if (ret != 0) {
		printf("xhci: controller run timeout\n");
		return ret;
	}

	list_for_each (&xhci->ctrl.hubs, node) {
		usb_hub_t *hub = container_of(node, usb_hub_t, node);
		xhci_root_hub_t *rh = container_of(hub, xhci_root_hub_t, hub);

		for (uint8_t port = 0; port < hub->port_count; port++) {
			// USB 2 ports need to be manually reset to enable them.
			if (xhci->ports[rh->port_offset + port].ver_major == 2) {
				volatile xhci_port_regs_t *portregs = &xhci->portregs[rh->port_offset + port];
				portregs->portsc |= XHCI_PORTSC_PR;
			}
		}

		__assert(usb_hub_start(hub) == 0);
	}

	return 0;
}

static int xhci_ctrl_enumerate(usb_ctrl_t *ctrl) {
	xhci_ctrl_t *xhci = container_of(ctrl, xhci_ctrl_t, ctrl);
	semaphore_signal(&xhci->sem);
	return 0;
}

// Forward declaration.
static int xhci_ctrl_xfer(usb_ctrl_t *ctrl, usb_device_t *dev, usb_xfer_t *xfer);

static int xhci_ctrl_address_device(usb_ctrl_t *ctrl, usb_hub_t *hub, uint8_t port, usb_device_t **dev) {
	xhci_ctrl_t *xhci = container_of(ctrl, xhci_ctrl_t, ctrl);
	xhci_root_hub_t *rh = container_of(hub, xhci_root_hub_t, hub);

	xhci_device_t *xhci_dev = alloc(sizeof(xhci_device_t));
	__assert(xhci_dev != NULL);

	xhci_dev->device.hub = hub;
	xhci_dev->device.port_number = port;

	if (hub->device != NULL) {
		xhci_device_t *parent = container_of(hub->device, xhci_device_t, device);
		xhci_dev->device_tier = parent->device_tier + 1;
		xhci_dev->route_string = parent->route_string | (port << (parent->device_tier * 4));
	} else {
		xhci_dev->device_tier = 0;
		xhci_dev->route_string = 0;
	}

	uint32_t portsc = xhci->portregs[rh->port_offset + port - 1].portsc;
	uint32_t port_speed = (portsc >> 10) & 0xf;
	if (port_speed == XHCI_PORT_SPEED_LOW)
		xhci_dev->device.speed = USB_SPEED_LOW;
	else if (port_speed == XHCI_PORT_SPEED_FULL)
		xhci_dev->device.speed = USB_SPEED_FULL;
	else if (port_speed == XHCI_PORT_SPEED_HIGH)
		xhci_dev->device.speed = USB_SPEED_HIGH;
	else if (port_speed >= XHCI_PORT_SPEED_SS_1X1 && port_speed <= XHCI_PORT_SPEED_SS_2X2)
		xhci_dev->device.speed = USB_SPEED_SUPER;

	xhci_trb_t event_trb;
	xhci_trb_t associated_trb;

	{
		xhci_trb_t trb = {0};
		trb.dw3 = XHCI_TRB_DW3_TYPE(TRB_ENABLE_SLOT);
		xhci_ring_submit_and_wait(&xhci->command_ring, &trb, &event_trb, &associated_trb);
		__assert(((event_trb.dw2 >> 24) & 0xff) == TRB_SUCCESS);
	}

	void *device_ctx_phys = pmm_allocpage(PMM_SECTION_DEFAULT);
	__assert(device_ctx_phys != NULL);
	xhci_dev->device_ctx_phys = device_ctx_phys;
	xhci_dev->device_ctx = MAKE_HHDM(device_ctx_phys);
	memset(xhci_dev->device_ctx, 0, PAGE_SIZE);

	void *input_ctx_phys = pmm_allocpage(PMM_SECTION_DEFAULT);
	__assert(input_ctx_phys != NULL);
	xhci_dev->input_ctx_phys = input_ctx_phys;
	xhci_dev->input_ctx = MAKE_HHDM(input_ctx_phys);
	memset(xhci_dev->input_ctx, 0, PAGE_SIZE);

	xhci_dev->slot_id = (event_trb.dw3 >> 24) & 0xff;
	printf("xhci: allocated slot %u for device %s:%u\n", xhci_dev->slot_id, hub->name, port);

	// Set up the Device Context Base Address Array entry.
	xhci->dcbaa[xhci_dev->slot_id] = (uint64_t)device_ctx_phys;

	// Allocate the EP0 ring.
	__assert(xhci_alloc_ring(xhci, &xhci_dev->ep_rings[0], &xhci->dbs[xhci_dev->slot_id], 1));

	// Set up the input context and address the device.
	volatile xhci_input_ctx_t *input_ctx = xhci_get_input_ctrl_ctx(xhci, xhci_dev);
	volatile xhci_slot_ctx_t *slot_ctx = xhci_get_input_slot_ctx(xhci, xhci_dev);
	volatile xhci_ep_ctx_t *ep0_ctx = xhci_get_input_ep_ctx(xhci, xhci_dev, 0);

	// Enable slot and EP0 contexts.
	input_ctx->a |= (1 << 0) | (1 << 1);

	// Set up slot context.
	slot_ctx->dw0.route_string = xhci_dev->route_string;
	slot_ctx->dw0.speed = port_speed;
	slot_ctx->dw0.ctx_entries = 1;
	slot_ctx->dw1.root_hub_port_number = rh->port_offset + port;

	// Set up EP0 context.
	ep0_ctx->dw1.ep_type = XHCI_EP_TYPE_CTRL;
	ep0_ctx->dw1.cerr = 3;

	if (port_speed == XHCI_PORT_SPEED_LOW || port_speed == XHCI_PORT_SPEED_FULL)
		ep0_ctx->dw1.max_packet_size = 8;
	else if (port_speed == XHCI_PORT_SPEED_HIGH)
		ep0_ctx->dw1.max_packet_size = 64;
	else
		ep0_ctx->dw1.max_packet_size = 512;

	uint64_t ep0_ring_phys = (uint64_t)xhci_dev->ep_rings[0].ring_phys;
	ep0_ctx->dw2.dcs = xhci_dev->ep_rings[0].cycle;
	ep0_ctx->dw2.tr_dequeue_pointer_lo = (uint32_t)((ep0_ring_phys >> 4) & 0xffffffff);
	ep0_ctx->dw3.tr_dequeue_pointer_hi = (uint32_t)(ep0_ring_phys >> 32);

	// Now submit the Address Device command.
	{
		xhci_trb_t trb = {0};
		trb.parameters = (uint64_t)xhci_dev->input_ctx_phys;
		trb.dw3 = XHCI_TRB_DW3_TYPE(TRB_ADDRESS_DEVICE) | (xhci_dev->slot_id << 24);
		xhci_ring_submit_and_wait(&xhci->command_ring, &trb, &event_trb, &associated_trb);
		__assert(((event_trb.dw2 >> 24) & 0xff) == TRB_SUCCESS);
	}

	printf("xhci: addressed device on slot %u\n", xhci_dev->slot_id);

	// Update the input context and set the device address.
	xhci_update_input_context(xhci, xhci_dev);
	xhci_dev->device.address = slot_ctx->dw3.usb_device_address;

	// Figure out the max packet size for EP0 by reading the device descriptor.
	usb_device_desc_t dev_desc;

	usb_setup_t setup;
	setup.bmRequestType = USB_REQUEST_RECIP_DEVICE | USB_REQUEST_STANDARD | USB_REQUEST_DIR_TO_HOST;
	setup.bRequest = USB_REQUEST_GET_DESCRIPTOR;
	setup.wValue = (USB_DESCRIPTOR_TYPE_DEVICE << 8);
	setup.wIndex = 0;
	setup.wLength = 8;

	usb_xfer_t xfer = {0};
	xfer.dir = USB_TRANSFER_TO_HOST;
	xfer.type = USB_TRANSFER_CONTROL;
	xfer.setup = &setup;
	xfer.buffer = &dev_desc;
	xfer.length = 8;
	xfer.completion = NULL;

	int res = xhci_ctrl_xfer(ctrl, &xhci_dev->device, &xfer);
	__assert(res == 8);

	// Update EP0 context with correct max packet size.
	xhci_dev->device.max_packet_size0 = dev_desc.bMaxPacketSize0;
	ep0_ctx->dw1.max_packet_size = xhci_dev->device.max_packet_size0;

	input_ctx->a |= (1 << 1);

	// Submit an Evaluate Context command.
	{
		xhci_trb_t trb = {0};
		trb.parameters = (uint64_t)xhci_dev->input_ctx_phys;
		trb.dw3 = XHCI_TRB_DW3_TYPE(TRB_EVALUATE_CTX) | (xhci_dev->slot_id << 24);
		xhci_ring_submit_and_wait(&xhci->command_ring, &trb, &event_trb, &associated_trb);
		__assert(((event_trb.dw2 >> 24) & 0xff) == TRB_SUCCESS);
	}

	// Update the input context again.
	xhci_update_input_context(xhci, xhci_dev);

	// Return the device back to the caller.
	*dev = &xhci_dev->device;
	return 0;
}

static int xhci_ctrl_configure_ep(usb_ctrl_t *ctrl, usb_device_t *dev, usb_endpoint_t *ep) {
	xhci_ctrl_t *xhci = container_of(ctrl, xhci_ctrl_t, ctrl);
	xhci_device_t *xhci_dev = container_of(dev, xhci_device_t, device);

	bool is_in = (ep->desc.bEndpointAddress & USB_ENDPOINT_ADDRESS_DIR_IN) != 0;

	uint32_t ep_num = ep->desc.bEndpointAddress & USB_ENDPOINT_ADDRESS_NUM_MASK;
	uint32_t ep_index = ep_num ? ((ep_num << 1) | (is_in ? 1 : 0)) : 0;

	__assert(xhci_alloc_ring(xhci, &xhci_dev->ep_rings[ep_index - 1], NULL, 0));

	volatile xhci_input_ctx_t *input_ctx = xhci_get_input_ctrl_ctx(xhci, xhci_dev);
	volatile xhci_slot_ctx_t *slot_ctx = xhci_get_input_slot_ctx(xhci, xhci_dev);
	volatile xhci_ep_ctx_t *ep_ctx = xhci_get_input_ep_ctx(xhci, xhci_dev, ep_index - 1);

	memset((void *)ep_ctx, 0, sizeof(xhci_ep_ctx_t));

	input_ctx->a |= 1 << 0;
	input_ctx->a |= 1 << ep_index;

	slot_ctx->dw0.ctx_entries = 31;

	// Set up the endpoint context.
	uint8_t ep_type = ep->desc.bmAttributes & USB_ENDPOINT_ATTRIB_TYPE_MASK;

	if (ep_type == USB_ENDPOINT_ATTRIB_TYPE_BULK) {
		ep_ctx->dw1.ep_type = is_in ? XHCI_EP_TYPE_BULK_IN : XHCI_EP_TYPE_BULK_OUT;
	} else if (ep_type == USB_ENDPOINT_ATTRIB_TYPE_INTR) {
		ep_ctx->dw0.interval = ep->desc.bInterval - 1;
		ep_ctx->dw1.ep_type = is_in ? XHCI_EP_TYPE_INTR_IN : XHCI_EP_TYPE_INTR_OUT;
		ep_ctx->dw1.cerr = 3;
		ep_ctx->dw1.max_packet_size = ep->desc.wMaxPacketSize;
		ep_ctx->dw1.max_burst_size = (ep->desc.wMaxPacketSize >> 11) & 0x3;
		ep_ctx->dw4.max_esit_payload_lo = ep->desc.wMaxPacketSize;

		uint64_t ep_ring_phys = (uint64_t)xhci_dev->ep_rings[ep_index - 1].ring_phys;
		ep_ctx->dw2.dcs = xhci_dev->ep_rings[ep_index - 1].cycle;
		ep_ctx->dw2.tr_dequeue_pointer_lo = (uint32_t)((ep_ring_phys >> 4) & 0xffffffff);
		ep_ctx->dw3.tr_dequeue_pointer_hi = (uint32_t)(ep_ring_phys >> 32);
	} else {
		return -EINVAL;
	}

	// Submit a Configure Endpoint command.
	xhci_trb_t event_trb;
	xhci_trb_t associated_trb;
	xhci_trb_t trb = {0};
	trb.parameters = (uint64_t)xhci_dev->input_ctx_phys;
	trb.dw3 = XHCI_TRB_DW3_TYPE(TRB_CONFIGURE_EP) | (xhci_dev->slot_id << 24);
	xhci_ring_submit_and_wait(&xhci->command_ring, &trb, &event_trb, &associated_trb);
	__assert(((event_trb.dw2 >> 24) & 0xff) == TRB_SUCCESS);

	return 0;
}

static void xhci_control_xfer(xhci_ctrl_t *xhci, xhci_device_t *dev, usb_xfer_t *xfer, xhci_submission_t *sub, xhci_submission_t *data_sub) {
	usb_setup_t *setup = xfer->setup;

	// Make sure direction matches request type and length matches buffer length.
	__assert(xfer->dir == ((setup->bmRequestType >> 7) & 1));
	__assert(xfer->length == setup->wLength);

	bool has_data_stage = xfer->buffer != NULL && xfer->length > 0;
	bool data_stage_in = has_data_stage && xfer->dir == USB_TRANSFER_TO_HOST;

	uint32_t trt = 0;
	if (has_data_stage)
		// 0 = no data stage, 2 = data out, 3 = data in
		trt = data_stage_in ? 3 : 2;

	xhci_trb_t setup_trb = {0};
	setup_trb.dw0 = (uint32_t)setup->bmRequestType | ((uint32_t)setup->bRequest << 8) | ((uint32_t)setup->wValue << 16);
	setup_trb.dw1 = (uint32_t)setup->wIndex | ((uint32_t)setup->wLength << 16);
	setup_trb.dw2 = 8; // always 8 bytes for setup stage
	setup_trb.dw3 = XHCI_TRB_DW3_TYPE(TRB_SETUP_STAGE) | XHCI_TRB_DW3_TRT(trt) | XHCI_TRB_DW3_IDT;

	xhci_trb_t data_trb = {0};
	if (has_data_stage) {
		uint64_t page_offset = (uint64_t)xfer->buffer & (PAGE_SIZE - 1);
		__assert(page_offset + xfer->length <= PAGE_SIZE);

		void *page = vmm_getphysical(xfer->buffer - page_offset, true);
		__assert(page != NULL);

		data_trb.parameters = (uint64_t)page + page_offset;
		data_trb.dw2 = (uint32_t)xfer->length;
		data_trb.dw3 = XHCI_TRB_DW3_TYPE(TRB_DATA_STAGE) | XHCI_TRB_DW3_ISP | (data_stage_in ? XHCI_TRB_DW3_DIR : 0);

		if (data_sub != NULL)
			data_trb.dw3 |= XHCI_TRB_DW3_IOC;
	}

	xhci_trb_t status_trb = {0};
	status_trb.dw3 = XHCI_TRB_DW3_TYPE(TRB_STATUS_STAGE) | XHCI_TRB_DW3_IOC | (data_stage_in ? 0 : XHCI_TRB_DW3_DIR);

	xhci_ring_submit(&dev->ep_rings[0], &setup_trb, NULL, false);
	if (has_data_stage)
		xhci_ring_submit(&dev->ep_rings[0], &data_trb, data_sub, false);
	xhci_ring_submit(&dev->ep_rings[0], &status_trb, sub, false);

	// Ring the doorbell for the control endpoint.
	xhci->dbs[dev->slot_id] = 1;
}

static void xhci_data_xfer(xhci_ctrl_t *xhci, xhci_device_t *dev, usb_xfer_t *xfer, xhci_submission_t *sub) {
	xhci_trb_t trb = {0};
	uint64_t page_offset = (uint64_t)xfer->buffer & (PAGE_SIZE - 1);
	__assert(page_offset + xfer->length <= PAGE_SIZE);

	void *page = vmm_getphysical(xfer->buffer - page_offset, true);
	__assert(page != NULL);

	trb.parameters = (uint64_t)page + page_offset;
	trb.dw2 = (uint32_t)xfer->length;
	trb.dw3 = XHCI_TRB_DW3_TYPE(TRB_NORMAL) | XHCI_TRB_DW3_ISP | XHCI_TRB_DW3_IOC;

	uint32_t ep_num = xfer->ep->desc.bEndpointAddress & USB_ENDPOINT_ADDRESS_NUM_MASK;
	uint32_t ep_index = (ep_num << 1) | (xfer->ep->desc.bEndpointAddress & USB_ENDPOINT_ADDRESS_DIR_IN ? 1 : 0);

	xhci_ring_submit(&dev->ep_rings[ep_index - 1], &trb, sub, false);

	// Ring the doorbell for the endpoint.
	xhci->dbs[dev->slot_id] = ep_index;
}

typedef struct {
	usb_xfer_t *xfer;
	semaphore_t *sem;

	xhci_trb_t event_trb;
	usb_status_t status;
	uint32_t xfer_len;
} xhci_xfer_ctx_t;

static void xhci_xfer_data_complete(xhci_submission_t *sub) {
	pmm_release((void *)(sub->associated_trb.parameters & ~(uint64_t)(PAGE_SIZE - 1)));
}

static void xhci_xfer_complete(xhci_submission_t *sub) {
	xhci_xfer_ctx_t *ctx = sub->completion_ctx;

	memcpy(&ctx->event_trb, &sub->event_trb, sizeof(xhci_trb_t));
	uint32_t status = (ctx->event_trb.dw2 >> 24) & 0xff;

	if (status == TRB_SUCCESS) {
		ctx->status = USB_STATUS_SUCCESS;
		ctx->xfer_len = ctx->xfer->length;
	} else if (status == TRB_SHORT_PACKET) {
		uint32_t residual = ctx->event_trb.dw2 & 0xffffff;
		ctx->status = USB_STATUS_SUCCESS;
		ctx->xfer_len = ctx->xfer->length - residual;
	} else {
		ctx->status = USB_STATUS_ERROR;
		ctx->xfer_len = 0;
	}

	if (ctx->sem != NULL) {
		semaphore_signal(ctx->sem);
	} else {
		__assert(ctx->xfer->completion != NULL);
		ctx->xfer->completion(ctx->xfer, ctx->status, ctx->xfer_len);
	}

	// For bulk and interrupt transfers, this submission is for the data stage.
	if (ctx->xfer->type == USB_TRANSFER_BULK || ctx->xfer->type == USB_TRANSFER_INTERRUPT)
		xhci_xfer_data_complete(sub);
}

static int xhci_ctrl_xfer(usb_ctrl_t *ctrl, usb_device_t *dev, usb_xfer_t *xfer) {
	xhci_ctrl_t *xhci = container_of(ctrl, xhci_ctrl_t, ctrl);
	xhci_device_t *xhci_dev = container_of(dev, xhci_device_t, device);

	semaphore_t sem;
	SEMAPHORE_INIT(&sem, 0);

	xhci_xfer_ctx_t *ctx = alloc(sizeof(xhci_xfer_ctx_t));
	__assert(ctx != NULL);
	ctx->xfer = xfer;

	if (xfer->completion == NULL)
		ctx->sem = &sem;
	else
		ctx->sem = NULL;

	xhci_submission_t *sub = alloc(sizeof(xhci_submission_t));
	__assert(sub != NULL);
	sub->completion = xhci_xfer_complete;
	sub->completion_ctx = ctx;

	xhci_submission_t *data_sub = alloc(sizeof(xhci_submission_t));
	__assert(data_sub != NULL);
	data_sub->completion = xhci_xfer_data_complete;
	data_sub->completion_ctx = NULL;

	if (xfer->type == USB_TRANSFER_CONTROL) {
		__assert(xfer->ep == NULL);
		__assert(xfer->setup != NULL);

		xhci_control_xfer(xhci, xhci_dev, xfer, sub, data_sub);
	} else {
		__assert(xfer->ep != NULL);
		__assert(xfer->setup == NULL);

		xhci_data_xfer(xhci, xhci_dev, xfer, sub);
	}

	if (xfer->completion == NULL) {
		semaphore_wait(&sem, false);
		return (ctx->status == USB_STATUS_SUCCESS) ? ctx->xfer_len : -EIO;
	}

	return 0;
}

static usb_ctrl_ops_t xhci_ops = {
	.start = xhci_ctrl_start,
	.enumerate = xhci_ctrl_enumerate,
	.address_device = xhci_ctrl_address_device,
	.configure_ep = xhci_ctrl_configure_ep,
	.xfer = xhci_ctrl_xfer,
};

static void xhci_thread(void) {
	xhci_ctrl_t *xhci = current_thread()->kernelarg;

	for (;;) {
		semaphore_wait(&xhci->sem, false);

		list_for_each(&xhci->ctrl.hubs, node) {
			usb_hub_t *hub = container_of(node, usb_hub_t, node);
			__assert(usb_hub_enumerate(hub) == 0);
		}
	}
}

static void xhci_event_thread(void) {
	xhci_ctrl_t *xhci = current_thread()->kernelarg;

	for (;;) {
		semaphore_wait(&xhci->ev_sem, false);
		xhci_handle_events(xhci);
	}
}

static void init_ctrl(pcienum_t *e) {
	printf("xhci: found controller at %02x:%02x.%x\n", e->bus, e->device, e->function);

	pcibar_t bar0 = pci_getbar(e, 0);

	volatile xhci_caps_t *caps = (xhci_caps_t *)bar0.address;
	volatile xhci_opregs_t *opregs = (xhci_opregs_t *)(bar0.address + caps->caplength);
	volatile xhci_rtregs_t *rtregs = (xhci_rtregs_t *)(bar0.address + caps->rtsoff);

	// Make sure the controller supports 4K pages
	if ((opregs->pagesize & (1 << 0)) == 0) {
		printf("xhci: controller does not support 4K page size\n");
		return;
	}

	// Register the controller with the USB subsystem
	xhci_ctrl_t *ctrl = alloc(sizeof(xhci_ctrl_t));
	__assert(ctrl != NULL);

	ctrl->ctrl.ops = &xhci_ops;

	list_init(&ctrl->ctrl.hubs);
	list_init(&ctrl->submissions);

	SEMAPHORE_INIT(&ctrl->sem, 0);
	SEMAPHORE_INIT(&ctrl->ev_sem, 0);

	SPINLOCK_INIT(ctrl->lock);

	ctrl->pci_enum = e;
	ctrl->caps = caps;
	ctrl->opregs = opregs;
	ctrl->rtregs = rtregs;
	ctrl->portregs = (xhci_port_regs_t *)(bar0.address + caps->caplength + 0x400);
	ctrl->dbs = (uint32_t *)(bar0.address + caps->dboff);

	struct thread_t *thread = sched_newthread(xhci_thread, PAGE_SIZE * 4, 0, NULL, NULL);
	thread->kernelarg = ctrl;
	__assert(thread != NULL);
	sched_queue(thread);

	struct thread_t *evthread = sched_newthread(xhci_event_thread, PAGE_SIZE * 4, 0, NULL, NULL);
	evthread->kernelarg = ctrl;
	__assert(evthread != NULL);
	sched_queue(evthread);

	int res = xhci_ctrl_start(&ctrl->ctrl);
	__assert(res == 0);
}

void xhci_init(void) {
	for (int i = 0;; ++i) {
		pcienum_t *e = pci_getenum(PCI_CLASS_SERIAL_BUS, PCI_SUBCLASS_SERIAL_BUS_USB, PCI_PROGIF_USB_XHCI, -1, -1, -1, i);
		if (e == NULL)
			break;
		init_ctrl(e);
	}
}

INIT_ROUTINE_DEFINE(xhci, INIT_ROUTINE_FLAGS_NONE, xhci_init, scheduler);
