#ifndef _XHCI_H
#define _XHCI_H

#include <stddef.h>
#include <stdint.h>

typedef struct {
	union {
		uint64_t parameters;

		struct {
			uint32_t dw0;
			uint32_t dw1;
		};
	};

	uint32_t dw2;
	uint32_t dw3;
} xhci_trb_t;

typedef struct {
	uint8_t caplength;
	uint8_t reserved1;
	uint16_t hciversion;
	uint32_t hcsparams1;
	uint32_t hcsparams2;
	uint32_t hcsparams3;
	uint32_t hccparams1;
	uint32_t dboff;
	uint32_t rtsoff;
	uint32_t hccparams2;
} xhci_caps_t;

typedef struct {
	uint32_t portsc;
	uint32_t portpmsc;
	uint32_t portli;
	uint32_t porthlpmc;
} xhci_port_regs_t;

typedef struct {
	uint32_t usbcmd;
	uint32_t usbsts;
	uint32_t pagesize;
	uint32_t reserved1;
	uint32_t reserved2;
	uint32_t dnctrl;
	uint64_t crcr;
	uint32_t reserved3[4];
	uint64_t dcbaap;
	uint32_t config;
} xhci_opregs_t;

typedef struct {
	uint32_t iman;
	uint32_t imod;
	uint32_t erstsz;
	uint32_t reserved;
	uint64_t erstba;
	uint64_t erdp;
} xhci_ir_t;

typedef struct {
	uint32_t mfindex;
	uint32_t reserved[7];
	xhci_ir_t ir[1024];
} xhci_rtregs_t;

typedef struct {
	uint64_t ring_segment;
	uint16_t ring_segment_size;
	uint16_t reserved1;
	uint32_t reserved2;
} xhci_erst_entry_t;

typedef struct {
	struct {
		uint32_t route_string : 20;
		uint8_t speed : 4;
		uint8_t reserved1 : 1;
		uint8_t mtt : 1;
		uint8_t hub : 1;
		uint8_t ctx_entries : 5;
	} dw0;

	struct {
		uint16_t max_exit_latency;
		uint8_t root_hub_port_number;
		uint8_t number_of_ports;
	} dw1;

	struct {
		uint8_t tt_hub_slot_id;
		uint8_t tt_port_number;
		uint16_t ttt : 2;
		uint16_t reserved1 : 4;
		uint16_t interrupter_target : 10;
	} dw2;

	struct {
		uint8_t usb_device_address;
		uint8_t reserved1[2];
		uint8_t reserved2 : 3;
		uint8_t slot_state : 5;
	} dw3;

	uint32_t reserved[4];
} xhci_slot_ctx_t;

typedef struct {
	struct {
		uint8_t ep_state : 3;
		uint8_t reserved1 : 5;
		uint8_t mult : 2;
		uint8_t max_pstreams : 5;
		uint8_t lsa : 1;
		uint8_t interval;
		uint8_t max_esit_payload_hi;
	} dw0;

	struct {
		uint8_t reserved1 : 1;
		uint8_t cerr : 2;
		uint8_t ep_type : 3;
		uint8_t reserved2 : 1;
		uint8_t hid : 1;
		uint8_t max_burst_size;
		uint16_t max_packet_size;
	} dw1;

	struct {
		uint32_t dcs : 1;
		uint32_t reserved1 : 3;
		uint32_t tr_dequeue_pointer_lo : 28;
	} dw2;

	struct {
		uint32_t tr_dequeue_pointer_hi;
	} dw3;

	struct {
		uint16_t average_trb_length;
		uint16_t max_esit_payload_lo;
	} dw4;

	uint32_t reserved[3];
} xhci_ep_ctx_t;

typedef struct {
	uint32_t d;
	uint32_t a;
	uint32_t reserved[5];

	struct {
		uint8_t configuration_value;
		uint8_t interface_number;
		uint8_t alternate_setting;
		uint8_t reserved1;
	} dw7;
} xhci_input_ctx_t;

#define XHCI_HCCPARAMS1_CSZ (1 << 2)

#define XHCI_USBCMD_RS (1 << 0)
#define XHCI_USBCMD_HCRST (1 << 1)
#define XHCI_USBCMD_INTE (1 << 2)

#define XHCI_USBSTS_HCH (1 << 0)
#define XHCI_USBSTS_CNR (1 << 11)
#define XHCI_USBSTS_HCE (1 << 12)

#define XHCI_IMAN_IP (1 << 0)
#define XHCI_IMAN_IE (1 << 1)

#define XHCI_CRCR_RCS (1 << 0)

#define XHCI_ERDP_EHB (1 << 3)

#define XHCI_PORTSC_CCS (1 << 0)
#define XHCI_PORTSC_PED (1 << 1)
#define XHCI_PORTSC_OCA (1 << 3)
#define XHCI_PORTSC_PR (1 << 4)
#define XHCI_PORTSC_PP (1 << 9)
#define XHCI_PORTSC_CSC (1 << 17)
#define XHCI_PORTSC_PEC (1 << 18)
#define XHCI_PORTSC_OCC (1 << 20)
#define XHCI_PORTSC_PRC (1 << 21)

#define XHCI_TRB_DW3_C (1 << 0)
#define XHCI_TRB_DW3_TC (1 << 1)
#define XHCI_TRB_DW3_ISP (1 << 2)
#define XHCI_TRB_DW3_CH (1 << 4)
#define XHCI_TRB_DW3_IOC (1 << 5)
#define XHCI_TRB_DW3_IDT (1 << 6)
#define XHCI_TRB_DW3_DIR (1 << 16)

#define XHCI_TRB_DW3_TYPE(TYPE) (((TYPE) & 0x3f) << 10)
#define XHCI_TRB_DW3_TRT(TRT) (((TRT) & 0x3) << 16)

typedef enum {
	TRB_NORMAL = 1,
	TRB_SETUP_STAGE = 2,
	TRB_DATA_STAGE = 3,
	TRB_STATUS_STAGE = 4,
	TRB_LINK = 6,
	TRB_ENABLE_SLOT = 9,
	TRB_DISABLE_SLOT = 10,
	TRB_ADDRESS_DEVICE = 11,
	TRB_CONFIGURE_EP = 12,
	TRB_EVALUATE_CTX = 13,
	TRB_RESET_EP = 14,
	TRB_STOP_EP = 15,
	TRB_XFER_COMPLETION_EVENT = 32,
	TRB_COMMAND_COMPLETION_EVENT = 33,
	TRB_PORT_STATUS_CHANGE_EVENT = 34,
} xhci_trb_type_t;

typedef enum {
	TRB_SUCCESS = 1,
	TRB_SHORT_PACKET = 13,
} xhci_trb_status_t;

typedef enum {
	XHCI_PORT_SPEED_FULL = 1,
	XHCI_PORT_SPEED_LOW = 2,
	XHCI_PORT_SPEED_HIGH = 3,
	XHCI_PORT_SPEED_SS_1X1 = 4,
	XHCI_PORT_SPEED_SS_2X1 = 5,
	XHCI_PORT_SPEED_SS_1X2 = 6,
	XHCI_PORT_SPEED_SS_2X2 = 7,
} xhci_port_speed_t;

typedef enum {
	XHCI_EP_TYPE_ISOCH_OUT = 1,
	XHCI_EP_TYPE_BULK_OUT = 2,
	XHCI_EP_TYPE_INTR_OUT = 3,
	XHCI_EP_TYPE_CTRL = 4,
	XHCI_EP_TYPE_ISOCH_IN = 5,
	XHCI_EP_TYPE_BULK_IN = 6,
	XHCI_EP_TYPE_INTR_IN = 7,
} xhci_ep_type_t;

#endif
