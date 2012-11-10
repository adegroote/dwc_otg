/*	$NetBSD: $ */


struct dwc_otg_flags {
	uint8_t	change_connect:1;
	uint8_t	change_suspend:1;
	uint8_t change_reset:1;
	uint8_t change_enabled:1;
	uint8_t change_over_current:1;
	uint8_t	status_suspend:1;	/* set if suspended */
	uint8_t	status_vbus:1;		/* set if present */
	uint8_t	status_bus_reset:1;	/* set if reset complete */
	uint8_t	status_high_speed:1;	/* set if High Speed is selected */
	uint8_t	status_low_speed:1;	/* set if Low Speed is selected */
	uint8_t status_device_mode:1;	/* set if device mode */
	uint8_t	self_powered:1;
	uint8_t	clocks_off:1;
	uint8_t	port_powered:1;
	uint8_t	port_enabled:1;
	uint8_t port_over_current:1;
	uint8_t	d_pulled_up:1;
};

struct dwc_otg_td;

typedef uint8_t (dwc_otg_cmd_t)(struct dwc_otg_td *td);

typedef struct dwc_otg_td {
	struct dwc_otg_td *obj_next;
	dwc_otg_cmd_t *func;
#if 0
	struct usb_page_cache *pc;
#else
	usbd_xfer_handle xfer;
	usb_dma_t *dmap;
#endif

	uint32_t tx_bytes;
	uint32_t offset;
	uint32_t remainder;
	uint32_t hcchar;		/* HOST CFG */
	uint32_t hcsplt;		/* HOST CFG */
	uint16_t max_packet_size;	/* packet_size */
	uint16_t npkt;
	uint8_t errcnt;
	uint8_t tmr_res;
	uint8_t tmr_val;
	uint8_t	ep_no;
	uint8_t channel;
	uint8_t state;
#define	DWC_CHAN_ST_START 0
#define	DWC_CHAN_ST_WAIT_ANE 1
#define	DWC_CHAN_ST_WAIT_S_ANE 2
#define	DWC_CHAN_ST_WAIT_C_ANE 3
#define	DWC_CHAN_ST_RX_PKT 4
#define	DWC_CHAN_ST_RX_SPKT 5
#define	DWC_CHAN_ST_TX_PKT 4
#define	DWC_CHAN_ST_TX_CPKT 5
	uint8_t	error:1;
	uint8_t	error_any:1;
	uint8_t	error_stall:1;
	uint8_t	alt_next:1;
	uint8_t	short_pkt:1;
	uint8_t	did_stall:1;
	uint8_t toggle:1;
	uint8_t set_toggle:1;
	uint8_t got_short:1;
	uint8_t did_nak:1;
} dwc_otg_td_t;


#define	DWC_OTG_MAX_DEVICES MIN(USB_MAX_DEVICES, 32)
#define	DWC_OTG_FRAME_MASK 0x7FF
#define	DWC_OTG_MAX_TXP 4
#define	DWC_OTG_MAX_TXN (0x200 * DWC_OTG_MAX_TXP)
#define	DWC_OTG_MAX_CHANNELS 16
#define	DWC_OTG_MAX_ENDPOINTS 16
#define	DWC_OTG_HOST_TIMER_RATE 10 /* ms */

typedef struct dwc_otg_soft_ed {
} dwc_otg_soft_ed_t;

typedef struct dwc_otg_soft_td {
	dwc_otg_td_t *td;
	usb_dma_t dma;
	int offs;
} dwc_otg_soft_td_t;

typedef struct dwc_otg_softc {
	device_t sc_dev;
	struct usbd_bus sc_bus;
	bus_space_tag_t sc_iot;
	bus_space_handle_t sc_ioh;
	bus_size_t sc_size;

	kmutex_t sc_lock;
	kmutex_t sc_intr_lock;

	int sc_noport;

	usbd_xfer_handle sc_intrxfer;

	device_t sc_child;	/* /dev/usb# device */
	char sc_dying;
	struct usb_dma_reserve sc_dma_reserve;

	char sc_vendor[32];		/* vendor string for root hub */
	int sc_id_vendor;		/* vendor ID for root hub */

	/* From FreeBSD softc */
	uint32_t sc_rx_bounce_buffer[1024 / 4];
	uint32_t sc_tx_bounce_buffer[(512 * DWC_OTG_MAX_TXP) / 4];

	uint32_t sc_fifo_size;
	uint32_t sc_irq_mask;
	uint32_t sc_hprt_val;

	uint8_t sc_timer_active;
	uint8_t	sc_dev_ep_max;
	uint8_t sc_dev_in_ep_max;
	uint8_t	sc_host_ch_max;

	uint8_t sc_addr;		/* device address */
	uint8_t sc_conf;		/* device configuration */
	uint8_t sc_mode;		/* mode of operation */
#define DWC_MODE_OTG 0
#define DWC_MODE_DEVICE 1
#define DWC_MODE_HOST 2

	uint16_t sc_active_rx_ep;

	struct dwc_otg_flags sc_flags;

} dwc_otg_softc_t;

struct dwc_otg_xfer {
	struct usbd_xfer xfer;
	struct usb_task	abort_task;
};

usbd_status	dwc_otg_init(dwc_otg_softc_t *);
int		dwc_otg_intr(void *);
int		dwc_otg_detach(dwc_otg_softc_t *, int);
bool		dwc_otg_shutdown(device_t, int);
void		dwc_otg_childdet(device_t, device_t);
int		dwc_otg_activate(device_t, enum devact);
bool		dwc_otg_resume(device_t, const pmf_qual_t *);
bool		dwc_otg_suspend(device_t, const pmf_qual_t *);
usbd_status dwc_otg_device_request(usbd_xfer_handle);

