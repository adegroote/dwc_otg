
/*
 * Designware USB 2.0 OTG
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD: $");
#if 0
__FBSDID("$FreeBSD: src/sys/dev/usb/controller/dwc_otg.c,v 1.19 2012/09/28 15:24:14 hselasky Exp $");
#endif

#include "opt_usb.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kmem.h>
#include <sys/kernel.h>
#include <sys/device.h>
#include <sys/select.h>
#include <sys/proc.h>
#include <sys/queue.h>
#include <sys/cpu.h>

#include <machine/endian.h>

#include <dev/usb/usb.h>
#include <dev/usb/usbdi.h>
#include <dev/usb/usbdivar.h>
#include <dev/usb/usb_mem.h>
// #include <dev/usb/usb_quirks.h>

#include <dev/usb/dwc_otgreg.h>
#include <dev/usb/dwc_otgvar.h>
#include <dev/usb/usbroothub_subr.h>


#ifdef DWC_OTG_DEBUG
#define DPRINTF(x)	if (dwc_otgdebug) printf x
#define DPRINTFN(n,x)	if (dwc_otgdebug>(n)) printf x
int dwc_otgdebug = 0;
#else
#define DPRINTF(x)
#define DPRINTFN(n,x)
#endif

struct dwc_otg_pipe;

Static usbd_status	dwc_otg_open(usbd_pipe_handle);
Static void		dwc_otg_poll(struct usbd_bus *);
Static void		dwc_otg_softintr(void *);
Static int		dwc_otg_intr1(dwc_otg_softc_t *);
Static void		dwc_otg_waitintr(dwc_otg_softc_t *, usbd_xfer_handle);

Static usbd_status	dwc_otg_allocm(struct usbd_bus *, usb_dma_t *, u_int32_t);
Static void		dwc_otg_freem(struct usbd_bus *, usb_dma_t *);

Static usbd_xfer_handle	dwc_otg_allocx(struct usbd_bus *);
Static void		dwc_otg_freex(struct usbd_bus *, usbd_xfer_handle);
Static void		dwc_otg_get_lock(struct usbd_bus *, kmutex_t **);

Static usbd_status	dwc_otg_setup_isoc(usbd_pipe_handle pipe);
Static void		dwc_otg_device_isoc_enter(usbd_xfer_handle);

Static usbd_status	dwc_otg_root_ctrl_transfer(usbd_xfer_handle);
Static usbd_status	dwc_otg_root_ctrl_start(usbd_xfer_handle);
Static void		dwc_otg_root_ctrl_abort(usbd_xfer_handle);
Static void		dwc_otg_root_ctrl_close(usbd_pipe_handle);
Static void		dwc_otg_root_ctrl_done(usbd_xfer_handle);

Static usbd_status	dwc_otg_root_intr_transfer(usbd_xfer_handle);
Static usbd_status	dwc_otg_root_intr_start(usbd_xfer_handle);
Static void		dwc_otg_root_intr_abort(usbd_xfer_handle);
Static void		dwc_otg_root_intr_close(usbd_pipe_handle);
Static void		dwc_otg_root_intr_done(usbd_xfer_handle);

Static usbd_status	dwc_otg_device_ctrl_transfer(usbd_xfer_handle);
Static usbd_status	dwc_otg_device_ctrl_start(usbd_xfer_handle);
Static void		dwc_otg_device_ctrl_abort(usbd_xfer_handle);
Static void		dwc_otg_device_ctrl_close(usbd_pipe_handle);
Static void		dwc_otg_device_ctrl_done(usbd_xfer_handle);

Static usbd_status	dwc_otg_device_bulk_transfer(usbd_xfer_handle);
Static usbd_status	dwc_otg_device_bulk_start(usbd_xfer_handle);
Static void		dwc_otg_device_bulk_abort(usbd_xfer_handle);
Static void		dwc_otg_device_bulk_close(usbd_pipe_handle);
Static void		dwc_otg_device_bulk_done(usbd_xfer_handle);

Static usbd_status	dwc_otg_device_intr_transfer(usbd_xfer_handle);
Static usbd_status	dwc_otg_device_intr_start(usbd_xfer_handle);
Static void		dwc_otg_device_intr_abort(usbd_xfer_handle);
Static void		dwc_otg_device_intr_close(usbd_pipe_handle);
Static void		dwc_otg_device_intr_done(usbd_xfer_handle);

Static usbd_status	dwc_otg_device_isoc_transfer(usbd_xfer_handle);
Static usbd_status	dwc_otg_device_isoc_start(usbd_xfer_handle);
Static void		dwc_otg_device_isoc_abort(usbd_xfer_handle);
Static void		dwc_otg_device_isoc_close(usbd_pipe_handle);
Static void		dwc_otg_device_isoc_done(usbd_xfer_handle);

Static void		dwc_otg_close_pipe(usbd_pipe_handle, dwc_otg_soft_ed_t *);
Static void		dwc_otg_abort_xfer(usbd_xfer_handle, usbd_status);

Static void		dwc_otg_device_clear_toggle(usbd_pipe_handle pipe);
Static void		dwc_otg_noop(usbd_pipe_handle pipe);

#ifdef DWC_OTG_DEBUG
Static void		dwc_otg_dump_global_regs(dwc_otg_softc_t *);
Static void		dwc_otg_dump_host_regs(dwc_otg_softc_t *);
#endif

Static void		dwc_otg_timeout(void *);
Static void		dwc_otg_timeout_task(void *);

Static void		dwc_otg_pull_up(struct dwc_otg_softc *sc);
Static void		dwc_otg_pull_down(struct dwc_otg_softc *sc);
Static void		dwc_otg_vbus_interrupt(struct dwc_otg_softc *sc);


static int dwc_otg_init_fifo(struct dwc_otg_softc *sc, uint8_t mode);


#define DWC_OTG_READ_4(sc, reg) \
  bus_space_read_4((sc)->sc_iot, (sc)->sc_ioh, reg)
#define DWC_OTG_WRITE_4(sc, reg, data)  \
  bus_space_write_4((sc)->sc_iot, (sc)->sc_ioh, reg, data)
#define offonbits(sc, reg, off, on) \
  DWC_OTG_WRITE_4((sc),(reg),DWC_OTG_READ_4((sc),(reg)) & ~(off) | (on))


struct dwc_otg_pipe {
	struct usbd_pipe pipe;

	dwc_otg_soft_ed_t *sed;
	union {
		struct {
		} ctl;
		struct {
		} intr;
		struct {
		} bulk;
		struct {
		} iso;
	} u;
};

Static const struct usbd_bus_methods dwc_otg_bus_methods = {
	.open_pipe =	dwc_otg_open,
	.soft_intr =	dwc_otg_softintr,
	.do_poll =	dwc_otg_poll,
	.allocm =	dwc_otg_allocm,
	.freem =	dwc_otg_freem,
	.allocx =	dwc_otg_allocx,
	.freex =	dwc_otg_freex,
	.get_lock =	dwc_otg_get_lock,
};

Static const struct usbd_pipe_methods dwc_otg_root_ctrl_methods = {
	.transfer =	dwc_otg_root_ctrl_transfer,
	.start =	dwc_otg_root_ctrl_start,
	.abort =	dwc_otg_root_ctrl_abort,
	.close =	dwc_otg_root_ctrl_close,
	.cleartoggle =	dwc_otg_noop,
	.done =		dwc_otg_root_ctrl_done,
};

Static const struct usbd_pipe_methods dwc_otg_root_intr_methods = {
	.transfer =	dwc_otg_root_intr_transfer,
	.start =	dwc_otg_root_intr_start,
	.abort =	dwc_otg_root_intr_abort,
	.close =	dwc_otg_root_intr_close,
	.cleartoggle =	dwc_otg_noop,
	.done =		dwc_otg_root_intr_done,
};

Static const struct usbd_pipe_methods dwc_otg_device_ctrl_methods = {
	.transfer =	dwc_otg_device_ctrl_transfer,
	.start =	dwc_otg_device_ctrl_start,
	.abort =	dwc_otg_device_ctrl_abort,
	.close =	dwc_otg_device_ctrl_close,
	.cleartoggle =	dwc_otg_noop,
	.done =		dwc_otg_device_ctrl_done,
};

Static const struct usbd_pipe_methods dwc_otg_device_intr_methods = {
	.transfer =	dwc_otg_device_intr_transfer,
	.start =	dwc_otg_device_intr_start,
	.abort =	dwc_otg_device_intr_abort,
	.close =	dwc_otg_device_intr_close,
	.cleartoggle =	dwc_otg_device_clear_toggle,
	.done =		dwc_otg_device_intr_done,
};

Static const struct usbd_pipe_methods dwc_otg_device_bulk_methods = {
	.transfer =	dwc_otg_device_bulk_transfer,
	.start =	dwc_otg_device_bulk_start,
	.abort =	dwc_otg_device_bulk_abort,
	.close =	dwc_otg_device_bulk_close,
	.cleartoggle =	dwc_otg_device_clear_toggle,
	.done =		dwc_otg_device_bulk_done,
};

Static const struct usbd_pipe_methods dwc_otg_device_isoc_methods = {
	.transfer =	dwc_otg_device_isoc_transfer,
	.start =	dwc_otg_device_isoc_start,
	.abort =	dwc_otg_device_isoc_abort,
	.close =	dwc_otg_device_isoc_close,
	.cleartoggle =	dwc_otg_noop,
	.done =		dwc_otg_device_isoc_done,
};

Static usbd_status
dwc_otg_allocm(struct usbd_bus *bus, usb_dma_t *dma, u_int32_t size)
{
	struct dwc_otg_softc *sc = bus->hci_private;
	usbd_status status;

	status = usb_allocmem(&sc->sc_bus, size, 0, dma);
	if (status == USBD_NOMEM)
		status = usb_reserve_allocm(&sc->sc_dma_reserve, dma, size);
	return status;
}

Static void
dwc_otg_freem(struct usbd_bus *bus, usb_dma_t *dma)
{
        struct dwc_otg_softc *sc = bus->hci_private;
        if (dma->block->flags & USB_DMA_RESERVE) {
                usb_reserve_freem(&sc->sc_dma_reserve, dma);
                return;
        }
        usb_freemem(&sc->sc_bus, dma);
}

usbd_xfer_handle
dwc_otg_allocx(struct usbd_bus *bus)
{
	/* Unused for now, maybe add some kind of free list to avoid too much
	 * rellocation
	 struct dwc_otg_softc *sc = bus->hci_private;
	 */
	(void) bus;
	usbd_xfer_handle xfer;

	xfer = kmem_alloc(sizeof(struct dwc_otg_xfer), KM_SLEEP);
	if (xfer != NULL) {
		memset(xfer, 0, sizeof(struct dwc_otg_xfer));
#ifdef DIAGNOSTIC
		xfer->busy_free = XFER_BUSY;
#endif
	}
	return xfer;
}

void
dwc_otg_freex(struct usbd_bus *bus, usbd_xfer_handle xfer)
{
#ifdef DIAGNOASTIC
	if (xfer->busy_free != XFER_BUSY)
		printf("%s: xfer=%p not busy, 0x%08x\n", __func__, xfer,
		    xfer->busy_free);
	xfer->busy_free = XFER_FREE;
#endif
	kmem_free(xfer, sizeof(struct dwc_otg_xfer));
}

Static void
dwc_otg_get_lock(struct usbd_bus *bus, kmutex_t **lock)
{
	struct dwc_otg_softc *sc = bus->hci_private;

	*lock = &sc->sc_lock;
}


Static void
dwc_otg_softintr(void *v)
{
	struct usbd_bus *bus = v;
	dwc_otg_softc_t *sc = bus->hci_private;
}

Static void
dwc_otg_waitintr(dwc_otg_softc_t *sc, usbd_xfer_handle xfer)
{
}

Static void
dwc_otg_device_ctrl_done(usbd_xfer_handle xfer)
{
	struct dwc_otg_pipe *dpipe = (struct dwc_otg_pipe *)xfer->pipe;
}

Static void
dwc_otg_device_intr_done(usbd_xfer_handle xfer)
{
	struct dwc_otg_pipe *dpipe = (struct dwc_otg_pipe *)xfer->pipe;
	dwc_otg_softc_t *sc = dpipe->pipe.device->bus->hci_private;
}

Static void
dwc_otg_device_bulk_done(usbd_xfer_handle xfer)
{
	dwc_otg_softc_t *sc = xfer->pipe->device->bus->hci_private;
}

Static void
dwc_otg_root_intr_done(usbd_xfer_handle xfer)
{
}

Static void
dwc_otg_root_ctrl_done(usbd_xfer_handle xfer)
{
}

usbd_status
dwc_otg_device_request(usbd_xfer_handle xfer)
{
	struct dwc_otg_pipe *dpipe = (struct dwc_otg_pipe *)xfer->pipe;
	usb_device_request_t *req = &xfer->request;
	usbd_device_handle dev = dpipe->pipe.device;
	dwc_otg_softc_t *sc = dev->bus->hci_private;
}

Static void
dwc_otg_timeout(void *addr)
{
	struct dwc_otg_xfer *oxfer = addr;
	struct dwc_otg_pipe *dpipe = (struct dwc_otg_pipe *)oxfer->xfer.pipe;
	dwc_otg_softc_t *sc = dpipe->pipe.device->bus->hci_private;

	DPRINTF(("%s: oxfer=%p\n", __func__, oxfer));

	if (sc->sc_dying) {
		mutex_enter(&sc->sc_lock);
		dwc_otg_abort_xfer(&oxfer->xfer, USBD_TIMEOUT);
		mutex_exit(&sc->sc_lock);
		return;
	}

	/* Execute the abort in a process context. */
	usb_init_task(&oxfer->abort_task, dwc_otg_timeout_task, addr);
	usb_add_task(oxfer->xfer.pipe->device, &oxfer->abort_task,
	    USB_TASKQ_HC);
}

Static void
dwc_otg_timeout_task(void *addr)
{
	usbd_xfer_handle xfer = addr;
	dwc_otg_softc_t *sc = xfer->pipe->device->bus->hci_private;

	DPRINTF(("%s: xfer=%p\n", __func__, xfer));

	mutex_enter(&sc->sc_lock);
	dwc_otg_abort_xfer(xfer, USBD_TIMEOUT);
	mutex_exit(&sc->sc_lock);
}

usbd_status
dwc_otg_open(usbd_pipe_handle pipe)
{
	usbd_device_handle dev = pipe->device;
	dwc_otg_softc_t *sc = dev->bus->hci_private;
	usb_endpoint_descriptor_t *ed = pipe->endpoint->edesc;
	struct dwc_otg_pipe *dpipe = (struct dwc_otg_pipe *)pipe;
	uint8_t addr = dev->address;
	uint8_t xfertype = ed->bmAttributes & UE_XFERTYPE;
	dwc_otg_soft_ed_t *sed;
	dwc_otg_soft_td_t *std;
	usbd_status err = USBD_NOMEM;

	if (sc->sc_dying) {
		err = USBD_IOERROR;
		goto fail;
	}

	if (addr == sc->sc_addr) {
		switch (ed->bEndpointAddress) {
		case USB_CONTROL_ENDPOINT:
			pipe->methods = &dwc_otg_root_ctrl_methods;
			break;
		case UE_DIR_IN: // XXX to verify 
			pipe->methods = &dwc_otg_root_intr_methods;
			break;
		default:
			err = USBD_INVAL;
			goto fail;
		}
	} else {
#ifdef notyet
		sed = dwc_otg_alloc_sed(sc);
		if (sed == NULL)
			goto fail;
		dpipe->sed = sed;
#endif
		if (xfertype == UE_ISOCHRONOUS) {
			// XXX ...
		} else {
		}

		switch (xfertype) {
		case UE_CONTROL:
		case UE_INTERRUPT:
		case UE_ISOCHRONOUS:
		case UE_BULK:
			break;
		}
	}

	return USBD_NORMAL_COMPLETION;

fail:
	return err;
}

Static void
dwc_otg_poll(struct usbd_bus *bus)
{
	dwc_otg_softc_t *sc = bus->hci_private;

	mutex_spin_enter(&sc->sc_intr_lock);
	dwc_otg_intr1(sc);
	mutex_spin_exit(&sc->sc_intr_lock);
}

/*
 * Close a reqular pipe.
 * Assumes that there are no pending transactions.
 */
Static void
dwc_otg_close_pipe(usbd_pipe_handle pipe, dwc_otg_soft_ed_t *head)
{
	struct dwc_otg_pipe *dpipe = (struct dwc_otg_pipe *)pipe;
	dwc_otg_softc_t *sc = pipe->device->bus->hci_private;
	dwc_otg_soft_ed_t *sed = dpipe->sed;

	dwc_otg_rem_ed(sc, sed, head);
	dwc_otg_delayms(sc, 1);
	dwc_otg_free_sed(sc, dpipe->sed);
}

/*
 * Abort a device request.
 */
Static void
dwc_otg_abort_xfer(usbd_xfer_handle xfer, usbd_status status)
{
	struct dwc_otg_pipe *dpipe = (struct dwc_otg_pipe *)xfer->pipe;
	dwc_otg_softc_t *sc = dpipe->pipe.device->bus->hci_private;

	if (sc->sc_dying) {
		xfer->status = status;
		usb_transfer_complete(xfer);
		return;
	}

	if (xfer->hcflags & UXFER_ABORTING) {
		xfer->status = status;
		xfer->hcflags |= UXFER_ABORTWAIT;
		while (xfer->hcflags & UXFER_ABORTING)
			cv_wait(&xfer->hccv, &sc->sc_lock);
		return;
	}
	xfer->hcflags |= UXFER_ABORTING;

	DWC_OTG_WRITE_4(sc, DWC_OTG_HCINTMSK(ch), HCINTMSK_CHHLTDMSK);
	DWC_OTG_WRITE_4(sc, DWC_OTG_HCINT(ch), ~HCINTMSK_CHHLTDMSK);

	if ((DWC_OTG_READ_4(sc, DWC_OTG_HCCHAR(ch)) & HCCHAR_CHENA) == 0)
		return;

	offonbits(sc, DWC_OTG_HCCHAR(ch), HCCHAR_CHENA, HCCHAR_CHDIS);
}

Static void
dwc_otg_noop(usbd_pipe_handle pipe)
{
}

Static void
dwc_otg_device_clear_toggle(usbd_pipe_handle pipe)
{
}

/***********************************************************************/

/*
 * Data structures and routines to emulate the root hub.
 */
Static usb_device_descriptor_t dwc_otg_devd = {
	USB_DEVICE_DESCRIPTOR_SIZE,
	UDESC_DEVICE,		/* type */
	{0x00, 0x01},		/* USB version */
	UDCLASS_HUB,		/* class */
	UDSUBCLASS_HUB,		/* subclass */
	UDPROTO_FSHUB,		/* protocol */
	64,			/* max packet */
	{0},{0},{0x00,0x01},	/* device id */
	1,2,0,			/* string indicies */
	1			/* # of configurations */
};

Static const usb_config_descriptor_t dwc_otg_confd = {
	USB_CONFIG_DESCRIPTOR_SIZE,
	UDESC_CONFIG,
	{USB_CONFIG_DESCRIPTOR_SIZE +
	 USB_INTERFACE_DESCRIPTOR_SIZE +
	 USB_ENDPOINT_DESCRIPTOR_SIZE},
	1,
	1,
	0,
	UC_ATTR_MBO | UC_SELF_POWERED,
	0			/* max power */
};

Static const usb_interface_descriptor_t dwc_otg_ifcd = {
	USB_INTERFACE_DESCRIPTOR_SIZE,
	UDESC_INTERFACE,
	0,
	0,
	1,
	UICLASS_HUB,
	UISUBCLASS_HUB,
	UIPROTO_FSHUB,
	0
};

Static const usb_endpoint_descriptor_t dwc_otg_endpd = {
	.bLength = USB_ENDPOINT_DESCRIPTOR_SIZE,
	.bDescriptorType = UDESC_ENDPOINT,
	.bEndpointAddress = UE_DIR_IN | OHCI_INTR_ENDPT,
	.bmAttributes = UE_INTERRUPT,
	.wMaxPacketSize = {8, 0},			/* max packet */
	.bInterval = 255,
};


#define	HSETW(ptr, val) ptr = { (uint8_t)(val), (uint8_t)((val) >> 8) }
Static const usb_hub_descriptor_t dwc_otg_hubd = {
	.bDescLength = USB_HUB_DESCRIPTOR_SIZE,
	.bDescriptorType = UDESC_HUB,
	.bNbrPorts = 1,
	HSETW(.wHubCharacteristics, (UHD_PWR_NO_SWITCH | UHD_OC_INDIVIDUAL)),
	.bPwrOn2PwrGood = 50,
	.bHubContrCurrent = 0,
	.DeviceRemovable = {0},		/* port is removable */
};


Static usbd_status
dwc_otg_root_ctrl_transfer(usbd_xfer_handle xfer)
{
	dwc_otg_softc_t *sc = xfer->pipe->device->bus->hci_private;
	usb_status err;

	mutex_enter(&sc->sc_lock);
	err = usb_insert_transfer(xfer);
	mutex_exit(&sc->sc_lock);
	if (err)
		return err;

	return dwc_otg_root_ctrl_start(SIMPLEQ_FIRST(&xfer->pipe->queue));
}

Static usbd_status
dwc_otg_root_ctrl_start(usbd_xfer_handle xfer)
{
	dwc_otg_softc_t *sc = xfer->pipe->device->bus->hci_private;
	usb_device_request *req;
	uint8_t *buf;
	int port, i;
	int len, value, index, l, totlen;
	usb_port_status_t ps;
	usb_hub_descriptor_t hubd;
	usb_status_t err = USBD_IOERROR;
	uint32_t v;

	if (sc->sc_dying)
		return USBD_IOERROR;

	req = &xfer->request;
	len = UGETW(req->wLength);
	value = UGETW(req->wValue);
	index = UGETW(req->wIndex);

	buf = len ? KERNADDR(&xfer->dmabuf, 0) : NULL;

	totlen = 0;

#define C(x,y) ((x) | ((y) << 8))
	switch (C(req->bRequest, req->bmRequestType)) {
	case C(UR_CLEAR_FEATURE, UT_WRITE_DEVICE):
	case C(UR_CLEAR_FEATURE, UT_WRITE_INTERFACE):
	case C(UR_CLEAR_FEATURE, UT_WRITE_ENDPOINT):
		/*
		 * DEVICE_REMOTE_WAKEUP and ENDPOINT_HALT are no-ops
		 * for the integrated root hub.
		 */
		break;
	case C(UR_GET_CONFIG, UT_READ_DEVICE):
		if (len > 0) {
			*buf = sc->sc_conf;
			totlen = 1;
		}
		break;
	case C(UR_GET_DESCRIPTOR, UT_READ_DEVICE):
		if (len == 0)
			break;
		switch (value) {
		case C(0, UDESC_DEVICE):
			l = min(len, USB_DEVICE_DESCRIPTOR_SIZE);
			USETW(dwc_otg_devd.idVendor, sc->sc_id_vendor);
			memcpy(buf, &dwc_otg_devd, l);
			buf += l;
			len -= l;
			totlen += l;

			break;
		case C(0, UDESC_CONFIG):
			l = min(len, USB_CONFIG_DESCRIPTOR_SIZE);
			memcpy(buf, &dwc_otg_confd, l);
			buf += l;
			len -= l;
			totlen += l;

			l = min(len, USB_INTERFACE_DESCRIPTOR_SIZE);
			memcpy(buf, &dwc_otg_ifcd, l);
			buf += l;
			len -= l;
			totlen += l;

			l = min(len, USB_ENDPOINT_DESCRIPTOR_SIZE);
			memcpy(buf, &dwc_otg_endpd, l);
			buf += l;
			len -= l;
			totlen += l;

			break;
#define sd ((usb_string_descriptor_t *)buf)
		case C(0, UDESC_STRING):
			totlen = usb_makelangtbl(sd, len);
			break;
		case C(1, UDESC_STRING):
			totlen = usb_makestrdesc(sd, len, sc->sc_vendor);
			break;
		case C(2, UDESC_STRING):
			totlen = usb_makestrdesc(sd, len, "DWC OTG root hub");
			break;
#undef sd
		default:
			goto fail;
		}
		break;
	case C(UR_GET_INTERFACE, UT_READ_INTERFACE):
		if (len > 0) {
			*buf = 0;
			totlen = 1;
		}
                break;
	case C(UR_GET_STATUS, UT_READ_DEVICE):
		if (len > 1) {
			USETW(((usb_status_t *)buf)->wStatus,UDS_SELF_POWERED);
			totlen = 2;
		}
		break;
	case C(UR_GET_STATUS, UT_READ_INTERFACE):
	case C(UR_GET_STATUS, UT_READ_ENDPOINT):
		if (len > 1) {
			USETW(((usb_status_t *)buf)->wStatus, 0);
			totlen = 2;
		}
		break;
	case C(UR_SET_ADDRESS, UT_WRITE_DEVICE):
		if (value >= USB_MAX_DEVICES)
                        goto fail;

		sc->sc_addr = value;
		break;
	case C(UR_SET_CONFIG, UT_WRITE_DEVICE):
		if (value != 0 && value != 1)
                        goto fail;

		sc->sc_conf = value;
		break;
	case C(UR_SET_DESCRIPTOR, UT_WRITE_DEVICE):
		break;
	case C(UR_SET_FEATURE, UT_WRITE_DEVICE):
	case C(UR_SET_FEATURE, UT_WRITE_INTERFACE):
		err = USBD_IOERROR;
		goto fail;
	case C(UR_SET_FEATURE, UT_WRITE_ENDPOINT):
// 		switch (UGETW(req->wValue)) {
// 		case UF_ENDPOINT_HALT:
// 			goto tr_handle_clear_halt;
// 		case UF_DEVICE_REMOTE_WAKEUP:
// 			goto tr_handle_clear_wakeup;
// 		default:
// 			goto tr_stalled;
// 		}
// 		break;
		err = USBD_IOERROR;
		goto fail;
	case C(UR_SET_INTERFACE, UT_WRITE_INTERFACE):
		break;
	case C(UR_SYNCH_FRAME, UT_WRITE_ENDPOINT):
		break;

	/* Hub requests */
	case C(UR_CLEAR_FEATURE, UT_WRITE_CLASS_DEVICE):
		break;
	case C(UR_CLEAR_FEATURE, UT_WRITE_CLASS_OTHER):
		DPRINTFN(9, "UR_CLEAR_PORT_FEATURE on port %d\n", index);

		if (index < 1 || index > sc->sc_noport)
                        goto fail;

		switch (value) {
		case UHF_PORT_ENABLE:
			if (sc->sc_flags.status_device_mode == 0) {
				DWC_OTG_WRITE_4(sc, DWC_OTG_HPRT,
				    sc->sc_hprt_val | HPRT_PRTENA);
			}
			sc->sc_flags.port_enabled = 0;
			break;

		case UHF_PORT_SUSPEND:
			dwc_otg_wakeup_peer(sc);
			break;

		case UHF_PORT_POWER:
			sc->sc_flags.port_powered = 0;
			if (sc->sc_mode == DWC_MODE_HOST ||
			    sc->sc_mode == DWC_MODE_OTG) {
				sc->sc_hprt_val = 0;
				DWC_OTG_WRITE_4(sc, DWC_OTG_HPRT, HPRT_PRTENA);
			}
			dwc_otg_pull_down(sc);
			dwc_otg_clocks_off(sc);
			break;

		case UHF_C_PORT_CONNECTION:
			/* clear connect change flag */
			sc->sc_flags.change_connect = 0;
			break;
		case UHF_C_PORT_ENABLE:
			sc->sc_flags.change_enabled = 0;
			break;
		case UHF_C_PORT_SUSPEND:
			sc->sc_flags.change_suspend = 0;
			break;
		case UHF_C_PORT_OVER_CURRENT:
			sc->sc_flags.change_over_current = 0;
			break;
		case UHF_C_PORT_RESET:
			/* ??? *//* enable rhsc interrupt if condition is cleared */
			sc->sc_flags.change_reset = 0;
			break;
// 		case UHF_PORT_TEST:
// 		case UHF_PORT_INDICATOR:
// 			/* nops */
// 			break;
		default:
			goto fail;
		}
		break;
	case C(UR_GET_DESCRIPTOR, UT_READ_CLASS_DEVICE):
		if (len == 0)
			break;
		if ((value & 0xff) != 0)
			goto fail;

		hubd = dwc_otg_hubd;
		hubd.bNbrPorts = sc->sc_noport;

		l = min(len, hubd.bDescLength);
		memcpy(buf, &hubd, l);
		buf += l;
		len -= l;
		totlen += l;

		break;
	case C(UR_GET_STATUS, UT_READ_CLASS_DEVICE):
		if (len != 4)
			goto fail;
		memset(buf, 0, len); /* ? XXX */
		totlen = len;
		break;
	case C(UR_GET_STATUS, UT_READ_CLASS_OTHER):
		DPRINTFN(9, "UR_GET_PORT_STATUS\n");

		if (index < 1 || index > sc->sc_noport)
			goto fail;
		if (len != 4)
			goto fail;

		if (sc->sc_flags.status_vbus)
			dwc_otg_clocks_on(sc);
		else
			dwc_otg_clocks_off(sc);

		/* Select Device Side Mode */

		if (sc->sc_flags.status_device_mode) {
			value = UPS_PORT_MODE_DEVICE;
			dwc_otg_timer_stop(sc);
		} else {
			value = 0;
			dwc_otg_timer_start(sc);
		}

		if (sc->sc_flags.status_high_speed)
			value |= UPS_HIGH_SPEED;
		else if (sc->sc_flags.status_low_speed)
			value |= UPS_LOW_SPEED;

		if (sc->sc_flags.port_powered)
			value |= UPS_PORT_POWER;

		if (sc->sc_flags.port_enabled)
			value |= UPS_PORT_ENABLED;

		if (sc->sc_flags.port_over_current)
			value |= UPS_OVERCURRENT_INDICATOR;

		if (sc->sc_flags.status_vbus &&
		    sc->sc_flags.status_bus_reset)
			value |= UPS_CURRENT_CONNECT_STATUS;

		if (sc->sc_flags.status_suspend)
			value |= UPS_SUSPEND;

		USETW(ps.wPortStatus, value);

		value = 0;

		if (sc->sc_flags.change_connect)
			value |= UPS_C_CONNECT_STATUS;
		if (sc->sc_flags.change_suspend)
			value |= UPS_C_SUSPEND;
		if (sc->sc_flags.change_reset)
			value |= UPS_C_PORT_RESET;
		if (sc->sc_flags.change_over_current)
			value |= UPS_C_OVERCURRENT_INDICATOR;

		USETW(ps.wPortChange, value);

		l = min(len, sizeof(ps));
		memcpy(buf, &ps, l);
		buf += l;
		len -= l;
		totlen += l;

		break;
	case C(UR_SET_DESCRIPTOR, UT_WRITE_CLASS_DEVICE):
		goto fail;
	case C(UR_SET_FEATURE, UT_WRITE_CLASS_DEVICE):
		break;
	case C(UR_SET_FEATURE, UT_WRITE_CLASS_OTHER):
		if (index < 1 || index > sc->sc_noport)
			goto fail;

		DPRINTFN(9, "UR_SET_PORT_FEATURE\n");

		switch (value) {
		case UHF_PORT_ENABLE:
			break;

		case UHF_PORT_SUSPEND:
			if (sc->sc_flags.status_device_mode == 0) {
				/* set suspend BIT */
				sc->sc_hprt_val |= HPRT_PRTSUSP;
				DWC_OTG_WRITE_4(sc, DWC_OTG_HPRT, sc->sc_hprt_val);

				/* generate HUB suspend event */
				dwc_otg_suspend_irq(sc);
			}
			break;

		case UHF_PORT_RESET:
			if (sc->sc_flags.status_device_mode == 0) {

				DPRINTF("PORT RESET\n");

				/* enable PORT reset */
				DWC_OTG_WRITE_4(sc, DWC_OTG_HPRT,
				    sc->sc_hprt_val | HPRT_PRTRST);

				/* Wait 62.5ms for reset to complete */
				usb_delay_ms(&sc->sc_bus, hz / 16);

				DWC_OTG_WRITE_4(sc, DWC_OTG_HPRT, sc->sc_hprt_val);

				/* Wait 62.5ms for reset to complete */
				usb_delay_ms(&sc->sc_bus, hz / 16);

				/* reset FIFOs */
				dwc_otg_init_fifo(sc, DWC_MODE_HOST);

				sc->sc_flags.change_reset = 1;
			} else {
				err = USBD_IOERROR;
			}
			break;

// 		case UHF_PORT_TEST:
// 		case UHF_PORT_INDICATOR:
// 			/* nops */
// 			break;
		case UHF_PORT_POWER:
			if (sc->sc_mode == DWC_MODE_HOST ||
			    sc->sc_mode == DWC_MODE_OTG) {
				sc->sc_hprt_val |= HPRT_PRTPWR;
				DWC_OTG_WRITE_4(sc, DWC_OTG_HPRT, sc->sc_hprt_val);
			}
			sc->sc_flags.port_powered = 1;
			break;
		default:
			err = USBD_IOERROR;
			goto fail;
		}
		break;
	default:
		goto fail;
	}
	xfer->actlen = totlen;
	err = USBD_NORMAL_COMPLETION;

fail:
        xfer->status = err;

        mutex_enter(&sc->sc_lock);
        usb_transfer_complete(xfer);
        mutex_exit(&sc->sc_lock);

        return USBD_IN_PROGRESS;
}

Static void
dwc_otg_root_ctrl_abort(usbd_xfer_handle xfer)
{
	/* Nothing to do, all transfers are synchronous. */
}

Static void
dwc_otg_root_ctrl_close(usbd_pipe_handle pipe)
{
	/* Nothing to do. */
}

Static usbd_status
dwc_otg_root_intr_transfer(usbd_xfer_handle xfer)
{
	dwc_otg_softc_t *sc = xfer->pipe->device->bus->hci_private;
}

Static usbd_status
dwc_otg_root_intr_start(usbd_xfer_handle xfer)
{
	usbd_pipe_handle pipe = xfer->pipe;
	dwc_otg_softc_t *sc = pipe->device->bus->hci_private;
}

Static void
dwc_otg_root_intr_abort(usbd_xfer_handle xfer)
{
}

Static void
dwc_otg_root_intr_close(usbd_pipe_handle pipe)
{
	dwc_otg_softc_t *sc = pipe->device->bus->hci_private;
}

/***********************************************************************/

Static usbd_status
dwc_otg_device_ctrl_transfer(usbd_xfer_handle xfer)
{
	dwc_otg_softc_t *sc = xfer->pipe->device->bus->hci_private;
}

Static usbd_status
dwc_otg_device_ctrl_start(usbd_xfer_handle xfer)
{
	dwc_otg_softc_t *sc = xfer->pipe->device->bus->hci_private;
}

Static void
dwc_otg_device_ctrl_abort(usbd_xfer_handle xfer)
{
}

Static void
dwc_otg_device_ctrl_close(usbd_pipe_handle pipe)
{
	struct dwc_otg_pipe *dpipe = (struct dwc_otg_pipe *)pipe;
	dwc_otg_softc_t *sc = pipe->device->bus->hci_private;
}

/***********************************************************************/

Static usbd_status
dwc_otg_device_bulk_transfer(usbd_xfer_handle xfer)
{
	dwc_otg_softc_t *sc = xfer->pipe->device->bus->hci_private;
}

Static usbd_status
dwc_otg_device_bulk_start(usbd_xfer_handle xfer)
{
	dwc_otg_softc_t *sc = xfer->pipe->device->bus->hci_private;
}

Static void
dwc_otg_device_bulk_abort(usbd_xfer_handle xfer)
{
	dwc_otg_softc_t *sc = xfer->pipe->device->bus->hci_private;
}

Static void
dwc_otg_device_bulk_close(usbd_pipe_handle pipe)
{
	struct dwc_otg_pipe *dpipe = (struct dwc_otg_pipe *)pipe;
	dwc_otg_softc_t *sc = pipe->device->bus->hci_private;
}

/***********************************************************************/

Static usbd_status
dwc_otg_device_intr_transfer(usbd_xfer_handle xfer)
{
	dwc_otg_softc_t *sc = xfer->pipe->device->bus->hci_private;
}

Static usbd_status
dwc_otg_device_intr_start(usbd_xfer_handle xfer)
{
	struct dwc_otg_pipe *dpipe = (struct dwc_otg_pipe *)xfer->pipe;
	usbd_device_handle dev = dpipe->pipe.device;
	dwc_otg_softc_t *sc = dev->bus->hci_private;
}

Static void
dwc_otg_device_intr_abort(usbd_xfer_handle xfer)
{
	dwc_otg_softc_t *sc = xfer->pipe->device->bus->hci_private;
}

Static void
dwc_otg_device_intr_close(usbd_pipe_handle pipe)
{
	struct dwc_otg_pipe *dpipe = (struct dwc_otg_pipe *)pipe;
	dwc_otg_softc_t *sc = pipe->device->bus->hci_private;
}

/***********************************************************************/

usbd_status
dwc_otg_device_isoc_transfer(usbd_xfer_handle xfer)
{
	dwc_otg_softc_t *sc = xfer->pipe->device->bus->hci_private;
}

void
dwc_otg_device_isoc_enter(usbd_xfer_handle xfer)
{
	struct dwc_otg_pipe *dpipe = (struct dwc_otg_pipe *)xfer->pipe;
	usbd_device_handle dev = dpipe->pipe.device;
	dwc_otg_softc_t *sc = dev->bus->hci_private;
}

usbd_status
dwc_otg_device_isoc_start(usbd_xfer_handle xfer)
{
	struct dwc_otg_pipe *dpipe = (struct dwc_otg_pipe *)xfer->pipe;
	dwc_otg_softc_t *sc = dpipe->pipe.device->bus->hci_private;
}

void
dwc_otg_device_isoc_abort(usbd_xfer_handle xfer)
{
	struct dwc_otg_pipe *dpipe = (struct dwc_otg_pipe *)xfer->pipe;
	dwc_otg_softc_t *sc = dpipe->pipe.device->bus->hci_private;
}

void
dwc_otg_device_isoc_done(usbd_xfer_handle xfer)
{
}

usbd_status
dwc_otg_setup_isoc(usbd_pipe_handle pipe)
{
	struct dwc_otg_pipe *dpipe = (struct dwc_otg_pipe *)pipe;
	dwc_otg_softc_t *sc = pipe->device->bus->hci_private;
}

void
dwc_otg_device_isoc_close(usbd_pipe_handle pipe)
{
	struct dwc_otg_pipe *dpipe = (struct dwc_otg_pipe *)pipe;
	dwc_otg_softc_t *sc = pipe->device->bus->hci_private;
}

/***********************************************************************/

usbd_status
dwc_otg_init(dwc_otg_softc_t *sc)
{
	uint32_t temp;

	sc->sc_bus.hci_private = sc;
	sc->sc_bus.usbrev = USBREV_2_0;
	sc->sc_bus.methods = &dwc_otg_bus_methods;

	/* XXXNH */
	sc->sc_noport = 1;

	mutex_init(&sc->sc_lock, MUTEX_DEFAULT, IPL_SOFTUSB);
	mutex_init(&sc->sc_intr_lock, MUTEX_DEFAULT, IPL_SCHED);

	usb_setup_reserve(sc->sc_dev, &sc->sc_dma_reserve, sc->sc_bus.dmatag,
		USB_MEM_RESERVE);

	temp = DWC_OTG_READ_4(sc, DWC_OTG_GSNPSID);
	DPRINTF(("Version = 0x%08x\n", temp));

	/* disconnect */
	DWC_OTG_WRITE_4(sc, DWC_OTG_DCTL, DCTL_SFTDISCON);
	usb_delay_ms(&sc->sc_bus, 30);
	DWC_OTG_WRITE_4(sc, DWC_OTG_GRSTCTL, GRSTCTL_CSFTRST);
	usb_delay_ms(&sc->sc_bus, 8);

	sc->sc_mode = DWC_MODE_HOST;

	switch (sc->sc_mode) {
	case DWC_MODE_DEVICE:
		temp = GUSBCFG_FORCEDEVMODE;
		break;
	case DWC_MODE_HOST:
		temp = GUSBCFG_FORCEHOSTMODE;
		break;
	default:
		temp = 0;
		break;
	}

#ifdef DWC_OTG_USE_HSIC
	DWC_OTG_WRITE_4(sc, DWC_OTG_GUSBCFG,
		GUSBCFG_PHYIF |
		GUSBCFG_TRD_TIM_SET(5) | temp);
	DWC_OTG_WRITE_4(sc, DWC_OTG_GOTGCTL, 0x000000ec);

	temp = DWC_OTG_READ_4(sc, DWC_OTG_GLPMCFG);
	DWC_OTG_WRITE_4(sc, DWC_OTG_GLPMCFG, temp & ~GLPMCFG_HSIC_CONN);
	DWC_OTG_WRITE_4(sc, DWC_OTG_GLPMCFG, temp | GLPMCFG_HSIC_CONN);
#else
	DWC_OTG_WRITE_4(sc, DWC_OTG_GUSBCFG,
		GUSBCFG_ULPI_UTMI_SEL |
		GUSBCFG_TRD_TIM_SET(5) | temp);
	DWC_OTG_WRITE_4(sc, DWC_OTG_GOTGCTL, 0);

	temp = DWC_OTG_READ_4(sc, DWC_OTG_GLPMCFG);
	DWC_OTG_WRITE_4(sc, DWC_OTG_GLPMCFG, temp & ~GLPMCFG_HSIC_CONN);
#endif

	/* clear global nak */
	DWC_OTG_WRITE_4(sc, DWC_OTG_DCTL, DCTL_CGOUTNAK | DCTL_CGNPINNAK);

	/* disable USB port */
	DWC_OTG_WRITE_4(sc, DWC_OTG_PCGCCTL, 0xffffffff);
	usb_delay_ms(&sc->sc_bus, 10);

	/* enable USB port */
	DWC_OTG_WRITE_4(sc, DWC_OTG_PCGCCTL, 0);
	usb_delay_ms(&sc->sc_bus, 10);

	/* pull up D+ */
	dwc_otg_pull_up(sc);

	temp = DWC_OTG_READ_4(sc, DWC_OTG_GHWCFG3);
	sc->sc_fifo_size = 4 * GHWCFG3_DFIFODEPTH_GET(temp);

	temp = DWC_OTG_READ_4(sc, DWC_OTG_GHWCFG2);
	sc->sc_dev_ep_max = min(GHWCFG2_NUMDEVEPS_GET(temp),DWC_OTG_MAX_ENDPOINTS);
	sc->sc_host_ch_max = min(GHWCFG2_NUMHSTCHNL_GET(temp),DWC_OTG_MAX_CHANNELS);

	temp = DWC_OTG_READ_4(sc, DWC_OTG_GHWCFG4);
	sc->sc_dev_in_ep_max = GHWCFG4_NUM_IN_EP_GET(temp);

	DPRINTF(("Total FIFO size = %d bytes, Device EPs = %d/%d Host CHs = %d\n",
		sc->sc_fifo_size, sc->sc_dev_ep_max, sc->sc_dev_in_ep_max,
		sc->sc_host_ch_max));

	/* setup fifo */
	if (dwc_otg_init_fifo(sc, DWC_MODE_OTG))
		return EINVAL;

	/* enable interrupts */
	sc->sc_irq_mask = DWC_OTG_MSK_GINT_ENABLED;
	DWC_OTG_WRITE_4(sc, DWC_OTG_GINTMSK, sc->sc_irq_mask);

	switch (sc->sc_mode) {
	case DWC_MODE_DEVICE:
	case DWC_MODE_OTG:
		/* enable endpoint interrupts */
		temp = DWC_OTG_READ_4(sc, DWC_OTG_GHWCFG2);
		if (temp & GHWCFG2_MPI) {
			for (i = 0; i < sc->sc_dev_in_ep_max; ++i) {
				DWC_OTG_WRITE_4(sc, DWC_OTG_DIEPEACHINTMSK(i),
					DIEPMSK_XFERCOMPLMSK);
				DWC_OTG_WRITE_4(sc, DWC_OTG_DOEPEACHINTMSK(i), 0);
			}
			DWC_OTG_WRITE_4(sc, DWC_OTG_DEACHINTMSK, 0xffff);
		} else {
			DWC_OTG_WRITE_4(sc, DWC_OTG_DIEPMSK, DIEPMSK_XFERCOMPLMSK);
			DWC_OTG_WRITE_4(sc, DWC_OTG_DOEPMSK, 0);
			DWC_OTG_WRITE_4(sc, DWC_OTG_DAINTMSK, 0xffff);
		}
		break;
	}

	switch (sc->sc_mode) {
	case DWC_MODE_HOST:
	case DWC_MODE_OTG:
		/* setup clocks */
		offonbits(sc, DWC_OTG_HCFG,
			HCFG_FSLSSUPP | HCFG_FSLSPCLKSEL_MASK,
			1 << HCFG_FSLSPCLKSEL_SHIFT);
		break;
	}

	/* enable global IRQ */
	DWC_OTG_WRITE_4(sc, DWC_OTG_GAHBCFG, GAHBCFG_GLBLINTRMSK);

	temp = DWC_OTG_READ_4(sc, DWC_OTG_GOTGCTL);
	DPRINTFN(5, ("GOTCTL=0x%08x\n", temp));

	/* read initial VBUS state */
	dwc_otg_vbus_interrupt(sc);

	/* catch any lost interrupts */
	dwc_otg_poll(&sc->sc_bus);

	return 0;
}

//  XXX used somewhere? 
static
int dwc_otg_intr(void *p)
{
	dwc_otg_softc_t *sc = p;
	int ret = 0;

	if (sc == NULL)
		return 0;

	mutex_spin_enter(&sc->sc_intr_lock);

	if (sc->sc_dying || !device_has_power(sc->sc_dev))
		goto done;

	if (sc->sc_bus.use_polling) {
		uint32_t status = DWC_OTG_READ_4(sc, DWC_OTG_GINTSTS);
		DWC_OTG_WRITE_4(sc, DWC_OTG_GINTSTS, status);
	} else {
		ret = dwc_otg_intr1(sc);
	}

done:
	mutex_spin_exit(&sc->sc_intr_lock);

	return ret;
}

int
dwc_otg_detach(struct dwc_otg_softc *sc, int flags)
{
	int rv = 0;

	if (sc->sc_child != NULL)
		rv = config_detach(sc->sc_child, flags);

	if (rv != 0)
		return (rv);
}

bool
dwc_otg_shutdown(device_t self, int flags)
{
	dwc_otg_softc_t *sc = device_private(self);
}

void
dwc_otg_childdet(device_t self, device_t child)
{
	struct dwc_otg_softc *sc = device_private(self);
}

int
dwc_otg_activate(device_t self, enum devact act)
{
	struct dwc_otg_softc *sc = device_private(self);
}

bool
dwc_otg_resume(device_t dv, const pmf_qual_t *qual)
{
	dwc_otg_softc_t *sc = device_private(dv);
}

bool
dwc_otg_suspend(device_t dv, const pmf_qual_t *qual)
{
	dwc_otg_softc_t *sc = device_private(dv);
}

/***********************************************************************/

#ifdef DWC_OTG_DEBUG
void
dwc_otg_dump_global_regs(dwc_otg_softc_t *sc)
{
	int i, n;

	printf("GOTGCTL        0x%08x\n", DWC_OTG_READ_4(sc, DWC_OTG_GOTGCTL));
	printf("GOTGINT        0x%08x\n", DWC_OTG_READ_4(sc, DWC_OTG_GOTGINT));
	printf("GAHBCFG        0x%08x\n", DWC_OTG_READ_4(sc, DWC_OTG_GAHBCFG));
	printf("GUSBCFG        0x%08x\n", DWC_OTG_READ_4(sc, DWC_OTG_GUSBCFG));
	printf("GRSTCTL        0x%08x\n", DWC_OTG_READ_4(sc, DWC_OTG_GRSTCTL));
	printf("GINTSTS        0x%08x\n", DWC_OTG_READ_4(sc, DWC_OTG_GINTSTS));
	printf("GINTMSK        0x%08x\n", DWC_OTG_READ_4(sc, DWC_OTG_GINTMSK));
	printf("GRXSTSR        0x%08x\n", DWC_OTG_READ_4(sc, DWC_OTG_GRXSTSR));
	printf("GRXSTSP        0x%08x\n", DWC_OTG_READ_4(sc, DWC_OTG_GRXSTSP));
	printf("GRXFSIZ        0x%08x\n", DWC_OTG_READ_4(sc, DWC_OTG_GRXFSIZ));
	printf("GNPTXFSIZ      0x%08x\n", DWC_OTG_READ_4(sc, DWC_OTG_GNPTXFSIZ));
	printf("GNPTXSTS       0x%08x\n", DWC_OTG_READ_4(sc, DWC_OTG_GNPTXSTS));
	printf("GI2CCTL        0x%08x\n", DWC_OTG_READ_4(sc, DWC_OTG_GI2CCTL));
	printf("GPVNDCTL       0x%08x\n", DWC_OTG_READ_4(sc, DWC_OTG_GPVNDCTL));
	printf("GGPIO          0x%08x\n", DWC_OTG_READ_4(sc, DWC_OTG_GGPIO));
	printf("GUID           0x%08x\n", DWC_OTG_READ_4(sc, DWC_OTG_GUID));
	printf("GSNPSID        0x%08x\n", DWC_OTG_READ_4(sc, DWC_OTG_GSNPSID));
	printf("GHWCFG1        0x%08x\n", DWC_OTG_READ_4(sc, DWC_OTG_GHWCFG1));
	printf("GHWCFG2        0x%08x\n", DWC_OTG_READ_4(sc, DWC_OTG_GHWCFG2));
	printf("GHWCFG3        0x%08x\n", DWC_OTG_READ_4(sc, DWC_OTG_GHWCFG3));
	printf("GHWCFG4        0x%08x\n", DWC_OTG_READ_4(sc, DWC_OTG_GHWCFG4));
	printf("GLPMCFG        0x%08x\n", DWC_OTG_READ_4(sc, DWC_OTG_GLPMCFG));
	printf("HPTXFSIZ       0x%08x\n", DWC_OTG_READ_4(sc, DWC_OTG_HPTXFSIZ));

	n = GHWCFG4_NUMDEVPERIOEPS_GET(DWC_OTG_READ_4(sc, DWC_OTG_HWCFG4));
	for (i=1; i<n; ++i) {
		printf("DPTXFSIZ[%2d]  0x%08x\n", i,
			DWC_OTG_READ_4(sc, DWC_OTG_DPTXFSIZ(i)));
	}

	printf("PCGCCTL        0x%08x\n", DWC_OTG_READ_4(sc, DWC_OTG_PCGCCTL));


}

void
dwc_otg_dump_host_regs(dwc_otg_softc_t *sc)
{
	int i, n;

	printf("HCFG           0x%08x\n", DWC_OTG_READ_4(sc, DWC_OTG_HCFG));
	printf("HFIR           0x%08x\n", DWC_OTG_READ_4(sc, DWC_OTG_HFIR));
	printf("HFNUM          0x%08x\n", DWC_OTG_READ_4(sc, DWC_OTG_HFNUM));
	printf("HPTXSTS        0x%08x\n", DWC_OTG_READ_4(sc, DWC_OTG_HPTXSTS));
	printf("HAINT          0x%08x\n", DWC_OTG_READ_4(sc, DWC_OTG_HAINT));
	printf("HAINTMSK       0x%08x\n", DWC_OTG_READ_4(sc, DWC_OTG_HAINTMSK));
	printf("HFLBADDR       0x%08x\n", DWC_OTG_READ_4(sc, DWC_OTG_HFLBADDR));
	printf("HPRT0          0x%08x\n", DWC_OTG_READ_4(sc, DWC_OTG_HPRT0));

	n = GHWCFG2_NUMHSTCHNL_GET(DWC_OTG_READ_4(sc, DWC_OTG_HWCFG2));
	for (i=0; i<n; ++i) {
		printf("Host Channel %d Specific Registers\n", i);
		printf("HCCHAR         0x%08x\n", DWC_OTG_READ_4(sc,DWC_OTG_HCCHAR(i)));
		printf("HCSPLT         0x%08x\n", DWC_OTG_READ_4(sc,DWC_OTG_HCSPLT(i)));
		printf("HCINT          0x%08x\n", DWC_OTG_READ_4(sc,DWC_OTG_HCINT(i)));
		printf("HCINTMSK       0x%08x\n", DWC_OTG_READ_4(sc,DWC_OTG_HCINTMSK(i)));
		printf("HCTSIZ         0x%08x\n", DWC_OTG_READ_4(sc,DWC_OTG_HCTSIZ(i)));
		printf("HCDMA          0x%08x\n", DWC_OTG_READ_4(sc,DWC_OTG_HCDMA(i)));
		printf("HCDMAB         0x%08x\n", DWC_OTG_READ_4(sc,DWC_OTG_HCDMAB(i)));
	}


}
#endif

/***********************************************************************/

Static void
dwc_otg_pull_up(struct dwc_otg_softc *sc)
{
	if (!sc->sc_d_pulled_up) {
		sc->sc_d_pulled_up = 1;
		offonbits(sc, DCTL_SFTDISCON, 0);
	}
}

Static void
dwc_otg_pull_down(struct dwc_otg_softc *sc)
{
	if (sc->sc_d_pulled_up) {
		sc->sc_d_pulled_up = 0;
		offonbits(sc, 0, DCTL_SFTDISCON);
	}
}

Static void
dwc_otg_write_td(struct dwc_otg_softc *sc, dwc_otg_soft_td_t *std,
	void *buf, size_t bytes, uint32_t flags)
{
	std->td->buf = buf;
	std->td->status = flags | BS_HOST_BUSY;
	usb_syncmem(sd->dma, sd->offs, sizeof(dwc_otg_td_t),
		BUS_DMASYNC_PREWRITE | BUS_DMASYNC_PREREAD);
	std->td->status = flags | bytes | BS_HOST_BUSY;
	std->td->status = flags | bytes | BS_HOST_READY;
	usb_syncmem(sd->dma, sd->offs, sizeof(dwc_otg_td_t),
		BUS_DMASYNC_PREWRITE | BUS_DMASYNC_PREREAD);
}

#if 0
Static void
dwc_otg_start_dma(...)
{
	KASSERT(tdbuf is properly aligned);
	KASSERT(size doesn't span 2K pages);

	dopng = 0;
	if (do_ping? && !ep_is_in)
		dopng = HCTSIZ_DOPNG;

	if (isoc) {
		if (speed == DWC_OTG_EP_SPEED_HIGH) {
			if (ep_is_in) {
				if (mc == 1)
					pid = DWC_OTG_HC_PID_DATA0;
				else if (mc == 2)
					pid = DWC_OTG_HC_PID_DATA1;
				else
					pid = DWC_OTG_HC_PID_DATA2;
			} else {
				if (mc == 1)
					pid = DWC_OTG_HC_PID_DATA0;
				else
					pid = DWC_OTG_HC_PID_MDATA;
			}
		} else {
			pid = DWC_OTG_HC_PID_DATA0;
		}
	}

	DWC_OTG_WRITE_4(sc, DWC_OTG_HCTSIZ(ch),
		dopng
		| pid << HCTSIZ_PID_SHIFT
		| (numtd-1) << HCTSIZ_NTD_SHIFT
		| schinfo << HCTSIZ_SCHINFO_SHIFT);
	DWC_OTG_WRITE_4(sc, DWC_OTG_HCDMA(ch),
		(uint32_t)(uintptr_t)td);
	offonbits(sc, DWC_OTG_HCCHAR(ch),
		HCCHAR_MC_MASK | HCCHAR_CHDIS,
		mc << HCCHAR_MC_SHIFT | HCCHAR_CHENA);
}

Static void
dwc_otg_set_address(strcut dwc_otg_softc *sc, uint8_t addr)
{
	offonbits(sc, DWC_OTG_DCFG,
		DCFG_DEVADDR_SET(0x7f),
		DCFG_DEVADDR_SET(addr));
}
#endif

/***********************************************************************/

int
dwc_otg_intr1(dwc_otg_softc_t *sc)
{
	uint32_t status;

	status = DWC_OTG_READ_4(sc, DWC_OTG_GINTSTS);
	DWC_OTG_WRITE_4(sc, DWC_OTG_GINTSTS, status);

	if (status & GINTSTS_USBRST) {
		/* USB reset */
		// "root_intr" ?
	}

	if (status & GINTSTS_ENUMDONE) {
		/* enumeration complete, "end of reset" */

		/*
		reset FIFOs
		reset function address
		figure out enumeration speed
		disable resume and enable suspend interrupt
		"root_intr" ?
		*/
	}

	if (status & GINTSTS_PRTINT) {
		sc->sc_hprt = DWC_OTG_READ_4(sc, DWC_OTG_HPRT);
		DWC_OTG_WRITE_4(sc, DWC_OTG_HPRT, (hprt & (
			HPRT_PRTPWR | HPRT_PRTENCHNG |
			HPRT_PRTCONNDET | HPRT_PRTOVRCURRCHNG)) |
			sc->sc_hprt_val);

		if (sc->sc_hprt & HPRT_PRTSUSP)
			dwc_otg_suspend_irq(sc);
		else
			dwc_otg_resume_irq(sc);
		//"root_intr" ?
	}

	if (status & GINTSTS_WKUPINT) {
		dwc_otg_resume_irq(sc);
	}

	if (status & GINTSTS_USBSUSP) {
		dwc_otg_suspend_irq(sc);
	}

	if (status & (GINTSTS_USBSUSP | GINTSTS_USBRST | GINTMSK_OTGINTMSK | GINTSTS_SESSREQINT)) {
		vbus_interrupt(sc);
	}

	if (status & GINTSTS_IEPINT) {
		/* clear all IN endpoint interrupts */
	}

	if (status & GINTSTS_SOF) {
		if (sc->sc_irq_mask & GINTMSK_SOFMSK) {
			search for channel that is waiting for SOF
			if none found
				disable GINTMSK_SOFMSK
		}
	}

	/* poll FIFOs */
	dwc_otg_intr_xxx(sc);
}

dwc_otg_intr_xxx(dwc_otg_softc_t *sc)
{

repeat:
	for (ch = 0; ch < sc->sc_host_ch_max; ++ch) {
		intrs = DWC_OTG_READ_4(sc, DWC_OTG_HCINT(ch));
		DWC_OTG_WRITE_4(sc, DWC_OTG_HCINT(ch), intrs);
	}

	rx = 0;
	temp = DWC_OTG_READ_4(sc, DWC_OTG_GINTSTS);
	if (temp & GINTSTS_RXFLVL)
		rx = DWC_OTG_READ_4(sc, DWC_OTG_GRXSTSPD);

	switch (rx & GRXSTSRD_PKTSTS_MASK) {
	case GRXSTSRD_STP_DATA:
	case GRXSTSRD_OUT_DATA:
		bcnt = GRXSTSRD_BCNT_GET(rx);
		epno = GRXSTSRD_CHNUM_GET(rx);

		if (bcnt) {
			/* read bytes from fifo */
			bus_space_read_region(sc->sc_iot, sc->sc_ioh,
				DWC_OTG_DFIFO(epno),
				sc->sc_rx_bounce_buffer, (bcnt+3)/4);
		}
#ifdef notyet
		if (ep not active) {
			enable GINTSTS_RXFLVL in GINTMSK
		}
#endif
		break;
	default:
		epno = GRXSTSRD_CHNUM_GET(rx);

		/* skip non-data messages */
		if (ep not active) {
			enable GINTSTS_RXFLVL in GINTMSK
			goto repeat;
		}
		break;
	}

	poll transfers
	if ("queue has been modified") goto repeat

	if (rx == 0)
		goto repeat;

	disable GINTSTS_RXFLVL in GINTMSK
}

Static void
dwc_otg_vbus_interrupt(struct dwc_otg_softc *sc)
{
	uint32_t temp;

	temp = DWC_OTG_READ_4(sc, DWC_OTG_GOTGCTL);

	if (temp & (GOTGCTL_ASESVLD | GOTGCTL_BSESVLD)) {
		if (!sc->sc_flags.status_vbus) {
			sc->sc_flags.status_vbus = 1;
			dwc_otg_root_intr(sc);
		}
	} else {
		if (sc->sc_flags.status_vbus) {
			sc->sc_flags.status_vbus = 0;
			sc->sc_flags.status_bus_reset = 0;
			sc->sc_flags.status_suspend = 0;
			sc->sc_flags.change_suspend = 0;
			sc->sc_flags.change_connect = 1;
			dwc_otg_root_intr(sc);
		}
	}
}

static int
dwc_otg_init_fifo(struct dwc_otg_softc *sc, uint8_t mode)
{
	struct dwc_otg_profile *pf;
	uint32_t fifo_size;
	uint32_t fifo_regs;
	uint32_t tx_start;
	uint8_t x;

	fifo_size = sc->sc_fifo_size;

	fifo_regs = 4 * (sc->sc_dev_ep_max + sc->sc_dev_in_ep_max);

	if (fifo_size >= fifo_regs)
		fifo_size -= fifo_regs;
	else
		fifo_size = 0;

	/* split equally for IN and OUT */
	fifo_size /= 2;

	DWC_OTG_WRITE_4(sc, DWC_OTG_GRXFSIZ, fifo_size / 4);

	/* align to 4-bytes */
	fifo_size &= ~3;

	tx_start = fifo_size;

	if (fifo_size < 0x40) {
		DPRINTFN(-1, "Not enough data space for EP0 FIFO.\n");
		return EINVAL;
	}

	if (mode == DWC_MODE_HOST) {

		/* reset active endpoints */
		sc->sc_active_rx_ep = 0;

		fifo_size /= 2;

		DWC_OTG_WRITE_4(sc, DWC_OTG_GNPTXFSIZ,
		    ((fifo_size / 4) << 16) | (tx_start / 4));

		tx_start += fifo_size;

		DWC_OTG_WRITE_4(sc, DWC_OTG_HPTXFSIZ,
		    ((fifo_size / 4) << 16) | (tx_start / 4));

		for (x = 0; x != sc->sc_host_ch_max; x++) {
			/* enable interrupts */
			DWC_OTG_WRITE_4(sc, DWC_OTG_HCINTMSK(x),
			    HCINT_STALL | HCINT_BBLERR |
			    HCINT_XACTERR |
			    HCINT_NAK | HCINT_ACK | HCINT_NYET |
			    HCINT_CHHLTD | HCINT_FRMOVRUN |
			    HCINT_DATATGLERR);
		}

		/* enable host channel interrupts */
		DWC_OTG_WRITE_4(sc, DWC_OTG_HAINTMSK,
		    (1U << sc->sc_host_ch_max) - 1U);

	}

#ifdef notyet
	if (mode == DWC_MODE_DEVICE) {

	    DWC_OTG_WRITE_4(sc, DWC_OTG_GNPTXFSIZ,
		(0x10 << 16) | (tx_start / 4));
	    fifo_size -= 0x40;
	    tx_start += 0x40;

	    /* setup control endpoint profile */
	    sc->sc_hw_ep_profile[0].usb = dwc_otg_ep_profile[0];

	    /* reset active endpoints */
	    sc->sc_active_rx_ep = 1;

	    for (x = 1; x != sc->sc_dev_ep_max; x++) {

		pf = sc->sc_hw_ep_profile + x;

		pf->usb.max_out_frame_size = 1024 * 3;
		pf->usb.is_simplex = 0;	/* assume duplex */
		pf->usb.support_bulk = 1;
		pf->usb.support_interrupt = 1;
		pf->usb.support_isochronous = 1;
		pf->usb.support_out = 1;

		if (x < sc->sc_dev_in_ep_max) {
			uint32_t limit;

			limit = (x == 1) ? DWC_OTG_MAX_TXN :
			    (DWC_OTG_MAX_TXN / 2);

			if (fifo_size >= limit) {
				DWC_OTG_WRITE_4(sc, DWC_OTG_DIEPTXF(x),
				    ((limit / 4) << 16) |
				    (tx_start / 4));
				tx_start += limit;
				fifo_size -= limit;
				pf->usb.max_in_frame_size = 0x200;
				pf->usb.support_in = 1;
				pf->max_buffer = limit;

			} else if (fifo_size >= 0x80) {
				DWC_OTG_WRITE_4(sc, DWC_OTG_DIEPTXF(x),
				    ((0x80 / 4) << 16) | (tx_start / 4));
				tx_start += 0x80;
				fifo_size -= 0x80;
				pf->usb.max_in_frame_size = 0x40;
				pf->usb.support_in = 1;

			} else {
				pf->usb.is_simplex = 1;
				DWC_OTG_WRITE_4(sc, DWC_OTG_DIEPTXF(x),
				    (0x0 << 16) | (tx_start / 4));
			}
		} else {
			pf->usb.is_simplex = 1;
		}

		DPRINTF("FIFO%d = IN:%d / OUT:%d\n", x,
		    pf->usb.max_in_frame_size,
		    pf->usb.max_out_frame_size);
	    }
	}
#endif

	/* reset RX FIFO */
	DWC_OTG_WRITE_4(sc, DWC_OTG_GRSTCTL, GRSTCTL_RXFFLSH);

	if (mode != DWC_MODE_OTG) {
		/* reset all TX FIFOs */
		DWC_OTG_WRITE_4(sc, DWC_OTG_GRSTCTL,
		    GRSTCTL_TXFIFO(0x10) | GRSTCTL_TXFFLSH);
	} else {
		/* reset active endpoints */
		sc->sc_active_rx_ep = 0;
	}

	return 0;
}

