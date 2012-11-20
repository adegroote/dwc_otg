/*	$NetBSD$	*/

/*-
 * Copyright (c) 2012 Hans Petter Selasky. All rights reserved.
 * Copyright (c) 2010-2011 Aleksandr Rybalko. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

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

#define	DWC_OTG_MSK_GINT_ENABLED	\
   (GINTSTS_ENUMDONE |			\
   GINTSTS_USBRST |			\
   GINTSTS_USBSUSP |			\
   GINTSTS_IEPINT |			\
   GINTSTS_RXFLVL |			\
   GINTSTS_SESSREQINT |			\
   GINTMSK_OTGINTMSK |			\
   GINTMSK_HCHINTMSK |			\
   GINTSTS_PRTINT)

#define	DWC_OTG_BUS2SC(bus) ((bus)->hci_private)

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

Static void		dwc_otg_pull_up(struct dwc_otg_softc *);
Static void		dwc_otg_pull_down(struct dwc_otg_softc *);
Static void 		dwc_otg_clocks_on(dwc_otg_softc_t*);
Static void	 	dwc_otg_clocks_off(dwc_otg_softc_t*);
Static void		dwc_otg_timer(void*);
Static void		dwc_otg_timer_start(struct dwc_otg_softc *);
Static void		dwc_otg_timer_stop(struct dwc_otg_softc *);
Static void		dwc_otg_suspend_irq(struct dwc_otg_softc *);
Static void		dwc_otg_resume_irq(struct dwc_otg_softc *);
Static void		dwc_otg_wakeup_peer(struct dwc_otg_softc *);
Static void		dwc_otg_interrupt_poll(dwc_otg_softc_t *);
Static void		dwc_otg_rhc(void *);

Static void		dwc_otg_vbus_interrupt(struct dwc_otg_softc *);

Static void		dwc_otg_rem_ed(dwc_otg_softc_t*, dwc_otg_soft_ed_t* ,
								dwc_otg_soft_ed_t* );
Static dwc_otg_soft_ed_t* dwc_otg_alloc_sed(dwc_otg_softc_t*);
Static void		dwc_otg_free_sed(dwc_otg_softc_t*, dwc_otg_soft_ed_t* );

static int dwc_otg_init_fifo(struct dwc_otg_softc *, uint8_t);


#define DWC_OTG_READ_4(sc, reg) \
  bus_space_read_4((sc)->sc_iot, (sc)->sc_ioh, reg)
#define DWC_OTG_WRITE_4(sc, reg, data)  \
  bus_space_write_4((sc)->sc_iot, (sc)->sc_ioh, reg, data)
#define offonbits(sc, reg, off, on) \
  DWC_OTG_WRITE_4((sc),(reg),(DWC_OTG_READ_4((sc),(reg)) & ~(off)) | (on))

#define dwc_otg_add_intr_list(sc, ex) \
	TAILQ_INSERT_TAIL(&(sc)->sc_intrhead, (ex), inext);
#define dwc_otg_del_intr_list(sc, ex) \
	do { \
		TAILQ_REMOVE(&sc->sc_intrhead, (ex), inext); \
	} while (0)

struct dwc_otg_pipe {
	struct usbd_pipe pipe;

#if 0
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
#endif
};

#define DWC_OTG_INTR_ENDPT 1

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

	xfer = kmem_zalloc(sizeof(struct dwc_otg_xfer), KM_SLEEP);
#ifdef DIAGNOSTIC
	if (xfer != NULL) {
		xfer->busy_free = XFER_BUSY;
	}
#endif
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

	KASSERT(sc->sc_bus.use_polling || mutex_owned(&sc->sc_lock));

	DPRINTF(("%s\n", __func__));


}

Static void
dwc_otg_waitintr(dwc_otg_softc_t *sc, usbd_xfer_handle xfer)
{
}

Static void
dwc_otg_device_ctrl_done(usbd_xfer_handle xfer)
{
	struct dwc_otg_pipe *dpipe = (struct dwc_otg_pipe *)xfer->pipe;

	DPRINTF(("%s\n", __func__));

	dpipe = dpipe;
}

Static void
dwc_otg_device_intr_done(usbd_xfer_handle xfer)
{
	struct dwc_otg_pipe *dpipe = (struct dwc_otg_pipe *)xfer->pipe;
	dwc_otg_softc_t *sc = dpipe->pipe.device->bus->hci_private;

	DPRINTF(("%s\n", __func__));

	sc = sc;
}

Static void
dwc_otg_device_bulk_done(usbd_xfer_handle xfer)
{
	dwc_otg_softc_t *sc = xfer->pipe->device->bus->hci_private;

	DPRINTF(("%s\n", __func__));

	sc = sc;
}

usbd_status
dwc_otg_device_request(usbd_xfer_handle xfer)
{
	struct dwc_otg_pipe *dpipe = (struct dwc_otg_pipe *)xfer->pipe;
	usb_device_request_t *req = &xfer->request;
	usbd_device_handle dev = dpipe->pipe.device;
	dwc_otg_softc_t *sc = dev->bus->hci_private;
	usbd_status err = 0;

	DPRINTF(("%s\n", __func__));

	req = req;
	sc = sc;

	return err;
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
	uint8_t xfertype = UE_GET_XFERTYPE(ed->bmAttributes);
	uint8_t epnum = UE_GET_ADDR(ed->bmAttributes);
	uint32_t hcchar, hcsplt, ival;
	usbd_status err;

	DPRINTF(("%s: addr %d xfertype %d dir %s\n", __func__, addr, xfertype,
	    UE_GET_DIR(ed->bmAttributes) == UE_DIR_IN ? "in" : "out"));

	if (sc->sc_dying) {
		err = USBD_IOERROR;
		goto fail;
	}

	if (addr == sc->sc_addr) {
		switch (ed->bEndpointAddress) {
		case USB_CONTROL_ENDPOINT:
			pipe->methods = &dwc_otg_root_ctrl_methods;
			break;
		case UE_DIR_IN | DWC_OTG_INTR_ENDPT:
			pipe->methods = &dwc_otg_root_intr_methods;
			break;
		default:
			DPRINTF(("%s: bad bEndpointAddress 0x%02x\n", __func__,
			    ed->bEndpointAddress));
			return USBD_INVAL;
		}
		DPRINTF(("%s: root hub pipe open\n", __func__));
		return USBD_NORMAL_COMPLETION;
	}

	switch (xfertype) {
	case UE_CONTROL:
		pipe->methods = &dwc_otg_device_ctrl_methods;
		DPRINTF(("%s: UE_CONTROL methods\n", __func__));
		break;
	case UE_INTERRUPT:
		DPRINTF(("%s: UE_INTERRUPT methods\n", __func__));
		pipe->methods = &dwc_otg_device_intr_methods;
		break;
	case UE_ISOCHRONOUS:
		DPRINTF(("%s: US_ISOCHRONOUS methods\n", __func__));
		pipe->methods = &dwc_otg_device_isoc_methods;
		break;
	case UE_BULK:
		DPRINTF(("%s: UE_BULK methods\n", __func__));
		pipe->methods = &dwc_otg_device_bulk_methods;
		break;
	default:
		DPRINTF(("%s: bad xfer type %d\n", __func__, xfertype));
		return USBD_INVAL;
	}

	hcsplt = 0;
	hcchar = HCCHAR_CHENA |
	    (addr << HCCHAR_DEVADDR_SHIFT) |
	    (xfertype << HCCHAR_EPTYPE_SHIFT) |
	    (epnum << HCCHAR_EPNUM_SHIFT) |
	    ((UGETW(ed->wMaxPacketSize)) << HCCHAR_MPS_SHIFT);

	if (UE_GET_DIR(ed->bEndpointAddress) == UE_DIR_IN)
		hcchar |= HCCHAR_EPDIR_IN;

	switch (dev->speed) {
	case USB_SPEED_LOW:
		DPRINTF(("%s: USB_SPEED_LOW\n", __func__));
		hcchar |= HCCHAR_LSPDDEV;
		/* FALLTHROUGH */
	case USB_SPEED_FULL:
		DPRINTF(("%s: USB_SPEED_FULL\n", __func__));
		/* check if root HUB port is running High Speed */
		if (sc->sc_flags.status_high_speed != 0) {
			hcsplt = HCSPLT_SPLTENA |
			    (dev->myhsport->portno << HCSPLT_PRTADDR_SHIFT) |
			    (dev->myhsport->parent->address <<
			    HCSPLT_HUBADDR_SHIFT);
			if (xfertype == UE_ISOCHRONOUS)  /* XXX */
				hcsplt |= (3 << HCSPLT_XACTPOS_SHIFT);
		}
		break;

	case USB_SPEED_HIGH:
		DPRINTF(("%s: USB_SPEED_HIGH\n", __func__));
		if (xfertype == UE_ISOCHRONOUS || xfertype == UE_INTERRUPT) {
			hcchar |= (/*(xfer->max_packet_count & 3)*/ 1
				<< HCCHAR_MC_SHIFT);
		}
		break;

	default:
		break;
	}

	int fps_shift = 1;

	switch (xfertype) {
	case UE_ISOCHRONOUS:
		ival = 1 << fps_shift;
		break;
	case UE_INTERRUPT:
		ival = pipe->interval / DWC_OTG_HOST_TIMER_RATE;
		if (ival == 0)
			ival = 1;
		else if (ival > 127)
			ival = 127;
		break;
	default:
		ival = 0;
	}

	dpipe->hcchar = hcchar;
	dpipe->hcsplt = hcsplt;
	dpipe->tmr_res = ival;

	DPRINTF(("%s: hcchar 0x%08x hcchar 0x%08x ival %d\n", __func__, hcchar,
	    hcsplt, ival));

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
	usb_delay_ms(&sc->sc_bus, 1);
	dwc_otg_free_sed(sc, dpipe->sed);
}

Static void
dwc_otg_rem_ed(dwc_otg_softc_t* sc, dwc_otg_soft_ed_t* sed, dwc_otg_soft_ed_t* head)
{
	// XXX TODO
}

Static dwc_otg_soft_ed_t*
dwc_otg_alloc_sed(dwc_otg_softc_t* sc)
{
	// XXX TODO
}

Static void
dwc_otg_free_sed(dwc_otg_softc_t* sc, dwc_otg_soft_ed_t* sed)
{
	// XXX TODO
}


/*
 * Abort a device request.
 */
Static void
dwc_otg_abort_xfer(usbd_xfer_handle xfer, usbd_status status)
{
	struct dwc_otg_pipe *dpipe = (struct dwc_otg_pipe *)xfer->pipe;
	dwc_otg_softc_t *sc = dpipe->pipe.device->bus->hci_private;


	DPRINTF(("%s\n", __func__));

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

	/* XXX Where does come from the channel for now ? */
	DWC_OTG_WRITE_4(sc, DOTG_HCINTMSK(ch), HCINTMSK_CHHLTDMSK);
	DWC_OTG_WRITE_4(sc, DOTG_HCINT(ch), ~HCINTMSK_CHHLTDMSK);

	if ((DWC_OTG_READ_4(sc, DOTG_HCCHAR(ch)) & HCCHAR_CHENA) == 0)
		return;

	offonbits(sc, DOTG_HCCHAR(ch), HCCHAR_CHENA, HCCHAR_CHDIS);
}

Static void
dwc_otg_noop(usbd_pipe_handle pipe)
{
	DPRINTF(("%s\n", __func__));

}

Static void
dwc_otg_device_clear_toggle(usbd_pipe_handle pipe)
{
	DPRINTF(("%s\n", __func__));

}

/***********************************************************************/

/*
 * Data structures and routines to emulate the root hub.
 */

Static const usb_device_descriptor_t dwc_otg_devd = {
	.bLength = sizeof(usb_device_descriptor_t),
	.bDescriptorType = UDESC_DEVICE,
	.bcdUSB = {0x00, 0x02},
	.bDeviceClass = UDCLASS_HUB,
	.bDeviceSubClass = UDSUBCLASS_HUB,
	.bDeviceProtocol = UDPROTO_HSHUBSTT,
	.bMaxPacketSize = 64,
	.bcdDevice = {0x00, 0x01},
	.iManufacturer = 1,
	.iProduct = 2,
	.bNumConfigurations = 1,
};


struct dwc_otg_config_desc {
	usb_config_descriptor_t confd;
	usb_interface_descriptor_t ifcd;
	usb_endpoint_descriptor_t endpd;
} __packed;

Static const struct dwc_otg_config_desc dwc_otg_confd = {
	.confd = {
		.bLength = USB_CONFIG_DESCRIPTOR_SIZE,
		.bDescriptorType = UDESC_CONFIG,
		.wTotalLength[0] = sizeof(dwc_otg_confd),
		.bNumInterface = 1,
		.bConfigurationValue = 1,
		.iConfiguration = 0,
		.bmAttributes = UC_SELF_POWERED,
		.bMaxPower = 0,
	},
	.ifcd = {
		.bLength = USB_INTERFACE_DESCRIPTOR_SIZE,
		.bDescriptorType = UDESC_INTERFACE,
		.bInterfaceNumber = 0,
		.bAlternateSetting = 0,
		.bNumEndpoints = 1,
		.bInterfaceClass = UICLASS_HUB,
		.bInterfaceSubClass = UISUBCLASS_HUB,
		.bInterfaceProtocol = UIPROTO_FSHUB,
		.iInterface = 0
	},
	.endpd = {
		.bLength = USB_ENDPOINT_DESCRIPTOR_SIZE,
		.bDescriptorType = UDESC_ENDPOINT,
		.bEndpointAddress = UE_DIR_IN | DWC_OTG_INTR_ENDPT,
		.bmAttributes = UE_INTERRUPT,
		.wMaxPacketSize = {8, 0},			/* max packet */
		.bInterval = 255,
	},
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
	usbd_status err;

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
	usb_device_request_t *req;
	uint8_t *buf;
	int port, i;
	int len, value, index, l, totlen;
	usb_port_status_t ps;
	usb_hub_descriptor_t hubd;
	usbd_status err = USBD_IOERROR;
	uint32_t v;

	if (sc->sc_dying)
		return USBD_IOERROR;

	req = &xfer->request;

	DPRINTFN(4, ("%s: type=0x%02x request=%02x\n", __func__,
	    req->bmRequestType, req->bRequest));

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
		DPRINTFN(8, ("%s: wValue=0x%04x\n", __func__, value));

		if (len == 0)
			break;
		switch (value) {
		case C(0, UDESC_DEVICE):
			l = min(len, USB_DEVICE_DESCRIPTOR_SIZE);
// 			USETW(dwc_otg_devd.idVendor, sc->sc_id_vendor);
			memcpy(buf, &dwc_otg_devd, l);
			buf += l;
			len -= l;
			totlen += l;

			break;
		case C(0, UDESC_CONFIG):
			l = min(len, sizeof(dwc_otg_confd));
			memcpy(buf, &dwc_otg_confd, l);
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
		DPRINTFN(9, ("%s: UR_CLEAR_FEATURE port=%d feature=%d\n",
		    __func__, index, value));
		if (index < 1 || index > sc->sc_noport)
                        goto fail;

		switch (value) {
		case UHF_PORT_ENABLE:
			if (sc->sc_flags.status_device_mode == 0) {
				DWC_OTG_WRITE_4(sc, DOTG_HPRT,
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
				DWC_OTG_WRITE_4(sc, DOTG_HPRT, HPRT_PRTENA);
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
		DPRINTFN(8, ("%s: get port status i=%d\n", __func__, index));

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
			/* XXX FreeBSD specific, which value ?
			value = UPS_PORT_MODE_DEVICE;
			*/
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
		DPRINTFN(9, ("%s: UR_SET_FEATURE port=%d feature=%d\n",
		    __func__, index, value));

		if (index < 1 || index > sc->sc_noport)
			goto fail;

		switch (value) {
		case UHF_PORT_ENABLE:
			DPRINTF(("%s: UHF_PORT_ENABLE\n", __func__));
			break;

		case UHF_PORT_SUSPEND:
			DPRINTF(("%s: UHF_PORT_SUSPEND device mode %d\n",
			    __func__, sc->sc_flags.status_device_mode));

			if (sc->sc_flags.status_device_mode == 0) {
				/* set suspend BIT */
				sc->sc_hprt_val |= HPRT_PRTSUSP;
				DWC_OTG_WRITE_4(sc, DOTG_HPRT, sc->sc_hprt_val);

				/* generate HUB suspend event */
				dwc_otg_suspend_irq(sc);
			}
			break;

		case UHF_PORT_RESET:
			DPRINTF(("%s: UHF_PORT_RESET device_mode %d\n",
			    __func__, sc->sc_flags.status_device_mode));
			if (sc->sc_flags.status_device_mode == 0) {

				/* enable PORT reset */
				DWC_OTG_WRITE_4(sc, DOTG_HPRT,
				    sc->sc_hprt_val | HPRT_PRTRST);

				/* Wait 62.5ms for reset to complete */
				usb_delay_ms(&sc->sc_bus, 63);

				DWC_OTG_WRITE_4(sc, DOTG_HPRT, sc->sc_hprt_val);

				/* Wait 62.5ms for reset to complete */
				usb_delay_ms(&sc->sc_bus, 63);

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
			DPRINTF(("%s: UHF_PORT_POWER mode %d\n",
			   __func__, sc->sc_mode));

			if (sc->sc_mode == DWC_MODE_HOST ||
			    sc->sc_mode == DWC_MODE_OTG) {

				sc->sc_hprt_val |= HPRT_PRTPWR;
				DWC_OTG_WRITE_4(sc, DOTG_HPRT, sc->sc_hprt_val);
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
	DPRINTF(("%s\n", __func__));

	/* Nothing to do, all transfers are synchronous. */
}

Static void
dwc_otg_root_ctrl_close(usbd_pipe_handle pipe)
{
	DPRINTF(("%s\n", __func__));

	/* Nothing to do. */
}

Static void
dwc_otg_root_ctrl_done(usbd_xfer_handle xfer)
{
	DPRINTF(("%s\n", __func__));

	/* Nothing to do. */
}

Static usbd_status
dwc_otg_root_intr_transfer(usbd_xfer_handle xfer)
{
	dwc_otg_softc_t *sc = xfer->pipe->device->bus->hci_private;
	usbd_status err;

	DPRINTF(("%s\n", __func__));

	/* Insert last in queue. */
	mutex_enter(&sc->sc_lock);
	err = usb_insert_transfer(xfer);
	mutex_exit(&sc->sc_lock);
	if (err)
		return (err);

	/* Pipe isn't running, start first */
	return (dwc_otg_root_intr_start(SIMPLEQ_FIRST(&xfer->pipe->queue)));
}

Static usbd_status
dwc_otg_root_intr_start(usbd_xfer_handle xfer)
{
	usbd_pipe_handle pipe = xfer->pipe;
	dwc_otg_softc_t *sc = pipe->device->bus->hci_private;

	DPRINTF(("%s\n", __func__));

	if (sc->sc_dying)
		return (USBD_IOERROR);

	mutex_enter(&sc->sc_lock);
	KASSERT(sc->sc_intrxfer == NULL);
	sc->sc_intrxfer = xfer;
	mutex_exit(&sc->sc_lock);

	return (USBD_IN_PROGRESS);
}

Static void
dwc_otg_root_intr_abort(usbd_xfer_handle xfer)
{
#ifdef DIAGNOSTIC
	dwc_otg_softc_t *sc = xfer->pipe->device->bus->hci_private;
#endif

	DPRINTF(("%s\n", __func__));

	KASSERT(mutex_owned(&sc->sc_lock));

	if (xfer->pipe->intrxfer == xfer) {
		DPRINTF(("dwc_otg_root_intr_abort: remove\n"));
		xfer->pipe->intrxfer = NULL;
	}
	xfer->status = USBD_CANCELLED;
	usb_transfer_complete(xfer);
}

Static void
dwc_otg_root_intr_close(usbd_pipe_handle pipe)
{
	dwc_otg_softc_t *sc = pipe->device->bus->hci_private;

	DPRINTF(("%s\n", __func__));

	KASSERT(mutex_owned(&sc->sc_lock));

	sc->sc_intrxfer = NULL;
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

	DPRINTF(("%s\n", __func__));

	dpipe = dpipe;
	sc = sc;
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

	sc = sc;
	DPRINTF(("%s\n", __func__));

	return (USBD_IN_PROGRESS);

}

Static void
dwc_otg_device_intr_abort(usbd_xfer_handle xfer)
{
	dwc_otg_softc_t *sc = xfer->pipe->device->bus->hci_private;

	sc = sc;
	DPRINTF(("%s\n", __func__));
}

Static void
dwc_otg_device_intr_close(usbd_pipe_handle pipe)
{
	struct dwc_otg_pipe *dpipe = (struct dwc_otg_pipe *)pipe;
	dwc_otg_softc_t *sc = pipe->device->bus->hci_private;

	dpipe = dpipe;
	sc = sc;
	DPRINTF(("%s\n", __func__));
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

	DPRINTF(("%s\n", __func__));

	sc = sc;
}

usbd_status
dwc_otg_device_isoc_start(usbd_xfer_handle xfer)
{
	struct dwc_otg_pipe *dpipe = (struct dwc_otg_pipe *)xfer->pipe;
	dwc_otg_softc_t *sc = dpipe->pipe.device->bus->hci_private;

	sc = sc;
	DPRINTF(("%s\n", __func__));

	return USBD_IN_PROGRESS;
}

void
dwc_otg_device_isoc_abort(usbd_xfer_handle xfer)
{
	struct dwc_otg_pipe *dpipe = (struct dwc_otg_pipe *)xfer->pipe;
	dwc_otg_softc_t *sc = dpipe->pipe.device->bus->hci_private;

	sc = sc;
	DPRINTF(("%s\n", __func__));
}

void
dwc_otg_device_isoc_done(usbd_xfer_handle xfer)
{
	DPRINTF(("%s\n", __func__));
}


usbd_status
dwc_otg_setup_isoc(usbd_pipe_handle pipe)
{
	struct dwc_otg_pipe *dpipe = (struct dwc_otg_pipe *)pipe;
	dwc_otg_softc_t *sc = pipe->device->bus->hci_private;

	dpipe = dpipe;
	sc = sc;
	DPRINTF(("%s\n", __func__));

	return USBD_NORMAL_COMPLETION;
}

void
dwc_otg_device_isoc_close(usbd_pipe_handle pipe)
{
	struct dwc_otg_pipe *dpipe = (struct dwc_otg_pipe *)pipe;
	dwc_otg_softc_t *sc = pipe->device->bus->hci_private;

	dpipe = dpipe;
	sc = sc;
	DPRINTF(("%s\n", __func__));
}

/***********************************************************************/

usbd_status
dwc_otg_init(dwc_otg_softc_t *sc)
{
	uint32_t temp;

	sc->sc_bus.hci_private = sc;
	sc->sc_bus.usbrev = USBREV_2_0;
	sc->sc_bus.methods = &dwc_otg_bus_methods;
	sc->sc_bus.pipe_size = sizeof(struct dwc_otg_pipe);

	/* XXXNH */
	sc->sc_noport = 1;

	callout_init(&sc->sc_timer, CALLOUT_MPSAFE);

	mutex_init(&sc->sc_lock, MUTEX_DEFAULT, IPL_SOFTUSB);
	mutex_init(&sc->sc_intr_lock, MUTEX_DEFAULT, IPL_SCHED);

	TAILQ_INIT(&sc->sc_intrhead);

	sc->sc_rhc_si = softint_establish(SOFTINT_NET | SOFTINT_MPSAFE,
	    dwc_otg_rhc, sc);

	usb_setup_reserve(sc->sc_dev, &sc->sc_dma_reserve, sc->sc_bus.dmatag,
		USB_MEM_RESERVE);

	temp = DWC_OTG_READ_4(sc, DOTG_GSNPSID);
	DPRINTF(("Version = 0x%08x\n", temp));

	/* disconnect */
	DWC_OTG_WRITE_4(sc, DOTG_DCTL, DCTL_SFTDISCON);
	usb_delay_ms(&sc->sc_bus, 30);
	DWC_OTG_WRITE_4(sc, DOTG_GRSTCTL, GRSTCTL_CSFTRST);
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
	DWC_OTG_WRITE_4(sc, DOTG_GUSBCFG,
		GUSBCFG_PHYIF |
		GUSBCFG_TRD_TIM_SET(5) | temp);
	DWC_OTG_WRITE_4(sc, DOTG_GOTGCTL, 0x000000ec);

	temp = DWC_OTG_READ_4(sc, DOTG_GLPMCFG);
	DWC_OTG_WRITE_4(sc, DOTG_GLPMCFG, temp & ~GLPMCFG_HSIC_CONN);
	DWC_OTG_WRITE_4(sc, DOTG_GLPMCFG, temp | GLPMCFG_HSIC_CONN);
#else
	DWC_OTG_WRITE_4(sc, DOTG_GUSBCFG,
		GUSBCFG_ULPI_UTMI_SEL |
		GUSBCFG_TRD_TIM_SET(5) | temp);
	DWC_OTG_WRITE_4(sc, DOTG_GOTGCTL, 0);

	temp = DWC_OTG_READ_4(sc, DOTG_GLPMCFG);
	DWC_OTG_WRITE_4(sc, DOTG_GLPMCFG, temp & ~GLPMCFG_HSIC_CONN);
#endif

	/* clear global nak */
	DWC_OTG_WRITE_4(sc, DOTG_DCTL, DCTL_CGOUTNAK | DCTL_CGNPINNAK);

	/* disable USB port */
	DWC_OTG_WRITE_4(sc, DOTG_PCGCCTL, 0xffffffff);
	usb_delay_ms(&sc->sc_bus, 10);

	/* enable USB port */
	DWC_OTG_WRITE_4(sc, DOTG_PCGCCTL, 0);
	usb_delay_ms(&sc->sc_bus, 10);

	/* pull up D+ */
	dwc_otg_pull_up(sc);

	temp = DWC_OTG_READ_4(sc, DOTG_GHWCFG3);
	sc->sc_fifo_size = 4 * GHWCFG3_DFIFODEPTH_GET(temp);

	temp = DWC_OTG_READ_4(sc, DOTG_GHWCFG2);
	sc->sc_dev_ep_max = min(GHWCFG2_NUMDEVEPS_GET(temp),DWC_OTG_MAX_ENDPOINTS);
	sc->sc_host_ch_max = min(GHWCFG2_NUMHSTCHNL_GET(temp),DWC_OTG_MAX_CHANNELS);

	temp = DWC_OTG_READ_4(sc, DOTG_GHWCFG4);
	sc->sc_dev_in_ep_max = GHWCFG4_NUM_IN_EP_GET(temp);

	DPRINTF(("Total FIFO size = %d bytes, Device EPs = %d/%d Host CHs = %d\n",
		sc->sc_fifo_size, sc->sc_dev_ep_max, sc->sc_dev_in_ep_max,
		sc->sc_host_ch_max));

	/* setup fifo */
	if (dwc_otg_init_fifo(sc, DWC_MODE_OTG))
		return EINVAL;

	/* enable interrupts */
	sc->sc_irq_mask = DWC_OTG_MSK_GINT_ENABLED;
	DWC_OTG_WRITE_4(sc, DOTG_GINTMSK, sc->sc_irq_mask);

	switch (sc->sc_mode) {
	case DWC_MODE_DEVICE:
	case DWC_MODE_OTG:
		/* enable endpoint interrupts */
		temp = DWC_OTG_READ_4(sc, DOTG_GHWCFG2);
		if (temp & GHWCFG2_MPI) {
			for (i = 0; i < sc->sc_dev_in_ep_max; ++i) {
				DWC_OTG_WRITE_4(sc, DOTG_DIEPEACHINTMSK(i),
					DIEPMSK_XFERCOMPLMSK);
				DWC_OTG_WRITE_4(sc, DOTG_DOEPEACHINTMSK(i), 0);
			}
			DWC_OTG_WRITE_4(sc, DOTG_DEACHINTMSK, 0xffff);
		} else {
			DWC_OTG_WRITE_4(sc, DOTG_DIEPMSK, DIEPMSK_XFERCOMPLMSK);
			DWC_OTG_WRITE_4(sc, DOTG_DOEPMSK, 0);
			DWC_OTG_WRITE_4(sc, DOTG_DAINTMSK, 0xffff);
		}
		break;
	}

	switch (sc->sc_mode) {
	case DWC_MODE_HOST:
	case DWC_MODE_OTG:
		/* setup clocks */
		offonbits(sc, DOTG_HCFG,
			HCFG_FSLSSUPP | HCFG_FSLSPCLKSEL_MASK,
			1 << HCFG_FSLSPCLKSEL_SHIFT);
		break;
	}

	/* enable global IRQ */
	DWC_OTG_WRITE_4(sc, DOTG_GAHBCFG, GAHBCFG_GLBLINTRMSK);

	temp = DWC_OTG_READ_4(sc, DOTG_GOTGCTL);
	DPRINTFN(5, ("GOTCTL=0x%08x\n", temp));

	/* read initial VBUS state */
	dwc_otg_vbus_interrupt(sc);

	/* catch any lost interrupts */
	dwc_otg_poll(&sc->sc_bus);

	return 0;
}

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
		uint32_t status = DWC_OTG_READ_4(sc, DOTG_GINTSTS);
		DWC_OTG_WRITE_4(sc, DOTG_GINTSTS, status);
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

	printf("GOTGCTL        0x%08x\n", DWC_OTG_READ_4(sc, DOTG_GOTGCTL));
	printf("GOTGINT        0x%08x\n", DWC_OTG_READ_4(sc, DOTG_GOTGINT));
	printf("GAHBCFG        0x%08x\n", DWC_OTG_READ_4(sc, DOTG_GAHBCFG));
	printf("GUSBCFG        0x%08x\n", DWC_OTG_READ_4(sc, DOTG_GUSBCFG));
	printf("GRSTCTL        0x%08x\n", DWC_OTG_READ_4(sc, DOTG_GRSTCTL));
	printf("GINTSTS        0x%08x\n", DWC_OTG_READ_4(sc, DOTG_GINTSTS));
	printf("GINTMSK        0x%08x\n", DWC_OTG_READ_4(sc, DOTG_GINTMSK));
	printf("GRXSTSR        0x%08x\n", DWC_OTG_READ_4(sc, DOTG_GRXSTSR));
	printf("GRXSTSP        0x%08x\n", DWC_OTG_READ_4(sc, DOTG_GRXSTSP));
	printf("GRXFSIZ        0x%08x\n", DWC_OTG_READ_4(sc, DOTG_GRXFSIZ));
	printf("GNPTXFSIZ      0x%08x\n", DWC_OTG_READ_4(sc, DOTG_GNPTXFSIZ));
	printf("GNPTXSTS       0x%08x\n", DWC_OTG_READ_4(sc, DOTG_GNPTXSTS));
	printf("GI2CCTL        0x%08x\n", DWC_OTG_READ_4(sc, DOTG_GI2CCTL));
	printf("GPVNDCTL       0x%08x\n", DWC_OTG_READ_4(sc, DOTG_GPVNDCTL));
	printf("GGPIO          0x%08x\n", DWC_OTG_READ_4(sc, DOTG_GGPIO));
	printf("GUID           0x%08x\n", DWC_OTG_READ_4(sc, DOTG_GUID));
	printf("GSNPSID        0x%08x\n", DWC_OTG_READ_4(sc, DOTG_GSNPSID));
	printf("GHWCFG1        0x%08x\n", DWC_OTG_READ_4(sc, DOTG_GHWCFG1));
	printf("GHWCFG2        0x%08x\n", DWC_OTG_READ_4(sc, DOTG_GHWCFG2));
	printf("GHWCFG3        0x%08x\n", DWC_OTG_READ_4(sc, DOTG_GHWCFG3));
	printf("GHWCFG4        0x%08x\n", DWC_OTG_READ_4(sc, DOTG_GHWCFG4));
	printf("GLPMCFG        0x%08x\n", DWC_OTG_READ_4(sc, DOTG_GLPMCFG));
	printf("HPTXFSIZ       0x%08x\n", DWC_OTG_READ_4(sc, DOTG_HPTXFSIZ));

	n = GHWCFG4_NUMDEVPERIOEPS_GET(DWC_OTG_READ_4(sc, DOTG_GHWCFG4));
	for (i=1; i<n; ++i) {
		printf("DPTXFSIZ[%2d]  0x%08x\n", i,
			DWC_OTG_READ_4(sc, DOTG_DPTXFSIZ(i)));
	}

	printf("PCGCCTL        0x%08x\n", DWC_OTG_READ_4(sc, DOTG_PCGCCTL));


}

void
dwc_otg_dump_host_regs(dwc_otg_softc_t *sc)
{
	int i, n;

	printf("HCFG           0x%08x\n", DWC_OTG_READ_4(sc, DOTG_HCFG));
	printf("HFIR           0x%08x\n", DWC_OTG_READ_4(sc, DOTG_HFIR));
	printf("HFNUM          0x%08x\n", DWC_OTG_READ_4(sc, DOTG_HFNUM));
	printf("HPTXSTS        0x%08x\n", DWC_OTG_READ_4(sc, DOTG_HPTXSTS));
	printf("HAINT          0x%08x\n", DWC_OTG_READ_4(sc, DOTG_HAINT));
	printf("HAINTMSK       0x%08x\n", DWC_OTG_READ_4(sc, DOTG_HAINTMSK));
	printf("HFLBADDR       0x%08x\n", DWC_OTG_READ_4(sc, DOTG_HFLBADDR));
	printf("HPRT           0x%08x\n", DWC_OTG_READ_4(sc, DOTG_HPRT));

	n = GHWCFG2_NUMHSTCHNL_GET(DWC_OTG_READ_4(sc, DOTG_GHWCFG2));
	for (i=0; i<n; ++i) {
		printf("Host Channel %d Specific Registers\n", i);
		printf("HCCHAR         0x%08x\n", DWC_OTG_READ_4(sc,DOTG_HCCHAR(i)));
		printf("HCSPLT         0x%08x\n", DWC_OTG_READ_4(sc,DOTG_HCSPLT(i)));
		printf("HCINT          0x%08x\n", DWC_OTG_READ_4(sc,DOTG_HCINT(i)));
		printf("HCINTMSK       0x%08x\n", DWC_OTG_READ_4(sc,DOTG_HCINTMSK(i)));
		printf("HCTSIZ         0x%08x\n", DWC_OTG_READ_4(sc,DOTG_HCTSIZ(i)));
		printf("HCDMA          0x%08x\n", DWC_OTG_READ_4(sc,DOTG_HCDMA(i)));
		printf("HCDMAB         0x%08x\n", DWC_OTG_READ_4(sc,DOTG_HCDMAB(i)));
	}


}
#endif

/***********************************************************************/

Static void
dwc_otg_pull_up(struct dwc_otg_softc *sc)
{
	uint32_t temp;

	/* pullup D+, if possible */

	if (!sc->sc_flags.d_pulled_up &&
	    sc->sc_flags.port_powered) {
		sc->sc_flags.d_pulled_up = 1;

		temp = DWC_OTG_READ_4(sc, DOTG_DCTL);
		temp &= ~DCTL_SFTDISCON;
		DWC_OTG_WRITE_4(sc, DOTG_DCTL, temp);
	}
}

Static void
dwc_otg_pull_down(struct dwc_otg_softc *sc)
{
	uint32_t temp;

	/* pulldown D+, if possible */

	if (sc->sc_flags.d_pulled_up) {
		sc->sc_flags.d_pulled_up = 0;

		temp = DWC_OTG_READ_4(sc, DOTG_DCTL);
		temp |= DCTL_SFTDISCON;
		DWC_OTG_WRITE_4(sc, DOTG_DCTL, temp);
	}
}

Static void
dwc_otg_clocks_on(dwc_otg_softc_t* sc)
{
	if (sc->sc_flags.clocks_off &&
	    sc->sc_flags.port_powered) {

		DPRINTFN(5, ("\n"));

		/* TODO - platform specific */

		sc->sc_flags.clocks_off = 0;
	}
}

Static void
dwc_otg_clocks_off(dwc_otg_softc_t* sc)
{
	if (!sc->sc_flags.clocks_off) {

		DPRINTFN(5, ("\n"));

		/* TODO - platform specific */

		sc->sc_flags.clocks_off = 1;
	}
}

static void
dwc_otg_common_rx_ack(struct dwc_otg_softc *sc)
{
	DPRINTFN(5, ("RX status clear\n"));

	/* enable RX FIFO level interrupt */
	sc->sc_irq_mask |= GINTSTS_RXFLVL;
	DWC_OTG_WRITE_4(sc, DOTG_GINTMSK, sc->sc_irq_mask);

#ifdef notyet
	/* clear cached status */
	sc->sc_last_rx_status = 0;
#endif
}

Static void
dwc_otg_timer(void *_sc)
{
	struct dwc_otg_softc *sc = _sc;
	struct dwc_otg_xfer *xfer;
	struct dwc_otg_soft_td *td;

	KASSERT(mutex_owned(&sc->sc_lock));

	DPRINTF(("\n"));

	/* increment timer value */
	sc->sc_tmr_val++;

	TAILQ_FOREACH(xfer, &sc->sc_intrhead, inext) {
		td = xfer->td;
		if (td != NULL && td->td != NULL)
			td->td->did_nak = 0;
	}

	/* poll jobs */
	dwc_otg_interrupt_poll(sc);

	if (sc->sc_timer_active) {
		/* restart timer */
		callout_reset(&sc->sc_timer,
		    hz / (1000 / DWC_OTG_HOST_TIMER_RATE),
		    &dwc_otg_timer, sc);
	}
}

Static void
dwc_otg_timer_start(struct dwc_otg_softc *sc)
{
	if (sc->sc_timer_active != 0)
		return;

	sc->sc_timer_active = 1;

	/* restart timer */
	callout_reset(&sc->sc_timer,
	    hz / (1000 / DWC_OTG_HOST_TIMER_RATE),
	    &dwc_otg_timer, sc);
}

Static void
dwc_otg_timer_stop(struct dwc_otg_softc *sc)
{
	if (sc->sc_timer_active == 0)
		return;

	sc->sc_timer_active = 0;

	/* stop timer */
	callout_stop(&sc->sc_timer);
}

Static void
dwc_otg_suspend_irq(struct dwc_otg_softc *sc)
{
	if (!sc->sc_flags.status_suspend) {
		/* update status bits */
		sc->sc_flags.status_suspend = 1;
		sc->sc_flags.change_suspend = 1;

		if (sc->sc_flags.status_device_mode) {
			/*
			 * Disable suspend interrupt and enable resume
			 * interrupt:
			 */
			sc->sc_irq_mask &= ~GINTSTS_USBSUSP;
			sc->sc_irq_mask |= GINTSTS_WKUPINT;
			DWC_OTG_WRITE_4(sc, DOTG_GINTMSK, sc->sc_irq_mask);
		}

		/* complete root HUB interrupt endpoint */
		softint_schedule(sc->sc_rhc_si);
	}
}

Static void
dwc_otg_resume_irq(struct dwc_otg_softc *sc)
{
	if (sc->sc_flags.status_suspend) {
		/* update status bits */
		sc->sc_flags.status_suspend = 0;
		sc->sc_flags.change_suspend = 1;

		if (sc->sc_flags.status_device_mode) {
			/*
			 * Disable resume interrupt and enable suspend
			 * interrupt:
			 */
			sc->sc_irq_mask &= ~GINTSTS_WKUPINT;
			sc->sc_irq_mask |= GINTSTS_USBSUSP;
			DWC_OTG_WRITE_4(sc, DOTG_GINTMSK, sc->sc_irq_mask);
		}

		/* complete root HUB interrupt endpoint */
		softint_schedule(sc->sc_rhc_si);
	}
}

Static void
dwc_otg_wakeup_peer(struct dwc_otg_softc *sc)
{
	if (!sc->sc_flags.status_suspend)
		return;

	DPRINTFN(5, ("Remote wakeup\n"));

	if (sc->sc_flags.status_device_mode) {
		uint32_t temp;

		/* enable remote wakeup signalling */
		temp = DWC_OTG_READ_4(sc, DOTG_DCTL);
		temp |= DCTL_RMTWKUPSIG;
		DWC_OTG_WRITE_4(sc, DOTG_DCTL, temp);

		/* Wait 8ms for remote wakeup to complete. */
		usb_delay_ms_locked(&sc->sc_bus, 8, &sc->sc_lock);

		temp &= ~DCTL_RMTWKUPSIG;
		DWC_OTG_WRITE_4(sc, DOTG_DCTL, temp);
	} else {
		/* enable USB port */
		DWC_OTG_WRITE_4(sc, DOTG_PCGCCTL, 0);

		/* wait 10ms */
		usb_delay_ms_locked(&sc->sc_bus, 10, &sc->sc_lock);

		/* resume port */
		sc->sc_hprt_val |= HPRT_PRTRES;
		DWC_OTG_WRITE_4(sc, DOTG_HPRT, sc->sc_hprt_val);

		/* Wait 100ms for resume signalling to complete. */
		usb_delay_ms_locked(&sc->sc_bus, 100, &sc->sc_lock);

		/* clear suspend and resume */
		sc->sc_hprt_val &= ~(HPRT_PRTSUSP | HPRT_PRTRES);
		DWC_OTG_WRITE_4(sc, DOTG_HPRT, sc->sc_hprt_val);

		/* Wait 4ms */
		usb_delay_ms_locked(&sc->sc_bus, 4, &sc->sc_lock);
	}

	/* need to fake resume IRQ */
	dwc_otg_resume_irq(sc);
}

#if 0
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

Static void
dwc_otg_start_dma(...)
{
	KASSERT(tdbuf is properly aligned);
	KASSERT(size doesnt span 2K pages);

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

	DWC_OTG_WRITE_4(sc, DOTG_HCTSIZ(ch),
		dopng
		| pid << HCTSIZ_PID_SHIFT
		| (numtd-1) << HCTSIZ_NTD_SHIFT
		| schinfo << HCTSIZ_SCHINFO_SHIFT);
	DWC_OTG_WRITE_4(sc, DOTG_HCDMA(ch),
		(uint32_t)(uintptr_t)td);
	offonbits(sc, DOTG_HCCHAR(ch),
		HCCHAR_MC_MASK | HCCHAR_CHDIS,
		mc << HCCHAR_MC_SHIFT | HCCHAR_CHENA);
}

Static void
dwc_otg_set_address(struct dwc_otg_softc *sc, uint8_t addr)
{
	offonbits(sc, DOTG_DCFG,
		DCFG_DEVADDR_SET(0x7f),
		DCFG_DEVADDR_SET(addr));
}
#endif

/***********************************************************************/

int
dwc_otg_intr1(dwc_otg_softc_t *sc)
{
	uint32_t status;

	status = DWC_OTG_READ_4(sc, DOTG_GINTSTS);
	DWC_OTG_WRITE_4(sc, DOTG_GINTSTS, status);

	if (status & GINTSTS_USBRST) {
		/* USB reset */
		// "root_intr" ?
	}

	/* check for any bus state change interrupts */
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
		sc->sc_hprt_val = DWC_OTG_READ_4(sc, DOTG_HPRT);
		DWC_OTG_WRITE_4(sc, DOTG_HPRT, (hprt & (
			HPRT_PRTPWR | HPRT_PRTENCHNG |
			HPRT_PRTCONNDET | HPRT_PRTOVRCURRCHNG)) |
			sc->sc_hprt_val);

		if (sc->sc_hprt_val & HPRT_PRTSUSP)
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
		dwc_otg_vbus_interrupt(sc);
	}

	if (status & GINTSTS_IEPINT) {
		/* clear all IN endpoint interrupts */
	}

	if (status & GINTSTS_SOF) {
		if (sc->sc_irq_mask & GINTMSK_SOFMSK) {
			/*
			 * XXX TODO
			search for channel that is waiting for SOF
			if none found
				disable GINTMSK_SOFMSK
			 */
		}
	}

	/* poll FIFOs */
	dwc_otg_interrupt_poll(sc);
}

Static void
dwc_otg_interrupt_poll(dwc_otg_softc_t *sc)
{
	uint8_t ch, epno;
	uint32_t rx, temp, bcnt, intrs;

repeat:
	for (ch = 0; ch < sc->sc_host_ch_max; ++ch) {
		intrs = DWC_OTG_READ_4(sc, DOTG_HCINT(ch));
		DWC_OTG_WRITE_4(sc, DOTG_HCINT(ch), intrs);
	}

	rx = 0;
	temp = DWC_OTG_READ_4(sc, DOTG_GINTSTS);
	if (temp & GINTSTS_RXFLVL)
		rx = DWC_OTG_READ_4(sc, DOTG_GRXSTSPD);

	switch (rx & GRXSTSRD_PKTSTS_MASK) {
	case GRXSTSRD_STP_DATA:
	case GRXSTSRD_OUT_DATA:
		bcnt = GRXSTSRD_BCNT_GET(rx);
		epno = GRXSTSRD_CHNUM_GET(rx);

		if (bcnt) {
			/* read bytes from fifo */
			bus_space_read_region_4(sc->sc_iot, sc->sc_ioh,
				DOTG_DFIFO(epno),
				sc->sc_rx_bounce_buffer, (bcnt+3)/4);
		}
		/* check if we should dump the data */
		if (!(sc->sc_active_rx_ep & (1U << epno))) {
			dwc_otg_common_rx_ack(sc);
			goto repeat;
		}
		break;
	default:
		epno = GRXSTSRD_CHNUM_GET(rx);

		if (!(sc->sc_active_rx_ep & (1U << epno))) {
			dwc_otg_common_rx_ack(sc);
			goto repeat;
		}
		break;
	}

#ifdef notyet
	poll transfers
	if ("queue has been modified") goto repeat
#endif

	if (rx == 0)
		goto repeat;

	/* disable RX FIFO level interrupt */
	sc->sc_irq_mask &= ~GINTSTS_RXFLVL;
	DWC_OTG_WRITE_4(sc, DOTG_GINTMSK, sc->sc_irq_mask);
}

/* Must be called with the lock helded */
Static void
dwc_otg_rhc(void *addr)
{
	struct dwc_otg_softc *sc = addr;
	usbd_xfer_handle xfer;
	usbd_pipe_handle pipe;
	u_char *p;

	mutex_enter(&sc->sc_lock);

	xfer = sc->sc_intrxfer;
	if (xfer == NULL) {
		/* Just ignore the change. */
		mutex_exit(&sc->sc_lock);
		return;
	}

	/* set port bit */
	pipe = xfer->pipe;

	p = KERNADDR(&xfer->dmabuf, 0);
	p[0]= 0x02; /* we only have one port */

	xfer->actlen = xfer->length;
	xfer->status = USBD_NORMAL_COMPLETION;

	usb_transfer_complete(xfer);
	mutex_exit(&sc->sc_lock);
}

Static void
dwc_otg_vbus_interrupt(struct dwc_otg_softc *sc)
{
	uint32_t temp;

	temp = DWC_OTG_READ_4(sc, DOTG_GOTGCTL);

	if (temp & (GOTGCTL_ASESVLD | GOTGCTL_BSESVLD)) {
		if (!sc->sc_flags.status_vbus) {
			sc->sc_flags.status_vbus = 1;
			/* complete root HUB interrupt endpoint */
			softint_schedule(sc->sc_rhc_si);
		}
	} else {
		if (sc->sc_flags.status_vbus) {
			sc->sc_flags.status_vbus = 0;
			sc->sc_flags.status_bus_reset = 0;
			sc->sc_flags.status_suspend = 0;
			sc->sc_flags.change_suspend = 0;
			sc->sc_flags.change_connect = 1;
			/* complete root HUB interrupt endpoint */
			softint_schedule(sc->sc_rhc_si);
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

	DWC_OTG_WRITE_4(sc, DOTG_GRXFSIZ, fifo_size / 4);

	/* align to 4-bytes */
	fifo_size &= ~3;

	tx_start = fifo_size;

	if (fifo_size < 0x40) {
		DPRINTFN(-1, ("Not enough data space for EP0 FIFO.\n"));
		return EINVAL;
	}

	if (mode == DWC_MODE_HOST) {

		/* reset active endpoints */
		sc->sc_active_rx_ep = 0;

		fifo_size /= 2;

		DWC_OTG_WRITE_4(sc, DOTG_GNPTXFSIZ,
		    ((fifo_size / 4) << 16) | (tx_start / 4));

		tx_start += fifo_size;

		DWC_OTG_WRITE_4(sc, DOTG_HPTXFSIZ,
		    ((fifo_size / 4) << 16) | (tx_start / 4));

		for (x = 0; x != sc->sc_host_ch_max; x++) {
			/* enable interrupts */
			DWC_OTG_WRITE_4(sc, DOTG_HCINTMSK(x),
			    HCINT_STALL | HCINT_BBLERR |
			    HCINT_XACTERR |
			    HCINT_NAK | HCINT_ACK | HCINT_NYET |
			    HCINT_CHHLTD | HCINT_FRMOVRUN |
			    HCINT_DATATGLERR);
		}

		/* enable host channel interrupts */
		DWC_OTG_WRITE_4(sc, DOTG_HAINTMSK,
		    (1U << sc->sc_host_ch_max) - 1U);

	}

#ifdef notyet
	if (mode == DWC_MODE_DEVICE) {

	    DWC_OTG_WRITE_4(sc, DOTG_GNPTXFSIZ,
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
				DWC_OTG_WRITE_4(sc, DOTG_DIEPTXF(x),
				    ((limit / 4) << 16) |
				    (tx_start / 4));
				tx_start += limit;
				fifo_size -= limit;
				pf->usb.max_in_frame_size = 0x200;
				pf->usb.support_in = 1;
				pf->max_buffer = limit;

			} else if (fifo_size >= 0x80) {
				DWC_OTG_WRITE_4(sc, DOTG_DIEPTXF(x),
				    ((0x80 / 4) << 16) | (tx_start / 4));
				tx_start += 0x80;
				fifo_size -= 0x80;
				pf->usb.max_in_frame_size = 0x40;
				pf->usb.support_in = 1;

			} else {
				pf->usb.is_simplex = 1;
				DWC_OTG_WRITE_4(sc, DOTG_DIEPTXF(x),
				    (0x0 << 16) | (tx_start / 4));
			}
		} else {
			pf->usb.is_simplex = 1;
		}

		DPRINTF(("FIFO%d = IN:%d / OUT:%d\n", x,
		    pf->usb.max_in_frame_size,
		    pf->usb.max_out_frame_size));
	    }
	}
#endif

	/* reset RX FIFO */
	DWC_OTG_WRITE_4(sc, DOTG_GRSTCTL, GRSTCTL_RXFFLSH);

	if (mode != DWC_MODE_OTG) {
		/* reset all TX FIFOs */
		DWC_OTG_WRITE_4(sc, DOTG_GRSTCTL,
		    GRSTCTL_TXFIFO(0x10) | GRSTCTL_TXFFLSH);
	} else {
		/* reset active endpoints */
		sc->sc_active_rx_ep = 0;
	}

	return 0;
}

