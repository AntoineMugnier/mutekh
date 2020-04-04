/*
  This file is part of MutekH.

  MutekH is free software; you can redistribute it and/or modify it
  under the terms of the GNU Lesser General Public License as
  published by the Free Software Foundation; version 2.1 of the
  License.

  MutekH is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this program.  If not, see
  <http://www.gnu.org/licenses/>.

  Copyright (c) Nicolas Pouillon <nipo@ssji.net>, 2016
*/

#define LOGK_MODULE_ID "nusb"

//#define ERRATA_104
#define ERRATA_166
#define ERRATA_171
#define ERRATA_187
#define ERRATA_199

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>
#include <mutek/bytecode.h>
#include <mutek/kroutine.h>

#include <hexo/bit.h>
#include <hexo/endian.h>
#include <hexo/iospace.h>

#include <device/resources.h>
#include <device/device.h>
#include <device/driver.h>
#include <device/clock.h>
#include <device/irq.h>

#include <string.h>

#include <arch/nrf5x/ids.h>
#include <arch/nrf5x/peripheral.h>
#include <arch/nrf5x/usbd.h>
#include <arch/nrf5x/usbd_intbus.h>

#include <device/class/usbdev.h>
#include <device/usb/usb.h>

#define NRF5X_EP_COUNT 8

struct nrf5x_usb_private_s
{
  struct kroutine_s update;
  struct kroutine_s irq_handler;

  uintptr_t addr;
  uint32_t irq_pending;
  bool_t wanted;
  bool_t vbus_present;
  bool_t reg_ready;
  bool_t hfclk_ok;
  bool_t started;
  bool_t connected;
  bool_t setup_pending;
  uint8_t event;

  uint8_t tx_running;
  uint8_t dma_rx_running;
  uint8_t data_rx_pending;

  struct dev_irq_src_s irq_ep;
  struct dev_clock_sink_ep_s clock_sink[NRF5X_USBD_CLK_COUNT];
  struct usbdev_endpoint_s ep[NRF5X_EP_COUNT - 1][2];

  struct dev_usbdev_context_s usbdev_ctx;

  struct dev_usbdev_config_s *cfg;
  struct dev_usbdev_rq_s *tr[NRF5X_EP_COUNT][2];
  uint8_t mps[NRF5X_EP_COUNT][2];
};

DRIVER_PV(struct nrf5x_usb_private_s);

static uint_fast8_t nrf5x_usbd_intbus_read(uintptr_t addr, uintptr_t intaddr)
{
  cpu_mem_write_32(addr + USBD_INTBUS_ADDR_ADDR, intaddr);
  return cpu_mem_read_32(addr + USBD_INTBUS_DATA_ADDR);
}

static uint_fast8_t nrf5x_usbd_intbus_read2(uintptr_t addr, uintptr_t intaddr)
{
  uint_fast8_t ret = nrf5x_usbd_intbus_read(addr, intaddr);
  if (ret)
    ret &= cpu_mem_read_32(addr + USBD_INTBUS_DATA_ADDR);
  return ret;
}

static void nrf5x_usbd_intbus_write(uintptr_t addr, uintptr_t intaddr, uint_fast8_t data)
{
  cpu_mem_write_32(addr + USBD_INTBUS_ADDR_ADDR, intaddr);
  cpu_mem_write_32(addr + USBD_INTBUS_DATA_ADDR, data);
}

static void dma_start(struct nrf5x_usb_private_s *pv, uintptr_t off, uint_fast8_t end)
{
#if defined(ERRATA_104) && 0
  do {
    for (;;) {
      nrf_event_clear(pv->addr, USBD_INTEN_STARTED_SHIFT);
      cpu_mem_write_32(pv->addr + off, 1);

      for (uint_fast8_t i = 32; i; --i)
        asm volatile("");

      if (nrf_event_check(pv->addr, USBD_INTEN_STARTED_SHIFT)) {
        nrf_event_clear(pv->addr, USBD_INTEN_STARTED_SHIFT);
        return;
      }
    }

    logk_trace("Grumpf");

    while ((cpu_mem_read_32(pv->addr + USBD_EP0CS_ADDR) & USBD_EP0CS_CHGSET) == 0) {
      for (uint_fast8_t i = 32; i; --i)
        asm volatile("");
    }
  } while (!nrf_event_check(pv->addr, end));
#else
  cpu_mem_write_32(pv->addr + off, 1);
#endif
}

static void nrf5x_usbd_tx(struct nrf5x_usb_private_s *pv,
                              struct dev_usbdev_rq_s *tr)
{
  logk_trace("DMA+Tx to IN/EP%d (%d bytes): %P", tr->ep, tr->size, tr->data, tr->size);

  assert(dev_usbdev_get_transfer_dir(tr) == USB_DEVICE_TO_HOST);
  assert(bit_get(cpu_mem_read_32(pv->addr + USBD_EPINEN_ADDR), tr->ep));
  
  cpu_mem_write_32(pv->addr + USBD_EPDATASTATUS_ADDR, bit(tr->ep));
  cpu_mem_write_32(pv->addr + USBD_EPSTATUS_ADDR, bit(tr->ep));
  cpu_mem_write_32(pv->addr + USBD_EPIN_PTR_ADDR(tr->ep), (uintptr_t)tr->data);
  cpu_mem_write_32(pv->addr + USBD_EPIN_MAXCNT_ADDR(tr->ep), __MIN(tr->size, pv->mps[tr->ep][1]));
  nrf_it_enable(pv->addr, USBD_INTEN_EPDATA_SHIFT);
  if (tr->ep == 0) {
    nrf_it_enable(pv->addr, USBD_INTEN_EP0DATADONE_SHIFT);
  }

  pv->tx_running |= bit(tr->ep);

  dma_start(pv, USBD_TASKS_STARTEPIN_ADDR(tr->ep), USBD_INTEN_ENDEPIN_SHIFT(tr->ep));
#ifdef ERRATA_199
  *(volatile uint32_t *)0x40027c1c = 0x00000082;
#endif

  kroutine_exec(&pv->irq_handler);
}

static void nrf5x_usbd_rx_dma(struct nrf5x_usb_private_s *pv,
                              struct dev_usbdev_rq_s *tr)
{
  size_t epsize = cpu_mem_read_32(pv->addr + USBD_SIZE_EPOUT_ADDR(tr->ep));

  logk_trace("DMA from OUT/EP%d buffer, tr size %d, current ep size: %d, outcs: %02x",
             tr->ep, tr->size,
             epsize,
             nrf5x_usbd_intbus_read(pv->addr, 0x7c6 + 2 * tr->ep));

  assert(dev_usbdev_get_transfer_dir(tr) == USB_HOST_TO_DEVICE);

  cpu_mem_write_32(pv->addr + USBD_EPOUT_PTR_ADDR(tr->ep), (uintptr_t)tr->data);
  cpu_mem_write_32(pv->addr + USBD_EPOUT_MAXCNT_ADDR(tr->ep), __MIN(tr->size, epsize));
  nrf_it_enable(pv->addr, USBD_INTEN_ENDEPOUT_SHIFT(tr->ep));
  
  pv->dma_rx_running |= bit(tr->ep);

  dma_start(pv, USBD_TASKS_STARTEPOUT_ADDR(tr->ep), USBD_INTEN_ENDEPOUT_SHIFT(tr->ep));
#ifdef ERRATA_199
  *(volatile uint32_t *)0x40027c1c = 0x00000082;
#endif
}

static void nrf5x_usbd_rx_prepare(struct nrf5x_usb_private_s *pv,
                                  struct dev_usbdev_rq_s *tr)
{
  logk_trace("Rx from OUT/EP%d prepare (%d) current ep size: %d, outcs: %02x",
             tr->ep, tr->size,
             cpu_mem_read_32(pv->addr + USBD_SIZE_EPOUT_ADDR(tr->ep)),
             nrf5x_usbd_intbus_read(pv->addr, 0x7c6 + 2 * tr->ep));

  assert(dev_usbdev_get_transfer_dir(tr) == USB_HOST_TO_DEVICE);
  assert(bit_get(cpu_mem_read_32(pv->addr + USBD_EPOUTEN_ADDR), tr->ep));

  cpu_mem_write_32(pv->addr + USBD_EPDATASTATUS_ADDR, bit(tr->ep + 16));
  cpu_mem_write_32(pv->addr + USBD_EPSTATUS_ADDR, bit(tr->ep + 16));
  if (tr->ep) {
    nrf_it_enable(pv->addr, USBD_INTEN_EPDATA_SHIFT);
    //    cpu_mem_write_32(pv->addr + USBD_SIZE_EPOUT_ADDR(tr->ep), 0);
  } else {
    cpu_mem_write_32(pv->addr + USBD_TASKS_EP0RCVOUT_ADDR, 1);
    nrf_it_enable(pv->addr, USBD_INTEN_EP0DATADONE_SHIFT);
  }

  pv->data_rx_pending |= bit(tr->ep);

  cpu_mem_write_32(pv->addr + USBD_EPOUT_PTR_ADDR(tr->ep), (uintptr_t)tr->data);
  cpu_mem_write_32(pv->addr + USBD_EPOUT_MAXCNT_ADDR(tr->ep), 0);

  nrf5x_usbd_intbus_write(pv->addr, 0x7c5 + 2 * tr->ep, 64 - pv->mps[tr->ep][0]);

  logk_trace("Rx from OUT/EP%d prepared, ep size: %d, outcs: %02x",
             tr->ep,
             cpu_mem_read_32(pv->addr + USBD_SIZE_EPOUT_ADDR(tr->ep)),
             nrf5x_usbd_intbus_read(pv->addr, 0x7c6 + 2 * tr->ep));

  kroutine_exec(&pv->irq_handler);
}

static DEV_USBDEV_REQUEST(nrf5x_usb_transfer)
{
  struct device_s *dev = ctx->dev;
  struct nrf5x_usb_private_s *pv = dev->drv_pv;
  error_t err = -EAGAIN;

  if (tr->ep == 0 && pv->event) {
    err = -EIO;
    logk_trace("request returned event %N", pv->event, ENUM_DESC_DEV_USBDEV_EVENT_E);
    tr->event = pv->event;
    pv->event = 0;
    goto end;
  }

  bool_t in = dev_usbdev_get_transfer_dir(tr) == USB_DEVICE_TO_HOST;

  logk_trace("Transfer rq type %N on %s/EP%d", tr->type, ENUM_DESC_DEV_USBDEV_RQ_TYPE_E,
             in ? "IN" : "OUT", tr->ep);

  assert(pv->tr[tr->ep][in] == NULL);

  switch (tr->type) {
  case DEV_USBDEV_CTRL_SETUP:
    if (pv->setup_pending) {
      logk_trace("Servicing pending setup");
      pv->setup_pending = 0;
      for (uint_fast8_t i = 0; i < 8; ++i)
        ((uint8_t *)tr->data)[i] = cpu_mem_read_32(pv->addr + USBD_BMREQUESTTYPE_ADDR + i * 4);

      return 0;
    }
    logk_trace("No ctrl setup waiting");
    break;

  case DEV_USBDEV_DATA_IN:
  case DEV_USBDEV_PARTIAL_DATA_IN:
    logk_trace("IN/EP%d (%d bytes): %P", tr->ep, tr->size, tr->data, tr->size);

    nrf5x_usbd_tx(pv, tr);
    break;

  case DEV_USBDEV_PARTIAL_DATA_OUT:
  case DEV_USBDEV_DATA_OUT:
    logk_trace("OUT/EP%d (%d bytes)", tr->ep, tr->size);

    nrf5x_usbd_rx_prepare(pv, tr);
    break;

  case DEV_USBDEV_CTRL_STATUS_OUT:
    logk_trace("STATUS out");

    if (pv->data_rx_pending & bit(0))
      nrf_short_enable_mask(pv->addr, USBD_SHORTS_EP0DATADONE_EP0STATUS);
    else
      cpu_mem_write_32(pv->addr + USBD_TASKS_EP0STATUS_ADDR, 1);

    return 0;

  case DEV_USBDEV_CTRL_STATUS_IN:
    logk_trace("STATUS in");

    cpu_mem_write_32(pv->addr + USBD_TASKS_EP0STATUS_ADDR, 1);
    return 0;

  case DEV_USBDEV_CTRL_STATUS_OUT_STALL:
  case DEV_USBDEV_CTRL_STATUS_IN_STALL:
  case DEV_USBDEV_DATA_IN_STALL:
  case DEV_USBDEV_DATA_OUT_STALL:
    logk_trace("STALL %s %d", in ? "in" : "out", tr->ep);

    cpu_mem_write_32(pv->addr + USBD_TASKS_EP0STALL_ADDR, 1);
    return 0;

  case DEV_USBDEV_EVENT:
    break;
  }

  pv->tr[tr->ep][in] = tr;

  logk_trace("request pushed for ep %d", tr->ep);

 end:
  return err;
}

static DEV_USBDEV_ENDPOINT(nrf5x_usb_endpoint)
{
  struct device_s *dev = ctx->dev;
  struct nrf5x_usb_private_s *pv = dev->drv_pv;
  struct usbdev_endpoint_s *ep;

  if (address > NRF5X_EP_COUNT)
    return NULL;

  if (dir == USB_EP_OUT)
    ep = &pv->ep[address - 1][0];
  else
    ep = &pv->ep[address - 1][1];

  return ep;
}

static error_t nrf5x_usbd_configure(struct nrf5x_usb_private_s *pv,
                                    const struct dev_usbdev_config_s *cfg)
{
  for (uint_fast8_t i = 1; i < NRF5X_EP_COUNT; ++i) {
    pv->mps[i][0] = 0;
    pv->mps[i][1] = 0;
  }

  uint32_t epout_en = 1;
  uint32_t epin_en = 1;
  
  for (uint8_t i = 0; i< CONFIG_USBDEV_MAX_INTERFACE_COUNT; i++) {
    const struct dev_usbdev_interface_cfg_s *icfg = &cfg->intf[i];
    const struct usbdev_interface_s *intf = icfg->i;

    if (intf == NULL)
      break;
    
    USBDEV_FOREACH_ENDPOINT(intf, icfg->epi, icfg->epo, {
        if (usb_ep_type_get(epdesc) == USB_EP_CONTROL)
          return -ENOTSUP;

        if (usb_ep_is_in(epdesc)) {
          pv->mps[epaddr][1] = usb_ep_mps_get(epdesc);
          epin_en |= bit(epaddr);

          cpu_mem_write_32(pv->addr + USBD_DTOGGLE_ADDR, 0
                           | USBD_DTOGGLE_EP_SHIFT_VAL(epaddr)
                           | USBD_DTOGGLE_IO_SHIFT_VAL(IN)
                           | USBD_DTOGGLE_VALUE_SHIFT_VAL(NOP));
          cpu_mem_write_32(pv->addr + USBD_DTOGGLE_ADDR, 0
                           | USBD_DTOGGLE_EP_SHIFT_VAL(epaddr)
                           | USBD_DTOGGLE_IO_SHIFT_VAL(IN)
                           | USBD_DTOGGLE_VALUE_SHIFT_VAL(DATA0));
          cpu_mem_write_32(pv->addr + USBD_EPSTALL_ADDR, 0
                           | USBD_EPSTALL_EP_SHIFT_VAL(epaddr)
                           | USBD_EPSTALL_IO_SHIFT_VAL(IN)
                           | USBD_EPSTALL_STALL_SHIFT_VAL(RESUME));
        }

        if (usb_ep_is_out(epdesc)) {
          pv->mps[epaddr][0] = usb_ep_mps_get(epdesc);
          epout_en |= bit(epaddr);

          cpu_mem_write_32(pv->addr + USBD_DTOGGLE_ADDR, 0
                           | USBD_DTOGGLE_EP_SHIFT_VAL(epaddr)
                           | USBD_DTOGGLE_IO_SHIFT_VAL(OUT)
                           | USBD_DTOGGLE_VALUE_SHIFT_VAL(NOP));
          cpu_mem_write_32(pv->addr + USBD_DTOGGLE_ADDR, 0
                           | USBD_DTOGGLE_EP_SHIFT_VAL(epaddr)
                           | USBD_DTOGGLE_IO_SHIFT_VAL(OUT)
                           | USBD_DTOGGLE_VALUE_SHIFT_VAL(DATA0));
          cpu_mem_write_32(pv->addr + USBD_EPSTALL_ADDR, 0
                           | USBD_EPSTALL_EP_SHIFT_VAL(epaddr)
                           | USBD_EPSTALL_IO_SHIFT_VAL(OUT)
                           | USBD_EPSTALL_STALL_SHIFT_VAL(RESUME));

        }
      });
  }

  cpu_mem_write_32(pv->addr + USBD_EPINEN_ADDR, epin_en);
  cpu_mem_write_32(pv->addr + USBD_EPOUTEN_ADDR, epout_en);

  return 0;
}

static error_t nrf5x_usbd_unconfigure(struct nrf5x_usb_private_s *pv)
{
  for (uint_fast8_t i = 1; i < NRF5X_EP_COUNT; ++i) {
    nrf_it_disable(pv->addr, USBD_INTEN_ENDEPIN_SHIFT(i));
    nrf_it_disable(pv->addr, USBD_INTEN_ENDEPOUT_SHIFT(i));

    for (uint_fast8_t j = 0; j < 2; ++j) {
      if (pv->tr[i][j]) {
        pv->tr[i][j]->error = -EPIPE;
        usbdev_stack_request_done(&pv->usbdev_ctx, pv->tr[i][j]);
        pv->tr[i][j] = NULL;
      }
    }      
  }
  
  cpu_mem_write_32(pv->addr + USBD_USBADDR_ADDR, 0);
  cpu_mem_write_32(pv->addr + USBD_EPINEN_ADDR, 1);
  cpu_mem_write_32(pv->addr + USBD_EPOUTEN_ADDR, 1);

  return 0;
}

static DEV_USBDEV_CONFIG(nrf5x_usb_config)
{
  struct device_s *dev = ctx->dev;
  struct nrf5x_usb_private_s *pv = dev->drv_pv;

  error_t err = -EAGAIN;
  assert(pv->cfg == NULL);

  logk("Config %d intf %p", cfg->type, cfg->intf);

  switch (cfg->type) {
  case DEV_USBDEV_SET_ADDRESS:
    cpu_mem_write_32(pv->addr + USBD_USBADDR_ADDR, cfg->addr);
    return 0;

  case DEV_USBDEV_UNCONFIGURE:
    return nrf5x_usbd_unconfigure(pv);

  case DEV_USBDEV_CONFIGURE:
    if (!cfg->intf) {
      pv->mps[0][0] = pv->mps[0][1] = usbdev_stack_get_ep0_mps(&pv->usbdev_ctx);
      nrf_it_enable(pv->addr, USBD_INTEN_EP0SETUP_SHIFT);
      return 0;
    }

    return nrf5x_usbd_configure(pv, cfg);

#if (CONFIG_USBDEV_MAX_ALTERNATE_COUNT > 0)
  case DEV_USBDEV_CHANGE_INTERFACE:
    break;
#endif

  default:
    return -EINVAL;
  }

  return err;
}

static void nrf5x_usbd_event(struct device_s *dev, uint8_t event)
{
  struct nrf5x_usb_private_s *pv = dev->drv_pv;

  logk_trace("event, %N -> %N",
             pv->event, ENUM_DESC_DEV_USBDEV_EVENT_E,
             event, ENUM_DESC_DEV_USBDEV_EVENT_E);
  switch (event) {
  case USBDEV_EVENT_DISCONNECT:
    if (!pv->connected)
      return;
    // Erase previous events
    pv->event = event;
    pv->connected = 0;
    break;

  case USBDEV_EVENT_IDLE:
  case USBDEV_EVENT_WAKEUP:
  case USBDEV_EVENT_RESET:
    if (!pv->connected)
      return;
    pv->event |= event;
    break;

  case USBDEV_EVENT_CONNECT:
    if (pv->connected)
      return;
    pv->connected = 1;
    pv->event |= event;
    break;

  case USBDEV_EVENT_STOP:
    pv->event |= event;
    break;
  }
}

static void nrf5x_usbd_stack_event(struct nrf5x_usb_private_s *pv)
{
  struct dev_usbdev_rq_s *tr;

  if (pv->tr[0][0])
    tr = pv->tr[0][0];
  else
    tr = pv->tr[0][1];

  if (tr == NULL) {
    logk_trace("%s no tr pending", __func__);
    return;
  }

  tr->event = pv->event;
  tr->error = -EIO;

  pv->event = 0;
  pv->tr[0][0] = pv->tr[0][1] = NULL;

  return usbdev_stack_request_done(&pv->usbdev_ctx, tr);
}

static void nrf5x_usbd_ip_start(struct device_s *dev)
{
  struct nrf5x_usb_private_s *pv = dev->drv_pv;

  if (pv->started) {
    logk_trace("Already started");
    return;
  }

#if defined(ERRATA_166)
  nrf5x_usbd_intbus_write(pv->addr, USBD_INTBUS_ISO_SIZE_ADDR, 64);
#endif
  nrf5x_usbd_event(dev, USBDEV_EVENT_CONNECT);
  nrf5x_usbd_stack_event(dev->drv_pv);

  cpu_mem_write_32(pv->addr + USBD_USBPULLUP_ADDR, USBD_USBPULLUP_CONNECT);
  pv->started = 1;
}

static void nrf5x_usbd_ip_stop(struct device_s *dev)
{
  struct nrf5x_usb_private_s *pv = dev->drv_pv;

  if (!pv->started) {
    logk_trace("Not started yet");
    return;
  }

  nrf5x_usbd_event(dev, USBDEV_EVENT_DISCONNECT);
  nrf5x_usbd_stack_event(dev->drv_pv);

  cpu_mem_write_32(pv->addr + USBD_USBPULLUP_ADDR, 0);
  pv->started = 0;
}

static DEV_USE(nrf5x_usb_use)
{
  switch (op) {
  case DEV_USE_START: {
    struct device_accessor_s *accessor = param;
    struct device_s *dev = accessor->dev;
    struct nrf5x_usb_private_s *pv = dev->drv_pv;

    if (dev->start_count == 0) {
      pv->wanted = 1;
      kroutine_exec(&pv->update);
    }
    break;
  }

  case DEV_USE_STOP: {
    struct device_accessor_s *accessor = param;
    struct device_s *dev = accessor->dev;
    struct nrf5x_usb_private_s *pv = dev->drv_pv;

    if (dev->start_count == 0) {
      pv->wanted = 0;
      kroutine_exec(&pv->update);
    }
    break;
  }

  case DEV_USE_CLOCK_SINK_GATE_DONE: {
    struct dev_clock_sink_ep_s *sink = param;
    struct device_s *dev = sink->dev;
    struct nrf5x_usb_private_s *pv = dev->drv_pv;
    uint_fast8_t sink_id = sink - pv->clock_sink;

    switch (sink_id) {
    case NRF5X_USBD_CLK_HFCLK:
      break;

    case NRF5X_USBD_CLK_USB_VBUS:
      pv->vbus_present = !!(sink->src->flags & DEV_CLOCK_EP_POWER);
      kroutine_exec(&pv->update);
      break;

    case NRF5X_USBD_CLK_USB_REG:
      pv->reg_ready = !!(sink->src->flags & DEV_CLOCK_EP_POWER);
      kroutine_exec(&pv->update);
      break;
    }
    break;
  }

  case DEV_USE_CLOCK_SINK_FREQ_CHANGED: {
    struct dev_clock_notify_s *notif = param;
    struct dev_clock_sink_ep_s *sink = notif->sink;
    struct device_s *dev = sink->dev;
    struct nrf5x_usb_private_s *pv = dev->drv_pv;
    uint_fast8_t sink_id = sink - pv->clock_sink;

    switch (sink_id) {
    case NRF5X_USBD_CLK_HFCLK:
      logk_trace("HFCLK changed, acc: %d", notif->freq.acc_e);
      pv->hfclk_ok = notif->freq.acc_e < 16;
      kroutine_exec(&pv->update);
      break;

    default:
      break;
    }

    return 0;
  }

  default:
    return dev_use_generic(param, op);
  }

  return 0;
}

/*
  USB Plug
   => VBUS detect (clock driver)
   => VBUS endpoint powered

  If USBD started
   => device enable (USB IP reg) + HFCLK request
   => wait reg ready + clk ready

  Reg ready
   => VREG endpoint powered

  HFCLK ready
   => HFCLK acc changes

  All ready
   => device start, endpoint config, etc
 */
static KROUTINE_EXEC(nrf5x_usbd_update)
{
  struct nrf5x_usb_private_s *pv = KROUTINE_CONTAINER(kr, *pv, update);
  struct device_s *dev = pv->irq_ep.base.dev;
  error_t err;

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  logk_trace("%s: wanted %s, clock %s, vbus %s, vreg %s, ip %s",
         __func__,
         pv->wanted ? "OK" : "--",
         pv->hfclk_ok ? "OK" : "--",
         pv->vbus_present ? "OK" : "--",
         pv->reg_ready ? "OK" : "--",
         pv->started ? "OK" : "--");

  if (pv->wanted) {
    logk_trace("USBD Wanted");

    if (pv->vbus_present) {
      if (cpu_mem_read_32(pv->addr + USBD_ENABLE_ADDR) == 0) {
#ifdef ERRATA_187
        *(volatile uint32_t *)0x4006EC00 = 0x00009375;
        *(volatile uint32_t *)0x4006ED14 = 0x00000003;
        *(volatile uint32_t *)0x4006EC00 = 0x00009375;
#endif
#ifdef ERRATA_171
        if(*(volatile uint32_t *)0x4006EC00 == 0x00000000)
          {
            *(volatile uint32_t *)0x4006EC00 = 0x00009375;
          }
        *(volatile uint32_t *)0x4006EC14 = 0x000000C0;
        *(volatile uint32_t *)0x4006EC00 = 0x00009375;
#endif
        cpu_mem_write_32(pv->addr + USBD_ENABLE_ADDR, USBD_ENABLE_ENABLE);
        logk_trace("Enabling IP");
      }

      dev_clock_sink_throttle(&pv->clock_sink[NRF5X_USBD_CLK_HFCLK], NRF5X_USBD_MODE_RUNNING);
      err = dev_clock_sink_gate(&pv->clock_sink[NRF5X_USBD_CLK_USB_REG], DEV_CLOCK_EP_POWER);

      if (!err && !pv->reg_ready)
        logk_trace("reg became ready in time");

      pv->reg_ready = err == 0;

      if (!pv->hfclk_ok || !pv->reg_ready) {
        logk_trace("Not ready yet, bail out");
        return;
      }

      logk_trace("Starting IP...");
      nrf5x_usbd_ip_start(dev);

      return;
    } else {
      logk_trace("No USB connection detected, disabling all");

      logk_trace("Stopping IP");
      nrf5x_usbd_ip_stop(dev);

      cpu_mem_write_32(pv->addr + USBD_ENABLE_ADDR, 0);
      dev_clock_sink_gate(&pv->clock_sink[NRF5X_USBD_CLK_USB_REG], 0);
      dev_clock_sink_throttle(&pv->clock_sink[NRF5X_USBD_CLK_HFCLK], NRF5X_USBD_MODE_IDLE);
    }
  } else {
    logk_trace("USBD not Wanted, disabling all");

    logk_trace("Stopping IP first");
    nrf5x_usbd_ip_stop(dev);

    cpu_mem_write_32(pv->addr + USBD_ENABLE_ADDR, 0);
    dev_clock_sink_gate(&pv->clock_sink[NRF5X_USBD_CLK_USB_REG], 0);
    dev_clock_sink_throttle(&pv->clock_sink[NRF5X_USBD_CLK_HFCLK], NRF5X_USBD_MODE_IDLE);
  }
}

static DEV_USBDEV_ALLOC(nrf5x_usb_alloc)
{
  return mem_alloc(size, mem_scope_sys);
}

static DEV_USBDEV_FREE(nrf5x_usb_free)
{
  mem_free(ptr);
}

static DEV_IRQ_SRC_PROCESS(nrf5x_usbd_irq)
{
  struct device_s *dev = ep->base.dev;
  struct nrf5x_usb_private_s *pv = dev->drv_pv;

  LOCK_SPIN_SCOPED(&dev->lock);

  for (uint_fast8_t i = 0; i < 32; ++i) {
    if (nrf_event_check(pv->addr, i)) {
      nrf_event_clear(pv->addr, i);

      pv->irq_pending |= bit(i);
    }
  }

  kroutine_exec(&pv->irq_handler);
}

static KROUTINE_EXEC(nrf5x_usbd_irq_handler)
{
  struct nrf5x_usb_private_s *pv = KROUTINE_CONTAINER(kr, *pv, irq_handler);
  struct device_s *dev = pv->irq_ep.base.dev;
  uint32_t pending;

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);
  pending = pv->irq_pending & ~USBD_INTEN_SOF;
  pv->irq_pending = 0;

#if defined(ERRATA_104)
  uint32_t pending_before = pending;
#if 1
  // Overall ripped off Nordic's SDK
  // I dont understand the double reading thing, and the enabling of happened IRQs.
  // Looks like implementer of the quirk mixed addresses
  uint8_t in_irq = nrf5x_usbd_intbus_read2(pv->addr, USBD_INTBUS_IN_IRQ_ADDR);
  uint8_t out_irq = nrf5x_usbd_intbus_read2(pv->addr, USBD_INTBUS_OUT_IRQ_ADDR);
  uint8_t usb_irq = nrf5x_usbd_intbus_read2(pv->addr, USBD_INTBUS_USB_IRQ_ADDR);

  if (in_irq) {
    pending |= in_irq << USBD_INTEN_ENDEPIN_SHIFT(0);
    nrf5x_usbd_intbus_write(pv->addr, USBD_INTBUS_IN_IEN_ADDR, in_irq);
    logk_trace("uii: %02x (%02x)", in_irq,
               nrf5x_usbd_intbus_read(pv->addr, USBD_INTBUS_IN_IRQ_ADDR));
  }

  if (out_irq) {
    pending |= out_irq << USBD_INTEN_ENDEPOUT_SHIFT(0);
    nrf5x_usbd_intbus_write(pv->addr, USBD_INTBUS_OUT_IEN_ADDR, out_irq);
    logk_trace("uoi: %02x (%02x)", out_irq,
               nrf5x_usbd_intbus_read(pv->addr, USBD_INTBUS_OUT_IRQ_ADDR));
  }

  if (usb_irq & (USBD_INTBUS_USB_IEN_SUDAV | USBD_INTBUS_USB_IEN_URES)) {
    if (usb_irq & USBD_INTBUS_USB_IEN_SUDAV)
      pending |= USBD_INTEN_EP0SETUP;

    if (usb_irq & USBD_INTBUS_USB_IEN_URES)
      pending |= USBD_INTEN_USBRESET;

    nrf5x_usbd_intbus_write(pv->addr, USBD_INTBUS_USB_IEN_ADDR, usb_irq);
    logk_trace("usbi: %02x (%02x)", usb_irq,
               nrf5x_usbd_intbus_read(pv->addr, USBD_INTBUS_USB_IRQ_ADDR));
  }
#else
  // My version doing usual ISR servicing: check IRQs, mask with
  // enabled, ack happened ones
  uint8_t in_irq
    = nrf5x_usbd_intbus_read(pv->addr, USBD_INTBUS_IN_IRQ_ADDR);
  uint8_t out_irq
    = nrf5x_usbd_intbus_read(pv->addr, USBD_INTBUS_OUT_IRQ_ADDR);
  uint8_t usb_irq
    = nrf5x_usbd_intbus_read(pv->addr, USBD_INTBUS_USB_IRQ_ADDR);

  if (in_irq) {
    pending |= in_irq << USBD_INTEN_ENDEPIN_SHIFT(0);
    nrf5x_usbd_intbus_write(pv->addr, USBD_INTBUS_IN_IRQ_ADDR, in_irq);
    logk_trace("uii: %02x (%02x)", in_irq,
               nrf5x_usbd_intbus_read(pv->addr, USBD_INTBUS_IN_IRQ_ADDR));
  }

  if (out_irq) {
    pending |= out_irq << USBD_INTEN_ENDEPOUT_SHIFT(0);
    nrf5x_usbd_intbus_write(pv->addr, USBD_INTBUS_OUT_IRQ_ADDR, out_irq);
    logk_trace("uoi: %02x (%02x)", out_irq,
               nrf5x_usbd_intbus_read(pv->addr, USBD_INTBUS_OUT_IRQ_ADDR));
  }

  if (usb_irq & (USBD_INTBUS_USB_IEN_SUDAV | USBD_INTBUS_USB_IEN_URES)) {
    if (usb_irq & USBD_INTBUS_USB_IEN_SUDAV)
      pending |= USBD_INTEN_EP0SETUP;

    if (usb_irq & USBD_INTBUS_USB_IEN_URES)
      pending |= USBD_INTEN_USBRESET;

    nrf5x_usbd_intbus_write(pv->addr, USBD_INTBUS_USB_IRQ_ADDR, usb_irq);
    logk_trace("usbi: %02x (%02x)", usb_irq,
               nrf5x_usbd_intbus_read(pv->addr, USBD_INTBUS_USB_IRQ_ADDR));
  }
#endif

  if ((in_irq & pv->tx_running & 1) || (out_irq & pv->data_rx_pending & 1))
    pending |= USBD_INTEN_EP0DATADONE;

  if (pending_before != pending) {
    logk_trace("QUIRKS added pending %08x", pending & ~pending_before);
  }
#endif

  logk_trace("IRQ, pending: 0x%08x", pending);

  if (pending & USBD_INTEN_USBRESET) {
    pending &= ~USBD_INTEN_USBRESET;

    logk_trace("USB Reset");

    nrf5x_usbd_event(dev, USBDEV_EVENT_RESET);
  }

  if (pending & USBD_INTEN_USBEVENT) {
    pending &= ~USBD_INTEN_USBEVENT;

    uint32_t cause = cpu_mem_read_32(pv->addr + USBD_EVENTCAUSE_ADDR);

    cpu_mem_write_32(pv->addr + USBD_EVENTCAUSE_ADDR, cause);

    if (cause & USBD_EVENTCAUSE_ISOOUTCRC) {
      logk_trace("USB ISO Out crc");
    }

    if (cause & USBD_EVENTCAUSE_SUSPEND) {
      logk_trace("USB Suspend");
      nrf5x_usbd_event(dev, USBDEV_EVENT_IDLE);
    }

    if (cause & USBD_EVENTCAUSE_RESUME) {
      logk_trace("USB Resume");
      nrf5x_usbd_event(dev, USBDEV_EVENT_WAKEUP);
    }

    if (cause & USBD_EVENTCAUSE_READY) {
      logk_trace("USB Ready");
#ifdef ERRATA_187
      *(volatile uint32_t *)0x4006EC00 = 0x00009375;
      *(volatile uint32_t *)0x4006ED14 = 0x00000000;
      *(volatile uint32_t *)0x4006EC00 = 0x00009375;
#endif
#ifdef ERRARA_171
      if(*(volatile uint32_t *)0x4006EC00 == 0x00000000)
        {
          *(volatile uint32_t *)0x4006EC00 = 0x00009375;
        }
      *(volatile uint32_t *)0x4006EC14 = 0x00000000;
      *(volatile uint32_t *)0x4006EC00 = 0x00009375;
#endif
    }
  }

  if (pending & USBD_INTEN_EP0SETUP) {
    pending &= ~USBD_INTEN_EP0SETUP;
    struct dev_usbdev_rq_s *tr = pv->tr[0][0];

    logk_trace("EP0 Setup IRQ %p", tr);
    
    if (tr && tr->type == DEV_USBDEV_CTRL_SETUP) {
      pv->setup_pending = 0;

      assert(tr->type == DEV_USBDEV_CTRL_SETUP);

      pv->tr[0][0] = NULL;

      for (uint_fast8_t i = 0; i < 8; ++i)
        ((uint8_t *)tr->data)[i] = cpu_mem_read_32(pv->addr + USBD_BMREQUESTTYPE_ADDR + i * 4);

      logk_trace("EP0 Setup data: %P", tr->data, 8);

      usbdev_stack_request_done(&pv->usbdev_ctx, tr);
    } else {
      pv->setup_pending = 1;
    }
  }

  if (pending & USBD_INTEN_EP0DATADONE) {
    pending &= ~USBD_INTEN_EP0DATADONE;
    nrf_it_disable(pv->addr, USBD_INTEN_EP0DATADONE_SHIFT);

    logk_trace("EP0DATADONE");

    if ((pv->data_rx_pending & bit(0)) && pv->tr[0][0]) {
      pv->data_rx_pending &= ~bit(0);

      logk_trace("OUT/EP0 rx buffer ready, start DMA");

      nrf5x_usbd_rx_dma(pv, pv->tr[0][0]);
    } else if((pv->tx_running & bit(0)) && pv->tr[0][1]) {
      struct dev_usbdev_rq_s *tr = pv->tr[0][1];
#ifdef ERRARA_199
      *(volatile uint32_t *)0x40027C1C = 0x00000000;
#endif
      
      pv->tx_running &= ~bit(0);

      size_t mps = pv->mps[0][1];
      size_t handled = __MIN(tr->size, mps);
      
      tr->size -= handled;
      tr->data += handled;

      logk_trace("IN/EP0 tx done, %d/%d %d left", handled, mps, tr->size);

      if (tr->type == DEV_USBDEV_PARTIAL_DATA_IN
          || handled < mps
          || tr->size == 0) {
        pv->tr[0][1] = NULL;
        usbdev_stack_request_done(&pv->usbdev_ctx, tr);
      } else {
        nrf5x_usbd_tx(pv, tr);
      }
    }
  }

  if (pending & USBD_INTEN_EPDATA) {
    pending &= ~USBD_INTEN_EPDATA;
    uint32_t epdatastatus = cpu_mem_read_32(pv->addr + USBD_EPDATASTATUS_ADDR);
    uint32_t epstatus = cpu_mem_read_32(pv->addr + USBD_EPSTATUS_ADDR);

    logk_trace("epdatastatus: %08x, epstatus: %08x, rx_pending: %02x, tx_buf wait: %02x, dma_rx wait: %02x",
               epdatastatus, epstatus, pv->data_rx_pending, pv->tx_running, pv->dma_rx_running);

    for (uint_fast8_t i = 1; i < 8; ++i) {
      if ((pv->data_rx_pending & bit(i))
          && pv->tr[i][0]
          && (epdatastatus & bit(i + 16))
          ) {
        pv->data_rx_pending &= ~bit(i);

        size_t epsize = cpu_mem_read_32(pv->addr + USBD_SIZE_EPOUT_ADDR(i));
        
        logk_trace("OUT/EP%d rx buffer ready, start DMA", i);

        logk_trace(" -> EP size: %d, mps: %d, amount: %d",
                   epsize,
                   pv->mps[i][0],
                   cpu_mem_read_32(pv->addr + USBD_EPOUT_AMOUNT_ADDR(i)));

        nrf5x_usbd_rx_dma(pv, pv->tr[i][0]);
      }

      if((pv->tx_running & bit(i))
         && pv->tr[i][1]
         && (epdatastatus & bit(i))) {
        struct dev_usbdev_rq_s *tr = pv->tr[i][1];

#ifdef ERRARA_199
      *(volatile uint32_t *)0x40027C1C = 0x00000000;
#endif

        pv->tx_running &= ~bit(i);

        size_t mps = pv->mps[i][1];
        size_t handled = __MIN(tr->size, mps);
        
        tr->size -= handled;
        tr->data += handled;

        logk_trace("IN/EP%d tx done, %d/%d %d left",
                   i, handled, mps, tr->size);
      
        if (tr->type == DEV_USBDEV_PARTIAL_DATA_IN
            || handled < mps
            || tr->size == 0) {
          pv->tr[i][1] = NULL;
          usbdev_stack_request_done(&pv->usbdev_ctx, tr);
        } else {
          nrf5x_usbd_tx(pv, tr);
        }
      }
    }
  }

  for (uint_fast8_t i = 0; i < 8; ++i) {
    if ((pv->dma_rx_running & bit(i))
        && (pending & USBD_INTEN_ENDEPOUT(i))
        && (pv->tr[i][0])
        ) {
      struct dev_usbdev_rq_s *tr = pv->tr[i][0];

#warning Ensure device is still enabled
      nrf_it_disable_mask(pv->addr, USBD_INTEN_ENDEPOUT(i));
      pending &= ~USBD_INTEN_ENDEPOUT(i);

      size_t mps = pv->mps[i][0];
      size_t handled = cpu_mem_read_32(pv->addr + USBD_EPOUT_AMOUNT_ADDR(i));
      size_t epsize = cpu_mem_read_32(pv->addr + USBD_SIZE_EPOUT_ADDR(i));

      logk_trace("OUT/EP%d rx DMA done, %d/%d/%d: %P", i, handled, mps, tr->size,
                 tr->data, handled);
      logk_trace("OUT/EP%d epsize after %d", i, epsize);

      tr->size -= handled;
      tr->data += handled;

      if (tr->type == DEV_USBDEV_PARTIAL_DATA_OUT
          || handled < mps
          || tr->size == 0) {
        pv->tr[i][0] = NULL;
        usbdev_stack_request_done(&pv->usbdev_ctx, tr);
      } else {
        nrf5x_usbd_rx_prepare(pv, tr);
      }
    }
  }

  pv->irq_pending |= pending;

  if (pv->event)
    nrf5x_usbd_stack_event(dev->drv_pv);

  if (pv->tx_running & 1)
    kroutine_exec(kr);
}

static const struct dev_usbdev_driver_ops_s nrf5x_usbd_ops =
{
  .f_transfer = nrf5x_usb_transfer,
  .f_config = nrf5x_usb_config,
  .f_alloc = nrf5x_usb_alloc,
  .f_free = nrf5x_usb_free,
  .f_endpoint = nrf5x_usb_endpoint
};

static DEV_INIT(nrf5x_usb_init);
static DEV_CLEANUP(nrf5x_usb_cleanup);
static DEV_USE(nrf5x_usb_use);

DRIVER_DECLARE(nrf5x_usbd_drv, 0, "nRF5x USB controller", nrf5x_usb,
               DRIVER_USBDEV_METHODS(nrf5x_usb));

DRIVER_REGISTER(nrf5x_usbd_drv);

static DEV_INIT(nrf5x_usb_init)
{
  struct nrf5x_usb_private_s *pv;
  error_t err;

  pv = mem_alloc(sizeof (*pv), (mem_scope_sys));

  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));

  dev->drv_pv = pv;

  err = device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL);
  if (err)
    goto free_pv;

  device_irq_source_init(dev, &pv->irq_ep, 1, &nrf5x_usbd_irq);
  err = device_irq_source_link(dev, &pv->irq_ep, 1, -1);
  if (err)
    goto free_pv;

  kroutine_init_deferred(&pv->update, nrf5x_usbd_update);
  kroutine_init_deferred(&pv->irq_handler, nrf5x_usbd_irq_handler);

  struct dev_freq_s freq;
  err = dev_drv_clock_init(dev, &pv->clock_sink[NRF5X_USBD_CLK_HFCLK], NRF5X_USBD_CLK_HFCLK,
                           DEV_CLOCK_EP_FREQ_NOTIFY | DEV_CLOCK_EP_VARFREQ, &freq);
  if (err)
    goto free_irq;

  pv->hfclk_ok = freq.acc_e < 16;

  err = dev_drv_clock_init(dev, &pv->clock_sink[NRF5X_USBD_CLK_USB_VBUS], NRF5X_USBD_CLK_USB_VBUS,
                           DEV_CLOCK_EP_FREQ_NOTIFY, NULL);
  if (err)
    goto free_hfclk;

  pv->vbus_present = pv->clock_sink[NRF5X_USBD_CLK_USB_VBUS].src->flags & DEV_CLOCK_EP_POWER;

  err = dev_drv_clock_init(dev, &pv->clock_sink[NRF5X_USBD_CLK_USB_REG], NRF5X_USBD_CLK_USB_REG,
                           DEV_CLOCK_EP_FREQ_NOTIFY, NULL);
  if (err)
    goto free_vbus;

  pv->reg_ready = pv->clock_sink[NRF5X_USBD_CLK_USB_REG].src->flags & DEV_CLOCK_EP_POWER;

  nrf_it_enable_mask(pv->addr, 0
                     | USBD_INTEN_USBRESET
                     | USBD_INTEN_USBEVENT
                     );

  kroutine_exec(&pv->update);

  /* USB stack context */
  if (usbdev_stack_init(dev, &pv->usbdev_ctx, 0xff, 0xff, &nrf5x_usbd_ops))
    goto free_vreg;

  return 0;

 free_vreg:
  dev_drv_clock_cleanup(dev, &pv->clock_sink[NRF5X_USBD_CLK_USB_REG]);
 free_vbus:
  dev_drv_clock_cleanup(dev, &pv->clock_sink[NRF5X_USBD_CLK_USB_VBUS]);
 free_hfclk:
  dev_drv_clock_cleanup(dev, &pv->clock_sink[NRF5X_USBD_CLK_HFCLK]);
 free_irq:
  device_irq_source_unlink(dev, &pv->irq_ep, 1);
 free_pv:
  mem_free(pv);
  return err;
}

static DEV_CLEANUP(nrf5x_usb_cleanup)
{
  struct nrf5x_usb_private_s *pv = dev->drv_pv;

  if (usbdev_stack_cleanup(&pv->usbdev_ctx))
    return -EBUSY;

  device_irq_source_unlink(dev, &pv->irq_ep, 1);

  mem_free(pv);

  return 0;
}
