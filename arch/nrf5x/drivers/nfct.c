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

    Copyright (c) 2016, Nicolas Pouillon <nipo@ssji.net>
*/

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/iospace.h>
#include <hexo/interrupt.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/irq.h>
#include <device/class/nfc.h>
#include <arch/nrf5x/nfct.h>
#include <arch/nrf5x/uicr.h>
#include <arch/nrf5x/ficr.h>
#include <arch/nrf5x/ids.h>

//#define dprintk printk
#define dprintk(...) do{}while(0)

#define NFCT_ADDR NRF_PERIPHERAL_ADDR(NRF5X_NFCT)

struct nrf52_nfct_priv
{
  dev_request_queue_root_t queue;

  struct dev_irq_src_s irq_ep;

  bool_t selected;
  bool_t active;

  struct dev_nfc_cfg_cache_s config_cache;
  struct dev_nfc_config_s config;
};

DRIVER_PV(struct nrf52_nfct_priv);

static void nrf52_nfct_req_done(struct device_s *dev, error_t err)
{
  struct nrf52_nfct_priv *pv = dev->drv_pv;
  struct dev_nfc_rq_s *rq;

  rq = dev_nfc_rq_s_cast(dev_request_queue_pop(&pv->queue));
  assert(rq);

  rq->error = err;
  kroutine_exec(&rq->base.kr);
}

static void nrf52_nfct_activate(struct device_s *dev)
{
  struct nrf52_nfct_priv *pv = dev->drv_pv;
  struct dev_nfc_rq_s *rq;

  rq = dev_nfc_rq_s_cast(dev_request_queue_head(&pv->queue));

  dprintk("%s\n", __FUNCTION__);

  if (rq) {
    pv->config_cache.id++;
    pv->config_cache.dirty = 0;
    rq->config_cache.id = pv->config_cache.id;
    rq->config_cache.dirty = 0;
    pv->config = *rq->config;
    pv->config.local.sens_res &= ~0xc0;
    switch (pv->config.local.uid_size) {
    case 10: pv->config.local.sens_res |= 0x80; break;
    case 7: pv->config.local.sens_res |= 0x40; break;
    }
  } else {
    memcpy(pv->config.local.uid, (const uint8_t []){0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa}, 10);
    pv->config.local.uid_size = 10;
    pv->config.local.sens_res = 0x44;
    pv->config.local.sel_res = 0x04;
  }

  printk("Reconfiguring, id=%d, UID=%P\n",
         pv->config_cache.id,
         pv->config.local.uid,
         pv->config.local.uid_size);

  nrf_reg_set(NFCT_ADDR, NRF_NFCT_SENSRES, pv->config.local.sens_res);
  nrf_reg_set(NFCT_ADDR, NRF_NFCT_SELRES, pv->config.local.sel_res);
  nrf_reg_set(NFCT_ADDR, NRF_NFCT_FRAMEDELAYMAX, 0xffff);
  nrf_reg_set(NFCT_ADDR, NRF_NFCT_FRAMEDELAYMODE, NRF_NFCT_FRAMEDELAYMODE_WINDOWGRID);

  switch (pv->config.local.uid_size) {
  case 10:
    nrf_reg_set(NFCT_ADDR, NRF_NFCT_NFCID1_LAST, endian_be32_na_load(pv->config.local.uid + 6));
    nrf_reg_set(NFCT_ADDR, NRF_NFCT_NFCID1_2ND_LAST, endian_be32_na_load(pv->config.local.uid + 3) >> 8);
    nrf_reg_set(NFCT_ADDR, NRF_NFCT_NFCID1_3RD_LAST, endian_be32_na_load(pv->config.local.uid) >> 8);
    break;
  case 7:
    nrf_reg_set(NFCT_ADDR, NRF_NFCT_NFCID1_LAST, endian_be32_na_load(pv->config.local.uid + 3));
    nrf_reg_set(NFCT_ADDR, NRF_NFCT_NFCID1_2ND_LAST, endian_be32_na_load(pv->config.local.uid) >> 8);
    break;
  case 4:
    nrf_reg_set(NFCT_ADDR, NRF_NFCT_NFCID1_LAST, endian_be32_na_load(pv->config.local.uid));
    break;
  }

  nrf_event_clear(NFCT_ADDR, NRF_NFCT_FIELDDETECT);
  nrf_event_clear(NFCT_ADDR, NRF_NFCT_FIELDLOST);
  nrf_event_clear(NFCT_ADDR, NRF_NFCT_SELECTED);
  nrf_event_clear(NFCT_ADDR, NRF_NFCT_READY);
  nrf_event_clear(NFCT_ADDR, NRF_NFCT_ERROR);
  nrf_event_clear(NFCT_ADDR, NRF_NFCT_TXFRAMEEND);
  nrf_event_clear(NFCT_ADDR, NRF_NFCT_RXFRAMEEND);
  nrf_event_clear(NFCT_ADDR, NRF_NFCT_AUTOCOLRESST);

  nrf_short_set(NFCT_ADDR, 0
                | bit(NRF_NFCT_FIELDDETECT_ACTIVATE)
                | bit(NRF_NFCT_FIELDLOST_SENSE));

  nrf_task_trigger(NFCT_ADDR, NRF_NFCT_SENSE);

  nrf_it_enable_mask(NFCT_ADDR, 0
                     | bit(NRF_NFCT_FIELDDETECT)
                     | bit(NRF_NFCT_FIELDLOST)
                     | bit(NRF_NFCT_SELECTED)
                     | bit(NRF_NFCT_READY)
                     | bit(NRF_NFCT_ERROR)
                     | bit(NRF_NFCT_AUTOCOLRESST)
                     );
}

static void nrf52_nfct_deactivate(struct device_s *dev)
{
  struct nrf52_nfct_priv *pv = dev->drv_pv;

  dprintk("%s\n", __FUNCTION__);

  nrf_short_set(NFCT_ADDR, 0);
  nrf_task_trigger(NFCT_ADDR, NRF_NFCT_DISABLE);
  nrf_it_disable_mask(NFCT_ADDR, -1);

  pv->config_cache.dirty = 1;
}

static void nrf52_nfct_start_next(struct device_s *dev)
{
  struct nrf52_nfct_priv *pv = dev->drv_pv;
  struct dev_nfc_rq_s *rq;

 next:
  rq = dev_nfc_rq_s_cast(dev_request_queue_head(&pv->queue));

  if (!rq) {
    device_sleep_schedule(dev);
    return;
  }

  if (rq->config_cache.id != pv->config_cache.id
      || rq->config_cache.dirty
      || pv->config_cache.dirty)
    nrf52_nfct_activate(dev);

  dprintk("%s %p %d %d\n", __FUNCTION__, rq, pv->active, pv->selected);

  switch (rq->type) {
  case DEV_NFC_POLL:
    dprintk("POLL\n");
    if (pv->active && pv->selected) {
      nrf52_nfct_req_done(dev, 0);
      goto next;
    }
    break;

  case DEV_NFC_WRITE:
    dprintk("Write\n");
    if (!pv->active || !pv->selected) {
      nrf52_nfct_req_done(dev, -EIO);
      goto next;
    }

    nrf_reg_set(NFCT_ADDR, NRF_NFCT_MAXLEN, rq->data.size);
    nrf_reg_set(NFCT_ADDR, NRF_NFCT_PACKETPTR, (uintptr_t)rq->data.data);
    if (rq->data.last_byte_bits) {
      nrf_reg_set(NFCT_ADDR, NRF_NFCT_TXD_AMOUNT,
                  rq->data.last_byte_bits | ((rq->data.size - 1) << 3));
      nrf_reg_set(NFCT_ADDR, NRF_NFCT_TXD_FRAMECONFIG, 0
                  | NRF_NFCT_TXD_FRAMECONFIG_SOF
                  | NRF_NFCT_TXD_FRAMECONFIG_DISCARD_END
                  );
    } else {
      nrf_reg_set(NFCT_ADDR, NRF_NFCT_TXD_AMOUNT, rq->data.size << 3);
      nrf_reg_set(NFCT_ADDR, NRF_NFCT_TXD_FRAMECONFIG, 0
                  | NRF_NFCT_TXD_FRAMECONFIG_SOF
                  | NRF_NFCT_TXD_FRAMECONFIG_PARITY
                  | NRF_NFCT_TXD_FRAMECONFIG_DISCARD_END
                  | NRF_NFCT_TXD_FRAMECONFIG_CRC16TX
                  );
    }

    nrf_event_clear(NFCT_ADDR, NRF_NFCT_ERROR);
    nrf_event_clear(NFCT_ADDR, NRF_NFCT_TXFRAMEEND);

    nrf_it_enable(NFCT_ADDR, NRF_NFCT_TXFRAMEEND);
    nrf_task_trigger(NFCT_ADDR, NRF_NFCT_STARTTX);
    break;

  case DEV_NFC_READ:
    dprintk("Read\n");
    if (!pv->active || !pv->selected) {
      nrf52_nfct_req_done(dev, -EIO);
      goto next;
    }

    nrf_reg_set(NFCT_ADDR, NRF_NFCT_MAXLEN, rq->data.size);
    nrf_reg_set(NFCT_ADDR, NRF_NFCT_PACKETPTR, (uintptr_t)rq->data.data);
    nrf_reg_set(NFCT_ADDR, NRF_NFCT_RXD_FRAMECONFIG, 0
                | NRF_NFCT_RXD_FRAMECONFIG_PARITY
                | NRF_NFCT_RXD_FRAMECONFIG_SOF
                | NRF_NFCT_RXD_FRAMECONFIG_CRC16RX
                );

    nrf_event_clear(NFCT_ADDR, NRF_NFCT_ERROR);
    nrf_event_clear(NFCT_ADDR, NRF_NFCT_RXFRAMEEND);

    nrf_it_enable(NFCT_ADDR, NRF_NFCT_RXFRAMEEND);
    nrf_task_trigger(NFCT_ADDR, NRF_NFCT_ENABLERXDATA);
    break;
  }
}

static DEV_IRQ_SRC_PROCESS(nrf52_nfct_irq)
{
  struct device_s *dev = ep->base.dev;
  struct nrf52_nfct_priv *pv = dev->drv_pv;
  bool_t start_next = 0;
  struct dev_nfc_rq_s *rq;

  lock_spin(&dev->lock);

  rq = dev_nfc_rq_s_cast(dev_request_queue_head(&pv->queue));

  if (nrf_event_check(NFCT_ADDR, NRF_NFCT_READY)) {
    nrf_event_clear(NFCT_ADDR, NRF_NFCT_READY);
    pv->active = 1;
    pv->selected = 0;

    dprintk("NFCT Ready\n");

    start_next = 1;
  }

  if (nrf_event_check(NFCT_ADDR, NRF_NFCT_SELECTED)) {
    nrf_event_clear(NFCT_ADDR, NRF_NFCT_SELECTED);
    pv->selected = 1;

    dprintk("NFCT Selected\n");

    nrf_event_clear(NFCT_ADDR, NRF_NFCT_COLLISION);
    nrf_event_clear(NFCT_ADDR, NRF_NFCT_ERROR);

    nrf52_nfct_req_done(dev, -EIO);

    start_next = 1;
  }

  if (nrf_event_check(NFCT_ADDR, NRF_NFCT_ERROR)) {
    nrf_event_clear(NFCT_ADDR, NRF_NFCT_ERROR);
    dprintk("NFCT Error %x\n", nrf_reg_get(NFCT_ADDR, NRF_NFCT_ERRORSTATUS));

    nrf_it_disable_mask(NFCT_ADDR, 0
                        | bit(NRF_NFCT_TXFRAMEEND)
                        | bit(NRF_NFCT_RXFRAMEEND)
                        );
  }

  if (nrf_event_check(NFCT_ADDR, NRF_NFCT_TXFRAMEEND)) {
    nrf_event_clear(NFCT_ADDR, NRF_NFCT_TXFRAMEEND);

    if (nrf_it_is_enabled(NFCT_ADDR, NRF_NFCT_TXFRAMEEND)) {
      dprintk("NFCT Txframeend\n");

      assert(rq && rq->type == DEV_NFC_WRITE);

      nrf_it_disable(NFCT_ADDR, NRF_NFCT_TXFRAMEEND);

      rq->data.data += rq->data.size;
      rq->data.size = 0;

      nrf52_nfct_req_done(dev, 0);
      start_next = 1;
    }
  }

  if (nrf_event_check(NFCT_ADDR, NRF_NFCT_AUTOCOLRESST)) {
    nrf_event_clear(NFCT_ADDR, NRF_NFCT_AUTOCOLRESST);
    dprintk("NFCT Autocoll\n");
  }

  if (nrf_event_check(NFCT_ADDR, NRF_NFCT_RXFRAMEEND)) {
    nrf_event_clear(NFCT_ADDR, NRF_NFCT_RXFRAMEEND);

    if (nrf_it_is_enabled(NFCT_ADDR, NRF_NFCT_RXFRAMEEND)) {
      uint32_t size = nrf_reg_get(NFCT_ADDR, NRF_NFCT_RXD_AMOUNT);
      dprintk("NFCT Rxframeend\n");

      assert(rq && rq->type == DEV_NFC_READ);

      nrf_it_disable(NFCT_ADDR, NRF_NFCT_RXFRAMEEND);

      rq->data.size -= size >> 3;
      rq->data.data += size >> 3;
      rq->data.last_byte_bits = size & 7;

      nrf52_nfct_req_done(dev, 0);
      start_next = 1;
    }
  }

  if (nrf_event_check(NFCT_ADDR, NRF_NFCT_FIELDDETECT)) {
    nrf_event_clear(NFCT_ADDR, NRF_NFCT_FIELDDETECT);

    dprintk("NFCT Field detected\n");
  }

  if (nrf_event_check(NFCT_ADDR, NRF_NFCT_FIELDLOST)) {
    nrf_event_clear(NFCT_ADDR, NRF_NFCT_FIELDLOST);

    nrf_it_disable_mask(NFCT_ADDR, 0
                        | bit(NRF_NFCT_TXFRAMEEND)
                        | bit(NRF_NFCT_RXFRAMEEND)
                        );

    dprintk("NFCT Field lost\n");

    pv->active = 0;
    pv->selected = 0;

    start_next = 1;
  }

  if (nrf_event_check(NFCT_ADDR, NRF_NFCT_COLLISION)) {
    nrf_event_clear(NFCT_ADDR, NRF_NFCT_COLLISION);

    dprintk("NFCT Collision\n");

    pv->selected = 0;
  }

  if (start_next)
    nrf52_nfct_start_next(dev);

  if (dev_request_queue_isempty(&pv->queue))
    device_sleep_schedule(dev);

  lock_release(&dev->lock);
}

static DEV_NFC_INFO_GET(nrf52_nfct_info_get)
{
  memcpy(info->uid, (const void *)NRF_FICR_NFC_TAGHEADER0, 10);

  info->uid_size = 10;

  info->sens_res = 0x44;
  info->sel_res = 0x04;
  info->rate = DEV_NFC_106K;
}

static DEV_NFC_REQUEST(nrf52_nfct_request)
{
  struct device_s *dev = accessor->dev;
  struct nrf52_nfct_priv *pv = dev->drv_pv;
  bool_t start = 1;

  dprintk("%s\n", __FUNCTION__);

  assert(rq->config);

  if (rq->config->mode != DEV_NFC_14443A
      || rq->config->side != DEV_NFC_TARGET
      || rq->config->rate != DEV_NFC_106K) {
    rq->error = -ENOTSUP;
    kroutine_exec(&rq->base.kr);
    return;
  }

  LOCK_SPIN_IRQ(&dev->lock);

  rq->error = 0;

  start &= dev_request_queue_isempty(&pv->queue);
  dev_request_queue_pushback(&pv->queue, &rq->base);

  if (start) {
    if (!pv->active && !dev->start_count) {
      dev->start_count |= 1;
      nrf52_nfct_activate(dev);
    } else if (pv->active) {
      nrf52_nfct_start_next(dev);
    }
  }

  LOCK_RELEASE_IRQ(&dev->lock);
}

static DEV_NFC_CANCEL(nrf52_nfct_cancel)
{
  struct device_s *dev = accessor->dev;
  struct nrf52_nfct_priv *pv = dev->drv_pv;
  error_t err;

  LOCK_SPIN_IRQ(&dev->lock);

  if (dev_request_queue_head(&pv->queue) == &rq->base) {
    err = -EBUSY;
  } else {
    dev_request_queue_remove(&pv->queue, &rq->base);
    err = 0;
  }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

static DEV_USE(nrf52_nfct_use)
{
  switch (op)
    {
    case DEV_USE_START: {
      struct device_accessor_s *acc = param;
      struct device_s *dev = acc->dev;

      if (!dev->start_count)
        nrf52_nfct_activate(dev);
      return 0;
    }

    case DEV_USE_STOP: {
      struct device_accessor_s *acc = param;
      struct device_s *dev = acc->dev;

      if (!dev->start_count)
        device_sleep_schedule(dev);
      return 0;
    }

    case DEV_USE_SLEEP: {
      struct device_s *dev = param;
      struct nrf52_nfct_priv *pv = dev->drv_pv;

      if (dev_request_queue_isempty(&pv->queue)) {
        dev->start_count &= ~1;

        if (dev->start_count == 0)
          nrf52_nfct_deactivate(dev);
      }

      return 0;
    }

    default:
      return dev_use_generic(param, op);
    }
}

static DEV_INIT(nrf52_nfct_init)
{
  struct nrf52_nfct_priv *pv;

  if ((cpu_mem_read_32(NRF_UICR_NFCPINS) & NRF_UICR_NFCPINS_PROTECT_MASK) != NRF_UICR_NFCPINS_PROTECT_NFC)
    return -ENOTSUP;

  __unused__ uintptr_t addr = 0;
  assert(device_res_get_uint(dev, DEV_RES_MEM, 0, &addr, NULL) == 0 &&
         NFCT_ADDR == addr);

  pv = mem_alloc(sizeof(*pv), mem_scope_sys);
  if (!pv)
    return -ENOMEM;

  dev->drv_pv = pv;
  memset(pv, 0, sizeof(*pv));

  dev_request_queue_init(&pv->queue);

  nrf52_nfct_deactivate(dev);

  device_irq_source_init(dev, &pv->irq_ep, 1, &nrf52_nfct_irq);

  if (device_irq_source_link(dev, &pv->irq_ep, 1, -1))
    goto free_queue;

  return 0;

 free_queue:
  dev_request_queue_destroy(&pv->queue);

 free_pv:
  mem_free(pv);

  return -1;
}

static DEV_CLEANUP(nrf52_nfct_cleanup)
{
  struct nrf52_nfct_priv *pv = dev->drv_pv;

  if (!dev_request_queue_isempty(&pv->queue))
    return -EBUSY;

  nrf52_nfct_deactivate(dev);
  device_irq_source_unlink(dev, &pv->irq_ep, 1);
  dev_request_queue_destroy(&pv->queue);

  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(nrf52_nfct_drv, 0, "nRF52 NFC Tag", nrf52_nfct,
               DRIVER_NFC_METHODS(nrf52_nfct));

DRIVER_REGISTER(nrf52_nfct_drv);
