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

    Copyright (c) 2021, Nicolas Pouillon <nipo@ssji.net>
*/

#define LOGK_MODULE_ID "nqdc"

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

#include <device/class/iomux.h>
#include <device/class/valio.h>
#include <device/valio/position.h>

#include <arch/nrf5x/qdec.h>

#define QDEC_ADDR NRF_PERIPHERAL_ADDR(NRF5X_QDEC)

struct nrf5x_qdec_pv_s
{
  dev_request_queue_root_t queue;
  struct dev_irq_src_s irq_ep;
  int32_t position;
};

DRIVER_PV(struct nrf5x_qdec_pv_s);

static DEV_IRQ_SRC_PROCESS(nrf5x_qdec_irq)
{
  struct device_s *dev = ep->base.dev;
  struct nrf5x_qdec_pv_s *pv = dev->drv_pv;

  logk_debug("irq");
  
  while (nrf_event_check(QDEC_ADDR, NRF_QDEC_REPORTRDY)) {
    nrf_event_clear(QDEC_ADDR, NRF_QDEC_REPORTRDY);

    nrf_task_trigger(QDEC_ADDR, NRF_QDEC_READCLRACC);
    int32_t offset = nrf_reg_get(QDEC_ADDR, NRF_QDEC_ACCREAD);
    pv->position += offset;
    logk_debug("qdec: off %d now at %d", offset, pv->position);

    GCT_FOREACH(dev_request_queue, &pv->queue, item, {
        struct dev_valio_rq_s *rq = dev_valio_rq_s_cast(item);
        int32_t *rq_value = rq->data;

        if (rq->type == DEVICE_VALIO_WAIT_EVENT
            && *rq_value == pv->position) {
          logk_debug("rq %p not changed", rq);
          GCT_FOREACH_CONTINUE;
        }

        *rq_value = pv->position;

        dev_valio_rq_remove(&pv->queue, rq);
        dev_valio_rq_done(rq);
      });
  }
}

static DEV_VALIO_REQUEST(nrf5x_qdec_request)
{
  struct device_s *dev = accessor->dev;
  struct nrf5x_qdec_pv_s *pv = dev->drv_pv;

  logk_debug("%s", __func__);

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  if (rq->attribute != VALIO_POSITION_VALUE)
    goto inval;

  int32_t *rq_ptr = (int32_t*)rq->data;
  int32_t pos = pv->position;

  switch (rq->type) {
  case DEVICE_VALIO_WAIT_EVENT:
    if (*rq_ptr == pos) {
      logk_debug("wait %d = %d queued", *rq_ptr, pos);
      rq->error = 0;
      dev_valio_rq_pushback(&pv->queue, rq);
      return;
    }

    // fallthrough
  case DEVICE_VALIO_READ:
    *rq_ptr = pos;
    goto done;

  case DEVICE_VALIO_WRITE:
    pv->position = *rq_ptr;

  done:
    rq->error = 0;
    dev_valio_rq_done(rq);
    return;

  default:
  inval:
    rq->error = -EINVAL;
    logk_debug("inval");
    dev_valio_rq_done(rq);
    return;
  }
}

static DEV_VALIO_CANCEL(nrf5x_qdec_cancel)
{
  struct device_s *dev = accessor->dev;
  struct nrf5x_qdec_pv_s *pv = dev->drv_pv;

  logk_trace("%s %p", __func__, rq);

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  GCT_FOREACH(dev_request_queue, &pv->queue, item, {
      struct dev_valio_rq_s *rq_item = dev_valio_rq_s_cast(item);
      if (rq == rq_item) {
        dev_valio_rq_remove(&pv->queue, rq);
        return 0;
      }
    });

  return -ENOENT;
}

#define nrf5x_qdec_use dev_use_generic

static DEV_INIT(nrf5x_qdec_init)
{
  struct nrf5x_qdec_pv_s *pv;
  iomux_io_id_t id[3];
  uintptr_t threshold, led_pre, led_active, period;
  error_t err;

  pv = mem_alloc(sizeof(*pv), mem_scope_sys);
  if (!pv)
    return -ENOMEM;

  dev->drv_pv = pv;
  memset(pv, 0, sizeof(*pv));

  err = device_get_param_uint(dev, "led_active", &led_active);
  if (err)
    led_active = 1;

  err = device_get_param_uint(dev, "threshold", &threshold);
  if (err)
    threshold = 0;
  else if (threshold == 1)
    threshold = 1;
  else
    threshold = threshold / 40;

  err = device_get_param_uint(dev, "led_pre_us", &led_pre);
  if (err)
    led_pre = 0;

  err = device_get_param_uint(dev, "period_us", &period);
  if (err)
    period = 128;
  period = (period / 128) - 1;

  if (device_iomux_setup(dev, "<a <b >led?", NULL, id, NULL))
    goto free_pv;

  nrf_reg_set(QDEC_ADDR, NRF_QDEC_ENABLE, NRF_QDEC_ENABLE_ENABLED);
  nrf_reg_set(QDEC_ADDR, NRF_QDEC_PSEL_A, id[0]);
  nrf_reg_set(QDEC_ADDR, NRF_QDEC_PSEL_B, id[1]);
  nrf_reg_set(QDEC_ADDR, NRF_QDEC_PSEL_LED,
              id[2] != IOMUX_INVALID_ID ? id[2] : (uint32_t)-1);
  
  device_irq_source_init(dev, &pv->irq_ep, 1, &nrf5x_qdec_irq);
  err = device_irq_source_link(dev, &pv->irq_ep, 1, -1);
  if (err) {
    logk_fatal("Cannot link IRQ");
    goto free_pv;
  }

  nrf_reg_set(QDEC_ADDR, NRF_QDEC_LEDPOL, led_active);
  nrf_reg_set(QDEC_ADDR, NRF_QDEC_SAMPLEPER, period);
  nrf_reg_set(QDEC_ADDR, NRF_QDEC_LEDPRE, led_pre);
  nrf_reg_set(QDEC_ADDR, NRF_QDEC_REPORTPER, threshold);
  nrf_task_trigger(QDEC_ADDR, NRF_QDEC_START);
  nrf_task_trigger(QDEC_ADDR, NRF_QDEC_READCLRACC);
  
  dev_rq_queue_init(&pv->queue);
  nrf_it_enable(QDEC_ADDR, NRF_QDEC_REPORTRDY);

  return 0;

 free_pv:
  mem_free(pv);

  return err;
}

static DEV_CLEANUP(nrf5x_qdec_cleanup)
{
  struct nrf5x_qdec_pv_s *pv = dev->drv_pv;

  nrf_task_trigger(QDEC_ADDR, NRF_QDEC_STOP);
  nrf_reg_set(QDEC_ADDR, NRF_QDEC_PSEL_A, -1);
  nrf_reg_set(QDEC_ADDR, NRF_QDEC_PSEL_B, -1);
  nrf_reg_set(QDEC_ADDR, NRF_QDEC_PSEL_LED, -1);
  nrf_reg_set(QDEC_ADDR, NRF_QDEC_ENABLE, NRF_QDEC_ENABLE_DISABLED);

  device_irq_source_unlink(dev, &pv->irq_ep, 1);
  dev_rq_queue_destroy(&pv->queue);
  dev->drv_pv = NULL;
  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(nrf5x_qdec_drv, 0, "nRF5x qdec", nrf5x_qdec,
               DRIVER_VALIO_METHODS(nrf5x_qdec));

DRIVER_REGISTER(nrf5x_qdec_drv);
