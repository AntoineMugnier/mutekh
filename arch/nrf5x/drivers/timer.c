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

    Copyright (c) Nicolas Pouillon <nipo@ssji.net>, 2014
*/

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/iospace.h>
#include <hexo/interrupt.h>
#include <hexo/bit.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/irq.h>
#include <device/class/timer.h>
#include <device/clock.h>

#include <arch/nrf5x/timer.h>
#include <arch/nrf5x/peripheral.h>
#include <arch/nrf5x/ids.h>

DRIVER_PV(struct nrf5x_timer_context_s
{
  uintptr_t addr;

  struct dev_irq_src_s irq_ep[1];
  dev_request_pqueue_root_t queue;
  dev_timer_value_t base;
  uint8_t width;
  uint8_t div;
  struct dev_freq_s freq;

#if defined(CONFIG_DEVICE_CLOCK)
  struct dev_clock_sink_ep_s clock_sink;
#endif
});

#define OVERFLOW 0
#define DEADLINE 1
#define VALUE 2

/** Offset a request deadline by this amount if it is close to the
    current timer value. */
#define NRF5X_TIMER_SKEW 0

static void nrf5x_timer_start(struct nrf5x_timer_context_s *pv)
{
#if defined(CONFIG_DEVICE_CLOCK)
  ensure(!dev_clock_sink_gate(&pv->clock_sink, DEV_CLOCK_EP_CLOCK));
#endif
  nrf_task_trigger(pv->addr, NRF_TIMER_START);
  nrf_it_enable(pv->addr, NRF_TIMER_COMPARE(OVERFLOW));
}

static void nrf5x_timer_stop(struct nrf5x_timer_context_s *pv)
{
  nrf_task_trigger(pv->addr, NRF_TIMER_STOP);
#if 52000 <= CONFIG_NRF5X_MODEL && CONFIG_NRF5X_MODEL <= 52999
  // PAN 78
  nrf_task_trigger(pv->addr, NRF_TIMER_SHUTDOWN);
#endif
  nrf_it_disable_mask(pv->addr, -1);
#if defined(CONFIG_DEVICE_CLOCK)
  dev_clock_sink_gate(&pv->clock_sink, DEV_CLOCK_EP_NONE);
#endif
}

static dev_timer_value_t nrf5x_timer_value_get(
    struct nrf5x_timer_context_s *pv)
{
  uint32_t counter;

  nrf_task_trigger(pv->addr, NRF_TIMER_CAPTURE(VALUE));
  counter = nrf_reg_get(pv->addr, NRF_TIMER_CC(VALUE));
  if (!(counter >> (pv->width - 1))
      && nrf_event_check(pv->addr, NRF_TIMER_COMPARE(OVERFLOW)))
    counter += bit(pv->width);

  return pv->base + counter;
}

static void
nrf5x_timer_flush(struct device_s *dev,
                  struct dev_timer_rq_s *head)
{
  struct nrf5x_timer_context_s *pv = dev->drv_pv;

  dev_timer_value_t v = nrf5x_timer_value_get(pv);

  while (head) {

    dev_timer_value_t d = head->deadline;

    if (v < d) {

      if (NRF5X_TIMER_SKEW != 0 &&
          v + NRF5X_TIMER_SKEW > d)
        d = v + NRF5X_TIMER_SKEW;

      nrf_reg_set(pv->addr, NRF_TIMER_CC(DEADLINE), d);
      nrf_it_enable(pv->addr, NRF_TIMER_COMPARE(DEADLINE));

      v = nrf5x_timer_value_get(pv);

      if (v < d)
        return;
    }

    dev_timer_rq_remove(&pv->queue, head);
    head->base.drvdata = NULL;

    dev_timer_rq_done(head);

    head = dev_timer_rq_head(&pv->queue);
  }

  if (dev->start_count & 1)
    {
      nrf_it_disable(pv->addr, NRF_TIMER_COMPARE(DEADLINE));

      if (--dev->start_count == 0)
        nrf5x_timer_stop(pv);
    }
}

static DEV_TIMER_REQUEST(nrf5x_timer_request)
{
  struct device_s *dev = accessor->dev;
  struct nrf5x_timer_context_s *pv = dev->drv_pv;
  dev_timer_value_t value;

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  bool_t empty = dev_rq_pqueue_isempty(&pv->queue);

  if (!dev->start_count)
    nrf5x_timer_start(pv);
  dev->start_count |= 1;

  value = nrf5x_timer_value_get(pv);

  if (rq->delay)
    rq->deadline = value + rq->delay;

  if (rq->deadline <= value) {

    if (empty && --dev->start_count == 0 /* clear lsb */)
      nrf5x_timer_stop(pv);

    return -ETIMEDOUT;
  }

  dev_timer_rq_insert(&pv->queue, rq);
  rq->base.drvdata = pv;

  if (dev_timer_rq_head(&pv->queue) == rq)
    nrf5x_timer_flush(dev, rq);

  return 0;
}

static DEV_TIMER_CANCEL(nrf5x_timer_cancel)
{
  struct device_s *dev = accessor->dev;
  struct nrf5x_timer_context_s *pv = dev->drv_pv;
  struct dev_timer_rq_s *head;

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  if (rq->base.drvdata != pv)
    return -ETIMEDOUT;

  head = dev_timer_rq_head(&pv->queue);

  dev_timer_rq_remove(&pv->queue, rq);
  rq->base.drvdata = NULL;

  if (rq == head) {
    head = dev_timer_rq_head(&pv->queue);
    nrf5x_timer_flush(dev, head);
  }

  return 0;
}

static DEV_USE(nrf5x_timer_use)
{
  switch (op) {
  default:
    return dev_use_generic(param, op);

  case DEV_USE_START: {
    struct device_accessor_s *acc = param;
    struct device_s *dev = acc->dev;
    struct nrf5x_timer_context_s *pv = dev->drv_pv;

    if (!dev->start_count)
      nrf5x_timer_start(pv);

    return 0;
  }

  case DEV_USE_STOP: {
    struct device_accessor_s *acc = param;
    struct device_s *dev = acc->dev;
    struct nrf5x_timer_context_s *pv = dev->drv_pv;

    if (!dev->start_count)
      nrf5x_timer_stop(pv);

    return 0;
  }
  }

  return 0;
}

static DEV_TIMER_GET_VALUE(nrf5x_timer_get_value)
{
  struct device_s *dev = accessor->dev;
  struct nrf5x_timer_context_s *pv = dev->drv_pv;

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  if (dev->start_count == 0)
    return -EBUSY;

  if (value)
    *value = nrf5x_timer_value_get(pv);

  return 0;
}

static DEV_IRQ_SRC_PROCESS(nrf5x_timer_irq)
{
  struct device_s *dev = ep->base.dev;
  struct nrf5x_timer_context_s *pv = dev->drv_pv;

  LOCK_SPIN_SCOPED(&dev->lock);

  if (nrf_event_check(pv->addr, NRF_TIMER_COMPARE(OVERFLOW))) {
    nrf_event_clear(pv->addr, NRF_TIMER_COMPARE(OVERFLOW));
    pv->base += 1ULL << pv->width;
  }

  if (nrf_event_check(pv->addr, NRF_TIMER_COMPARE(DEADLINE))) {
    nrf_event_clear(pv->addr, NRF_TIMER_COMPARE(DEADLINE));
    nrf5x_timer_flush(dev, dev_timer_rq_head(&pv->queue));
  }
}

static DEV_TIMER_CONFIG(nrf5x_timer_config)
{
  struct device_s *dev = accessor->dev;
  struct nrf5x_timer_context_s *pv = dev->drv_pv;

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  if (cfg) {
    cfg->freq = pv->freq;
    cfg->freq.num >>= 4;
    cfg->rev = 0;
    cfg->res = 1;
    cfg->cap = DEV_TIMER_CAP_STOPPABLE
      | DEV_TIMER_CAP_HIGHRES
      | DEV_TIMER_CAP_TICKLESS
      | DEV_TIMER_CAP_REQUEST;
    cfg->max = -1;
  }

  if (res > 1)
    return -ERANGE;

  return 0;
}


static DEV_INIT(nrf5x_timer_init)
{
  struct nrf5x_timer_context_s *pv;

  pv = mem_alloc(sizeof(*pv), mem_scope_sys);
  dev->drv_pv = pv;

  if (!pv)
    return -ENOMEM;

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL))
    goto free_pv;

  // 1% default
  pv->freq.num = 16000000;
  pv->freq.denom = 1;
  pv->freq.acc_m = 7;
  pv->freq.acc_e = 20;

  switch (pv->addr) {
  case NRF_PERIPHERAL_ADDR(NRF5X_TIMER0):
#if 52000 <= CONFIG_NRF5X_MODEL && CONFIG_NRF5X_MODEL <= 52999
  case NRF_PERIPHERAL_ADDR(NRF5X_TIMER3):
  case NRF_PERIPHERAL_ADDR(NRF5X_TIMER4):
#endif
    nrf_reg_set(pv->addr, NRF_TIMER_BITMODE, NRF_TIMER_BITMODE_32);
    pv->width = 32;
    break;

  case NRF_PERIPHERAL_ADDR(NRF5X_TIMER1):
  case NRF_PERIPHERAL_ADDR(NRF5X_TIMER2):
    nrf_reg_set(pv->addr, NRF_TIMER_BITMODE, NRF_TIMER_BITMODE_16);
    pv->width = 16;
    break;

  default:
    assert(!"Unsupported timer address");
    goto free_pv;
  }

  device_irq_source_init(dev, pv->irq_ep, 1, &nrf5x_timer_irq);

  if (device_irq_source_link(dev, pv->irq_ep, 1, -1))
    goto free_pv;

#if defined(CONFIG_DEVICE_CLOCK)
  if (dev_drv_clock_init(dev, &pv->clock_sink, 0,
                         DEV_CLOCK_EP_GATING_SYNC, &pv->freq))
    goto unlink_irq;
#endif

  pv->freq.num = 16000000;
  pv->freq.denom = 1;

  dev_rq_pqueue_init(&pv->queue);
  pv->base = 0;

  nrf_reg_set(pv->addr, NRF_TIMER_CC(OVERFLOW), 0);
  nrf_reg_set(pv->addr, NRF_TIMER_MODE, NRF_TIMER_MODE_TIMER);
  nrf_reg_set(pv->addr, NRF_TIMER_PRESCALER, 4);
  nrf_it_disable_mask(pv->addr, -1);

  return 0;

 unlink_irq:
  device_irq_source_unlink(dev, pv->irq_ep, 1);
 free_pv:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(nrf5x_timer_cleanup)
{
  struct nrf5x_timer_context_s *pv = dev->drv_pv;

  if (!dev_rq_pqueue_isempty(&pv->queue))
    return -EBUSY;

  dev_rq_pqueue_destroy(&pv->queue);
  nrf_it_disable_mask(pv->addr, -1);
#if defined(CONFIG_DEVICE_CLOCK)
  dev_drv_clock_cleanup(dev, &pv->clock_sink);
#endif

  device_irq_source_unlink(dev, pv->irq_ep, 1);

  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(nrf5x_timer_drv, 0, "nRF5x Timer", nrf5x_timer,
               DRIVER_TIMER_METHODS(nrf5x_timer));

DRIVER_REGISTER(nrf5x_timer_drv);

