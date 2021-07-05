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

#define LOGK_MODULE_ID "nrtc"

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
#include <device/class/cmu.h>

#include <arch/nrf5x/rtc.h>

struct nrf5x_rtc_context_s
{
  uintptr_t addr;

  struct dev_irq_src_s irq_ep[1];
  dev_request_pqueue_root_t queue;
  dev_timer_value_t base;
  struct dev_freq_s freq;

  /* This indicates if the start or stop task has been started on the
     hardware. The dev->start_count value indicates what the user has
     requested which may not be the same due to DEVICE_SLEEP latency. */
  bool_t running;

#if defined(CONFIG_DEVICE_CLOCK)
  struct dev_clock_sink_ep_s clock_sink;
#endif
};

DRIVER_PV(struct nrf5x_rtc_context_s);

/* NRF manual requires setting compare regs to N+2 but we need to
   account for time needed to read the current N and performs a few
   more computations before setting a CC reg. */
#define NRF5X_RTC_SKEW 3

static void nrf5x_rtc_start(struct device_s *dev)
{
  struct nrf5x_rtc_context_s *pv = dev->drv_pv;

  if (pv->running)
    return;

  logk_trace("%s", __FUNCTION__);

#if defined(CONFIG_DEVICE_CLOCK)
  ensure(!dev_clock_sink_gate(&pv->clock_sink, DEV_CLOCK_EP_CLOCK));
#endif
#if CONFIG_NRF5X_MODEL == 52840
  nrf_reg_set(pv->addr, NRF_RTC_PRESCALER, 0);
#endif
  nrf_task_trigger(pv->addr, NRF_RTC_START);
  nrf_it_enable(pv->addr, NRF_RTC_OVERFLW);
  pv->running = 1;
}

static void nrf5x_rtc_stop(struct device_s *dev)
{
  struct nrf5x_rtc_context_s *pv = dev->drv_pv;

  if (!pv->running)
    return;

  logk_trace("%s", __FUNCTION__);

  assert(dev_rq_pqueue_isempty(&pv->queue));

  nrf_task_trigger(pv->addr, NRF_RTC_STOP);
  nrf_it_disable_mask(pv->addr, -1);
#if defined(CONFIG_DEVICE_CLOCK)
  dev_clock_sink_gate(&pv->clock_sink, DEV_CLOCK_EP_NONE);
#endif
  pv->running = 0;
}

static dev_timer_value_t nrf5x_rtc_value_get(struct nrf5x_rtc_context_s *pv)
{
  uint32_t counter = nrf_reg_get(pv->addr, NRF_RTC_COUNTER);

  if (!bit_get(counter, 23) && nrf_event_check(pv->addr, NRF_RTC_OVERFLW))
    counter += bit(24);

  return pv->base + counter;
}

static void
nrf5x_rtc_flush(struct device_s *dev,
                struct dev_timer_rq_s *head)
{
  struct nrf5x_rtc_context_s *pv = dev->drv_pv;

  while (head) {

    dev_timer_value_t v = nrf5x_rtc_value_get(pv);
    dev_timer_value_t d = head->deadline;

    if (v < d) {

      if (v + NRF5X_RTC_SKEW > d)
        d = v + NRF5X_RTC_SKEW;

      nrf_reg_set(pv->addr, NRF_RTC_CC(0), d);
      nrf_it_enable(pv->addr, NRF_RTC_COMPARE(0));

      logk_trace("dl set %llu %llu", v, d);
      return;
    }

    dev_timer_rq_remove(&pv->queue, head);
    head->base.drvdata = NULL;

    logk_trace("rq done %p", head);
    dev_timer_rq_done(head);

    head = dev_timer_rq_head(&pv->queue);
  }

  if (dev->start_count & 1)
    {
      logk_trace("dl none");
      nrf_it_disable(pv->addr, NRF_RTC_COMPARE(0));

      if (--dev->start_count == 0) /* clear lsb */
        device_sleep_schedule(dev);
    }
}

static DEV_TIMER_REQUEST(nrf5x_rtc_request)
{
  struct device_s *dev = accessor->dev;
  struct nrf5x_rtc_context_s *pv = dev->drv_pv;
  dev_timer_value_t value;

  logk_trace("%s %p", __FUNCTION__, rq);

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  bool_t empty = dev_rq_pqueue_isempty(&pv->queue);

  if (!dev->start_count)
    nrf5x_rtc_start(dev);
  dev->start_count |= 1;

  value = nrf5x_rtc_value_get(pv);

  if (rq->delay)
    rq->deadline = value + rq->delay;

  if (rq->deadline <= value) {
    logk_trace("-ETIMEDOUT %llu %llu", rq->deadline, value);

    if (empty && --dev->start_count == 0 /* clear lsb */)
      device_sleep_schedule(dev);

    return -ETIMEDOUT;
  }

  logk_trace("insert");
  dev_timer_rq_insert(&pv->queue, rq);
  rq->base.drvdata = pv;

  if (dev_timer_rq_head(&pv->queue) == rq)
    nrf5x_rtc_flush(dev, rq);

  return 0;
}

static DEV_TIMER_CANCEL(nrf5x_rtc_cancel)
{
  struct device_s *dev = accessor->dev;
  struct nrf5x_rtc_context_s *pv = dev->drv_pv;
  struct dev_timer_rq_s *head;

  logk_trace("%s %p", __FUNCTION__, rq);

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  if (rq->base.drvdata != pv)
    return -ENOENT;

  head = dev_timer_rq_head(&pv->queue);

  dev_timer_rq_remove(&pv->queue, rq);
  rq->base.drvdata = NULL;

  if (rq == head) {
    head = dev_timer_rq_head(&pv->queue);
    nrf5x_rtc_flush(dev, head);
  }

  return 0;
}

static DEV_USE(nrf5x_rtc_use)
{
  switch (op) {
  case DEV_USE_SLEEP: {
    struct device_s *dev = param;

    if (dev->start_count == 0)
      nrf5x_rtc_stop(dev);

    return 0;
  }

  case DEV_USE_START: {
    struct device_accessor_s *acc = param;
    struct device_s *dev = acc->dev;

    if (dev->start_count == 0)
      nrf5x_rtc_start(dev);

    return 0;
  }

  case DEV_USE_STOP: {
    struct device_accessor_s *acc = param;
    struct device_s *dev = acc->dev;

    if (dev->start_count == 0)
      device_sleep_schedule(dev);

    return 0;
  }

  default:
    return dev_use_generic(param, op);
  }
}

static DEV_TIMER_GET_VALUE(nrf5x_rtc_get_value)
{
  struct device_s *dev = accessor->dev;
  struct nrf5x_rtc_context_s *pv = dev->drv_pv;

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  if (dev->start_count == 0)
    return -EBUSY;

  if (value)
    *value = nrf5x_rtc_value_get(pv);

  return 0;
}

static DEV_IRQ_SRC_PROCESS(nrf5x_rtc_irq)
{
  struct device_s *dev = ep->base.dev;
  struct nrf5x_rtc_context_s *pv = dev->drv_pv;

  logk_trace("%s", __FUNCTION__);

  LOCK_SPIN_SCOPED(&dev->lock);

  if (nrf_event_check(pv->addr, NRF_RTC_OVERFLW)) {
    nrf_event_clear(pv->addr, NRF_RTC_OVERFLW);
    pv->base += 1ULL << 24;
  }

  if (nrf_event_check(pv->addr, NRF_RTC_COMPARE(0))) {
    nrf_event_clear(pv->addr, NRF_RTC_COMPARE(0));
    nrf5x_rtc_flush(dev, dev_timer_rq_head(&pv->queue));
  }
}

static DEV_TIMER_CONFIG(nrf5x_rtc_config)
{
  struct device_s *dev = accessor->dev;
  struct nrf5x_rtc_context_s *pv = dev->drv_pv;
  error_t err = 0;

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  if (cfg) {
    cfg->freq.num = 32768;
    cfg->freq.denom = 1;
    cfg->freq = pv->freq;
    cfg->rev = 0;
    cfg->res = 1;
    cfg->cap = DEV_TIMER_CAP_STOPPABLE
      | DEV_TIMER_CAP_HIGHRES
      | DEV_TIMER_CAP_TICKLESS
      | DEV_TIMER_CAP_REQUEST;
    cfg->max = -1;
  }

  if (res > 1)
    err = -ERANGE;

  return err;
}


static DEV_INIT(nrf5x_rtc_init)
{
  struct nrf5x_rtc_context_s *pv;

  pv = mem_alloc(sizeof(*pv), mem_scope_sys);
  dev->drv_pv = pv;

  if (!pv)
    return -ENOMEM;

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL))
    goto free_pv;

  // 2% default
  pv->freq.num = 32768;
  pv->freq.denom = 1;
  pv->freq.acc_m = 0;
  pv->freq.acc_e = 22;

  device_irq_source_init(dev, pv->irq_ep, 1, &nrf5x_rtc_irq);

  if (device_irq_source_link(dev, pv->irq_ep, 1, -1))
    goto free_pv;

#if defined(CONFIG_DEVICE_CLOCK)
  if (dev_drv_clock_init(dev, &pv->clock_sink, 0,
                         DEV_CLOCK_EP_GATING_SYNC, &pv->freq))
    goto unlink_irq;
#endif

  dev_rq_pqueue_init(&pv->queue);
  pv->base = 0;

  nrf_it_disable_mask(pv->addr, -1);
  nrf_evt_disable_mask(pv->addr, -1);
  pv->running = 0;

  return 0;

 unlink_irq:
  device_irq_source_unlink(dev, pv->irq_ep, 1);
 free_pv:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(nrf5x_rtc_cleanup)
{
  struct nrf5x_rtc_context_s *pv = dev->drv_pv;

  if (dev->start_count)
    return -EBUSY;

  dev_rq_pqueue_destroy(&pv->queue);

  nrf_it_disable_mask(pv->addr, -1);
  nrf_evt_disable_mask(pv->addr, -1);

#if defined(CONFIG_DEVICE_CLOCK)
  dev_drv_clock_cleanup(dev, &pv->clock_sink);
#endif
  device_irq_source_unlink(dev, pv->irq_ep, 1);

  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(nrf5x_rtc_drv, 0, "nRF5x RTC", nrf5x_rtc,
               DRIVER_TIMER_METHODS(nrf5x_rtc));

DRIVER_REGISTER(nrf5x_rtc_drv);

