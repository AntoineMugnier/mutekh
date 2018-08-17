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
#include <device/class/cmu.h>

#include <arch/nrf5x/rtc.h>

struct nrf5x_rtc_context_s
{
  uintptr_t addr;

  struct dev_irq_src_s irq_ep[1];
  dev_request_pqueue_root_t queue;
  dev_timer_value_t base;
  struct dev_freq_s freq;

#if defined(CONFIG_DEVICE_CLOCK)
  struct dev_clock_sink_ep_s clock_sink;
#endif
};

DRIVER_PV(struct nrf5x_rtc_context_s);

static void nrf5x_rtc_start(struct device_s *dev)
{
  struct nrf5x_rtc_context_s *pv = dev->drv_pv;

  if (dev->start_count)
    return;

  logk_trace("%s\n", __FUNCTION__);

#if defined(CONFIG_DEVICE_CLOCK)
  dev_clock_sink_gate(&pv->clock_sink, DEV_CLOCK_EP_CLOCK);
#endif
  nrf_task_trigger(pv->addr, NRF_RTC_START);
  nrf_it_enable(pv->addr, NRF_RTC_OVERFLW);
}

static void nrf5x_rtc_stop(struct device_s *dev)
{
  struct nrf5x_rtc_context_s *pv = dev->drv_pv;

  if (dev->start_count)
    return;

  logk_trace("%s\n", __FUNCTION__);

  assert(dev_rq_pqueue_isempty(&pv->queue));

  nrf_task_trigger(pv->addr, NRF_RTC_STOP);
  nrf_it_disable_mask(pv->addr, -1);
#if defined(CONFIG_DEVICE_CLOCK)
  dev_clock_sink_gate(&pv->clock_sink, DEV_CLOCK_EP_NONE);
#endif
}

static dev_timer_value_t nrf5x_rtc_value_get(
  struct nrf5x_rtc_context_s *pv)
{
  uint32_t counter;

  CPU_INTERRUPT_SAVESTATE_DISABLE;
  counter = nrf_reg_get(pv->addr, NRF_RTC_COUNTER);
  if (!(counter & bit(23)) && nrf_event_check(pv->addr, NRF_RTC_OVERFLW))
    counter += bit(24);
  CPU_INTERRUPT_RESTORESTATE;

  return pv->base + counter;
}

static void nrf5x_rtc_deadline_set(struct nrf5x_rtc_context_s *pv,
                               dev_timer_value_t next_deadline)
{
  nrf_reg_set(pv->addr, NRF_RTC_CC(0), next_deadline);
  nrf_it_enable(pv->addr, NRF_RTC_COMPARE(0));

  if (next_deadline < nrf5x_rtc_value_get(pv) + 2)
    nrf_it_enable(pv->addr, NRF_RTC_TICK);

  logk_trace("%s %lld\n", __FUNCTION__, next_deadline);
}

static void nrf5x_rtc_deadline_disable(struct nrf5x_rtc_context_s *pv)
{
  nrf_it_disable(pv->addr, NRF_RTC_COMPARE(0));
  nrf_it_disable(pv->addr, NRF_RTC_TICK);
}

static DEV_TIMER_REQUEST(nrf5x_rtc_request)
{
  struct device_s *dev = accessor->dev;
  struct nrf5x_rtc_context_s *pv = dev->drv_pv;
  dev_timer_value_t value;
  error_t err = 0;

  logk_debug("%s %p\n", __FUNCTION__, rq);

  //  assert(rq->base.drvdata != pv);

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  if (dev_rq_pqueue_isempty(&pv->queue)) {
    nrf5x_rtc_start(dev);

    dev->start_count |= 1;
  }

  value = nrf5x_rtc_value_get(pv);

  if (rq->delay)
    rq->deadline = value + rq->delay;

  if (rq->deadline <= value) {
    err = -ETIMEDOUT;

    if (dev_rq_pqueue_isempty(&pv->queue)) {
      device_sleep_schedule(dev);
    }
  } else {
    dev_timer_rq_insert(&pv->queue, rq);
    rq->base.drvdata = pv;

    if (dev_timer_rq_head(&pv->queue) == rq)
      nrf5x_rtc_deadline_set(pv, rq->deadline);
  }

  return err;
}

static DEV_TIMER_CANCEL(nrf5x_rtc_cancel)
{
  struct device_s *dev = accessor->dev;
  struct nrf5x_rtc_context_s *pv = dev->drv_pv;
  struct dev_timer_rq_s *head;

  if (rq->base.drvdata != pv)
    return -ENOENT;

  logk_debug("%s %p\n", __FUNCTION__, rq);

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  head = dev_timer_rq_head(&pv->queue);

  dev_timer_rq_remove(&pv->queue, rq);
  rq->base.drvdata = NULL;

  if (rq == head) {
    head = dev_timer_rq_head(&pv->queue);

    if (head) {
      nrf5x_rtc_deadline_set(pv, head->deadline);
    } else {
      nrf5x_rtc_deadline_disable(pv);
      device_sleep_schedule(dev);
    }
  }

  return 0;
}

static DEV_USE(nrf5x_rtc_use)
{
  switch (op) {
  case DEV_USE_SLEEP: {
    struct device_s *dev = param;
    struct nrf5x_rtc_context_s *pv = dev->drv_pv;

    if (dev_rq_pqueue_isempty(&pv->queue)) {
      dev->start_count &= ~1;
      nrf5x_rtc_stop(dev);
    }

    return 0;
  }

  case DEV_USE_START: {
    struct device_accessor_s *acc = param;
    struct device_s *dev = acc->dev;

    nrf5x_rtc_start(dev);

    return 0;
  }

  case DEV_USE_STOP: {
    struct device_accessor_s *acc = param;
    struct device_s *dev = acc->dev;
    struct nrf5x_rtc_context_s *pv = dev->drv_pv;

    if (dev_rq_pqueue_isempty(&pv->queue))
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
  error_t err = 0;

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  if (value)
    *value = nrf5x_rtc_value_get(pv);

  return err;
}

static DEV_IRQ_SRC_PROCESS(nrf5x_rtc_irq)
{
  struct device_s *dev = ep->base.dev;
  struct nrf5x_rtc_context_s *pv = dev->drv_pv;
  struct dev_timer_rq_s *rq;

  logk_trace("%s\n", __FUNCTION__);

  LOCK_SPIN_SCOPED(&dev->lock);

  if (nrf_event_check(pv->addr, NRF_RTC_OVERFLW)) {
    nrf_event_clear(pv->addr, NRF_RTC_OVERFLW);
    pv->base += 1ULL << 24;
  }

  if (nrf_event_check(pv->addr, NRF_RTC_TICK)) {
    nrf_event_clear(pv->addr, NRF_RTC_TICK);
    nrf_it_disable(pv->addr, NRF_RTC_TICK);
  }

  if (nrf_event_check(pv->addr, NRF_RTC_COMPARE(0))) {
    nrf_event_clear(pv->addr, NRF_RTC_COMPARE(0));
  }

  while ((rq = dev_timer_rq_head(&pv->queue))) {
    if (nrf5x_rtc_value_get(pv) < rq->deadline)
      break;

    dev_timer_rq_remove(&pv->queue, rq);
    rq->base.drvdata = NULL;

    lock_release(&dev->lock);
    logk_trace("%s exec %p\n", __FUNCTION__, rq);
    dev_timer_rq_done(rq);
    lock_spin(&dev->lock);
  }

  if (rq) {
    nrf5x_rtc_deadline_set(pv, rq->deadline);
  } else {
    nrf5x_rtc_deadline_disable(pv);
    device_sleep_schedule(dev);
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
      | DEV_TIMER_CAP_KEEPVALUE
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
  if (dev_drv_clock_init(dev, &pv->clock_sink, 0, 0, &pv->freq))
    goto unlink_irq;
#endif

  dev_rq_pqueue_init(&pv->queue);
  pv->base = 0;

  nrf_it_disable_mask(pv->addr, -1);
  nrf_evt_disable_mask(pv->addr, -1);

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

